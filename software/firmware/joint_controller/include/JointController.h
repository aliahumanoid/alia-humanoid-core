/**
 * @file JointController.h
 * @brief Multi-DOF joint controller with cascade PID and tendon-driven actuation
 * 
 * This class manages the complete control of a multi-DOF joint in the humanoid robot.
 * It implements a sophisticated cascade control architecture with:
 * 
 * ARCHITECTURE:
 * - Outer loop: Position control per DOF using joint encoders (frequency configurable)
 * - Inner loop: Torque control per motor using motor encoders (500 Hz default)
 * - Agonist-antagonist muscle pairs per DOF (tendon-driven actuation)
 * 
 * KEY FEATURES:
 * - Multi-DOF coordinated movements (up to 3 DOF)
 * - Automatic calibration (zero-finding and motor-joint mapping)
 * - Linear equation-based motor angle computation
 * - Safety limits enforcement (joint and motor angles)
 * - SPI spike detection and filtering for encoder readings
 * - Flash storage for PID parameters and calibration data
 * 
 * CONTROL MODES:
 * 1. Trajectory Following: Smooth path from current to target position
 * 2. Position Holding: Active PID maintains position indefinitely
 * 3. Pretension/Release: Apply/remove tension for calibration
 * 4. Auto-mapping: Automatic calibration of motor-joint relationships
 * 
 * MOVEMENT COMMAND BEHAVIOR:
 * Rolling-impedance control with open-ended PID:
 * - Phase 1: Interpolation towards impedance target (position + stiffness + damping)
 * - Phase 2: INDEFINITE holding at last target with PID engaged
 *
 * The system leaves the holding phase ONLY when:
 * - STOP / Emergency Stop command is received
 * - New impedance targets arrive via CAN (SET_IMPEDANCE frames)
 * - Joint control commands (CMD_PRETENSION, CMD_RELEASE, etc.)
 * - Safety error (angle limits, invalid encoder, etc.)
 * 
 * This ensures stable position maintenance until the next command.
 * 
 * CALIBRATION WORKFLOW:
 * 1. Run auto-mapping to collect motor-joint angle pairs
 * 2. Calculate linear equations from mapping data
 * 3. Save equations to flash for persistence
 * 4. System is ready for movement commands
 */

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <DirectEncoders.h>
#include <JointConfig.h>
#include <LKM_Motor.h>
#include <PID.h>
#include <hot_path.h>
#include <global.h>
#include <shared_data.h>
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// Forward declaration
class JointController;

enum SafetyViolationType : uint8_t {
  SAFETY_VIOLATION_LIMIT = 0,
  SAFETY_VIOLATION_MAPPING_LIMIT = 1,
  SAFETY_VIOLATION_MOTOR_RANGE = 2,
};

// Movement callback type
typedef bool (*MovementCallback)(JointController *controller, int dof_index, float progress);

// Movement data for a single DOF
struct DofMovementData {
  float start_angle;
  float target_angle;
  float current_angle;
  float next_angle;
  float output;
  uint8_t state; // 0=idle, 1=moving, 2=holding
};

// Linear regression coefficients
struct LinearRegressionCoefficients {
  float slope;     // Slope (m)
  float intercept; // Intercept (b)
  float r_squared; // Coefficient of determination R^2
  float mse;       // Mean squared error
  int data_points; // Number of points used
  bool valid;      // Validity flag
};

// Joint↔motor mapping representation for a DOF
enum MapMode : uint8_t {
  MAP_LINEAR = 0,    // y = slope*x + intercept (default)
  MAP_PIECEWISE = 1, // piecewise-linear interpolation over dof_mappings[] points (fine map)
  MAP_BILINEAR = 2,  // bilinear (q0,q1) grid over q0_axis x q1_axis
};

// Minimum captured points required for a usable piecewise (fine) map
static constexpr int MIN_FINE_POINTS = 8;

// 2D bilinear (q0,q1) grid descriptor + RAM storage (sibling to DofMappingData_t).
// GRID_M_MAX / GRID_N_MAX live in global.h (so the v9 2D flash record can size its arrays);
// they are visible here because JointController.h includes global.h above.
struct DofGridData_t {                 // ~859 B; sibling to DofMappingData_t (NOT a reinterpret)
  uint8_t grid_m;                      // active q0 rows (>=1)
  uint8_t grid_n;                      // active q1 cols (>= MIN_FINE_POINTS)
  bool bl_valid;
  float q0_axis[GRID_M_MAX];           // ascending DOF0 (q0) sweep points
  float q1_axis[GRID_N_MAX];           // ascending DOF1 (q1) points
  float agonist[GRID_M_MAX * GRID_N_MAX];     // row-major cell = r*grid_n + c, NEUTRAL baseline
  float antagonist[GRID_M_MAX * GRID_N_MAX];
};

// q0 source DOF for the bilinear DOF1 coupling.
static constexpr uint8_t Q0_DOF = 0;  // ankle plantar/dorsi DOF0 = the coupling source for the bilinear DOF1
static_assert(Q0_DOF < MAX_DOFS, "Q0_DOF out of range");

// Linear equations for a DOF
struct DofLinearEquations {
  LinearRegressionCoefficients agonist;    // Equation for agonist motor
  LinearRegressionCoefficients antagonist; // Equation for antagonist motor
  uint8_t dof_index;                       // DOF index
  bool calculated;                         // Whether equations were calculated
  float joint_safe_min;                    // Safe min joint angle
  float joint_safe_max;                    // Safe max joint angle
  float agonist_safe_min;                  // Safe min agonist motor angle
  float agonist_safe_max;                  // Safe max agonist motor angle
  float antagonist_safe_min;               // Safe min antagonist motor angle
  float antagonist_safe_max;               // Safe max antagonist motor angle
  bool limits_valid;                       // Whether limits were calculated
  uint8_t map_mode;                        // MapMode: LINEAR (slope/intercept) or PIECEWISE (fine map)
  bool pw_valid;                           // Piecewise (fine) map validated & usable
  bool bl_valid;                           // bilinear (2D) grid validated & usable
  float q0_nominal;                        // q0 of the captured 1D slice (used when no live q0 supplied); default 0
};

/**
 * @brief Class for joint control
 */
class JointController {
private:
  JointConfig config;             // Joint configuration
  LKM_Motor **motors;             // Array of motor pointers
  DirectEncoders *encoders;       // Direct encoder reader (MT6835 via SPI0)
  PID **pid_controllers;          // PID controller per motor (inner loop)
  PID **outer_pid_controllers;    // PID controller per DOF (outer loop) - handles filtering & anti-windup
  DofMovementData *dof_movement;  // Movement data per DOF
  DofMappingData_t *dof_mappings; // Mapping data per DOF (RAW from auto‑mapping)
  DofGridData_t dof_grids[MAX_DOFS]; // Bilinear (q0,q1) grid per DOF (zero-init: bl_valid=false on boot)
  
  // Inter-core flash save request (Core1 requests, Core0 executes)
  volatile bool _pending_flash_save = false;

  // Fine remap ("command and record"): operator commands positions under normal
  // impedance and records (joint, agonist, antagonist) points; on commit they
  // become a per-DOF piecewise map. Scratch buffer holds one DOF's capture.
  bool _fine_capture_active = false;
  uint8_t _fine_capture_dof = 0;
  float _fc_joint[MAX_MAPPING_DATA_SIZE];
  float _fc_agonist[MAX_MAPPING_DATA_SIZE];
  float _fc_antagonist[MAX_MAPPING_DATA_SIZE];
  int _fc_size = 0;
  // Per-DOF co-contraction offset the cascade applied this cycle
  // (cascade_influence * 0.5 * stiffness_ref_effective). recordFinePoint subtracts it from
  // the measured agonist / adds it to the antagonist so the stored fine-map is the NEUTRAL
  // (zero co-contraction) baseline — otherwise the cascade would double-apply stiffness.
  float _cocontraction_offset[MAX_DOFS] = {0.0f};

  // Grid capture ("2D bilinear map"): the host drives a per-row 1D fine capture (reusing the
  // _fc_* scratch), then harvests each settled row at a q0 coordinate via recordGridRow, which
  // resamples it onto a shared q1_axis so the grid stays rectangular. The in-progress grid is
  // kept bl_valid=false; only a successful commitGridCapture makes it usable (MAP_BILINEAR).
  bool _grid_capture_active = false;
  uint8_t _grid_dof = 0;
  // Capture SCRATCH: the in-progress grid is built here and NEVER touches the live dof_grids[dof]
  // until a successful commitGridCapture atomically publishes it. An abort therefore leaves any
  // previously-committed live grid intact, and a torn capture can never be read by the control loop.
  DofGridData_t _grid_scratch;

  // Blended-row monotonicity guard for a 2D grid (the hysteresis discriminator). Factored out so it
  // runs at BOTH commitGridCapture (capture-time) AND loadGridFromFlash (boot-time defense against
  // flash corruption). Returns true iff every interior blend of adjacent q0-rows stays monotonic.
  bool gridBlendedRowsMonotonic(const DofGridData_t &g);

  // Per-row monotonicity guard: every stored raw row (agonist + antagonist) must be strictly
  // monotonic. Run at commit (validate the captured rows) and at load (defense-in-depth vs flash
  // bit-rot, since the blended-row guard alone could pass on individually-corrupt rows).
  bool gridRowsMonotonic(const DofGridData_t &g);

  // Atomically publish a validated grid into the LIVE dof_grids[dof] slot while the control loop on
  // Core1 may be reading it (commit runs on Core1, the CAN flash-load on Core0). Demote map_mode away
  // from BILINEAR first (so the loop stops using the about-to-be-overwritten grid), barrier, copy the
  // grid + recompute safe limits, barrier, then promote map_mode back to BILINEAR last.
  void publishBilinearGrid(uint8_t dof, const DofGridData_t &src, float q0_nominal);

  // Interpolation functions
  float interpolate_data(float target_value, float *data1, float *data2, int size);

  // Cache of calculated linear equations
  DofLinearEquations *linear_equations;

  // Flag: offsets recalculated after boot
  bool *motor_offsets_calibrated;

  // Saved motor offsets (for flash persistence and boot-time validation)
  struct SavedDofOffset {
    float agonist_offset;
    float antagonist_offset;
    float joint_angle_at_calib;
    bool valid;
  };
  SavedDofOffset _saved_offsets[MAX_DOFS];
  volatile bool _pending_offsets_save = false;  // Core1 sets, Core0 saves to flash

  // Outer‑loop (cascade) control parameters per DOF
  float *outer_loop_kp_values;
  float *outer_loop_ki_values;
  float *outer_loop_kd_values;
  float *stiffness_ref_values;
  float *cascade_influence_values;

  static constexpr float DEFAULT_INNER_LOOP_KP = PID_DEFAULT_INNER_KP;
  static constexpr float DEFAULT_INNER_LOOP_KI = PID_DEFAULT_INNER_KI;
  static constexpr float DEFAULT_INNER_LOOP_KD = PID_DEFAULT_INNER_KD;

  static constexpr float DEFAULT_OUTER_LOOP_KP     = PID_DEFAULT_OUTER_KP;
  static constexpr float DEFAULT_OUTER_LOOP_KI     = PID_DEFAULT_OUTER_KI;
  static constexpr float DEFAULT_OUTER_LOOP_KD     = PID_DEFAULT_OUTER_KD;
  static constexpr float DEFAULT_STIFFNESS_REF_DEG = PID_DEFAULT_STIFFNESS_DEG;
  static constexpr float DEFAULT_CASCADE_INFLUENCE = PID_DEFAULT_CASCADE;

  // Outer loop PID configuration constants
  static constexpr float DEFAULT_OUTER_LOOP_TAU        = 0.02f;  // Derivative filter time constant (20ms)
  static constexpr float DEFAULT_OUTER_LOOP_TS         = 0.01f;  // Initial sampling period (updated at runtime)
  static constexpr float DEFAULT_MAX_DELTA_THETA       = 30.0f;  // Maximum correction in degrees

  // Encoder read tracking to detect SPI spikes
  float *last_valid_angles;       // Last valid readings per DOF
  uint64_t *last_read_timestamps; // Timestamps of last readings (microseconds)
  uint32_t *spike_counters;       // Spike counters per DOF
  static constexpr float SPIKE_DETECTION_MARGIN = 1.5f; // Safety margin (150% of max speed)
  static constexpr int MAX_CONSECUTIVE_READS    = 3;    // Reads to confirm a value

  // Direct-drive feedback cache (updated on Core1 from motor internal encoder reads)
  float *direct_feedback_angles;
  float *direct_feedback_velocities;
  bool *direct_feedback_valid;

  // Private method to validate readings
  float getValidatedAngle(uint8_t dof_index, bool &is_valid);

public:
  // ==========================================================================
  // INITIALIZATION & LIFECYCLE
  // ==========================================================================

  /**
   * @brief Constructor
   * @param cfg Joint configuration
   * @param can CAN interface pointer
   * @param enc Direct encoders pointer (MT6835 via SPI0)
   */
  JointController(const JointConfig &cfg, MCP_CAN *can, DirectEncoders *enc);

  /**
   * @brief Destructor
   */
  ~JointController();

  /**
   * @brief Initialize controller
   * @return true if initialization succeeded
   */
  bool init();

  // ==========================================================================
  // CONFIGURATION & STATUS
  // ==========================================================================

  /**
   * @brief Get joint configuration
   * @return Reference to joint configuration
   */
  const JointConfig &getConfig() const {
    return config;
  }

  bool dofSupportsPretension(uint8_t dof_index) const {
    return dof_index < config.dof_count && config.dofs[dof_index].capabilities.supports_pretension;
  }

  bool dofSupportsRecalcOffset(uint8_t dof_index) const {
    return dof_index < config.dof_count && config.dofs[dof_index].capabilities.supports_recalc_offset;
  }

  bool dofSupportsAutoMapping(uint8_t dof_index) const {
    return dof_index < config.dof_count && config.dofs[dof_index].capabilities.supports_auto_mapping;
  }

  bool isDirectDriveDof(uint8_t dof_index) const {
    return dof_index < config.dof_count &&
           config.dofs[dof_index].drive_type == DRIVE_DIRECT_DRIVE;
  }

  // ==========================================================================
  // SAFETY & VALIDATION
  // ==========================================================================

  /**
   * @brief Run safety checks for all DOFs based on current angles
   * @param violation_message Description in case of violation
   * @param check_motors If true also check associated motor ranges
   * @return true if all DOFs are within limits, false otherwise
   */
  bool checkSafetyForAllDofs(char *violation_message, size_t violation_msg_size,
                             bool check_motors = false);

  /**
   * @brief Run safety checks for a single DOF
   * @param dof_index DOF index
   * @param current_angle Current joint angle (degrees)
   * @param violation_message Description in case of violation
   * @param check_motors If true also check associated motor ranges
   * @return true if DOF is within limits, false otherwise
   */
  // violation_message is a caller STACK buffer (heap-free v2 pass 2026-07-06: the old
  // String& forced a malloc(1)+free through the cross-core mutex EVERY moving outer cycle,
  // even with no violation). Built ONLY on the violation branch; empty string otherwise.
  bool checkSafetyForDof(uint8_t dof_index, float current_angle, char *violation_message,
                         size_t violation_msg_size, bool check_motors = false,
                         SafetyViolationType *violation_type = nullptr);
  bool getMappingSafeRange(uint8_t dof_index, float &min_safe, float &max_safe);
  bool canDirectDriveRecoverTowardSafeRange(uint8_t dof_index, float current_angle,
                                            float target_angle);

  // ==========================================================================
  // MOTOR & PID CONTROL
  // ==========================================================================

  /**
   * @brief Apply default PID values to all motors
   * @param log_details If true, print details to serial log
   */
  void applyDefaultPidTunings(bool log_details = true);

  /**
   * @brief Get a specific motor
   * @param motor_index Motor index
   * @return Motor pointer or nullptr if index is invalid
   */
  LKM_Motor *getMotor(uint8_t motor_index);

  // Get PID state for a specific motor
  bool getPid(uint8_t dof_index, uint8_t motor_type, float &kp, float &ki, float &kd, float &tau);

  // Set PID parameters for a specific motor
  bool setPid(uint8_t dof_index, uint8_t motor_type, float kp, float ki, float kd, float tau);

  // Get outer loop (cascade) parameters for a specific DOF
  bool getOuterLoopParameters(uint8_t dof_index, float &kp, float &ki, float &kd,
                              float &stiffness_deg, float &cascade_influence) const;

  // Set outer loop (cascade) parameters for a specific DOF
  bool setOuterLoopParameters(uint8_t dof_index, float kp, float ki, float kd, float stiffness_deg,
                              float cascade_influence);

  /**
   * @brief Get outer loop PID controller for a DOF
   * @param dof_index DOF index
   * @return PID pointer or nullptr if index is invalid
   *
   * Use this to access the outer PID directly for control operations.
   * The returned PID handles derivative filtering and anti-windup automatically.
   */
  HOT_INLINE PID *getOuterPID(uint8_t dof_index) {
    if (dof_index >= config.dof_count || outer_pid_controllers == nullptr) return nullptr;
    return outer_pid_controllers[dof_index];
  }

  /**
   * @brief Reset outer loop PID state for a DOF
   * @param dof_index DOF index
   *
   * Call this when starting a new movement sequence to clear accumulated
   * integral and derivative state. Not needed for smooth transitions.
   */
  void resetOuterPID(uint8_t dof_index) {
    if (dof_index < config.dof_count && outer_pid_controllers && outer_pid_controllers[dof_index]) {
      outer_pid_controllers[dof_index]->reset();
    }
  }

  /**
   * @brief Reset outer loop PID state for all DOFs
   */
  void resetAllOuterPIDs() {
    for (int i = 0; i < config.dof_count; i++) {
      resetOuterPID(i);
    }
  }

  /**
   * @brief Update outer loop sampling period for all DOFs
   * @param new_ts New sampling period in seconds
   *
   * Call this when the outer loop frequency changes (e.g., different OUTER_LOOP_DIV).
   * This ensures the PID integral and derivative terms are scaled correctly.
   */
  void setOuterLoopSamplingPeriod(float new_ts);

  /**
   * @brief Update inner (motor) loop sampling period for all motor PIDs
   * @param new_ts New sampling period in seconds
   *
   * The constructors seed Ts from the preset motion.sampling_period, which can
   * diverge from the runtime inner_loop_period_us; call this when the inner loop
   * frequency changes so Ki*Ts / Kd/Ts scale on the real period.
   */
  void setInnerLoopSamplingPeriod(float new_ts);

  // ==========================================================================
  // PRETENSION & RELEASE
  // ==========================================================================

  /**
   * @brief Apply pretension to a specific DOF
   * @param dof_index DOF index
   * @param torque Pretension torque (0 to use configured value)
   * @param duration_ms Duration in ms (0 to use configured value)
   * @return true on success
   */
  bool pretension(uint8_t dof_index, int torque = 0, int duration_ms = 0);

  /**
   * @brief Pretension all joint DOFs
   * @return true if successful for all DOFs
   */
  bool pretensionAll();

  /**
   * @brief Release a specific DOF (torque opposite to pretension)
   * @param dof_index DOF index
   * @param torque Release torque (0 to use configured value with inverted sign)
   * @param duration_ms Duration in ms (0 to use configured value)
   * @return true on success
   */
  bool release(uint8_t dof_index, int torque = 0, int duration_ms = 0);

  /**
   * @brief Release all joint DOFs (torque opposite to pretension)
   * @return true if successful for all DOFs
   */
  bool releaseAll();

  /**
   * @brief Set current position as zero for a DOF without moving the joint
   * @param dof_index DOF index
   * @return true on success
   */
  bool setZeroCurrentPos(uint8_t dof_index);

  /**
   * @brief Recalculate motor zero offsets with pretension
   * @param dof_index DOF index
   * @param pretension_torque Pretension torque to apply
   * @param pretension_duration_ms Duration to maintain pretension
   * @return true on success
   */
  bool recalculateMotorOffsets(uint8_t dof_index, float pretension_torque,
                                int pretension_duration_ms);

  // ==========================================================================
  // MOVEMENT COMMANDS
  // ==========================================================================

  /**
   * @brief Stop all motors
   */
  void stopAllMotors();

  /**
   * @brief S2 CARRY choke point — abandon any pair the Stage-2 cross-cycle CARRY left in flight.
   *
   * The last pair fired in a control cycle can stay outstanding across the cycle boundary (knob
   * bit1). Any motor-drive entry point reachable from pollHostCan that bypasses the control loop's
   * injection resolve (stopAllMotors, pretension/release, recalc, the diag-loop arm*) must call
   * this FIRST so a carried pair's _fire_pending flags + stale RX frames are cleared before it
   * touches the motor bus. Defined in JointController_ControlLoop.cpp (owner of the s2_carry slot).
   * Core-affinity guarded: a no-op (+ wrong-core counter) if called from core0.
   */
  void abandonCarriedPair();

  /**
   * @brief Stop motors of a specific DOF
   * @param dof_index DOF index
   */
  void stopDofMotors(uint8_t dof_index);

  /**
   * @brief Read current angle of a DOF
   * @param dof_index DOF index
   * @param is_valid Boolean reference for angle validity
   * @return Angle in degrees
   */
  float getCurrentAngle(uint8_t dof_index, bool &is_valid);

  /**
   * @brief Get maximum speed for a DOF
   * @param dof_index DOF index
   * @return Maximum speed in rad/s, or 0.0 if DOF invalid
   */
  float getMaxSpeed(uint8_t dof_index) const {
    if (dof_index >= config.dof_count) return 0.0f;
    return config.dofs[dof_index].motion.max_speed;
  }

  /**
   * @brief Check if an angle is within safe limits derived from auto‑mapping
   * @param dof_index DOF index
   * @param angle Angle to verify
   * @return true if the angle is within limits saved with linear equations
   */
  bool isAngleInMappingLimits(uint8_t dof_index, float angle);

  /**
   * @brief Check if an angle is within a DOF's limits
   * @param dof_index DOF index
   * @param angle Angle to verify
   * @return true if the angle is within limits
   */
  bool isAngleInLimits(uint8_t dof_index, float angle);

  /**
   * @brief Check if DOF motors are within their mapping range
   * @param dof_index DOF index
   * @param[out] violation_message Descriptive message in case of violation
   * @return true if all motors are within limits, false if at least one is out of range
   *
   * This function verifies that motors are within mapped ranges with safety margin.
   * If a motor exceeds these limits, it could indicate tendon breakage.
   */
  bool checkMotorsInRange(uint8_t dof_index, char *violation_message, size_t violation_msg_size);

  /**
   * @brief Execute joint-motor mapping
   * @param dof_index DOF index to map
   * @param out1 Torque for extensor
   * @param dps2 Flexor speed
   * @param angle_increment Angular increment
   * @param max_angle Maximum angle
   * @return true if mapping succeeded
   */
  

  /**
   * @brief Get mapping data for a DOF
   * @param dof_index DOF index
   * @return Pointer to mapping data or nullptr if index is invalid
   */
  DofMappingData_t *getMappingData(uint8_t dof_index);

  /**
   * @brief Set mapping completed flag for a DOF
   * @param dof_index DOF index
   * @return true on success
   */
  

  // ==========================================================================
  // AUTOMATIC CALIBRATION (AUTO-MAPPING)
  // ==========================================================================

  /**
   * @brief Start automatic mapping for all joint DOFs
   * @param auto_mapping_state Reference to automatic mapping state
   * @param tensioning_torque Tensioning torque during acquisition (0 = use
   * configured value)
   * @param steps Array of angular steps for each DOF (nullptr = use configured values)
   * @param settle_time_ms Settling time in ms (0 = use configured value)
   * @return true if start succeeded
   */
  bool startAutoMapping(AutoMappingState_t &auto_mapping_state, float tensioning_torque = 0,
                        float *steps = nullptr, int settle_time_ms = 0);

  /**
   * @brief Stop automatic mapping
   * @param auto_mapping_state Reference to automatic mapping state
   * @return true if stop succeeded
   */
  bool stopAutoMapping(AutoMappingState_t &auto_mapping_state);

  /**
   * @brief Update automatic mapping state
   * @param auto_mapping_state Reference to automatic mapping state
   * @return Status code: 0=in progress, 1=point acquired, 2=completed, 3=error
   */
  int updateAutoMapping(AutoMappingState_t &auto_mapping_state);

  /**
   * @brief Move to next point in automatic mapping
   * @param auto_mapping_state Reference to automatic mapping state
   * @return true if a new point is available, false if mapping is complete
   */
  bool moveToNextMappingPoint(AutoMappingState_t &auto_mapping_state);

  /**
   * @brief Apply necessary torques to reach target point
   * @param auto_mapping_state Reference to automatic mapping state
   */
  void applyTorquesForTargetPosition(AutoMappingState_t &auto_mapping_state);

  /**
   * @brief Verify whether the target position has been reached
   * @param auto_mapping_state Reference to auto‑mapping state
   * @return true if the target position has been reached
   */
  bool isPositionReached(AutoMappingState_t &auto_mapping_state);

  /**
   * @brief Acquire current point in auto‑mapping
   * @param auto_mapping_state Reference to auto‑mapping state
   * @return true if acquisition succeeded
   */
  bool acquireCurrentPoint(AutoMappingState_t &auto_mapping_state);

  /**
   * @brief Transfer data from auto‑mapping to DofMappingData_t structures
   * @param auto_mapping_state Reference to auto‑mapping state
   * @return true if the transfer succeeded
   */
  bool transferAutoMappingData(const AutoMappingState_t &auto_mapping_state);

  // ==========================================================================
  // FINE REMAP ("command and record") — manual point capture with cascade PID
  // running, producing a per-DOF piecewise (non-linear) joint↔motor map.
  // ==========================================================================

  /** @brief Begin a fine capture session for one tendon DOF (clears scratch buffer). */
  bool startFineCapture(uint8_t dof_index);

  /** @brief Record the current settled (joint, agonist, antagonist) sample for the active DOF.
   *  Uses cached angles (no extra SPI). Returns false if not active / not settled / buffer full. */
  bool recordFinePoint();

  /** @brief End capture without committing (keeps buffer for a subsequent commit). */
  void stopFineCapture();

  /** @brief Validate the captured points and, on success, install them as the DOF's
   *  piecewise map (map_mode=PIECEWISE). On failure the previous map is left intact. */
  bool commitFineCapture(uint8_t dof_index);

  /** @brief Discard the capture session and buffer. */
  void abortFineCapture();

  /** @brief Whether a fine capture is currently active. */
  bool isFineCaptureActive() const { return _fine_capture_active; }

  /** @brief DOF being captured (valid when isFineCaptureActive()). */
  uint8_t fineCaptureDof() const { return _fine_capture_dof; }

  /** @brief Number of points captured so far in the active/last session. */
  int fineCaptureCount() const { return _fc_size; }

  // ==========================================================================
  // GRID CAPTURE (2D bilinear map) — host drives a 1D fine capture per q0 row,
  // then harvests it as a grid row resampled onto a shared q1_axis. Stays DEAD
  // until commitGridCapture validates the full grid and sets MAP_BILINEAR.
  // ==========================================================================

  /** @brief Begin a grid-capture session for one tendon DOF (clears the in-progress grid,
   *  bl_valid=false). The host then drives per-row 1D fine captures + recordGridRow. */
  bool startGridCapture(uint8_t dof);

  /** @brief Harvest the CURRENT 1D fine-capture scratch as grid row `row` at q0 coordinate `q0`,
   *  resampled onto the shared q1_axis. Row 0 must be recorded first (it establishes q1_axis).
   *  Returns false (and logs) if the row is non-monotonic or out of order. */
  bool recordGridRow(uint8_t dof, uint8_t row, float q0);

  /** @brief Validate the full grid (ascending q0, blended-row monotonicity) and, on success,
   *  install it as the DOF's bilinear map (map_mode=BILINEAR). On failure leaves bl_valid=false. */
  bool commitGridCapture(uint8_t dof);

  /** @brief Discard the grid-capture session (leaves the committed grid, if any, untouched). */
  void abortGridCapture();

  // ==========================================================================
  // FLASH STORAGE (PERSISTENCE)
  // ==========================================================================

  /**
   * @brief Load only PID parameters from flash (new lightweight system)
   * @return true if loading succeeded
   */
  bool loadPIDDataFromFlash();

  /**
   * @brief Save only PID parameters to flash (new lightweight system)
   * @return true if saving succeeded
   */
  bool savePIDDataToFlash();

  /**
   * @brief Save linear equations to flash
   * @return true if saving succeeded
   * @note Should only be called from Core0 to avoid flash access conflicts
   */
  bool saveLinearEquationsToFlash();

  /**
   * @brief Load linear equations from flash
   * @return true if loading succeeded
   */
  bool loadLinearEquationsFromFlash();

  /**
   * @brief Save the per-DOF piecewise (fine) map to flash (v8 record)
   *
   * Co-located with the v5 linear-eq record in the LINEAR_EQ sector. Only DOFs whose committed
   * map is PIECEWISE/valid with >= MIN_FINE_POINTS points are persisted; others stay LINEAR.
   * @return true if the write was issued
   * @note Should only be called from Core0 to avoid flash access conflicts
   */
  bool saveFineMapToFlash();

  /**
   * @brief Restore the per-DOF piecewise (fine) map from flash (v8 record)
   *
   * Best-effort: absence/corruption/validation failure leaves the DOF LINEAR (never bricks).
   * @return true if a valid fine-map record was found
   */
  bool loadFineMapFromFlash();

  /**
   * @brief Save the per-DOF 2D bilinear grid to flash (v9 record)
   *
   * Co-located with the v5 linear-eq and v8 fine-map records in the LINEAR_EQ sector. Only DOFs
   * whose committed map is BILINEAR/valid with a usable grid are persisted; others are left out.
   * @return true if the write was issued
   * @note Should only be called from Core0 to avoid flash access conflicts
   */
  bool saveGridToFlash();

  /** Deliberately write EMPTY v8/v9 refined-map records (bypass of the accidental-clobber
   *  guards BY DESIGN): call after a successful coarse re-map, when every refined map is
   *  stale-by-construction, so a reboot cannot re-promote old-geometry maps. */
  bool invalidateRefinedMapsInFlash();

  /**
   * @brief Restore the per-DOF 2D bilinear grid from flash (v9 record)
   *
   * Best-effort overlay on top of the v5/v8 baseline: absence/corruption/validation failure leaves
   * the affected DOF at its v5/v8 mode (never bricks). Each restored slot re-runs the blended-row
   * monotonicity guard (defense-in-depth against flash corruption).
   * @return true if at least one valid grid was restored
   */
  bool loadGridFromFlash();

  /**
   * @brief Recalculate safe limits based on current equations and physical limits
   * 
   * This recalculates joint_safe_min/max using the current equation data and 
   * the physical limits from config, then saves to flash. Useful when the 
   * limit calculation algorithm changes without needing to redo auto-mapping.
   * 
   * @return true if recalculation and save succeeded
   */
  bool recalculateSafeLimits();
  
  /**
   * @brief Check if Core1 has requested a flash save
   * @return true if flash save is pending
   */
  bool isPendingFlashSave() const { return _pending_flash_save; }
  
  /**
   * @brief Clear the pending flash save flag (call after saving)
   */
  void clearPendingFlashSave() { _pending_flash_save = false; }

  // --- Motor offset persistence (for smart recalc detection) ---

  /**
   * @brief Check if Core1 has requested saving motor offsets to flash
   */
  bool isPendingOffsetsSave() const { return _pending_offsets_save; }

  /**
   * @brief Clear the pending offsets save flag (call after saving)
   */
  void clearPendingOffsetsSave() { _pending_offsets_save = false; }

  /**
   * @brief Save motor offsets to flash (called from Core0)
   * @return true if saved successfully
   */
  bool saveMotorOffsetsToFlash();

  /**
   * @brief Load motor offsets from flash (called at boot from Core0)
   * @return true if valid offsets were loaded
   */
  bool loadMotorOffsetsFromFlash();

  /**
   * @brief Result of offset validation check
   */
  struct OffsetValidationResult {
    bool valid;                  // true = offsets still good, skip recalc
    float error_agonist_deg;     // Deviation for agonist motor (degrees)
    float error_antagonist_deg;  // Deviation for antagonist motor (degrees)
    bool has_saved_data;         // true = saved offsets exist in flash
  };

  /**
   * @brief Validate saved offsets against current motor positions
   * 
   * Reads raw motor angles via CAN, applies saved offsets, and compares
   * with expected angles from linear equations. Must run on Core1 (CAN access).
   * 
   * @param dof_index DOF to validate
   * @return Validation result with error magnitudes
   */
  OffsetValidationResult validateSavedOffsets(uint8_t dof_index);

  /**
   * @brief Apply saved offsets from flash to motor encoder objects
   *
   * Used during startup when validateSavedOffsets confirms offsets are still
   * valid (motors kept power). Skips the full recalc pretension sequence.
   *
   * @param dof_index DOF to apply offsets for
   * @return true if offsets applied successfully
   */
  bool applySavedOffsetsToMotors(uint8_t dof_index);


  /**
   * @brief Check offset drift using cached motor angles (zero CAN overhead)
   * 
   * Uses cached_motor_angles (already read by control loop) and current
   * encoder offsets to detect if motor offsets have drifted since calibration.
   * Called once when entering HOLDING after a movement.
   * 
   * @param dof_index DOF to check
   * @return Validation result — valid=true means no drift detected
   */
  OffsetValidationResult checkOffsetDriftFromCache(uint8_t dof_index);

  /**
   * @brief Check whether auto‑mapping is active
   * @return true if auto‑mapping is active
   */
  bool isAutoMappingActive(const AutoMappingState_t &auto_mapping_state) const {
    return auto_mapping_state.active;
  }

  // REMOVED: testPidDirection - now we use static parameters agonist_drives_positive_movement

  // Reset counters used in auto mapping
  void resetAutoMappingCounters(AutoMappingState_t &auto_mapping_state);

  // ==========================================================================
  // LINEAR EQUATIONS (MOTOR-JOINT MAPPING)
  // ==========================================================================

  // Compute linear equations for all DOFs using linear regression
  bool calculateLinearEquationsFromMappingData();

  // Compute linear regression for a data set
  LinearRegressionCoefficients calculateLinearRegression(float *x_data, float *y_data,
                                                         int data_count);

  // Get linear equations for a specific DOF
  DofLinearEquations *getLinearEquations(uint8_t dof_index);

  // Quick check of linear equations status for a DOF
  bool hasValidEquations(uint8_t dof_index) const;

  // Compute motor angle using linear equations
  // Version with separate inputs for agonist and antagonist.
  // q_other: live q0 (DOF0) for the bilinear (MAP_BILINEAR) branch; NAN -> use eq.q0_nominal.
  bool calculateMotorAnglesWithEquations(uint8_t dof_index, float agonist_joint_angle,
                                         float antagonist_joint_angle, float &agonist_angle,
                                         float &antagonist_angle, float q_other = NAN);

  // Compute joint angle using inverse linear equations (motor → joint)
  // Unified version with separate inputs for agonist and antagonist.
  // q_other: live q0 (DOF0) for the bilinear (MAP_BILINEAR) branch; NAN -> use eq.q0_nominal.
  bool calculateJointAnglesWithEquations(uint8_t dof_index, float agonist_motor_angle,
                                         float antagonist_motor_angle, float &agonist_joint_angle,
                                         float &antagonist_joint_angle, float q_other = NAN);

  // ==========================================================================
  // SYSTEM STATUS & MONITORING
  // ==========================================================================

  /**
   * @brief Verify if the system is ready for movement
   * @return true if the system has linear equations available and offsets calibrated
   */
  bool isSystemReadyForMovement();

  /**
   * @brief Mark a DOF as movement-ready after startup validation.
   *
   * Used by non-tendon startup paths (e.g. direct-drive DOFs) that do not
   * populate tendon offset/equation state but still require an explicit
   * startup-time validation before movement is enabled.
   */
  void setMovementReadyForDof(uint8_t dof_index, bool ready);
  bool hasSavedReference(uint8_t dof_index) const;
  void updateDirectDriveFeedback(uint8_t dof_index, float angle_deg, float velocity_deg_s, bool valid);
  bool getDirectDriveFeedback(uint8_t dof_index, float &angle_deg, float &velocity_deg_s) const;

  /**
   * @brief Get the number of SPI spikes detected for a DOF
   * @param dof_index DOF index
   * @return Number of spikes detected since boot
   */
  uint32_t getSpikeCount(uint8_t dof_index) const {
    if (dof_index >= config.dof_count)
      return 0;
    return spike_counters[dof_index];
  }

  // ==========================================================================
  // IMPEDANCE-BASED CASCADE CONTROL LOOP
  // ==========================================================================

  /**
   * @brief Execute cascade control loop for all DOFs (500 Hz)
   *
   * Main entry point for control execution. Call from core1_loop() at 500 Hz.
   * Implements cascade control (outer joint PID + inner motor PID) with
   * impedance rolling-segment generation for smooth movement.
   *
   * - Outer loop runs every outer_loop_divisor cycles (default 1 = 500 Hz)
   * - Inner loop @ 500 Hz (every cycle)
   * - MOVING: impedance segment interpolation toward target
   * - HOLDING: maintain position with cascade PID active
   *
   * @return true if any DOF is actively moving
   */
  bool executeControlLoop();

  // Free/compliant hand-capture cycle — SEPARATE from executeControlLoop (the validated control loop
  // is never entered while free-capture is active). Zero-torques both tendons of `dof` (back-drivable)
  // and appends one hi-rate record this cycle. If `pending_exit`, re-seeds the impedance hold at the
  // current pose and returns true (caller clears free_capture_active so control resumes holding there
  // -> no jerk). Returns false otherwise.
  bool runFreeCaptureCycle(uint8_t dof, bool pending_exit);

  // True only if `dof` can safely ENTER free-capture: motor cache warm, rev-tracking bootstrapped on
  // both tendons, and the DOF currently HOLDING. The ENTER handler rejects otherwise (else free-capture
  // would zero-torque with no/garbage angles and leave the fragile joint free-to-fall).
  bool isFreeCaptureReady(uint8_t dof);

  // Vel-test (diag A): characterise the motor's internal velocity/position loop vs the torque cascade.
  // armVelTest validates + picks the direction at ENTER (pos_mode also precomputes the map far-end target);
  // runVelTestCycle drives one tendon in velocity (0xA2 setSpeed) OR position (0xA4 angle+maxSpeed) + the
  // other holding torque, records q, and de-powers (fail-safe) on EXIT / position guard.
  bool armVelTest(uint8_t dof, bool pos_mode);
  bool runVelTestCycle(uint8_t dof, bool pending_exit);

  // TORQUE-SWEEP (diag 0x0C/0x0D): armTorqueSweep seeds a firmware-driven smooth impedance-segment ramp and
  // keeps impedance active so the NORMAL executeControlLoop cascade tracks it (clean matched-speed torque
  // baseline, no host q_x100 stepping); torqueSweepStop is the de-power fail-safe on EXIT / backstop.
  bool armTorqueSweep(uint8_t dof);
  void torqueSweepStop();

  // fireAngle2 NO-MOTION ping (diag 0x10/0x11): Loop 2 step-2 primitive confirm — hold BOTH motors at their
  // current angle via non-blocking 0xA4 (fireAngle2) + collectPair, no motion, to validate the 0xA4 send +
  // auto-reply + collectPair routing before any Loop 2 motion. armFireAngle2Ping seeds; runFireAngle2PingCycle
  // drives one guarded cycle (joint motion-abort -> de-power).
  bool armFireAngle2Ping(uint8_t dof);
  bool runFireAngle2PingCycle(uint8_t dof, bool pending_exit);
};

#endif // JOINT_CONTROLLER_H
