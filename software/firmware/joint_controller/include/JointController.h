/**
 * @file JointController.h
 * @brief Multi-DOF joint controller with cascade PID and tendon-driven actuation
 * 
 * This class manages the complete control of a multi-DOF joint in the humanoid robot.
 * It implements a sophisticated cascade control architecture with:
 * 
 * ARCHITECTURE:
 * - Outer loop (100 Hz): Position control per DOF using joint encoders
 * - Inner loop (500 Hz): Torque control per motor using motor encoders
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
 * Movement commands implement an open-ended PID control:
 * - Phase 1: Trajectory-controlled motion towards the target
 * - Phase 2: INDEFINITE holding at target with PID engaged
 * 
 * The system leaves the holding phase ONLY when:
 * - STOP command is received
 * - A new movement command (CMD_MOVE_MULTI_DOF)
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

#include <Encoders.h>
#include <JointConfig.h>
#include <LKM_Motor.h>
#include <PID.h>
#include <global.h>
#include <shared_data.h>
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// Forward declaration
class JointController;

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
};

/**
 * @brief Class for joint control
 */
class JointController {
private:
  JointConfig config;             // Joint configuration
  LKM_Motor **motors;             // Array of motor pointers
  Encoders *encoders;             // Encoders pointer
  PID **pid_controllers;          // PID controller per motor
  DofMovementData *dof_movement;  // Movement data per DOF
  DofMappingData_t *dof_mappings; // Mapping data per DOF (RAW from auto‑mapping)

  // Interpolation functions
  float interpolate_data(float target_value, float *data1, float *data2, int size);

  // Cache of calculated linear equations
  DofLinearEquations *linear_equations;

  // Flag: offsets recalculated after boot
  bool *motor_offsets_calibrated;

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

  // Encoder read tracking to detect SPI spikes
  float *last_valid_angles;       // Last valid readings per DOF
  uint64_t *last_read_timestamps; // Timestamps of last readings (microseconds)
  uint32_t *spike_counters;       // Spike counters per DOF
  static constexpr float SPIKE_DETECTION_MARGIN = 1.5f; // Safety margin (150% of max speed)
  static constexpr int MAX_CONSECUTIVE_READS    = 3;    // Reads to confirm a value

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
   * @param enc Encoders pointer
   */
  JointController(const JointConfig &cfg, MCP_CAN *can, Encoders *enc);

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

  // ==========================================================================
  // SAFETY & VALIDATION
  // ==========================================================================

  /**
   * @brief Run safety checks for all DOFs based on current angles
   * @param violation_message Description in case of violation
   * @param check_motors If true also check associated motor ranges
   * @return true if all DOFs are within limits, false otherwise
   */
  bool checkSafetyForAllDofs(String &violation_message, bool check_motors = false);

  /**
   * @brief Run safety checks for a single DOF
   * @param dof_index DOF index
   * @param current_angle Current joint angle (degrees)
   * @param violation_message Description in case of violation
   * @param check_motors If true also check associated motor ranges
   * @return true if DOF is within limits, false otherwise
   */
  bool checkSafetyForDof(uint8_t dof_index, float current_angle, String &violation_message,
                         bool check_motors = false);

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
   * @brief Coordinated multi‑DOF movement with double‑loop cascade control
   *
   * Implements a cascade control with:
   * - Outer loop (100 Hz): joint PID to compute delta_theta
   * - Inner loop (500 Hz): motor PID to follow references
   *
   * CONTROL STRUCTURE:
   * 1. The outer PID computes delta_theta from joint error (q_des - q_curr)
   * 2. Motor references are computed as:
   *    - theta_A_ref = theta_0 + 0.5 * delta_theta + 0.5 * stiffness_ref
   *    - theta_B_ref = theta_0 + 0.5 * delta_theta - 0.5 * stiffness_ref
   * 3. Inner PIDs control motors to follow these references
   *
   * ADVANTAGES:
   * - Higher precision in joint control
   * - Better tendon tension management
   * - Automatic backlash compensation
   *
   * @param target_angles Array of target angles per DOF
   * @param active_dofs_mask Mask of DOFs to move (bit 0 = DOF 0, etc.)
   * @param path_type Path type (PATH_LINEAR, PATH_TRIG, PATH_QUAD)
   * @param sync_strategy Synchronization strategy (0=none, 1=duration, 2=speed)
   * @param max_speed Maximum speed in rad/s
   * @param accel_time Acceleration time in seconds
   * @param steps Number of steps for trajectory
   * @param sampling_period Base sampling period in microseconds (for inner loop)
   * @param verbose Verbose output flag
   * @param max_torque Maximum torque in motor units
   * @return MovementResult with exit code and message
   */
  MovementResult moveMultiDOF_cascade(float *target_angles, uint8_t active_dofs_mask, int path_type,
                                      int sync_strategy, float max_speed, float accel_time,
                                      int steps, uint64_t sampling_period, bool verbose,
                                      int max_torque, bool smooth_transition = false);

  /**
   * @brief Stop all motors
   */
  void stopAllMotors();

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
  bool checkMotorsInRange(uint8_t dof_index, String &violation_message);

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
   */
  bool saveLinearEquationsToFlash();

  /**
   * @brief Load linear equations from flash
   * @return true if loading succeeded
   */
  bool loadLinearEquationsFromFlash();

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
  // Version with separate inputs for agonist and antagonist
  bool calculateMotorAnglesWithEquations(uint8_t dof_index, float agonist_joint_angle,
                                         float antagonist_joint_angle, float &agonist_angle,
                                         float &antagonist_angle);

  // Compute joint angle using inverse linear equations (motor → joint)
  // Unified version with separate inputs for agonist and antagonist
  bool calculateJointAnglesWithEquations(uint8_t dof_index, float agonist_motor_angle,
                                         float antagonist_motor_angle, float &agonist_joint_angle,
                                         float &antagonist_joint_angle);

  // ==========================================================================
  // SYSTEM STATUS & MONITORING
  // ==========================================================================

  /**
   * @brief Verify if the system is ready for movement
   * @return true if the system has linear equations available and offsets calibrated
   */
  bool isSystemReadyForMovement();

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
};

#endif // JOINT_CONTROLLER_H
