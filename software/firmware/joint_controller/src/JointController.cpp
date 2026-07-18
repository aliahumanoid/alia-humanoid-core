/**
 * @file JointController.cpp
 * @brief Implementation of the JointController class
 */

#include <JointController.h>
#include <commands.h>
#include <debug.h>
#include <global.h>
#include <path.h>
#include "pico/util/queue.h"
#include <shared_data.h>
#include <utils.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <math.h>
#include "main_common.h"  // For shared_dof_angles
#include <hot_path.h>

// ============================================================================
// EXTERNAL VARIABLES & HELPER FUNCTIONS
// ============================================================================
// These variables and functions support inter-core communication and
// movement sample logging for debugging and telemetry.

// === EXTERNAL VARIABLES FOR THE NEW COMMUNICATION SYSTEM ===
// Note: emergency_stop_requested, buffer_ready, active_buffer, pending_command_type
// are declared in main_common.h (included above)

// ============================================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================================

// Constructor
JointController::JointController(const JointConfig &cfg, MCP_CAN *can, DirectEncoders *enc) {
  config   = cfg;
  encoders = enc;

  // Allocate memory for arrays
  motors          = new LKM_Motor *[config.motor_count];
  pid_controllers = new PID *[config.motor_count]; // Now one PID per motor
  dof_movement    = new DofMovementData[config.dof_count];
  dof_mappings    = new DofMappingData_t[config.dof_count](); // Initialize to zero

  // Zero-init bilinear (q0,q1) grids (so bl_valid=false on boot, like dof_mappings)
  memset(dof_grids, 0, sizeof(dof_grids));

  // NEW: Allocate memory for linear equations
  linear_equations = new DofLinearEquations[config.dof_count]();

  // Initialize linear equations
  for (int i = 0; i < config.dof_count; i++) {
    linear_equations[i].dof_index        = i;
    linear_equations[i].calculated       = false;
    linear_equations[i].agonist.valid    = false;
    linear_equations[i].antagonist.valid = false;
    linear_equations[i].limits_valid     = false;
    linear_equations[i].map_mode         = MAP_LINEAR; // default: linear; fine map opts in per-DOF
    linear_equations[i].pw_valid         = false;
    linear_equations[i].bl_valid         = false;       // bilinear (2D) grid not usable until captured
    linear_equations[i].q0_nominal       = 0.0f;        // center slice default
  }

  // Allocate and initialize array for offset calibration flags
  motor_offsets_calibrated = new bool[config.dof_count](); // Initialize to false

  // Initialize cascade control (outer loop) parameters
  outer_loop_kp_values     = new float[config.dof_count];
  outer_loop_ki_values     = new float[config.dof_count];
  outer_loop_kd_values     = new float[config.dof_count];
  stiffness_ref_values     = new float[config.dof_count];
  cascade_influence_values = new float[config.dof_count];

  for (int i = 0; i < config.dof_count; i++) {
    outer_loop_kp_values[i]     = DEFAULT_OUTER_LOOP_KP;
    outer_loop_ki_values[i]     = DEFAULT_OUTER_LOOP_KI;
    outer_loop_kd_values[i]     = DEFAULT_OUTER_LOOP_KD;
    stiffness_ref_values[i]     = DEFAULT_STIFFNESS_REF_DEG;
    cascade_influence_values[i] = DEFAULT_CASCADE_INFLUENCE;
  }

  // Allocate and initialize outer loop PID controllers (one per DOF)
  // These handle joint-level position control with proper filtering and anti-windup
  outer_pid_controllers = new PID *[config.dof_count];
  for (int i = 0; i < config.dof_count; i++) {
    // Outer loop PID is initialized with a conservative Ts (10ms)
    // Sampling period is updated at runtime via setOuterLoopSamplingPeriod()
    // Output is delta_theta in degrees, limited to ±MAX_DELTA_THETA
    outer_pid_controllers[i] = new PID(
        DEFAULT_OUTER_LOOP_TS,           // Initial sampling period (updated at runtime)
        DEFAULT_OUTER_LOOP_KP,           // Kp
        DEFAULT_OUTER_LOOP_KI,           // Ki
        DEFAULT_OUTER_LOOP_KD,           // Kd
        DEFAULT_MAX_DELTA_THETA,         // umax (max positive correction)
        -DEFAULT_MAX_DELTA_THETA,        // umin (max negative correction)
        DEFAULT_OUTER_LOOP_TAU           // Derivative filter time constant
    );
  }

  // Set encoder inversion flags based on configuration
  for (int i = 0; i < config.dof_count; i++) {
    if (cfg.dofs[i].drive_type == DRIVE_DIRECT_DRIVE) {
      continue;
    }
    uint8_t encoder_channel = cfg.dofs[i].encoder_channel;
    bool invert             = cfg.dofs[i].encoder_invert;
    // Set inversion flag directly in encoders object
    if (encoder_channel < DIRECT_ENCODER_COUNT) {
      encoders->setEncoderInvert(encoder_channel, invert);
    }
  }

  // Initialize motors and their PID controllers
  for (int i = 0; i < config.motor_count; i++) {
    motors[i] = new LKM_Motor(can, config.motors[i].id, config.motors[i].reduction_gear,
                              config.motors[i].invert);
    motors[i]->setMaxTorque(config.motors[i].max_torque);

    // Initialize PID for each motor
    uint8_t dof_index = config.motors[i].dof_index;
    float Ts = config.dofs[dof_index].motion.sampling_period / 1000000.0f; // Convert to seconds
    pid_controllers[i] = new PID(Ts, config.motors[i].pid.kp, config.motors[i].pid.ki,
                                 config.motors[i].pid.kd, config.motors[i].max_torque,
                                 -config.motors[i].max_torque, config.motors[i].pid.tau);
  }

  // Initialize movement and mapping data for each DOF
  for (int i = 0; i < config.dof_count; i++) {
    // Initialize movement data
    dof_movement[i].state = 0; // idle

    // Initialize mapping data
    dof_mappings[i].size = 0;
    dof_mappings[i].flag = 0;
  }

  // NEW: Initialize encoder reading tracking to detect SPI spikes
  last_valid_angles       = new float[config.dof_count];
  last_read_timestamps    = new uint64_t[config.dof_count];
  spike_counters          = new uint32_t[config.dof_count];
  direct_feedback_angles  = new float[config.dof_count];
  direct_feedback_velocities = new float[config.dof_count];
  direct_feedback_valid   = new bool[config.dof_count];

  // Apply default PID immediately; any values from flash will override them subsequently
  applyDefaultPidTunings(false);

  // Initialize with default values
  for (int i = 0; i < config.dof_count; i++) {
    last_valid_angles[i]    = 0.0f;
    last_read_timestamps[i] = 0;
    spike_counters[i]       = 0;
    direct_feedback_angles[i] = 0.0f;
    direct_feedback_velocities[i] = 0.0f;
    direct_feedback_valid[i] = false;
  }
}

// Destructor
JointController::~JointController() {
  // Free memory for motors
  for (int i = 0; i < config.motor_count; i++) {
    delete motors[i];
  }
  delete[] motors;

  // Free memory for PID controllers (inner loop - one per motor)
  for (int i = 0; i < config.motor_count; i++) {
    delete pid_controllers[i];
  }
  delete[] pid_controllers;

  // Free memory for outer loop PID controllers (one per DOF)
  for (int i = 0; i < config.dof_count; i++) {
    delete outer_pid_controllers[i];
  }
  delete[] outer_pid_controllers;

  // Free memory for movement data
  delete[] dof_movement;

  // Free memory for mapping data
  delete[] dof_mappings;

  // NEW: Free memory for linear equations
  delete[] linear_equations;

  // Free memory for offset calibration flags
  delete[] motor_offsets_calibrated;

  delete[] outer_loop_kp_values;
  delete[] outer_loop_ki_values;
  delete[] outer_loop_kd_values;
  delete[] stiffness_ref_values;
  delete[] cascade_influence_values;

  // NEW: Free memory for encoder reading tracking
  delete[] last_valid_angles;
  delete[] last_read_timestamps;
  delete[] spike_counters;
  delete[] direct_feedback_angles;
  delete[] direct_feedback_velocities;
  delete[] direct_feedback_valid;
}

// ============================================================================
// INITIALIZATION & LIFECYCLE
// ============================================================================

void JointController::applyDefaultPidTunings(bool log_details) {
  if (log_details) {
    LOG_C1_INFO("Applying configured default PID for all motors...");
  }

  for (int i = 0; i < config.motor_count; i++) {
    const float kp = config.motors[i].pid.kp;
    const float ki = config.motors[i].pid.ki;
    const float kd = config.motors[i].pid.kd;
    const float tau = config.motors[i].pid.tau;

    if (pid_controllers && pid_controllers[i] != nullptr) {
      pid_controllers[i]->setTunings(kp, ki, kd, tau);
    }
    // (chatter log removed, v2 String pass 2026-07-06)
  }

  if (log_details) {
    LOG_C1_INFO("Configured default PID applied");
  }
}

// Initialize controller
bool JointController::init() {
  LOG_C1_INFO(String("Initializing joint controller: ") + String(config.name));

  // Initialize all motors
  for (int i = 0; i < config.motor_count; i++) {
    motors[i]->init();
  }

  // Apply configured angular offsets for each DOF ONLY if flash has no valid data
  // If flash has valid calibration data, use that instead of defaults
  if (!encoders->isFlashDataValid()) {
    LOG_C1_INFO("No valid flash calibration - applying default zero_angle_offsets");
    for (int i = 0; i < config.dof_count; i++) {
      if (config.dofs[i].drive_type == DRIVE_DIRECT_DRIVE) {
        continue;
      }
      float zero_offset = config.dofs[i].zero_mapping.zero_angle_offset;
      if (zero_offset != 0.0f) {
        uint8_t encoder_channel = config.dofs[i].encoder_channel;
        LOG_C1_INFO(String("Applying angle offset for DOF ") + String(i) + ": " +
                 String(zero_offset));
        encoders->setJointOffset(encoder_channel, zero_offset, false);
      }
    }
  } else {
    LOG_C1_INFO("Using calibration offsets from flash (skipping defaults)");
  }

  return true;
}

// Returns a specific motor
LKM_Motor *JointController::getMotor(uint8_t motor_index) {
  if (motor_index >= config.motor_count) {
    return nullptr;
  }
  return motors[motor_index];
}

// ============================================================================
// MOTOR & PID CONTROL
// ============================================================================

// ============================================================================
// SAFE SLEEP HELPER (watchdog kick + emergency stop check)
// ============================================================================

/**
 * @brief Sleep with periodic watchdog kicks and emergency stop checks
 *
 * Replaces bare sleep_ms() in blocking motor operations to prevent
 * external watchdog timeout (~1s on Rev B) and allow emergency stop
 * processing during long operations like recalculateMotorOffsets.
 *
 * @param ms Total sleep duration in milliseconds
 * @return true if sleep completed normally, false if interrupted by emergency stop
 */
static bool safeSleepMs(int ms) {
  const int KICK_INTERVAL_MS = 50;  // Kick watchdog every 50ms (well within ~1s timeout)
  int remaining = ms;

  while (remaining > 0) {
    int chunk = (remaining > KICK_INTERVAL_MS) ? KICK_INTERVAL_MS : remaining;
    sleep_ms(chunk);
    remaining -= chunk;

    safety_watchdog_kick();

    if (emergency_stop_requested) {
      LOG_C1_WARN_F("[SAFETY] Emergency stop detected during blocking operation");
      return false;
    }
  }
  return true;
}

// Pretension motors of a specific DOF
bool JointController::pretension(uint8_t dof_index, int torque, int duration_ms) {
  // S2 CARRY choke point (MAJOR amendment): abandon any carried pair before the setTorque drives below
  // (also covers pretensionAll, which calls pretension per DOF). Idempotent; core-affinity guarded.
  abandonCarriedPair();
  if (dof_index >= config.dof_count) {
    return false;
  }

  if (!dofSupportsPretension(dof_index)) {
    LOG_C1_WARN_F("Pretension not supported for DOF %d (%s)", dof_index,
                  config.dofs[dof_index].name);
    return false;
  }

  // If parameters are not specified, use configured ones
  if (torque == 0) {
    torque = config.dofs[dof_index].zero_mapping.pretension_torque;
  }

  if (duration_ms == 0) {
    duration_ms = config.dofs[dof_index].zero_mapping.pretension_timeout;
  }

  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      LKM_Motor *motor = motors[i];

      // Assign opposite torques based on motor role
      if (config.motors[i].is_agonist) {
        motor->setTorque(torque);
      } else {
        motor->setTorque(-torque);
      }
    }
  }
  LOG_C1_INFO("Pretensioning parameters: index=" + String(dof_index) + " torque=" + String(torque) +
           " duration=" + String(duration_ms));

  // Wait for specified duration (with watchdog kick + e-stop check)
  if (!safeSleepMs(duration_ms)) {
    stopDofMotors(dof_index);
    return false;
  }

  // Stop all motors of the DOF
  stopDofMotors(dof_index);

  return true;
}

// Pretension all DOFs of the joint
bool JointController::pretensionAll() {
  bool result = true;
  bool any_supported = false;

  // Pretension each DOF using configured parameters
  for (int i = 0; i < config.dof_count; i++) {
    if (!dofSupportsPretension(i)) {
      LOG_C1_INFO("Skipping pretension for DOF " + String(i) + " (" +
                  String(config.dofs[i].name) + "): unsupported");
      continue;
    }
    any_supported = true;
    if (!pretension(i, 0, 0)) { // Use configured parameters (0 = use config)
      result = false;
    }
  }

  if (!any_supported) {
    LOG_C1_WARN_F("PretensionAll requested, but no DOF supports pretension");
    return false;
  }

  LOG_C1_INFO("Pretensioning all DOFs with configured parameters");

  // Wait at the end for all DOFs using first DOF timeout (or 100ms as fallback)
  int final_timeout = (config.dof_count > 0) ? config.dofs[0].zero_mapping.pretension_timeout : 100;

  // Wait at the end for all DOFs (with watchdog kick + e-stop check)
  safeSleepMs(final_timeout);

  // Stop all motors
  stopAllMotors();

  return result;
}

// Release motors of a specific DOF (inverse torque compared to pretensioning)
bool JointController::release(uint8_t dof_index, int torque, int duration_ms) {
  // S2 CARRY choke point (MAJOR amendment): abandon any carried pair before the setTorque drives below
  // (also covers releaseAll, which calls release per DOF). Idempotent; core-affinity guarded.
  abandonCarriedPair();
  if (dof_index >= config.dof_count) {
    return false;
  }

  if (!dofSupportsPretension(dof_index)) {
    LOG_C1_WARN_F("Release not supported for DOF %d (%s)", dof_index,
                  config.dofs[dof_index].name);
    return false;
  }

  // If parameters are not specified, use configured ones with inverted sign
  if (torque == 0) {
    // Release torque is the opposite of pretension torque
    torque = -config.dofs[dof_index].zero_mapping.pretension_torque;
  }

  if (duration_ms == 0) {
    duration_ms = config.dofs[dof_index].zero_mapping.pretension_timeout;
  }

  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      LKM_Motor *motor = motors[i];

      // Assign opposite torques based on motor role
      if (config.motors[i].is_agonist) {
        motor->setTorque(torque);
      } else {
        motor->setTorque(-torque);
      }
    }
  }
  LOG_C1_INFO("Releasing DOF: index=" + String(dof_index) + " torque=" + String(torque) +
           " duration=" + String(duration_ms));

  // Wait for specified duration (with watchdog kick + e-stop check)
  if (!safeSleepMs(duration_ms)) {
    stopDofMotors(dof_index);
    return false;
  }

  // Stop all motors of the DOF
  stopDofMotors(dof_index);

  return true;
}

// Release all DOFs of the joint (inverse torque compared to pretensioning)
bool JointController::releaseAll() {
  bool result = true;
  bool any_supported = false;

  // Release each DOF using configured parameters
  for (int i = 0; i < config.dof_count; i++) {
    if (!dofSupportsPretension(i)) {
      LOG_C1_INFO("Skipping release for DOF " + String(i) + " (" +
                  String(config.dofs[i].name) + "): unsupported");
      continue;
    }
    any_supported = true;
    if (!release(i, 0, 0)) { // Use configured parameters (0 = use config)
      result = false;
    }
  }

  if (!any_supported) {
    LOG_C1_WARN_F("ReleaseAll requested, but no DOF supports release");
    return false;
  }

  LOG_C1_INFO("Releasing all DOFs with configured parameters");

  // Wait at the end for all DOFs using first DOF timeout (or 100ms as fallback)
  int final_timeout = (config.dof_count > 0) ? config.dofs[0].zero_mapping.pretension_timeout : 100;

  // Wait at the end for all DOFs (with watchdog kick + e-stop check)
  safeSleepMs(final_timeout);

  // Stop all motors
  stopAllMotors();

  return result;
}

// Set current position as zero for a DOF without moving the joint
// NOTE: Called from Core1 - minimize Serial usage to avoid deadlocks!
bool JointController::setZeroCurrentPos(uint8_t dof_index) {
  if (dof_index >= config.dof_count) {
    return false;
  }

  if (config.dofs[dof_index].drive_type == DRIVE_DIRECT_DRIVE) {
    LKM_Motor *direct_motor = nullptr;
    int direct_motor_idx = -1;
    for (int i = 0; i < config.motor_count; i++) {
      if (config.motors[i].dof_index == dof_index &&
          config.motors[i].role == MOTOR_ROLE_DIRECT) {
        direct_motor = motors[i];
        direct_motor_idx = i;
        break;
      }
    }

    if (direct_motor == nullptr) {
      LOG_C1_ERROR_F("Set Reference failed: no direct-drive motor for DOF %d", dof_index);
      return false;
    }

    float reference_angle = config.dofs[dof_index].zero_mapping.zero_angle_offset;
    LKM_Motor::MultiAngleData current_angle = direct_motor->getSingleAngleSync();
    if (isnan(current_angle.angle)) {
      diag_note_motor_timeout(dof_index,
                              direct_motor_idx >= 0 ? static_cast<uint8_t>(direct_motor_idx) : 0xFF);
      LOG_C1_ERROR_F("Set Reference failed: no single-turn angle from direct-drive motor DOF %d",
                     dof_index);
      return false;
    }

    _saved_offsets[dof_index].agonist_offset = current_angle.angle - reference_angle;
    _saved_offsets[dof_index].antagonist_offset = 0.0f;
    _saved_offsets[dof_index].joint_angle_at_calib = reference_angle;
    _saved_offsets[dof_index].valid = true;
    _pending_offsets_save = true;
    motor_offsets_calibrated[dof_index] = true;

    LOG_C1_INFO("Set Reference: DOF " + String(dof_index) +
                " direct-drive offset=" + String(_saved_offsets[dof_index].agonist_offset, 2) +
                " target=" + String(reference_angle, 2) + "°");
    return true;
  }

  // Get the encoder channel for this DOF
  uint8_t encoder_channel = config.dofs[dof_index].encoder_channel;

  // Get the configured angular offset (this is what the current position should become)
  float zero_offset = config.dofs[dof_index].zero_mapping.zero_angle_offset;

  // Request encoder reset with target angle - will be executed by Core0 on next update cycle
  // This is thread-safe and avoids SPI conflicts between cores
  encoders->requestReset(encoder_channel, zero_offset);

  return true;
}

// Stop all motors
// Can be safely called from both Core0 and Core1 (noInterrupts() protects SPI1 access)
void JointController::stopAllMotors() {
  // S2 CARRY choke point (MAJOR amendment): abandon any carried pair BEFORE touching the motor bus,
  // so its stale _fire_pending / buffered replies can't corrupt these motorStop transactions. This
  // covers the e-stop, torqueSweepStop, free-capture EXIT, and pretension/release tail paths that all
  // funnel through stopAllMotors. Core-affinity guarded inside (core0 startup calls are a no-op).
  abandonCarriedPair();
  for (int i = 0; i < config.motor_count; i++) {
    if (motors[i] != nullptr) {
      motors[i]->motorStop();
    }
  }
  LOG_C1_INFO_F("Stopping all motors in JointController");
}

// Stop motors for a specific DOF
void JointController::stopDofMotors(uint8_t dof_index) {
  if (dof_index >= config.dof_count) {
    return;
  }

  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      motors[i]->motorStop();
    }
  }
}

// ============================================================================
// POSITION & ANGLE READING
// ============================================================================

// Read current angle of a DOF from shared memory (thread-safe, no SPI access)
// Validation and spike detection is handled by DirectEncoders in Core0
float JointController::getValidatedAngle(uint8_t dof_index, bool &is_valid) {
  if (dof_index >= config.dof_count) {
    is_valid = false;
    return 0.0f;
  }

  if (config.dofs[dof_index].drive_type == DRIVE_DIRECT_DRIVE) {
    is_valid = direct_feedback_valid[dof_index];
    return direct_feedback_angles[dof_index];
  }

  // Read from shared memory (updated by Core0)
  // This is thread-safe and does NOT access SPI
  is_valid = shared_dof_angles.valid[dof_index];
  return shared_dof_angles.angles[dof_index];
}

void JointController::updateDirectDriveFeedback(uint8_t dof_index, float angle_deg,
                                                float velocity_deg_s, bool valid) {
  if (dof_index >= config.dof_count ||
      config.dofs[dof_index].drive_type != DRIVE_DIRECT_DRIVE) {
    return;
  }
  direct_feedback_angles[dof_index] = angle_deg;
  direct_feedback_velocities[dof_index] = velocity_deg_s;
  direct_feedback_valid[dof_index] = valid;
}

bool JointController::getDirectDriveFeedback(uint8_t dof_index, float &angle_deg,
                                             float &velocity_deg_s) const {
  if (dof_index >= config.dof_count ||
      config.dofs[dof_index].drive_type != DRIVE_DIRECT_DRIVE ||
      !direct_feedback_valid[dof_index]) {
    angle_deg = 0.0f;
    velocity_deg_s = 0.0f;
    return false;
  }

  angle_deg = direct_feedback_angles[dof_index];
  velocity_deg_s = direct_feedback_velocities[dof_index];
  return true;
}

// Read current angle of a DOF (alias for getValidatedAngle)
float JointController::getCurrentAngle(uint8_t dof_index, bool &is_valid) {
  return getValidatedAngle(dof_index, is_valid);
}

// Check if an angle is within a DOF's limits
bool HOT_FUNC(JointController::isAngleInLimits)(uint8_t dof_index, float angle) {
  if (dof_index >= config.dof_count) {
    return false;
  }

  return (angle >= config.dofs[dof_index].limits.min_angle &&
          angle <= config.dofs[dof_index].limits.max_angle);
}

// ============================================================================
// SAFETY & VALIDATION
// ============================================================================

// Check if an angle is within safety limits derived from equations
// Small tolerance to account for encoder noise and floating point precision
static const float LIMIT_TOLERANCE = 0.5f;  // Allow 0.5° beyond limits
static const float CONSERVATIVE_MARGIN = 2.0f;

bool HOT_FUNC(JointController::getMappingSafeRange)(uint8_t dof_index, float &min_safe, float &max_safe) {
  if (dof_index >= config.dof_count) {
    return false;
  }

  // FINE-CAPTURE WIDE-BAND EXCEPTION (2026-07-10, safe-range RATCHET fix): while a fine
  // capture is active for THIS DOF, clamp against the wide physical band instead of the
  // committed fine span. Without this a fine map can never be recaptured WIDER than the
  // current one: commitFineCapture narrows joint_safe to the captured span and the NEXT
  // capture's motion (SET_IMPEDANCE clamp + MAPPING_LIMIT) rides this same range — every
  // settle-undershoot ratchets the span inward (bench history: +17.8 -> +9.85). The
  // FF-only capture sweep at this band is the same trust level as the coarse auto-map,
  // which sweeps the identical physical-minus-margin band routinely.
  if (_fine_capture_active && dof_index == _fine_capture_dof) {
    min_safe = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN - LIMIT_TOLERANCE;
    max_safe = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN + LIMIT_TOLERANCE;
    return true;
  }

  if (hasValidEquations(dof_index)) {
    min_safe = linear_equations[dof_index].joint_safe_min - LIMIT_TOLERANCE;
    max_safe = linear_equations[dof_index].joint_safe_max + LIMIT_TOLERANCE;
    return true;
  }

  min_safe = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN - LIMIT_TOLERANCE;
  max_safe = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN + LIMIT_TOLERANCE;
  return true;
}

bool HOT_FUNC(JointController::isAngleInMappingLimits)(uint8_t dof_index, float angle) {
  if (dof_index >= config.dof_count) {
    return false;
  }

  float min_angle = 0.0f;
  float max_angle = 0.0f;
  if (!getMappingSafeRange(dof_index, min_angle, max_angle)) {
    return false;
  }

  bool in_limits = (angle >= min_angle && angle <= max_angle);

  if (!in_limits) {
    // Rate-limited 250ms (deliberate behavior change, v2 String pass 2026-07-06): this WARN
    // fires EVERY moving outer cycle while grazing/recovering outside the safe range — it was
    // an unthrottled per-cycle heap log inside a HOT_FUNC.
    static uint32_t last_mapping_limit_warn_ms[MAX_DOFS] = {0};
    if (millis() - last_mapping_limit_warn_ms[dof_index] > 250) {
      last_mapping_limit_warn_ms[dof_index] = millis();
      const bool has_equations = hasValidEquations(dof_index);
      char f1[48], f2[48], f3[48];
      LOG_C1_WARN_F("Angle %s° for DOF %d%s%s°, %s°]%s",
                    c1f(f1, angle, 2), dof_index,
                    has_equations ? " outside safe limits [" : " outside conservative limits [",
                    c1f(f2, min_angle, 2), c1f(f3, max_angle, 2),
                    has_equations ? " (from equations)" : " (equations unavailable)");
    }
  }

  return in_limits;
}

bool JointController::canDirectDriveRecoverTowardSafeRange(uint8_t dof_index, float current_angle,
                                                           float target_angle) {
  if (!isDirectDriveDof(dof_index)) {
    return false;
  }

  if (!isAngleInLimits(dof_index, current_angle)) {
    return false;
  }

  float min_safe = 0.0f;
  float max_safe = 0.0f;
  if (!getMappingSafeRange(dof_index, min_safe, max_safe)) {
    return false;
  }

  if (current_angle < min_safe) {
    return target_angle > current_angle;
  }

  if (current_angle > max_safe) {
    return target_angle < current_angle;
  }

  return false;
}

// Check if a DOF's motors are within their mapping ranges
bool HOT_FUNC(JointController::checkMotorsInRange)(uint8_t dof_index, char *violation_message,
                                                   size_t violation_msg_size) {
  if (dof_index >= config.dof_count) {
    snprintf(violation_message, violation_msg_size, "DOF index out of range");
    return false;
  }

  if (config.dofs[dof_index].drive_type == DRIVE_DIRECT_DRIVE) {
    return true;
  }

  if (!hasValidEquations(dof_index)) {
    // If equations are invalid, the entire movement should not be authorized
    snprintf(violation_message, violation_msg_size, "Equation limits unavailable");
    return false;
  }

  float agonist_min    = linear_equations[dof_index].agonist_safe_min;
  float agonist_max    = linear_equations[dof_index].agonist_safe_max;
  float antagonist_min = linear_equations[dof_index].antagonist_safe_min;
  float antagonist_max = linear_equations[dof_index].antagonist_safe_max;

  // === OPTIMIZATION: Use cached motor angles instead of redundant CAN reads ===
  // The control loop already reads motor angles every cycle and caches them in
  // cached_motor_angles. Using the cache eliminates ~2ms delay per motor that
  // was caused by blocking CAN reads, reducing safety check overhead from
  // ~4000µs to ~50µs.

  // Persistence counter for "cannot trust cached motor angles" conditions.
  // A MOMENTARY stale/invalid cache (normal during a CAN retry, or in IDLE) must NOT
  // fault — historically this fell open (returned in-range). But a PERSISTENTLY
  // stale/garbage cache means we have NO real motor feedback, so we cannot certify
  // the tendons are intact; once it persists we must treat the DOF as out-of-range
  // so the caller's MOTOR_RANGE handling engages instead of silently passing.
  // Core1-only state (checkMotorsInRange runs only on Core1, same core as the cache).
  static uint8_t cache_untrusted_streak[MAX_DOFS] = {0};
  // checkMotorsInRange is invoked at most ~every 40 ms (HOLDING safety check, every 10
  // inner cycles at the 250Hz/4ms default), so a streak of 5 ≈ 0.2 s of continuously-
  // untrusted feedback before we fault — well past any single CAN retry, while still well
  // inside the downstream MOTOR_RANGE escalation persistence so a genuine loss of motor
  // feedback ultimately latches power off.
  const uint8_t CACHE_UNTRUSTED_FAULT_CYCLES = 5;

  // Check if cache is valid for this DOF
  bool cache_untrusted = false;
  const char *untrusted_reason = nullptr;
  if (!cached_motor_angles.valid[dof_index]) {
    // Cache not yet populated - normal on first cycles after startup, but if it
    // NEVER populates we have no motor feedback at all.
    cache_untrusted = true;
    untrusted_reason = "cache not populated";
  } else if ((millis() - cached_motor_angles.last_update_ms) > 100) {
    // Cache is stale - control loop may not be feeding it.
    cache_untrusted = true;
    untrusted_reason = "cache stale";
  } else if (abs(cached_motor_angles.agonist[dof_index]) > 10000.0f ||
             abs(cached_motor_angles.antagonist[dof_index]) > 10000.0f) {
    // Garbage values.
    cache_untrusted = true;
    untrusted_reason = "cache garbage";
  }

  if (cache_untrusted) {
    if (cache_untrusted_streak[dof_index] < 0xFF) {
      cache_untrusted_streak[dof_index]++;
    }
    static uint32_t last_untrusted_warn = 0;
    if (millis() - last_untrusted_warn > 1000) {
      LOG_C1_WARN_F("[Safety] Motor angle cache untrusted (%s) for DOF %d streak=%d",
                    untrusted_reason, dof_index, cache_untrusted_streak[dof_index]);
      last_untrusted_warn = millis();
    }
    if (cache_untrusted_streak[dof_index] >= CACHE_UNTRUSTED_FAULT_CYCLES) {
      // Persistently no trustworthy motor feedback — fail the motor check (out-of-range).
      snprintf(violation_message, violation_msg_size,
               "MOTOR FEEDBACK LOST (%s) DOF %d — cannot verify tendon integrity",
               untrusted_reason, dof_index);
      return false;
    }
    // Momentary: preserve the historical fail-open so a single CAN retry doesn't fault.
    return true;
  }

  // Cache trusted this cycle — clear the streak.
  cache_untrusted_streak[dof_index] = 0;

  // Get cached angles
  float agonist_angle = cached_motor_angles.agonist[dof_index];
  float antagonist_angle = cached_motor_angles.antagonist[dof_index];

  // Check agonist motor limits
  if (agonist_angle < agonist_min || agonist_angle > agonist_max) {
    char f1[48], f2[48], f3[48];
    snprintf(violation_message, violation_msg_size,
             "!!! POSSIBLE TENDON BREAKAGE !!! AGONIST motor DOF %d out of range: %s deg "
             "[safe range: %s / %s]",
             dof_index, c1f(f1, agonist_angle, 1), c1f(f2, agonist_min, 1),
             c1f(f3, agonist_max, 1));
    return false;
  }
  
  // Check antagonist motor limits
  if (antagonist_angle < antagonist_min || antagonist_angle > antagonist_max) {
    char f1[48], f2[48], f3[48];
    snprintf(violation_message, violation_msg_size,
             "!!! POSSIBLE TENDON BREAKAGE !!! ANTAGONIST motor DOF %d out of range: %s deg "
             "[safe range: %s / %s]",
             dof_index, c1f(f1, antagonist_angle, 1), c1f(f2, antagonist_min, 1),
             c1f(f3, antagonist_max, 1));
    return false;
  }

  return true; // All motors are within limits
}

bool HOT_FUNC(JointController::checkSafetyForDof)(uint8_t dof_index, float current_angle,
                                        char *violation_message, size_t violation_msg_size,
                                        bool check_motors,
                                        SafetyViolationType *violation_type) {
  // Heap-free (v2 pass): a single byte store on the happy path — the old String& cleared
  // an Arduino String here = malloc(1)+free under the cross-core mutex every check.
  if (violation_msg_size) violation_message[0] = '\0';
  if (violation_type != nullptr) {
    *violation_type = SAFETY_VIOLATION_LIMIT;
  }

  if (dof_index >= config.dof_count) {
    snprintf(violation_message, violation_msg_size, "DOF index out of range");
    return false;
  }

  if (!isAngleInLimits(dof_index, current_angle)) {
    const DofConfig &dof_config = config.dofs[dof_index];
    if (violation_type != nullptr) {
      *violation_type = SAFETY_VIOLATION_LIMIT;
    }
    char f1[48], f2[48], f3[48];
    snprintf(violation_message, violation_msg_size,
             "JOINT LIMIT VIOLATED - DOF %d: angle=%s deg [physical range: %s / %s]",
             dof_index, c1f(f1, current_angle, 2), c1f(f2, dof_config.limits.min_angle, 1),
             c1f(f3, dof_config.limits.max_angle, 1));
    return false;
  }

  if (!isAngleInMappingLimits(dof_index, current_angle)) {
    float min_safe;
    float max_safe;
    if (violation_type != nullptr) {
      *violation_type = SAFETY_VIOLATION_MAPPING_LIMIT;
    }

    if (hasValidEquations(dof_index)) {
      min_safe = linear_equations[dof_index].joint_safe_min;
      max_safe = linear_equations[dof_index].joint_safe_max;
    } else {
      min_safe = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN;
      max_safe = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN;
    }

    char f1[48], f2[48], f3[48];
    snprintf(violation_message, violation_msg_size,
             "MAPPING LIMIT VIOLATED - DOF %d: angle=%s deg [safe range: %s / %s]",
             dof_index, c1f(f1, current_angle, 2), c1f(f2, min_safe, 1), c1f(f3, max_safe, 1));
    return false;
  }

  if (check_motors && config.dofs[dof_index].drive_type != DRIVE_DIRECT_DRIVE) {
    if (!checkMotorsInRange(dof_index, violation_message, violation_msg_size)) {
      if (violation_type != nullptr) {
        *violation_type = SAFETY_VIOLATION_MOTOR_RANGE;
      }
      return false;
    }
  }

  return true;
}

bool JointController::checkSafetyForAllDofs(char *violation_message,
                                            size_t violation_msg_size, bool check_motors) {
  if (violation_msg_size) violation_message[0] = '\0';

  for (int dof = 0; dof < config.dof_count; dof++) {
    // Use shared_dof_angles (updated by Core0)
    if (!shared_dof_angles.valid[dof]) {
      snprintf(violation_message, violation_msg_size, "Invalid encoder reading for DOF %d", dof);
      return false;
    }
    float current_angle = shared_dof_angles.angles[dof];

    if (!checkSafetyForDof(dof, current_angle, violation_message, violation_msg_size,
                           check_motors)) {
      return false;
    }
  }

  return true;
}

// ============================================================================
// PID CONFIGURATION
// ============================================================================

// Implementation of recalculateMotorOffsets
bool JointController::recalculateMotorOffsets(uint8_t dof_index, float pretension_torque,
                                              int pretension_duration_ms) {
  // S2 CARRY choke point (MAJOR amendment): abandon any carried pair before this sequence drives the
  // motor bus (pretension + offset probing). Idempotent; core-affinity guarded.
  abandonCarriedPair();
  if (dof_index >= config.dof_count) {
    LOG_C1_ERROR_F("DOF index out of range in recalculateMotorOffsets");
    return false;
  }

  if (!dofSupportsRecalcOffset(dof_index)) {
    LOG_C1_WARN_F("Recalc offset not supported for DOF %d (%s)", dof_index,
                  config.dofs[dof_index].name);
    return false;
  }
  // (chatter log removed, v2 String pass 2026-07-06)

  // Identify motors associated with this DOF
  LKM_Motor *agonist_motor    = nullptr;
  LKM_Motor *antagonist_motor = nullptr;
  int agonist_motor_id        = -1;
  int antagonist_motor_id     = -1;

  // Find agonist and antagonist motors for this DOF using the is_agonist property
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      LOG_C1_INFO("FOUND motor for DOF " + String(dof_index) + ": " + String(config.motors[i].name) +
               " (ID=" + String(config.motors[i].id) +
               ", Agonist=" + String(config.motors[i].is_agonist ? "YES" : "NO") + ")");

      if (config.motors[i].is_agonist) {
        agonist_motor    = motors[i];
        agonist_motor_id = config.motors[i].id;
        LOG_C1_INFO("-> Assigned as AGONIST (CAN ID " + String(agonist_motor_id) + ")");
      } else {
        antagonist_motor    = motors[i];
        antagonist_motor_id = config.motors[i].id;
        LOG_C1_INFO("-> Assigned as ANTAGONIST (CAN ID " + String(antagonist_motor_id) + ")");
      }
    }
  }

  // (chatter log removed, v2 String pass 2026-07-06)

  // Verify that both motors are available
  if (agonist_motor == nullptr || antagonist_motor == nullptr) {
    LOG_C1_ERROR_F("Both motors are not available for this DOF");
    return false;
  }

  // Verify that linear equations are available (only supported method)
  if (!linear_equations[dof_index].calculated || !linear_equations[dof_index].agonist.valid ||
      !linear_equations[dof_index].antagonist.valid || !linear_equations[dof_index].limits_valid) {
    LOG_C1_ERROR_F("Unable to recalculate offsets - linear equations not available for DOF %d", dof_index);
    LOG_C1_WARN_F("Required: Perform auto-mapping to compute linear equations first");
    LOG_C1_WARN_F("Alternatively: Load linear equations from flash with CMD:LOAD_LINEAR_EQUATIONS");
    stopDofMotors(dof_index);
    return false;
  }

  LOG_C1_INFO("Linear equations available for DOF " + String(dof_index) + " - offset recalculation enabled");

  // Apply pretension to eliminate tendon slack
  // Uses progressive tensioning: if tendons are slack, gradually increase torque
  LOG_C1_INFO("Applying pretension for DOF " + String(dof_index));

  // Check if inversion logic is requested
  bool invert_logic = config.dofs[dof_index].zero_mapping.auto_mapping_invert_direction;
  if (invert_logic) {
      LOG_C1_INFO("Using INVERTED pretension logic for DOF " + String(dof_index));
  }

  const float MIN_TENSION_DISPLACEMENT = 0.1f;  // Minimum displacement indicating tension (degrees)
  const float MAX_TENSION_DISPLACEMENT = 20.0f; // Maximum displacement indicating tendons too loose (degrees)
  const int MAX_TENSION_ATTEMPTS = 3;           // Maximum attempts to achieve proper tension
  const float TORQUE_INCREMENT_FACTOR = 1.5f;   // Multiply torque by this factor each attempt
  const float MAX_TORQUE_MULTIPLIER = 3.0f;     // Maximum torque = initial * this (safety limit)
  
  float current_torque = pretension_torque;
  float max_allowed_torque = pretension_torque * MAX_TORQUE_MULTIPLIER;
  bool tension_ok = false;
  int tension_attempt = 0;
  
  float agonist_displacement = 0.0f;
  float antagonist_displacement = 0.0f;
  
  // Progressive tensioning loop
  while (!tension_ok && tension_attempt < MAX_TENSION_ATTEMPTS) {
    tension_attempt++;
    
    LOG_C1_INFO("Tension attempt " + String(tension_attempt) + "/" + String(MAX_TENSION_ATTEMPTS) + 
             " with torque=" + String((int)current_torque));
    
    // First read current angles without torque (or with previous torque on retry)
    float initial_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
    float initial_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

    // Apply opposite torque to tension the system
    float effective_agonist_torque = invert_logic ? current_torque : -current_torque;
    float effective_antagonist_torque = invert_logic ? -current_torque : current_torque;
    
    agonist_motor->setTorque(effective_agonist_torque);
    antagonist_motor->setTorque(effective_antagonist_torque);

    // Wait for system to react (longer on retries to allow slack to be taken up)
    if (!safeSleepMs(100 + (tension_attempt - 1) * 50)) {
      stopDofMotors(dof_index);
      return false;
    }

    // Read angles under tension
    float tensioned_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
    float tensioned_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

    // Calculate displacement under tension
    agonist_displacement    = fabs(tensioned_agonist_angle - initial_agonist_angle);
    antagonist_displacement = fabs(tensioned_antagonist_angle - initial_antagonist_angle);

    // Check if both motors show proper tension
    bool agonist_tensioned = (agonist_displacement >= MIN_TENSION_DISPLACEMENT &&
                              agonist_displacement <= MAX_TENSION_DISPLACEMENT);
    bool antagonist_tensioned = (antagonist_displacement >= MIN_TENSION_DISPLACEMENT &&
                                 antagonist_displacement <= MAX_TENSION_DISPLACEMENT);
    
    tension_ok = agonist_tensioned && antagonist_tensioned;
    
    // Log detailed status
    LOG_C1_DEBUG(String("Agonist displacement: ") + String(agonist_displacement, 2) + "°" +
              (agonist_displacement < MIN_TENSION_DISPLACEMENT ? " (too stiff)" :
               (agonist_displacement > MAX_TENSION_DISPLACEMENT ? " (too loose)" : " (OK)")));
    LOG_C1_DEBUG(String("Antagonist displacement: ") + String(antagonist_displacement, 2) + "°" +
              (antagonist_displacement < MIN_TENSION_DISPLACEMENT ? " (too stiff)" :
               (antagonist_displacement > MAX_TENSION_DISPLACEMENT ? " (too loose)" : " (OK)")));
    
    if (!tension_ok && tension_attempt < MAX_TENSION_ATTEMPTS) {
      // Check if tendons are too loose (not too stiff - can't fix stiff)
      bool agonist_loose = agonist_displacement > MAX_TENSION_DISPLACEMENT;
      bool antagonist_loose = antagonist_displacement > MAX_TENSION_DISPLACEMENT;
      
      if (agonist_loose || antagonist_loose) {
        // Increase torque for next attempt
        float new_torque = current_torque * TORQUE_INCREMENT_FACTOR;
        if (new_torque > max_allowed_torque) {
          new_torque = max_allowed_torque;
          LOG_C1_WARN_F("Torque limited to max allowed: %d", (int)max_allowed_torque);
        }
        
        if (new_torque > current_torque) {
          current_torque = new_torque;
          LOG_C1_INFO("Tendons loose - increasing torque to " + String((int)current_torque) + " for next attempt");
        } else {
          LOG_C1_WARN_F("Already at max torque, cannot increase further");
          break;  // No point in retrying at same torque
        }
      } else {
        // Both too stiff - increasing torque won't help
        LOG_C1_WARN_F("Tendons appear too stiff - cannot improve with more torque");
        break;
      }
    }
  }
  
  // Final verdict on tension
  if (!tension_ok) {
    bool is_too_loose = (agonist_displacement > MAX_TENSION_DISPLACEMENT || 
                         antagonist_displacement > MAX_TENSION_DISPLACEMENT);
    
    if (is_too_loose) {
      LOG_C1_ERROR_F("Tendon tension FAILED for DOF %d after %d attempts", dof_index,
                     tension_attempt);
      LOG_C1_ERROR_F("Tendons may be disconnected or severely slack");
      LOG_C1_ERROR_F("Check tendon connections before retrying");
      stopDofMotors(dof_index);
      return false;  // FAIL - tendons too loose even with max torque
    } else {
      // Too stiff - this is unusual but not necessarily fatal
      LOG_C1_WARN_F("Tendon tension suboptimal for DOF %d (too stiff)", dof_index);
      LOG_C1_WARN_F("Continuing calibration; results may be suboptimal");
      // Don't fail - stiff tendons might just mean the system is already tensioned
    }
  } else {
    LOG_C1_INFO("Tendon tension achieved after " + String(tension_attempt) + " attempt(s)");
  }
  
  // Store the effective torque for the rest of the calibration
  float effective_agonist_torque = invert_logic ? current_torque : -current_torque;
  float effective_antagonist_torque = invert_logic ? -current_torque : current_torque;

  // Continue with full pretension for the remaining duration
  // Tensioning loop may have taken: ~100-200ms per attempt
  // Ensure we hold pretension for at least the specified duration
  int elapsed_tension_ms = tension_attempt * 150;  // Approximate time spent in tension loop
  int remaining_pretension_ms = pretension_duration_ms - elapsed_tension_ms;
  if (remaining_pretension_ms > 0) {
    if (!safeSleepMs(remaining_pretension_ms)) {
      stopDofMotors(dof_index);
      return false;
    }
  }

  // FIX: Maintain FULL pretension torque during measurement.
  // Previously we reduced to (pretension_torque / 2), but this caused heavy joints
  // to sag due to gravity, failing the stability check.
  agonist_motor->setTorque(effective_agonist_torque);
  antagonist_motor->setTorque(effective_antagonist_torque);

  // === STICTION-BREAK DITHER (2026-07-08) ===
  // The convergence check below only waits for motion to STOP — but under tendon
  // friction "stopped" is a STICTION-FROZEN position, hysteresis- and approach-
  // dependent, NOT the true torque equilibrium. Recalc then anchors the encoder
  // offsets to that stuck point, so every boot's offsets (and every downstream
  // residual/ema) vary with the approach history — masquerading as thermal drift.
  // Mirror the fine-capture dither fix: perturb the co-contraction balance a few
  // times so the tendon slips to equilibrium, then let it settle before sampling.
  // The [RECALC_DITHER] delta line reports how much stiction was broken.
#ifndef RECALC_STICTION_DITHER
#define RECALC_STICTION_DITHER 1
#endif
#if RECALC_STICTION_DITHER
  {
    const float RECALC_DITHER_FRACTION = 0.30f;  // co-contraction imbalance vs pretension
    const int   RECALC_DITHER_CYCLES   = 2;      // back-and-forth wiggles
    const int   RECALC_DITHER_DWELL_MS = 150;    // dwell at each extreme (let it slip)
    const int   RECALC_DITHER_SETTLE_MS = 200;   // settle at nominal before converge
    const float mag_cap = max_allowed_torque;    // never exceed the pretension safety cap
    const float d = RECALC_DITHER_FRACTION;
    // Scaling preserves the antagonistic sign structure; clamp only the raised side.
    float ag_hi = fmaxf(-mag_cap, fminf(mag_cap, effective_agonist_torque    * (1.0f + d)));
    float an_hi = fmaxf(-mag_cap, fminf(mag_cap, effective_antagonist_torque * (1.0f + d)));
    float ag_lo = effective_agonist_torque    * (1.0f - d);
    float an_lo = effective_antagonist_torque * (1.0f - d);
    float pre_dither_joint = shared_dof_angles.valid[dof_index]
                               ? shared_dof_angles.angles[dof_index] : NAN;
    bool dither_ok = true;
    for (int i = 0; i < RECALC_DITHER_CYCLES && dither_ok; i++) {
      agonist_motor->setTorque(ag_hi);  antagonist_motor->setTorque(an_lo);
      dither_ok = safeSleepMs(RECALC_DITHER_DWELL_MS);
      if (dither_ok) { agonist_motor->setTorque(ag_lo);  antagonist_motor->setTorque(an_hi);
        dither_ok = safeSleepMs(RECALC_DITHER_DWELL_MS); }
    }
    // Restore nominal pretension and let it slip to equilibrium before sampling.
    agonist_motor->setTorque(effective_agonist_torque);
    antagonist_motor->setTorque(effective_antagonist_torque);
    if (!dither_ok || !safeSleepMs(RECALC_DITHER_SETTLE_MS)) {
      stopDofMotors(dof_index);
      return false;
    }
    float post_dither_joint = shared_dof_angles.valid[dof_index]
                                ? shared_dof_angles.angles[dof_index] : NAN;
    {
      char f1[48], f2[48], f3[48];
      LOG_C1_INFO_F("[RECALC_DITHER] DOF %d stiction break: joint %s -> %s (delta %s deg)",
                    dof_index, c1f(f1, pre_dither_joint, 2), c1f(f2, post_dither_joint, 2),
                    c1f(f3, post_dither_joint - pre_dither_joint, 2));
    }
  }
#endif

  // === CONVERGENCE-BASED STABILITY CHECK ===
  // Instead of fixed delay, wait until motors STOP MOVING (or timeout).
  // This handles slack tendon recovery where motors need time to "wind up" slack.
  // 
  // Algorithm:
  // 1. Poll motor positions every POLL_INTERVAL_MS
  // 2. If movement < STABILITY_THRESHOLD for STABLE_READINGS_REQUIRED consecutive readings → stable
  // 3. If MAX_CONVERGENCE_TIME_MS exceeded → fail
  
  const float STABILITY_THRESHOLD = 2.0f;      // Max degrees of movement to consider "stable"
  const int POLL_INTERVAL_MS = 100;            // How often to check position
  const int STABLE_READINGS_REQUIRED = 3;      // Need N consecutive stable readings
  const int MAX_CONVERGENCE_TIME_MS = 5000;    // Maximum wait time (5 seconds for slack recovery)
  const int MIN_INITIAL_WAIT_MS = 200;         // Minimum initial wait before first check
  
  if (!safeSleepMs(MIN_INITIAL_WAIT_MS)) {
    stopDofMotors(dof_index);
    return false;
  }
  
  float prev_agonist = agonist_motor->getMultiAngleSync(false).angle;
  float prev_antagonist = antagonist_motor->getMultiAngleSync(false).angle;
  
  int stable_count = 0;
  int total_wait_ms = MIN_INITIAL_WAIT_MS;
  float stability_agonist = 0;
  float stability_antagonist = 0;
  bool is_stable = false;
  
  LOG_C1_INFO("Waiting for system convergence (max " + String(MAX_CONVERGENCE_TIME_MS) + "ms)...");
  
  while (total_wait_ms < MAX_CONVERGENCE_TIME_MS) {
    if (!safeSleepMs(POLL_INTERVAL_MS)) {
      stopDofMotors(dof_index);
      return false;
    }
    total_wait_ms += POLL_INTERVAL_MS;
    
    float curr_agonist = agonist_motor->getMultiAngleSync(false).angle;
    float curr_antagonist = antagonist_motor->getMultiAngleSync(false).angle;
    
    stability_agonist = fabs(curr_agonist - prev_agonist);
    stability_antagonist = fabs(curr_antagonist - prev_antagonist);
    
    if (stability_agonist < STABILITY_THRESHOLD && stability_antagonist < STABILITY_THRESHOLD) {
      stable_count++;
      if (stable_count >= STABLE_READINGS_REQUIRED) {
        is_stable = true;
        LOG_C1_INFO("System converged after " + String(total_wait_ms) + "ms (" + 
                 String(stable_count) + " stable readings)");
        break;
      }
    } else {
      // Reset counter - system still moving
      if (stable_count > 0) {
        LOG_C1_DEBUG("Convergence reset: agon=" + String(stability_agonist, 1) + 
                  "° antag=" + String(stability_antagonist, 1) + "°");
      }
      stable_count = 0;
    }
    
    prev_agonist = curr_agonist;
    prev_antagonist = curr_antagonist;
    
    // Log progress every second for slack recovery visibility
    if (total_wait_ms % 1000 == 0 && total_wait_ms > 0) {
      LOG_C1_INFO("Still converging at " + String(total_wait_ms) + "ms: agon_move=" + 
               String(stability_agonist, 1) + "° antag_move=" + String(stability_antagonist, 1) + "°");
    }
  }

  if (!is_stable) {
    LOG_C1_WARN_F("System not stable under tension for DOF %d after %dms", dof_index,
                  total_wait_ms);
    // (chatter log removed, v2 String pass 2026-07-06)
    LOG_C1_ERROR_F("Calibration failed - system did not converge. Possible causes:");
    LOG_C1_ERROR_F("  - Tendons extremely slack (try again)");
    LOG_C1_ERROR_F("  - Mechanical obstruction");
    LOG_C1_ERROR_F("  - Motor communication issues");
    stopDofMotors(dof_index);
    return false;
  } else {
    LOG_C1_INFO("System stable under tension.");
  }

  // Read current joint angle from shared state (updated by Core0)
  if (!shared_dof_angles.valid[dof_index]) {
    LOG_C1_ERROR_F("Cannot recalculate offsets: invalid encoder reading for DOF %d",
                   dof_index);
    stopDofMotors(dof_index);
    return false;
  }
  float current_joint_angle = shared_dof_angles.angles[dof_index];

  // Calculate expected motor angles using linear equations (joint->motor). Pass the
  // live q0 so the bilinear DOF1 anchor is captured on the SAME map surface the
  // control loop uses (q_other=NAN would bake the center-slice q0-coupling bias of
  // the calibration pose into the saved anchor, appearing later as a systematic
  // residual against the live surface).
  float recalc_q0_live = shared_dof_angles.valid[Q0_DOF] ? shared_dof_angles.angles[Q0_DOF] : NAN;
  float expected_agonist_angle, expected_antagonist_angle;
  bool equations_available =
      calculateMotorAnglesWithEquations(dof_index, current_joint_angle, current_joint_angle,
                                        expected_agonist_angle, expected_antagonist_angle,
                                        recalc_q0_live);

  if (!equations_available) {
    LOG_C1_ERROR_F("Unable to recalculate offsets - linear equations not available for DOF %d",
                   dof_index);
    LOG_C1_WARN_F("Required: perform auto-mapping to compute linear equations first");
    LOG_C1_WARN_F("Alternatively: load linear equations from flash with CMD:LOAD_LINEAR_EQUATIONS");
    stopDofMotors(dof_index);
    return false;
  }

  LOG_C1_INFO("Using linear equations to compute expected angles");

  // Read current motor angles (without applying offset)
  float current_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
  float current_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

  // (chatter collapsed to one summary line, v2 String pass 2026-07-06)
  if (LOG_LEVEL >= 3) {
    char f1[48], f2[48], f3[48];
    char ln[120]; int off = 0;
    c1cat(ln, sizeof ln, &off, "Recalc DOF %d: joint=%s expected A=%s B=%s", dof_index,
          c1f(f1, current_joint_angle, 2), c1f(f2, expected_agonist_angle, 2),
          c1f(f3, expected_antagonist_angle, 2));
    c1cat(ln, sizeof ln, &off, " raw A=%s B=%s", c1f(f1, current_agonist_angle, 2),
          c1f(f2, current_antagonist_angle, 2));
    LOG_C1_DEBUG_F("%s", ln);
  }

  // Calculate new offsets
  float new_agonist_offset;
  float new_antagonist_offset;

  if (invert_logic) {
      // For INVERTED logic (e.g., Knee Right), the motor seems to ADD the offset
      // or the sign relationship is reversed.
      // Logic: If Raw + Offset = Target, then Offset = Target - Raw
      new_agonist_offset = expected_agonist_angle - current_agonist_angle;
      new_antagonist_offset = expected_antagonist_angle - current_antagonist_angle;
  } else {
      // For STANDARD logic (e.g., Ankle), the motor SUBTRACTS the offset
      // Logic: If Raw - Offset = Target, then Offset = Raw - Target
      new_agonist_offset = current_agonist_angle - expected_agonist_angle;
      new_antagonist_offset = current_antagonist_angle - expected_antagonist_angle;
  }

  {
    char f1[48], f2[48];
    LOG_C1_DEBUG_F("New offsets: agonist=%s antagonist=%s", c1f(f1, new_agonist_offset, 2),
                   c1f(f2, new_antagonist_offset, 2));
  }

  // Set new offsets
  agonist_motor->setOffsetEncoder(new_agonist_offset);
  antagonist_motor->setOffsetEncoder(new_antagonist_offset);

  // Verify that offsets were applied correctly
  float verified_agonist_angle    = agonist_motor->getMultiAngleSync().angle;
  float verified_antagonist_angle = antagonist_motor->getMultiAngleSync().angle;

  // Calculate residual error
  float agonist_error    = fabs(verified_agonist_angle - expected_agonist_angle);
  float antagonist_error = fabs(verified_antagonist_angle - expected_antagonist_angle);

  // MODIFIED: Variable error threshold depending on DOF
  // Use a higher threshold for inversion/eversion (DOF 1) due to greater elasticity
  float ERROR_THRESHOLD = 2.0f; // Standard threshold for other DOFs

  // (chatter collapsed to one summary line, v2 String pass 2026-07-06)
  if (LOG_LEVEL >= 3) {
    char f1[48], f2[48], f3[48];
    char ln[120]; int off = 0;
    c1cat(ln, sizeof ln, &off, "Recalc verify DOF %d: A=%s B=%s", dof_index,
          c1f(f1, verified_agonist_angle, 2), c1f(f2, verified_antagonist_angle, 2));
    c1cat(ln, sizeof ln, &off, " err A=%s B=%s (limit %s°)", c1f(f1, agonist_error, 2),
          c1f(f2, antagonist_error, 2), c1f(f3, ERROR_THRESHOLD, 2));
    LOG_C1_DEBUG_F("%s", ln);
  }

  if (agonist_error > ERROR_THRESHOLD || antagonist_error > ERROR_THRESHOLD) {
    LOG_C1_WARN_F("Residual error after offset calibration");
    // (chatter log removed, v2 String pass 2026-07-06)

    // Stop motors on failure only
    stopDofMotors(dof_index);
    return false;
  }

  LOG_C1_DEBUG_F("Offsets successfully recalculated for DOF %d", dof_index);
  // (chatter log removed, v2 String pass 2026-07-06)

  // NOTE: Motors NOT stopped here — pretension torque is maintained.
  // Core0 will inject HOLDING targets immediately after all DOFs complete,
  // and the PID bumpless transfer (initializeState) will take over seamlessly.
  // This eliminates the torque gap where gravity could move the joint.

  // Set flag that offsets have been calibrated for this DOF
  motor_offsets_calibrated[dof_index] = true;

  // Save offsets for flash persistence (Core0 will write to flash)
  _saved_offsets[dof_index].agonist_offset = new_agonist_offset;
  _saved_offsets[dof_index].antagonist_offset = new_antagonist_offset;
  _saved_offsets[dof_index].joint_angle_at_calib = current_joint_angle;
  _saved_offsets[dof_index].valid = true;
  _pending_offsets_save = true;

  SERIAL_C1_COM_LN("CALIBRATION_STATUS: Offsets calibrated for DOF " + String(dof_index) +
                 " - movement enabled");

  return true;
}

// Apply saved offsets from flash to motor encoder objects (skip full recalc)
bool JointController::applySavedOffsetsToMotors(uint8_t dof_index) {
  abandonCarriedPair();  // S2 carry choke: this drives motor-CAN (getMultiAngleSync -> 0x92);
                         // a pair carried across the cycle boundary must not collide with it.
  if (dof_index >= config.dof_count) return false;

  if (!_saved_offsets[dof_index].valid) {
    LOG_C1_INFO("No saved offsets for DOF " + String(dof_index));
    return false;
  }

  if (config.dofs[dof_index].drive_type == DRIVE_DIRECT_DRIVE) {
    motor_offsets_calibrated[dof_index] = true;

    LOG_C1_INFO("DOF " + String(dof_index) + " direct-drive reference applied from flash: offset=" +
                String(_saved_offsets[dof_index].agonist_offset, 2) + " target=" +
                String(_saved_offsets[dof_index].joint_angle_at_calib, 2) + "°");
    return true;
  }

  // Find agonist and antagonist motors for this DOF
  LKM_Motor *agonist_motor = nullptr;
  LKM_Motor *antagonist_motor = nullptr;

  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      if (config.motors[i].is_agonist) {
        agonist_motor = motors[i];
      } else {
        antagonist_motor = motors[i];
      }
    }
  }

  if (agonist_motor == nullptr || antagonist_motor == nullptr) {
    LOG_C1_ERROR_F("Motors not found for DOF %d", dof_index);
    return false;
  }

  // Apply saved offsets to motor encoder objects
  agonist_motor->setOffsetEncoder(_saved_offsets[dof_index].agonist_offset);
  antagonist_motor->setOffsetEncoder(_saved_offsets[dof_index].antagonist_offset);

  // Read current joint angle for logging
  float joint_angle = shared_dof_angles.valid[dof_index]
                        ? shared_dof_angles.angles[dof_index] : NAN;

  // Verify post-apply: read calibrated motor angles and compare with expected
  float verified_agonist = agonist_motor->getMultiAngleSync().angle;   // with offset
  float verified_antagonist = antagonist_motor->getMultiAngleSync().angle;

  float expected_agonist = NAN, expected_antagonist = NAN;
  float verify_err_a = NAN, verify_err_b = NAN;
  if (hasValidEquations(dof_index) && !isnan(joint_angle)) {
    calculateMotorAnglesWithEquations(dof_index, joint_angle, joint_angle,
                                      expected_agonist, expected_antagonist);
    verify_err_a = fabs(verified_agonist - expected_agonist);
    verify_err_b = fabs(verified_antagonist - expected_antagonist);
  }

  motor_offsets_calibrated[dof_index] = true;

  LOG_C1_INFO("DOF " + String(dof_index) + " offsets applied from flash: agon=" +
           String(_saved_offsets[dof_index].agonist_offset, 2) +
           " antag=" + String(_saved_offsets[dof_index].antagonist_offset, 2));
  LOG_C1_INFO("  joint=" + String(joint_angle, 2) + "° verified: A=" +
           String(verified_agonist, 2) + " B=" + String(verified_antagonist, 2) +
           " expected: A=" + String(expected_agonist, 2) + " B=" + String(expected_antagonist, 2));
  LOG_C1_INFO("  post-apply error: A=" + String(verify_err_a, 2) +
           "° B=" + String(verify_err_b, 2) + "°");

  return true;
}


// Verify if the system is ready for movement
bool JointController::isSystemReadyForMovement() {
  // Verify that each DOF completed the startup path appropriate to its drive topology.
  for (int dof = 0; dof < config.dof_count; dof++) {
    if (config.dofs[dof].drive_type == DRIVE_DIRECT_DRIVE) {
      // Direct-drive DOFs do not use tendon equations, but they still require
      // an explicit startup-time reference validation before movement is enabled.
      if (!motor_offsets_calibrated[dof]) {
        return false;
      }
      continue;
    }

    if (!linear_equations[dof].calculated || !linear_equations[dof].agonist.valid ||
        !linear_equations[dof].antagonist.valid || !linear_equations[dof].limits_valid) {
      return false; // Missing tendon equations
    }

    // Verify that offsets have been calibrated for this DOF
    if (!motor_offsets_calibrated[dof]) {
      return false; // Offsets not calibrated
    }
  }

  return true; // System fully ready
}

void JointController::setMovementReadyForDof(uint8_t dof_index, bool ready) {
  if (dof_index >= config.dof_count) {
    return;
  }
  motor_offsets_calibrated[dof_index] = ready;
}

bool JointController::hasSavedReference(uint8_t dof_index) const {
  if (dof_index >= config.dof_count) {
    return false;
  }
  return _saved_offsets[dof_index].valid;
}

// Get mapping data for a DOF
DofMappingData_t *JointController::getMappingData(uint8_t dof_index) {
  if (dof_index >= config.dof_count) {
    return nullptr;
  }
  return &dof_mappings[dof_index];
}

// (legacy) setMappingDataFlag removed

// Implementation of getPid
bool JointController::getPid(uint8_t dof_index, uint8_t motor_type, float &kp, float &ki, float &kd,
                             float &tau) {
  // Validate parameters
  if (dof_index >= config.dof_count || (motor_type != 1 && motor_type != 2 && motor_type != 3)) {
    SERIAL_C1_COM_LN("Invalid parameters in getPid");
    return false;
  }

  int motor_index = -1;
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index != dof_index) {
      continue;
    }

    bool role_match = false;
    if (motor_type == 1) {
      role_match = (config.motors[i].role == MOTOR_ROLE_AGONIST) || config.motors[i].is_agonist;
    } else if (motor_type == 2) {
      role_match = (config.motors[i].role == MOTOR_ROLE_ANTAGONIST) ||
                   (config.motors[i].role != MOTOR_ROLE_DIRECT && !config.motors[i].is_agonist);
    } else if (motor_type == 3) {
      role_match = (config.motors[i].role == MOTOR_ROLE_DIRECT);
    }

    if (role_match) {
      motor_index = i;
      break;
    }
  }

  if (motor_index == -1) {
    LOG_C1_ERROR_F("Motor not found in getPid (DOF=%d, type=%d)", dof_index, motor_type);
    return false;
  }

  // Get PID parameters from controller
  if (pid_controllers[motor_index] != nullptr) {
    kp  = pid_controllers[motor_index]->getKp();
    ki  = pid_controllers[motor_index]->getKi();
    kd  = pid_controllers[motor_index]->getKd();
    tau = pid_controllers[motor_index]->getTau();

    LOG_C1_INFO("PID parameters fetched for DOF " + String(dof_index) + ", motor " +
             String(motor_type) + ":");
    LOG_C1_INFO("Kp: " + String(kp, 4) + ", Ki: " + String(ki, 4) + ", Kd: " + String(kd, 4) +
             ", Tau: " + String(tau, 4));

    return true;
  } else {
    LOG_C1_ERROR_F("PID controller not initialized");
    return false;
  }
}

// Implementation of setPid
bool JointController::setPid(uint8_t dof_index, uint8_t motor_type, float kp, float ki, float kd,
                             float tau) {
  // Validate parameters
  if (dof_index >= config.dof_count || (motor_type != 1 && motor_type != 2 && motor_type != 3)) {
    LOG_C1_ERROR_F("Invalid parameters in setPid");
    return false;
  }

  if (config.dofs[dof_index].drive_type == DRIVE_ANTAGONISTIC_TENDON &&
      (motor_type == 1 || motor_type == 2) &&
      kd < PID_MIN_TENDON_INNER_KD) {
    char f1[48], f2[48];
    LOG_C1_ERROR_F("Rejected unsafe tendon inner Kd for DOF %d, motor %d: Kd=%s < floor %s",
                   dof_index, motor_type, c1f(f1, kd, 3), c1f(f2, PID_MIN_TENDON_INNER_KD, 3));
    return false;
  }

  int motor_index = -1;
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index != dof_index) {
      continue;
    }

    bool role_match = false;
    if (motor_type == 1) {
      role_match = (config.motors[i].role == MOTOR_ROLE_AGONIST) || config.motors[i].is_agonist;
    } else if (motor_type == 2) {
      role_match = (config.motors[i].role == MOTOR_ROLE_ANTAGONIST) ||
                   (config.motors[i].role != MOTOR_ROLE_DIRECT && !config.motors[i].is_agonist);
    } else if (motor_type == 3) {
      role_match = (config.motors[i].role == MOTOR_ROLE_DIRECT);
    }

    if (role_match) {
      motor_index = i;
      break;
    }
  }

  if (motor_index == -1) {
    LOG_C1_ERROR_F("Motor not found in setPid (DOF=%d, type=%d)", dof_index, motor_type);
    return false;
  }

  // Set PID parameters
  if (pid_controllers[motor_index] != nullptr) {
    pid_controllers[motor_index]->setTunings(kp, ki, kd, tau);

    LOG_C1_INFO("PID parameters set for DOF " + String(dof_index) + ", motor " + String(motor_type) +
             ":");
    LOG_C1_INFO("Kp: " + String(kp, 4) + ", Ki: " + String(ki, 4) + ", Kd: " + String(kd, 4) +
             ", Tau: " + String(tau, 4));

    return true;
  } else {
    LOG_C1_ERROR_F("PID controller not initialized");
    return false;
  }
}

bool JointController::setOuterLoopParameters(uint8_t dof_index, float kp, float ki, float kd,
                                             float stiffness_deg, float cascade_influence) {
  if (dof_index >= config.dof_count) {
    LOG_C1_ERROR_F("Invalid parameters in setOuterLoopParameters (DOF out of range)");
    return false;
  }

  if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
      !std::isfinite(stiffness_deg) || !std::isfinite(cascade_influence)) {
    SERIAL_C1_COM_LN("NaN/Inf parameters in setOuterLoopParameters");
    return false;
  }

  kp                = std::max(0.0f, kp);
  ki                = std::max(0.0f, ki);
  kd                = std::max(0.0f, kd);
  stiffness_deg     = std::max(0.0f, stiffness_deg);
  cascade_influence = std::clamp(cascade_influence, 0.0f, 1.0f);

  outer_loop_kp_values[dof_index]     = kp;
  outer_loop_ki_values[dof_index]     = ki;
  outer_loop_kd_values[dof_index]     = kd;
  stiffness_ref_values[dof_index]     = stiffness_deg;
  cascade_influence_values[dof_index] = cascade_influence;

  // Sync the actual PID controller instance with new parameters
  // This ensures the PID object uses the same gains as the stored values
  if (outer_pid_controllers && outer_pid_controllers[dof_index]) {
    outer_pid_controllers[dof_index]->setTunings(kp, ki, kd, DEFAULT_OUTER_LOOP_TAU);
  }

  // HOST-PARSED echo (_extract_outer_update: marker line + Kp/Ki/Kd regex) — bytes must not change.
  {
    char f1[48], f2[48], f3[48];
    SERIAL_C1_COM_LN_F("Outer loop parameters updated for DOF %d", dof_index);
    SERIAL_C1_COM_LN_F("  Kp=%s, Ki=%s, Kd=%s", c1f(f1, kp, 4), c1f(f2, ki, 4), c1f(f3, kd, 4));
    SERIAL_C1_COM_LN_F("  Stiffness=%s°, Influence=%s%%", c1f(f1, stiffness_deg, 4),
                       c1f(f2, cascade_influence * 100.0f, 1));
  }

  return true;
}

bool HOT_FUNC(JointController::getOuterLoopParameters)(uint8_t dof_index, float &kp, float &ki, float &kd,
                                             float &stiffness_deg, float &cascade_influence) const {
  if (dof_index >= config.dof_count) {
    return false;
  }

  kp = outer_loop_kp_values ? outer_loop_kp_values[dof_index] : DEFAULT_OUTER_LOOP_KP;
  ki = outer_loop_ki_values ? outer_loop_ki_values[dof_index] : DEFAULT_OUTER_LOOP_KI;
  kd = outer_loop_kd_values ? outer_loop_kd_values[dof_index] : DEFAULT_OUTER_LOOP_KD;
  stiffness_deg =
      stiffness_ref_values ? stiffness_ref_values[dof_index] : DEFAULT_STIFFNESS_REF_DEG;
  cascade_influence =
      cascade_influence_values ? cascade_influence_values[dof_index] : DEFAULT_CASCADE_INFLUENCE;

  return true;
}

void JointController::setOuterLoopSamplingPeriod(float new_ts) {
  if (new_ts <= 0.0f || !std::isfinite(new_ts)) {
    char f1[48];
    LOG_C1_WARN_F("Invalid sampling period for outer loop: %s", c1f(f1, new_ts, 6));
    return;
  }

  // Update sampling period for all outer loop PID controllers
  // This is called when OUTER_LOOP_DIV changes or when control frequency varies
  if (outer_pid_controllers) {
    for (int i = 0; i < config.dof_count; i++) {
      if (outer_pid_controllers[i]) {
        outer_pid_controllers[i]->setSamplingPeriod(new_ts);
      }
    }
  }
}

void JointController::setInnerLoopSamplingPeriod(float new_ts) {
  if (new_ts <= 0.0f || !std::isfinite(new_ts)) {
    char f1[48];
    LOG_C1_WARN_F("Invalid sampling period for inner loop: %s", c1f(f1, new_ts, 6));
    return;
  }

  // Update sampling period for all motor (inner) PID controllers. The constructors
  // seed Ts from the PRESET motion.sampling_period (3000 us on the tendon presets),
  // which diverges from the runtime inner_loop_period_us (4000 us boot default,
  // 0x006-changeable) - the Ki*Ts / Kd/Ts scaling then runs on the wrong base.
  // Called from the control loop's on-change sync alongside the outer twin.
  if (pid_controllers) {
    for (int i = 0; i < config.motor_count; i++) {
      if (pid_controllers[i]) {
        pid_controllers[i]->setSamplingPeriod(new_ts);
      }
    }
  }
}
