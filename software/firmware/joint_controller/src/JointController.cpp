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

  // NEW: Allocate memory for linear equations
  linear_equations = new DofLinearEquations[config.dof_count]();

  // Initialize linear equations
  for (int i = 0; i < config.dof_count; i++) {
    linear_equations[i].dof_index        = i;
    linear_equations[i].calculated       = false;
    linear_equations[i].agonist.valid    = false;
    linear_equations[i].antagonist.valid = false;
    linear_equations[i].limits_valid     = false;
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
    uint8_t encoder_channel = cfg.dofs[i].encoder_channel;
    bool invert             = cfg.dofs[i].encoder_invert;
    // Set inversion flag directly in encoders object
    if (encoder_channel < DIRECT_ENCODER_COUNT) {
      encoders->setEncoderInvert(encoder_channel, invert);
      LOG_C1_DEBUG("Set inversion flag for encoder " + String(encoder_channel) + ": " +
                String(invert ? "true" : "false"));
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
  last_valid_angles    = new float[config.dof_count];
  last_read_timestamps = new uint64_t[config.dof_count];
  spike_counters       = new uint32_t[config.dof_count];

  // Apply default PID immediately; any values from flash will override them subsequently
  applyDefaultPidTunings(false);

  // Initialize with default values
  for (int i = 0; i < config.dof_count; i++) {
    last_valid_angles[i]    = 0.0f;
    last_read_timestamps[i] = 0;
    spike_counters[i]       = 0;
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
}

// ============================================================================
// INITIALIZATION & LIFECYCLE
// ============================================================================

void JointController::applyDefaultPidTunings(bool log_details) {
  if (log_details) {
    LOG_C1_INFO("Applying default PID for all motors...");
  }

  for (int i = 0; i < config.motor_count; i++) {
    float tau = config.motors[i].pid.tau;

    config.motors[i].pid.kp = DEFAULT_INNER_LOOP_KP;
    config.motors[i].pid.ki = DEFAULT_INNER_LOOP_KI;
    config.motors[i].pid.kd = DEFAULT_INNER_LOOP_KD;

    if (pid_controllers && pid_controllers[i] != nullptr) {
      pid_controllers[i]->setTunings(DEFAULT_INNER_LOOP_KP, DEFAULT_INNER_LOOP_KI,
                                     DEFAULT_INNER_LOOP_KD, tau);
    }

    if (log_details) {
      LOG_C1_DEBUG("  Motor " + String(i) + ": kp=" + String(DEFAULT_INNER_LOOP_KP, 3) +
                ", ki=" + String(DEFAULT_INNER_LOOP_KI, 3) + ", kd=" +
                String(DEFAULT_INNER_LOOP_KD, 3) + ", tau=" + String(tau, 4));
    }
  }

  if (log_details) {
    LOG_C1_INFO("Default PID applied (kp=" + String(DEFAULT_INNER_LOOP_KP, 3) +
             ", ki=" + String(DEFAULT_INNER_LOOP_KI, 3) + ", kd=" +
             String(DEFAULT_INNER_LOOP_KD, 3) + ")");
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
      LOG_C1_WARN("[SAFETY] Emergency stop detected during blocking operation");
      return false;
    }
  }
  return true;
}

// Pretension motors of a specific DOF
bool JointController::pretension(uint8_t dof_index, int torque, int duration_ms) {
  if (dof_index >= config.dof_count) {
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

  // Pretension each DOF using configured parameters
  for (int i = 0; i < config.dof_count; i++) {
    if (!pretension(i, 0, 0)) { // Use configured parameters (0 = use config)
      result = false;
    }
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
  if (dof_index >= config.dof_count) {
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

  // Release each DOF using configured parameters
  for (int i = 0; i < config.dof_count; i++) {
    if (!release(i, 0, 0)) { // Use configured parameters (0 = use config)
      result = false;
    }
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
  for (int i = 0; i < config.motor_count; i++) {
    if (motors[i] != nullptr) {
      motors[i]->motorStop();
    }
  }
  LOG_C1_INFO("Stopping all motors in JointController");
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

  // Read from shared memory (updated by Core0)
  // This is thread-safe and does NOT access SPI
  is_valid = shared_dof_angles.valid[dof_index];
  return shared_dof_angles.angles[dof_index];
}

// Read current angle of a DOF (alias for getValidatedAngle)
float JointController::getCurrentAngle(uint8_t dof_index, bool &is_valid) {
  return getValidatedAngle(dof_index, is_valid);
}

// Check if an angle is within a DOF's limits
bool JointController::isAngleInLimits(uint8_t dof_index, float angle) {
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

bool JointController::isAngleInMappingLimits(uint8_t dof_index, float angle) {
  if (dof_index >= config.dof_count) {
    return false;
  }

  if (hasValidEquations(dof_index)) {
    float min_angle = linear_equations[dof_index].joint_safe_min - LIMIT_TOLERANCE;
    float max_angle = linear_equations[dof_index].joint_safe_max + LIMIT_TOLERANCE;

    bool in_limits = (angle >= min_angle && angle <= max_angle);

    if (!in_limits) {
      LOG_C1_WARN("Angle " + String(angle, 2) + "° for DOF " + String(dof_index) +
               " outside safe limits [" + String(min_angle, 2) + "°, " +
               String(max_angle, 2) + "°] (from equations)");
    }

    return in_limits;
  }

  // Conservative fallback: use physical limits with margin if equations are unavailable
  const float CONSERVATIVE_MARGIN = 2.0f;
  float joint_min                 = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN - LIMIT_TOLERANCE;
  float joint_max                 = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN + LIMIT_TOLERANCE;

  bool in_limits = (angle >= joint_min && angle <= joint_max);

  if (!in_limits) {
    LOG_C1_WARN("Angle " + String(angle, 2) + "° for DOF " + String(dof_index) +
             " outside conservative limits [" + String(joint_min, 2) + "°, " +
             String(joint_max, 2) + "°] (equations unavailable)");
  }

  return in_limits;
}

// Check if a DOF's motors are within their mapping ranges
bool JointController::checkMotorsInRange(uint8_t dof_index, String &violation_message) {
  if (dof_index >= config.dof_count) {
    violation_message = "DOF index out of range";
    return false;
  }

  if (!hasValidEquations(dof_index)) {
    // If equations are invalid, the entire movement should not be authorized
    violation_message = "Equation limits unavailable";
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
  
  // Check if cache is valid for this DOF
  if (!cached_motor_angles.valid[dof_index]) {
    // Cache not yet populated - this is normal on first few cycles after startup
    // Skip motor check but don't fail (joint limits are still checked)
    return true;
  }
  
  // Check cache freshness (should be updated within last 100ms during active control)
  uint32_t cache_age_ms = millis() - cached_motor_angles.last_update_ms;
  if (cache_age_ms > 100) {
    // Cache is stale - control loop may not be running
    // Log warning but don't fail safety check (might be in IDLE state)
    static uint32_t last_stale_warn = 0;
    if (millis() - last_stale_warn > 1000) {
      LOG_C1_WARN("[Safety] Motor angle cache stale (" + String(cache_age_ms) + "ms old) for DOF " + 
               String(dof_index));
      last_stale_warn = millis();
    }
    return true;  // Don't fail, just skip motor check
  }
  
  // Get cached angles
  float agonist_angle = cached_motor_angles.agonist[dof_index];
  float antagonist_angle = cached_motor_angles.antagonist[dof_index];
  
  // Validate cached values (sanity check)
  if (abs(agonist_angle) > 10000.0f || abs(antagonist_angle) > 10000.0f) {
    LOG_C1_WARN("[Safety] Cached motor angles invalid for DOF " + String(dof_index) + 
             ": A=" + String(agonist_angle, 1) + " B=" + String(antagonist_angle, 1));
    return true;  // Skip check rather than fail on bad cache data
  }
  
  // Check agonist motor limits
  if (agonist_angle < agonist_min || agonist_angle > agonist_max) {
    violation_message = "!!! POSSIBLE TENDON BREAKAGE !!! AGONIST motor DOF " +
                        String(dof_index) + " out of range: " + String(agonist_angle, 1) +
                        " deg [safe range: " + String(agonist_min, 1) + " / " +
                        String(agonist_max, 1) + "]";
    return false;
  }
  
  // Check antagonist motor limits
  if (antagonist_angle < antagonist_min || antagonist_angle > antagonist_max) {
    violation_message = "!!! POSSIBLE TENDON BREAKAGE !!! ANTAGONIST motor DOF " +
                        String(dof_index) + " out of range: " + String(antagonist_angle, 1) +
                        " deg [safe range: " + String(antagonist_min, 1) + " / " +
                        String(antagonist_max, 1) + "]";
    return false;
  }

  return true; // All motors are within limits
}

bool JointController::checkWaypointSafety(uint8_t dof_index, float current_angle, float target_angle,
                                          uint32_t t_arrival_ms, uint32_t t_now, String &violation_message) {
  violation_message = "";

  // === CHECK 1: DOF index validity ===
  if (dof_index >= config.dof_count) {
    violation_message = "WAYPOINT REJECTED: DOF index " + String(dof_index) + " out of range";
    return false;
  }

  // === CHECK 2: Time validity (arrival must be in the future) ===
  if (t_arrival_ms <= t_now) {
    violation_message = "WAYPOINT REJECTED: DOF " + String(dof_index) + 
                        " arrival time in the past (t_arrival=" + String(t_arrival_ms) + 
                        " ms, t_now=" + String(t_now) + " ms)";
    return false;
  }

  // === CHECK 3: Target angle within joint limits ===
  if (!isAngleInLimits(dof_index, target_angle)) {
    const DofConfig &dof_config = config.dofs[dof_index];
    violation_message = "WAYPOINT REJECTED: DOF " + String(dof_index) + 
                        " target angle " + String(target_angle, 2) + "° outside joint limits " +
                        "[" + String(dof_config.limits.min_angle, 1) + " / " + 
                        String(dof_config.limits.max_angle, 1) + "]";
    return false;
  }

  // === CHECK 4: Target angle within mapping limits (more conservative) ===
  if (!isAngleInMappingLimits(dof_index, target_angle)) {
    float min_safe, max_safe;
    if (hasValidEquations(dof_index)) {
      min_safe = linear_equations[dof_index].joint_safe_min;
      max_safe = linear_equations[dof_index].joint_safe_max;
    } else {
      const float CONSERVATIVE_MARGIN = 2.0f;
      min_safe = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN;
      max_safe = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN;
    }
    violation_message = "WAYPOINT REJECTED: DOF " + String(dof_index) + 
                        " target angle " + String(target_angle, 2) + "° outside safe mapping limits " +
                        "[" + String(min_safe, 1) + " / " + String(max_safe, 1) + "]";
    return false;
  }

  // === CHECK 5: Velocity safety (with emergency margin) ===
  float angle_delta = abs(target_angle - current_angle);
  float time_delta_s = (t_arrival_ms - t_now) / 1000.0f;

  if (time_delta_s > 0.001f) { // Avoid division by zero (1ms minimum)
    float requested_velocity_deg_s = angle_delta / time_delta_s;
    float requested_velocity_rad_s = requested_velocity_deg_s * DEG_TO_RAD;
    
    // HARD CAP: Absolute maximum velocity regardless of configuration (150°/s)
    // This is a safety net to prevent misconfigured max_speed from causing damage
    const float ABSOLUTE_MAX_VELOCITY_DEG_S = 150.0f;
    const float ABSOLUTE_MAX_VELOCITY_RAD_S = ABSOLUTE_MAX_VELOCITY_DEG_S * DEG_TO_RAD;
    
    if (requested_velocity_deg_s > ABSOLUTE_MAX_VELOCITY_DEG_S) {
      violation_message = "WAYPOINT REJECTED (HARD LIMIT): DOF " + String(dof_index) + 
                          " requested velocity " + String(requested_velocity_deg_s, 1) + " °/s " +
                          "exceeds absolute safety limit of " + String(ABSOLUTE_MAX_VELOCITY_DEG_S, 0) + " °/s";
      return false;
    }
    
    float max_speed_rad_s = config.dofs[dof_index].motion.max_speed;
    float max_speed_deg_s = max_speed_rad_s * RAD_TO_DEG;
    
    // Emergency margin: 1.5x max_speed is considered critically dangerous
    const float EMERGENCY_MARGIN = 1.5f;
    
    if (requested_velocity_rad_s > max_speed_rad_s * EMERGENCY_MARGIN) {
      // CRITICAL: Velocity exceeds safe limits by 1.5x
      violation_message = "WAYPOINT REJECTED (EMERGENCY): DOF " + String(dof_index) + 
                          " requested velocity " + String(requested_velocity_deg_s, 1) + " °/s (" +
                          String(requested_velocity_rad_s, 2) + " rad/s) exceeds " +
                          String(EMERGENCY_MARGIN, 1) + "x max_speed " + String(max_speed_deg_s, 1) + " °/s";
      return false;
    } else if (requested_velocity_rad_s > max_speed_rad_s) {
      // WARNING: Velocity exceeds configured max but within safety margin
      // Log warning but allow movement (PID will limit actual speed)
      LOG_C1_WARN("[WAYPOINT SAFETY] DOF " + String(dof_index) + 
               " requested velocity " + String(requested_velocity_deg_s, 1) + " °/s exceeds max_speed " + 
               String(max_speed_deg_s, 1) + " °/s. Movement allowed but may lag behind.");
    }
  }

  // All checks passed
  return true;
}

bool JointController::checkSafetyForDof(uint8_t dof_index, float current_angle,
                                        String &violation_message, bool check_motors) {
  violation_message = "";

  if (dof_index >= config.dof_count) {
    violation_message = "DOF index out of range";
    return false;
  }

  if (!isAngleInLimits(dof_index, current_angle)) {
    const DofConfig &dof_config = config.dofs[dof_index];
    violation_message           = "JOINT LIMIT VIOLATED - DOF " + String(dof_index) +
                        ": angle=" + String(current_angle, 2) + " deg " +
                        "[physical range: " + String(dof_config.limits.min_angle, 1) + " / " +
                        String(dof_config.limits.max_angle, 1) + "]";
    return false;
  }

  if (!isAngleInMappingLimits(dof_index, current_angle)) {
    float min_safe;
    float max_safe;

    if (hasValidEquations(dof_index)) {
      min_safe = linear_equations[dof_index].joint_safe_min;
      max_safe = linear_equations[dof_index].joint_safe_max;
    } else {
      const float CONSERVATIVE_MARGIN = 2.0f;
      min_safe = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN;
      max_safe = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN;
    }

    violation_message = "MAPPING LIMIT VIOLATED - DOF " + String(dof_index) +
                        ": angle=" + String(current_angle, 2) + " deg " +
                        "[safe range: " + String(min_safe, 1) + " / " + String(max_safe, 1) + "]";
    return false;
  }

  if (check_motors) {
    if (!checkMotorsInRange(dof_index, violation_message)) {
      return false;
    }
  }

  return true;
}

bool JointController::checkSafetyForAllDofs(String &violation_message, bool check_motors) {
  violation_message = "";

  for (int dof = 0; dof < config.dof_count; dof++) {
    // Use shared_dof_angles (updated by Core0)
    if (!shared_dof_angles.valid[dof]) {
      violation_message = "Invalid encoder reading for DOF " + String(dof);
      return false;
    }
    float current_angle = shared_dof_angles.angles[dof];

    if (!checkSafetyForDof(dof, current_angle, violation_message, check_motors)) {
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
  if (dof_index >= config.dof_count) {
    LOG_C1_ERROR("DOF index out of range in recalculateMotorOffsets");
    return false;
  }

  // DEBUG: Print detailed information about request
  LOG_C1_DEBUG("=== DEBUG RECALC_OFFSET ===");
  LOG_C1_DEBUG("Requested DOF: " + String(dof_index));
  LOG_C1_DEBUG("DOF name: " + String(config.dofs[dof_index].name));
  LOG_C1_DEBUG("Total motors in joint: " + String(config.motor_count));

  // DEBUG: Print all motors and their DOFs
  for (int i = 0; i < config.motor_count; i++) {
    LOG_C1_DEBUG("Motor " + String(i) + ": ID=" + String(config.motors[i].id) + ", DOF=" +
                String(config.motors[i].dof_index) + ", Name=" + String(config.motors[i].name) +
                ", Agonist=" + String(config.motors[i].is_agonist ? "YES" : "NO"));
  }

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

  LOG_C1_INFO("MOTOR SEARCH RESULT:");
  LOG_C1_INFO(String("- Agonist: ") + String(agonist_motor != nullptr ? "FOUND" : "NOT FOUND") +
           (agonist_motor_id >= 0 ? String(" (CAN ID ") + String(agonist_motor_id) + ")" : ""));
  LOG_C1_INFO(String("- Antagonist: ") +
           String(antagonist_motor != nullptr ? "FOUND" : "NOT FOUND") +
           (antagonist_motor_id >= 0 ? String(" (CAN ID ") + String(antagonist_motor_id) + ")" :
                                       ""));
  LOG_C1_INFO("============================");

  // Verify that both motors are available
  if (agonist_motor == nullptr || antagonist_motor == nullptr) {
    LOG_C1_ERROR("Both motors are not available for this DOF");
    return false;
  }

  // Verify that linear equations are available (only supported method)
  if (!linear_equations[dof_index].calculated || !linear_equations[dof_index].agonist.valid ||
      !linear_equations[dof_index].antagonist.valid || !linear_equations[dof_index].limits_valid) {
    LOG_C1_ERROR("Unable to recalculate offsets - linear equations not available for DOF " + String(dof_index));
    LOG_C1_WARN("Required: Perform auto-mapping to compute linear equations first");
    LOG_C1_WARN("Alternatively: Load linear equations from flash with CMD:LOAD_LINEAR_EQUATIONS");
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
  const float MAX_TENSION_DISPLACEMENT = 10.0f; // Maximum displacement indicating tendons too loose (degrees)
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
          LOG_C1_WARN("Torque limited to max allowed: " + String((int)max_allowed_torque));
        }
        
        if (new_torque > current_torque) {
          current_torque = new_torque;
          LOG_C1_INFO("Tendons loose - increasing torque to " + String((int)current_torque) + " for next attempt");
        } else {
          LOG_C1_WARN("Already at max torque, cannot increase further");
          break;  // No point in retrying at same torque
        }
      } else {
        // Both too stiff - increasing torque won't help
        LOG_C1_WARN("Tendons appear too stiff - cannot improve with more torque");
        break;
      }
    }
  }
  
  // Final verdict on tension
  if (!tension_ok) {
    bool is_too_loose = (agonist_displacement > MAX_TENSION_DISPLACEMENT || 
                         antagonist_displacement > MAX_TENSION_DISPLACEMENT);
    
    if (is_too_loose) {
      LOG_C1_ERROR("Tendon tension FAILED for DOF " + String(dof_index) + 
               " after " + String(tension_attempt) + " attempts");
      LOG_C1_ERROR("Tendons may be disconnected or severely slack");
      LOG_C1_ERROR("Check tendon connections before retrying");
      stopDofMotors(dof_index);
      return false;  // FAIL - tendons too loose even with max torque
    } else {
      // Too stiff - this is unusual but not necessarily fatal
      LOG_C1_WARN("Tendon tension suboptimal for DOF " + String(dof_index) + " (too stiff)");
      LOG_C1_WARN("Continuing calibration; results may be suboptimal");
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
    LOG_C1_WARN("System not stable under tension for DOF " + String(dof_index) + 
             " after " + String(total_wait_ms) + "ms");
    LOG_C1_DEBUG(String("Final agonist movement: ") + String(stability_agonist));
    LOG_C1_DEBUG(String("Final antagonist movement: ") + String(stability_antagonist));
    LOG_C1_ERROR("Calibration failed - system did not converge. Possible causes:");
    LOG_C1_ERROR("  - Tendons extremely slack (try again)");
    LOG_C1_ERROR("  - Mechanical obstruction");
    LOG_C1_ERROR("  - Motor communication issues");
    stopDofMotors(dof_index);
    return false;
  } else {
    LOG_C1_INFO("System stable under tension.");
  }

  // Read current joint angle from shared state (updated by Core0)
  if (!shared_dof_angles.valid[dof_index]) {
    LOG_C1_ERROR("Cannot recalculate offsets: invalid encoder reading for DOF " +
              String(dof_index));
    stopDofMotors(dof_index);
    return false;
  }
  float current_joint_angle = shared_dof_angles.angles[dof_index];

  // Calculate expected motor angles using linear equations (joint->motor)
  float expected_agonist_angle, expected_antagonist_angle;
  bool equations_available =
      calculateMotorAnglesWithEquations(dof_index, current_joint_angle, current_joint_angle,
                                        expected_agonist_angle, expected_antagonist_angle);

  if (!equations_available) {
    LOG_C1_ERROR("Unable to recalculate offsets - linear equations not available for DOF " +
              String(dof_index));
    LOG_C1_WARN("Required: perform auto-mapping to compute linear equations first");
    LOG_C1_WARN("Alternatively: load linear equations from flash with CMD:LOAD_LINEAR_EQUATIONS");
    stopDofMotors(dof_index);
    return false;
  }

  LOG_C1_INFO("Using linear equations to compute expected angles");

  LOG_C1_DEBUG(String("Current joint angle: ") + String(current_joint_angle));
  LOG_C1_DEBUG(String("Expected agonist angle: ") + String(expected_agonist_angle));
  LOG_C1_DEBUG(String("Expected antagonist angle: ") + String(expected_antagonist_angle));

  // Read current motor angles (without applying offset)
  float current_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
  float current_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

  LOG_C1_DEBUG(String("Current agonist motor angle (raw): ") + String(current_agonist_angle));
  LOG_C1_DEBUG(String("Current antagonist motor angle (raw): ") + String(current_antagonist_angle));

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

  LOG_C1_DEBUG(String("New agonist offset: ") + String(new_agonist_offset));
  LOG_C1_DEBUG(String("New antagonist offset: ") + String(new_antagonist_offset));

  // Set new offsets
  agonist_motor->setOffsetEncoder(new_agonist_offset);
  antagonist_motor->setOffsetEncoder(new_antagonist_offset);

  // Verify that offsets were applied correctly
  float verified_agonist_angle    = agonist_motor->getMultiAngleSync().angle;
  float verified_antagonist_angle = antagonist_motor->getMultiAngleSync().angle;
  LOG_C1_DEBUG(String("Verified agonist angle: ") + String(verified_agonist_angle));
  LOG_C1_DEBUG(String("Verified antagonist angle: ") + String(verified_antagonist_angle));

  // Calculate residual error
  float agonist_error    = fabs(verified_agonist_angle - expected_agonist_angle);
  float antagonist_error = fabs(verified_antagonist_angle - expected_antagonist_angle);

  // MODIFIED: Variable error threshold depending on DOF
  // Use a higher threshold for inversion/eversion (DOF 1) due to greater elasticity
  float ERROR_THRESHOLD = 2.0f; // Standard threshold for other DOFs

  if (agonist_error > ERROR_THRESHOLD || antagonist_error > ERROR_THRESHOLD) {
    LOG_C1_WARN("Residual error after offset calibration");
    LOG_C1_DEBUG(String("Agonist error: ") + String(agonist_error));
    LOG_C1_DEBUG(String("Antagonist error: ") + String(antagonist_error));
    LOG_C1_DEBUG(String("Allowed error threshold: ") + String(ERROR_THRESHOLD) + "°");

    // Advanced error analysis
    LOG_C1_DEBUG("=== DETAILED ERROR ANALYSIS ===");
    LOG_C1_DEBUG(String("Joint type: ") + String(config.name));
    LOG_C1_DEBUG(String("DOF: ") + String(config.dofs[dof_index].name) + " (index " +
              String(dof_index) + ")");
    LOG_C1_DEBUG(String("Encoder inverted: ") +
             String(config.dofs[dof_index].encoder_invert ? "YES" : "NO"));
    LOG_C1_DEBUG(String("Encoder channel: ") + String(config.dofs[dof_index].encoder_channel));
    LOG_C1_DEBUG(String("Angle limits: ") + String(config.dofs[dof_index].limits.min_angle) +
              "° / " + String(config.dofs[dof_index].limits.max_angle) + "°");
    LOG_C1_DEBUG(String("Current joint angle: ") + String(current_joint_angle) + "°");
    LOG_C1_DEBUG(String("Agonist error %: ") +
              String((agonist_error / fabs(expected_agonist_angle)) * 100, 1) + "%");
    LOG_C1_DEBUG(String("Antagonist error %: ") +
              String((antagonist_error / fabs(expected_antagonist_angle)) * 100, 1) + "%");
    LOG_C1_DEBUG("====================================");

    // Stop motors on failure only
    stopDofMotors(dof_index);
    return false;
  }

  LOG_C1_DEBUG("Offsets successfully recalculated for DOF " + String(dof_index));
  LOG_C1_DEBUG("New agonist offset: " + String(new_agonist_offset));
  LOG_C1_DEBUG("New antagonist offset: " + String(new_antagonist_offset));
  LOG_C1_DEBUG("Residual agonist error: " + String(agonist_error) +
            "° (limit: " + String(ERROR_THRESHOLD) + "°)");
  LOG_C1_DEBUG("Residual antagonist error: " + String(antagonist_error) +
            "° (limit: " + String(ERROR_THRESHOLD) + "°)");

  // Calibration details
  LOG_C1_DEBUG("AGONIST: raw=" + String(current_agonist_angle) +
            " expected=" + String(expected_agonist_angle) +
            " calibrated=" + String(verified_agonist_angle) +
            " deviation=" + String(agonist_error));
  LOG_C1_DEBUG("ANTAGONIST: raw=" + String(current_antagonist_angle) +
            " expected=" + String(expected_antagonist_angle) +
            " calibrated=" + String(verified_antagonist_angle) +
            " deviation=" + String(antagonist_error));
  LOG_C1_DEBUG("Joint angle: " + String(current_joint_angle));

  // Tension and stability information
  LOG_C1_DEBUG("Tension: agonist_disp=" + String(agonist_displacement) +
            " antagonist_disp=" + String(antagonist_displacement));
  LOG_C1_DEBUG("Stability: agonist=" + String(stability_agonist) +
            " antagonist=" + String(stability_antagonist));
  LOG_C1_DEBUG("Tendon tension: " + String(tension_ok ? "OK" : "NOT OPTIMAL") +
            ", System stability: " + String(is_stable ? "OK" : "NOT OPTIMAL"));

  // Method used
  LOG_C1_DEBUG("Method: LINEAR EQUATIONS (direct computation)");

  // NOTE: Motors NOT stopped here — pretension torque is maintained.
  // Core0 will inject HOLDING waypoints immediately after all DOFs complete,
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

// Verify if the system is ready for movement
bool JointController::isSystemReadyForMovement() {
  // Verify that linear equations are available for all DOFs
  for (int dof = 0; dof < config.dof_count; dof++) {
    if (!linear_equations[dof].calculated || !linear_equations[dof].agonist.valid ||
        !linear_equations[dof].antagonist.valid || !linear_equations[dof].limits_valid) {
      return false; // Missing linear equations
    }

    // Verify that offsets have been calibrated for this DOF
    if (!motor_offsets_calibrated[dof]) {
      return false; // Offsets not calibrated
    }
  }

  return true; // System fully ready
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
  if (dof_index >= config.dof_count || (motor_type != 1 && motor_type != 2)) {
    SERIAL_C1_COM_LN("Invalid parameters in getPid");
    return false;
  }

  // Find corresponding motor using is_agonist flag (1=agonist, 2=antagonist)
  bool want_agonist = (motor_type == 1);
  int motor_index = -1;
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index && config.motors[i].is_agonist == want_agonist) {
      motor_index = i;
      break;
    }
  }

  if (motor_index == -1) {
    LOG_C1_ERROR("Motor not found in getPid (DOF=" + String(dof_index) +
              ", type=" + String(motor_type) + ")");
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
    LOG_C1_ERROR("PID controller not initialized");
    return false;
  }
}

// Implementation of setPid
bool JointController::setPid(uint8_t dof_index, uint8_t motor_type, float kp, float ki, float kd,
                             float tau) {
  // Validate parameters
  if (dof_index >= config.dof_count || (motor_type != 1 && motor_type != 2)) {
    LOG_C1_ERROR("Invalid parameters in setPid");
    return false;
  }

  // Find corresponding motor using is_agonist flag (1=agonist, 2=antagonist)
  bool want_agonist = (motor_type == 1);
  int motor_index = -1;
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index && config.motors[i].is_agonist == want_agonist) {
      motor_index = i;
      break;
    }
  }

  if (motor_index == -1) {
    LOG_C1_ERROR("Motor not found in setPid (DOF=" + String(dof_index) +
              ", type=" + String(motor_type) + ")");
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
    LOG_C1_ERROR("PID controller not initialized");
    return false;
  }
}

bool JointController::setOuterLoopParameters(uint8_t dof_index, float kp, float ki, float kd,
                                             float stiffness_deg, float cascade_influence) {
  if (dof_index >= config.dof_count) {
    LOG_C1_ERROR("Invalid parameters in setOuterLoopParameters (DOF out of range)");
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

  SERIAL_C1_COM_LN("Outer loop parameters updated for DOF " + String(dof_index));
  SERIAL_C1_COM_LN("  Kp=" + String(kp, 4) + ", Ki=" + String(ki, 4) + ", Kd=" + String(kd, 4));
  SERIAL_C1_COM_LN("  Stiffness=" + String(stiffness_deg, 4) +
                 "°, Influence=" + String(cascade_influence * 100.0f, 1) + "%");

  return true;
}

bool JointController::getOuterLoopParameters(uint8_t dof_index, float &kp, float &ki, float &kd,
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
    LOG_C1_WARN("Invalid sampling period for outer loop: " + String(new_ts, 6));
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

