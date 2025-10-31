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

// ============================================================================
// EXTERNAL VARIABLES & HELPER FUNCTIONS
// ============================================================================
// These variables and functions support inter-core communication and
// movement sample logging for debugging and telemetry.

// === EXTERNAL VARIABLES FOR THE NEW COMMUNICATION SYSTEM ===
extern volatile bool emergency_stop_requested;
extern volatile bool buffer_ready[2];
extern volatile int active_buffer;
extern volatile uint8_t pending_command_type;
extern queue_t movement_sample_queue;
extern volatile bool movement_sample_stream_active;
extern volatile bool movement_sample_stream_done;
extern volatile uint8_t movement_sample_joint_id;
extern volatile bool movement_sample_overflow;
extern uint16_t movement_sample_counters[MAX_DOFS];

static inline void drainMovementSampleQueue() {
  movement_sample_t tmp;
  while (queue_try_remove(&movement_sample_queue, &tmp)) {
  }
}

static inline void startMovementLogging(const JointConfig &cfg) {
  drainMovementSampleQueue();
  for (int i = 0; i < MAX_DOFS; ++i) {
    movement_sample_counters[i] = 0;
  }
  movement_sample_joint_id      = cfg.joint_id;
  movement_sample_stream_active = true;
  movement_sample_stream_done   = false;
  movement_sample_overflow      = false;
}

static inline void stopMovementLogging() {
  movement_sample_stream_done   = true;
  movement_sample_stream_active = false;
}

static inline void logMovementSample(uint8_t dof_index, float joint_target, float joint_actual,
                                     float motor_agonist_curr, float motor_antagonist_curr,
                                     float motor_agonist_ref, float motor_antagonist_ref,
                                     float torque_agonist, float torque_antagonist) {
  if (!movement_sample_stream_active) {
    return;
  }
  if (dof_index >= MAX_DOFS) {
    return;
  }
  movement_sample_t sample;
  sample.dof                   = dof_index;
  sample.index                 = movement_sample_counters[dof_index]++;
  sample.joint_target          = joint_target;
  sample.joint_actual          = joint_actual;
  sample.motor_agonist_curr    = motor_agonist_curr;
  sample.motor_antagonist_curr = motor_antagonist_curr;
  sample.motor_agonist_ref     = motor_agonist_ref;
  sample.motor_antagonist_ref  = motor_antagonist_ref;
  sample.torque_agonist        = torque_agonist;
  sample.torque_antagonist     = torque_antagonist;

  if (!queue_try_add(&movement_sample_queue, &sample)) {
    movement_sample_overflow = true;
  }
}

// ============================================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================================

// Constructor
JointController::JointController(const JointConfig &cfg, MCP_CAN *can, Encoders *enc) {
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

  // Set encoder inversion flags based on configuration
  for (int i = 0; i < config.dof_count; i++) {
    uint8_t encoder_channel = cfg.dofs[i].encoder_channel;
    bool invert             = cfg.dofs[i].encoder_invert;
    // Set inversion flag directly in encoders object
    if (encoder_channel < ENCODER_COUNT) {
      encoders->setEncoderInvert(encoder_channel, invert);
      LOG_DEBUG("Set inversion flag for encoder " + String(encoder_channel) + ": " +
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

  // Free memory for PID controllers
  for (int i = 0; i < config.motor_count; i++) { // Now one PID per motor
    delete pid_controllers[i];
  }
  delete[] pid_controllers;

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
    LOG_INFO("Applying default PID for all motors...");
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
      LOG_DEBUG("  Motor " + String(i) + ": kp=" + String(DEFAULT_INNER_LOOP_KP, 3) +
                ", ki=" + String(DEFAULT_INNER_LOOP_KI, 3) + ", kd=" +
                String(DEFAULT_INNER_LOOP_KD, 3) + ", tau=" + String(tau, 4));
    }
  }

  if (log_details) {
    LOG_INFO("Default PID applied (kp=" + String(DEFAULT_INNER_LOOP_KP, 3) +
             ", ki=" + String(DEFAULT_INNER_LOOP_KI, 3) + ", kd=" +
             String(DEFAULT_INNER_LOOP_KD, 3) + ")");
  }
}

// Initialize controller
bool JointController::init() {
  LOG_INFO(String("Initializing joint controller: ") + String(config.name));

  // Initialize all motors
  for (int i = 0; i < config.motor_count; i++) {
    motors[i]->init();
  }

  // Apply configured angular offsets for each DOF
  for (int i = 0; i < config.dof_count; i++) {
    float zero_offset = config.dofs[i].zero_mapping.zero_angle_offset;
    if (zero_offset != 0.0f) {
      uint8_t encoder_channel = config.dofs[i].encoder_channel;
      LOG_INFO(String("Applying angle offset for DOF ") + String(i) + ": " +
               String(zero_offset));
      encoders->setJointOffset(encoder_channel, zero_offset);
    }
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
  LOG_INFO("Pretensioning parameters: index=" + String(dof_index) + " torque=" + String(torque) +
           " duration=" + String(duration_ms));

  // Wait for specified duration
  sleep_ms(duration_ms);

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

  LOG_INFO("Pretensioning all DOFs with configured parameters");

  // Wait at the end for all DOFs using first DOF timeout (or 100ms as fallback)
  int final_timeout = (config.dof_count > 0) ? config.dofs[0].zero_mapping.pretension_timeout : 100;

  // Wait at the end for all DOFs
  sleep_ms(final_timeout);

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
  LOG_INFO("Releasing DOF: index=" + String(dof_index) + " torque=" + String(torque) +
           " duration=" + String(duration_ms));

  // Wait for specified duration
  sleep_ms(duration_ms);

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

  LOG_INFO("Releasing all DOFs with configured parameters");

  // Wait at the end for all DOFs using first DOF timeout (or 100ms as fallback)
  int final_timeout = (config.dof_count > 0) ? config.dofs[0].zero_mapping.pretension_timeout : 100;

  // Wait at the end for all DOFs
  sleep_ms(final_timeout);

  // Stop all motors
  stopAllMotors();

  return result;
}

// Set current position as zero for a DOF without moving the joint
bool JointController::setZeroCurrentPos(uint8_t dof_index) {
  if (dof_index >= config.dof_count) {
    LOG_ERROR("Invalid DOF index in setZeroCurrentPos");
    return false;
  }

  LOG_INFO("Setting current position as zero for DOF: " + String(dof_index));

  // Get the encoder channel for this DOF
  uint8_t encoder_channel = config.dofs[dof_index].encoder_channel;

  // Get the configured angular offset
  float zero_offset = config.dofs[dof_index].zero_mapping.zero_angle_offset;

  // Zero ONLY the encoder for the specific channel for this DOF
  encoders->zeroEncoders(encoder_channel);

  // Set the desired offset (or keep the previous one if equal)
  if (zero_offset != 0.0f) {
    Serial.print("Applying angular offset: ");
    Serial.println(zero_offset);

    // Set the encoder offset for this DOF
    encoders->setJointOffset(encoder_channel, zero_offset);
  }

  // Zero motor offsets for this DOF
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      motors[i]->zeroEncoderOffset();
    }
  }

  LOG_INFO("Zero successfully set for DOF " + String(dof_index));
  return true;
}

// Stop all motors
void JointController::stopAllMotors() {
  for (int i = 0; i < config.motor_count; i++) {
    motors[i]->motorStop();
  }
  LOG_INFO("Stopping all motors in JointController");
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

// New method to validate encoder readings and detect SPI spikes
float JointController::getValidatedAngle(uint8_t dof_index, bool &is_valid) {
  if (dof_index >= config.dof_count) {
    is_valid = false;
    return 0.0f;
  }

  // Get the raw reading
  uint8_t encoder_channel = config.dofs[dof_index].encoder_channel;
  float raw_angle         = encoders->getJointAngle(encoder_channel, is_valid);

  if (!is_valid) {
    // If the reading is invalid, return the last valid reading
    return last_valid_angles[dof_index];
  }

  // Get the current timestamp
  uint64_t current_time = time_us_64();

  // If this is the first read or too much time has passed, accept the value
  if (last_read_timestamps[dof_index] == 0 ||
      (current_time - last_read_timestamps[dof_index]) > 1000000) { // > 1 second
    last_valid_angles[dof_index]    = raw_angle;
    last_read_timestamps[dof_index] = current_time;
    return raw_angle;
  }

  // Calculate elapsed time in seconds
  float dt = (current_time - last_read_timestamps[dof_index]) / 1000000.0f;

  // Compute implicit speed
  float angle_diff       = fabs(raw_angle - last_valid_angles[dof_index]);
  float implied_velocity = angle_diff / dt; // degrees/second

  // Convert to rad/s for comparison with max_speed
  float implied_velocity_rad = implied_velocity * PI / 180.0f;

  // Get the configured maximum speed for this DOF
  float max_speed = config.dofs[dof_index].motion.max_speed;

  // Check whether the implicit speed exceeds the limit with margin
  if (implied_velocity_rad > max_speed * SPIKE_DETECTION_MARGIN) {
    // Possible spike detected — re‑read multiple times to confirm
    spike_counters[dof_index]++; // Increment spike counter

    LOG_WARN(String("[SPIKE DETECT #") + String(spike_counters[dof_index]) + "] DOF " +
             String(dof_index) + ": abnormal speed " + String(implied_velocity) +
             "°/s (" + String(implied_velocity_rad) + " rad/s) > limit " +
             String(max_speed * SPIKE_DETECTION_MARGIN) + " rad/s. Delta: " +
             String(angle_diff) + "° in " + String(dt * 1000) +
             " ms. Suspicious reading: " + String(raw_angle) +
             "° (from " + String(last_valid_angles[dof_index]) + "°)");

    float readings[MAX_CONSECUTIVE_READS];
    bool all_valid = true;

    for (int i = 0; i < MAX_CONSECUTIVE_READS; i++) {
      bool read_valid;
      readings[i] = encoders->getJointAngle(encoder_channel, read_valid);
      if (!read_valid) {
        all_valid = false;
        break;
      }
      sleep_us(100); // Brief pause between readings
    }

    if (all_valid) {
      // Calculate the average of the readings
      float sum = 0.0f;
      for (int i = 0; i < MAX_CONSECUTIVE_READS; i++) {
        sum += readings[i];
      }
      float avg_reading = sum / MAX_CONSECUTIVE_READS;

      // Calculate the standard deviation
      float variance = 0.0f;
      for (int i = 0; i < MAX_CONSECUTIVE_READS; i++) {
        float diff = readings[i] - avg_reading;
        variance += diff * diff;
      }
      float std_dev = sqrt(variance / MAX_CONSECUTIVE_READS);

      // If readings are consistent (low deviation), use the average
      if (std_dev < 1.0f) { // Threshold of 1 degree
        raw_angle = avg_reading;

        DBG_PRINT("[SPIKE CORRECTED] DOF ");
        DBG_PRINT(dof_index);
        DBG_PRINT(": Original reading ");
        DBG_PRINT(last_valid_angles[dof_index]);
        DBG_PRINT("° -> spike ");
        DBG_PRINT(encoders->getJointAngle(encoder_channel, is_valid));
        DBG_PRINT("° -> corrected to ");
        DBG_PRINT(avg_reading);
        DBG_PRINT("° (3 readings: ");
        for (int i = 0; i < MAX_CONSECUTIVE_READS; i++) {
          DBG_PRINT(readings[i]);
          if (i < MAX_CONSECUTIVE_READS - 1)
            DBG_PRINT(", ");
        }
        DBG_PRINT(", std dev: ");
        DBG_PRINT(std_dev);
        DBG_PRINTLN("°)");
      } else {
        // Inconsistent readings, keep the last valid value
        is_valid = false;

        DBG_PRINT("[SPIKE REJECTED] DOF ");
        DBG_PRINT(dof_index);
        DBG_PRINT(": Inconsistent readings (std dev: ");
        DBG_PRINT(std_dev);
        DBG_PRINT("°). 3 readings: ");
        for (int i = 0; i < MAX_CONSECUTIVE_READS; i++) {
          DBG_PRINT(readings[i]);
          if (i < MAX_CONSECUTIVE_READS - 1)
            DBG_PRINT(", ");
        }
        DBG_PRINT(". Keeping last valid value: ");
        DBG_PRINT(last_valid_angles[dof_index]);
        DBG_PRINTLN("°");

        return last_valid_angles[dof_index];
      }
    } else {
      // Not all readings are valid, keep the last value
      is_valid = false;

      DBG_PRINT("[SPIKE ERROR] DOF ");
      DBG_PRINT(dof_index);
      DBG_PRINT(": Invalid SPI readings during spike verification. Keeping last value: ");
      DBG_PRINT(last_valid_angles[dof_index]);
      DBG_PRINTLN("°");

      return last_valid_angles[dof_index];
    }
  }

  // Update historical values
  last_valid_angles[dof_index]    = raw_angle;
  last_read_timestamps[dof_index] = current_time;

  return raw_angle;
}

// Read current angle of a DOF
float JointController::getCurrentAngle(uint8_t dof_index, bool &is_valid) {
  if (dof_index >= config.dof_count) {
    is_valid = false;
    return 0.0f;
  }

  // Use new validation method that detects and filters SPI spikes
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
bool JointController::isAngleInMappingLimits(uint8_t dof_index, float angle) {
  if (dof_index >= config.dof_count) {
    return false;
  }

  if (hasValidEquations(dof_index)) {
    float min_angle = linear_equations[dof_index].joint_safe_min;
    float max_angle = linear_equations[dof_index].joint_safe_max;

    bool in_limits = (angle >= min_angle && angle <= max_angle);

    if (!in_limits) {
      DBG_PRINTLN("WARNING: Angle " + String(angle, 2) + "° for DOF " + String(dof_index) +
                  " outside safe limits derived from equations");
      DBG_PRINTLN("  Allowed range: [" + String(min_angle, 2) + "°, " + String(max_angle, 2) +
                  "°]");
    }

    return in_limits;
  }

  // Conservative fallback: use physical limits with margin if equations are unavailable
  const float CONSERVATIVE_MARGIN = 2.0f;
  float joint_min                 = config.dofs[dof_index].limits.min_angle + CONSERVATIVE_MARGIN;
  float joint_max                 = config.dofs[dof_index].limits.max_angle - CONSERVATIVE_MARGIN;

  bool in_limits = (angle >= joint_min && angle <= joint_max);

  if (!in_limits) {
    DBG_PRINTLN("WARNING: Angle " + String(angle, 2) + "° for DOF " + String(dof_index) +
                " outside conservative physical limits (equations unavailable)");
    DBG_PRINTLN("  Conservative range: [" + String(joint_min, 2) + "°, " + String(joint_max, 2) +
                "°]");
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

  // Check motors for this DOF
  for (int motor_idx = 0; motor_idx < config.motor_count; motor_idx++) {
    if (config.motors[motor_idx].dof_index == dof_index) {
      LKM_Motor *motor = motors[motor_idx];
      if (motor != nullptr) {
        float motor_angle = motor->getMultiAngleSync().angle;
        bool is_agonist   = config.motors[motor_idx].is_agonist;

        if (is_agonist) {
          // Agonist motor check
          if (motor_angle < agonist_min || motor_angle > agonist_max) {
            violation_message = "!!! POSSIBLE TENDON BREAKAGE !!! AGONIST motor DOF " +
                                String(dof_index) + " out of range: " + String(motor_angle, 1) +
                                " deg [safe range: " + String(agonist_min, 1) + " / " +
                                String(agonist_max, 1) + "]";
            return false;
          }
        } else {
          // Antagonist motor check
          if (motor_angle < antagonist_min || motor_angle > antagonist_max) {
            violation_message = "!!! POSSIBLE TENDON BREAKAGE !!! ANTAGONIST motor DOF " +
                                String(dof_index) + " out of range: " + String(motor_angle, 1) +
                                " deg [safe range: " + String(antagonist_min, 1) + " / " +
                                String(antagonist_max, 1) + "]";
            return false;
          }
        }
      }
    }
  }

  return true; // All motors are within limits
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
    bool is_valid;
    float current_angle = getCurrentAngle(dof, is_valid);

    if (!is_valid) {
      violation_message = "Invalid encoder reading for DOF " + String(dof);
      return false;
    }

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
    DBG_PRINTLN("DOF index out of range in recalculateMotorOffsets");
    return false;
  }

  // DEBUG: Print detailed information about request
  DBG_PRINTLN("=== DEBUG RECALC_OFFSET ===");
  DBG_PRINTLN("Requested DOF: " + String(dof_index));
  DBG_PRINTLN("DOF name: " + String(config.dofs[dof_index].name));
  DBG_PRINTLN("Total motors in joint: " + String(config.motor_count));

  // DEBUG: Print all motors and their DOFs
  for (int i = 0; i < config.motor_count; i++) {
    DBG_PRINTLN("Motor " + String(i) + ": ID=" + String(config.motors[i].id) + ", DOF=" +
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
      LOG_INFO("FOUND motor for DOF " + String(dof_index) + ": " + String(config.motors[i].name) +
               " (ID=" + String(config.motors[i].id) +
               ", Agonist=" + String(config.motors[i].is_agonist ? "YES" : "NO") + ")");

      if (config.motors[i].is_agonist) {
        agonist_motor    = motors[i];
        agonist_motor_id = config.motors[i].id;
        LOG_INFO("-> Assigned as AGONIST (CAN ID " + String(agonist_motor_id) + ")");
      } else {
        antagonist_motor    = motors[i];
        antagonist_motor_id = config.motors[i].id;
        LOG_INFO("-> Assigned as ANTAGONIST (CAN ID " + String(antagonist_motor_id) + ")");
      }
    }
  }

  LOG_INFO("MOTOR SEARCH RESULT:");
  LOG_INFO(String("- Agonist: ") + String(agonist_motor != nullptr ? "FOUND" : "NOT FOUND") +
           (agonist_motor_id >= 0 ? String(" (CAN ID ") + String(agonist_motor_id) + ")" : ""));
  LOG_INFO(String("- Antagonist: ") +
           String(antagonist_motor != nullptr ? "FOUND" : "NOT FOUND") +
           (antagonist_motor_id >= 0 ? String(" (CAN ID ") + String(antagonist_motor_id) + ")" :
                                       ""));
  LOG_INFO("============================");

  // Verify that both motors are available
  if (agonist_motor == nullptr || antagonist_motor == nullptr) {
    LOG_ERROR("Both motors are not available for this DOF");
    return false;
  }

  // Verify that linear equations are available (only supported method)
  if (!linear_equations[dof_index].calculated || !linear_equations[dof_index].agonist.valid ||
      !linear_equations[dof_index].antagonist.valid || !linear_equations[dof_index].limits_valid) {
    LOG_ERROR("Unable to recalculate offsets - linear equations not available for DOF " + String(dof_index));
    LOG_WARN("Required: Perform auto-mapping to compute linear equations first");
    LOG_WARN("Alternatively: Load linear equations from flash with CMD:LOAD_LINEAR_EQUATIONS");
    stopDofMotors(dof_index);
    return false;
  }

  LOG_INFO("Linear equations available for DOF " + String(dof_index) + " - offset recalculation enabled");

  // Apply pretension to eliminate tendon slack
  LOG_INFO("Applying pretension for DOF " + String(dof_index));

  // First read current angles without torque
  float initial_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
  float initial_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

  // Apply opposite torque to tension the system
  agonist_motor->setTorque(-pretension_torque);
  antagonist_motor->setTorque(pretension_torque);

  // Brief wait to allow system to react
  sleep_ms(100);

  // Read angles under tension
  float tensioned_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
  float tensioned_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

  // Calculate displacement under tension
  float agonist_displacement    = fabs(tensioned_agonist_angle - initial_agonist_angle);
  float antagonist_displacement = fabs(tensioned_antagonist_angle - initial_antagonist_angle);

  const float MIN_TENSION_DISPLACEMENT = 0.1f; // Minimum displacement indicating tension (degrees)
  const float MAX_TENSION_DISPLACEMENT =
      10.0f; // Maximum displacement indicating tendons too loose (degrees)

  // Verify that there is correct tension
  bool tension_ok = (agonist_displacement >= MIN_TENSION_DISPLACEMENT &&
                     agonist_displacement <= MAX_TENSION_DISPLACEMENT &&
                     antagonist_displacement >= MIN_TENSION_DISPLACEMENT &&
                     antagonist_displacement <= MAX_TENSION_DISPLACEMENT);

  if (!tension_ok) {
    LOG_WARN("Tendon tension not optimal for DOF " + String(dof_index));
    LOG_DEBUG(String("Agonist displacement: ") + String(agonist_displacement) +
              (agonist_displacement < MIN_TENSION_DISPLACEMENT
                   ? " (too stiff)"
                   : (agonist_displacement > MAX_TENSION_DISPLACEMENT ? " (too loose)" :
                                                                      " (ok)")));
    LOG_DEBUG(String("Antagonist displacement: ") + String(antagonist_displacement) +
              (antagonist_displacement < MIN_TENSION_DISPLACEMENT
                   ? " (too stiff)"
                   : (antagonist_displacement > MAX_TENSION_DISPLACEMENT ? " (too loose)" :
                                                                         " (ok)")));
    LOG_WARN("Continuing calibration; results may be suboptimal.");
  } else {
    LOG_INFO("Tendon tension optimal.");
  }

  // Continue with full pretension
  sleep_ms(pretension_duration_ms - 100); // Subtract the 100 ms already elapsed

  // Apply a lighter torque while maintaining tension during calibration
  float holding_torque = pretension_torque / 2;
  agonist_motor->setTorque(-holding_torque);
  antagonist_motor->setTorque(holding_torque);
  sleep_ms(150); // Brief pause to stabilize

  // Verify that position is stable (no continuous movement)
  float stability_check_agonist    = agonist_motor->getMultiAngleSync(false).angle;
  float stability_check_antagonist = antagonist_motor->getMultiAngleSync(false).angle;

  sleep_ms(100); // Wait a brief moment

  float stability_check_agonist2    = agonist_motor->getMultiAngleSync(false).angle;
  float stability_check_antagonist2 = antagonist_motor->getMultiAngleSync(false).angle;

  float stability_agonist    = fabs(stability_check_agonist2 - stability_check_agonist);
  float stability_antagonist = fabs(stability_check_antagonist2 - stability_check_antagonist);

  const float STABILITY_THRESHOLD = 2.0f; // Threshold to consider stable (degrees)

  bool is_stable =
      (stability_agonist < STABILITY_THRESHOLD && stability_antagonist < STABILITY_THRESHOLD);

  if (!is_stable) {
    LOG_WARN("System not stable under tension for DOF " + String(dof_index));
    LOG_DEBUG(String("Agonist movement: ") + String(stability_agonist));
    LOG_DEBUG(String("Antagonist movement: ") + String(stability_antagonist));
    LOG_ERROR("Calibration failed.");
    stopDofMotors(dof_index);
    return false;
  } else {
    LOG_INFO("System stable under tension.");
  }

  // Read current joint angle
  bool isValid;
  float current_joint_angle = getCurrentAngle(dof_index, isValid);
  if (!isValid) {
    LOG_ERROR("Cannot recalculate offsets: invalid encoder reading for DOF " +
              String(dof_index));
    stopDofMotors(dof_index);
    return false;
  }

  // Calculate expected motor angles using linear equations (joint->motor)
  float expected_agonist_angle, expected_antagonist_angle;
  bool equations_available =
      calculateMotorAnglesWithEquations(dof_index, current_joint_angle, current_joint_angle,
                                        expected_agonist_angle, expected_antagonist_angle);

  if (!equations_available) {
    LOG_ERROR("Unable to recalculate offsets - linear equations not available for DOF " +
              String(dof_index));
    LOG_WARN("Required: perform auto-mapping to compute linear equations first");
    LOG_WARN("Alternatively: load linear equations from flash with CMD:LOAD_LINEAR_EQUATIONS");
    stopDofMotors(dof_index);
    return false;
  }

  LOG_INFO("Using linear equations to compute expected angles");

  LOG_DEBUG(String("Current joint angle: ") + String(current_joint_angle));
  LOG_DEBUG(String("Expected agonist angle: ") + String(expected_agonist_angle));
  LOG_DEBUG(String("Expected antagonist angle: ") + String(expected_antagonist_angle));

  // Read current motor angles (without applying offset)
  float current_agonist_angle    = agonist_motor->getMultiAngleSync(false).angle;
  float current_antagonist_angle = antagonist_motor->getMultiAngleSync(false).angle;

  LOG_DEBUG(String("Current agonist motor angle (raw): ") + String(current_agonist_angle));
  LOG_DEBUG(String("Current antagonist motor angle (raw): ") + String(current_antagonist_angle));

  // Calculate new offsets
  // CORRECT: The offset is the difference between expected and current angle, with inverted sign
  // because the motor SUBTRACTS the offset instead of adding it
  float new_agonist_offset = current_agonist_angle - expected_agonist_angle; // Inverted sign
  float new_antagonist_offset =
      current_antagonist_angle - expected_antagonist_angle; // Inverted sign

  LOG_DEBUG(String("New agonist offset: ") + String(new_agonist_offset));
  LOG_DEBUG(String("New antagonist offset: ") + String(new_antagonist_offset));

  // Set new offsets
  agonist_motor->setOffsetEncoder(new_agonist_offset);
  antagonist_motor->setOffsetEncoder(new_antagonist_offset);

  // Verify that offsets were applied correctly
  float verified_agonist_angle    = agonist_motor->getMultiAngleSync().angle;
  float verified_antagonist_angle = antagonist_motor->getMultiAngleSync().angle;
  LOG_DEBUG(String("Verified agonist angle: ") + String(verified_agonist_angle));
  LOG_DEBUG(String("Verified antagonist angle: ") + String(verified_antagonist_angle));

  // Calculate residual error
  float agonist_error    = fabs(verified_agonist_angle - expected_agonist_angle);
  float antagonist_error = fabs(verified_antagonist_angle - expected_antagonist_angle);

  // MODIFIED: Variable error threshold depending on DOF
  // Use a higher threshold for inversion/eversion (DOF 1) due to greater elasticity
  float ERROR_THRESHOLD = 2.0f; // Standard threshold for other DOFs

  // NOW stop motors after verification
  stopDofMotors(dof_index);

  if (agonist_error > ERROR_THRESHOLD || antagonist_error > ERROR_THRESHOLD) {
    LOG_WARN("Residual error after offset calibration");
    LOG_DEBUG(String("Agonist error: ") + String(agonist_error));
    LOG_DEBUG(String("Antagonist error: ") + String(antagonist_error));
    LOG_DEBUG(String("Allowed error threshold: ") + String(ERROR_THRESHOLD) + "°");

    // Advanced error analysis
    LOG_DEBUG("=== DETAILED ERROR ANALYSIS ===");
    LOG_DEBUG(String("Joint type: ") + String(config.name));
    LOG_DEBUG(String("DOF: ") + String(config.dofs[dof_index].name) + " (index " +
              String(dof_index) + ")");
    LOG_DEBUG(String("Encoder inverted: ") +
             String(config.dofs[dof_index].encoder_invert ? "YES" : "NO"));
    LOG_DEBUG(String("Encoder channel: ") + String(config.dofs[dof_index].encoder_channel));
    LOG_DEBUG(String("Angle limits: ") + String(config.dofs[dof_index].limits.min_angle) +
              "° / " + String(config.dofs[dof_index].limits.max_angle) + "°");
    LOG_DEBUG(String("Current joint angle: ") + String(current_joint_angle) + "°");
    LOG_DEBUG(String("Agonist error %: ") +
              String((agonist_error / fabs(expected_agonist_angle)) * 100, 1) + "%");
    LOG_DEBUG(String("Antagonist error %: ") +
              String((antagonist_error / fabs(expected_antagonist_angle)) * 100, 1) + "%");
    LOG_DEBUG("====================================");

    return false;
  }

  DBG_PRINTLN("Offsets successfully recalculated for DOF " + String(dof_index));
  DBG_PRINT("New agonist offset: ");
  DBG_PRINTLN(new_agonist_offset);
  DBG_PRINT("New antagonist offset: ");
  DBG_PRINTLN(new_antagonist_offset);
  DBG_PRINTLN("Residual agonist error: " + String(agonist_error) +
              "° (limit: " + String(ERROR_THRESHOLD) + "°)");
  DBG_PRINTLN("Residual antagonist error: " + String(antagonist_error) +
              "° (limit: " + String(ERROR_THRESHOLD) + "°)");

  // Add calibration details
  DBG_PRINTLN("\nCalibration details:");
  DBG_PRINTLN("AGONIST:");
  DBG_PRINT("  Current angle (raw): ");
  DBG_PRINTLN(current_agonist_angle);
  DBG_PRINT("  Expected angle: ");
  DBG_PRINTLN(expected_agonist_angle);
  DBG_PRINT("  Angle after calibration: ");
  DBG_PRINTLN(verified_agonist_angle);
  DBG_PRINT("  Final deviation: ");
  DBG_PRINTLN(agonist_error);

  DBG_PRINTLN("ANTAGONIST:");
  DBG_PRINT("  Current angle (raw): ");
  DBG_PRINTLN(current_antagonist_angle);
  DBG_PRINT("  Expected angle: ");
  DBG_PRINTLN(expected_antagonist_angle);
  DBG_PRINT("  Angle after calibration: ");
  DBG_PRINTLN(verified_antagonist_angle);
  DBG_PRINT("  Final deviation: ");
  DBG_PRINTLN(antagonist_error);

  DBG_PRINT("\nJoint angle: ");
  DBG_PRINTLN(current_joint_angle);

  // Print tension and stability information
  DBG_PRINTLN("\nTension and stability info:");
  DBG_PRINT("  Initial agonist displacement: ");
  DBG_PRINTLN(agonist_displacement);
  DBG_PRINT("  Initial antagonist displacement: ");
  DBG_PRINTLN(antagonist_displacement);
  DBG_PRINT("  Agonist stability: ");
  DBG_PRINTLN(stability_agonist);
  DBG_PRINT("  Antagonist stability: ");
  DBG_PRINTLN(stability_antagonist);
  DBG_PRINT("  Tendon tension: ");
  DBG_PRINTLN(tension_ok ? "OK" : "NOT OPTIMAL");
  DBG_PRINT("  System stability: ");
  DBG_PRINTLN(is_stable ? "OK" : "NOT OPTIMAL");

  // Information about method used
  DBG_PRINTLN("\nCalculation method info:");
  DBG_PRINTLN("  Method used: LINEAR EQUATIONS");
  DBG_PRINTLN("  Direct computation without interpolation");

  // DEBUG: Verify elastic effect after tension release
  sleep_ms(300); // Wait for system to stabilize without tension

  // Read angles again without tension
  float post_release_agonist_angle    = agonist_motor->getMultiAngleSync().angle;
  float post_release_antagonist_angle = antagonist_motor->getMultiAngleSync().angle;

  DBG_PRINTLN("\n=== TENSION RELEASE EFFECT ===");
  DBG_PRINTLN("Agonist angle under tension: " + String(verified_agonist_angle) + "°");
  DBG_PRINTLN("Agonist angle after release: " + String(post_release_agonist_angle) + "°");
  DBG_PRINTLN(
      "Difference: " + String(fabs(post_release_agonist_angle - verified_agonist_angle), 2) + "°");

  DBG_PRINTLN("Antagonist angle under tension: " + String(verified_antagonist_angle) + "°");
  DBG_PRINTLN("Antagonist angle after release: " + String(post_release_antagonist_angle) + "°");
  DBG_PRINTLN("Difference: " +
              String(fabs(post_release_antagonist_angle - verified_antagonist_angle), 2) + "°");
  DBG_PRINTLN("==================================");

  // Set flag that offsets have been calibrated for this DOF
  motor_offsets_calibrated[dof_index] = true;
  Serial.println("CALIBRATION_STATUS: Offsets calibrated for DOF " + String(dof_index) +
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
    Serial.println("Invalid parameters in getPid");
    return false;
  }

  // Find corresponding motor (1=agonist, 2=antagonist)
  int motor_index = -1;
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      // The first motor found for this DOF is the agonist (1), the second is the antagonist (2)
      if (motor_type == 1 && motor_index == -1) {
        motor_index = i;
        break;
      } else if (motor_type == 2 && motor_index != -1) {
        motor_index = i;
        break;
      } else if (motor_index == -1) {
        motor_index = i;
      }
    }
  }

  if (motor_index == -1) {
    LOG_ERROR("Motor not found in getPid");
    return false;
  }

  // Get PID parameters from controller
  if (pid_controllers[motor_index] != nullptr) {
    kp  = pid_controllers[motor_index]->getKp();
    ki  = pid_controllers[motor_index]->getKi();
    kd  = pid_controllers[motor_index]->getKd();
    tau = pid_controllers[motor_index]->getTau();

    LOG_INFO("PID parameters fetched for DOF " + String(dof_index) + ", motor " +
             String(motor_type) + ":");
    LOG_INFO("Kp: " + String(kp, 4) + ", Ki: " + String(ki, 4) + ", Kd: " + String(kd, 4) +
             ", Tau: " + String(tau, 4));

    return true;
  } else {
    LOG_ERROR("PID controller not initialized");
    return false;
  }
}

// Implementation of setPid
bool JointController::setPid(uint8_t dof_index, uint8_t motor_type, float kp, float ki, float kd,
                             float tau) {
  // Validate parameters
  if (dof_index >= config.dof_count || (motor_type != 1 && motor_type != 2)) {
    LOG_ERROR("Invalid parameters in setPid");
    return false;
  }

  // Find corresponding motor (1=agonist, 2=antagonist)
  int motor_index = -1;
  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      // The first motor found for this DOF is the agonist (1), the second is the antagonist (2)
      if (motor_type == 1 && motor_index == -1) {
        motor_index = i;
        break;
      } else if (motor_type == 2 && motor_index != -1) {
        motor_index = i;
        break;
      } else if (motor_index == -1) {
        motor_index = i;
      }
    }
  }

  if (motor_index == -1) {
    LOG_ERROR("Motor not found in setPid");
    return false;
  }

  // Set PID parameters
  if (pid_controllers[motor_index] != nullptr) {
    pid_controllers[motor_index]->setTunings(kp, ki, kd, tau);

    LOG_INFO("PID parameters set for DOF " + String(dof_index) + ", motor " + String(motor_type) +
             ":");
    LOG_INFO("Kp: " + String(kp, 4) + ", Ki: " + String(ki, 4) + ", Kd: " + String(kd, 4) +
             ", Tau: " + String(tau, 4));

    return true;
  } else {
    LOG_ERROR("PID controller not initialized");
    return false;
  }
}

bool JointController::setOuterLoopParameters(uint8_t dof_index, float kp, float ki, float kd,
                                             float stiffness_deg, float cascade_influence) {
  if (dof_index >= config.dof_count) {
    LOG_ERROR("Invalid parameters in setOuterLoopParameters (DOF out of range)");
    return false;
  }

  if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
      !std::isfinite(stiffness_deg) || !std::isfinite(cascade_influence)) {
    Serial.println("NaN/Inf parameters in setOuterLoopParameters");
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

  Serial.println("Outer loop parameters updated for DOF " + String(dof_index));
  Serial.println("  Kp=" + String(kp, 4) + ", Ki=" + String(ki, 4) + ", Kd=" + String(kd, 4));
  Serial.println("  Stiffness=" + String(stiffness_deg, 4) +
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

