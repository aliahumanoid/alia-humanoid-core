/**
 * @file JointController_Movement.cpp
 * @brief Cascade movement control implementation for JointController
 * 
 * This file contains the implementation of the cascade PID control system for
 * coordinated multi-DOF movements with:
 * - Outer loop (100 Hz): Joint-level PID control
 * - Inner loop (500 Hz): Motor-level PID control
 * - Smooth transitions between movements
 * - Position holding with continuous control
 * - Emergency stop handling
 * 
 * This method is part of the JointController class but separated here
 * for better code organization and maintainability.
 */

#include <JointController.h>
#include <commands.h>
#include <debug.h>
#include <global.h>
#include <path.h>
#include <shared_data.h>
#include <utils.h>
#include <algorithm>
#include <array>
#include <cmath>

// External variables for inter-core communication
extern volatile bool emergency_stop_requested;
extern volatile bool buffer_ready[2];
extern volatile int active_buffer;
extern volatile uint8_t pending_command_type;

// External variables for movement sample logging
extern queue_t movement_sample_queue;
extern volatile bool movement_sample_stream_active;
extern volatile bool movement_sample_stream_done;
extern volatile uint8_t movement_sample_joint_id;
extern volatile bool movement_sample_overflow;
extern uint16_t movement_sample_counters[MAX_DOFS];

// ============================================================================
// MOVEMENT LOGGING HELPERS
// ============================================================================

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

// RAII guard for movement logging
class MovementLogGuard {
public:
  explicit MovementLogGuard(const JointConfig &cfg) {
    startMovementLogging(cfg);
  }
  ~MovementLogGuard() {
    stopMovementLogging();
  }
};

// ============================================================================
// CASCADE MOVEMENT CONTROL
// ============================================================================

// Coordinated movement of multiple DOFs simultaneously
MovementResult JointController::moveMultiDOF_cascade(float *target_angles, uint8_t active_dofs_mask,
                                                     int path_type, int sync_strategy,
                                                     float max_speed, float accel_time, int steps,
                                                     uint64_t sampling_period, bool verbose,
                                                     int max_torque, bool smooth_transition) {

  MovementExitCode exit_code = MOVEMENT_COMPLETED;

  // === SMOOTH TRANSITION HANDLING ===
  if (smooth_transition && verbose) {
    LOG_INFO("=== SMOOTH TRANSITION ACTIVE ===");
    LOG_INFO("Motors already active, continuing control without interruption");
  }

  // === LOCAL COPY OF TARGETS ===
  // IMPORTANT: Create a local copy to prevent external modifications during
  // execution (e.g., new command) from causing abrupt jumps in control
  float local_target_angles[MAX_DOFS];
  for (int i = 0; i < MAX_DOFS; i++) {
    local_target_angles[i] = target_angles[i];
  }
  // From here on, use local_target_angles instead of target_angles

  // === PARAMETER VALIDATION AND INITIALIZATION ===

  // Count active DOFs
  int active_dof_count = 0;
  int active_dof_indices[MAX_DOFS];

  for (int i = 0; i < config.dof_count; i++) {
    if (active_dofs_mask & (1 << i)) {
      active_dof_indices[active_dof_count] = i;
      active_dof_count++;
    }
  }

  if (active_dof_count == 0) {
    LOG_ERROR("No DOF selected for movement");
    return MovementResult(MOVEMENT_ERROR, "Error: No DOF selected for movement\n");
  }

  MovementLogGuard log_guard(config);

  // Retrieve outer loop parameters for each active DOF
  float outer_kp_per_dof[MAX_DOFS]  = {0};
  float outer_ki_per_dof[MAX_DOFS]  = {0};
  float outer_kd_per_dof[MAX_DOFS]  = {0};
  float stiffness_per_dof[MAX_DOFS] = {0};
  float cascade_per_dof[MAX_DOFS]   = {0};

  for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];

    float kp_tmp, ki_tmp, kd_tmp, stiffness_tmp, cascade_tmp;
    if (!getOuterLoopParameters(dof_idx, kp_tmp, ki_tmp, kd_tmp, stiffness_tmp, cascade_tmp)) {
      kp_tmp        = DEFAULT_OUTER_LOOP_KP;
      ki_tmp        = DEFAULT_OUTER_LOOP_KI;
      kd_tmp        = DEFAULT_OUTER_LOOP_KD;
      stiffness_tmp = DEFAULT_STIFFNESS_REF_DEG;
      cascade_tmp   = DEFAULT_CASCADE_INFLUENCE;
    }

    if (!std::isfinite(kp_tmp) || kp_tmp <= 0.0f)
      kp_tmp = DEFAULT_OUTER_LOOP_KP;
    if (!std::isfinite(ki_tmp) || ki_tmp < 0.0f)
      ki_tmp = DEFAULT_OUTER_LOOP_KI;
    if (!std::isfinite(kd_tmp) || kd_tmp < 0.0f)
      kd_tmp = DEFAULT_OUTER_LOOP_KD;
    if (!std::isfinite(stiffness_tmp) || stiffness_tmp < 0.0f)
      stiffness_tmp = DEFAULT_STIFFNESS_REF_DEG;
    if (!std::isfinite(cascade_tmp))
      cascade_tmp = DEFAULT_CASCADE_INFLUENCE;

    cascade_tmp = std::clamp(cascade_tmp, 0.0f, 1.0f);

    outer_kp_per_dof[dof_idx]  = kp_tmp;
    outer_ki_per_dof[dof_idx]  = ki_tmp;
    outer_kd_per_dof[dof_idx]  = kd_tmp;
    stiffness_per_dof[dof_idx] = stiffness_tmp;
    cascade_per_dof[dof_idx]   = cascade_tmp;
  }

  if (verbose) {
    LOG_INFO("=== MULTI-DOF CASCADE MOTION ===");
    LOG_INFO("Double-loop cascade control active");
    LOG_INFO("Outer loop: 100 Hz (joint PID)");
    LOG_INFO("Inner loop: 500 Hz (motor PID)");
    LOG_INFO("Active DOF: " + String(active_dof_count));
    for (int i = 0; i < active_dof_count; i++) {
      int dof_idx = active_dof_indices[i];
      String gains = "DOF " + String(dof_idx) + ": Kp=" +
                     String(outer_kp_per_dof[dof_idx], 3) + ", Ki=" +
                     String(outer_ki_per_dof[dof_idx], 3) + ", Kd=" +
                     String(outer_kd_per_dof[dof_idx], 3);
      LOG_INFO(gains);
      String cascade_msg = "  Stiffness ref: " + String(stiffness_per_dof[dof_idx], 3) +
                           " deg | Cascade influence: " +
                           String(cascade_per_dof[dof_idx] * 100.0f, 1) + "% ";
      if (cascade_per_dof[dof_idx] < 0.01f) {
        cascade_msg += "(equivalent to moveMultiDOF)";
      } else if (cascade_per_dof[dof_idx] > 0.99f) {
        cascade_msg += "(full cascade)";
      } else {
        cascade_msg += "(mixed control)";
      }
      LOG_INFO(cascade_msg);
    }
    LOG_INFO("Note: Using per-motor trajectories with dynamic theta_0(t)");
    LOG_INFO("==================================");
  }

  // === VARIABLES FOR CASCADE CONTROL ===

  // Variables for outer loop (joint control)
  float q_curr[MAX_DOFS]           = {0}; // Measured joint angle
  float q_des[MAX_DOFS]            = {0}; // Desired joint angle (from trajectory)
  float error_integral_q[MAX_DOFS] = {0}; // Outer PID integral error
  float previous_error_q[MAX_DOFS] = {0}; // Previous error for derivative
  float delta_theta[MAX_DOFS]      = {0}; // Outer PID output

  // References for motors (outer loop output)
  float theta_A_ref[MAX_DOFS] = {0};
  float theta_B_ref[MAX_DOFS] = {0};

  // Loop frequencies
  // Compute dynamic divider to keep outer loop at ~100 Hz
  // sampling_period in μs: 2000μs→div=5, 3000μs→div=3, 5000μs→div=2
  const int OUTER_LOOP_DIV = max(1, (int)(10000 / sampling_period)); // Target: ~100Hz

  // === VERIFY EQUATIONS AND LIMITS ===

  for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];

    if (!hasValidEquations(dof_idx)) {
      LOG_ERROR("Linear equations or limits unavailable for DOF " + String(dof_idx));
      return MovementResult(MOVEMENT_ERROR,
                            "Error: Linear equations or limits unavailable for DOF " +
                                String(dof_idx) + "\n");
    }

    if (verbose) {
      LOG_DEBUG("NOTE: DOF " + String(dof_idx) + " controlled through stored linear equations");
    }

    // Check angular limits
    if (!isAngleInLimits(dof_idx, local_target_angles[dof_idx])) {
      String error_msg = "Error: Angle outside limits for DOF " + String(dof_idx) + ": ";
      error_msg += String(local_target_angles[dof_idx]) +
                   " (min: " + String(config.dofs[dof_idx].limits.min_angle);
      error_msg += ", max: " + String(config.dofs[dof_idx].limits.max_angle) + ")\n";
      Serial.print(error_msg);
      return MovementResult(MOVEMENT_ERROR, error_msg);
    }

    // Check safety limits computed by equations
    if (!isAngleInMappingLimits(dof_idx, local_target_angles[dof_idx])) {
      String error_msg = "Error: Angle outside safety limits (equations) for DOF " +
                         String(dof_idx) + ": ";
      error_msg += String(local_target_angles[dof_idx]) + "°\n";
      Serial.print(error_msg);
      return MovementResult(MOVEMENT_ERROR, error_msg);
    }
  }

  // === FIND MOTORS ===

  struct MotorPair {
    LKM_Motor *agonist;
    LKM_Motor *antagonist;
    int agonist_index;
    int antagonist_index;
    PID *pid_agonist;
    PID *pid_antagonist;
  };

  MotorPair motor_pairs[MAX_DOFS];

  // Initialize motor pairs for each active DOF
  for (int i = 0; i < active_dof_count; i++) {
    int dof_idx    = active_dof_indices[i];
    motor_pairs[i] = {nullptr, nullptr, -1, -1, nullptr, nullptr};

    // Find motors for this DOF
    for (int j = 0; j < config.motor_count; j++) {
      if (config.motors[j].dof_index == dof_idx) {
        if (config.motors[j].is_agonist) {
          motor_pairs[i].agonist       = motors[j];
          motor_pairs[i].agonist_index = j;
          motor_pairs[i].pid_agonist   = pid_controllers[j];
        } else {
          motor_pairs[i].antagonist       = motors[j];
          motor_pairs[i].antagonist_index = j;
          motor_pairs[i].pid_antagonist   = pid_controllers[j];
        }
      }
    }

    if (motor_pairs[i].agonist == nullptr || motor_pairs[i].antagonist == nullptr) {
      LOG_ERROR("Motors not found for DOF " + String(dof_idx));
      return MovementResult(MOVEMENT_ERROR,
                            "Error: Motors not found for DOF " + String(dof_idx) + "\n");
    }
  }

  // === COMPUTE START ANGLES USING LINEAR EQUATIONS (LIKE moveMultiDOF) ===

  float start_angles[MAX_DOFS];
  float agonist_start_angles[MAX_DOFS];
  float antagonist_start_angles[MAX_DOFS];
  float max_movement_time = 0.0f;

  for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];
    bool isValid;
    start_angles[dof_idx] = getCurrentAngle(dof_idx, isValid);

    if (!isValid) {
      LOG_ERROR("Invalid joint encoder reading for DOF " + String(dof_idx));
      return MovementResult(MOVEMENT_ERROR,
                            "Error: Invalid joint encoder reading for DOF " + String(dof_idx) +
                                "\n");
    }

    // Read current motor angles
    float gamma_curr = motor_pairs[i].agonist->getMultiAngleSync().angle;
    float theta_curr = motor_pairs[i].antagonist->getMultiAngleSync().angle;

    if (verbose) {
      LOG_DEBUG("DOF " + String(dof_idx) + " current motor angles:");
      LOG_DEBUG("  Agonist: " + String(gamma_curr, 6) + "°");
      LOG_DEBUG("  Antagonist: " + String(theta_curr, 6) + "°");
    }

    // Compute starting angles using inverse linear equations (motor -> joint)
    bool equations_available = calculateJointAnglesWithEquations(dof_idx, gamma_curr, theta_curr,
                                                                 agonist_start_angles[dof_idx],
                                                                 antagonist_start_angles[dof_idx]);

    if (!equations_available) {
      LOG_ERROR("Linear equations not available for DOF " + String(dof_idx) +
                " — required for cascade control");
      return MovementResult(MOVEMENT_ERROR, "Error: Linear equations not available for DOF " +
                                                String(dof_idx) +
                                                " — required for cascade control\n");
    }

    if (verbose) {
      LOG_DEBUG("Joint angles computed from motor positions:");
      LOG_DEBUG("  From agonist: " + String(agonist_start_angles[dof_idx], 6) + "°");
      LOG_DEBUG("  From antagonist: " + String(antagonist_start_angles[dof_idx], 6) + "°");
      LOG_DEBUG("  Joint encoder: " + String(start_angles[dof_idx], 6) + "°");
    }

    // Verify consistency of starting angles
    float angle_diff = fabs(agonist_start_angles[dof_idx] - antagonist_start_angles[dof_idx]);
    if (angle_diff > MAX_ANGLE_DIFFERENCE) {
      String error_msg = "Error: Difference between start angles too high for DOF " +
                         String(dof_idx) + ": " + String(angle_diff, 6) + "°\n";
      Serial.print(error_msg);
      return MovementResult(MOVEMENT_ERROR, error_msg);
    }

    // Compute movement time for synchronization
    if (sync_strategy == 1) {
      float angle_diff     = fabs(local_target_angles[dof_idx] - start_angles[dof_idx]);
      float dof_max_speed  = (max_speed > 0) ? max_speed : config.dofs[dof_idx].motion.max_speed;
      float estimated_time = angle_diff / (dof_max_speed * 180.0f / PI);
      max_movement_time    = max(max_movement_time, estimated_time);
    }
  }

  // Set default parameters if unspecified
  for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];

    if (max_speed <= 0) {
      max_speed = config.dofs[dof_idx].motion.max_speed;
    }
    if (accel_time <= 0) {
      accel_time = config.dofs[dof_idx].motion.accel_time;
    }
    if (steps <= 0) {
      steps = config.dofs[dof_idx].motion.path_steps;
    }
  }

  // === GENERATE SEPARATE TRAJECTORIES FOR AGONIST AND ANTAGONIST ===

  std::array<float, MAX_STEPS> time_arrays_agonist[MAX_DOFS];
  std::array<float, MAX_STEPS> alpha_path_arrays_agonist[MAX_DOFS];
  std::array<float, MAX_STEPS> velocity_arrays_agonist[MAX_DOFS];
  std::array<float, MAX_STEPS> accel_arrays_agonist[MAX_DOFS];

  std::array<float, MAX_STEPS> time_arrays_antagonist[MAX_DOFS];
  std::array<float, MAX_STEPS> alpha_path_arrays_antagonist[MAX_DOFS];
  std::array<float, MAX_STEPS> velocity_arrays_antagonist[MAX_DOFS];
  std::array<float, MAX_STEPS> accel_arrays_antagonist[MAX_DOFS];

  float common_stop_time = 0.0f;

  for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];

    // Compute DOF-specific parameters
    float dof_max_speed  = max_speed;
    float dof_accel_time = accel_time;

    if (sync_strategy == 1 && max_movement_time > 0) {
      float angle_diff = fabs(local_target_angles[dof_idx] - start_angles[dof_idx]);
      if (angle_diff > 0.1f) {
        dof_max_speed = (angle_diff * PI / 180.0f) / max_movement_time;
      }
    }

    // Generate SEPARATE trajectories for agonist and antagonist
    if (path_type == PATH_LINEAR) {
      path_linear(radians(agonist_start_angles[dof_idx]), radians(local_target_angles[dof_idx]),
                  dof_max_speed, dof_accel_time, steps, time_arrays_agonist[dof_idx],
                  alpha_path_arrays_agonist[dof_idx], velocity_arrays_agonist[dof_idx],
                  accel_arrays_agonist[dof_idx]);

      path_linear(radians(antagonist_start_angles[dof_idx]), radians(local_target_angles[dof_idx]),
                  dof_max_speed, dof_accel_time, steps, time_arrays_antagonist[dof_idx],
                  alpha_path_arrays_antagonist[dof_idx], velocity_arrays_antagonist[dof_idx],
                  accel_arrays_antagonist[dof_idx]);
    } else if (path_type == PATH_TRIG) {
      path_trig(radians(agonist_start_angles[dof_idx]), radians(local_target_angles[dof_idx]),
                dof_max_speed, dof_accel_time, steps, time_arrays_agonist[dof_idx],
                alpha_path_arrays_agonist[dof_idx], velocity_arrays_agonist[dof_idx],
                accel_arrays_agonist[dof_idx]);

      path_trig(radians(antagonist_start_angles[dof_idx]), radians(local_target_angles[dof_idx]),
                dof_max_speed, dof_accel_time, steps, time_arrays_antagonist[dof_idx],
                alpha_path_arrays_antagonist[dof_idx], velocity_arrays_antagonist[dof_idx],
                accel_arrays_antagonist[dof_idx]);
    } else if (path_type == PATH_QUAD) {
      path_quad(radians(agonist_start_angles[dof_idx]), radians(local_target_angles[dof_idx]),
                dof_max_speed, dof_accel_time, steps, time_arrays_agonist[dof_idx],
                alpha_path_arrays_agonist[dof_idx], velocity_arrays_agonist[dof_idx],
                accel_arrays_agonist[dof_idx]);

      path_quad(radians(antagonist_start_angles[dof_idx]), radians(local_target_angles[dof_idx]),
                dof_max_speed, dof_accel_time, steps, time_arrays_antagonist[dof_idx],
                alpha_path_arrays_antagonist[dof_idx], velocity_arrays_antagonist[dof_idx],
                accel_arrays_antagonist[dof_idx]);
    } else {
      LOG_ERROR("Invalid path type");
      return MovementResult(MOVEMENT_ERROR, "Error: Invalid path type\n");
    }

    common_stop_time = max(common_stop_time, time_arrays_agonist[dof_idx][steps - 1]);
    common_stop_time = max(common_stop_time, time_arrays_antagonist[dof_idx][steps - 1]);

    if (verbose) {
      LOG_INFO("DOF " + String(dof_idx) + " - Generated trajectories:");
      LOG_INFO("  Agonist: from " + String(agonist_start_angles[dof_idx], 2) + "° to " +
               String(local_target_angles[dof_idx], 2) + "°");
      LOG_INFO("  Antagonist: from " + String(antagonist_start_angles[dof_idx], 2) + "° to " +
               String(local_target_angles[dof_idx], 2) + "°");
    }
  }

  // === RESET PID ===

  // Reset PID only if NOT a smooth transition
  if (!smooth_transition) {
    // Reset all motor PID controllers
    for (int i = 0; i < active_dof_count; i++) {
      motor_pairs[i].pid_agonist->reset();
      motor_pairs[i].pid_antagonist->reset();
    }

    // Reset outer PID variables
    for (int i = 0; i < config.dof_count; i++) {
      error_integral_q[i] = 0.0f;
      previous_error_q[i] = 0.0f;
      delta_theta[i]      = 0.0f;
    }

    if (verbose) {
      LOG_INFO("PID reset (new movement sequence)");
    }
  } else {
    if (verbose) {
      LOG_INFO("PID preserved (smooth transition)");
    }
  }

  // === VARIABLES FOR CONTROL LOOP ===

  uint64_t tstart    = time_us_64();
  uint64_t next_time = tstart;
  int cycle_count    = 0;

  float outer_loop_dt =
      OUTER_LOOP_DIV * sampling_period / 1000000.0f;  // Time in seconds for outer PID
  float inner_loop_dt = sampling_period / 1000000.0f; // Time in seconds for inner PID

  if (verbose) {
    Serial.println("Starting cascade control");
    Serial.println("Sampling period: " + String((unsigned long)sampling_period) + " μs (" +
                   String(sampling_period / 1000.0f, 1) + " ms)");
    Serial.println("Inner loop frequency: " + String(1.0f / inner_loop_dt, 1) + " Hz");
    Serial.println("Outer loop frequency: " + String(1.0f / outer_loop_dt, 1) + " Hz (every " +
                   String(OUTER_LOOP_DIV) + " cycles)");
    Serial.println("Estimated duration: " + String(common_stop_time, 2) + " s\n");
  }

  // === MAIN CONTROL LOOP ===

  while (true) {
    next_time += sampling_period;
    cycle_count++;

    // Compute current time
    float tcurr = (time_us_64() - tstart) / 1000000.0f;

    if (tcurr <= common_stop_time) {

      // === OUTER LOOP (100 Hz) - JOINT CONTROL ===
      if ((cycle_count - 1) % OUTER_LOOP_DIV == 0) {

        for (int i = 0; i < active_dof_count; i++) {
          int dof_idx = active_dof_indices[i];

          // Read current joint angle
          bool isValid;
          q_curr[dof_idx] = getCurrentAngle(dof_idx, isValid);

          if (!isValid) {
            stopAllMotors();
            Serial.println("ERROR: Invalid joint encoder reading for DOF " + String(dof_idx));
            return MovementResult(MOVEMENT_ERROR,
                                  "Error: Invalid joint encoder reading for DOF " +
                                      String(dof_idx) + "\n");
          }

          // Interpolate desired angle as AVERAGE of agonist and antagonist trajectories
          // (joint angle - alpha_path). Both converge to the same target, so
          // the average is more accurate
          float q_des_agonist = interpolate_data(tcurr, time_arrays_agonist[dof_idx].data(),
                                                  alpha_path_arrays_agonist[dof_idx].data(), steps);
          float q_des_antagonist =
              interpolate_data(tcurr, time_arrays_antagonist[dof_idx].data(),
                                alpha_path_arrays_antagonist[dof_idx].data(), steps);
          q_des[dof_idx] =
              degrees((q_des_agonist + q_des_antagonist) / 2.0f); // Average and convert to degrees

          // Compute error
          float error = q_des[dof_idx] - q_curr[dof_idx];

          String safety_message;
          if (!checkSafetyForDof(dof_idx, q_curr[dof_idx], safety_message, false)) {
            stopAllMotors();
            Serial.println("SAFETY ERROR: " + safety_message);
            return MovementResult(MOVEMENT_ERROR, "SAFETY ERROR: " + safety_message + "\n");
          }

          // Outer PID to compute delta_theta
          error_integral_q[dof_idx] += error * outer_loop_dt;
          float error_derivative = (error - previous_error_q[dof_idx]) / outer_loop_dt;

          // Limit integral to avoid windup. Windup is the phenomenon where the integral
          // continues to grow without limits, causing overshoot and underdamping.
          const float MAX_INTEGRAL = 10.0f; // Degrees
          error_integral_q[dof_idx] =
              constrain(error_integral_q[dof_idx], -MAX_INTEGRAL, MAX_INTEGRAL);

          // Compute delta_theta
          delta_theta[dof_idx] = outer_kp_per_dof[dof_idx] * error +
                                 outer_ki_per_dof[dof_idx] * error_integral_q[dof_idx] +
                                 outer_kd_per_dof[dof_idx] * error_derivative;

          // Limit delta_theta for safety
          const float MAX_DELTA_THETA = 30.0f; // Maximum correction degrees
          delta_theta[dof_idx] = constrain(delta_theta[dof_idx], -MAX_DELTA_THETA, MAX_DELTA_THETA);

          previous_error_q[dof_idx] = error;

          if (verbose && cycle_count % 500 == 0) { // Log every ~1 second
            Serial.println("Outer loop DOF " + String(dof_idx) + ":");
            Serial.println("  q_des=" + String(q_des[dof_idx], 2) +
                           "° (avg: ago=" + String(degrees(q_des_agonist), 2) +
                           "°, ant=" + String(degrees(q_des_antagonist), 2) + "°)");
            Serial.println("  q_curr=" + String(q_curr[dof_idx], 2) + "°");
            Serial.println("  error=" + String(error, 2) +
                           "°, delta_theta=" + String(delta_theta[dof_idx], 2) + "°");
          }
        }
      }

      // === INNER LOOP (500 Hz) - MOTOR CONTROL ===

      for (int i = 0; i < active_dof_count; i++) {
        int dof_idx = active_dof_indices[i];

        // Interpolate theta_0 from trajectories (changes over time!)
        float theta_0_agonist_t =
            interpolate_data(tcurr, time_arrays_agonist[dof_idx].data(),
                              alpha_path_arrays_agonist[dof_idx].data(), steps);
        theta_0_agonist_t = degrees(theta_0_agonist_t); // Convert to degrees

        float theta_0_antagonist_t =
            interpolate_data(tcurr, time_arrays_antagonist[dof_idx].data(),
                              alpha_path_arrays_antagonist[dof_idx].data(), steps);
        theta_0_antagonist_t = degrees(theta_0_antagonist_t); // Convert to degrees

        // Compute target motor angles using linear equations
        float theta_0_agonist_motor, theta_0_antagonist_motor;
        bool equations_ok =
            calculateMotorAnglesWithEquations(dof_idx, theta_0_agonist_t, theta_0_antagonist_t,
                                              theta_0_agonist_motor, theta_0_antagonist_motor);

        if (!equations_ok) {
          stopAllMotors();
          Serial.println(
              "Error: Linear equations unavailable during cascade control for DOF " +
              String(dof_idx));
          return MovementResult(
              MOVEMENT_ERROR,
              "Error: Linear equations unavailable during cascade control for DOF " +
                  String(dof_idx) + "\n");
        }

        // Compute motor references using cascade control formula with theta_0(t)
        // Apply cascade_influence parameter to scale the cascade control effect
        float cascade_value   = cascade_per_dof[dof_idx];
        float stiffness_value = stiffness_per_dof[dof_idx];

        theta_A_ref[dof_idx] =
            theta_0_agonist_motor +
            cascade_value * (0.5f * delta_theta[dof_idx] + 0.5f * stiffness_value);
        theta_B_ref[dof_idx] =
            theta_0_antagonist_motor +
            cascade_value * (0.5f * delta_theta[dof_idx] - 0.5f * stiffness_value);

        // Read current motor angles
        float theta_A_curr = motor_pairs[i].agonist->getMultiAngleSync().angle;
        float theta_B_curr = motor_pairs[i].antagonist->getMultiAngleSync().angle;

        // Inner PID for motors
        float error_A = theta_A_ref[dof_idx] - theta_A_curr;
        float error_B = theta_B_ref[dof_idx] - theta_B_curr;

        float command_A = motor_pairs[i].pid_agonist->control(theta_A_ref[dof_idx], theta_A_curr);
        float command_B =
            motor_pairs[i].pid_antagonist->control(theta_B_ref[dof_idx], theta_B_curr);

        // Apply torque limits
        command_A = constrain(command_A, -max_torque, max_torque);
        command_B = constrain(command_B, -max_torque, max_torque);

        // Send commands to motors
        motor_pairs[i].agonist->setTorque((int)command_A);
        motor_pairs[i].antagonist->setTorque((int)command_B);

        if ((cycle_count - 1) % OUTER_LOOP_DIV == 0) {
          logMovementSample(dof_idx, q_des[dof_idx], q_curr[dof_idx], theta_A_curr, theta_B_curr,
                            theta_A_ref[dof_idx], theta_B_ref[dof_idx], command_A, command_B);
        }

        if (verbose && cycle_count % 2500 == 0) { // Log every ~5 seconds for inner loop
          Serial.println("Inner loop DOF " + String(dof_idx) + ":");
          Serial.println("  Theta_0(t): agonist=" + String(theta_0_agonist_t, 2) +
                         "°, antagonist=" + String(theta_0_antagonist_t, 2) + "°");
          Serial.println("  Theta_0 motors: agonist=" + String(theta_0_agonist_motor, 2) +
                         "°, antagonist=" + String(theta_0_antagonist_motor, 2) + "°");
          Serial.print("  Delta_theta: " + String(delta_theta[dof_idx], 2) + "°");
          if (cascade_value < 0.99f) {
            Serial.print(" (scaled to " + String(cascade_value * delta_theta[dof_idx], 2) +
                         "° with influence=" + String(cascade_value, 2) + ")");
          }
          Serial.println();
          Serial.println(
              "  Cascade correction: agonist=+" +
              String(cascade_value * (0.5f * delta_theta[dof_idx] + 0.5f * stiffness_value), 2) +
              "°, antagonist=+" +
              String(cascade_value * (0.5f * delta_theta[dof_idx] - 0.5f * stiffness_value), 2) +
              "°");
          Serial.println("  Agonist: ref=" + String(theta_A_ref[dof_idx], 2) +
                         "°, curr=" + String(theta_A_curr, 2) + "°, err=" + String(error_A, 2) +
                         "°, cmd=" + String(command_A, 1));
          Serial.println("  Antagonist: ref=" + String(theta_B_ref[dof_idx], 2) +
                         "°, curr=" + String(theta_B_curr, 2) + "°, err=" + String(error_B, 2) +
                         "°, cmd=" + String(command_B, 1));
        }
      }
    } else {
      // === HOLDING PHASE ===

      if (verbose) {
        Serial.println("\nStarting position holding phase with cascade control");
      }

      // Maintain position using cascade control
      int safety_check_counter = 0; // Counter for periodic safety checks
      while (true) {
        next_time += sampling_period;
        cycle_count++;
        safety_check_counter++;

        // Outer loop to update delta_theta
        if ((cycle_count - 1) % OUTER_LOOP_DIV == 0) {

          for (int i = 0; i < active_dof_count; i++) {
            int dof_idx = active_dof_indices[i];

            bool isValid;
            q_curr[dof_idx] = getCurrentAngle(dof_idx, isValid);

            if (!isValid) {
              stopAllMotors();
              Serial.println("ERROR: Encoder issue during hold");
              return MovementResult(MOVEMENT_ERROR, "Error: Encoder issue during hold\n");
            }

            // PERIODIC SAFETY CHECKS: run every 100 cycles to reduce overhead
            if (safety_check_counter >= 100) {
              String hold_safety_message;
              if (!checkSafetyForDof(dof_idx, q_curr[dof_idx], hold_safety_message, false)) {
                stopAllMotors();
                return MovementResult(MOVEMENT_SAFETY_STOP,
                                      "SAFETY ERROR: " + hold_safety_message + "\n");
              }
            }

            // The desired angle is now the final target
            float error = local_target_angles[dof_idx] - q_curr[dof_idx];

            // Outer PID
            error_integral_q[dof_idx] += error * outer_loop_dt;
            float error_derivative = (error - previous_error_q[dof_idx]) / outer_loop_dt;

            error_integral_q[dof_idx] = constrain(error_integral_q[dof_idx], -10.0f, 10.0f);

            delta_theta[dof_idx] = outer_kp_per_dof[dof_idx] * error +
                                   outer_ki_per_dof[dof_idx] * error_integral_q[dof_idx] +
                                   outer_kd_per_dof[dof_idx] * error_derivative;

            delta_theta[dof_idx] = constrain(delta_theta[dof_idx], -30.0f, 30.0f);

            previous_error_q[dof_idx] = error;

            // Verify position error
            if (fabs(error) > config.dofs[dof_idx].motion.holding_position_error_threshold) {
              stopAllMotors();
              return MovementResult(MOVEMENT_SAFETY_STOP,
                                    "POSITION ERROR: DOF " + String(dof_idx) + " error " +
                                        String(error, 3) + " deg exceeds threshold\n");
            }
          }
        }

        // Inner loop continues to control motors
        for (int i = 0; i < active_dof_count; i++) {
          int dof_idx = active_dof_indices[i];

          // In holding, theta_0 is the final target angle (constant)
          float theta_0_target = local_target_angles[dof_idx];

          // Get mapping data
          if (!hasValidEquations(dof_idx)) {
            stopAllMotors();
            Serial.println(
                "Error: Equation limits unavailable during hold for DOF " +
                String(dof_idx));
            return MovementResult(
                MOVEMENT_ERROR,
                "Error: Equation limits unavailable during hold for DOF " +
                    String(dof_idx) + "\n");
          }

          // Calculate target motor angles using linear equations
          float theta_0_agonist_motor, theta_0_antagonist_motor;
          bool equations_ok =
              calculateMotorAnglesWithEquations(dof_idx, theta_0_target, theta_0_target,
                                                theta_0_agonist_motor, theta_0_antagonist_motor);

          if (!equations_ok) {
            stopAllMotors();
            Serial.println(
                "Error: Linear equations unavailable during hold for DOF " +
                String(dof_idx));
            return MovementResult(
                MOVEMENT_ERROR,
                "Error: Linear equations unavailable during hold for DOF " +
                    String(dof_idx) + "\n");
          }

          // Apply cascade formula with constant final values and cascade_influence
          float cascade_value_hold   = cascade_per_dof[dof_idx];
          float stiffness_value_hold = stiffness_per_dof[dof_idx];

          theta_A_ref[dof_idx] =
              theta_0_agonist_motor +
              cascade_value_hold * (0.5f * delta_theta[dof_idx] + 0.5f * stiffness_value_hold);
          theta_B_ref[dof_idx] =
              theta_0_antagonist_motor +
              cascade_value_hold * (0.5f * delta_theta[dof_idx] - 0.5f * stiffness_value_hold);

          float theta_A_curr = motor_pairs[i].agonist->getMultiAngleSync().angle;
          float theta_B_curr = motor_pairs[i].antagonist->getMultiAngleSync().angle;

          // PERIODIC SAFETY CHECK: Verify motor limits based on saved equations
          if (safety_check_counter >= 100) {
            float agonist_min    = linear_equations[dof_idx].agonist_safe_min;
            float agonist_max    = linear_equations[dof_idx].agonist_safe_max;
            float antagonist_min = linear_equations[dof_idx].antagonist_safe_min;
            float antagonist_max = linear_equations[dof_idx].antagonist_safe_max;

            // Verify agonist motor limits
            if (theta_A_curr < agonist_min || theta_A_curr > agonist_max) {
              stopAllMotors();
              return MovementResult(
                  MOVEMENT_SAFETY_STOP,
                  "SAFETY ERROR: Agonist motor DOF " + String(dof_idx) +
                      " out of range during hold: " + String(theta_A_curr, 1) +
                      " deg (safe range: " + String(agonist_min, 1) + "-" +
                      String(agonist_max, 1) + " deg)\n");
            }

            // Verify antagonist motor limits
            if (theta_B_curr < antagonist_min || theta_B_curr > antagonist_max) {
              stopAllMotors();
              return MovementResult(
                  MOVEMENT_SAFETY_STOP,
                  "SAFETY ERROR: Antagonist motor DOF " + String(dof_idx) +
                      " out of range during hold: " + String(theta_B_curr, 1) +
                      " deg (safe range: " + String(antagonist_min, 1) + "-" +
                      String(antagonist_max, 1) + " deg)\n");
            }
          }

          float command_A = motor_pairs[i].pid_agonist->control(theta_A_ref[dof_idx], theta_A_curr);
          float command_B =
              motor_pairs[i].pid_antagonist->control(theta_B_ref[dof_idx], theta_B_curr);

          command_A = constrain(command_A, -max_torque, max_torque);
          command_B = constrain(command_B, -max_torque, max_torque);

          motor_pairs[i].agonist->setTorque((int)command_A);
          motor_pairs[i].antagonist->setTorque((int)command_B);
        }

        // Reset safety check counter
        if (safety_check_counter >= 100) {
          safety_check_counter = 0;
        }

        // === COMMAND CONTROL WITH NEW SYSTEM ===

        // Priority check: EMERGENCY STOP
        if (emergency_stop_requested) {
          LOG_ERROR("EMERGENCY STOP received during hold");
          exit_code = MOVEMENT_STOPPED;
          break;
        }

        // Check for new command available
        if (buffer_ready[active_buffer]) {
          uint8_t new_command = pending_command_type;

          if (new_command == CMD_STOP) {
            LOG_WARN("STOP command received during hold");
            exit_code = MOVEMENT_STOPPED;
            break;
          } else if (new_command == CMD_MOVE_MULTI_DOF) {
            // Do NOT reset the buffer flag here — let core1_loop handle it
            LOG_INFO("CASCADE TRANSITION: Ready for new movement");
            exit_code = MOVEMENT_TRANSITION;
            break;
          }
          // Other commands are ignored during holding
        }

        // Sleep until next cycle
        int64_t sleep_time = next_time - time_us_64();
        if (sleep_time > 0) {
          sleep_us(sleep_time);
        }
      }

      break; // End of control
    }

    // === COMMAND CONTROL DURING MOVEMENT ===

    // Priority check: EMERGENCY STOP
    if (emergency_stop_requested) {
      LOG_ERROR("EMERGENCY STOP received during movement");
      exit_code = MOVEMENT_STOPPED;
      break;
    }

    // Check for normal STOP command
    if (buffer_ready[active_buffer] && pending_command_type == CMD_STOP) {
      LOG_WARN("STOP command received during movement");
      exit_code = MOVEMENT_STOPPED;
      break;
    }

    // Sleep until next cycle
    int64_t sleep_time = next_time - time_us_64();
    if (sleep_time > 0) {
      sleep_us(sleep_time);
    }
  }

  // === MOTOR STOP HANDLING ===

  if (exit_code == MOVEMENT_TRANSITION) {
    if (verbose) {
      LOG_INFO("CASCADE TRANSITION: Motors kept active");
    }
  } else {
    stopAllMotors();
    if (verbose) {
      LOG_INFO("Motors stopped");
    }
  }

  // === FINAL REPORT ===

  if (verbose) {
    LOG_INFO("\n=== CASCADE CONTROL RESULTS ===");

    for (int i = 0; i < active_dof_count; i++) {
      int dof_idx = active_dof_indices[i];
      bool isValid;
      float final_angle = getCurrentAngle(dof_idx, isValid);

      if (isValid) {
        float error = fabs(final_angle - local_target_angles[dof_idx]);
        LOG_INFO("DOF " + String(dof_idx) + ": target=" + String(local_target_angles[dof_idx], 2) +
                 " deg, final=" + String(final_angle, 2) + " deg, error=" + String(error, 3) +
                 " deg");
        LOG_INFO("  Final delta_theta: " + String(delta_theta[dof_idx], 2) + " deg");
      }
    }

  LOG_INFO("\nCascade control completed successfully");
    LOG_INFO("Total cycles: " + String(cycle_count));
    LOG_INFO("Outer-loop cycles: " + String(cycle_count / OUTER_LOOP_DIV));
    LOG_INFO("=====================================");
  }

  return MovementResult(exit_code, "");
}

