/**
 * @file JointController_Waypoint.cpp
 * @brief Waypoint-based trajectory execution with cascade control
 * 
 * Implementation follows CAN_CONTROL_PROTOCOL.md section 5.2 and reuses the
 * exact same cascade control logic from moveMultiDOF_cascade.
 * 
 * Key features:
 * - Linear interpolation between waypoints (smoothness from waypoint density @ 50-100 Hz)
 * - Per-DOF waypoint buffers (independent control)
 * - State management: IDLE → MOVING → HOLDING
 * - Cascade control architecture:
 *   * SAMPLING_PERIOD = 2000 µs (2 ms) → 500 Hz
 *   * Outer PID @ 100 Hz (joint-level, computes delta_theta)
 *   * Inner PID @ 500 Hz (motor-level, computes torque commands)
 *   * theta_ref = theta_0 + cascade_correction
 * 
 * @see waypoint_buffer.h for buffer management
 * @see CAN_CONTROL_PROTOCOL.md section 5.2 for detailed specification
 * @see JointController_Movement.cpp::moveMultiDOF_cascade for reference implementation
 */

#include "JointController.h"
#include <waypoint_buffer.h>
#include <Arduino.h>
#include <debug.h>
#include "main_common.h"  // For shared_dof_angles

// External time sync function (defined in core1.cpp)
extern uint32_t getAbsoluteTimeMs();

// Cycle counter for outer/inner loop division (wraps at 1000 to prevent overflow)
static uint16_t cycle_count = 0;

// Constants from moveMultiDOF_cascade
#define SAMPLING_PERIOD_US 2000  // 2 ms = 500 Hz
#define OUTER_LOOP_DIV 5         // 500 Hz / 100 Hz = 5

// Outer loop variables (persistent across calls)
static float error_integral_q[MAX_DOFS] = {0}; // Outer PID integral error
static float previous_error_q[MAX_DOFS] = {0}; // Previous error for derivative
static float delta_theta[MAX_DOFS] = {0};      // Outer PID output

// Safety check counter (for periodic motor checks in HOLDING mode)
static uint16_t safety_check_counter = 0;

// Track previous state to detect MOVING → HOLDING transitions
static WaypointState prev_dof_state[MAX_DOFS] = {WaypointState::IDLE};

// Track if PID state needs reset when transitioning IDLE/HOLDING → MOVING
static bool pid_reset_needed[MAX_DOFS] = {true, true, true};

// Timing constants
static const float outer_loop_dt = OUTER_LOOP_DIV * SAMPLING_PERIOD_US / 1000000.0f; // ~10ms = 0.01s
static const float inner_loop_dt = SAMPLING_PERIOD_US / 1000000.0f;                   // ~2ms = 0.002s

/**
 * @brief Execute waypoint-based movement for all DOFs
 * 
 * This is the main entry point called from core1_loop() at 500 Hz.
 * Implements the same cascade control as moveMultiDOF_cascade but with
 * continuous waypoint consumption instead of pre-generated trajectory arrays.
 * 
 * Following CAN_CONTROL_PROTOCOL.md section 5.2.3:
 * - Outer loop @ 100 Hz (every 5 cycles)
 * - Inner loop @ 500 Hz (every cycle)
 * - Linear interpolation between waypoints
 * - Hold position when buffer empty
 * 
 * @return true if any DOF is actively moving
 */
bool JointController::executeWaypointMovement() {
  // Wrap cycle_count to prevent overflow (1000 cycles = 2 seconds at 500Hz)
  cycle_count = (cycle_count + 1) % 1000;
  safety_check_counter++; // Increment for periodic safety checks
  bool any_movement = false;
  
  uint32_t t_now = getAbsoluteTimeMs();
  
  // Process each DOF independently
  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    
    // === EARLY EXIT: Skip if DOF is IDLE (no waypoints ever received) ===
    // This saves CPU time when no waypoint control is active
    WaypointState dof_state = waypoint_buffer_state(dof);
    if (dof_state == WaypointState::IDLE) {
      // Mark PID reset needed for when this DOF becomes active
      pid_reset_needed[dof] = true;
      prev_dof_state[dof] = WaypointState::IDLE;
      continue; // Skip this DOF entirely
    }
    
    // === RESET PID STATE when transitioning from IDLE to MOVING ===
    // Only reset when starting a NEW sequence (from IDLE), not when resuming from HOLDING
    // This prevents integral windup from previous sequences while preserving
    // the integral compensation during HOLDING (needed for gravity/friction)
    bool just_started_from_idle = (prev_dof_state[dof] == WaypointState::IDLE) && 
                                   (dof_state == WaypointState::MOVING);
    if (pid_reset_needed[dof] && just_started_from_idle) {
      error_integral_q[dof] = 0.0f;
      previous_error_q[dof] = 0.0f;
      delta_theta[dof] = 0.0f;
      pid_reset_needed[dof] = false;
      LOG_DEBUG("[Waypoint] DOF " + String(dof) + " PID state reset (new sequence)");
    }
    
    // === METRICS: Initialize tracker for NEW movement (from IDLE or HOLDING) ===
    // Detect when a new movement starts:
    // - IDLE → MOVING: first movement ever
    // - HOLDING → MOVING: new movement after previous one completed
    // Use !tracking_active as additional guard to prevent multiple initializations
    bool new_movement_started = (prev_dof_state[dof] != WaypointState::MOVING) && 
                                 (dof_state == WaypointState::MOVING);
    
    if (metrics_tracking_enabled && new_movement_started && dof < 3 && 
        !metrics_tracker[dof].tracking_active) {
      WaypointEntry first_wp;
      if (waypoint_buffer_peek(dof, first_wp)) {
        float start_angle = shared_dof_angles.valid[dof] ? shared_dof_angles.angles[dof] : 0.0f;
        metrics_tracker[dof].reset(start_angle, first_wp.target_angle_deg);
        LOG_DEBUG("[Metrics] DOF " + String(dof) + " tracking started: " + 
                  String(start_angle, 2) + "° → " + String(first_wp.target_angle_deg, 2) + "°");
      }
    }
    
    // === CHECK WAYPOINT TRANSITION ===
    // Check if we've reached the current waypoint target
    WaypointEntry next_waypoint;
    if (waypoint_buffer_peek(dof, next_waypoint)) {
      if (t_now >= next_waypoint.t_arrival_ms) {
        // Waypoint reached - pop from buffer and update prev state
        waypoint_buffer_pop(dof);
        waypoint_buffer_set_prev(dof, next_waypoint.target_angle_deg, next_waypoint.t_arrival_ms);
        
        // Reduce logging overhead: only log every 50th waypoint to avoid serial bottleneck
        // at high waypoint densities (e.g., 500 waypoints @ 12ms = 6000 waypoints/min)
        static uint16_t waypoint_log_counter[MAX_DOFS] = {0};
        waypoint_log_counter[dof]++;
        if (waypoint_log_counter[dof] >= 50) {
          LOG_INFO("[Waypoint] DOF " + String(dof) + " progress: " + 
                    String(next_waypoint.target_angle_deg, 2) + "° at t=" + String(t_now));
          waypoint_log_counter[dof] = 0;
        }
        
        // Check if more waypoints available
        WaypointEntry peek_next;
        if (!waypoint_buffer_peek(dof, peek_next)) {
          // No more waypoints - this was the LAST waypoint
          // prev_angle is now set to this waypoint's target
          float final_target = waypoint_buffer_prev_angle(dof);
          LOG_INFO("[Waypoint] DOF " + String(dof) + " LAST waypoint consumed: " + 
                    String(next_waypoint.target_angle_deg, 2) + "° → prev_angle=" + 
                    String(final_target, 2) + "°");
          waypoint_log_counter[dof] = 0; // Reset for next sequence
        }
      }
    }
    
    // === OUTER LOOP @ 100 Hz (Joint PID) ===
    // Execute outer loop every 5 cycles (500 Hz / 5 = 100 Hz)
    if ((cycle_count - 1) % OUTER_LOOP_DIV == 0) {
      
      float q_des = 0.0f;
      bool is_moving = false;
      
      // Check if we have waypoints to process
      WaypointEntry current_target;
      if (waypoint_buffer_peek(dof, current_target)) {
        // MOVING state - linear interpolation
        is_moving = true;
        any_movement = true;
        
        float prev_angle = waypoint_buffer_prev_angle(dof);
        uint32_t prev_time = waypoint_buffer_prev_time(dof);
        
        float target_angle = current_target.target_angle_deg;
        float time_total = current_target.t_arrival_ms - prev_time;
        float time_elapsed = t_now - prev_time;
        
        // Compute progress (0.0 to 1.0)
        float progress = 0.0f;
        if (time_total > 0) {
          progress = time_elapsed / time_total;
          progress = constrain(progress, 0.0f, 1.0f);
        }
        
        // Linear interpolation: q_des = start + (end - start) × progress
        q_des = prev_angle + (target_angle - prev_angle) * progress;
        
      } else {
        // HOLDING mode - maintain TARGET position (last waypoint target)
        // Use the last waypoint's target angle, NOT current encoder reading
        // This ensures the PID keeps trying to reach and hold the target
        q_des = waypoint_buffer_prev_angle(dof);
      }
      
      // Read current angle from shared state (updated by Core0)
      if (!shared_dof_angles.valid[dof]) {
        LOG_WARN("[Waypoint] Invalid encoder reading for DOF " + String(dof));
        continue;
      }
      float q_curr = shared_dof_angles.angles[dof];
      
      // === RUNTIME SAFETY CHECK (same as moveMultiDOF_cascade) ===
      // Check joint limits, mapping limits, and optionally motor limits (tendon breakage)
      // - MOVING mode: check every cycle (100 Hz) for immediate detection
      // - HOLDING mode: check immediately on transition, then every 100 cycles (~1 second)
      
      // Determine current state based on waypoint buffer
      WaypointEntry check_waypoint;
      bool has_waypoints = waypoint_buffer_peek(dof, check_waypoint);
      bool is_holding = !has_waypoints; // If no waypoints, we're holding position
      
      // Detect MOVING → HOLDING transition by comparing with previous cycle's state
      bool just_entered_holding = (prev_dof_state[dof] == WaypointState::MOVING) && is_holding;
      
      // Update buffer state to HOLDING if buffer is empty and we were MOVING
      // This ensures the state machine is consistent
      if (is_holding && (dof_state == WaypointState::MOVING || just_entered_holding)) {
        waypoint_buffer_set_state(dof, WaypointState::HOLDING);
        dof_state = WaypointState::HOLDING; // Update local variable for this cycle
        
        if (just_entered_holding) {
          // Get the holding target for this DOF
          float holding_target = waypoint_buffer_prev_angle(dof);
          LOG_DEBUG("[Waypoint] DOF " + String(dof) + " transitioned MOVING → HOLDING");
          
          // Send structured message for UI display
          Serial.println("HOLDING_TARGET:DOF=" + String(dof) + ":ANGLE=" + String(holding_target, 2));
          
          // DO NOT reset PID integral here - we need it to maintain position
          // against static loads (gravity, friction). The integral was compensating
          // for steady-state error, and resetting it would cause drift.
          // PID will be reset only when a new sequence starts from IDLE.
          
          // === METRICS: Finalize movement metrics ===
          if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
            MetricsTracker& mt = metrics_tracker[dof];
            
            // Update final target (in case it changed during movement)
            float original_target = mt.target_angle_deg;
            mt.target_angle_deg = holding_target;
            
            // Finalize and store metrics
            MovementMetrics m = mt.finalize();
            m.dof_index = dof;
            last_movement_metrics[dof] = m;
            metrics_ready[dof] = true;
            
            // Stop tracking
            mt.tracking_active = false;
            
            // Detailed logging for debugging
            LOG_INFO("[Metrics] DOF " + String(dof) + " FINAL:");
            LOG_INFO("  start=" + String(mt.start_angle_deg, 2) + 
                     "° → target=" + String(holding_target, 2) + 
                     "° (orig=" + String(original_target, 2) + "°)");
            LOG_INFO("  rise=" + String(m.rise_time_ms) + "ms (90%=" + 
                     String(mt.reached_90_percent ? "yes" : "no") + ")");
            LOG_INFO("  settle=" + String(m.settling_time_ms) + "ms, max_err=" + 
                     String(mt.max_error_deg, 2) + "°");
            LOG_INFO("  overshoot=" + String(m.overshoot_x100 / 100.0f, 1) + 
                     "% (max=" + String(mt.max_overshoot_deg, 2) + 
                     "°, dir=" + String(mt.movement_direction, 0) + ")");
            LOG_INFO("  sse=" + String(m.sse_x100 / 100.0f, 2) + 
                     "° (samples=" + String(mt.sse_sample_count) + ")");
          }
        }
      }
      
      // Determine if we should check safety:
      // - Always check joint limits in MOVING mode (every outer loop cycle = 100 Hz)
      // - Check periodically in HOLDING mode (every 20 cycles = ~200ms at 100Hz)
      // NOTE: We do NOT check immediately when entering HOLDING because motors may still be settling
      bool should_check_safety = has_waypoints || (is_holding && safety_check_counter >= 20);
      
      if (should_check_safety) {
        String safety_message;
        // Check motors (tendon breakage) only in HOLDING mode periodically
        // NOT immediately when entering HOLDING - motors need time to settle
        bool check_motors = is_holding && (safety_check_counter >= 20);
        
        // Log when we're doing a periodic motor check in HOLDING mode
        if (check_motors) {
          LOG_DEBUG("[Waypoint] DOF " + String(dof) + " periodic motor safety check (counter=" + 
                    String(safety_check_counter) + ")");
        }
        
        if (!checkSafetyForDof(dof, q_curr, safety_message, check_motors)) {
          // Safety violation detected - stop all motors immediately
          stopAllMotors();
          LOG_ERROR("[Waypoint Safety] MOVEMENT STOPPED: " + safety_message);
          
          // Reset waypoint state to IDLE for this DOF
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
          waypoint_buffer_clear(dof);
          prev_dof_state[dof] = WaypointState::IDLE;
          
          // Mark PID reset needed for next sequence
          pid_reset_needed[dof] = true;
          
          // Continue checking other DOFs (don't return, just skip this one)
          continue;
        }
      }
      
      // Update previous state for next cycle (use updated dof_state)
      prev_dof_state[dof] = is_holding ? WaypointState::HOLDING : WaypointState::MOVING;
      
      // Compute error
      float error = q_des - q_curr;
      
      // Outer PID to compute delta_theta (same as moveMultiDOF_cascade)
      // Get outer loop parameters
      float outer_kp, outer_ki, outer_kd, stiffness_ref, cascade_influence;
      if (!getOuterLoopParameters(dof, outer_kp, outer_ki, outer_kd, stiffness_ref, cascade_influence)) {
        // Use defaults if not configured
        outer_kp = DEFAULT_OUTER_LOOP_KP;
        outer_ki = DEFAULT_OUTER_LOOP_KI;
        outer_kd = DEFAULT_OUTER_LOOP_KD;
        stiffness_ref = DEFAULT_STIFFNESS_REF_DEG;
        cascade_influence = DEFAULT_CASCADE_INFLUENCE;
      }
      
      // Compute PID terms
      error_integral_q[dof] += error * outer_loop_dt;
      float error_derivative = (error - previous_error_q[dof]) / outer_loop_dt;
      
      // Limit integral to avoid windup
      const float MAX_INTEGRAL = 10.0f; // Degrees
      error_integral_q[dof] = constrain(error_integral_q[dof], -MAX_INTEGRAL, MAX_INTEGRAL);
      
      // Compute delta_theta
      delta_theta[dof] = outer_kp * error + 
                         outer_ki * error_integral_q[dof] + 
                         outer_kd * error_derivative;
      
      // Limit delta_theta for safety
      const float MAX_DELTA_THETA = 30.0f; // Maximum correction degrees
      delta_theta[dof] = constrain(delta_theta[dof], -MAX_DELTA_THETA, MAX_DELTA_THETA);
      
      previous_error_q[dof] = error;
      
      // === METRICS: Update tracking during movement ===
      if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
        MetricsTracker& mt = metrics_tracker[dof];
        float abs_error = fabs(error);
        uint32_t elapsed_ms = millis() - mt.movement_start_ms;
        
        // Track maximum error
        if (abs_error > mt.max_error_deg) {
          mt.max_error_deg = abs_error;
        }
        
        // Rise time: detect when we first reach 90% of target
        if (!mt.reached_90_percent) {
          float range = fabs(mt.target_angle_deg - mt.start_angle_deg);
          float progress_toward_target = fabs(q_curr - mt.start_angle_deg);
          if (range > 0.1f && progress_toward_target >= range * 0.9f) {
            mt.reached_90_percent = true;
            mt.rise_time_ms = elapsed_ms;
          }
        }
        
        // Overshoot: detect if we go past target
        float overshoot_amount = (q_curr - mt.target_angle_deg) * mt.movement_direction;
        if (overshoot_amount > 0 && overshoot_amount > mt.max_overshoot_deg) {
          mt.max_overshoot_deg = overshoot_amount;
          mt.overshoot_detected = true;
        }
        
        // Settling: track when we enter and stay in ±0.5° band
        const float SETTLING_BAND = 0.5f;
        bool in_band = abs_error < SETTLING_BAND;
        
        if (in_band) {
          if (!mt.in_settling_band) {
            // Just entered the band
            mt.in_settling_band = true;
            mt.settling_enter_ms = elapsed_ms;
            mt.settling_stable_count = 0;
          } else {
            // Still in band - increment stable count
            mt.settling_stable_count++;
            // After 10 consecutive cycles in band (~100ms), consider settled
            if (mt.settling_stable_count >= 10 && mt.settling_time_ms == 0) {
              mt.settling_time_ms = mt.settling_enter_ms;
            }
          }
        } else {
          // Exited band - reset
          mt.in_settling_band = false;
          mt.settling_stable_count = 0;
        }
        
        // SSE tracking: accumulate error samples during HOLDING
        if (is_holding) {
          mt.sse_accumulator += abs_error;
          mt.sse_sample_count++;
        }
      }
    }
    
    // === INNER LOOP @ 500 Hz (Motor Control) ===
    // Run at full 500 Hz in both MOVING and HOLDING modes for best accuracy.
    // The MCP2515 flush mechanism in LKM_Motor handles any CAN buffer issues.
    // See: LKM_Motor::getMultiAngleSync() for the flush and retry logic.
    
    // Find motors for this DOF
    LKM_Motor *agonist = nullptr;
    LKM_Motor *antagonist = nullptr;
    PID *pid_agonist = nullptr;
    PID *pid_antagonist = nullptr;
    int agonist_idx = -1;
    int antagonist_idx = -1;
    
    for (int i = 0; i < config.motor_count; i++) {
      if (config.motors[i].dof_index == dof) {
        if (config.motors[i].is_agonist) {
          agonist = motors[i];
          agonist_idx = i;
          pid_agonist = pid_controllers[i];
        } else {
          antagonist = motors[i];
          antagonist_idx = i;
          pid_antagonist = pid_controllers[i];
        }
      }
    }
    
    if (agonist == nullptr || antagonist == nullptr) {
      // No motors for this DOF, skip
      continue;
    }
    
    // Get current waypoint (or last position for HOLDING)
    WaypointEntry current_target;
    float theta_0_joint; // Joint angle for theta_0 calculation
    
    if (waypoint_buffer_peek(dof, current_target)) {
      // MOVING: use interpolated position
      float prev_angle = waypoint_buffer_prev_angle(dof);
      uint32_t prev_time = waypoint_buffer_prev_time(dof);
      float target_angle = current_target.target_angle_deg;
      float time_total = current_target.t_arrival_ms - prev_time;
      float time_elapsed = t_now - prev_time;
      float progress = (time_total > 0) ? (time_elapsed / time_total) : 0.0f;
      progress = constrain(progress, 0.0f, 1.0f);
      theta_0_joint = prev_angle + (target_angle - prev_angle) * progress;
    } else {
      // HOLDING: use last known position
      theta_0_joint = waypoint_buffer_prev_angle(dof);
    }
    
    // Compute theta_0 for motors using linear equations
    float theta_0_agonist_motor, theta_0_antagonist_motor;
    bool equations_ok = calculateMotorAnglesWithEquations(dof, theta_0_joint, theta_0_joint,
                                                          theta_0_agonist_motor, theta_0_antagonist_motor);
    
    if (!equations_ok) {
      // No linear equations, cannot control this DOF
      static uint32_t last_warn_time = 0;
      if (millis() - last_warn_time > 5000) {
        LOG_WARN("[Waypoint] DOF " + String(dof) + " has no linear equations, cannot move");
        last_warn_time = millis();
      }
      continue;
    }
    
    // Get cascade parameters
    float outer_kp, outer_ki, outer_kd, stiffness_ref, cascade_influence;
    if (!getOuterLoopParameters(dof, outer_kp, outer_ki, outer_kd, stiffness_ref, cascade_influence)) {
      stiffness_ref = DEFAULT_STIFFNESS_REF_DEG;
      cascade_influence = DEFAULT_CASCADE_INFLUENCE;
    }
    
    // Compute motor references using cascade control formula (same as moveMultiDOF_cascade)
    float theta_A_ref = theta_0_agonist_motor + 
                        cascade_influence * (0.5f * delta_theta[dof] + 0.5f * stiffness_ref);
    float theta_B_ref = theta_0_antagonist_motor + 
                        cascade_influence * (0.5f * delta_theta[dof] - 0.5f * stiffness_ref);
    
    // Read current motor angles
    MultiAngleData data_A = agonist->getMultiAngleSync();
    MultiAngleData data_B = antagonist->getMultiAngleSync();
    float theta_A_curr = data_A.angle;
    float theta_B_curr = data_B.angle;
    
    // === SANITY CHECK: Detect obviously invalid readings ===
    // Values outside ±100000° are clearly garbage (CAN corruption)
    bool invalid_A = (theta_A_curr < -100000.0f || theta_A_curr > 100000.0f || isnan(theta_A_curr));
    bool invalid_B = (theta_B_curr < -100000.0f || theta_B_curr > 100000.0f || isnan(theta_B_curr));
    
    if (invalid_A || invalid_B) {
      static uint32_t last_invalid_log = 0;
      if (millis() - last_invalid_log > 100) { // Log max every 100ms
        LOG_ERROR("[Waypoint] DOF " + String(dof) + " INVALID CAN READ: A=" + 
                  String(theta_A_curr, 2) + " B=" + String(theta_B_curr, 2));
        last_invalid_log = millis();
      }
      // Skip this cycle entirely - don't send any torque command
      continue;
    }
    
    // === DIAGNOSTIC: Detect suspicious motor readings ===
    // Check for sudden large jumps in motor angle (possible CAN corruption)
    static float last_theta_A[MAX_DOFS] = {0};
    static float last_theta_B[MAX_DOFS] = {0};
    static bool first_read[MAX_DOFS] = {true, true, true};
    static uint8_t consecutive_errors[MAX_DOFS] = {0};
    
    if (!first_read[dof]) {
      float jump_A = abs(theta_A_curr - last_theta_A[dof]);
      float jump_B = abs(theta_B_curr - last_theta_B[dof]);
      
      // If motor angle jumped more than 30° in one cycle (2ms), something is wrong
      // Reduced from 50° to 30° for earlier detection
      if (jump_A > 30.0f || jump_B > 30.0f) {
        consecutive_errors[dof]++;
        
        LOG_ERROR("[Waypoint DIAG] DOF " + String(dof) + " MOTOR ANGLE JUMP #" + 
                  String(consecutive_errors[dof]) + "!");
        LOG_ERROR("  Agonist: " + String(last_theta_A[dof], 2) + " → " + String(theta_A_curr, 2) + 
                  " (jump=" + String(jump_A, 2) + "°)");
        LOG_ERROR("  Antagonist: " + String(last_theta_B[dof], 2) + " → " + String(theta_B_curr, 2) + 
                  " (jump=" + String(jump_B, 2) + "°)");
        
        // After 3 consecutive errors, trigger emergency stop
        if (consecutive_errors[dof] >= 3) {
          LOG_ERROR("[Waypoint] DOF " + String(dof) + " - 3 consecutive CAN errors, EMERGENCY STOP!");
          stopAllMotors();
          waypoint_buffer_clear(dof);
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
          consecutive_errors[dof] = 0;
          first_read[dof] = true;
          continue;
        }
        
        // Skip this cycle to avoid sending bad commands, use last known good values
        continue;
      } else {
        // Good reading - reset error counter
        consecutive_errors[dof] = 0;
      }
    }
    
    last_theta_A[dof] = theta_A_curr;
    last_theta_B[dof] = theta_B_curr;
    first_read[dof] = false;
    
    // Inner PID for motors (compute torque commands)
    float command_A = pid_agonist->control(theta_A_ref, theta_A_curr);
    float command_B = pid_antagonist->control(theta_B_ref, theta_B_curr);
    
    // Apply torque limits from motor configuration
    float max_torque_A = config.motors[agonist_idx].max_torque;
    float max_torque_B = config.motors[antagonist_idx].max_torque;
    command_A = constrain(command_A, -max_torque_A, max_torque_A);
    command_B = constrain(command_B, -max_torque_B, max_torque_B);
    
    // === DIAGNOSTIC: Log extreme torque commands ===
    if (abs(command_A) >= max_torque_A * 0.95f || abs(command_B) >= max_torque_B * 0.95f) {
      static uint32_t last_torque_warn = 0;
      if (millis() - last_torque_warn > 500) { // Log max every 500ms
        LOG_WARN("[Waypoint DIAG] DOF " + String(dof) + " HIGH TORQUE: A=" + String(command_A, 0) + 
                 " B=" + String(command_B, 0) + " (max=" + String(max_torque_A, 0) + ")");
        LOG_WARN("  refs: A=" + String(theta_A_ref, 2) + " B=" + String(theta_B_ref, 2));
        LOG_WARN("  curr: A=" + String(theta_A_curr, 2) + " B=" + String(theta_B_curr, 2));
        last_torque_warn = millis();
      }
    }
    
    // Send torque commands to motors
    agonist->setTorque((int)command_A);
    antagonist->setTorque((int)command_B);
    
    // === UPDATE PID DIAGNOSTICS for CAN streaming ===
    // Store values for diagnostic stream (read by sendPIDDiagStreamData in core1.cpp)
    // Use theta_0_joint (target from interpolation) and shared_dof_angles (current reading)
    if (dof < 3) {
      float q_curr_diag = shared_dof_angles.angles[dof];
      pid_diagnostics.target_deg_x100[dof] = (int16_t)(theta_0_joint * 100.0f);
      pid_diagnostics.error_deg_x100[dof] = (int16_t)((theta_0_joint - q_curr_diag) * 100.0f);
      pid_diagnostics.torque_A[dof] = (int16_t)command_A;
      pid_diagnostics.torque_B[dof] = (int16_t)command_B;
      
      // === METRICS: Track torque for movement metrics ===
      if (metrics_tracking_enabled && metrics_tracker[dof].tracking_active) {
        int16_t abs_A = abs((int16_t)command_A);
        int16_t abs_B = abs((int16_t)command_B);
        if (abs_A > metrics_tracker[dof].max_torque_A) {
          metrics_tracker[dof].max_torque_A = abs_A;
        }
        if (abs_B > metrics_tracker[dof].max_torque_B) {
          metrics_tracker[dof].max_torque_B = abs_B;
        }
        // Accumulate torque integral (energy proxy) - saturate to avoid overflow
        uint32_t torque_sum = abs_A + abs_B;
        if (metrics_tracker[dof].torque_integral < 0xFFFFFFFF - torque_sum) {
          metrics_tracker[dof].torque_integral += torque_sum;
        }
      }
    }
  }
  
  // Mark diagnostics as valid after processing all DOFs
  pid_diagnostics.last_update_ms = millis();
  pid_diagnostics.valid = true;
  
  // Reset safety check counter after processing all DOFs
  // This ensures all DOFs in HOLDING mode are checked in the same cycle
  // Using 20 cycles = ~200ms at 100Hz outer loop rate
  if (safety_check_counter >= 20) {
    safety_check_counter = 0;
  }
  
  return any_movement;
}
