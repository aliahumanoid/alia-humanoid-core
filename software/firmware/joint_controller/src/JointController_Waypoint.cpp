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

// External time sync function (defined in core1.cpp)
extern uint32_t getAbsoluteTimeMs();

// Cycle counter for outer/inner loop division
static int cycle_count = 0;

// Constants from moveMultiDOF_cascade
#define SAMPLING_PERIOD_US 2000  // 2 ms = 500 Hz
#define OUTER_LOOP_DIV 5         // 500 Hz / 100 Hz = 5

// Outer loop variables (persistent across calls)
static float error_integral_q[MAX_DOFS] = {0}; // Outer PID integral error
static float previous_error_q[MAX_DOFS] = {0}; // Previous error for derivative
static float delta_theta[MAX_DOFS] = {0};      // Outer PID output

// Safety check counter (for periodic motor checks in HOLDING mode)
static int safety_check_counter = 0;

// Track previous state to detect MOVING → HOLDING transitions
static WaypointState prev_dof_state[MAX_DOFS] = {WaypointState::IDLE};

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
  cycle_count++;
  safety_check_counter++; // Increment for periodic safety checks
  bool any_movement = false;
  
  uint32_t t_now = getAbsoluteTimeMs();
  
  // Process each DOF independently
  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    
    // === EARLY EXIT: Skip if DOF is IDLE (no waypoints ever received) ===
    // This saves CPU time when no waypoint control is active
    WaypointState dof_state = waypoint_buffer_state(dof);
    if (dof_state == WaypointState::IDLE) {
      continue; // Skip this DOF entirely
    }
    
    // === CHECK WAYPOINT TRANSITION ===
    // Check if we've reached the current waypoint target
    WaypointEntry next_waypoint;
    if (waypoint_buffer_peek(dof, next_waypoint)) {
      if (t_now >= next_waypoint.t_arrival_ms) {
        // Waypoint reached - pop from buffer and update prev state
        waypoint_buffer_pop(dof);
        waypoint_buffer_set_prev(dof, next_waypoint.target_angle_deg, next_waypoint.t_arrival_ms);
        
        LOG_DEBUG("[Waypoint] DOF " + String(dof) + " reached: " + 
                  String(next_waypoint.target_angle_deg, 2) + "° at t=" + String(t_now));
        
        // Check if more waypoints available
        WaypointEntry peek_next;
        if (!waypoint_buffer_peek(dof, peek_next)) {
          // No more waypoints - will enter HOLDING mode in outer loop
          LOG_DEBUG("[Waypoint] DOF " + String(dof) + " buffer empty, entering HOLDING");
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
        // HOLDING mode - maintain current position
        bool is_valid = false;
        q_des = getCurrentAngle(dof, is_valid);
        
        if (!is_valid) {
          LOG_WARN("[Waypoint] Invalid encoder for DOF " + String(dof) + ", skipping");
          continue;
        }
      }
      
      // Read current angle
      bool is_valid = false;
      float q_curr = getCurrentAngle(dof, is_valid);
      
      if (!is_valid) {
        LOG_WARN("[Waypoint] Invalid encoder reading for DOF " + String(dof));
        continue;
      }
      
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
          LOG_DEBUG("[Waypoint] DOF " + String(dof) + " transitioned MOVING → HOLDING");
        }
      }
      
      // Determine if we should check safety:
      // - Always check in MOVING mode (every outer loop cycle = 100 Hz)
      // - Check immediately when entering HOLDING mode
      // - Check periodically in HOLDING mode (every 100 cycles)
      bool should_check_safety = has_waypoints || just_entered_holding || (is_holding && safety_check_counter >= 100);
      
      if (should_check_safety) {
        String safety_message;
        // Check motors (tendon breakage) only in HOLDING mode:
        // - Immediately when entering HOLDING (to catch issues right away)
        // - Periodically every 100 cycles (to catch issues during long holds)
        bool check_motors = is_holding && (just_entered_holding || (safety_check_counter >= 100));
        
        if (!checkSafetyForDof(dof, q_curr, safety_message, check_motors)) {
          // Safety violation detected - stop all motors immediately
          stopAllMotors();
          LOG_ERROR("[Waypoint Safety] MOVEMENT STOPPED: " + safety_message);
          
          // Reset waypoint state to IDLE for this DOF
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
          waypoint_buffer_clear(dof);
          prev_dof_state[dof] = WaypointState::IDLE;
          
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
    }
    
    // === INNER LOOP @ 500 Hz (Motor Control) ===
    // Execute every cycle (same as moveMultiDOF_cascade)
    
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
    float theta_A_curr = agonist->getMultiAngleSync().angle;
    float theta_B_curr = antagonist->getMultiAngleSync().angle;
    
    // Inner PID for motors (compute torque commands)
    float command_A = pid_agonist->control(theta_A_ref, theta_A_curr);
    float command_B = pid_antagonist->control(theta_B_ref, theta_B_curr);
    
    // Apply torque limits
    const int MAX_TORQUE = 1000; // TODO: Make this configurable
    command_A = constrain(command_A, -MAX_TORQUE, MAX_TORQUE);
    command_B = constrain(command_B, -MAX_TORQUE, MAX_TORQUE);
    
    // Send torque commands to motors
    agonist->setTorque((int)command_A);
    antagonist->setTorque((int)command_B);
  }
  
  // Reset safety check counter after processing all DOFs
  // This ensures all DOFs in HOLDING mode are checked in the same cycle
  if (safety_check_counter >= 100) {
    safety_check_counter = 0;
  }
  
  return any_movement;
}
