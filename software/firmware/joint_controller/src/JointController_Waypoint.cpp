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
#include "main_common.h"  // For shared_dof_angles, notch_filter_config
#include <NotchFilter.h>  // For torque resonance suppression
#include <math.h>  // For cosf, M_PI

// External time sync function (defined in core1.cpp)
extern uint32_t getAbsoluteTimeMs();

// External interpolation mode (defined in core1.cpp, set via CAN command)
extern volatile uint8_t waypoint_interpolation_mode;
#define INTERPOLATION_LINEAR 0
#define INTERPOLATION_COSINE 1

// Cycle counter for outer/inner loop division (wraps at 1000 to prevent overflow)
static uint16_t cycle_count = 0;

// Control loop timing - use global configurable variables
// Declared in main_common.h, defined in main.cpp
extern volatile uint16_t inner_loop_period_us;  // Inner loop period in µs
extern volatile uint8_t outer_loop_divisor;     // Outer loop divisor

// Outer loop variables (persistent across calls)
// NOTE: error_integral and previous_error are now handled by outer_pid_controllers
// which provides filtered derivative and proper anti-windup
static float delta_theta[MAX_DOFS] = {0};      // Outer PID output (from outer_pid_controllers)

// Notch filters for torque resonance suppression (one per motor)
// Indexed as [dof * 2 + motor_idx] where motor_idx: 0=agonist, 1=antagonist
static NotchFilter torque_notch_filters[MAX_DOFS * 2];
static bool notch_filters_initialized = false;

// Safety check counter (for periodic motor checks in HOLDING mode)
static uint16_t safety_check_counter = 0;

// Track previous state to detect MOVING → HOLDING transitions
static WaypointState prev_dof_state[MAX_DOFS] = {WaypointState::IDLE};

// Track if PID state needs reset when transitioning IDLE/HOLDING → MOVING
static bool pid_reset_needed[MAX_DOFS] = {true, true, true};

// === MOTOR POINTER CACHE ===
// Cache motor pointers to avoid searching every cycle (saves ~10µs per DOF per cycle)
static LKM_Motor* cached_agonist[MAX_DOFS] = {nullptr};
static LKM_Motor* cached_antagonist[MAX_DOFS] = {nullptr};
static PID* cached_pid_agonist[MAX_DOFS] = {nullptr};
static PID* cached_pid_antagonist[MAX_DOFS] = {nullptr};
static int cached_agonist_idx[MAX_DOFS] = {-1};
static int cached_antagonist_idx[MAX_DOFS] = {-1};
static bool motor_cache_valid[MAX_DOFS] = {false};

// Timing is calculated dynamically based on configurable frequencies
// inner_loop_period_us: base loop period (default 2000µs = 500Hz)
// outer_loop_divisor: outer loop runs every N inner cycles (default 1 = 500Hz)

// === CYCLE TIME PROFILING ===
// Measures actual execution time of the control loop for performance analysis
static uint32_t cycle_time_us_last = 0;       // Last cycle time in microseconds
static uint32_t cycle_time_us_max = 0;        // Maximum cycle time (for diagnostics)
static uint32_t cycle_time_us_avg = 0;        // Running average (exponential)
static uint32_t profiling_start_us = 0;       // Start timestamp for current cycle

/**
 * @brief Get current profiling statistics
 * @param last_us Output: last cycle time in µs
 * @param avg_us Output: average cycle time in µs
 * @param max_us Output: max cycle time since last reset in µs
 */
void getWaypointProfilingStats(uint32_t& last_us, uint32_t& avg_us, uint32_t& max_us) {
  last_us = cycle_time_us_last;
  avg_us = cycle_time_us_avg;
  max_us = cycle_time_us_max;
}

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
  // === PROFILING: Record cycle start time ===
  profiling_start_us = time_us_32();
  
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
      // Reset outer loop PID controller (handles integral, derivative, and filter state)
      resetOuterPID(dof);
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
        // Pass the first waypoint's arrival time so duration is measured from actual movement start
        metrics_tracker[dof].reset(start_angle, first_wp.target_angle_deg, first_wp.t_arrival_ms);
        LOG_DEBUG("[Metrics] DOF " + String(dof) + " tracking started: " + 
                  String(start_angle, 2) + "° → " + String(first_wp.target_angle_deg, 2) + 
                  "° (t_arrival=" + String(first_wp.t_arrival_ms) + ")");
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
    
    // === OUTER LOOP (Joint PID) ===
    // Execute outer loop every N inner cycles (configurable via outer_loop_divisor)
    // Default: 500Hz / 1 = 500Hz (same as inner loop for reduced vibrations)
    if ((cycle_count - 1) % outer_loop_divisor == 0) {
      
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
        
        // === TRAJECTORY DIAGNOSTIC: Log timing details ===
        // Log every 50 outer loop cycles (~500ms) or when progress crosses thresholds
        static uint16_t traj_diag_counter[MAX_DOFS] = {0};
        static float last_logged_progress[MAX_DOFS] = {0};
        traj_diag_counter[dof]++;
        
        bool should_log = (traj_diag_counter[dof] >= 50) || 
                          (progress >= 0.99f && last_logged_progress[dof] < 0.99f) ||
                          (progress < 0.01f && last_logged_progress[dof] > 0.1f);
        
        if (should_log && dof == 0) { // Only DOF 0 to reduce log spam
          float q_curr_now = shared_dof_angles.angles[dof];
          int32_t time_delta = (int32_t)t_now - (int32_t)current_target.t_arrival_ms;
          
          LOG_INFO("[TRAJ_DIAG] DOF" + String(dof) + 
                   " t=" + String(t_now) + 
                   " arr=" + String(current_target.t_arrival_ms) +
                   " dt=" + String(time_delta) + "ms" +
                   " prog=" + String(progress * 100, 1) + "%" +
                   " tgt=" + String(target_angle, 2) +
                   " cur=" + String(q_curr_now, 2) +
                   " err=" + String(target_angle - q_curr_now, 2));
          traj_diag_counter[dof] = 0;
          last_logged_progress[dof] = progress;
        }
        
        // Apply interpolation based on mode
        float smooth_progress = progress;
        if (waypoint_interpolation_mode == INTERPOLATION_COSINE) {
          // S-curve using cosine: smooth start and end (zero acceleration at boundaries)
          // smooth_progress = 0.5 * (1 - cos(π * progress))
          smooth_progress = 0.5f * (1.0f - cosf(progress * M_PI));
        }
        
        // Interpolation: q_des = start + (end - start) × smooth_progress
        q_des = prev_angle + (target_angle - prev_angle) * smooth_progress;
        
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

      // === UPDATE OUTER LOOP SAMPLING PERIOD ===
      // The PID controller needs the correct Ts for proper integral/derivative scaling
      // Only update when it changes (avoid overhead on every cycle)
      static float last_outer_loop_dt = 0.0f;
      float outer_loop_dt = outer_loop_divisor * inner_loop_period_us / 1000000.0f;
      if (outer_loop_dt != last_outer_loop_dt) {
        setOuterLoopSamplingPeriod(outer_loop_dt);
        last_outer_loop_dt = outer_loop_dt;
      }

      // Compute delta_theta using outer loop PID controller
      // The PID class handles:
      // - Filtered derivative (reduces noise sensitivity from encoder readings)
      // - Anti-windup (prevents integral saturation during large errors)
      // - Output saturation (limits delta_theta to ±MAX_DELTA_THETA)
      PID *outer_pid = getOuterPID(dof);
      if (outer_pid) {
        delta_theta[dof] = outer_pid->control(q_des, q_curr);
      }

      // Compute error for metrics tracking
      float error = q_des - q_curr;
      
      // === METRICS: Update tracking during movement ===
      if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
        MetricsTracker& mt = metrics_tracker[dof];
        uint32_t now = millis();
        
        // Only update metrics after movement has actually started
        // (movement_start_ms is the first waypoint's t_arrival, which may be in the future)
        if (now >= mt.movement_start_ms) {
          float abs_error = fabs(error);
          uint32_t elapsed_ms = now - mt.movement_start_ms;
          
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
        }  // End of: if (now >= mt.movement_start_ms)
      }
    }
    
    // === INNER LOOP @ 500 Hz (Motor Control) ===
    // Run at full 500 Hz in both MOVING and HOLDING modes for best accuracy.
    // The MCP2515 flush mechanism in LKM_Motor handles any CAN buffer issues.
    // See: LKM_Motor::getMultiAngleSync() for the flush and retry logic.

    // Use cached motor pointers (populated on first access per DOF)
    if (!motor_cache_valid[dof]) {
      // Populate cache for this DOF
      cached_agonist[dof] = nullptr;
      cached_antagonist[dof] = nullptr;
      cached_pid_agonist[dof] = nullptr;
      cached_pid_antagonist[dof] = nullptr;
      cached_agonist_idx[dof] = -1;
      cached_antagonist_idx[dof] = -1;

      for (int i = 0; i < config.motor_count; i++) {
        if (config.motors[i].dof_index == dof) {
          if (config.motors[i].is_agonist) {
            cached_agonist[dof] = motors[i];
            cached_agonist_idx[dof] = i;
            cached_pid_agonist[dof] = pid_controllers[i];
          } else {
            cached_antagonist[dof] = motors[i];
            cached_antagonist_idx[dof] = i;
            cached_pid_antagonist[dof] = pid_controllers[i];
          }
        }
      }
      motor_cache_valid[dof] = true;
    }

    // Use cached pointers
    LKM_Motor *agonist = cached_agonist[dof];
    LKM_Motor *antagonist = cached_antagonist[dof];
    PID *pid_agonist = cached_pid_agonist[dof];
    PID *pid_antagonist = cached_pid_antagonist[dof];
    int agonist_idx = cached_agonist_idx[dof];
    int antagonist_idx = cached_antagonist_idx[dof];

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
      
      // Apply interpolation based on mode
      float smooth_progress = progress;
      if (waypoint_interpolation_mode == INTERPOLATION_COSINE) {
        smooth_progress = 0.5f * (1.0f - cosf(progress * M_PI));
      }
      theta_0_joint = prev_angle + (target_angle - prev_angle) * smooth_progress;
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
    // Uses time-window based detection: N errors within M ms triggers emergency stop
    // This is more robust than consecutive count - tolerates brief EMI glitches
    static float last_theta_A[MAX_DOFS] = {0};
    static float last_theta_B[MAX_DOFS] = {0};
    static bool first_read[MAX_DOFS] = {true, true, true};

    // Time-window error tracking: circular buffer of error timestamps per DOF
    #define CAN_ERROR_HISTORY_SIZE 8
    static uint32_t error_timestamps[MAX_DOFS][CAN_ERROR_HISTORY_SIZE] = {{0}};
    static uint8_t error_head[MAX_DOFS] = {0};  // Next write position in circular buffer

    // Helper lambda: count errors within time window
    auto countRecentErrors = [](uint8_t dof_idx, uint32_t now_ms, uint32_t window_ms) -> uint8_t {
      uint8_t count = 0;
      uint32_t cutoff = (now_ms > window_ms) ? (now_ms - window_ms) : 0;
      for (int i = 0; i < CAN_ERROR_HISTORY_SIZE; i++) {
        if (error_timestamps[dof_idx][i] > cutoff) {
          count++;
        }
      }
      return count;
    };

    if (!first_read[dof]) {
      float jump_A = abs(theta_A_curr - last_theta_A[dof]);
      float jump_B = abs(theta_B_curr - last_theta_B[dof]);

      // If motor angle jumped more than 30° in one cycle (2ms), something is wrong
      if (jump_A > 30.0f || jump_B > 30.0f) {
        // Record error timestamp in circular buffer
        uint32_t now_ms = millis();
        error_timestamps[dof][error_head[dof]] = now_ms;
        error_head[dof] = (error_head[dof] + 1) % CAN_ERROR_HISTORY_SIZE;

        uint8_t recent_errors = countRecentErrors(dof, now_ms, can_error_window_ms);

        LOG_ERROR("[Waypoint DIAG] DOF " + String(dof) + " MOTOR ANGLE JUMP (" +
                  String(recent_errors) + " errors in " + String(can_error_window_ms) + "ms)!");
        LOG_ERROR("  Agonist: " + String(last_theta_A[dof], 2) + " → " + String(theta_A_curr, 2) +
                  " (jump=" + String(jump_A, 2) + "°)");
        LOG_ERROR("  Antagonist: " + String(last_theta_B[dof], 2) + " → " + String(theta_B_curr, 2) +
                  " (jump=" + String(jump_B, 2) + "°)");

        // Trigger emergency stop if too many errors within time window
        if (recent_errors >= can_error_threshold) {
          LOG_ERROR("[Waypoint] DOF " + String(dof) + " - " + String(recent_errors) +
                    " CAN errors in " + String(can_error_window_ms) + "ms, EMERGENCY STOP!");
          stopAllMotors();
          waypoint_buffer_clear(dof);
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
          // Clear error history for this DOF
          for (int i = 0; i < CAN_ERROR_HISTORY_SIZE; i++) {
            error_timestamps[dof][i] = 0;
          }
          first_read[dof] = true;
          continue;
        }

        // Skip this cycle to avoid sending bad commands, use last known good values
        continue;
      }
      // Good reading - no need to reset anything, old errors expire naturally
    }
    
    last_theta_A[dof] = theta_A_curr;
    last_theta_B[dof] = theta_B_curr;
    first_read[dof] = false;
    
    // === UPDATE MOTOR ANGLE CACHE ===
    // This cache is used by checkMotorsInRange() to avoid redundant CAN reads
    // which were causing ~2ms delays per motor during safety checks
    cached_motor_angles.agonist[dof] = theta_A_curr;
    cached_motor_angles.antagonist[dof] = theta_B_curr;
    cached_motor_angles.valid[dof] = true;
    cached_motor_angles.last_update_ms = millis();
    
    // Inner PID for motors (compute torque commands)
    float command_A = pid_agonist->control(theta_A_ref, theta_A_curr);
    float command_B = pid_antagonist->control(theta_B_ref, theta_B_curr);
    
    // NOTE: Co-contraction is achieved through stiffness_ref parameter in cascade control
    // theta_A_ref = theta_0 + 0.5*delta_theta + 0.5*stiffness_ref
    // theta_B_ref = theta_0 + 0.5*delta_theta - 0.5*stiffness_ref
    // The stiffness_ref separates motor references, creating constant tension on both tendons.
    // Increase stiffness_ref (via UI) to reduce vibrations from slack tendons.
    
    // === NOTCH FILTER: Apply resonance suppression if enabled ===
    // Unlike general low-pass filtering, a notch filter only attenuates a narrow
    // frequency band (the mechanical resonance) with minimal phase delay elsewhere.
    // This avoids the instability issues seen with broad-spectrum filtering.
    {
      // Check if configuration changed
      if (notch_filter_config.config_changed) {
        // Reconfigure all filters with new parameters
        float sample_rate = 1000000.0f / inner_loop_period_us;  // Actual inner loop Hz
        for (int i = 0; i < MAX_DOFS * 2; i++) {
          torque_notch_filters[i].configure(
            notch_filter_config.center_freq_hz,
            sample_rate,
            notch_filter_config.quality
          );
          torque_notch_filters[i].setEnabled(notch_filter_config.enabled);
        }
        notch_filter_config.config_changed = false;
        notch_filters_initialized = true;
        LOG_INFO("[Notch] Configured: " + String(notch_filter_config.center_freq_hz, 1) + 
                 " Hz, Q=" + String(notch_filter_config.quality, 2) + 
                 ", enabled=" + String(notch_filter_config.enabled ? "YES" : "NO"));
      }
      
      // Apply filters if initialized
      if (notch_filters_initialized) {
        int filter_idx_A = dof * 2;
        int filter_idx_B = dof * 2 + 1;
        command_A = torque_notch_filters[filter_idx_A].process(command_A);
        command_B = torque_notch_filters[filter_idx_B].process(command_B);
      }
    }
    
    // Get torque limits from motor configuration
    float max_torque_A = config.motors[agonist_idx].max_torque;
    float max_torque_B = config.motors[antagonist_idx].max_torque;

    // === TORQUE RATE LIMITING ===
    // Limit how fast torque can change to reduce mechanical stress and vibrations
    // Rate is calculated based on torque_ramp_time_ms (0 = disabled)
    static float prev_command_A[MAX_DOFS] = {0};
    static float prev_command_B[MAX_DOFS] = {0};

    if (torque_ramp_time_ms > 0) {
      // Calculate max torque change per cycle
      // At 500 Hz with 100ms ramp time: 50 cycles to go 0→max
      // max_rate = max_torque / (ramp_time_ms / inner_loop_period_ms)
      //          = max_torque * inner_loop_period_us / (ramp_time_ms * 1000)
      float rate_A = max_torque_A * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);
      float rate_B = max_torque_B * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);

      // Apply rate limiting
      command_A = constrain(command_A, prev_command_A[dof] - rate_A, prev_command_A[dof] + rate_A);
      command_B = constrain(command_B, prev_command_B[dof] - rate_B, prev_command_B[dof] + rate_B);
    }

    // Store for next cycle (after rate limiting, before saturation)
    prev_command_A[dof] = command_A;
    prev_command_B[dof] = command_B;

    // Apply absolute torque limits
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
  
  // === PROFILING: Calculate cycle time ===
  {
    uint32_t cycle_end_us = time_us_32();
    cycle_time_us_last = cycle_end_us - profiling_start_us;
    
    // Update max (reset every ~10 seconds)
    if (cycle_time_us_last > cycle_time_us_max) {
      cycle_time_us_max = cycle_time_us_last;
    }
    
    // Exponential moving average (α = 0.1 for smoothing)
    cycle_time_us_avg = (cycle_time_us_avg * 9 + cycle_time_us_last) / 10;
    
    // Log periodically (every 5 seconds = 2500 cycles @ 500Hz)
    static uint16_t profiling_log_counter = 0;
    profiling_log_counter++;
    if (profiling_log_counter >= 2500) {
      profiling_log_counter = 0;
      LOG_INFO("[PROFILING] Cycle time: last=" + String(cycle_time_us_last) + "µs, " +
               "avg=" + String(cycle_time_us_avg) + "µs, " +
               "max=" + String(cycle_time_us_max) + "µs " +
               "(budget=" + String(inner_loop_period_us) + "µs)");
      
      // Check if we're over budget
      if (cycle_time_us_max > inner_loop_period_us) {
        LOG_WARN("[PROFILING] OVER BUDGET! Max " + String(cycle_time_us_max) + 
                 "µs > " + String(inner_loop_period_us) + "µs budget");
      }
      
      // Reset max for next period
      cycle_time_us_max = 0;
    }
  }
  
  return any_movement;
}
