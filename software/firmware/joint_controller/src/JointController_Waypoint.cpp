/**
 * @file JointController_Waypoint.cpp
 * @brief Waypoint-based trajectory execution with cascade control
 * 
 * Implementation follows CAN_SYSTEM_ARCHITECTURE.md section 6.2 with cascade
 * control (outer joint PID + inner motor PID) and continuous waypoint consumption.
 * 
 * Key features:
 * - Linear interpolation between waypoints (smoothness from waypoint density @ 50-100 Hz)
 * - Per-DOF waypoint buffers (independent control)
 * - State management: IDLE → MOVING → HOLDING
 * - Cascade control architecture:
 *   * SAMPLING_PERIOD = 2000 µs (2 ms) → 500 Hz
 *   * Outer PID runs every outer_loop_divisor cycles (default 1 = 500 Hz)
 *   * Inner PID @ 500 Hz (motor-level, computes torque commands)
 *   * theta_ref = theta_0 + cascade_correction
 * 
 * @see waypoint_buffer.h for buffer management
 * @see CAN_SYSTEM_ARCHITECTURE.md section 6.2 for detailed specification
 * @see CAN_SYSTEM_ARCHITECTURE.md for protocol specification
 */

#include "JointController.h"
#include <waypoint_buffer.h>
#include <Arduino.h>
#include <debug.h>
#include "main_common.h"  // For shared_dof_angles
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
static float delta_theta[MAX_DOFS] = {0};           // Current outer PID output (target)
static float delta_theta_prev[MAX_DOFS] = {0};      // Previous outer PID output (for interpolation)
static float velocity_filtered[MAX_DOFS] = {0};     // Filtered joint velocity (deg/s)
static float expected_velocity_cache[MAX_DOFS] = {0}; // Cached expected velocity for inner loop stiffness scaling
static float theta_A_ema[MAX_DOFS] = {0};            // EMA-filtered motor A angle
static float theta_B_ema[MAX_DOFS] = {0};            // EMA-filtered motor B angle
static bool ema_initialized[MAX_DOFS] = {false};     // EMA initialization flag
static float inner_tau_default[MAX_DOFS] = {0};       // Original PID tau per DOF (captured at first use)
static bool inner_tau_default_captured[MAX_DOFS] = {false};
static float joint_ema_value[MAX_DOFS] = {0};          // EMA-filtered joint angle
static bool joint_ema_initialized[MAX_DOFS] = {false}; // Joint EMA initialization flag
static uint32_t last_anti_slack_log_ms[MAX_DOFS] = {0};
// When outer_loop_divisor > 1, delta_theta changes every N cycles creating "steps".
// To smooth this, we interpolate between delta_theta_prev and delta_theta based on
// where we are in the current outer loop period. This eliminates vibrations caused
// by discontinuous reference changes while maintaining cascade control benefits.

#if CONTROLLER_DEBUG
struct WaypointMicroProfile {
  uint32_t accum_dof_us = 0;
  uint32_t accum_outer_us = 0;
  uint32_t accum_eq_us = 0;
  uint32_t accum_can_us = 0;
  uint32_t accum_pid_us = 0;
  uint32_t accum_safety_us = 0;
  uint32_t accum_torque_us = 0;   // setTorque pair (non-blocking SPI)
  uint32_t max_dof_us = 0;
  uint32_t max_outer_us = 0;
  uint32_t max_eq_us = 0;
  uint32_t max_can_us = 0;
  uint32_t max_pid_us = 0;
  uint32_t max_safety_us = 0;
  uint32_t max_torque_us = 0;
  uint32_t samples = 0;
  uint32_t safety_samples = 0;
  uint32_t last_log_ms = 0;
};
static WaypointMicroProfile wp_micro_profile;
static const uint32_t WP_MICRO_LOG_INTERVAL_MS = 2000;
static const uint16_t WP_MICRO_MIN_SAMPLES = 50;
#endif

// Safety check counter (for periodic motor checks in HOLDING mode)
static uint16_t safety_check_counter = 0;

// Track previous state to detect MOVING → HOLDING transitions
static WaypointState prev_dof_state[MAX_DOFS] = {WaypointState::IDLE};

// Track if PID state needs reset when transitioning IDLE/HOLDING → MOVING
static bool pid_reset_needed[MAX_DOFS] = {true, true, true};

// Track if inner PID needs bumpless initialization (set in outer section, consumed in inner section)
// This bridges the gap between the IDLE→MOVING detection (early in loop) and the inner PID
// execution (later, after motor refs and CAN readings are available).
static bool inner_pid_init_needed[MAX_DOFS] = {false};

// === CAN ERROR TRACKING (file scope for reset on new movement) ===
static float wp_last_theta_A[MAX_DOFS] = {0};
static float wp_last_theta_B[MAX_DOFS] = {0};

// === OSCILLATION DETECTION (safety feature) ===
// Detects dangerous oscillations (e.g., from too-high stiffness) and triggers emergency stop
struct OscillationDetector {
  float prev_error = 0.0f;           // Previous error value
  int8_t prev_error_sign = 0;        // Sign of previous error (+1, -1, 0)
  uint8_t sign_change_count = 0;     // Number of error sign changes
  uint32_t window_start_ms = 0;      // Start of detection window
  float max_error_in_window = 0.0f;  // Maximum absolute error in window
  float min_error_in_window = 0.0f;  // Minimum error (for amplitude calculation)
  bool oscillation_detected = false; // Flag to prevent repeated triggers
};
static OscillationDetector osc_detector[MAX_DOFS];

// Oscillation detection parameters (tunable)
static const uint32_t OSC_WINDOW_MS = 500;           // Time window to count sign changes
static const uint8_t OSC_MIN_SIGN_CHANGES = 4;       // Min sign changes to trigger (4 = 2 full oscillations)
static const float OSC_MIN_AMPLITUDE_DEG = 3.0f;     // Min oscillation amplitude to be considered dangerous
static const float OSC_MIN_ERROR_TO_CHECK = 1.0f;    // Don't check if error is very small

static bool wp_first_read[MAX_DOFS] = {true, true, true};
static CANErrorTracker wp_canErrorTracker;

// === WAYPOINT TRAJECTORY DUMP (debug diagnostic) ===
// Records consumed waypoints for post-movement validation.
// Stores angle + arrival_ms for each consumed waypoint per DOF.
static const uint16_t WP_DUMP_MAX = 250;  // max waypoints to record per DOF
struct WpDumpEntry {
  float angle_deg;
  uint32_t t_arrival_ms;
};
static WpDumpEntry wp_dump[MAX_DOFS][WP_DUMP_MAX];
static uint16_t wp_dump_count[MAX_DOFS] = {0};
volatile int8_t wp_dump_pending_dof = -1;  // -1 = no dump pending, 0..2 = DOF to dump

// === RE-ANCHOR CORRECTION STATE (per-DOF, consume side only) ===
// When wp_reanchor_interval > 0, every N consumed waypoints the firmware
// computes a timing correction to re-synchronize arrival times with wall clock.
static int32_t wp_reanchor_correction_ms[MAX_DOFS] = {0};
static uint16_t wp_reanchor_consumed_count[MAX_DOFS] = {0};

void wp_reanchor_reset_all() {
  for (uint8_t i = 0; i < MAX_DOFS; i++) {
    wp_reanchor_correction_ms[i] = 0;
    wp_reanchor_consumed_count[i] = 0;
  }
}

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
 * Following CAN_SYSTEM_ARCHITECTURE.md section 6.2:
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
  // Increment safety check counter with wrap to prevent overflow
  // Counter wraps at 1000 to stay well within uint16_t range
  safety_check_counter = (safety_check_counter + 1) % 1000;
  bool any_movement = false;
  
  uint32_t t_now = getAbsoluteTimeMs();

  // Read a consistent snapshot of shared DOF angles for this cycle
  SharedDofAngles dof_snapshot;
  readSharedDofAnglesSnapshot(dof_snapshot);
  const SharedDofAngles &dof_data = dof_snapshot;

  // === OUTER LOOP SCHEDULING ===
  uint8_t effective_divisor = (outer_loop_divisor < 1) ? 1 : outer_loop_divisor;
  const bool outer_cycle_due = ((cycle_count - 1) % effective_divisor) == 0;
  
  // Process each DOF independently
  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    
    // === EARLY EXIT: Skip if DOF is IDLE (no waypoints ever received) ===
    // This saves CPU time when no waypoint control is active
    WaypointState dof_state = waypoint_buffer_state(dof);
    if (dof_state == WaypointState::IDLE) {
      // Mark PID reset needed for when this DOF becomes active
      pid_reset_needed[dof] = true;
      inner_pid_init_needed[dof] = false;  // Clear any stale flag
      prev_dof_state[dof] = WaypointState::IDLE;
      compliance_state[dof].reset();
      velocity_filtered[dof] = 0.0f;
      expected_velocity_cache[dof] = 0.0f;
      ema_initialized[dof] = false;
      inner_tau_default_captured[dof] = false;
      joint_ema_initialized[dof] = false;
      continue; // Skip this DOF entirely
    }

#if CONTROLLER_DEBUG
    uint32_t dof_start_us = time_us_32();
#endif
    
    // === RESET PID STATE when transitioning to MOVING ===
    // Reset ONLY on IDLE → MOVING (new sequence starting from stopped state)
    // DO NOT reset on HOLDING → MOVING - preserve integral compensation for gravity/friction
    // This prevents the "spike" movement when resuming from HOLDING
    bool just_started_from_idle = (prev_dof_state[dof] == WaypointState::IDLE) &&
                                   (dof_state == WaypointState::MOVING);
    bool should_reset = pid_reset_needed[dof] && just_started_from_idle;
    
    if (should_reset) {
      // Bumpless transfer for outer loop PID controller:
      // Initialize state at current joint angle so P and D terms start at zero.
      // Since q_des ≈ q_curr for a HOLDING waypoint, error ≈ 0 and only the
      // integral term will contribute a tiny correction on the first cycle.
      float q_init = dof_data.valid[dof] ? dof_data.angles[dof] : 0.0f;
      PID *outer_pid_init = getOuterPID(dof);
      if (outer_pid_init) {
        outer_pid_init->initializeState(q_init, q_init, 0.0f);
      }
      delta_theta[dof] = 0.0f;
      delta_theta_prev[dof] = 0.0f;
      pid_reset_needed[dof] = false;

      // Flag inner PID for bumpless initialization (done later when motor data is available)
      inner_pid_init_needed[dof] = true;

      // Clear CAN error history for this DOF to prevent old errors from triggering false stops
      wp_canErrorTracker.clearErrors(dof);
      wp_first_read[dof] = true;  // Reset jump detection for clean start

      LOG_C1_DEBUG("[Waypoint] DOF " + String(dof) + " bumpless init (IDLE → MOVING) at " + String(q_init, 1) + "°");
    }

    // === TRAJECTORY DUMP: Reset on new movement ===
    if (prev_dof_state[dof] != WaypointState::MOVING && dof_state == WaypointState::MOVING) {
      wp_dump_count[dof] = 0;
    }

    // === METRICS: Initialize tracker for NEW movement (from IDLE or HOLDING) ===
    // Detect when a new movement starts:
    // - IDLE → MOVING: first movement ever
    // - HOLDING → MOVING: new movement after previous one completed
    // Use !tracking_active as additional guard to prevent multiple initializations
    bool new_movement_started = (prev_dof_state[dof] != WaypointState::MOVING) && 
                                 (dof_state == WaypointState::MOVING);

    if (new_movement_started) {
      compliance_state[dof].reset();
      velocity_filtered[dof] = 0.0f;
    }
    
    if (metrics_tracking_enabled && new_movement_started && dof < 3 && 
        !metrics_tracker[dof].tracking_active) {
      WaypointEntry first_wp;
      if (waypoint_buffer_peek(dof, first_wp)) {
        float start_angle = dof_data.valid[dof] ? dof_data.angles[dof] : 0.0f;
        // Pass the first waypoint's arrival time so duration is measured from actual movement start
        metrics_tracker[dof].reset(start_angle, first_wp.target_angle_deg, first_wp.t_arrival_ms);
        LOG_C1_DEBUG("[Metrics] DOF " + String(dof) + " tracking started: " + 
                  String(start_angle, 2) + "° → " + String(first_wp.target_angle_deg, 2) + 
                  "° (t_arrival=" + String(first_wp.t_arrival_ms) + ")");
      }
    }
    
    // === CHECK WAYPOINT TRANSITION ===
    // Check if we've reached the current waypoint target
    WaypointEntry next_waypoint;
    if (waypoint_buffer_peek(dof, next_waypoint)) {
      // Apply re-anchor correction to arrival time (consume-side drift compensation)
      uint32_t effective_arrival = next_waypoint.t_arrival_ms;
      if (wp_reanchor_correction_ms[dof] != 0) {
        effective_arrival = (uint32_t)((int32_t)next_waypoint.t_arrival_ms + wp_reanchor_correction_ms[dof]);
      }

      // Wrap-safe: signed difference handles uint32_t overflow (valid for <24.8 days)
      if ((int32_t)(t_now - effective_arrival) >= 0) {
        // Waypoint reached - pop from buffer and update prev state
        waypoint_buffer_pop(dof);
        // Set prev to EFFECTIVE arrival so interpolation sees corrected timestamps
        waypoint_buffer_set_prev(dof, next_waypoint.target_angle_deg, effective_arrival);

        // Record consumed waypoint with effective arrival for diagnostic visibility
        if (dof < MAX_DOFS && wp_dump_count[dof] < WP_DUMP_MAX) {
          wp_dump[dof][wp_dump_count[dof]] = {next_waypoint.target_angle_deg, effective_arrival};
          wp_dump_count[dof]++;
        }

        // Re-anchor interval check: every N consumed WPs, recompute correction
        wp_reanchor_consumed_count[dof]++;
        uint16_t interval = wp_reanchor_interval;  // Read volatile once
        if (interval > 0 && wp_reanchor_consumed_count[dof] >= interval) {
          WaypointEntry peek_reanchor;
          if (waypoint_buffer_peek(dof, peek_reanchor)) {
            int32_t new_correction = (int32_t)(t_now - peek_reanchor.t_arrival_ms);
            if (new_correction > 0) {
              wp_reanchor_correction_ms[dof] = new_correction;
              LOG_C1_INFO("[Reanchor] DOF " + String(dof) + " correction=" +
                          String(new_correction) + "ms after " + String(interval) + " WPs");
            } else {
              // Drift gone or negative: clear any stale positive correction
              wp_reanchor_correction_ms[dof] = 0;
              LOG_C1_INFO("[Reanchor] DOF " + String(dof) + " no drift (delta=" +
                          String(new_correction) + "ms) after " + String(interval) + " WPs");
            }
          }
          wp_reanchor_consumed_count[dof] = 0;
        }

        // Check if more waypoints available
        WaypointEntry peek_next;
        if (!waypoint_buffer_peek(dof, peek_next)) {
          // No more waypoints - this was the LAST waypoint
          // prev_angle is now set to this waypoint's target
          float final_target = waypoint_buffer_prev_angle(dof);
          LOG_C1_INFO("[Waypoint] DOF " + String(dof) + " LAST waypoint consumed: " +
                    String(next_waypoint.target_angle_deg, 2) + "° → prev_angle=" +
                    String(final_target, 2) + "°");
        }
      }
    }
    
    // === OUTER LOOP (Joint PID) ===
    // Execute outer loop every N inner cycles (configurable via outer_loop_divisor)
    if (outer_cycle_due) {

#if CONTROLLER_DEBUG
      uint32_t outer_start_us = time_us_32();
#endif
      
      float q_des = 0.0f;
      float expected_velocity_deg_s = 0.0f;
      bool is_moving = false;
      
      // Use last known waypoint as reference
      float prev_angle = waypoint_buffer_prev_angle(dof);
      uint32_t prev_time = waypoint_buffer_prev_time(dof);
      
      // Check if we have waypoints to process
      WaypointEntry current_target;
      bool has_waypoints = waypoint_buffer_peek(dof, current_target);
      if (has_waypoints) {
        // MOVING state - linear interpolation
        is_moving = true;
        any_movement = true;

        float target_angle = current_target.target_angle_deg;
        // Apply re-anchor correction to target arrival (consistent with prev_time which was set to effective_arrival)
        uint32_t corrected_arrival = current_target.t_arrival_ms;
        if (wp_reanchor_correction_ms[dof] != 0) {
          corrected_arrival = (uint32_t)((int32_t)current_target.t_arrival_ms + wp_reanchor_correction_ms[dof]);
        }
        float time_total = corrected_arrival - prev_time;
        float time_elapsed = t_now - prev_time;
        
        // Compute progress (0.0 to 1.0)
        float progress = 0.0f;
        if (time_total > 0) {
          progress = time_elapsed / time_total;
          progress = constrain(progress, 0.0f, 1.0f);
          expected_velocity_deg_s = (target_angle - prev_angle) * 1000.0f / time_total;
        }
        
        // NOTE: Trajectory diagnostic logging removed to reduce serial overhead
        // Only errors > 5° are logged (see below)
        
        // === ERROR DETECTION: Log only significant tracking errors ===
        if (dof_data.valid[dof]) {
          float q_curr_now = dof_data.angles[dof];
          float tracking_error = fabs(target_angle - q_curr_now);
          if (tracking_error > 5.0f && dof == 0) {
            static uint32_t last_error_log = 0;
            if (millis() - last_error_log > 500) { // Max 1 log per 500ms
              LOG_C1_WARN("[TRAJ] DOF" + String(dof) + " LARGE ERROR: " + 
                       String(tracking_error, 1) + "° (tgt=" + String(target_angle, 1) + 
                       " cur=" + String(q_curr_now, 1) + ")");
              last_error_log = millis();
            }
          }
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
        q_des = prev_angle;
      }
      
      // Read current angle from shared state (updated by Core0)
      if (!dof_data.valid[dof]) {
        LOG_C1_WARN("[Waypoint] Invalid encoder reading for DOF " + String(dof));
        continue;
      }
      float q_curr = dof_data.angles[dof];
      
      // === COMPLIANCE DETECTION (Deflection / Stall) ===
      ComplianceState &cs = compliance_state[dof];
      float error = q_des - q_curr;
      float abs_error = fabs(error);

      // Filter actual velocity (deg/s) - only update if encoder reading is valid
      // If invalid, keep previous filtered value (graceful degradation)
      if (dof_data.valid[dof]) {
        float actual_velocity = dof_data.velocities[dof];
        uint8_t samples = velocity_filter_samples;
        if (samples < 1) samples = 1;
        float alpha = 1.0f / (float)samples;
        velocity_filtered[dof] = (1.0f - alpha) * velocity_filtered[dof] + alpha * actual_velocity;
      }
      // Note: if invalid, velocity_filtered[dof] retains its previous value

      // === JOINT ANGLE EMA FILTER (before outer PID) ===
      // At low speeds, smooth joint encoder to prevent outer PID from amplifying micro-variations.
      // Disabled at high speed to preserve tracking responsiveness.
      if (joint_ema_enabled && joint_ema_alpha < 1.0f) {
        float speed = fabs(velocity_filtered[dof]);
        if (speed <= joint_ema_speed_threshold) {
          if (!joint_ema_initialized[dof]) {
            joint_ema_value[dof] = q_curr;
            joint_ema_initialized[dof] = true;
          } else {
            float a = joint_ema_alpha;
            joint_ema_value[dof] = a * q_curr + (1.0f - a) * joint_ema_value[dof];
          }
          q_curr = joint_ema_value[dof];
        } else {
          // High speed: passthrough, but keep EMA state updated for smooth transition
          joint_ema_value[dof] = q_curr;
        }
      }

      bool expected_holding = fabs(expected_velocity_deg_s) <= expected_velocity_deadband_deg_s;
      bool compliance_should_activate = false;

      if (!cs.compliance_active) {
        if (expected_holding) {
          // HOLDING / fixed target: detect external deflection by error only
          if (abs_error > hold_error_threshold_deg) {
            if (cs.hold_candidate_start_ms == 0) {
              cs.hold_candidate_start_ms = t_now;
            }
            if (t_now - cs.hold_candidate_start_ms >= hold_time_threshold_ms) {
              compliance_should_activate = true;
            }
          } else if (abs_error < hold_release_threshold_deg) {
            cs.hold_candidate_start_ms = 0;
          }
          cs.move_candidate_start_ms = 0;
          cs.release_candidate_start_ms = 0;
        } else {
          // MOVING: detect stall when actual velocity is far below expected
          cs.hold_candidate_start_ms = 0;
          cs.release_candidate_start_ms = 0;
          float expected_speed = fabs(expected_velocity_deg_s);
          bool stalled = (abs_error > move_error_threshold_deg) &&
                         (fabs(velocity_filtered[dof]) < expected_speed * move_velocity_ratio);
          if (stalled) {
            if (cs.move_candidate_start_ms == 0) {
              cs.move_candidate_start_ms = t_now;
            }
            if (t_now - cs.move_candidate_start_ms >= move_time_threshold_ms) {
              compliance_should_activate = true;
            }
          } else {
            cs.move_candidate_start_ms = 0;
          }
        }

        if (compliance_should_activate) {
          cs.compliance_active = true;
          cs.stall_entry_angle_deg = q_curr;
          cs.original_target_deg = q_des;
          cs.stall_count++;
          cs.last_stall_ms = t_now;
          cs.hold_candidate_start_ms = 0;
          cs.move_candidate_start_ms = 0;

          const char *mode_label = expected_holding ? "HOLDING" : "MOVING";
          LOG_C1_INFO("[Compliance] DOF " + String(dof) + " ACTIVE mode=" + String(mode_label) +
                   " err=" + String(error, 2) + "deg exp_v=" + String(expected_velocity_deg_s, 2) +
                   "deg/s act_v=" + String(velocity_filtered[dof], 2) + "deg/s");

          // If stall detected during MOVING, abort trajectory and switch to HOLDING
          // The host will decide what to do next (new trajectory, give up, etc.)
          if (!expected_holding) {
            if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
              metrics_tracker[dof].aborted_by_stall = true;
              metrics_tracker[dof].abort_target_deg = cs.original_target_deg;
            }
            waypoint_buffer_clear(dof);
            waypoint_buffer_set_prev(dof, q_curr, t_now);
            waypoint_buffer_set_state(dof, WaypointState::HOLDING);
            dof_state = WaypointState::HOLDING;
            has_waypoints = false;
            q_des = q_curr;
            error = 0.0f;
            abs_error = 0.0f;

            LOG_C1_INFO("[Compliance] DOF " + String(dof) + 
                     " STALL->HOLDING: trajectory aborted, holding at " + String(q_curr, 2) + "deg");
            
            // Notify host that trajectory was aborted due to stall
            // Host should stop sending waypoints and handle the situation
            SERIAL_C1_COM_LN("EVT:STALL_ABORT:DOF=" + String(dof) + ":ANGLE=" + String(q_curr, 2));
          }
        }
      } else {
        // Compliance active: check release condition
        bool compliance_should_release = false;
        if (expected_holding) {
          // Error-based release: requires BOTH low error AND velocity below max threshold
          // This prevents releasing while "flying through" the target position
          bool release_by_error = (abs_error < hold_release_threshold_deg) &&
                                  (fabs(velocity_filtered[dof]) < hold_release_max_velocity_deg_s);
          // Velocity-based release: requires velocity settled for hold_release_time_ms
          bool release_by_velocity = false;
          if (fabs(velocity_filtered[dof]) < hold_release_velocity_deg_s) {
            if (cs.release_candidate_start_ms == 0) {
              cs.release_candidate_start_ms = t_now;
            }
            if (t_now - cs.release_candidate_start_ms >= hold_release_time_ms) {
              release_by_velocity = true;
            }
          } else {
            cs.release_candidate_start_ms = 0;
          }
          compliance_should_release = release_by_error || release_by_velocity;
          if (compliance_should_release) {
            String reason = release_by_error && release_by_velocity ? "HOLD_ERR+VEL"
                            : (release_by_velocity ? "HOLD_VEL" : "HOLD_ERR");
            LOG_C1_INFO("[Compliance] DOF " + String(dof) + " RELEASE reason=" + reason +
                     " err=" + String(error, 2) + "deg exp_v=" + String(expected_velocity_deg_s, 2) +
                     "deg/s act_v=" + String(velocity_filtered[dof], 2) + "deg/s");
          }
        } else {
          float expected_speed = fabs(expected_velocity_deg_s);
          bool release_by_error = (abs_error < move_error_threshold_deg);
          bool release_by_velocity = (fabs(velocity_filtered[dof]) >= expected_speed * move_velocity_ratio);
          if (release_by_error || release_by_velocity) {
            compliance_should_release = true;
            String reason = release_by_error && release_by_velocity ? "MOVE_ERR+VEL"
                            : (release_by_velocity ? "MOVE_VEL" : "MOVE_ERR");
            LOG_C1_INFO("[Compliance] DOF " + String(dof) + " RELEASE reason=" + reason +
                     " err=" + String(error, 2) + "deg exp_v=" + String(expected_velocity_deg_s, 2) +
                     "deg/s act_v=" + String(velocity_filtered[dof], 2) + "deg/s");
          }
        }

        if (compliance_should_release) {
          cs.compliance_active = false;
          cs.hold_candidate_start_ms = 0;
          cs.move_candidate_start_ms = 0;
          cs.release_candidate_start_ms = 0;

          // Teach mode: keep the current position as new target
          if (recovery_policy == RECOVERY_STAY_AT_CURRENT) {
            waypoint_buffer_clear(dof);
            waypoint_buffer_set_prev(dof, q_curr, t_now);
            waypoint_buffer_set_state(dof, WaypointState::HOLDING);
            dof_state = WaypointState::HOLDING;
            has_waypoints = false;
            q_des = q_curr;
            error = 0.0f;
            abs_error = 0.0f;

            // Reset outer PID to avoid integral bump after deflection
            resetOuterPID(dof);
            delta_theta[dof] = 0.0f;
            delta_theta_prev[dof] = 0.0f;
          }
        }
      }

      // === OSCILLATION DETECTION (safety feature) ===
      // Detects dangerous oscillations (e.g., from too-high stiffness) and triggers emergency stop
      // An oscillation is detected when:
      // 1. Error changes sign frequently (multiple times in short window)
      // 2. Oscillation amplitude exceeds threshold
      if (is_moving && abs_error > OSC_MIN_ERROR_TO_CHECK) {
        OscillationDetector &od = osc_detector[dof];
        uint32_t now_ms = millis();
        
        // Determine error sign
        int8_t error_sign = (error > 0.1f) ? 1 : ((error < -0.1f) ? -1 : 0);
        
        // Reset window if expired
        if (now_ms - od.window_start_ms > OSC_WINDOW_MS) {
          od.window_start_ms = now_ms;
          od.sign_change_count = 0;
          od.max_error_in_window = abs_error;
          od.min_error_in_window = abs_error;
        }
        
        // Track amplitude in window
        if (abs_error > od.max_error_in_window) od.max_error_in_window = abs_error;
        if (abs_error < od.min_error_in_window) od.min_error_in_window = abs_error;
        
        // Detect sign change
        if (error_sign != 0 && od.prev_error_sign != 0 && error_sign != od.prev_error_sign) {
          od.sign_change_count++;
        }
        od.prev_error_sign = error_sign;
        od.prev_error = error;
        
        // Calculate oscillation amplitude (peak-to-peak / 2)
        float osc_amplitude = (od.max_error_in_window - od.min_error_in_window) / 2.0f;
        
        // Check if oscillation is dangerous
        if (!od.oscillation_detected && 
            od.sign_change_count >= OSC_MIN_SIGN_CHANGES && 
            osc_amplitude >= OSC_MIN_AMPLITUDE_DEG) {
          
          LOG_C1_ERROR("[OSCILLATION SAFETY] DOF " + String(dof) + 
                    " DANGEROUS OSCILLATION DETECTED!");
          LOG_C1_ERROR("  Sign changes: " + String(od.sign_change_count) + 
                    " in " + String(now_ms - od.window_start_ms) + "ms");
          LOG_C1_ERROR("  Amplitude: " + String(osc_amplitude, 1) + 
                    "° (threshold: " + String(OSC_MIN_AMPLITUDE_DEG, 1) + "°)");
          
          // Trigger emergency stop
          od.oscillation_detected = true;
          stopAllMotors();
          
          // Reset waypoint state
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
          waypoint_buffer_clear(dof);
          prev_dof_state[dof] = WaypointState::IDLE;
          pid_reset_needed[dof] = true;
          
          // Notify host via serial
          SERIAL_C1_COM_LN("⚠️ OSCILLATION_STOP:DOF=" + String(dof) + 
                        ":AMPLITUDE=" + String(osc_amplitude, 1) +
                        ":SIGN_CHANGES=" + String(od.sign_change_count));
          
          continue; // Skip to next DOF
        }
      } else {
        // Reset oscillation detector when not in danger zone
        OscillationDetector &od = osc_detector[dof];
        if (od.oscillation_detected && abs_error < OSC_MIN_ERROR_TO_CHECK * 0.5f) {
          od.oscillation_detected = false; // Allow re-detection after recovery
        }
        od.sign_change_count = 0;
        od.window_start_ms = millis();
      }

      // === RUNTIME SAFETY CHECK ===
      // Check joint limits, mapping limits, and optionally motor limits (tendon breakage)
      // - MOVING mode: check every outer loop cycle (default 500 Hz)
      // - HOLDING mode: check immediately on transition, then every 10 cycles (period depends on outer loop rate)
      
      // Determine current state based on waypoint buffer
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
          LOG_C1_DEBUG("[Waypoint] DOF " + String(dof) + " transitioned MOVING → HOLDING");

          // Send structured message for UI display
          SERIAL_C1_COM_LN("EVT:HOLDING_TARGET:DOF=" + String(dof) + ":ANGLE=" + String(holding_target, 2));

          // === TRAJECTORY DUMP: Signal Core0 to print the detailed dump ===
          // Core1 must NOT do heavy serial I/O — it blocks CAN polling.
          // Set a flag so Core0 prints the dump at its own pace.
          if (wp_dump_count[dof] > 0) {
            // Quick 1-line summary is safe from Core1 (single print)
            char sum[100];
            uint32_t total_dt = wp_dump[dof][wp_dump_count[dof]-1].t_arrival_ms
                              - wp_dump[dof][0].t_arrival_ms;
            snprintf(sum, sizeof(sum), "[WP_DUMP] DOF %d: %d pts, total_dt=%lums — dump pending on Core0",
                     dof, wp_dump_count[dof], (unsigned long)total_dt);
            LOG_C1_INFO(sum);
            // Signal Core0 to print the full dump
            wp_dump_pending_dof = dof;
          }
          
          // DO NOT reset PID integral here - we need it to maintain position
          // against static loads (gravity, friction). The integral was compensating
          // for steady-state error, and resetting it would cause drift.
          // PID will be reset only when a new sequence starts from IDLE.
          
          // === METRICS: Finalize movement metrics ===
          if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
            MetricsTracker& mt = metrics_tracker[dof];
            
            // Update final target (in case it changed during movement)
            float original_target = mt.target_angle_deg;
            float metrics_target = holding_target;
            if (mt.aborted_by_stall) {
              metrics_target = mt.abort_target_deg;
            }
            mt.target_angle_deg = metrics_target;
            
            // Finalize and store metrics
            MovementMetrics m = mt.finalize();
            m.dof_index = dof;
            last_movement_metrics[dof] = m;
            metrics_ready[dof] = true;
            
            // Stop tracking
            mt.tracking_active = false;
            
            // Detailed logging for debugging
            LOG_C1_INFO("[Metrics] DOF " + String(dof) + " FINAL:");
            LOG_C1_INFO("  start=" + String(mt.start_angle_deg, 2) + 
                     "° → target=" + String(metrics_target, 2) + 
                     "° (orig=" + String(original_target, 2) + "°)");
            LOG_C1_INFO("  rise=" + String(m.rise_time_ms) + "ms (90%=" + 
                     String(mt.reached_90_percent ? "yes" : "no") + ")");
            LOG_C1_INFO("  settle=" + String(m.settling_time_ms) + "ms, max_err=" + 
                     String(mt.max_error_deg, 2) + "°");
            LOG_C1_INFO("  overshoot=" + String(m.overshoot_x100 / 100.0f, 1) + 
                     "% (max=" + String(mt.max_overshoot_deg, 2) + 
                     "°, dir=" + String(mt.movement_direction, 0) + ")");
            LOG_C1_INFO("  sse=" + String(m.sse_x100 / 100.0f, 2) + 
                     "° (samples=" + String(mt.sse_sample_count) + ")");
          }
        }
      }
      
      // Determine if we should check safety:
      // - Always check joint limits in MOVING mode (every outer loop cycle = 100 Hz)
      // - Check periodically in HOLDING mode (every 10 cycles = ~100ms at 100Hz)
      // NOTE: We do NOT check immediately when entering HOLDING because motors may still be settling
      // Reduced from 20 cycles (200ms) to 10 cycles (100ms) for faster detection during manual push
      bool should_check_safety = has_waypoints || (is_holding && safety_check_counter >= 10);
      
      if (should_check_safety) {
#if CONTROLLER_DEBUG
        uint32_t safety_start_us = time_us_32();
#endif
        String safety_message;
        // Check motors (tendon breakage) only in HOLDING mode periodically
        // NOT immediately when entering HOLDING - motors need time to settle
        bool check_motors = is_holding && (safety_check_counter >= 10);
        
        // NOTE: Debug log for periodic safety check was removed here because
        // String concatenation + Serial.print was causing ~2ms overhead per call,
        // which exceeded the 2ms cycle budget and caused control loop jitter.
        
        bool safety_ok = checkSafetyForDof(dof, q_curr, safety_message, check_motors);
#if CONTROLLER_DEBUG
        {
          uint32_t safety_dt = time_us_32() - safety_start_us;
          wp_micro_profile.accum_safety_us += safety_dt;
          wp_micro_profile.safety_samples++;
          if (safety_dt > wp_micro_profile.max_safety_us) {
            wp_micro_profile.max_safety_us = safety_dt;
          }
        }
#endif
        if (!safety_ok) {
          // Safety violation detected - stop all motors immediately
          stopAllMotors();
          LOG_C1_ERROR("[Waypoint Safety] MOVEMENT STOPPED: " + safety_message);
          
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
      float outer_loop_dt = effective_divisor * inner_loop_period_us / 1000000.0f;
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
        // Save previous value for interpolation (smooth transitions when divisor > 1)
        delta_theta_prev[dof] = delta_theta[dof];

        // Cache expected velocity for inner loop stiffness scaling
        expected_velocity_cache[dof] = expected_velocity_deg_s;

        // Normal PID control - compute delta_theta
        delta_theta[dof] = outer_pid->control(q_des, q_curr);

        // Store outer PID term breakdown for diagnostics (DOF 0 only)
        if (dof == 0 && pid_diag_terms_enabled) {
          // Values scaled ×100 for int16 transmission (incremental terms are small floats)
          pid_diagnostics.outer_p_term = (int16_t)constrain(outer_pid->last_up * 100.0f, -32767, 32767);
          pid_diagnostics.outer_i_term = (int16_t)constrain(outer_pid->last_ui * 100.0f, -32767, 32767);
          pid_diagnostics.outer_d_term = (int16_t)constrain(outer_pid->last_udfilt * 100.0f, -32767, 32767);
          pid_diagnostics.outer_output = (int16_t)(delta_theta[dof] * 100.0f);
        }
      }

      // Error already computed above (used for compliance detection)
      
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

#if CONTROLLER_DEBUG
      {
        uint32_t outer_dt = time_us_32() - outer_start_us;
        wp_micro_profile.accum_outer_us += outer_dt;
        if (outer_dt > wp_micro_profile.max_outer_us) {
          wp_micro_profile.max_outer_us = outer_dt;
        }
      }
#endif
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
      // Apply re-anchor correction (consistent with prev_time set to effective_arrival)
      uint32_t corrected_arrival = current_target.t_arrival_ms;
      if (wp_reanchor_correction_ms[dof] != 0) {
        corrected_arrival = (uint32_t)((int32_t)current_target.t_arrival_ms + wp_reanchor_correction_ms[dof]);
      }
      float time_total = corrected_arrival - prev_time;
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
#if CONTROLLER_DEBUG
    uint32_t eq_start_us = time_us_32();
#endif
    bool equations_ok = calculateMotorAnglesWithEquations(dof, theta_0_joint, theta_0_joint,
                                                          theta_0_agonist_motor, theta_0_antagonist_motor);
#if CONTROLLER_DEBUG
    {
      uint32_t eq_dt = time_us_32() - eq_start_us;
      wp_micro_profile.accum_eq_us += eq_dt;
      if (eq_dt > wp_micro_profile.max_eq_us) {
        wp_micro_profile.max_eq_us = eq_dt;
      }
    }
#endif
    
    if (!equations_ok) {
      // No linear equations, cannot control this DOF
      static uint32_t last_warn_time = 0;
      if (millis() - last_warn_time > 5000) {
        LOG_C1_WARN("[Waypoint] DOF " + String(dof) + " has no linear equations, cannot move");
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
    
    // === DELTA_THETA INTERPOLATION ===
    // When outer_loop_divisor > 1, delta_theta updates every N cycles creating "steps".
    // To avoid vibrations from discontinuous reference changes, we linearly interpolate
    // between the previous and current delta_theta values based on where we are in the
    // outer loop period. When divisor = 1, alpha = 1.0 so no interpolation occurs.
    float delta_theta_smooth;
    if (effective_divisor <= 1) {
      // No interpolation needed (outer and inner at same frequency)
      delta_theta_smooth = delta_theta[dof];
    } else {
      // Interpolate: cycle_in_outer goes from 0 to (divisor-1)
      // alpha goes from 1/divisor to 1.0 (we use new value immediately, blend out old)
      int cycle_in_outer = (cycle_count - 1) % effective_divisor;
      float alpha = (float)(cycle_in_outer + 1) / (float)effective_divisor;
      delta_theta_smooth = delta_theta_prev[dof] + alpha * (delta_theta[dof] - delta_theta_prev[dof]);
    }
    
    // === VELOCITY-DEPENDENT STIFFNESS SCALING ===
    // At low speeds, reduce co-contraction (stiffness) to avoid tendon-fighting oscillations.
    // delta_theta (tracking) stays at full cascade_influence for position accuracy.
    float stiffness_eff = stiffness_ref;
    if (cascade_speed_scaling_enabled) {
      float speed = fabs(expected_velocity_cache[dof]);
      float stiffness_factor;
      if (speed <= cascade_speed_low) {
        stiffness_factor = cascade_min_factor;
      } else if (speed >= cascade_speed_high) {
        stiffness_factor = 1.0f;
      } else {
        stiffness_factor = cascade_min_factor + (1.0f - cascade_min_factor) *
                          (speed - cascade_speed_low) / (cascade_speed_high - cascade_speed_low);
      }
      stiffness_eff = stiffness_ref * stiffness_factor;
    }

    // Compute motor references using cascade control formula
    // delta_theta at full cascade, stiffness scaled by velocity
    float theta_A_ref = theta_0_agonist_motor +
                        cascade_influence * (0.5f * delta_theta_smooth + 0.5f * stiffness_eff);
    float theta_B_ref = theta_0_antagonist_motor +
                        cascade_influence * (0.5f * delta_theta_smooth - 0.5f * stiffness_eff);

    // === ANTI-SLACK CLAMP (active during compliance) ===
    if (anti_slack_enabled && compliance_state[dof].compliance_active) {
      float theta_A_ref_before = theta_A_ref;
      float theta_B_ref_before = theta_B_ref;
      float expected_A = 0.0f;
      float expected_B = 0.0f;
      float q_curr_inner = dof_data.angles[dof];
      if (calculateMotorAnglesWithEquations(dof, q_curr_inner, q_curr_inner, expected_A, expected_B)) {
        const float DELTA_THETA_EPS = 0.01f;
        if (delta_theta_smooth > DELTA_THETA_EPS) {
          // Agonist pulling, antagonist releasing
          theta_B_ref = constrain(theta_B_ref,
                                  expected_B - anti_slack_margin_deg,
                                  expected_B + anti_slack_margin_deg);
        } else if (delta_theta_smooth < -DELTA_THETA_EPS) {
          // Antagonist pulling, agonist releasing
          theta_A_ref = constrain(theta_A_ref,
                                  expected_A - anti_slack_margin_deg,
                                  expected_A + anti_slack_margin_deg);
        } else {
          // Near zero delta_theta: clamp both to prevent slack
          theta_A_ref = constrain(theta_A_ref,
                                  expected_A - anti_slack_margin_deg,
                                  expected_A + anti_slack_margin_deg);
          theta_B_ref = constrain(theta_B_ref,
                                  expected_B - anti_slack_margin_deg,
                                  expected_B + anti_slack_margin_deg);
        }

        bool clamped = (fabs(theta_A_ref - theta_A_ref_before) > 0.001f) ||
                       (fabs(theta_B_ref - theta_B_ref_before) > 0.001f);
        if (clamped) {
          uint32_t now_ms = millis();
          if (now_ms - last_anti_slack_log_ms[dof] > 500) {
            LOG_C1_INFO("[AntiSlack] DOF " + String(dof) +
                     " Aref=" + String(theta_A_ref, 2) +
                     " Bref=" + String(theta_B_ref, 2) +
                     " expA=" + String(expected_A, 2) +
                     " expB=" + String(expected_B, 2));
            last_anti_slack_log_ms[dof] = now_ms;
          }
        }
      }
    }
    
    // Read current motor angles
#if CONTROLLER_DEBUG
    uint32_t can_start_us = time_us_32();
#endif
    PipelinedAngleData pipelined = LKM_Motor::getMultiAnglePairPipelined(agonist, antagonist);
    MultiAngleData data_A = pipelined.dataA;
    MultiAngleData data_B = pipelined.dataB;
#if CONTROLLER_DEBUG
    {
      uint32_t can_dt = time_us_32() - can_start_us;
      wp_micro_profile.accum_can_us += can_dt;
      if (can_dt > wp_micro_profile.max_can_us) {
        wp_micro_profile.max_can_us = can_dt;
      }
    }
#endif
    float theta_A_curr = data_A.angle;
    float theta_B_curr = data_B.angle;
    
    // === SANITY CHECK: Detect obviously invalid readings ===
    // Values outside ±100000° are clearly garbage (CAN corruption)
    bool invalid_A = (theta_A_curr < -100000.0f || theta_A_curr > 100000.0f || isnan(theta_A_curr));
    bool invalid_B = (theta_B_curr < -100000.0f || theta_B_curr > 100000.0f || isnan(theta_B_curr));
    
    if (invalid_A || invalid_B) {
      static uint32_t last_invalid_log = 0;
      if (millis() - last_invalid_log > 100) { // Log max every 100ms
        LOG_C1_ERROR("[Waypoint] DOF " + String(dof) + " INVALID CAN READ: A=" + 
                  String(theta_A_curr, 2) + " B=" + String(theta_B_curr, 2));
        last_invalid_log = millis();
      }
      // Skip this cycle entirely - don't send any torque command
      continue;
    }
    
    // === DIAGNOSTIC: Detect suspicious motor readings ===
    // Check for sudden large jumps in motor angle (possible CAN corruption)
    // Uses time-window based detection via shared CANErrorTracker (main_common.h)
    // Variables are at file scope (wp_last_theta_A/B, wp_first_read, wp_canErrorTracker)
    // so they can be reset when a new waypoint sequence starts

    if (!wp_first_read[dof]) {
      float jump_A = fabs(theta_A_curr - wp_last_theta_A[dof]);
      float jump_B = fabs(theta_B_curr - wp_last_theta_B[dof]);

      // If motor angle jumped more than 30° in one cycle (2ms), something is wrong
      if (jump_A > 30.0f || jump_B > 30.0f) {
        wp_canErrorTracker.recordError(dof);
        uint8_t recent_errors = wp_canErrorTracker.countRecentErrors(dof);

        LOG_C1_ERROR("[Waypoint DIAG] DOF " + String(dof) + " MOTOR ANGLE JUMP (" +
                  String(recent_errors) + " errors in " + String(can_error_window_ms) + "ms)!");
        LOG_C1_ERROR("  Agonist: " + String(wp_last_theta_A[dof], 2) + " → " + String(theta_A_curr, 2) +
                  " (jump=" + String(jump_A, 2) + "°)");
        LOG_C1_ERROR("  Antagonist: " + String(wp_last_theta_B[dof], 2) + " → " + String(theta_B_curr, 2) +
                  " (jump=" + String(jump_B, 2) + "°)");

        // Trigger emergency stop if too many errors within time window
        if (wp_canErrorTracker.shouldStop(dof)) {
          LOG_C1_ERROR("[Waypoint] DOF " + String(dof) + " - " + String(recent_errors) +
                    " CAN errors in " + String(can_error_window_ms) + "ms, EMERGENCY STOP!");
          stopAllMotors();
          waypoint_buffer_clear(dof);
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
          wp_canErrorTracker.clearErrors(dof);
          wp_first_read[dof] = true;
          continue;
        }

        // Skip this cycle to avoid sending bad commands, use last known good values
        continue;
      }
      // Good reading - no need to reset anything, old errors expire naturally
    }

    wp_last_theta_A[dof] = theta_A_curr;
    wp_last_theta_B[dof] = theta_B_curr;
    wp_first_read[dof] = false;
    
    // === UPDATE MOTOR ANGLE CACHE ===
    // This cache is used by checkMotorsInRange() to avoid redundant CAN reads
    // which were causing ~2ms delays per motor during safety checks
    cached_motor_angles.agonist[dof] = theta_A_curr;
    cached_motor_angles.antagonist[dof] = theta_B_curr;
    cached_motor_angles.valid[dof] = true;
    cached_motor_angles.last_update_ms = millis();

    // === EMA FILTER ON MOTOR ANGLES ===
    // Smooth encoder noise before inner PID to reduce derivative-amplified oscillations.
    // alpha=1.0 → passthrough, alpha=0.5 → moderate, alpha=0.1 → heavy filtering.
    float theta_A_pid = theta_A_curr;
    float theta_B_pid = theta_B_curr;
    if (motor_ema_enabled && motor_ema_alpha < 1.0f) {
      float a = motor_ema_alpha;
      if (!ema_initialized[dof]) {
        theta_A_ema[dof] = theta_A_curr;
        theta_B_ema[dof] = theta_B_curr;
        ema_initialized[dof] = true;
      } else {
        theta_A_ema[dof] = a * theta_A_curr + (1.0f - a) * theta_A_ema[dof];
        theta_B_ema[dof] = a * theta_B_curr + (1.0f - a) * theta_B_ema[dof];
      }
      theta_A_pid = theta_A_ema[dof];
      theta_B_pid = theta_B_ema[dof];
    }

    // === VELOCITY-DEPENDENT TAU SCALING ===
    // At low speeds, increase D-term filter tau to reduce stick-slip oscillations.
    // The filter adapts smoothly (no state reset) via PID::setTau().
    if (inner_tau_scaling_enabled) {
      // Capture original tau on first use (from PID config / flash)
      if (!inner_tau_default_captured[dof]) {
        inner_tau_default[dof] = pid_agonist->getTau();
        inner_tau_default_captured[dof] = true;
      }
      float speed = fabs(expected_velocity_cache[dof]);
      float tau_target = (speed <= inner_tau_speed_threshold) ? inner_tau_high : inner_tau_default[dof];
      if (tau_target != pid_agonist->getTau()) {
        pid_agonist->setTau(tau_target);
        pid_antagonist->setTau(tau_target);
      }
    }

    // === BUMPLESS TRANSFER for inner PIDs on IDLE → MOVING ===
    // Initialize inner PID state at current motor positions so the first control()
    // call produces zero P and D terms, avoiding derivative kick and proportional jump.
    // This is deferred from the IDLE→MOVING detection above because motor references
    // (theta_A_ref, theta_B_ref) and CAN readings (theta_A_pid, theta_B_pid) are only
    // available at this point in the loop.
    if (inner_pid_init_needed[dof]) {
      pid_agonist->initializeState(theta_A_pid, theta_A_ref, 0.0f);
      pid_antagonist->initializeState(theta_B_pid, theta_B_ref, 0.0f);
      inner_pid_init_needed[dof] = false;

      LOG_C1_DEBUG("[Waypoint] DOF " + String(dof) + " inner PID bumpless init:"
                   " Aref=" + String(theta_A_ref, 1) + " Acurr=" + String(theta_A_pid, 1) +
                   " Bref=" + String(theta_B_ref, 1) + " Bcurr=" + String(theta_B_pid, 1));
    }

    // === FRICTION FEEDFORWARD ===
    // At low speeds, tendon systems exhibit stick-slip friction: the joint stalls
    // until the PID accumulates enough error to overcome static friction, then snaps
    // forward violently. This feedforward adds torque in the direction of expected
    // motion to overcome the motor deadband and static friction.
    //
    // Profile (trapezoidal with soft fade-out):
    //   speed = 0            → FF = 0          (holding, dead zone)
    //   0.01 < speed ≤ T     → FF = full       (constant, exceeds deadband)
    //   T < speed ≤ 2*T      → FF = fade→0     (linear ramp-down, smooth release)
    //   speed > 2*T          → FF = 0          (kinetic friction only, PID handles it)
    //
    // The constant region ensures FF always exceeds the motor deadband (~30 units).
    // The fade-out region avoids a hard step when crossing the threshold.
    float uff_A = 0.0f;
    float uff_B = 0.0f;
    if (friction_ff_enabled) {
      float speed = fabs(expected_velocity_cache[dof]);
      float T = friction_ff_speed_thresh;
      if (speed > 0.01f && speed <= 2.0f * T) {
        float direction = (expected_velocity_cache[dof] >= 0.0f) ? 1.0f : -1.0f;
        float gain;
        if (speed <= T) {
          gain = 1.0f;                         // Full FF below threshold
        } else {
          gain = 1.0f - (speed - T) / T;       // Linear fade from T to 2*T
        }
        float ff = friction_ff_torque * direction * gain;
        uff_A = ff;       // Agonist: push in movement direction
        uff_B = -ff;      // Antagonist: opposite (tendon opposition)
      }
    }

    // Inner PID for motors (compute torque commands)
#if CONTROLLER_DEBUG
    uint32_t pid_start_us = time_us_32();
#endif
    float command_A = pid_agonist->control(theta_A_ref, theta_A_pid, uff_A);
    float command_B = pid_antagonist->control(theta_B_ref, theta_B_pid, uff_B);

    // Store inner PID term breakdown for diagnostics (DOF 0, agonist only)
    // Values scaled ×100 for int16 transmission (incremental terms are small floats)
    if (dof == 0 && pid_diag_terms_enabled) {
      pid_diagnostics.inner_p_term = (int16_t)constrain(pid_agonist->last_up * 100.0f, -32767, 32767);
      pid_diagnostics.inner_i_term = (int16_t)constrain(pid_agonist->last_ui * 100.0f, -32767, 32767);
      pid_diagnostics.inner_d_term = (int16_t)constrain(pid_agonist->last_udfilt * 100.0f, -32767, 32767);
      pid_diagnostics.inner_ff_term = (int16_t)constrain(uff_A * 100.0f, -32767, 32767);
      pid_diagnostics.pid_terms_valid = true;
    }

#if CONTROLLER_DEBUG
    {
      uint32_t pid_dt = time_us_32() - pid_start_us;
      wp_micro_profile.accum_pid_us += pid_dt;
      if (pid_dt > wp_micro_profile.max_pid_us) {
        wp_micro_profile.max_pid_us = pid_dt;
      }
    }
#endif
    
    // NOTE: Co-contraction is achieved through stiffness_ref parameter in cascade control
    // theta_A_ref = theta_0 + 0.5*delta_theta + 0.5*stiffness_ref
    // theta_B_ref = theta_0 + 0.5*delta_theta - 0.5*stiffness_ref
    // The stiffness_ref separates motor references, creating constant tension on both tendons.
    // Increase stiffness_ref (via UI) to reduce vibrations from slack tendons.
    
    // Get torque limits from motor configuration
    float max_torque_A = config.motors[agonist_idx].max_torque;
    float max_torque_B = config.motors[antagonist_idx].max_torque;

    // === SOFT HOLD TORQUE SCALING (compliance) ===
    ComplianceState &cs = compliance_state[dof];
    float target_ratio = (soft_hold_enabled && cs.compliance_active) ? soft_hold_torque_ratio : 1.0f;
    uint32_t now_ms = millis();

    if (target_ratio != cs.torque_ratio_target) {
      cs.torque_ratio_start = cs.torque_ratio_current;
      cs.torque_ratio_target = target_ratio;
      cs.torque_ramp_start_ms = now_ms;
      uint16_t ramp_ms =
          (target_ratio < cs.torque_ratio_current) ? soft_hold_ramp_down_ms : soft_hold_ramp_up_ms;
      cs.torque_ramp_duration_ms = ramp_ms;
      cs.torque_ramp_active = (ramp_ms > 0);
      if (!cs.torque_ramp_active) {
        cs.torque_ratio_current = target_ratio;
      }

      LOG_C1_INFO("[Compliance] DOF " + String(dof) + " torque_ratio " +
               String(cs.torque_ratio_start, 2) + " -> " + String(target_ratio, 2) +
               " ramp=" + String(ramp_ms) + "ms");
    }

    if (cs.torque_ramp_active) {
      uint32_t elapsed_ms = (now_ms >= cs.torque_ramp_start_ms)
                                ? (now_ms - cs.torque_ramp_start_ms)
                                : 0;
      if (elapsed_ms >= cs.torque_ramp_duration_ms) {
        cs.torque_ratio_current = cs.torque_ratio_target;
        cs.torque_ramp_active = false;
      } else {
        float t = (float)elapsed_ms / (float)cs.torque_ramp_duration_ms;
        cs.torque_ratio_current =
            cs.torque_ratio_start + (cs.torque_ratio_target - cs.torque_ratio_start) * t;
      }
    }

    float max_torque_A_effective = max_torque_A;
    float max_torque_B_effective = max_torque_B;
    if (soft_hold_enabled) {
      max_torque_A_effective = max_torque_A * cs.torque_ratio_current;
      max_torque_B_effective = max_torque_B * cs.torque_ratio_current;
      if (cs.compliance_active) {
        if (min_tension_torque > max_torque_A_effective) {
          max_torque_A_effective = min(min_tension_torque, max_torque_A);
        }
        if (min_tension_torque > max_torque_B_effective) {
          max_torque_B_effective = min(min_tension_torque, max_torque_B);
        }
      }
    }

    // === TORQUE SATURATION & RATE LIMITING ===
    // IMPORTANT: Saturate FIRST, then rate-limit the saturated value
    // This prevents prev_command from storing unsaturated values which would
    // cause asymmetric rate limiting near the saturation boundary
    static float prev_command_A[MAX_DOFS] = {0};
    static float prev_command_B[MAX_DOFS] = {0};

    // Step 1: Apply absolute torque limits FIRST
    command_A = constrain(command_A, -max_torque_A_effective, max_torque_A_effective);
    command_B = constrain(command_B, -max_torque_B_effective, max_torque_B_effective);

    // Step 2: Apply rate limiting on saturated values
    if (torque_ramp_time_ms > 0) {
      // Calculate max torque change per cycle
      // At 500 Hz with 100ms ramp time: 50 cycles to go 0→max
      // max_rate = max_torque / (ramp_time_ms / inner_loop_period_ms)
      //          = max_torque * inner_loop_period_us / (ramp_time_ms * 1000)
      float rate_A = max_torque_A_effective * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);
      float rate_B = max_torque_B_effective * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);

      command_A = constrain(command_A, prev_command_A[dof] - rate_A, prev_command_A[dof] + rate_A);
      command_B = constrain(command_B, prev_command_B[dof] - rate_B, prev_command_B[dof] + rate_B);
    }

    // Step 3: Store saturated+rate-limited value for next cycle
    prev_command_A[dof] = command_A;
    prev_command_B[dof] = command_B;
    
    // === DIAGNOSTIC: Log extreme torque commands ===
    if (fabs(command_A) >= max_torque_A_effective * 0.95f ||
        fabs(command_B) >= max_torque_B_effective * 0.95f) {
      static uint32_t last_torque_warn = 0;
      if (millis() - last_torque_warn > 500) { // Log max every 500ms
        LOG_C1_WARN("[Waypoint DIAG] DOF " + String(dof) + " HIGH TORQUE: A=" + String(command_A, 0) + 
                 " B=" + String(command_B, 0) + " (max=" + String(max_torque_A_effective, 0) + ")");
        LOG_C1_WARN("  refs: A=" + String(theta_A_ref, 2) + " B=" + String(theta_B_ref, 2));
        LOG_C1_WARN("  curr: A=" + String(theta_A_curr, 2) + " B=" + String(theta_B_curr, 2));
        last_torque_warn = millis();
      }
    }
    
    // Send torque commands to motors
#if CONTROLLER_DEBUG
    uint32_t torque_start_us = time_us_32();
#endif
    agonist->setTorque((int)command_A);
    antagonist->setTorque((int)command_B);
#if CONTROLLER_DEBUG
    {
      uint32_t torque_dt = time_us_32() - torque_start_us;
      wp_micro_profile.accum_torque_us += torque_dt;
      if (torque_dt > wp_micro_profile.max_torque_us) {
        wp_micro_profile.max_torque_us = torque_dt;
      }
    }
#endif
    
    // === UPDATE PID DIAGNOSTICS for CAN streaming ===
    // Store values for diagnostic stream (read by sendPIDDiagStreamData in core1.cpp)
    // Use theta_0_joint (target from interpolation) and snapshot angle (current reading)
    if (dof < 3) {
      float q_curr_diag = dof_data.angles[dof];
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

#if CONTROLLER_DEBUG
    {
      uint32_t dof_dt = time_us_32() - dof_start_us;
      wp_micro_profile.accum_dof_us += dof_dt;
      if (dof_dt > wp_micro_profile.max_dof_us) {
        wp_micro_profile.max_dof_us = dof_dt;
      }
      wp_micro_profile.samples++;
    }
#endif
  }

  // Mark diagnostics as valid after processing all DOFs
  pid_diagnostics.last_update_ms = millis();
  pid_diagnostics.valid = true;
  
  // Reset safety check counter after processing all DOFs
  // This ensures all DOFs in HOLDING mode are checked in the same cycle
  // Using 10 cycles = ~100ms at 100Hz outer loop rate
  if (safety_check_counter >= 10) {
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
    
    // Log only when over budget (every 5 seconds = 2500 cycles @ 500Hz)
    // This avoids logging overhead during normal operation
    static uint16_t profiling_log_counter = 0;
    profiling_log_counter++;
    if (profiling_log_counter >= 2500) {
      profiling_log_counter = 0;
      
      // Only log if we exceeded the budget during this period
      if (cycle_time_us_max > inner_loop_period_us) {
        LOG_C1_WARN("[PROFILING] OVER BUDGET! last=" + String(cycle_time_us_last) + "µs, " +
                 "avg=" + String(cycle_time_us_avg) + "µs, " +
                 "max=" + String(cycle_time_us_max) + "µs " +
                 "(budget=" + String(inner_loop_period_us) + "µs)");
      }
      
      // Reset max for next period
      cycle_time_us_max = 0;
    }
  }
  
#if CONTROLLER_DEBUG
  if (wp_micro_profile.samples >= WP_MICRO_MIN_SAMPLES) {
    uint32_t now_ms = millis();
    if (wp_micro_profile.last_log_ms == 0) {
      wp_micro_profile.last_log_ms = now_ms;
    }
    if (now_ms - wp_micro_profile.last_log_ms >= WP_MICRO_LOG_INTERVAL_MS) {
      // Pre-compute averages as integers (avoids snprintf %f which uses ~1KB stack on ARM)
      uint32_t n = wp_micro_profile.samples;
      uint32_t sn = wp_micro_profile.safety_samples;
      uint32_t avg_dof = wp_micro_profile.accum_dof_us / n;
      uint32_t avg_outer = wp_micro_profile.accum_outer_us / n;
      uint32_t avg_eq = wp_micro_profile.accum_eq_us / n;
      uint32_t avg_can = wp_micro_profile.accum_can_us / n;
      uint32_t avg_pid = wp_micro_profile.accum_pid_us / n;
      uint32_t avg_torque = wp_micro_profile.accum_torque_us / n;
      uint32_t avg_safety = (sn > 0) ? wp_micro_profile.accum_safety_us / sn : 0;
      // Split into 2 messages to keep buffer small
      LOG_C1_INFO_F("[WP PROF] avg_us dof=%lu outer=%lu eq=%lu can=%lu pid=%lu torq=%lu safety=%lu",
                 avg_dof, avg_outer, avg_eq, avg_can, avg_pid, avg_torque, avg_safety);
      LOG_C1_INFO_F("[WP PROF] max_us dof=%lu outer=%lu eq=%lu can=%lu pid=%lu torq=%lu safety=%lu",
                 (unsigned long)wp_micro_profile.max_dof_us,
                 (unsigned long)wp_micro_profile.max_outer_us,
                 (unsigned long)wp_micro_profile.max_eq_us,
                 (unsigned long)wp_micro_profile.max_can_us,
                 (unsigned long)wp_micro_profile.max_pid_us,
                 (unsigned long)wp_micro_profile.max_torque_us,
                 (unsigned long)wp_micro_profile.max_safety_us);
      wp_micro_profile.accum_dof_us = 0;
      wp_micro_profile.accum_outer_us = 0;
      wp_micro_profile.accum_eq_us = 0;
      wp_micro_profile.accum_can_us = 0;
      wp_micro_profile.accum_pid_us = 0;
      wp_micro_profile.accum_torque_us = 0;
      wp_micro_profile.accum_safety_us = 0;
      wp_micro_profile.max_dof_us = 0;
      wp_micro_profile.max_outer_us = 0;
      wp_micro_profile.max_eq_us = 0;
      wp_micro_profile.max_can_us = 0;
      wp_micro_profile.max_pid_us = 0;
      wp_micro_profile.max_torque_us = 0;
      wp_micro_profile.max_safety_us = 0;
      wp_micro_profile.samples = 0;
      wp_micro_profile.safety_samples = 0;
      wp_micro_profile.last_log_ms = now_ms;
    }
  }
#endif
  
  return any_movement;
}

// === Core0-safe trajectory dump ===
// Called from Core0 main loop when wp_dump_pending_dof >= 0.
// Prints the full waypoint table over serial at Core0's pace.
void wp_dump_print_from_core0() {
  int8_t dof = wp_dump_pending_dof;
  if (dof < 0 || dof >= MAX_DOFS) return;
  wp_dump_pending_dof = -1;  // clear immediately to avoid re-entry

  uint16_t count = wp_dump_count[dof];
  if (count == 0) return;

  SERIAL_COM_LN("INFO: [WP_DUMP] DOF " + String(dof) + ": " + String(count) + " waypoints consumed");
  SERIAL_COM_LN("INFO: [WP_DUMP] idx | angle_deg | t_arrival_ms | dt_ms | vel_deg_s");

  for (uint16_t i = 0; i < count; i++) {
    float angle = wp_dump[dof][i].angle_deg;
    uint32_t t_arr = wp_dump[dof][i].t_arrival_ms;
    char buf[90];
    if (i == 0) {
      snprintf(buf, sizeof(buf), "INFO: [WP_DUMP] %3d | %8.2f | %10lu |   --- |       ---",
               i, angle, (unsigned long)t_arr);
    } else {
      float prev_angle = wp_dump[dof][i-1].angle_deg;
      uint32_t prev_t = wp_dump[dof][i-1].t_arrival_ms;
      uint32_t dt_ms = t_arr - prev_t;
      float vel = (dt_ms > 0) ? fabsf(angle - prev_angle) / ((float)dt_ms / 1000.0f) : 0.0f;
      snprintf(buf, sizeof(buf), "INFO: [WP_DUMP] %3d | %8.2f | %10lu | %5lu | %9.1f",
               i, angle, (unsigned long)t_arr, (unsigned long)dt_ms, vel);
    }
    SERIAL_COM_LN(buf);
  }
  SERIAL_COM_LN("INFO: [WP_DUMP] END");
}
