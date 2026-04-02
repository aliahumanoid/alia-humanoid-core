/**
 * @file JointController_ControlLoop.cpp
 * @brief Impedance-based trajectory execution with cascade control
 *
 * Implementation follows CAN_SYSTEM_ARCHITECTURE.md section 6.2 with cascade
 * control (outer joint PID + inner motor PID) driven by impedance commands.
 *
 * Key features:
 * - Impedance control via SET_IMPEDANCE CAN command
 * - Per-DOF state management: IDLE → MOVING → HOLDING
 * - Cascade control architecture:
 *   * SAMPLING_PERIOD = 2000 µs (2 ms) → 500 Hz
 *   * Outer PID runs every outer_loop_divisor cycles (default 1 = 500 Hz)
 *   * Inner PID @ 500 Hz (motor-level, computes torque commands)
 *   * theta_ref = theta_0 + cascade_correction
 *
 * @see CAN_SYSTEM_ARCHITECTURE.md for protocol specification
 */

#include "JointController.h"
#include <Arduino.h>
#include <debug.h>
#include "main_common.h"  // For shared_dof_angles, DofState
#include <math.h>  // For cosf, M_PI

// External time sync function (defined in core1.cpp)
extern uint32_t getAbsoluteTimeMs();

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
static uint32_t last_anti_slack_log_ms[MAX_DOFS] = {0};
static float direct_drive_last_angle[MAX_DOFS] = {0};
static uint32_t direct_drive_last_update_us[MAX_DOFS] = {0};

// === SLACK MONITOR: delta_theta bias tracking during HOLDING ===
// Exponential moving average of delta_theta while in HOLDING state.
// A persistent non-zero bias indicates the outer PID is doing static compensation
// work. This is a joint-space symptom — it does NOT directly identify which motor
// offset is wrong (see SLACK_DETECTION_AND_TENSION_TRIM.md for rationale).
// Used as a diagnostic/gating signal, not as a direct correction source.
static float holding_dtheta_ema[MAX_DOFS] = {0};     // EMA of delta_theta during HOLDING
static uint32_t holding_ema_samples[MAX_DOFS] = {0}; // samples accumulated (for warm-up)
static uint32_t last_holding_bias_log[MAX_DOFS] = {0}; // rate-limit log
static uint32_t hold_ki_ramp_start_ms[MAX_DOFS] = {0}; // HOLDING entry timestamp for Ki ramp-down
static uint32_t last_trim_dry_run_update[MAX_DOFS] = {0}; // separate cadence from DIAG_HOLD telemetry
static const float HOLDING_EMA_ALPHA = 0.005f;       // ~200 samples window at 500Hz ≈ 0.4s
static float last_hold_event_q[MAX_DOFS] = {0};      // previous q sample while in HOLDING
static bool hold_event_q_valid[MAX_DOFS] = {false};  // previous q is initialized
static uint32_t last_hold_event_log_ms[MAX_DOFS] = {0}; // rate-limit [HOLD_EVT] logging
static float last_outer_ki_scale_dbg[MAX_DOFS] = {1.0f}; // last applied outer Ki scale
static bool last_outer_i_freeze_dbg[MAX_DOFS] = {false}; // last applied outer I freeze state
static float last_retension_boost_dbg[MAX_DOFS] = {0.0f}; // last applied probe boost
static uint8_t hold_entry_stable_count[MAX_DOFS] = {0};  // stable outer-loop samples before MOVING -> HOLDING
static bool metrics_finalize_pending[MAX_DOFS] = {false};
static uint32_t metrics_finalize_hold_start_ms[MAX_DOFS] = {0};
static const float HOLD_EVENT_Q_STEP_TH_DEG = 0.10f; // log when HOLDING q jumps by >= 0.10°
static const float HOLD_EVENT_VEL_TH_DEG_S = 1.00f; // or when HOLDING velocity exceeds 1 deg/s
static const uint16_t HOLD_EVENT_MIN_INTERVAL_MS = 50; // high-rate but not every cycle
static const uint16_t RETN_PROBE_HOLD_EVT_BLOCK_MS = 500; // don't probe right after a HOLD_EVT
static const float HOLD_ENTRY_ERROR_BAND_DEG = 0.50f;     // require this residual error before declaring HOLDING
static const float HOLD_ENTRY_MAX_VEL_DEG_S = 5.0f;       // filtered velocity must also be low
static const uint8_t HOLD_ENTRY_STABLE_CYCLES = 3;        // ~30 ms at 100 Hz outer loop
static const uint16_t METRICS_FINALIZE_MIN_HOLD_SAMPLES = 10; // require some post-arrival hold samples
static const uint16_t METRICS_FINALIZE_MAX_HOLD_MS = 250; // don't wait forever if a joint never fully settles

enum RetensionProbeClassCode : uint8_t {
  RPROBE_CLASS_UNKNOWN = 0,
  RPROBE_CLASS_LOW_EFFORT = 1,
  RPROBE_CLASS_NO_CORRECTION = 2,
  RPROBE_CLASS_NO_EFFECT = 3,
  RPROBE_CLASS_SLACK_LIKELY = 4
};

enum class RetensionProbePhase : uint8_t {
  IDLE = 0,
  ARMED = 1,
  ACTIVE = 2,
  POST = 3,
  DONE = 4
};

struct RetensionProbeState {
  RetensionProbePhase phase;
  uint32_t scheduled_start_ms;
  uint32_t active_until_ms;
  uint32_t post_until_ms;
  float pre_abs_iq_a_sum;
  float pre_abs_iq_b_sum;
  float pre_q_sum;
  float pre_ema_sum;
  uint16_t pre_count;
  float during_abs_iq_a_sum;
  float during_abs_iq_b_sum;
  float during_q_sum;
  float during_ema_sum;
  uint16_t during_count;
  float post_abs_iq_a_sum;
  float post_abs_iq_b_sum;
  float post_q_sum;
  float post_ema_sum;
  uint16_t post_count;
};

static RetensionProbeState retension_probe_state[MAX_DOFS] = {};

// === PROPOSED TRIM (dry-run, not applied to control) ===
// Computed during gated HOLDING, transmitted to UI for visualization.
// Positive → would increase agonist-side preload.
// Negative → would increase antagonist-side preload.
// See SLACK_DETECTION_AND_TENSION_TRIM.md §D.
static float proposed_trim_deg[MAX_DOFS] = {0};
static const float TRIM_STEP_DEG = 0.05f;             // micro-step per update (~every 3s)
static const float TRIM_MAX_DEG = 2.0f;               // absolute clamp
static const float TRIM_DECAY = 0.99f;                // slow decay when balanced
static const float TRIM_SLACK_RATIO_TH = 0.30f;       // iq ratio below this → slack detected
static const float TRIM_BALANCED_RATIO_TH = 0.50f;    // iq ratio above this → balanced
static const float TRIM_BIAS_TH = 0.30f;              // |ema| above this → bias present

static void resetRetensionProbeState(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  retension_probe_state[dof] = {};
  retension_probe_state[dof].phase = RetensionProbePhase::IDLE;
  last_retension_boost_dbg[dof] = 0.0f;
}

static void resetMovementSettleState(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  hold_entry_stable_count[dof] = 0;
  metrics_finalize_pending[dof] = false;
  metrics_finalize_hold_start_ms[dof] = 0;
}

static void finalizeMovementMetricsForDof(uint8_t dof) {
  if (dof >= 3 || !metrics_tracking_enabled) return;

  MetricsTracker &mt = metrics_tracker[dof];
  if (!mt.tracking_active) {
    metrics_finalize_pending[dof] = false;
    metrics_finalize_hold_start_ms[dof] = 0;
    return;
  }

  // Final target may differ from the first streamed impedance command. Finalize against
  // the actual hold target (or the original desired target if the move stalled).
  float original_target = mt.target_angle_deg;
  float metrics_target = dof_hold_angle[dof];
  if (mt.aborted_by_stall) {
    metrics_target = mt.abort_target_deg;
  }
  mt.target_angle_deg = metrics_target;
  mt.movement_direction = (metrics_target > mt.start_angle_deg) ? 1.0f : -1.0f;

  MovementMetrics m = mt.finalize();
  m.dof_index = dof;
  last_movement_metrics[dof] = m;
  metrics_ready[dof] = true;

  mt.tracking_active = false;
  metrics_finalize_pending[dof] = false;
  metrics_finalize_hold_start_ms[dof] = 0;

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

static void clearRetensionProbeWindows(RetensionProbeState &rps) {
  rps.pre_abs_iq_a_sum = 0.0f;
  rps.pre_abs_iq_b_sum = 0.0f;
  rps.pre_q_sum = 0.0f;
  rps.pre_ema_sum = 0.0f;
  rps.pre_count = 0;
  rps.during_abs_iq_a_sum = 0.0f;
  rps.during_abs_iq_b_sum = 0.0f;
  rps.during_q_sum = 0.0f;
  rps.during_ema_sum = 0.0f;
  rps.during_count = 0;
  rps.post_abs_iq_a_sum = 0.0f;
  rps.post_abs_iq_b_sum = 0.0f;
  rps.post_q_sum = 0.0f;
  rps.post_ema_sum = 0.0f;
  rps.post_count = 0;
}

// Reset session-local diagnostics for a DOF.
// Called from clearImpedanceControlState(), IDLE path, and E-Stop (via core1).
void resetDiagHoldState(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  proposed_trim_deg[dof] = 0;
  holding_ema_samples[dof] = 0;
  holding_dtheta_ema[dof] = 0;
  last_holding_bias_log[dof] = 0;
  hold_ki_ramp_start_ms[dof] = 0;
  last_trim_dry_run_update[dof] = 0;
  last_hold_event_q[dof] = 0;
  hold_event_q_valid[dof] = false;
  last_hold_event_log_ms[dof] = 0;
  last_outer_ki_scale_dbg[dof] = 1.0f;
  last_outer_i_freeze_dbg[dof] = false;
  resetRetensionProbeState(dof);
  resetMovementSettleState(dof);
}

static void clearImpedanceControlState(uint8_t dof, JointController *jc) {
  restoreInnerPidGains(dof, jc);
  restoreOuterLoopParameters(dof, jc);
  resetImpedanceSegment(dof);
  impedance_target[dof].watchdog_timed_out = false;
  impedance_target[dof].valid = false;
  inner_pid_reinit_after_impedance[dof] = true;
  resetDiagHoldState(dof);
}

static void freezeImpedanceToLocalHold(uint8_t dof, float q_hold_deg, uint32_t now_ms,
                                       bool latch_watchdog_timeout) {
  if (dof >= MAX_DOFS) return;

  ImpedanceRollingSegment &seg = impedance_segment[dof];
  seg.q_goal_deg = q_hold_deg;
  seg.q_start_deg = q_hold_deg;
  seg.q_ref_deg = q_hold_deg;
  seg.dq_ref_deg_s = 0.0f;
  seg.speed_abs_deg_s = 0.0f;
  seg.t_start_ms = now_ms;
  seg.t_arrival_ms = now_ms;
  seg.active = false;
  seg.initialized = true;

  impedance_target[dof].q_target_deg = q_hold_deg;
  impedance_target[dof].dq_target_deg_s = 0.0f;
  impedance_target[dof].last_update_ms = now_ms;
  impedance_target[dof].watchdog_timed_out = latch_watchdog_timeout;

  dof_hold_angle[dof] = q_hold_deg;
  dof_hold_time[dof] = now_ms;
  dof_state[dof] = DofState::HOLDING;
}

static void applyImpedanceOuterOverrides(uint8_t dof, JointController *jc) {
  if (!jc || !impedance_target[dof].valid) {
    return;
  }

  float cur_kp, cur_ki, cur_kd, cur_stiff, cur_influence;
  if (!jc->getOuterLoopParameters(dof, cur_kp, cur_ki, cur_kd, cur_stiff, cur_influence)) {
    return;
  }

  OuterLoopBackup &backup = outer_loop_backup[dof];
  if (!backup.saved) {
    backup = {cur_kp, cur_ki, cur_kd, cur_stiff, cur_influence, true};
  }

  const float desired_influence = backup.cascade_influence;
  if (fabsf(cur_kp - impedance_target[dof].kp) > 0.001f ||
      fabsf(cur_ki - impedance_target[dof].ki) > 0.001f ||
      fabsf(cur_kd - impedance_target[dof].kd) > 0.001f ||
      fabsf(cur_stiff - impedance_target[dof].stiffness_deg) > 0.001f ||
      fabsf(cur_influence - desired_influence) > 0.001f) {
    jc->setOuterLoopParameters(dof,
                               impedance_target[dof].kp,
                               impedance_target[dof].ki,
                               impedance_target[dof].kd,
                               impedance_target[dof].stiffness_deg,
                               desired_influence);
  }
}

static void applyImpedanceInnerOverrides(uint8_t dof, PID *pid_agonist, PID *pid_antagonist) {
  if (!pid_agonist || !pid_antagonist || !impedance_target[dof].valid) {
    return;
  }

  if (!inner_pid_backup[dof][0].saved) {
    inner_pid_backup[dof][0] = {pid_agonist->getKp(), pid_agonist->getKi(), pid_agonist->getKd(), true};
    inner_pid_backup[dof][1] = {pid_antagonist->getKp(), pid_antagonist->getKi(), pid_antagonist->getKd(), true};
    LOG_C1_INFO("[IMPEDANCE] DOF" + String(dof) + " inner PID backed up:"
                " Kp=" + String(inner_pid_backup[dof][0].kp, 3) +
                " Ki=" + String(inner_pid_backup[dof][0].ki, 3) +
                " Kd=" + String(inner_pid_backup[dof][0].kd, 3));
  }

  const float kp_inner = impedance_target[dof].kp_inner;
  const float ki_inner = impedance_target[dof].ki_inner;
  const float kd_inner = impedance_target[dof].kd_inner;
  if (fabsf(pid_agonist->getKp() - kp_inner) > 0.001f ||
      fabsf(pid_agonist->getKi() - ki_inner) > 0.001f ||
      fabsf(pid_agonist->getKd() - kd_inner) > 0.001f) {
    pid_agonist->setTunings(kp_inner, ki_inner, kd_inner, pid_agonist->getTau());
  }
  if (fabsf(pid_antagonist->getKp() - kp_inner) > 0.001f ||
      fabsf(pid_antagonist->getKi() - ki_inner) > 0.001f ||
      fabsf(pid_antagonist->getKd() - kd_inner) > 0.001f) {
    pid_antagonist->setTunings(kp_inner, ki_inner, kd_inner, pid_antagonist->getTau());
  }
}

static void applyImpedanceInnerOverrideSingle(uint8_t dof, PID *pid_direct) {
  if (!pid_direct || !impedance_target[dof].valid) {
    return;
  }

  if (!inner_pid_backup[dof][0].saved) {
    inner_pid_backup[dof][0] = {pid_direct->getKp(), pid_direct->getKi(), pid_direct->getKd(), true};
    inner_pid_backup[dof][1] = {};
    LOG_C1_INFO("[IMPEDANCE] DOF" + String(dof) + " direct-drive PID backed up:"
                " Kp=" + String(inner_pid_backup[dof][0].kp, 3) +
                " Ki=" + String(inner_pid_backup[dof][0].ki, 3) +
                " Kd=" + String(inner_pid_backup[dof][0].kd, 3));
  }

  const float kp_inner = impedance_target[dof].kp_inner;
  const float ki_inner = impedance_target[dof].ki_inner;
  const float kd_inner = impedance_target[dof].kd_inner;
  if (fabsf(pid_direct->getKp() - kp_inner) > 0.001f ||
      fabsf(pid_direct->getKi() - ki_inner) > 0.001f ||
      fabsf(pid_direct->getKd() - kd_inner) > 0.001f) {
    pid_direct->setTunings(kp_inner, ki_inner, kd_inner, pid_direct->getTau());
  }
}
// When outer_loop_divisor > 1, delta_theta changes every N cycles creating "steps".
// To smooth this, we interpolate between delta_theta_prev and delta_theta based on
// where we are in the current outer loop period. This eliminates vibrations caused
// by discontinuous reference changes while maintaining cascade control benefits.

#if CONTROLLER_DEBUG
struct LoopMicroProfile {
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
static LoopMicroProfile loop_micro_profile;
static const uint32_t LOOP_MICRO_LOG_INTERVAL_MS = 2000;
static const uint16_t LOOP_MICRO_MIN_SAMPLES = 50;
#endif

// Safety check counter (for periodic motor checks in HOLDING mode)
static uint16_t safety_check_counter = 0;

// Track previous state to detect MOVING → HOLDING transitions
static DofState prev_dof_state[MAX_DOFS] = {DofState::IDLE};
static DofState prev_state_before_update[MAX_DOFS] = {DofState::IDLE}; // snapshot for bias/slack gate

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

// === PHASE 2: 0xA1-only motor angle feedback with 0x92 watchdog ===
// First cycle: 0x92 bootstraps revolution tracking (absolute multi-turn angle).
// Subsequent cycles: tracked angle from 0xA1 torque response is source of truth.
// Periodic 0x92 watchdog (~1Hz) verifies tracking hasn't drifted.
static bool wp_rev_track_init[MAX_DOFS] = {false, false, false};
static bool wp_0x92_bootstrap_done[MAX_DOFS] = {false, false, false};
static uint32_t wp_watchdog_cycle_count[MAX_DOFS] = {0, 0, 0};
static uint8_t wp_a1_miss_count[MAX_DOFS] = {0, 0, 0};
static uint32_t wp_last_a1_miss_log_ms[MAX_DOFS] = {0, 0, 0};
static const uint32_t WP_WATCHDOG_INTERVAL = 500; // 0x92 every 500 cycles ≈ 1Hz @ 500Hz
static int wp_prev_torque_A[MAX_DOFS] = {0};
static int wp_prev_torque_B[MAX_DOFS] = {0};

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
void getLoopProfilingStats(uint32_t& last_us, uint32_t& avg_us, uint32_t& max_us) {
  last_us = cycle_time_us_last;
  avg_us = cycle_time_us_avg;
  max_us = cycle_time_us_max;
}

/**
 * @brief Execute impedance-based cascade control for all DOFs
 *
 * Main control loop entry point called from core1_loop() at 500 Hz.
 * Implements cascade control (outer joint PID + inner motor PID) with
 * impedance targets received via SET_IMPEDANCE CAN command (0x01D).
 *
 * Control architecture:
 * - Outer loop: configurable via outer_loop_divisor (default 500 Hz)
 * - Inner loop: 500 Hz (motor PID → torque command)
 * - Impedance: rolling segment interpolation from current to goal
 * - HOLDING: maintain position when no impedance target is active
 *
 * @return true if any DOF is actively moving
 */
bool JointController::executeControlLoop() {
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
  uint32_t feedback_now_us = time_us_32();

  // Direct-drive DOFs use the motor's internal absolute encoder on Core1.
  // Override the snapshot locally so the rest of the control loop can stay
  // topology-agnostic.
  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    if (config.dofs[dof].drive_type != DRIVE_DIRECT_DRIVE) {
      continue;
    }

    LKM_Motor *direct_motor = nullptr;
    for (int i = 0; i < config.motor_count; i++) {
      if (config.motors[i].dof_index == dof &&
          config.motors[i].role == MOTOR_ROLE_DIRECT) {
        direct_motor = motors[i];
        break;
      }
    }

    if (direct_motor == nullptr) {
      dof_snapshot.valid[dof] = false;
      dof_snapshot.velocities[dof] = 0.0f;
      direct_drive_last_update_us[dof] = 0;
      updateDirectDriveFeedback(dof, 0.0f, 0.0f, false);
      continue;
    }

    LKM_Motor::MultiAngleData raw_angle = direct_motor->getSingleAngleSync();
    if (isnan(raw_angle.angle) || !_saved_offsets[dof].valid) {
      dof_snapshot.valid[dof] = false;
      dof_snapshot.velocities[dof] = 0.0f;
      direct_drive_last_update_us[dof] = 0;
      updateDirectDriveFeedback(dof, 0.0f, 0.0f, false);
      continue;
    }

    const float calibrated_angle = raw_angle.angle - _saved_offsets[dof].agonist_offset;

    float velocity_deg_s = 0.0f;
    if (direct_drive_last_update_us[dof] > 0) {
      float dt_s = (feedback_now_us - direct_drive_last_update_us[dof]) / 1000000.0f;
      if (dt_s > 0.0001f) {
        velocity_deg_s = (calibrated_angle - direct_drive_last_angle[dof]) / dt_s;
      }
    }

    dof_snapshot.angles[dof] = calibrated_angle;
    dof_snapshot.velocities[dof] = velocity_deg_s;
    dof_snapshot.valid[dof] = true;
    direct_drive_last_angle[dof] = calibrated_angle;
    direct_drive_last_update_us[dof] = feedback_now_us;
    updateDirectDriveFeedback(dof, calibrated_angle, velocity_deg_s, true);
  }
  const SharedDofAngles &dof_data = dof_snapshot;

  // === OUTER LOOP SCHEDULING ===
  uint8_t effective_divisor = (outer_loop_divisor < 1) ? 1 : outer_loop_divisor;
  const bool outer_cycle_due = ((cycle_count - 1) % effective_divisor) == 0;
  
  // Process each DOF independently
  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    
    // === EARLY EXIT: Skip if DOF is IDLE ===
    // This saves CPU time when no control is active
    DofState cur_dof_state = dof_state[dof];
    if (cur_dof_state == DofState::IDLE) {
      // Mark PID reset needed for when this DOF becomes active
      pid_reset_needed[dof] = true;
      inner_pid_init_needed[dof] = false;  // Clear any stale flag
      prev_dof_state[dof] = DofState::IDLE;
      compliance_state[dof].reset();
      velocity_filtered[dof] = 0.0f;
      resetDiagHoldState(dof);
      expected_velocity_cache[dof] = 0.0f;
      // Reset Phase2 tracking state
      wp_rev_track_init[dof] = false;
      wp_0x92_bootstrap_done[dof] = false;
      wp_watchdog_cycle_count[dof] = 0;
      wp_a1_miss_count[dof] = 0;
      wp_last_a1_miss_log_ms[dof] = 0;
      wp_prev_torque_A[dof] = 0;
      wp_prev_torque_B[dof] = 0;
      continue; // Skip this DOF entirely
    }

    // === LAZY PID RESTORE: Clean up stale impedance gain overrides ===
    // When impedance mode ends (via disable, watchdog, startup, E-Stop), the
    // impedance_target is invalidated but the PID gains may still be overridden.
    // The backup structs (.saved == true) indicate gains that need restoring.
    // This runs on Core1 where the PID objects live, avoiding cross-core mutation.
    if (!impedance_target[dof].valid) {
      if (inner_pid_backup[dof][0].saved) {
        restoreInnerPidGains(dof, this);
      }
      if (outer_loop_backup[dof].saved) {
        restoreOuterLoopParameters(dof, this);
      }
    }

#if CONTROLLER_DEBUG
    uint32_t dof_start_us = time_us_32();
#endif

    // === RESET PID STATE when exiting IDLE ===
    // Reset on IDLE → MOVING or IDLE → HOLDING (re-entering control from stopped state)
    // DO NOT reset on HOLDING → MOVING - preserve integral compensation for gravity/friction
    // This prevents torque bumps from stale PID state after recalibration/startup
    bool just_exited_idle = (prev_dof_state[dof] == DofState::IDLE);
    bool should_reset = pid_reset_needed[dof] && just_exited_idle;
    
    if (should_reset) {
      // Bumpless transfer for outer loop PID controller:
      // Initialize state at current joint angle so P and D terms start at zero.
      // Since q_des ≈ q_curr at entry, error ≈ 0 and only the
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

      // Reset Phase2 tracking for clean start
      wp_rev_track_init[dof] = false;
      wp_0x92_bootstrap_done[dof] = false;
      wp_watchdog_cycle_count[dof] = 0;
      wp_a1_miss_count[dof] = 0;
      wp_last_a1_miss_log_ms[dof] = 0;
      wp_prev_torque_A[dof] = 0;
      wp_prev_torque_B[dof] = 0;
      resetMovementSettleState(dof);

      LOG_C1_DEBUG("[Control] DOF " + String(dof) + " bumpless init (IDLE → " +
                   String(cur_dof_state == DofState::MOVING ? "MOVING" : "HOLDING") +
                   ") at " + String(q_init, 1) + "°");
    }

    // === TRAJECTORY DUMP: Reset on new movement ===
    // (trajectory dump removed)

    // === METRICS: Initialize tracker for NEW movement (from IDLE or HOLDING) ===
    // Detect when a new movement starts:
    // - IDLE → MOVING: first movement ever
    // - HOLDING → MOVING: new movement after previous one completed
    // Use !tracking_active as additional guard to prevent multiple initializations
    bool new_movement_started = (prev_dof_state[dof] != DofState::MOVING) &&
                                 (cur_dof_state == DofState::MOVING);

    if (new_movement_started) {
      if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
        finalizeMovementMetricsForDof(dof);
      }
      compliance_state[dof].reset();
      velocity_filtered[dof] = 0.0f;
      resetMovementSettleState(dof);
    }
    
    if (metrics_tracking_enabled && new_movement_started && dof < 3) {
      if (impedance_target[dof].valid) {
        float start_angle = dof_data.valid[dof] ? dof_data.angles[dof] : 0.0f;
        metrics_tracker[dof].reset(start_angle, impedance_target[dof].q_target_deg);
      }
    }
    
    // === IMPEDANCE MODE: Watchdog and PID init ===
    // Check impedance watchdog BEFORE outer loop so timeout transitions happen promptly
    if (impedance_target[dof].valid && !impedance_target[dof].watchdog_timed_out) {
      uint32_t elapsed = t_now - impedance_target[dof].last_update_ms;
      if (elapsed > impedance_watchdog_ms) {
        // Host stream timed out. Freeze the current joint position as a local
        // impedance hold and keep the impedance controller active.
        //
        // Why:
        // - A stable HOLDING state should not be torn down just because the host
        //   stops sending keepalives.
        // - Teardown/restoration of PID gains was causing an artificial "kick"
        //   unrelated to the real joint mechanics.
        //
        // New policy:
        // - Stop chasing host updates
        // - Latch the current pose as a firmware-local hold
        // - Keep outer/inner overrides active until an explicit disable/E-stop
        //   or a new SET_IMPEDANCE command arrives
        float q_curr_wd = dof_data.valid[dof] ? dof_data.angles[dof]
                                              : getImpedanceHoldReference(dof);
        freezeImpedanceToLocalHold(dof, q_curr_wd, t_now, true);
        cur_dof_state = DofState::HOLDING;
        diag_note_watchdog_timeout(dof, elapsed);
        LOG_C1_WARN("[IMPEDANCE] DOF" + String(dof) + " watchdog timeout (" +
                    String(elapsed) + "ms > " + String((uint32_t)impedance_watchdog_ms) +
                    "ms) → LOCAL HOLD at " + String(q_curr_wd, 2) +
                    "° (impedance preserved)");
      }

      // Bumpless init on first impedance entry (from IDLE)
      if (pid_reset_needed[dof]) {
        float q_init = dof_data.valid[dof] ? dof_data.angles[dof] : 0.0f;
        PID *outer_pid_init = getOuterPID(dof);
        if (outer_pid_init) {
          outer_pid_init->initializeState(q_init, q_init, 0.0f);
        }
        delta_theta[dof] = 0.0f;
        delta_theta_prev[dof] = 0.0f;
        pid_reset_needed[dof] = false;
        inner_pid_init_needed[dof] = true;
        wp_canErrorTracker.clearErrors(dof);
        wp_first_read[dof] = true;
        wp_rev_track_init[dof] = false;
        wp_0x92_bootstrap_done[dof] = false;
        wp_watchdog_cycle_count[dof] = 0;
        wp_a1_miss_count[dof] = 0;
        wp_last_a1_miss_log_ms[dof] = 0;
        wp_prev_torque_A[dof] = 0;
        wp_prev_torque_B[dof] = 0;
        resetMovementSettleState(dof);
        LOG_C1_INFO("[IMPEDANCE] DOF " + String(dof) + " bumpless init at " + String(q_init, 1) + "°");
      }
    }

    // Track impedance mode status for this DOF (used by both outer and inner loop)
    bool impedance_active = impedance_target[dof].valid;

    // === OUTER LOOP (Joint PID) ===
    // Execute outer loop every N inner cycles (configurable via outer_loop_divisor)
    if (outer_cycle_due) {

#if CONTROLLER_DEBUG
      uint32_t outer_start_us = time_us_32();
#endif

      float q_des = 0.0f;
      float expected_velocity_deg_s = 0.0f;
      bool is_moving = false;

      if (impedance_active) {
        q_des = getImpedanceHoldReference(dof);
        evaluateImpedanceSegment(dof, t_now, q_des, expected_velocity_deg_s);
        is_moving = impedance_segment[dof].active;
        if (is_moving) {
          any_movement = true;
        }
        applyImpedanceOuterOverrides(dof, this);
      } else {
        // HOLDING mode - maintain target position
        q_des = dof_hold_angle[dof];
      }
      
      // Read current angle from shared state (updated by Core0)
      if (!dof_data.valid[dof]) {
        LOG_C1_WARN("[Control] Invalid encoder reading for DOF " + String(dof));
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

      // Segment timing reaching t_arrival is not enough to declare arrival. Keep the
      // joint in MOVING until the position error and filtered velocity are both small
      // for a few consecutive outer-loop cycles.
      if (impedance_active) {
        if (impedance_segment[dof].active || compliance_state[dof].compliance_active) {
          hold_entry_stable_count[dof] = 0;
        } else {
          const bool hold_ready = (abs_error <= HOLD_ENTRY_ERROR_BAND_DEG) &&
                                  (fabs(velocity_filtered[dof]) <= HOLD_ENTRY_MAX_VEL_DEG_S);
          if (hold_ready) {
            if (hold_entry_stable_count[dof] < 255) {
              hold_entry_stable_count[dof]++;
            }
          } else {
            hold_entry_stable_count[dof] = 0;
          }

          if (hold_entry_stable_count[dof] < HOLD_ENTRY_STABLE_CYCLES) {
            is_moving = true;
            any_movement = true;
          } else {
            is_moving = false;
          }
        }
      } else {
        hold_entry_stable_count[dof] = 0;
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
            if (impedance_active) {
              ImpedanceRollingSegment &seg = impedance_segment[dof];
              seg.q_goal_deg = q_curr;
              seg.q_start_deg = q_curr;
              seg.q_ref_deg = q_curr;
              seg.dq_ref_deg_s = 0.0f;
              seg.speed_abs_deg_s = 0.0f;
              seg.t_start_ms = t_now;
              seg.t_arrival_ms = t_now;
              seg.active = false;
              seg.initialized = true;
              impedance_target[dof].q_target_deg = q_curr;
              impedance_target[dof].dq_target_deg_s = 0.0f;
            }
            dof_hold_angle[dof] = q_curr;
            dof_hold_time[dof] = t_now;
            dof_state[dof] = DofState::HOLDING;
            cur_dof_state = DofState::HOLDING;
            is_moving = false;
            q_des = q_curr;
            expected_velocity_deg_s = 0.0f;
            error = 0.0f;
            abs_error = 0.0f;

            LOG_C1_INFO("[Compliance] DOF " + String(dof) + 
                     " STALL->HOLDING: trajectory aborted, holding at " + String(q_curr, 2) + "deg");
            
            // Notify host that trajectory was aborted due to stall
            // Host should stop sending targets and handle the situation
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
            if (impedance_active) {
              ImpedanceRollingSegment &seg = impedance_segment[dof];
              seg.q_goal_deg = q_curr;
              seg.q_start_deg = q_curr;
              seg.q_ref_deg = q_curr;
              seg.dq_ref_deg_s = 0.0f;
              seg.speed_abs_deg_s = 0.0f;
              seg.t_start_ms = t_now;
              seg.t_arrival_ms = t_now;
              seg.active = false;
              seg.initialized = true;
              impedance_target[dof].q_target_deg = q_curr;
              impedance_target[dof].dq_target_deg_s = 0.0f;
            }
            dof_hold_angle[dof] = q_curr;
            dof_hold_time[dof] = t_now;
            dof_state[dof] = DofState::HOLDING;
            cur_dof_state = DofState::HOLDING;
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

          float hold_ref = dof_data.valid[dof] ? dof_data.angles[dof] : getImpedanceHoldReference(dof);
          if (impedance_active) {
            clearImpedanceControlState(dof, this);
          }

          dof_hold_angle[dof] = hold_ref;
          dof_hold_time[dof] = t_now;
          dof_state[dof] = DofState::HOLDING;
          prev_dof_state[dof] = DofState::HOLDING;
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
      
      // Determine current state based on impedance target
      bool is_holding = !is_moving;
      
      // Detect MOVING → HOLDING transition by comparing with previous cycle's state
      bool just_entered_holding = (prev_dof_state[dof] == DofState::MOVING) && is_holding;
      
      // Update state to HOLDING if movement ended and we were MOVING
      // This ensures the state machine is consistent
      if (is_holding && (cur_dof_state == DofState::MOVING || just_entered_holding)) {
        if (impedance_active) {
          dof_hold_angle[dof] = getImpedanceHoldReference(dof);
          dof_hold_time[dof] = t_now;
        }
        dof_state[dof] = DofState::HOLDING;
        cur_dof_state = DofState::HOLDING;

        if (just_entered_holding) {
          hold_ki_ramp_start_ms[dof] = t_now;
          last_hold_event_q[dof] = q_curr;
          hold_event_q_valid[dof] = true;
          // Get the holding target for this DOF
          float holding_target = dof_hold_angle[dof];
          LOG_C1_DEBUG("[Control] DOF " + String(dof) + " transitioned MOVING → HOLDING");

          // Send structured message for UI display
          SERIAL_C1_COM_LN("EVT:HOLDING_TARGET:DOF=" + String(dof) + ":ANGLE=" + String(holding_target, 2));

          // Rebase the outer PID at HOLDING entry.
          //
          // Why:
          // - During MOVING the outer incremental PID may accumulate a large bias
          //   (uprev / delta_theta) to push through friction and track the segment.
          // - Carrying that bias into HOLDING can keep driving the joint even after
          //   the segment has ended, producing the observed drift past target.
          //
          // Strategy:
          // - Initialize the PID around the CURRENT joint angle, not the hold target.
          // - Start from zero output. This discards the movement bias but keeps the
          //   transfer bumpless: on the same cycle the controller sees the actual
          //   HOLDING error (holding_target - q_curr) as a fresh P-only correction.
          //
          // Result:
          // - No large carry-over from MOVING
          // - Still allows small HOLDING corrections if q_curr != holding_target
          PID *outer_pid_hold = getOuterPID(dof);
          if (outer_pid_hold) {
            outer_pid_hold->initializeState(q_curr, q_curr, 0.0f);
          }
          delta_theta[dof] = 0.0f;
          delta_theta_prev[dof] = 0.0f;
          
          if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
            metrics_finalize_pending[dof] = true;
            metrics_finalize_hold_start_ms[dof] = t_now;
          }
        }
      }
      
      // Determine if we should check safety:
      // - Always check joint limits in MOVING mode (every outer loop cycle = 100 Hz)
      // - Check periodically in HOLDING mode (every 10 cycles = ~100ms at 100Hz)
      // NOTE: We do NOT check immediately when entering HOLDING because motors may still be settling
      // Reduced from 20 cycles (200ms) to 10 cycles (100ms) for faster detection during manual push
      bool should_check_safety = is_moving || (is_holding && safety_check_counter >= 10);
      
      if (should_check_safety) {
#if CONTROLLER_DEBUG
        uint32_t safety_start_us = time_us_32();
#endif
        String safety_message;
        SafetyViolationType safety_violation_type = SAFETY_VIOLATION_LIMIT;
        // Check motors (tendon breakage) only in HOLDING mode periodically
        // NOT immediately when entering HOLDING - motors need time to settle
        bool check_motors = is_holding && (safety_check_counter >= 10);
        
        // NOTE: Debug log for periodic safety check was removed here because
        // String concatenation + Serial.print was causing ~2ms overhead per call,
        // which exceeded the 2ms cycle budget and caused control loop jitter.
        
        bool safety_ok = checkSafetyForDof(
            dof,
            q_curr,
            safety_message,
            check_motors,
            &safety_violation_type);
#if CONTROLLER_DEBUG
        {
          uint32_t safety_dt = time_us_32() - safety_start_us;
          loop_micro_profile.accum_safety_us += safety_dt;
          loop_micro_profile.safety_samples++;
          if (safety_dt > loop_micro_profile.max_safety_us) {
            loop_micro_profile.max_safety_us = safety_dt;
          }
        }
#endif
        if (!safety_ok) {
          // Safety violation detected - stop all motors immediately
          stopAllMotors();
          LOG_C1_ERROR("[Safety] MOVEMENT STOPPED: " + safety_message);
          diag_note_safety_violation(dof, safety_violation_type);

          float hold_ref = dof_data.valid[dof] ? q_curr : getImpedanceHoldReference(dof);
          if (impedance_active) {
            clearImpedanceControlState(dof, this);
          }

          dof_hold_angle[dof] = hold_ref;
          dof_hold_time[dof] = t_now;
          dof_state[dof] = DofState::HOLDING;
          prev_dof_state[dof] = DofState::HOLDING;
          pid_reset_needed[dof] = true;

          // Continue checking other DOFs (don't return, just skip this one)
          continue;
        }
      }
      
      // Snapshot previous state BEFORE updating — used by bias/slack gate_no_transition
      // Uses file-scope array declared near prev_dof_state.
      prev_state_before_update[dof] = prev_dof_state[dof];
      // Update previous state for next cycle (use updated dof_state)
      prev_dof_state[dof] = is_holding ? DofState::HOLDING : DofState::MOVING;

      // === UPDATE OUTER LOOP SAMPLING PERIOD ===
      // The PID controller needs the correct Ts for proper integral/derivative scaling
      // Only update when it changes (avoid overhead on every cycle)
      static float last_outer_loop_dt = 0.0f;
      float outer_loop_dt = effective_divisor * inner_loop_period_us / 1000000.0f;
      if (outer_loop_dt != last_outer_loop_dt) {
        setOuterLoopSamplingPeriod(outer_loop_dt);
        last_outer_loop_dt = outer_loop_dt;
      }

      // Compute delta_theta using outer loop controller
      PID *outer_pid = getOuterPID(dof);
      if (outer_pid) {
        // Save previous value for interpolation (smooth transitions when divisor > 1)
        delta_theta_prev[dof] = delta_theta[dof];

        // Cache expected velocity for inner loop stiffness scaling
        expected_velocity_cache[dof] = expected_velocity_deg_s;

        bool outer_holding_mode = !is_moving;
        float outer_ki_scale = 1.0f;
        if (outer_holding_mode) {
          if (hold_ki_ramp_start_ms[dof] == 0) {
            hold_ki_ramp_start_ms[dof] = t_now;
          }
          if (outer_hold_ki_ramp_ms == 0) {
            outer_ki_scale = outer_hold_ki_scale;
          } else {
            float ramp_alpha =
                (float)(t_now - hold_ki_ramp_start_ms[dof]) / (float)outer_hold_ki_ramp_ms;
            ramp_alpha = constrain(ramp_alpha, 0.0f, 1.0f);
            outer_ki_scale = 1.0f + (outer_hold_ki_scale - 1.0f) * ramp_alpha;
          }
        }
        bool freeze_outer_integrator =
            outer_holding_mode &&
            abs_error <= outer_hold_integral_freeze_error_deg &&
            fabs(velocity_filtered[dof]) <= outer_hold_integral_freeze_velocity_deg_s;
        last_outer_ki_scale_dbg[dof] = outer_ki_scale;
        last_outer_i_freeze_dbg[dof] = freeze_outer_integrator;

        // In HOLDING, static friction can turn tiny residual errors into stick-slip.
        // Keep Ki high for the first ~1 s to finish settling under load, then ramp
        // it down to a smaller steady-state value and freeze the integral entirely
        // once the joint is already near target and nearly still.
        delta_theta[dof] = outer_pid->control(q_des, q_curr, 0.0f, outer_ki_scale,
                                              freeze_outer_integrator);

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
        // (movement_start_ms is the first target's t_arrival, which may be in the future)
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

      if (metrics_finalize_pending[dof] && metrics_tracker[dof].tracking_active) {
        MetricsTracker &mt = metrics_tracker[dof];
        const bool hold_samples_ready = mt.sse_sample_count >= METRICS_FINALIZE_MIN_HOLD_SAMPLES;
        const bool settled_ready = mt.settling_time_ms > 0;
        const bool hold_timeout = metrics_finalize_hold_start_ms[dof] > 0 &&
                                  (t_now - metrics_finalize_hold_start_ms[dof] >= METRICS_FINALIZE_MAX_HOLD_MS);
        if (is_holding && (hold_samples_ready || settled_ready || hold_timeout)) {
          finalizeMovementMetricsForDof(dof);
        }
      }

#if CONTROLLER_DEBUG
      {
        uint32_t outer_dt = time_us_32() - outer_start_us;
        loop_micro_profile.accum_outer_us += outer_dt;
        if (outer_dt > loop_micro_profile.max_outer_us) {
          loop_micro_profile.max_outer_us = outer_dt;
        }
      }
#endif
    }
    
    // === INNER LOOP @ 500 Hz (Motor Control) ===
    // Run at full 500 Hz in both MOVING and HOLDING modes for best accuracy.
    // The MCP2515 flush mechanism in LKM_Motor handles any CAN buffer issues.
    // See: LKM_Motor::getMultiAngleSync() for the flush and retry logic.

    const bool direct_drive = (config.dofs[dof].drive_type == DRIVE_DIRECT_DRIVE);

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

    // Get current position reference for inner loop theta_0 calculation
    float theta_0_joint;

    if (impedance_active) {
      theta_0_joint = getImpedanceHoldReference(dof);
      float dq_ref_unused = 0.0f;
      evaluateImpedanceSegment(dof, t_now, theta_0_joint, dq_ref_unused);
    } else {
      // HOLDING: use last known position
      theta_0_joint = dof_hold_angle[dof];
    }

    if (direct_drive) {
      LKM_Motor *direct_motor = nullptr;
      PID *pid_direct = nullptr;
      int direct_idx = -1;
      for (int i = 0; i < config.motor_count; i++) {
        if (config.motors[i].dof_index == dof && config.motors[i].role == MOTOR_ROLE_DIRECT) {
          direct_motor = motors[i];
          pid_direct = pid_controllers[i];
          direct_idx = i;
          break;
        }
      }

      if (direct_motor == nullptr || pid_direct == nullptr || direct_idx < 0) {
        static uint32_t last_direct_warn_ms[MAX_DOFS] = {0};
        if (t_now - last_direct_warn_ms[dof] > 1000) {
          LOG_C1_WARN("[Control] DOF " + String(dof) +
                      " direct-drive motor/PID not configured");
          last_direct_warn_ms[dof] = t_now;
        }
        continue;
      }

      if (!dof_data.valid[dof]) {
        continue;
      }

      float q_direct_curr = dof_data.angles[dof];
      float q_direct_ref = theta_0_joint;

      if (inner_pid_init_needed[dof] || inner_pid_reinit_after_impedance[dof]) {
        pid_direct->initializeState(q_direct_curr, q_direct_ref, 0.0f);
        inner_pid_init_needed[dof] = false;
        inner_pid_reinit_after_impedance[dof] = false;
        LOG_C1_DEBUG("[Control] DOF " + String(dof) +
                     " direct-drive PID init: ref=" + String(q_direct_ref, 1) +
                     " curr=" + String(q_direct_curr, 1));
      }

      float uff_direct = 0.0f;
      if (friction_ff_enabled) {
        float speed = fabs(expected_velocity_cache[dof]);
        float T = friction_ff_speed_thresh;
        if (speed > 0.01f && speed <= 2.0f * T) {
          float direction = (expected_velocity_cache[dof] >= 0.0f) ? 1.0f : -1.0f;
          float gain = (speed <= T) ? 1.0f : (1.0f - (speed - T) / T);
          uff_direct = friction_ff_torque * direction * gain;
        }
      }
      if (impedance_active && impedance_target[dof].tau_ff != 0) {
        uff_direct += (float)impedance_target[dof].tau_ff;
      }

#if CONTROLLER_DEBUG
      uint32_t pid_start_us = time_us_32();
#endif
      if (impedance_active) {
        applyImpedanceInnerOverrideSingle(dof, pid_direct);
      }

      float command_A = pid_direct->control(q_direct_ref, q_direct_curr, uff_direct);
      float command_B = 0.0f;

      if (dof == 0 && pid_diag_terms_enabled) {
        pid_diagnostics.inner_p_term = (int16_t)constrain(pid_direct->last_up * 100.0f, -32767, 32767);
        pid_diagnostics.inner_i_term = (int16_t)constrain(pid_direct->last_ui * 100.0f, -32767, 32767);
        pid_diagnostics.inner_d_term = (int16_t)constrain(pid_direct->last_udfilt * 100.0f, -32767, 32767);
        pid_diagnostics.inner_ff_term = (int16_t)constrain(uff_direct * 100.0f, -32767, 32767);
        pid_diagnostics.pid_terms_valid = true;
      }

#if CONTROLLER_DEBUG
      {
        uint32_t pid_dt = time_us_32() - pid_start_us;
        loop_micro_profile.accum_pid_us += pid_dt;
        if (pid_dt > loop_micro_profile.max_pid_us) {
          loop_micro_profile.max_pid_us = pid_dt;
        }
      }
#endif

      float max_torque_direct = config.motors[direct_idx].max_torque;
      static float prev_command_direct[MAX_DOFS] = {0};
      command_A = constrain(command_A, -max_torque_direct, max_torque_direct);
      if (torque_ramp_time_ms > 0) {
        float rate = max_torque_direct * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);
        command_A = constrain(command_A,
                              prev_command_direct[dof] - rate,
                              prev_command_direct[dof] + rate);
      }
      prev_command_direct[dof] = command_A;

#if CONTROLLER_DEBUG
      uint32_t torque_start_us = time_us_32();
#endif
      direct_motor->setTorque((int)command_A);
#if CONTROLLER_DEBUG
      {
        uint32_t torque_dt = time_us_32() - torque_start_us;
        loop_micro_profile.accum_torque_us += torque_dt;
        if (torque_dt > loop_micro_profile.max_torque_us) {
          loop_micro_profile.max_torque_us = torque_dt;
        }
      }
#endif

      if (dof < 3) {
        pid_diagnostics.target_deg_x100[dof] = (int16_t)(theta_0_joint * 100.0f);
        pid_diagnostics.error_deg_x100[dof] = (int16_t)((theta_0_joint - q_direct_curr) * 100.0f);
        pid_diagnostics.torque_A[dof] = (int16_t)command_A;
        pid_diagnostics.torque_B[dof] = 0;

        if (metrics_tracking_enabled && metrics_tracker[dof].tracking_active) {
          int16_t abs_A = abs((int16_t)command_A);
          if (abs_A > metrics_tracker[dof].max_torque_A) {
            metrics_tracker[dof].max_torque_A = abs_A;
          }
          uint32_t torque_sum = abs_A;
          if (metrics_tracker[dof].torque_integral < 0xFFFFFFFF - torque_sum) {
            metrics_tracker[dof].torque_integral += torque_sum;
          }
        }
      }

#if CONTROLLER_DEBUG
      {
        uint32_t dof_dt = time_us_32() - dof_start_us;
        loop_micro_profile.accum_dof_us += dof_dt;
        if (dof_dt > loop_micro_profile.max_dof_us) {
          loop_micro_profile.max_dof_us = dof_dt;
        }
        loop_micro_profile.samples++;
      }
#endif
      continue;
    }

    if (agonist == nullptr || antagonist == nullptr) {
      // No motors for this DOF, skip
      continue;
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
      loop_micro_profile.accum_eq_us += eq_dt;
      if (eq_dt > loop_micro_profile.max_eq_us) {
        loop_micro_profile.max_eq_us = eq_dt;
      }
    }
#endif
    
    if (!equations_ok) {
      // No linear equations, cannot control this DOF
      static uint32_t last_warn_time = 0;
      if (millis() - last_warn_time > 5000) {
        LOG_C1_WARN("[Control] DOF " + String(dof) + " has no linear equations, cannot move");
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
    float q_retension = dof_data.valid[dof] ? dof_data.angles[dof] : 0.0f;

    // === RETENSION PROBE (periodic active sensing, host-driven policy) ===
    // Use the antagonistic actuation as an active probe: apply a short,
    // symmetric co-contraction pulse during clean HOLDING and measure whether
    // the weak side gets "recruited". Firmware only measures and reports the
    // result; host-side policy decides whether to react or learn over time.
    float stiffness_ref_effective = stiffness_ref;
    if (dof < MAX_DOFS) {
      RetensionProbeState &rps = retension_probe_state[dof];
      bool recent_hold_event =
          (last_hold_event_log_ms[dof] != 0) &&
          ((t_now - last_hold_event_log_ms[dof]) < RETN_PROBE_HOLD_EVT_BLOCK_MS);
      bool hard_probe_context =
          retension_probe_enabled &&
          impedance_active &&
          dof_state[dof] == DofState::HOLDING &&
          retension_probe_boost_deg > 0.0f &&
          fabs(q_retension) >= retension_probe_min_hold_q_deg &&
          !compliance_state[dof].compliance_active &&
          impedance_target[dof].tau_ff == 0 &&
          dof_data.valid[dof];
      bool soft_probe_ready =
          fabs(velocity_filtered[dof]) < 1.0f &&
          prev_state_before_update[dof] == DofState::HOLDING &&
          !recent_hold_event;

      if (!hard_probe_context) {
        resetRetensionProbeState(dof);
      } else {
        if (rps.phase == RetensionProbePhase::IDLE) {
          rps.phase = RetensionProbePhase::ARMED;
          rps.scheduled_start_ms = t_now + retension_probe_start_delay_ms;
          clearRetensionProbeWindows(rps);
        }

        if (rps.phase == RetensionProbePhase::DONE &&
            retension_probe_repeat_ms > 0 &&
            t_now >= rps.scheduled_start_ms) {
          rps.phase = RetensionProbePhase::ARMED;
          rps.scheduled_start_ms = t_now + retension_probe_start_delay_ms;
          clearRetensionProbeWindows(rps);
        }

        if (rps.phase == RetensionProbePhase::ARMED && !soft_probe_ready) {
          rps.scheduled_start_ms = t_now + retension_probe_start_delay_ms;
          clearRetensionProbeWindows(rps);
        }

        if (rps.phase == RetensionProbePhase::ARMED &&
            soft_probe_ready &&
            t_now >= rps.scheduled_start_ms &&
            retension_probe_pulse_ms > 0) {
          rps.phase = RetensionProbePhase::ACTIVE;
          rps.active_until_ms = t_now + retension_probe_pulse_ms;
          LOG_C1_INFO("[RPROBE] DOF" + String(dof) +
                      " start q=" + String(q_retension, 2) +
                      " stiff=" + String(stiffness_ref, 1) + "->" +
                      String(stiffness_ref + retension_probe_boost_deg, 1) +
                      " age=" + String(t_now - dof_hold_time[dof]) + "ms" +
                      " dur=" + String(retension_probe_pulse_ms) + "ms");
        }

        if (rps.phase == RetensionProbePhase::ACTIVE && t_now >= rps.active_until_ms) {
          rps.phase = RetensionProbePhase::POST;
          rps.post_until_ms = t_now + retension_probe_post_ms;
          LOG_C1_INFO("[RPROBE] DOF" + String(dof) +
                      " end q=" + String(q_retension, 2) +
                      " age=" + String(t_now - dof_hold_time[dof]) + "ms");
        }

        if (rps.phase == RetensionProbePhase::ACTIVE) {
          stiffness_ref_effective += retension_probe_boost_deg;
          last_retension_boost_dbg[dof] = retension_probe_boost_deg;
        } else {
          last_retension_boost_dbg[dof] = 0.0f;
        }
      }
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
    
    // Compute motor references using cascade control formula
    float theta_A_ref = theta_0_agonist_motor +
                        cascade_influence * (0.5f * delta_theta_smooth + 0.5f * stiffness_ref_effective);
    float theta_B_ref = theta_0_antagonist_motor +
                        cascade_influence * (0.5f * delta_theta_smooth - 0.5f * stiffness_ref_effective);

    // === HOLDING BIAS TRACKER ===
    // Track EMA of delta_theta during HOLDING to detect persistent outer-loop
    // compensation. Gated per SLACK_DETECTION_AND_TENSION_TRIM.md §Gating.
    // Reset when ANY gate condition falls (bias is only meaningful in clean steady state).
    {
      bool gate_holding      = (dof < MAX_DOFS && dof_state[dof] == DofState::HOLDING);
      // Gate on baseline stiffness, not probe/effective stiffness, so
      // diagnostics stay comparable and probe boosts do not change the
      // nominal "is this a stiff hold?" context.
      bool gate_stiffness    = (stiffness_ref > 1.0f);
      bool gate_no_compliance = !compliance_state[dof].compliance_active;
      bool gate_no_tau_ff    = (!impedance_active || impedance_target[dof].tau_ff == 0);
      bool gate_low_velocity = (fabs(velocity_filtered[dof]) < 0.5f);  // near-zero motion
      bool gate_no_transition = (prev_state_before_update[dof] == DofState::HOLDING);  // not just entered
      bool gate_valid_encoder = dof_data.valid[dof];

      bool all_gates = gate_holding && gate_stiffness && gate_no_compliance &&
                       gate_no_tau_ff && gate_low_velocity && gate_no_transition &&
                       gate_valid_encoder;

      if (all_gates) {
        if (holding_ema_samples[dof] == 0) {
          // First sample after all gates pass — seed EMA
          holding_dtheta_ema[dof] = delta_theta_smooth;
        } else {
          holding_dtheta_ema[dof] += HOLDING_EMA_ALPHA * (delta_theta_smooth - holding_dtheta_ema[dof]);
        }
        holding_ema_samples[dof]++;

        // Note: unified [DIAG_HOLD] log is emitted after the slack detector block,
        // where trResp (Iq data) is also available.
      } else if (dof < MAX_DOFS) {
        // Any gate failed — reset accumulator so bias starts clean next time
        if (holding_ema_samples[dof] > 0) {
          holding_ema_samples[dof] = 0;
          holding_dtheta_ema[dof] = 0;
        }
      }
    }

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
    
    // === PHASE 2: Motor angle acquisition ===
    // Bootstrap (first cycle): 0x92 provides absolute multi-turn angle for rev tracking init.
    // Normal cycles: use tracked angle from previous 0xA1 torque response (no CAN round-trip).
    // Watchdog (~1Hz): re-read 0x92 to verify tracking hasn't drifted.
    bool recover_after_a1_miss = wp_a1_miss_count[dof] > 0;
    bool need_0x92 = !wp_0x92_bootstrap_done[dof] || recover_after_a1_miss;
    if (wp_0x92_bootstrap_done[dof] && !recover_after_a1_miss) {
      wp_watchdog_cycle_count[dof]++;
      if (wp_watchdog_cycle_count[dof] >= WP_WATCHDOG_INTERVAL) {
        need_0x92 = true;
        wp_watchdog_cycle_count[dof] = 0;
      }
    }

    PipelinedAngleData pipelined = {};
    MultiAngleData data_A = {};
    MultiAngleData data_B = {};
    data_A.angle = NAN;
    data_B.angle = NAN;

#if CONTROLLER_DEBUG
    uint32_t can_start_us = time_us_32();
#endif
    if (need_0x92) {
      pipelined = LKM_Motor::getMultiAnglePairPipelined(agonist, antagonist);
      data_A = pipelined.dataA;
      data_B = pipelined.dataB;
    }
#if CONTROLLER_DEBUG
    {
      uint32_t can_dt = time_us_32() - can_start_us;
      loop_micro_profile.accum_can_us += can_dt;
      if (can_dt > loop_micro_profile.max_can_us) {
        loop_micro_profile.max_can_us = can_dt;
      }
    }
#endif

    float theta_A_curr, theta_B_curr;
    if (need_0x92) {
      // Bootstrap or watchdog cycle: 0x92 is source of truth
      theta_A_curr = data_A.angle;
      theta_B_curr = data_B.angle;
    } else {
      // Normal cycle: use tracked angle from 0xA1 (updated by setTorquePairPipelined)
      theta_A_curr = agonist->getTrackedAngle();
      theta_B_curr = antagonist->getTrackedAngle();
      if (isnan(theta_A_curr) || isnan(theta_B_curr)) {
        // Tracking not yet initialized — should not happen after bootstrap, skip DOF
        continue;
      }
    }
    
    // === SANITY CHECK: Detect obviously invalid readings ===
    // Values outside ±100000° are clearly garbage (CAN corruption)
    bool invalid_A = (theta_A_curr < -100000.0f || theta_A_curr > 100000.0f || isnan(theta_A_curr));
    bool invalid_B = (theta_B_curr < -100000.0f || theta_B_curr > 100000.0f || isnan(theta_B_curr));
    
    if (invalid_A || invalid_B) {
      diag_note_encoder_invalid(dof);
      static uint32_t last_invalid_log = 0;
      if (millis() - last_invalid_log > 100) { // Log max every 100ms
        LOG_C1_ERROR("[Control] DOF " + String(dof) + " INVALID CAN READ: A=" + 
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
    // so they can be reset when a new impedance sequence starts

    if (!wp_first_read[dof]) {
      float jump_A = fabs(theta_A_curr - wp_last_theta_A[dof]);
      float jump_B = fabs(theta_B_curr - wp_last_theta_B[dof]);

      // If motor angle jumped more than 30° in one cycle (2ms), something is wrong
      if (jump_A > 30.0f || jump_B > 30.0f) {
        wp_canErrorTracker.recordError(dof);
        uint8_t recent_errors = wp_canErrorTracker.countRecentErrors(dof);

        LOG_C1_ERROR("[DIAG] DOF " + String(dof) + " MOTOR ANGLE JUMP (" +
                  String(recent_errors) + " errors in " + String(can_error_window_ms) + "ms)!");
        LOG_C1_ERROR("  Agonist: " + String(wp_last_theta_A[dof], 2) + " → " + String(theta_A_curr, 2) +
                  " (jump=" + String(jump_A, 2) + "°)");
        LOG_C1_ERROR("  Antagonist: " + String(wp_last_theta_B[dof], 2) + " → " + String(theta_B_curr, 2) +
                  " (jump=" + String(jump_B, 2) + "°)");

        // Trigger emergency stop if too many errors within time window
        if (wp_canErrorTracker.shouldStop(dof)) {
          LOG_C1_ERROR("[Control] DOF " + String(dof) + " - " + String(recent_errors) +
                    " CAN errors in " + String(can_error_window_ms) + "ms, EMERGENCY STOP!");
          stopAllMotors();
          float hold_ref = dof_data.valid[dof] ? dof_data.angles[dof] : getImpedanceHoldReference(dof);
          if (impedance_active) {
            clearImpedanceControlState(dof, this);
          }
          dof_hold_angle[dof] = hold_ref;
          dof_hold_time[dof] = t_now;
          dof_state[dof] = DofState::HOLDING;
          wp_canErrorTracker.clearErrors(dof);
          wp_first_read[dof] = true;
          prev_dof_state[dof] = DofState::HOLDING;
          pid_reset_needed[dof] = true;
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

    // Motor angles pass through directly to inner PID (no filtering)
    float theta_A_pid = theta_A_curr;
    float theta_B_pid = theta_B_curr;

    // === BUMPLESS TRANSFER for inner PIDs on IDLE → MOVING ===
    // Initialize inner PID state at current motor positions so the first control()
    // call produces zero P and D terms, avoiding derivative kick and proportional jump.
    // This is deferred from the IDLE→MOVING detection above because motor references
    // (theta_A_ref, theta_B_ref) and CAN readings (theta_A_pid, theta_B_pid) are only
    // available at this point in the loop.
    if (inner_pid_init_needed[dof] || inner_pid_reinit_after_impedance[dof]) {
      pid_agonist->initializeState(theta_A_pid, theta_A_ref, 0.0f);
      pid_antagonist->initializeState(theta_B_pid, theta_B_ref, 0.0f);
      if (inner_pid_reinit_after_impedance[dof]) {
        LOG_C1_DEBUG("[Control] DOF " + String(dof) + " inner PID reinit after impedance");
      } else {
        LOG_C1_DEBUG("[Control] DOF " + String(dof) + " inner PID bumpless init:"
                     " Aref=" + String(theta_A_ref, 1) + " Acurr=" + String(theta_A_pid, 1) +
                     " Bref=" + String(theta_B_ref, 1) + " Bcurr=" + String(theta_B_pid, 1));
      }
      inner_pid_init_needed[dof] = false;
      inner_pid_reinit_after_impedance[dof] = false;
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

    // === IMPEDANCE FEEDFORWARD TORQUE ===
    // In impedance mode, add tau_ff from Jetson to the friction feedforward.
    // tau_ff > 0 pushes agonist direction, distributed as agonist+, antagonist-.
    if (impedance_active && impedance_target[dof].tau_ff != 0) {
      float tau_ff = (float)impedance_target[dof].tau_ff;
      uff_A += tau_ff;
      uff_B -= tau_ff;
    }

    // === INNER LOOP: incremental PID for impedance control ===
#if CONTROLLER_DEBUG
    uint32_t pid_start_us = time_us_32();
#endif
    float command_A, command_B;

    if (impedance_active) {
      applyImpedanceInnerOverrides(dof, pid_agonist, pid_antagonist);
    }

    command_A = pid_agonist->control(theta_A_ref, theta_A_pid, uff_A);
    command_B = pid_antagonist->control(theta_B_ref, theta_B_pid, uff_B);

    // Diagnostics (DOF 0)
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
      loop_micro_profile.accum_pid_us += pid_dt;
      if (pid_dt > loop_micro_profile.max_pid_us) {
        loop_micro_profile.max_pid_us = pid_dt;
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
        LOG_C1_WARN("[DIAG] DOF " + String(dof) + " HIGH TORQUE: A=" + String(command_A, 0) + 
                 " B=" + String(command_B, 0) + " (max=" + String(max_torque_A_effective, 0) + ")");
        LOG_C1_WARN("  refs: A=" + String(theta_A_ref, 2) + " B=" + String(theta_B_ref, 2));
        LOG_C1_WARN("  curr: A=" + String(theta_A_curr, 2) + " B=" + String(theta_B_curr, 2));
        last_torque_warn = millis();
      }
    }
    
    // Send torque commands to motors via pipelined 0xA1 + read response (shadow mode)
    // The 0xA1 response contains motor state (temp, iq, speed, encoder) which was
    // previously discarded. We now parse it for revolution tracking validation.
#if CONTROLLER_DEBUG
    uint32_t torque_start_us = time_us_32();
#endif
    PipelinedTorqueResponseData trResp = LKM_Motor::setTorquePairPipelined(
        agonist, (int)command_A, antagonist, (int)command_B);

    // === PHASE 2: Revolution tracking bootstrap and watchdog ===
    // Bootstrap (first cycle): init rev tracking from 0x92 absolute + 0xA1 encoder.
    // Watchdog (~1Hz): compare tracked angle vs 0x92, resync if drift > 1°.
    // Normal cycles: 0xA1 response updates rev tracking internally (no action needed here).
    if (trResp.dataA.valid && trResp.dataB.valid) {
      if (!wp_rev_track_init[dof] || recover_after_a1_miss) {
        // Bootstrap or recover after a missed 0xA1 response: need 0x92 absolute
        // angle to re-anchor the tracked encoder stream before trusting it again.
        agonist->initRevTracking(data_A.rawMotorAngle_centideg, trResp.dataA.encoder);
        antagonist->initRevTracking(data_B.rawMotorAngle_centideg, trResp.dataB.encoder);
        wp_rev_track_init[dof] = true;
        wp_0x92_bootstrap_done[dof] = true;
        wp_a1_miss_count[dof] = 0;
        if (recover_after_a1_miss) {
          LOG_C1_WARN("[Phase2] DOF " + String(dof) +
                      " re-anchored after missed 0xA1 response"
                      " (encA=" + String(trResp.dataA.encoder) +
                      " encB=" + String(trResp.dataB.encoder) + ")");
        } else {
          LOG_C1_INFO("[Phase2] DOF " + String(dof) + " bootstrap done → tracked-only mode"
                      " (encA=" + String(trResp.dataA.encoder) +
                      " encB=" + String(trResp.dataB.encoder) + ")");
        }
      } else if (need_0x92 && !isnan(data_A.angle) && !isnan(data_B.angle)) {
        // Watchdog cycle: compare tracked vs 0x92 absolute
        float tracked_A = agonist->getTrackedAngle();
        float tracked_B = antagonist->getTrackedAngle();
        float err_A = fabs(tracked_A - data_A.angle);
        float err_B = fabs(tracked_B - data_B.angle);

        LOG_C1_INFO("[Watchdog] DOF " + String(dof) +
                    " A: 0x92=" + String(data_A.angle, 3) +
                    " tracked=" + String(tracked_A, 3) + " err=" + String(err_A, 4) +
                    " spd=" + String(trResp.dataA.motorSpeed) +
                    " iq=" + String(trResp.dataA.torqueCurrent));
        LOG_C1_INFO("[Watchdog] DOF " + String(dof) +
                    " B: 0x92=" + String(data_B.angle, 3) +
                    " tracked=" + String(tracked_B, 3) + " err=" + String(err_B, 4) +
                    " spd=" + String(trResp.dataB.motorSpeed) +
                    " iq=" + String(trResp.dataB.torqueCurrent));

        // Resync if drift exceeds 1° (output shaft)
        if (err_A > 1.0f || err_B > 1.0f) {
          agonist->initRevTracking(data_A.rawMotorAngle_centideg, trResp.dataA.encoder);
          antagonist->initRevTracking(data_B.rawMotorAngle_centideg, trResp.dataB.encoder);
          LOG_C1_WARN("[Watchdog] DOF " + String(dof) + " RESYNC: errA=" +
                      String(err_A, 2) + "° errB=" + String(err_B, 2) + "°");
        }
      }
    } else if (wp_0x92_bootstrap_done[dof]) {
      // Tracked-only cycle but at least one 0xA1 response was missed.
      // Do not keep consuming stale tracked angles indefinitely: force a 0x92
      // recovery on the very next cycle.
      if (wp_a1_miss_count[dof] < 255) {
        wp_a1_miss_count[dof]++;
      }
      uint32_t now_ms = millis();
      if (now_ms - wp_last_a1_miss_log_ms[dof] > 500) {
        LOG_C1_WARN("[Phase2] DOF " + String(dof) +
                    " missed 0xA1 response (A=" + String(trResp.dataA.valid ? 1 : 0) +
                    " B=" + String(trResp.dataB.valid ? 1 : 0) +
                    ") → forcing 0x92 recovery next cycle");
        wp_last_a1_miss_log_ms[dof] = now_ms;
      }
    }

    // === SLACK TENDON DETECTION ===
    // In clean HOLDING with stiffness > 0, both motors must pull (non-zero Iq).
    // If one motor's |Iq| is much smaller than the other's, the tendon is likely slack.
    // Gravity shifts the ratio but never zeroes one side completely.
    // Gated per SLACK_DETECTION_AND_TENSION_TRIM.md §Gating to avoid false positives.
    {
      static uint16_t slack_count[MAX_DOFS] = {};       // consecutive low-ratio samples per DOF
      static uint32_t last_slack_warn[MAX_DOFS] = {};   // rate-limit warnings
      const uint16_t SLACK_THRESHOLD_COUNT = 7500;  // ~15s at 500Hz before alarm (let gravity/transient settle)
      const float   SLACK_RATIO_THRESHOLD = 0.05f; // 5% ratio = nearly zero on one side
      const int16_t SLACK_MIN_IQ = 30;           // ignore when both motors are near-idle

      bool slack_gated = trResp.dataA.valid && trResp.dataB.valid &&
                         dof_state[dof] == DofState::HOLDING && stiffness_ref > 1.0f &&
                         !compliance_state[dof].compliance_active &&
                         (!impedance_active || impedance_target[dof].tau_ff == 0) &&
                         fabs(velocity_filtered[dof]) < 0.5f &&
                         prev_state_before_update[dof] == DofState::HOLDING &&
                         dof_data.valid[dof];

      if (slack_gated) {
        int16_t iq_A = abs(trResp.dataA.torqueCurrent);
        int16_t iq_B = abs(trResp.dataB.torqueCurrent);
        int16_t iq_max = max(iq_A, iq_B);
        int16_t iq_min = min(iq_A, iq_B);

        if (dof < MAX_DOFS) {
          if (iq_max > SLACK_MIN_IQ) {
            float ratio = (float)iq_min / (float)iq_max;
            if (ratio < SLACK_RATIO_THRESHOLD) {
              slack_count[dof]++;
              if (slack_count[dof] >= SLACK_THRESHOLD_COUNT &&
                  t_now - last_slack_warn[dof] > 3000) {
                const char* side = (iq_A < iq_B) ? "AGONIST" : "ANTAGONIST";
                LOG_C1_WARN("⚠️ [SLACK] DOF " + String(dof) + " " + side +
                            " tendon slack! iqA=" + String(trResp.dataA.torqueCurrent) +
                            " iqB=" + String(trResp.dataB.torqueCurrent) +
                            " ratio=" + String(ratio, 3) +
                            " stiffBase=" + String(stiffness_ref, 1) +
                            " stiffEff=" + String(stiffness_ref_effective, 1));
                last_slack_warn[dof] = t_now;
                slack_count[dof] = 0;  // reset after warning
              }
            } else {
              slack_count[dof] = 0;  // reset on healthy sample
            }
          } else {
            slack_count[dof] = 0;  // near-idle: non-informative, reset persistence
          }
        }
      } else if (dof < 3) {
        // Gate failed — reset so persistence requires continuous clean holding
        slack_count[dof] = 0;
      }
    }

    // === HOLDING EVENT LOGGER (high-rate trigger) ===
    // Purpose: catch the exact instant a "mysterious" HOLDING jump happens and
    // tell whether it was preceded by a command change or by a mechanical release.
    // Trigger on either:
    // - sudden q jump between consecutive HOLDING samples
    // - non-trivial velocity while already in HOLDING
    {
      bool holding_event_gated = (dof < MAX_DOFS) &&
                                 (dof_state[dof] == DofState::HOLDING) &&
                                 dof_data.valid[dof];
      if (holding_event_gated) {
        float q_curr_hold_evt = dof_data.angles[dof];
        float q_des_hold_evt = impedance_active ? getImpedanceHoldReference(dof)
                                                : dof_hold_angle[dof];
        float error_hold_evt = q_des_hold_evt - q_curr_hold_evt;
        if (!hold_event_q_valid[dof]) {
          last_hold_event_q[dof] = q_curr_hold_evt;
          hold_event_q_valid[dof] = true;
        } else {
          float q_prev_hold = last_hold_event_q[dof];
          float q_step_hold = fabs(q_curr_hold_evt - q_prev_hold);
          float vel_hold = fabs(velocity_filtered[dof]);
          bool hold_jump = (q_step_hold >= HOLD_EVENT_Q_STEP_TH_DEG);
          bool hold_motion = (vel_hold >= HOLD_EVENT_VEL_TH_DEG_S);

          if ((hold_jump || hold_motion) &&
              (t_now - last_hold_event_log_ms[dof] >= HOLD_EVENT_MIN_INTERVAL_MS)) {
            bool iq_valid_evt = trResp.dataA.valid && trResp.dataB.valid;
            int16_t iq_A_evt = iq_valid_evt ? trResp.dataA.torqueCurrent : 0;
            int16_t iq_B_evt = iq_valid_evt ? trResp.dataB.torqueCurrent : 0;
            int16_t iq_abs_max_evt = max(abs(iq_A_evt), abs(iq_B_evt));
            float iq_ratio_evt = (iq_valid_evt && iq_abs_max_evt > 0)
                ? (float)min(abs(iq_A_evt), abs(iq_B_evt)) / (float)iq_abs_max_evt
                : -1.0f;

            LOG_C1_WARN("[HOLD_EVT] DOF" + String(dof) +
                        " qPrev=" + String(q_prev_hold, 3) +
                        " q=" + String(q_curr_hold_evt, 3) +
                        " qDes=" + String(q_des_hold_evt, 3) +
                        " err=" + String(error_hold_evt, 3) +
                        " dq=" + String(q_curr_hold_evt - q_prev_hold, 3) +
                        " vel=" + String(velocity_filtered[dof], 3) +
                        " dth=" + String(delta_theta_smooth, 3) +
                        " kiS=" + String(last_outer_ki_scale_dbg[dof], 3) +
                        " frz=" + String(last_outer_i_freeze_dbg[dof] ? 1 : 0) +
                        " rt=" + String(last_retension_boost_dbg[dof], 2) +
                        " Aref=" + String(theta_A_ref, 3) +
                        " Bref=" + String(theta_B_ref, 3) +
                        " cmdA=" + String(command_A, 1) +
                        " cmdB=" + String(command_B, 1) +
                        " iqA=" + String(iq_A_evt) +
                        " iqB=" + String(iq_B_evt) +
                        " iqR=" + String(iq_ratio_evt, 3));
            last_hold_event_log_ms[dof] = t_now;
          }

          last_hold_event_q[dof] = q_curr_hold_evt;
        }
      } else if (dof < MAX_DOFS) {
        hold_event_q_valid[dof] = false;
      }
    }

    // === RETENSION PROBE METRICS (per-cycle, independent from DIAG_HOLD cadence) ===
    if (dof < MAX_DOFS && trResp.dataA.valid && trResp.dataB.valid) {
      RetensionProbeState &rps = retension_probe_state[dof];
      float abs_iq_a = (float)abs(trResp.dataA.torqueCurrent);
      float abs_iq_b = (float)abs(trResp.dataB.torqueCurrent);
      float ema_probe = holding_dtheta_ema[dof];
      float q_probe = dof_data.angles[dof];

      if (rps.phase == RetensionProbePhase::ARMED) {
        rps.pre_abs_iq_a_sum += abs_iq_a;
        rps.pre_abs_iq_b_sum += abs_iq_b;
        rps.pre_q_sum += q_probe;
        rps.pre_ema_sum += ema_probe;
        rps.pre_count++;
      } else if (rps.phase == RetensionProbePhase::ACTIVE) {
        rps.during_abs_iq_a_sum += abs_iq_a;
        rps.during_abs_iq_b_sum += abs_iq_b;
        rps.during_q_sum += q_probe;
        rps.during_ema_sum += ema_probe;
        rps.during_count++;
      }

      if (rps.phase == RetensionProbePhase::POST) {
        rps.post_abs_iq_a_sum += abs_iq_a;
        rps.post_abs_iq_b_sum += abs_iq_b;
        rps.post_q_sum += q_probe;
        rps.post_ema_sum += ema_probe;
        rps.post_count++;
      }

      if (rps.phase == RetensionProbePhase::POST && t_now >= rps.post_until_ms) {
        if (rps.pre_count > 0 && rps.during_count > 0 && rps.post_count > 0) {
          float pre_a = rps.pre_abs_iq_a_sum / (float)rps.pre_count;
          float pre_b = rps.pre_abs_iq_b_sum / (float)rps.pre_count;
          float pre_q = rps.pre_q_sum / (float)rps.pre_count;
          float pre_ema = rps.pre_ema_sum / (float)rps.pre_count;
          float dur_a = rps.during_abs_iq_a_sum / (float)rps.during_count;
          float dur_b = rps.during_abs_iq_b_sum / (float)rps.during_count;
          float dur_q = rps.during_q_sum / (float)rps.during_count;
          float dur_ema = rps.during_ema_sum / (float)rps.during_count;
          float post_a = rps.post_abs_iq_a_sum / (float)rps.post_count;
          float post_b = rps.post_abs_iq_b_sum / (float)rps.post_count;
          float post_q = rps.post_q_sum / (float)rps.post_count;
          float post_ema = rps.post_ema_sum / (float)rps.post_count;

          float pre_min = min(pre_a, pre_b);
          float pre_max = max(pre_a, pre_b);
          float dur_min = min(dur_a, dur_b);
          float dur_max = max(dur_a, dur_b);
          float pre_ratio = (pre_max > 0.0f) ? (pre_min / pre_max) : -1.0f;
          float dur_ratio = (dur_max > 0.0f) ? (dur_min / dur_max) : -1.0f;
          float delta_ratio = (pre_ratio >= 0.0f && dur_ratio >= 0.0f)
              ? (dur_ratio - pre_ratio)
              : 0.0f;
          float recruit_norm = (pre_max > 0.0f)
              ? ((dur_min - pre_min) / pre_max)
              : 0.0f;
          float effort_pre = pre_a + pre_b;
          const char* weak_side = (pre_a <= pre_b) ? "A" : "B";
          uint8_t class_code = RPROBE_CLASS_NO_EFFECT;
          const char* classification = "NO_EFFECT";
          if (effort_pre < 120.0f) {
            class_code = RPROBE_CLASS_LOW_EFFORT;
            classification = "LOW_EFFORT";
          } else if (pre_ratio >= 0.0f && pre_ratio < 0.65f &&
                     delta_ratio > 0.08f && recruit_norm > 0.12f) {
            class_code = RPROBE_CLASS_SLACK_LIKELY;
            classification = "SLACK_LIKELY";
          } else if (pre_ratio >= 0.80f || fabs(delta_ratio) < 0.03f) {
            class_code = RPROBE_CLASS_NO_CORRECTION;
            classification = "NO_CORRECTION";
          }

          if (dof < MAX_DOFS) {
            uint16_t min_samples_u16 = rps.pre_count;
            if (rps.during_count < min_samples_u16) min_samples_u16 = rps.during_count;
            if (rps.post_count < min_samples_u16) min_samples_u16 = rps.post_count;
            if (min_samples_u16 > 255) min_samples_u16 = 255;
            uint8_t min_samples = (uint8_t)min_samples_u16;
            __atomic_add_fetch(&retension_probe_result_data[dof].seq, 1, __ATOMIC_RELEASE);
            retension_probe_result_data[dof].dof = dof;
            retension_probe_result_data[dof].q_x100 = (int16_t)(pre_q * 100.0f);
            retension_probe_result_data[dof].base_stiffness_x10 = (int16_t)(stiffness_ref * 10.0f);
            retension_probe_result_data[dof].pre_ratio_x1000 = (int16_t)(pre_ratio * 1000.0f);
            retension_probe_result_data[dof].dur_ratio_x1000 = (int16_t)(dur_ratio * 1000.0f);
            retension_probe_result_data[dof].delta_ratio_x1000 = (int16_t)(delta_ratio * 1000.0f);
            retension_probe_result_data[dof].recruit_norm_x1000 = (int16_t)(recruit_norm * 1000.0f);
            retension_probe_result_data[dof].effort_pre = (uint16_t)constrain((int)lroundf(effort_pre), 0, 65535);
            retension_probe_result_data[dof].boost_x10 = (int16_t)(retension_probe_boost_deg * 10.0f);
            retension_probe_result_data[dof].pulse_ms = retension_probe_pulse_ms;
            retension_probe_result_data[dof].flags = (pre_a <= pre_b) ? 0x00 : 0x01;
            retension_probe_result_data[dof].class_code = class_code;
            retension_probe_result_data[dof].min_samples = min_samples;
            __atomic_add_fetch(&retension_probe_result_data[dof].seq, 1, __ATOMIC_RELEASE);
          }

          LOG_C1_INFO("[RPROBE] DOF" + String(dof) +
                      " result weak=" + String(weak_side) +
                      " cls=" + String(classification) +
                      " preR=" + String(pre_ratio, 3) +
                      " durR=" + String(dur_ratio, 3) +
                      " dR=" + String(delta_ratio, 3) +
                      " dMinN=" + String(recruit_norm, 3) +
                      " preEff=" + String(effort_pre, 1) +
                      " dqD=" + String(dur_q - pre_q, 3) +
                      " dqP=" + String(post_q - pre_q, 3) +
                      " dEmaD=" + String(dur_ema - pre_ema, 3) +
                      " dEmaP=" + String(post_ema - pre_ema, 3) +
                      " preA=" + String(pre_a, 1) +
                      " preB=" + String(pre_b, 1) +
                      " durA=" + String(dur_a, 1) +
                      " durB=" + String(dur_b, 1) +
                      " postA=" + String(post_a, 1) +
                      " postB=" + String(post_b, 1));
        } else {
          LOG_C1_WARN("[RPROBE] DOF" + String(dof) +
                      " incomplete pre=" + String(rps.pre_count) +
                      " dur=" + String(rps.during_count) +
                      " post=" + String(rps.post_count));
        }
        rps.phase = RetensionProbePhase::DONE;
        rps.scheduled_start_ms = t_now + retension_probe_repeat_ms;
      }
    }

    // === UNIFIED DIAGNOSTIC LOG ===
    // All Phase 1 signals in one parsable line. Emission cadence is intentionally
    // independent from the slow trim dry-run cadence: we want fast telemetry from
    // the first gated samples to study short transients and external perturbations,
    // without making proposed_trim evolve too quickly.
    if (dof < 3 && holding_ema_samples[dof] >= diag_hold_min_samples &&
        t_now - last_holding_bias_log[dof] > diag_hold_period_ms) {
      // Motor residual: actual calibrated motor angle vs expected from equations.
      // Uses cached_motor_angles (live CAN with offset applied) — correct motor-space
      // geometric residual per SLACK_DETECTION_AND_TENSION_TRIM.md §C.
      float expected_A_res = 0.0f, expected_B_res = 0.0f;
      float residual_A = 0.0f, residual_B = 0.0f;
      float q_joint = dof_data.angles[dof];
      if (cached_motor_angles.valid[dof] &&
          calculateMotorAnglesWithEquations(dof, q_joint, q_joint, expected_A_res, expected_B_res)) {
        residual_A = cached_motor_angles.agonist[dof] - expected_A_res;
        residual_B = cached_motor_angles.antagonist[dof] - expected_B_res;
      }

      // Iq data (use trResp if valid, flag for offline parsing)
      bool iq_valid = trResp.dataA.valid && trResp.dataB.valid;
      int16_t iq_A_diag = iq_valid ? trResp.dataA.torqueCurrent : 0;
      int16_t iq_B_diag = iq_valid ? trResp.dataB.torqueCurrent : 0;
      int16_t iq_abs_max = max(abs(iq_A_diag), abs(iq_B_diag));
      float iq_ratio = iq_valid && iq_abs_max > 0
          ? (float)min(abs(iq_A_diag), abs(iq_B_diag)) / (float)iq_abs_max
          : -1.0f;  // sentinel: invalid

      // === PROPOSED TRIM DRY-RUN ===
      // Computes what a tension trim would do, but does NOT apply it.
      // Keep this conservative and slow even if DIAG_HOLD telemetry is emitted
      // immediately and frequently.
      bool trim_update_due =
          holding_ema_samples[dof] >= trim_dry_run_min_samples &&
          t_now - last_trim_dry_run_update[dof] > trim_dry_run_period_ms;

      if (trim_update_due && dof < MAX_DOFS && iq_valid && iq_abs_max > 30) {
        bool slack_detected = (iq_ratio >= 0 && iq_ratio < TRIM_SLACK_RATIO_TH);
        bool bias_present = (fabs(holding_dtheta_ema[dof]) > TRIM_BIAS_TH);
        bool balanced = (iq_ratio >= TRIM_BALANCED_RATIO_TH);

        // Residual concordance: do both residuals have same sign as the slack side?
        // For now just flag it, don't use for direction.
        // bool residual_concordant = ...;  // Phase 2 refinement

        if (slack_detected && bias_present) {
          // Slack side from torque ratio: low Iq side needs more preload
          float slack_sign = (abs(iq_A_diag) < abs(iq_B_diag)) ? 1.0f : -1.0f;
          proposed_trim_deg[dof] += slack_sign * TRIM_STEP_DEG;
          proposed_trim_deg[dof] = constrain(proposed_trim_deg[dof], -TRIM_MAX_DEG, TRIM_MAX_DEG);
        } else if (balanced) {
          // Tendons balanced — slow decay toward zero
          proposed_trim_deg[dof] *= TRIM_DECAY;
          if (fabs(proposed_trim_deg[dof]) < 0.01f) proposed_trim_deg[dof] = 0;
        }
        // else: gates valid but ambiguous — freeze trim (no update)
        last_trim_dry_run_update[dof] = t_now;
      }
      // Note: if gates fall, the DIAG_HOLD block is not entered at all,
      // so proposed_trim_deg is implicitly frozen.

      LOG_C1_INFO("[DIAG_HOLD] DOF" + String(dof) +
                  " q=" + String(q_joint, 1) +
                  " ema=" + String(holding_dtheta_ema[dof], 2) +
                  " resA=" + String(residual_A, 2) +
                  " resB=" + String(residual_B, 2) +
                  " iqA=" + String(iq_A_diag) +
                  " iqB=" + String(iq_B_diag) +
                  " iqR=" + String(iq_ratio, 2) +
                  " iqV=" + String(iq_valid ? 1 : 0) +
                  " stiffBase=" + String(stiffness_ref, 1) +
                  " stiffEff=" + String(stiffness_ref_effective, 1) +
                  " rt=" + String(last_retension_boost_dbg[dof], 2) +
                  " trim=" + String(proposed_trim_deg[dof], 3) +
                  " n=" + String(holding_ema_samples[dof]));

      // Populate shared struct for CAN streaming to host UI.
      // Sequence counter protocol: Core0 increments seq to odd before writing,
      // then to even after. Core1 retries if seq is odd or changed during read.
      if (dof < MAX_DOFS) {
        __atomic_add_fetch(&diag_hold_data[dof].seq, 1, __ATOMIC_RELEASE);  // odd = writing
        diag_hold_data[dof].dof = dof;
        diag_hold_data[dof].ema_x100 = (int16_t)(holding_dtheta_ema[dof] * 100.0f);
        diag_hold_data[dof].residual_A_x100 = (int16_t)(residual_A * 100.0f);
        diag_hold_data[dof].residual_B_x100 = (int16_t)(residual_B * 100.0f);
        diag_hold_data[dof].iq_A = iq_A_diag;
        diag_hold_data[dof].iq_B = iq_B_diag;
        diag_hold_data[dof].stiffness_x10 = (int16_t)(stiffness_ref_effective * 10.0f);
        diag_hold_data[dof].tension_trim_x100 = (int16_t)(proposed_trim_deg[dof] * 100.0f);
        uint8_t diag_flags = 0;
        if (iq_valid) diag_flags |= 0x01;
        if (holding_ema_samples[dof] >= diag_hold_ema_settled_samples) diag_flags |= 0x02;
        diag_hold_data[dof].flags = diag_flags;
        __atomic_add_fetch(&diag_hold_data[dof].seq, 1, __ATOMIC_RELEASE);  // even = done
      }

      last_holding_bias_log[dof] = t_now;
    }

#if CONTROLLER_DEBUG
    {
      uint32_t torque_dt = time_us_32() - torque_start_us;
      loop_micro_profile.accum_torque_us += torque_dt;
      if (torque_dt > loop_micro_profile.max_torque_us) {
        loop_micro_profile.max_torque_us = torque_dt;
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
      loop_micro_profile.accum_dof_us += dof_dt;
      if (dof_dt > loop_micro_profile.max_dof_us) {
        loop_micro_profile.max_dof_us = dof_dt;
      }
      loop_micro_profile.samples++;
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

    if (cycle_time_us_last > inner_loop_period_us) {
      diag_note_loop_overrun();
    }

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
  if (loop_micro_profile.samples >= LOOP_MICRO_MIN_SAMPLES) {
    uint32_t now_ms = millis();
    if (loop_micro_profile.last_log_ms == 0) {
      loop_micro_profile.last_log_ms = now_ms;
    }
    if (now_ms - loop_micro_profile.last_log_ms >= LOOP_MICRO_LOG_INTERVAL_MS) {
      // Pre-compute averages as integers (avoids snprintf %f which uses ~1KB stack on ARM)
      uint32_t n = loop_micro_profile.samples;
      uint32_t sn = loop_micro_profile.safety_samples;
      uint32_t avg_dof = loop_micro_profile.accum_dof_us / n;
      uint32_t avg_outer = loop_micro_profile.accum_outer_us / n;
      uint32_t avg_eq = loop_micro_profile.accum_eq_us / n;
      uint32_t avg_can = loop_micro_profile.accum_can_us / n;
      uint32_t avg_pid = loop_micro_profile.accum_pid_us / n;
      uint32_t avg_torque = loop_micro_profile.accum_torque_us / n;
      uint32_t avg_safety = (sn > 0) ? loop_micro_profile.accum_safety_us / sn : 0;
      // Split into 2 messages to keep buffer small
      LOG_C1_INFO_F("[WP PROF] avg_us dof=%lu outer=%lu eq=%lu can=%lu pid=%lu torq=%lu safety=%lu",
                 avg_dof, avg_outer, avg_eq, avg_can, avg_pid, avg_torque, avg_safety);
      LOG_C1_INFO_F("[WP PROF] max_us dof=%lu outer=%lu eq=%lu can=%lu pid=%lu torq=%lu safety=%lu",
                 (unsigned long)loop_micro_profile.max_dof_us,
                 (unsigned long)loop_micro_profile.max_outer_us,
                 (unsigned long)loop_micro_profile.max_eq_us,
                 (unsigned long)loop_micro_profile.max_can_us,
                 (unsigned long)loop_micro_profile.max_pid_us,
                 (unsigned long)loop_micro_profile.max_torque_us,
                 (unsigned long)loop_micro_profile.max_safety_us);
      loop_micro_profile.accum_dof_us = 0;
      loop_micro_profile.accum_outer_us = 0;
      loop_micro_profile.accum_eq_us = 0;
      loop_micro_profile.accum_can_us = 0;
      loop_micro_profile.accum_pid_us = 0;
      loop_micro_profile.accum_torque_us = 0;
      loop_micro_profile.accum_safety_us = 0;
      loop_micro_profile.max_dof_us = 0;
      loop_micro_profile.max_outer_us = 0;
      loop_micro_profile.max_eq_us = 0;
      loop_micro_profile.max_can_us = 0;
      loop_micro_profile.max_pid_us = 0;
      loop_micro_profile.max_torque_us = 0;
      loop_micro_profile.max_safety_us = 0;
      loop_micro_profile.samples = 0;
      loop_micro_profile.safety_samples = 0;
      loop_micro_profile.last_log_ms = now_ms;
    }
  }
#endif
  
  return any_movement;
}
