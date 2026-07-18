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
 *   * Inner loop period = 4000 µs (4 ms) → 250 Hz (default; inner_loop_period_us)
 *   * Outer PID runs every outer_loop_divisor cycles (default 1 = 250 Hz)
 *   * Inner PID @ 250 Hz (motor-level, computes torque commands)
 *   * theta_ref = theta_0 + cascade_correction
 *
 * @see CAN_SYSTEM_ARCHITECTURE.md for protocol specification
 */

#include "JointController.h"
#include <Arduino.h>
#include <debug.h>
#include "main_common.h"  // For shared_dof_angles, DofState
#include "safety_system.h"  // For safety_motor_power_disable() — the free-capture EXIT cuts motor power
#include <math.h>  // For cosf, M_PI
#include <hot_path.h>

// External time sync function (defined in core1.cpp)
extern uint32_t getAbsoluteTimeMs();

// Cycle counter for outer/inner loop division (wraps at 1000 to prevent overflow)
static uint16_t cycle_count = 0;

// Wrap-safe outer-loop phase (outer-divisor fix, 2026-07-08): 0 = outer-due cycle.
// Deriving the phase from cycle_count (wraps %1000) had two divisor>1 artifacts:
// a stretched outer period at every wrap for divisors not dividing 1000, and
// (cycle_count - 1) promoting to int -1 at the wrap cycle, which made the
// delta_theta interpolation alpha = 0 (a one-inner-cycle revert kick at ~0.6 Hz).
// A dedicated phase counter has neither. Divisor 1 behavior is identical.
static uint16_t outer_phase_counter = 0;
static uint8_t outer_phase_divisor = 1;

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

// Torque rate-limiter history (previous saturated+rate-limited command, per DOF). File-scope so the
// IDLE early-exit and the e-stop/fault reset helper can zero it: the limiter must NEVER ramp from a
// stale pre-fault command — constrain(0, prev±rate) would re-inflate a designed zero back toward the
// previous torque (the zero_fire "torque replay": after a fault latched at high torque + PRETENSION
// recovery, the bootstrap would fire the stale command open-loop against un-anchored motors).
static float prev_command_A[MAX_DOFS] = {0};
static float prev_command_B[MAX_DOFS] = {0};
static float prev_command_direct[MAX_DOFS] = {0};

// === GMS break-away bristle friction feedforward (replaces the old constant friction-FF) ===
// Why: the old constant FF keyed off the COMMANDED velocity (which holds at cruise right through a stall),
// so it was a DC bias the inner PID absorbed and it never broke the DOF1 stick-slip. This single-state
// bristle winds a presliding deflection z while the joint is STALLED (measured |v|~0 but commanded vc!=0),
// rising toward the break-away level Fs, then collapses when the joint slips (measured |v| jumps). That
// climbing-then-collapsing torque is exactly what the feedback delta_theta pumps today, so the FF supplies
// it and the feedback stops winding up. Validated offline against logged (v,vc): corr(tau_fric,dth)=+0.68.
// Knob reuse: friction_ff_torque is now Fs (the break-away level), tuned live via CAN 0x015 / --fric-torque.
static float gms_z[MAX_DOFS] = {0};                 // GMS presliding bristle deflection (deg), per DOF
static uint32_t gms_prev_cycle_us = 0;              // prev inner-cycle timestamp (us) for the measured-Ts bristle dt
static float gms_dt_s = 0.004f;                     // measured inner-loop dt (s) for the bristle (v2 timing fix)
static constexpr float GMS_Z_MAX    = 0.5f;         // presliding deflection ceiling (deg); sigma0 = Fs/z_max
static constexpr float GMS_VC_MIN   = 1.0f;         // deg/s; below this there is no commanded motion -> leak z->0
static constexpr float GMS_TAU_MULT = 1.4f;         // hard clamp FF_TAU_MAX = GMS_TAU_MULT * Fs (eyelet guard)

// One inner-cycle update of the bristle; returns the (clamped) feedforward torque. v = MEASURED joint
// velocity, vc = COMMANDED joint velocity, Fs = break-away level (friction_ff_torque), sigma1 = over-slip damping (friction_ff_speed_thresh), Ts = inner-loop dt (s).
static float HOT_FUNC(updateGmsFriction)(uint8_t dof, float v, float vc, float Fs, float sigma1, float Ts) {
  if (dof >= MAX_DOFS) return 0.0f;
  if (Fs < 1e-3f) { gms_z[dof] = 0.0f; return 0.0f; }  // knob off -> cold-start (no stale snap-back on re-enable)
  Fs = fminf(Fs, 100.0f);                              // bound tau_max=1.4*Fs at point of use (defense-in-depth)
  float sigma0 = Fs / GMS_Z_MAX;                    // presliding stiffness (torque/deg)
  float &z = gms_z[dof];
  if (fabsf(vc) < GMS_VC_MIN) {
    z *= 0.9f;                                      // arrived/holding: leak the bristle out (no hold bias)
  } else {
    float veff = fmaxf(fabsf(v), fabsf(vc));        // veff>0 keeps the denominator defined; |v| collapses z on a slip
    z += Ts * (vc - sigma0 * z * veff / Fs);        // wind during stick (v~0), collapse on slip (|v| jumps)
    z = constrain(z, -GMS_Z_MAX, GMS_Z_MAX);        // hard z saturation -> |sigma0*z| <= Fs
  }
  // v3.1 over-slip DAMPING: sigma1*(vc - v) opposes an OVER-SLIP, GATED to fire ONLY when the joint moves
  // faster than commanded in the command direction ((v-vc)*vc>0). The ungated v3 also ASSISTED the stick
  // (v<vc -> +sigma1*vc forward), which over-drove the break-away and OVERSHOT the target (e-stop at +13deg
  // on a +10 command). Gating to over-slip-only = pure damping: the bristle handles break-away, this only
  // damps the over-shoot that sustains the limit cycle; it is 0 during the stick and at steady tracking.
  float damp = (fabsf(vc) >= GMS_VC_MIN && (v - vc) * vc > 0.0f) ? sigma1 * (vc - v) : 0.0f;
  float tau_max = GMS_TAU_MULT * fmaxf(Fs, 40.0f);  // ceiling independent of a tiny Fs (so pure-damping is evaluable)
  return constrain(sigma0 * z + damp, -tau_max, tau_max);  // final clamp (eyelet guard before it reaches uff)
}

// Per-direction break-away level. The stiction is tension/geometry dependent and differs by
// travel direction (the 07-02 knee 2x2 showed the symmetric FF SHIFTS the UP/DOWN asymmetry
// instead of removing it - per-direction Fs is the evidence-backed lever). Selected by the
// COMMANDED direction sign; friction_ff_torque_neg = 0 keeps the legacy symmetric behavior.
static inline float HOT_FUNC(gmsFsForDirection)(float vc) {
  return (friction_ff_torque_neg > 1e-3f && vc < 0.0f) ? friction_ff_torque_neg
                                                       : friction_ff_torque;
}
static uint32_t last_anti_slack_log_ms[MAX_DOFS] = {0};
static float direct_drive_last_angle[MAX_DOFS] = {0};
static uint32_t direct_drive_last_update_us[MAX_DOFS] = {0};
static uint32_t direct_drive_next_probe_ms[MAX_DOFS] = {0};
static uint8_t direct_drive_invalid_streak[MAX_DOFS] = {0};
static bool direct_drive_feedback_fault_active[MAX_DOFS] = {false};
static uint32_t direct_drive_feedback_fault_log_ms[MAX_DOFS] = {0};
static uint32_t direct_drive_feedback_zero_torque_ms[MAX_DOFS] = {0};
static const uint16_t DIRECT_DRIVE_TIMEOUT_BACKOFF_MS = 20;
static const uint16_t DIRECT_DRIVE_UNREFERENCED_PROBE_MS = 250;
static const uint16_t DIRECT_DRIVE_IDLE_PROBE_MS = 20;
static const uint8_t DIRECT_DRIVE_INVALID_STREAK_LIMIT = 3;
// Tendon DOFs: on invalid encoder, cut motor torque after a short run of bad cycles so
// the joint is never driven open-loop on a frozen command (slam-to-stop prevention).
static uint16_t tendon_encoder_fault_streak[MAX_DOFS] = {0};
static uint32_t tendon_encoder_fault_log_ms[MAX_DOFS] = {0};
static const uint8_t TENDON_ENCODER_FAULT_CUT_CYCLES = 3; // ~12 ms @ 250 Hz inner loop

// Last VALID actual joint angle (deg) and a "have we ever seen a valid read" flag,
// refreshed every inner cycle while the encoder is valid. Used by the host-watchdog
// freeze (ITEM 4) so that, when the current read is momentarily invalid, the local hold
// latches at the last real joint position instead of the segment reference (which leads
// the joint mid-move). No effect in nominal operation (the live valid read is used).
static float    last_valid_joint_angle[MAX_DOFS] = {0.0f};
static bool     has_last_valid_joint_angle[MAX_DOFS] = {false};

// Persistent "possible tendon breakage" (MOTOR_RANGE) escalation.
// A single transient MOTOR_RANGE read still soft-stops (HOLDING) as before — only a
// SUSTAINED condition escalates to the latched hardware power-cut (same path as the
// 0x000 host e-stop), which needs a physical PRETENSION recovery. We require this many
// CONSECUTIVE MOTOR_RANGE trips before escalating. The MOTOR_RANGE / tendon check only
// runs in HOLDING at safety_check_counter>=10 (every 10 inner cycles ≈ 40 ms at the
// 250 Hz / 4 ms default inner loop), so 3 consecutive trips ≈ 120 ms of sustained
// out-of-range — long enough to rule out a one-cycle glitch, short enough to cut before
// a broken tendon drives the joint far.
static uint8_t motor_range_trip_streak[MAX_DOFS] = {0};
static const uint8_t MOTOR_RANGE_ESCALATE_CYCLES = 3;

// === SLACK MONITOR: delta_theta bias tracking during HOLDING ===
// Exponential moving average of delta_theta while in HOLDING state.
// A persistent non-zero bias indicates the outer PID is doing static compensation
// work. This is a joint-space symptom — it does NOT directly identify which motor
// offset is wrong (see SLACK_DETECTION_AND_TENSION_TRIM.md for rationale).
// Used as a diagnostic/gating signal, not as a direct correction source.
static float holding_dtheta_ema[MAX_DOFS] = {0};     // EMA of delta_theta during HOLDING
static uint32_t holding_ema_samples[MAX_DOFS] = {0}; // samples accumulated (for warm-up)
static uint32_t last_holding_bias_log[MAX_DOFS] = {0}; // rate-limit log
static uint32_t last_motion_diag_log[MAX_DOFS] = {0};  // rate-limit [DIAG_MOVE] motion telemetry (TELEMETRY ONLY)
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

// [Metrics] deferred-emission ring (dev pass 2026-07-07): finalize pre-formats its 6 lines
// here; the loop tail drains ONE line per cycle via core1LogPush, keeping the burst out of
// the WP-PROF dof spans. 12 slots = two DOFs finalizing on the same stroke. core1-only state.
static const int METRICS_LINE_LEN = 120;
static const int METRICS_RING_SLOTS = 6 * MAX_DOFS;  // every DOF can finalize on one stroke (hip = 3)
static char metrics_pending_ring[METRICS_RING_SLOTS][METRICS_LINE_LEN];
static uint8_t metrics_pending_head = 0;   // next slot to drain
static uint8_t metrics_pending_count = 0;  // pending lines
static char *HOT_FUNC(metricsPendingSlot)() {
  if (metrics_pending_count >= METRICS_RING_SLOTS) return nullptr;  // full: drop (diag-only)
  uint8_t idx = (uint8_t)((metrics_pending_head + metrics_pending_count) % METRICS_RING_SLOTS);
  metrics_pending_count++;
  metrics_pending_ring[idx][0] = '\0';
  return metrics_pending_ring[idx];
}
static inline void metricsDrainOnePending() {
  if (metrics_pending_count == 0) return;
  core1LogPush(C1_LOG_LEVEL_INFO, metrics_pending_ring[metrics_pending_head]);
  metrics_pending_head = (uint8_t)((metrics_pending_head + 1) % METRICS_RING_SLOTS);
  metrics_pending_count--;
}
// E-stop/loop-stop flush: without this, lines still in the 0-11-cycle drain window at an
// emergency stop would be lost — and those are exactly the last-stroke forensics.
void jcMetricsFlushPendingLogs() {
  while (metrics_pending_count > 0) metricsDrainOnePending();
}

static void HOT_FUNC(finalizeMovementMetricsForDof)(uint8_t dof) {
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

  // Deferred emission (dev pass 2026-07-07): the 6-line burst at stroke reversal was the
  // measured outer-span spike (bench: outer max 548-650 in 13/13 burst windows even with the
  // heap-free _F build — the newlib flash execution per line). Now: pre-format into a pending
  // ring here (cheap with the RAM formatters), DRAIN ONE LINE PER CYCLE from the loop tail
  // (outside every WP-PROF dof span). Line text unchanged; only the emission cycle shifts by
  // 0-11 cycles (~0-22ms at 500Hz) — nothing parses Metrics timing.
  if (LOG_LEVEL >= 2) {
    char f1[48], f2[48], f3[48];
    char *slot;
    int off;
    #define _MT_SLOT() (slot = metricsPendingSlot(), off = 0, slot)
    if (_MT_SLOT()) c1cat(slot, METRICS_LINE_LEN, &off, "[Metrics] DOF %d FINAL:", dof);
    if (_MT_SLOT()) c1cat(slot, METRICS_LINE_LEN, &off, "  start=%s° → target=%s° (orig=%s°)",
                          c1f(f1, mt.start_angle_deg, 2), c1f(f2, metrics_target, 2),
                          c1f(f3, original_target, 2));
    if (_MT_SLOT()) c1cat(slot, METRICS_LINE_LEN, &off, "  rise=%ums (90%%=%s)",
                          (unsigned)m.rise_time_ms, mt.reached_90_percent ? "yes" : "no");
    if (_MT_SLOT()) c1cat(slot, METRICS_LINE_LEN, &off, "  settle=%ums, max_err=%s°",
                          (unsigned)m.settling_time_ms, c1f(f1, mt.max_error_deg, 2));
    if (_MT_SLOT()) c1cat(slot, METRICS_LINE_LEN, &off, "  overshoot=%s%% (max=%s°, dir=%s)",
                          c1f(f1, m.overshoot_x100 / 100.0f, 1), c1f(f2, mt.max_overshoot_deg, 2),
                          c1f(f3, mt.movement_direction, 0));
    if (_MT_SLOT()) c1cat(slot, METRICS_LINE_LEN, &off, "  sse=%s° (samples=%u)",
                          c1f(f1, m.sse_x100 / 100.0f, 2), (unsigned)mt.sse_sample_count);
    #undef _MT_SLOT
  }
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
static void resetMotorFeedbackSnapshot(uint8_t dof);

static float loop2_target_A[MAX_DOFS] = {};
static float loop2_target_B[MAX_DOFS] = {};
static bool loop2_target_valid[MAX_DOFS] = {};
static uint32_t loop2_last_log_ms[MAX_DOFS] = {};
// Streak gate for the Loop2 motor-feedback terminal faults (NaN/range/jump/reply-miss):
// a SINGLE transient used to latch an irreversible power-cut (hair-trigger); now it takes
// LOOP2_FEEDBACK_FAULT_STREAK_CYCLES consecutive bad cycles (~12 ms @250 Hz, mirroring the
// ITEM-1 tendon-encoder streak). During the streak the cycle is skipped and the motor holds
// its last VALID latched 0xA4 target - benign for that window. Reset on a clean collect.
static uint8_t loop2_feedback_fault_streak[MAX_DOFS] = {};
static constexpr uint8_t LOOP2_FEEDBACK_FAULT_STREAK_CYCLES = 3;
static uint32_t loop2_stiffness_floor_log_ms[MAX_DOFS] = {};
static uint32_t loop2_stiffness_ceiling_log_ms[MAX_DOFS] = {};
static uint32_t loop2_spread_clamp_log_ms[MAX_DOFS] = {};
static constexpr float LOOP2_MIN_STIFFNESS_REF_DEG = 5.0f;
static constexpr float LOOP2_MAX_STIFFNESS_REF_DEG = PID_DEFAULT_STIFFNESS_DEG;

static float slewLoop2Target(float current, float target, float step_deg) {
  const float err = target - current;
  if (fabsf(err) <= step_deg) return target;
  return current + ((err > 0.0f) ? step_deg : -step_deg);
}

void resetLoop2ActuationState(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  loop2_target_A[dof] = 0.0f;
  loop2_target_B[dof] = 0.0f;
  loop2_target_valid[dof] = false;
  loop2_last_log_ms[dof] = 0;
  loop2_feedback_fault_streak[dof] = 0;
}

void forceLoop1ActuationMode(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  inner_actuation_mode[dof] = INNER_ACTUATION_LOOP1_TORQUE;
  resetLoop2ActuationState(dof);
  inner_pid_reinit_after_impedance[dof] = true;
}

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

// Reset per-DOF safety fault streak counters. The counters are file-scope statics in
// this translation unit, so the core1 E-Stop path reaches them through this helper.
// Mirrors the IDLE early-exit reset: a fault streak must never carry across an e-stop
// into the next session (otherwise a single fresh trip could latch power / cut torque).
void resetControlLoopFaultStreaks(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  motor_range_trip_streak[dof] = 0;
  tendon_encoder_fault_streak[dof] = 0;
  resetMotorFeedbackSnapshot(dof);
  resetLoop2ActuationState(dof);
  prev_command_A[dof] = 0.0f;      // torque rate-limiter history: a post-e-stop ramp restarts
  prev_command_B[dof] = 0.0f;      // from zero, never from the stale pre-fault command
  prev_command_direct[dof] = 0.0f;
  gms_z[dof] = 0.0f;               // GMS bristle cold-start (same-iteration SET_IMPEDANCE can
                                   // bypass the IDLE cleanup that normally zeroes it)
}

static void latchTerminalMotionFault(uint8_t source_dof, const char *reason, JointController *jc) {
  if (jc != nullptr) {
    jc->stopAllMotors();
  }
  safety_motor_power_disable();
  emergency_stop_requested = true;  // next core1 iteration latches E-stop and performs full cleanup

  for (uint8_t d = 0; d < MAX_DOFS; ++d) {
    impedance_target[d].watchdog_timed_out = false;
    impedance_target[d].valid = false;
    resetImpedanceSegment(d);
    forceLoop1ActuationMode(d);
    dof_state[d] = DofState::IDLE;
  }

  LOG_C1_ERROR_F("[Safety] TERMINAL MOTION LOCKOUT: DOF %d %s — motor power cut; recovery required",
                 (int)source_dof, reason);
}

static void clearImpedanceControlState(uint8_t dof, JointController *jc) {
  restoreInnerPidGains(dof, jc);
  restoreOuterLoopParameters(dof, jc);
  resetImpedanceSegment(dof);
  forceLoop1ActuationMode(dof);
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

static void HOT_FUNC(applyImpedanceOuterOverrides)(uint8_t dof, JointController *jc) {
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

static void HOT_FUNC(applyImpedanceInnerOverrides)(uint8_t dof, PID *pid_agonist, PID *pid_antagonist) {
  if (!pid_agonist || !pid_antagonist || !impedance_target[dof].valid) {
    return;
  }

  if (!inner_pid_backup[dof][0].saved) {
    inner_pid_backup[dof][0] = {pid_agonist->getKp(), pid_agonist->getKi(), pid_agonist->getKd(), true};
    inner_pid_backup[dof][1] = {pid_antagonist->getKp(), pid_antagonist->getKi(), pid_antagonist->getKd(), true};
    // Heap-free build (2026-07-06): one-shot per session but sits inside the pid WP-PROF
    // span (bench: the only pid-max 584/712us excursions were exactly these emissions).
    if (LOG_LEVEL >= 2) {
      char f1[48], f2[48], f3[48];
      LOG_C1_INFO_F("[IMPEDANCE] DOF%d inner PID backed up: Kp=%s Ki=%s Kd=%s", dof,
                    c1f(f1, inner_pid_backup[dof][0].kp, 3),
                    c1f(f2, inner_pid_backup[dof][0].ki, 3),
                    c1f(f3, inner_pid_backup[dof][0].kd, 3));
    }
  }

  const float kp_inner = impedance_target[dof].kp_inner;
  const float ki_inner = impedance_target[dof].ki_inner;
  const float kd_inner = fmaxf(impedance_target[dof].kd_inner, PID_MIN_TENDON_INNER_KD);
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

static void HOT_FUNC(applyImpedanceInnerOverrideSingle)(uint8_t dof, PID *pid_direct) {
  if (!pid_direct || !impedance_target[dof].valid) {
    return;
  }

  if (!inner_pid_backup[dof][0].saved) {
    inner_pid_backup[dof][0] = {pid_direct->getKp(), pid_direct->getKi(), pid_direct->getKd(), true};
    inner_pid_backup[dof][1] = {};
    // Heap-free build (2026-07-06): mirror of the tendon backup above (pid WP-PROF span).
    if (LOG_LEVEL >= 2) {
      char f1[48], f2[48], f3[48];
      LOG_C1_INFO_F("[IMPEDANCE] DOF%d direct-drive PID backed up: Kp=%s Ki=%s Kd=%s", dof,
                    c1f(f1, inner_pid_backup[dof][0].kp, 3),
                    c1f(f2, inner_pid_backup[dof][0].ki, 3),
                    c1f(f3, inner_pid_backup[dof][0].kd, 3));
    }
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
static const float LOOP2_OSC_MIN_AMPLITUDE_DEG = 6.0f; // Loop2 goes TERMINAL on oscillation -> higher bar (2026-07-03)
static const float OSC_MIN_ERROR_TO_CHECK = 1.0f;    // Don't check if error is very small

// === PASSIVE SAFETY GOVERNOR: command corridor + divergence soft-stop ===
// The hard MAPPING_LIMIT/MOTOR_RANGE checks are terminal backstops. These guards act earlier:
// keep the tendon motor references inside a soft map corridor and stop a DOF that is moving
// quickly in the direction that increases position error.
static constexpr float MAP_CORRIDOR_SOFT_MARGIN_DEG = 2.0f;   // inset from motor safe range
static constexpr float MAP_CORRIDOR_NEAR_MARGIN_DEG = 0.75f;  // log/track near-corridor dwell
static constexpr float MAP_CORRIDOR_MIN_SPAN_DEG = 1.5f;      // avoid degenerate soft corridors
static constexpr float MAP_CORRIDOR_DTH_ABS_LIMIT_DEG = 30.0f;
static constexpr uint32_t MAP_CORRIDOR_LOG_PERIOD_MS = 250;
static constexpr float MAP_CORRIDOR_MIN_CASCADE_INFLUENCE = 0.001f;
static constexpr float MAP_CORRIDOR_MIN_DERATED_STIFFNESS_DEG = 5.0f;
static uint8_t map_corridor_dwell[MAX_DOFS] = {0};
static uint32_t map_corridor_last_log_ms[MAX_DOFS] = {0};
static uint32_t map_corridor_last_skip_log_ms[MAX_DOFS] = {0};
static uint16_t map_corridor_derate_count[MAX_DOFS] = {0};
static float map_corridor_last_derate_scale[MAX_DOFS] = {1.0f, 1.0f, 1.0f};

static constexpr float DIVERGENCE_MIN_ERROR_DEG = 2.0f;
static constexpr float DIVERGENCE_AWAY_SPEED_DEG_S = 120.0f;
static constexpr uint8_t DIVERGENCE_STOP_CYCLES = 2;
static uint8_t divergence_guard_streak[MAX_DOFS] = {0};

// === MOTION GUARD V2 (MG2-S) — Stage 1+2 + review batch (2026-07-18) ===
// Design (source of truth): job memory/MOTION_GUARDS_V2_DESIGN_2026-07-18.md.
// Publication tier: PUBLIC-CORE (D068 s12). Modes: 0 LEGACY (verbatim blocks below run,
// behaviorally identical), 1 SHADOW (v2 computes + logs would-actions, legacy
// authoritative, state-faithful to ACTIVE), 2 ACTIVE (v2 replaces the legacy guards).
// Boot default 0; e-stop forces 0; mode changes rejected mid-move and while faulted.
// DEVIATIONS from the design doc, deferred to the bench-return iteration (deliberate,
// reviewed): L1b stiffness-derate rung; R1 effort-derate + |dth|>=24 effort term (the
// corridor-clamp freshness flag + x3 integration stand in); v_band/M_res envelope and
// EVT:DIV_GROWTH / ENV_BREACH telemetry; runtime tunable knobs + 0x530 echo beyond the
// mode byte; host-side EVT:DIV_SOFT reaction (mid-stream a soft-hold holds only until the
// next SET_IMPEDANCE unless the host reacts). COMPENSATING additions from review: ERR_CEIL
// 18deg/40ms blocked-foot rung; x3 effort-clamped i_stall gain (hard block aborts ~300ms,
// legacy parity); speed-scaled direction-agnostic progress threshold (creep coverage);
// post-soft brake-grace discriminator; independent L3d arm; gap-reset state hygiene.
#ifndef MOTION_GUARD_V2
#define MOTION_GUARD_V2 1
#endif
#if MOTION_GUARD_V2
static constexpr float MG2_QS_VREF_MAX_DPS = 40.0f;   // at/below: QS backstop (legacy parity —
                                                      // away>=120 with cmd<=40 GUARANTEES error
                                                      // growth >=80dps = confirmed runaway)
static constexpr float MG2_GATE_FLOOR_DPS  = 120.0f;  // arm gate floor (the ratified legacy gate)
static constexpr float MG2_GATE_K          = 3.0f;    // TRK arm gate = max(floor, K*v_ref)
static constexpr float MG2_TAU_GATE_MS     = 300.0f;  // v_ref decay time constant
static constexpr float MG2_VREF_FLOOR_DPS  = 10.0f;
static constexpr float MG2_MIN_ERR_DEG     = 2.0f;    // same as legacy DIVERGENCE_MIN_ERROR_DEG
static constexpr float MG2_QS_TRIP_MS      = 8.0f;    // legacy 2-cycle@250Hz parity, wall-time
                                                      // (fixes the x-divisor scaling, 07-06 log)
static constexpr float MG2_SOFT_TRIP_MS    = 40.0f;   // TRK L2 soft-hold
static constexpr float MG2_L3A_AWAY_DPS    = 300.0f;  // TRK direct-terminal: fast runaway
static constexpr float MG2_L3A_ERR_DEG     = 5.0f;
static constexpr float MG2_L3A_TRIP_MS     = 16.0f;
static constexpr uint32_t MG2_L3B_REARM_MS = 250;     // re-divergence after soft = confirmed
static constexpr uint8_t  MG2_L3C_SOFT_MAX = 3;       // 3 softs in the window = confirmed
static constexpr uint32_t MG2_L3C_WINDOW_MS= 2000;
static constexpr float MG2_LIMIT_DIST_DEG  = 3.0f;    // L3d limit-approach fastpath
static constexpr float MG2_LIMIT_VEL_DPS   = 60.0f;
static constexpr float MG2_LIMIT_TRIP_MS   = 20.0f;   // L3d own wall-time arm (independent)
static constexpr uint32_t MG2_EVT_PERIOD_MS= 250;     // shadow/EVT rate limit
static constexpr uint32_t MG2_GAP_TICK_MS  = 20;      // dt above this = missed cycles: credit 4ms
static constexpr uint32_t MG2_VREF_GAP_RESET_MS = 300;// hold gap beyond tau: v_ref fully decayed
static constexpr uint32_t MG2_SOFT_GRACE_MS = 120;    // post-soft brake-out window (motor pole
                                                      // ~25ms: a braking plant sheds away-speed)
static constexpr float MG2_GRACE_AWAY_MARGIN_DPS = 30.0f;
// Stall side: windowed toward-goal PROGRESS replaces the instantaneous velocity-ratio
// (which trips on slow spin-up against co-contraction — the step-asymmetry FP, session
// 10b). Ladder: ride warn -> confirmed abort at 900ms integrated no-progress (x3 under
// corridor-clamped effort -> ~300ms on a hard block), ERR_CEIL backstop at 18deg/40ms.
static constexpr uint32_t MG2_STALL_WINDOW_MS   = 300;   // progress measurement window
static constexpr float MG2_STALL_PROGRESS_MIN_DEG = 0.5f; // absolute displacement floor
static constexpr float MG2_STALL_PROG_FRAC      = 0.20f; // + fraction of commanded travel/window
                                                         // (0.20 = exact legacy velocity-ratio
                                                         // equivalence: catches the same creep
                                                         // band, e.g. <2dps at v_exp=10)
static constexpr float MG2_STALL_ERR_FLOOR_DEG  = 4.0f;  // err floor to arm (normal branch)
static constexpr float MG2_STALL_EFFORT_ERR_DEG = 2.0f;  // lower floor when effort-clamped
static constexpr float MG2_EFFORT_ISTALL_GAIN   = 3.0f;  // corridor-railed no-progress: x3
static constexpr float MG2_STALL_WARN_MS        = 400.0f;
static constexpr float MG2_STALL_ABORT_MS       = 900.0f;
static constexpr float MG2_ERR_CEIL_DEG         = 18.0f; // blocked-foot backstop rung
static constexpr float MG2_ERR_CEIL_TRIP_MS     = 40.0f;
static constexpr uint32_t MG2_STALL_GAP_RESET_MS = 50;   // segment boundary: virgin restart
static constexpr uint32_t MG2_STALL_EFFORT_FRESH_MS = 50; // corridor-clamp flag freshness
struct Mg2State {
  float v_ref = 0.0f;            // commanded-speed envelope (decay tau 300ms, floor 10)
  float err_abs_prev = 0.0f;
  float streak_ms = 0.0f;        // leaky wall-time arm streak
  uint8_t streak_samples = 0;
  float limit_streak_ms = 0.0f;  // L3d independent arm streak
  uint8_t limit_streak_samples = 0;
  uint32_t last_ms = 0;
  uint32_t last_soft_ms = 0;
  float away_at_soft = 0.0f;     // brake-grace discriminator reference
  uint32_t soft_window_start_ms = 0;
  uint8_t soft_count = 0;
  uint32_t last_evt_ms = 0;
  float err_hist[6] = {0};       // |err| ring for the growth test (~100ms lookback)
  uint32_t err_hist_ms[6] = {0};
  uint8_t hist_idx = 0;
  // stall side
  float stall_anchor_q = 0.0f;   // progress window anchor
  uint32_t stall_anchor_ms = 0;
  bool stall_no_progress = false;
  uint32_t stall_last_ms = 0;    // own tick (the divergence block runs later in the cycle)
  float i_stall_ms = 0.0f;       // leaky no-progress integrator (leak 2x on progress)
  float err_ceil_ms = 0.0f;      // ERR_CEIL rung timer
  uint32_t last_stall_evt_ms = 0;
  uint32_t last_sabort_evt_ms = 0;
  uint32_t effort_clamp_ms = 0;  // last time the map-corridor dth clamp engaged
};
static Mg2State mg2[MAX_DOFS];
static inline void mg2ResetDof(uint8_t d) { mg2[d] = Mg2State{}; }
void motionGuardV2ResetState() {
  for (uint8_t d = 0; d < MAX_DOFS; ++d) mg2ResetDof(d);
}
#else
void motionGuardV2ResetState() {}
#endif

static bool HOT_FUNC(computeDeltaThetaCorridor)(const DofLinearEquations &eq,
                                      float theta_A0, float theta_B0,
                                      float stiffness_deg, float cascade_influence,
                                      float margin_deg,
                                      float &dth_min, float &dth_max,
                                      float slope_ratio_A = 1.0f,
                                      float slope_ratio_B = 1.0f) {
  if (cascade_influence < MAP_CORRIDOR_MIN_CASCADE_INFLUENCE) {
    return false;
  }

  float a_min = eq.agonist_safe_min + margin_deg;
  float a_max = eq.agonist_safe_max - margin_deg;
  float b_min = eq.antagonist_safe_min + margin_deg;
  float b_max = eq.antagonist_safe_max - margin_deg;
  if (a_min >= a_max || b_min >= b_max) {
    return false;
  }

  // With cascade slope scaling ON the per-motor term is 0.5*dth*slope_ratio_x, so the
  // dth bound inverts through the ratio (ratios are clamped positive by the caller;
  // 1.0 = the legacy formula, exact same bounds).
  const float inv_half_cascade = 2.0f / cascade_influence;
  const float a_dth_min = ((a_min - theta_A0) * inv_half_cascade - stiffness_deg) / slope_ratio_A;
  const float a_dth_max = ((a_max - theta_A0) * inv_half_cascade - stiffness_deg) / slope_ratio_A;
  const float b_dth_min = ((b_min - theta_B0) * inv_half_cascade + stiffness_deg) / slope_ratio_B;
  const float b_dth_max = ((b_max - theta_B0) * inv_half_cascade + stiffness_deg) / slope_ratio_B;

  dth_min = fmaxf(fmaxf(a_dth_min, b_dth_min), -MAP_CORRIDOR_DTH_ABS_LIMIT_DEG);
  dth_max = fminf(fminf(a_dth_max, b_dth_max), MAP_CORRIDOR_DTH_ABS_LIMIT_DEG);
  return dth_min <= dth_max && (dth_max - dth_min) >= MAP_CORRIDOR_MIN_SPAN_DEG;
}

static bool wp_first_read[MAX_DOFS] = {true, true, true};
static CANErrorTracker wp_canErrorTracker;

// === PHASE 2: motor command reply angle feedback with 0x92 watchdog ===
// First cycle: 0x92 bootstraps revolution tracking (absolute multi-turn angle).
// Subsequent cycles: tracked angle from the 0xA1/0xA4 state reply is source of truth.
// Periodic 0x92 watchdog (~1Hz) verifies tracking hasn't drifted.
static bool wp_rev_track_init[MAX_DOFS] = {false, false, false};
static bool wp_0x92_bootstrap_done[MAX_DOFS] = {false, false, false};
static uint32_t wp_watchdog_cycle_count[MAX_DOFS] = {0, 0, 0};
static uint8_t wp_a1_miss_count[MAX_DOFS] = {0, 0, 0};
static uint32_t wp_last_a1_miss_log_ms[MAX_DOFS] = {0, 0, 0};
// The 0x92 tracking-drift watchdog interval is now a RUNTIME value (wp_watchdog_interval, main.cpp),
// rate-scaled = 1e6/inner_loop_period_us cycles so the wall-clock cadence stays ~1s at every loop rate
// (250 @250Hz, 500 @500Hz, 1000 @1kHz). Recomputed on boot + in the 0x006 handler (which also re-clamps
// each wp_watchdog_cycle_count). The bit2 substitute-fire fires the 0x92 pair on a watchdog-due cycle
// INSTEAD of blocking; the drift protection itself is never weakened (same threshold/pairing/stagger).
extern volatile uint32_t wp_watchdog_interval;
// O10 (500Hz Stage 1): STAGGER the per-DOF watchdog phases so two DOFs never run their 0x92
// double-round-trip in the SAME cycle - that coincidence was the measured 2.8-3.1ms max outlier
// (a second blocking pair read on top of the normal fire/collect). Seeding the counter puts
// DOF k's first fire wp_watchdog_interval*k/MAX_DOFS cycles apart; the cadence stays ~1s.
static inline uint32_t wpWatchdogSeed(uint8_t dof) {
  return ((uint32_t)wp_watchdog_interval * dof) / MAX_DOFS;
}

// 0x006 LOOP_FREQUENCY handler hook (core1): re-derive the rate-scaled watchdog interval and re-clamp each
// per-DOF counter to min(count, new_interval-1). Without the re-clamp, a rate change to a faster loop (shorter
// interval) could leave a counter already >= the new interval -> the watchdog fires EVERY cycle until it laps.
// Owns wp_watchdog_cycle_count[] (this file's statics); called only from the 0x006 handler on core1.
void s2RescaleWatchdogInterval(uint16_t new_inner_period_us) {
  // Gate on SUB92: only the substitute-fire path (bit2) rate-scales the interval. With bit2 OFF keep
  // the fixed Stage-1 cadence (500) so the validated OFF/serial timing is bit-identical (a rate-scaled
  // interval with the BLOCKING watchdog read would double the 4.5-6.3ms stretch cycles). Re-derived on
  // boot, on a 0x006 rate change, AND on a 0x08 bit2 toggle.
  const uint32_t new_iv = sched_sub92_enabled ? deriveWpWatchdogInterval(new_inner_period_us)
                                              : WP_WATCHDOG_INTERVAL_FIXED;
  wp_watchdog_interval = new_iv;
  const uint32_t ceil = (new_iv > 0) ? (new_iv - 1U) : 0U;
  for (uint8_t d = 0; d < MAX_DOFS; d++) {
    if (wp_watchdog_cycle_count[d] > ceil) wp_watchdog_cycle_count[d] = ceil;
  }
}
static int wp_prev_torque_A[MAX_DOFS] = {0};
static int wp_prev_torque_B[MAX_DOFS] = {0};

// === S2 CARRY (Stage-2 cross-cycle deferred collect, IMPEDANCE_CTRL 0x08 bit1) ===
// Function-locals that cross a compute/fire/consume phase boundary live in DofPhaseCtx (per-DOF, on
// executeControlLoop's stack). The TYPE is hoisted to file scope (textually unchanged from the historical
// in-function definition) so the single-slot S2Carry can byte-copy one at fire time and carry it across
// the cycle boundary; the per-cycle phase_ctx[] instances still live on the stack (unchanged).
//   +2 fields vs. the historical struct: kind (PAIR_TORQUE or PAIR_92) and
//   fire_us (the pair's fireTimestampUs snapshot — single time authority for all age/staleness math).
static constexpr uint8_t PAIR_TORQUE = 0;   // 0xA1 torque pair (Loop1) or 0xA4 position pair (Loop2).
static constexpr uint8_t PAIR_92    = 1;    // 0x92 angle-read pair fired INSTEAD of torque on a bit2
                                            // substitute-fire watchdog cycle (S2 SUBSTITUTE-FIRE).
struct DofPhaseCtx {
  LKM_Motor *agonist = nullptr;         // this DOF's motor pair (tendon path)
  LKM_Motor *antagonist = nullptr;
  bool  impedance_active = false;       // consume: slack/HOLD_EVT/DIAG gates
  bool  need_0x92 = false;              // consume: bootstrap/watchdog re-anchor discrimination
  bool  recover_after_a1_miss = false;  // consume: re-anchor after a missed reply
  bool  zero_fire = false;              // fire: bootstrap/recover cycles always fire zero torque
  bool  loop2_active = false;           // fire: 0xA4 dual-position vs 0xA1 torque
  bool  collected = false;              // pair already drained by collectPendingPair() mid-compute
  uint8_t kind = PAIR_TORQUE;           // fire: which pair was fired (PAIR_TORQUE / PAIR_92); OWNED by computeDof
  uint32_t fire_us = 0;                 // fire: fireTimestampUs() snapshot (S2 CARRY single time authority)
  float theta_0_joint = 0.0f;           // consume: qDes for DIAG_MOVE / hi-rate / PID diag
  float theta_0_agonist_motor = 0.0f;   // fire: Loop2 spread-clamp reference
  float theta_0_antagonist_motor = 0.0f;
  float cascade_influence = 0.0f;       // fire: Loop2 max spread
  float stiffness_ref = 0.0f;           // consume: slack/DIAG_HOLD gates
  float stiffness_ref_effective = 0.0f; // consume: DIAG_HOLD/probe telemetry
  float delta_theta_smooth = 0.0f;      // consume: HOLD_EVT/DIAG_MOVE/hi-rate telemetry
  float theta_A_ref = 0.0f;             // fire: Loop2 slew target; consume: HOLD_EVT
  float theta_B_ref = 0.0f;
  float theta_A_curr = 0.0f;            // fire: Loop2 latched-target seed
  float theta_B_curr = 0.0f;
  float command_A = 0.0f;               // fire: 0xA1 torque; consume: HOLD_EVT/PID diag
  float command_B = 0.0f;
  MultiAngleData data_A = {};           // 0x92 absolute read (consume re-anchors from it)
  MultiAngleData data_B = {};
#if CONTROLLER_DEBUG
  uint32_t dof_start_us = 0;            // per-DOF micro-profile window (starts in compute)
  uint32_t torque_start_us = 0;         // torque/diag micro-profile window (starts in fire)
#endif
};

// The single cross-cycle CARRY slot. Owner: core1 executeControlLoop ONLY; core0 NEVER touches it.
// dof < 0 = nothing outstanding (single authority). Written at exactly one point (scheduler tail under
// bit1, carry-eligible cycles); resolved at exactly two (injection at scheduler start, abandonCarriedPair).
struct S2Carry {
  int8_t   dof = -1;             // -1 = nothing outstanding. SINGLE AUTHORITY.
  uint8_t  kind = PAIR_TORQUE;   // which pair is carried
  uint32_t fire_us = 0;          // = agonist->fireTimestampUs() — SINGLE TIME AUTHORITY
  uint32_t gen = 0;              // ++ on every set AND every resolve (double-resolve tripwire)
  DofPhaseCtx ctx;               // POD byte-copy taken AT FIRE TIME (~120B)
};
static S2Carry s2_carry;         // owner: core1 executeControlLoop; core0 NEVER touches it

// S2 CARRY diagnostic counters (cumulative since boot; BUSDIAG-adjacent, read by the [S2DIAG] log).
static uint32_t s2_carried_count = 0;        // pairs carried across a cycle boundary (write point)
static uint32_t s2_injected_count = 0;        // carried pairs re-injected as pending_fire_dof (nominal resolve)
static uint32_t s2_idle_discard_count = 0;    // carried pairs discarded because the DOF went IDLE
static uint32_t s2_stale_backstop_count = 0;  // carried pairs resolved via the staleness backstop
static uint32_t s2_abandon_count = 0;         // carried pairs abandoned at a motor-drive choke point
static uint32_t s2_wrong_core_count = 0;      // abandonCarriedPair() called from core0 (guard no-op)
static uint32_t s2_orphan_count = 0;          // slot still occupied at the write point (MUST stay 0 —
                                              // the injection failed to resolve a prior carry; the
                                              // single-authority tripwire the design mandated)
static uint32_t s2_sub92_drift_deferred_count = 0;  // substitute-cycle drift checks skipped under motion
                                              // (err folds in ~1 cycle of travel -> defer, never
                                              // false-drop torque). See the gate in the PAIR_92 consume.
// SUB92 substitute-cycle drift check: the 0x92 samples the motor at cycle N but getTrackedAngle()
// reflects the last 0xA1 (cycle N-1), so |tracked-absolute| folds in ~1 inner-period of real motion.
// Only escalate on drift when the joint is slow enough that the motion term is negligible vs the 1deg
// threshold; above this JOINT speed, DEFER (the ZOH is unchanged, A1-miss detection still guards
// missed replies, the next slow watchdog re-checks). CONSERVATIVE default — tune at G2 (parity log):
// at ~40 dps bench the one-cycle skew is ~0.16deg (safe); the trip risk starts ~250 dps.
static const float S2_SUB92_DRIFT_CHECK_MAX_VEL_DEG_S = 60.0f;

static constexpr int16_t MOTOR_FEEDBACK_UNUSED_I16 = 0x7FFF;
static constexpr uint8_t MOTOR_FEEDBACK_REASON_NAN_A = 0x01;
static constexpr uint8_t MOTOR_FEEDBACK_REASON_NAN_B = 0x02;
static constexpr uint8_t MOTOR_FEEDBACK_REASON_RANGE_A = 0x04;
static constexpr uint8_t MOTOR_FEEDBACK_REASON_RANGE_B = 0x08;
static constexpr uint8_t MOTOR_FEEDBACK_REASON_A1_MISS = 0x10;
static constexpr uint8_t MOTOR_FEEDBACK_REASON_JUMP = 0x20;

static int16_t HOT_FUNC(motorFeedbackErrX100)(float value) {
  if (isnan(value)) return MOTOR_FEEDBACK_UNUSED_I16;
  const long scaled = lroundf(value * 100.0f);
  return (int16_t)constrain(scaled, -32768L, 32767L);
}

static void resetMotorFeedbackSnapshot(uint8_t dof) {
  if (dof >= MAX_DOFS) return;
  diag_motor_feedback_snapshot[dof].flags = 0;
  diag_motor_feedback_snapshot[dof].a1_miss_count = 0;
  diag_motor_feedback_snapshot[dof].watchdog_cycle_count = 0;
  diag_motor_feedback_snapshot[dof].watchdog_err_a_x100 = MOTOR_FEEDBACK_UNUSED_I16;
  diag_motor_feedback_snapshot[dof].watchdog_err_b_x100 = MOTOR_FEEDBACK_UNUSED_I16;
  diag_motor_feedback_snapshot[dof].invalid_reason = 0;
  diag_motor_feedback_snapshot[dof].last_reply_mask = 0;
}

static void HOT_FUNC(updateMotorFeedbackSnapshot)(uint8_t dof, bool need_0x92, bool recover_after_a1_miss) {
  if (dof >= MAX_DOFS) return;
  uint8_t flags = 0;
  if (wp_rev_track_init[dof]) flags |= 0x01;
  if (wp_0x92_bootstrap_done[dof]) flags |= 0x02;
  if (need_0x92) flags |= 0x04;
  if (recover_after_a1_miss) flags |= 0x08;
  const uint8_t reply_mask = diag_motor_feedback_snapshot[dof].last_reply_mask;
  if (reply_mask & 0x01) flags |= 0x10;
  if (reply_mask & 0x02) flags |= 0x20;
  if (diag_motor_feedback_snapshot[dof].invalid_reason & MOTOR_FEEDBACK_REASON_JUMP) flags |= 0x40;
  diag_motor_feedback_snapshot[dof].flags = flags;
  diag_motor_feedback_snapshot[dof].a1_miss_count = wp_a1_miss_count[dof];
  diag_motor_feedback_snapshot[dof].watchdog_cycle_count =
      (wp_watchdog_cycle_count[dof] > 65535U) ? 65535U : (uint16_t)wp_watchdog_cycle_count[dof];
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
// Set by jcResetCycleProfiling() on a rate change; consumed by the profiling block. Without
// this, the [PROFILING] max register resets only at emission (every 2500 cycles), so a warn
// printed just after a rate switch reports cycles from the PREVIOUS rate (bench 2026-07-07:
// the lone 'max=2274' at 500Hz was almost certainly a 250Hz-era cycle).
static volatile bool cycle_profiling_reset_pending = false;
void jcResetCycleProfiling() { cycle_profiling_reset_pending = true; }
static uint32_t cycle_time_us_avg = 0;        // Running average (exponential)
static uint32_t profiling_start_us = 0;       // Start timestamp for current cycle
static constexpr uint32_t LOOP_OVERRUN_MARGIN_US = 100;
static constexpr uint8_t LOOP_OVERRUN_CONSECUTIVE_LIMIT = 2;   // streak -> latch the LOOP_OVERRUN fault
static constexpr uint8_t LOOP_OVERRUN_ESTOP_LIMIT = 6;        // SUSTAINED overrun (~24ms) -> e-stop guard (tunable)
static constexpr uint32_t BURST_COLLECT_TIMEOUT_US = 1500;   // per-pair poll-drain budget (replies ~0.7-1ms; 2 DOFs -> ~3ms worst, < 4ms)
static uint8_t loop_overrun_streak = 0;
static bool loop_overrun_burst_active = false;

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

// ============================================================================================
// FREE / COMPLIANT HAND-CAPTURE CYCLE  (SEPARATE from executeControlLoop — the validated control
// loop is never entered while free-capture is active; see core1_loop branch + diag 0x07/0x08).
// Zero-torques both tendons of `dof` (back-drivable) and appends ONE hi-rate record this cycle, so
// an operator can move the joint by hand while the 250 Hz recorder captures q + motor angles.
// PRECONDITION: rev-tracking already bootstrapped (enter from a post-startup HOLDING state) and the
// motor rail enabled — otherwise getTrackedAngle() is NaN and the cycle is skipped (no record).
// ============================================================================================
bool JointController::runFreeCaptureCycle(uint8_t dof, bool pending_exit) {
  if (pending_exit) {
    // SAFETY REDESIGN (2026-06-26): the EXIT does NOT re-seed the impedance hold or resume closed-loop
    // control. On HW the old re-hold resumed DOF1 control from a back-driven state and triggered the
    // known saturated-P limit cycle -> a DRIVEN slam to -14.77 deg (the hardening contained it at the
    // -17.43 deg mapping limit, no break, but close). Instead leave the WHOLE joint free and de-powered:
    // (1) stop EVERY motor (motorStop) so NO motor — including the non-captured DOF0 — keeps a non-zero
    //     torque setpoint that a later rail re-enable (e.g. a PRETENSION recovery that flips GP22 HIGH
    //     WITHOUT a full MCU power-cycle) could re-apply open-loop; (2) cut motor power (GP22 LOW -> no
    //     torque is physically possible); (3) park EVERY DOF in IDLE with no valid impedance target so the
    // resumed executeControlLoop drives nothing on either DOF. GP22 gates only motor power, so the MCU/CAN
    // stay alive and the hi-rate dump still works. The operator recovers with a power-cycle. This whole
    // path is dof-independent and fail-safe — it de-powers even if `dof` is somehow invalid.
    stopAllMotors();                       // motorStop() on ALL motors (both DOFs) -> no stale torque setpoint
    safety_motor_power_disable();          // GP22 LOW -> motor power off; joint limp/free
    for (uint8_t d = 0; d < config.dof_count && d < MAX_DOFS; d++) {
      impedance_target[d].valid = false;   // no impedance-watchdog re-hold on any DOF
      dof_state[d] = DofState::IDLE;       // executeControlLoop skips every DOF (no drive)
    }
    return true;                           // caller clears free_capture_active -> loop resumes into IDLE (no-op)
  }

  if (dof >= MAX_DOFS) return false;       // invalid dof on the CAPTURE path -> skip this record (the eventual EXIT de-powers)

  LKM_Motor *ag = cached_agonist[dof];
  LKM_Motor *an = cached_antagonist[dof];
  if (ag == nullptr || an == nullptr) return false;

  // Zero torque on both tendons (back-drivable). The pipelined 0xA1 reply also refreshes each motor's
  // rev-tracking, so getTrackedAngle() below returns the hand-moved angle. Gate on the REPLY's valid:
  // a missed 0xA1 leaves getTrackedAngle stale (NOT NaN), so isnan alone would not catch it.
  PipelinedTorqueResponseData tr = LKM_Motor::setTorquePairPipelined(ag, 0, an, 0);
  if (!tr.dataA.valid || !tr.dataB.valid) return false;   // missed 0xA1 reply -> stale angle, skip record
  float theta_A = ag->getTrackedAngle();
  float theta_B = an->getTrackedAngle();
  if (isnan(theta_A) || isnan(theta_B)) return false;     // defensive (rev-tracking guaranteed by ENTER)

  SharedDofAngles snap;
  readSharedDofAnglesSnapshot(snap);
  if (!snap.valid[dof]) return false;   // skip this record rather than inject a 0 deg outlier into the metric
  float q  = snap.angles[dof];
  float q0 = snap.valid[Q0_DOF] ? snap.angles[Q0_DOF] : linear_equations[dof].q0_nominal;

  // Residuals (motor actual vs map-expected) — SAME quantity as the controlled hi-rate path, so the
  // free-vs-controlled coordination analysis is apples-to-apples. No command in free mode, so
  // qdes := q and dth := 0; iq is ~0 under zero torque -> pass 0.
  float expA = 0.0f, expB = 0.0f, resA = 0.0f, resB = 0.0f;
  if (calculateMotorAnglesWithEquations(dof, q, q, expA, expB, q0)) {
    resA = theta_A - expA;
    resB = theta_B - expB;
  }
  diag_hirate_capture_sample(dof, q, q, 0.0f, 0, 0, resA, resB);
  return false;
}

// ============================================================================================
// VEL-TEST CYCLE  (diag A: characterise the motor's internal velocity loop vs our torque cascade)
// One tendon is driven in VELOCITY mode (setSpeed -> the motor's fast internal loop compensates the
// planetary/tendon friction), the other holds tension in TORQUE (the proven auto-mapping pattern). The
// hi-rate recorder captures q so the SAME smoothness/stall analysis compares velocity-mode vs torque-mode.
// SAFETY: velocity mode bypasses the cascade's position pre-send clamp, so this path enforces its OWN
// per-cycle joint-angle guard (stop short of the mapping safe range) and de-powers on EXIT / guard / a bad
// read / the backstop — fail-safe to motors-off, exactly like the free-capture EXIT. The move is
// position-bounded (away from the start angle), so it can never run open-loop past the safe range.
// ============================================================================================
static bool vel_test_increase = true;                        // direction picked at ENTER (away from start)
static bool vel_test_is_position = false;                    // ENTER mode: false = velocity (0xA2), true = position (0xA4)
static float vel_test_pos_target_A = 0.0f;                   // pos-mode: precomputed driver target motor angles (map
static float vel_test_pos_target_B = 0.0f;                   // far-end), same logical space as sendMultiLoopAngle2Command
static constexpr float VEL_TEST_GUARD_MARGIN_DEG = 1.5f;     // stop this far short of the mapping safe range
static constexpr uint16_t VEL_TEST_POS_MAXSPEED = 100;      // pos-mode 0xA4 maxSpeed. Run #1 (50) measured 1.37 deg/s
                                                            // joint -> 100 ~= 2.74 deg/s to MATCH the velocity run (2.67);
                                                            // the unit is ~0.0274 deg/s-at-the-joint per LSB. The per-cycle
                                                            // guard + the clamped +10.2 target keep it eyelet-safe here.
static constexpr uint32_t VEL_TEST_SNAP_STALE_US = 20000;   // de-power if the joint-angle snapshot is >20ms stale (a
                                                            // frozen-but-valid encoder while a latched command drives)
static constexpr float TORQUE_SWEEP_SPEED_DEG_S = 2.7f;     // torque-sweep cruise: matches the postest (~2.74) / veltest (2.67)
                                                            // realized speeds for an apples-to-apples slips/deg comparison
// fireAngle2 NO-MOTION ping (diag 0x10/0x11): confirms the non-blocking 0xA4 send + auto-reply + collectPair on a
// 0xA4 frame, with ZERO motion (both motors commanded to their CURRENT angle = getTrackedAngle). SLOW maxSpeed +
// a tight joint motion-abort make it safe-by-construction even if the angle convention were off (gentle + guarded).
static float fireangle2_ping_seed_A = 0.0f;                 // agonist motor angle to hold (= current at ENTER)
static float fireangle2_ping_seed_B = 0.0f;                 // antagonist motor angle to hold
static float fireangle2_ping_q_start = 0.0f;               // joint angle at ENTER (the motion-abort reference)
static constexpr uint16_t FIREANGLE2_PING_MAXSPEED = 20;    // SLOW: any convention-error motion is gentle, the guard catches it
static constexpr float FIREANGLE2_PING_MOTION_ABORT_DEG = 0.8f;  // abort+de-power if the joint moves this far (must be no-motion)

// Validate readiness and pick the travel direction (toward the far end of the safe range). Returns false
// (ENTER rejected) unless the motor cache is warm, the DOF is HOLDING, and the joint angle is valid.
bool JointController::armVelTest(uint8_t dof, bool pos_mode) {
  // S2 CARRY choke point (MAJOR amendment): entering the vel/pos-test SEPARATE loop stops executeControlLoop
  // (and thus the carry-injection resolve) from running, so abandon any carried pair now — otherwise its
  // buffered replies would sit ownerless through the diag loop's own bus transactions.
  abandonCarriedPair();
  if (dof >= MAX_DOFS) return false;
  // SAFETY: require a FINE map (PIECEWISE/BILINEAR). The guard rides getMappingSafeRange's joint_safe band; a
  // FINE map gives the eyelet-accurate band (a LINEAR map — especially an un-calibrated one right after a
  // reflash — is lower map-fidelity near the eyelet). velocity/position bypass the cascade's pre-send clamp,
  // so the per-cycle guard is the only net. Reject -> the operator re-calibrates to a FINE map first.
  if (linear_equations[dof].map_mode == MAP_LINEAR) return false;
  if (cached_agonist[dof] == nullptr || cached_antagonist[dof] == nullptr) return false;
  if (dof_state[dof] != DofState::HOLDING) return false;
  SharedDofAngles snap;
  readSharedDofAnglesSnapshot(snap);
  if (!snap.valid[dof]) return false;
  float lo = 0.0f, hi = 0.0f;
  if (!getMappingSafeRange(dof, lo, hi)) return false;
  vel_test_increase = (snap.angles[dof] < 0.5f * (lo + hi));  // move toward the far end of the safe range
  vel_test_is_position = pos_mode;
  if (pos_mode) {
    // POSITION mode (0xA4): precompute the driver's target = the motor angle the MAP gives for the FAR end of
    // the safe range (same logical space as sendMultiLoopAngle2Command; the bilinear lookup uses the live q0).
    // The motor's internal position loop drives there; the per-cycle joint guard stops the joint short of the
    // eyelet. Reject if the map can't produce a target (don't fire a blind absolute position command).
    // SAFETY (super-review): the 0xA4 target is LATCHED in the motor's internal loop and the diagnostic loop
    // BYPASSES the cascade's MAPPING-LIMIT e-stop, so on a hung loop this destination is the system's deepest
    // intent — it MUST point strictly INSIDE the +11.2 eyelet. hi = joint_safe_max + LIMIT_TOLERANCE is ~0.5 deg
    // PAST it; use the GUARD-STOP point (hi - margin), where the per-cycle guard ends the move anyway, so the
    // latched target is fail-safe (~+10.2, ~1 deg inside the eyelet) AND the loop still drives at maxSpeed to it.
    float far_joint = vel_test_increase ? (hi - VEL_TEST_GUARD_MARGIN_DEG) : (lo + VEL_TEST_GUARD_MARGIN_DEG);
    float q0 = (dof != 0 && config.dof_count > 0 && snap.valid[0]) ? snap.angles[0] : NAN;
    if (!calculateMotorAnglesWithEquations(dof, far_joint, far_joint,
                                           vel_test_pos_target_A, vel_test_pos_target_B, q0)) {
      return false;
    }
  }
  return true;
}

// One vel-test cycle. Returns true to END the test (caller clears vel_test_active); false to keep driving
// + capturing. EVERY exit path de-powers (fail-safe, motors off). Mirrors runFreeCaptureCycle's structure.
bool JointController::runVelTestCycle(uint8_t dof, bool pending_exit) {
  bool stop = pending_exit;
  if (!stop && dof >= MAX_DOFS) stop = true;
  LKM_Motor *ag = (dof < MAX_DOFS) ? cached_agonist[dof] : nullptr;
  LKM_Motor *an = (dof < MAX_DOFS) ? cached_antagonist[dof] : nullptr;
  if (!stop && (ag == nullptr || an == nullptr)) stop = true;
  if (!stop) {
    SharedDofAngles snap;
    readSharedDofAnglesSnapshot(snap);
    if (!snap.valid[dof] || (micros() - snap.timestamp_us) > VEL_TEST_SNAP_STALE_US) {
      stop = true;                                            // lost / >20ms-stale joint encoder -> cannot guard -> de-power
    } else {
      float q = snap.angles[dof];
      float lo = 0.0f, hi = 0.0f;
      bool range_ok = getMappingSafeRange(dof, lo, hi);       // defense-in-depth: top-of-fn bound uses MAX_DOFS, the
                                                              // ENTER gate uses dof_count -> close the latent asymmetry
      // Position-bounded: stop short of the safe range in the travel direction; hard-guard both ends too.
      bool reached = vel_test_increase ? (q >= hi - VEL_TEST_GUARD_MARGIN_DEG)
                                       : (q <= lo + VEL_TEST_GUARD_MARGIN_DEG);
      bool out_of_band = (q <= lo - VEL_TEST_GUARD_MARGIN_DEG) || (q >= hi + VEL_TEST_GUARD_MARGIN_DEG);
      if (!range_ok || reached || out_of_band) {
        stop = true;                                          // no safe range / bound reached (or overshoot) -> done
      } else {
        // Command: the DRIVER tendon moves (velocity 0xA2 OR position 0xA4), the OTHER holds TENSION in torque
        // (auto-mapping pattern, proven values). Mirror the auto-mapping motor<->direction mapping EXACTLY,
        // INCLUDING auto_mapping_invert_direction (else an inverted DOF heads the wrong way — guard-bounded but
        // a wrong test). (want_increase XOR invert) picks the antagonist-drives branch.
        int   rtorque = (int)config.dofs[dof].zero_mapping.auto_mapping_resistance_torque;
        bool  invert  = config.dofs[dof].zero_mapping.auto_mapping_invert_direction;
        if (vel_test_is_position) {
          // POSITION mode: drive the same tendon to its MAP far-end target via the motor's internal position
          // loop (angle + maxSpeed). The target is at the safe-range END (just beyond the guard) so the loop
          // keeps driving at maxSpeed and the joint guard above stops it before the eyelet — no early decel.
          if (vel_test_increase ^ invert) { an->sendMultiLoopAngle2Command(vel_test_pos_target_B, VEL_TEST_POS_MAXSPEED); ag->setTorque(rtorque); }
          else                            { ag->sendMultiLoopAngle2Command(vel_test_pos_target_A, VEL_TEST_POS_MAXSPEED); an->setTorque(-rtorque); }
        } else {
          float speed = config.dofs[dof].zero_mapping.auto_mapping_speed;   // FULL auto-mapping speed (proven)
          if (vel_test_increase ^ invert) { an->setSpeed(-speed); ag->setTorque(rtorque); }
          else                            { ag->setSpeed(speed);  an->setTorque(-rtorque); }
        }
        diag_hirate_capture_sample(dof, q, q, 0.0f, 0, 0, 0.0f, 0.0f);  // record q (the comparison signal)
        return false;                                         // continue driving + capturing
      }
    }
  }
  // STOP path (EXIT / invalid / bad-read / position guard / backstop): de-power everything, fail-safe.
  stopAllMotors();
  safety_motor_power_disable();
  for (uint8_t d = 0; d < config.dof_count && d < MAX_DOFS; d++) {
    impedance_target[d].valid = false;
    dof_state[d] = DofState::IDLE;
  }
  return true;
}

// === TORQUE-SWEEP (diag 0x0C/0x0D): firmware-driven smooth ramp tracked by the NORMAL cascade ===
// Seeds a constant-velocity impedance ROLLING SEGMENT (the firmware's own smooth, wall-clock-interpolated
// reference) and keeps impedance active, so executeControlLoop's existing outer->inner->burst-2 cascade tracks
// it with NO host 50 Hz q_x100 stepping — a clean matched-speed TORQUE baseline. UNLIKE armVelTest it does NOT
// bypass the cascade and does NOT precompute a motor target. Returns false (ENTER rejected) unless: FINE map,
// motor cache warm, DOF HOLDING, joint angle valid, an existing valid impedance target (carries the gains from
// the host's startup SET_IMPEDANCE), and a safe range yielding a meaningful (>1 deg) away-from-start span.
bool JointController::armTorqueSweep(uint8_t dof) {
  // S2 CARRY choke point (MAJOR amendment): the torque-sweep KEEPS executeControlLoop running (so the
  // injection would resolve the carry next cycle anyway), but abandon here too for uniformity/safety —
  // the state re-seed below (dof_state->MOVING) is a clean carry boundary. Idempotent, core-affinity guarded.
  abandonCarriedPair();
  if (dof >= MAX_DOFS) return false;
  if (linear_equations[dof].map_mode == MAP_LINEAR) return false;          // FINE map -> eyelet-accurate safe band
  if (cached_agonist[dof] == nullptr || cached_antagonist[dof] == nullptr) return false;
  if (dof_state[dof] != DofState::HOLDING) return false;
  if (!impedance_target[dof].valid) return false;                          // reuse the host's gains; never run with zero gains
  SharedDofAngles snap;
  readSharedDofAnglesSnapshot(snap);
  if (!snap.valid[dof]) return false;
  float lo = 0.0f, hi = 0.0f;
  if (!getMappingSafeRange(dof, lo, hi)) return false;
  const float q_start = snap.angles[dof];
  const bool  increase = (q_start < 0.5f * (lo + hi));                     // sweep toward the far end of the safe range
  const float far_joint = increase ? (hi - VEL_TEST_GUARD_MARGIN_DEG) : (lo + VEL_TEST_GUARD_MARGIN_DEG);
  if (fabsf(far_joint - q_start) < 1.0f) return false;                     // span too small to characterise
  const uint32_t now_ms = millis();

  // Seed the rolling segment like the SET_IMPEDANCE handler, but with a firmware-chosen far-end goal + matched
  // cruise. The existing wall-clock interpolator (sampleImpedanceReferenceInternal) then emits a smooth q_ref
  // ramp, consumed identically at the outer (q_des) and inner (theta_0_joint) cascade reads.
  ImpedanceRollingSegment &seg = impedance_segment[dof];
  seg.q_start_deg = q_start;
  seg.q_goal_deg = far_joint;
  seg.q_ref_deg = q_start;
  seg.speed_abs_deg_s = TORQUE_SWEEP_SPEED_DEG_S;
  seg.dq_ref_deg_s = (far_joint >= q_start ? 1.0f : -1.0f) * TORQUE_SWEEP_SPEED_DEG_S;
  seg.t_start_ms = now_ms;
  uint32_t duration_ms = (uint32_t)(fabsf(far_joint - q_start) / TORQUE_SWEEP_SPEED_DEG_S * 1000.0f + 0.5f);
  if (duration_ms < 1u) duration_ms = 1u;
  seg.t_arrival_ms = now_ms + duration_ms;
  seg.active = true;
  seg.initialized = true;

  // Keep impedance active (so the cascade reads the segment, not the static hold) + the watchdog alive at ENTER.
  // PRESERVE the existing gains (kp/ki/kd/stiffness/...): only the goal/segment/timestamps change.
  impedance_target[dof].q_target_deg = far_joint;
  impedance_target[dof].dq_target_deg_s = TORQUE_SWEEP_SPEED_DEG_S;
  impedance_target[dof].last_update_ms = now_ms;
  impedance_target[dof].watchdog_timed_out = false;
  impedance_target[dof].valid = true;
  dof_state[dof] = DofState::MOVING;
  return true;
}

// Fail-safe stop for the torque-sweep: de-power everything (motors off, rail latched off, every DOF IDLE), the
// same model as runVelTestCycle's exit. Recover via power-cycle.
void JointController::torqueSweepStop() {
  stopAllMotors();
  safety_motor_power_disable();
  for (uint8_t d = 0; d < config.dof_count && d < MAX_DOFS; d++) {
    impedance_target[d].valid = false;
    dof_state[d] = DofState::IDLE;
  }
}

// === fireAngle2 NO-MOTION PING (diag 0x10/0x11): Loop 2 step-2 primitive confirm ===
// armFireAngle2Ping seeds BOTH motors' hold target to their CURRENT angle (getTrackedAngle, the SAME logical
// space fireAngle2 expects — the inner PID tracks theta_ref [map output] vs getTrackedAngle), so re-commanding
// it via 0xA4 is NO-MOTION. Confirms the just-built (committed but never-called) fireAngle2 + collectPair on a
// 0xA4 frame: the send works, the LKM auto-replies to 0xA4, collectPair routes it (rx_miss=0). Rejected unless
// the cache is warm, the DOF is HOLDING, rev-tracking is up (else getTrackedAngle is NaN), and the angle is valid.
bool JointController::armFireAngle2Ping(uint8_t dof) {
  // S2 CARRY choke point (MAJOR amendment): entering the a2ping SEPARATE loop stops executeControlLoop
  // (and the carry-injection resolve), so abandon any carried pair before this loop drives the bus.
  abandonCarriedPair();
  if (dof >= MAX_DOFS) return false;
  if (cached_agonist[dof] == nullptr || cached_antagonist[dof] == nullptr) return false;
  if (dof_state[dof] != DofState::HOLDING) return false;
  if (!wp_rev_track_init[dof]) return false;                 // getTrackedAngle would be NaN/stale
  SharedDofAngles snap;
  readSharedDofAnglesSnapshot(snap);
  if (!snap.valid[dof]) return false;
  float sa = cached_agonist[dof]->getTrackedAngle();
  float sb = cached_antagonist[dof]->getTrackedAngle();
  if (isnan(sa) || isnan(sb)) return false;
  fireangle2_ping_seed_A = sa;
  fireangle2_ping_seed_B = sb;
  fireangle2_ping_q_start = snap.angles[dof];
  LKM_Motor::resetBusDiag();                                 // clean-slate the counters for this ping
  return true;
}

// One fireAngle2-ping cycle: hold BOTH motors at their ENTER angle via non-blocking 0xA4 (fireAngle2) +
// collectPair, NO motion (target = the seed = current angle). The joint motion-abort guard de-powers on ANY
// motion (a convention error / disturbance), the SLOW maxSpeed keeps any such motion gentle. Returns true to END.
bool JointController::runFireAngle2PingCycle(uint8_t dof, bool pending_exit) {
  bool stop = pending_exit;
  if (!stop && dof >= MAX_DOFS) stop = true;
  LKM_Motor *ag = (dof < MAX_DOFS) ? cached_agonist[dof] : nullptr;
  LKM_Motor *an = (dof < MAX_DOFS) ? cached_antagonist[dof] : nullptr;
  if (!stop && (ag == nullptr || an == nullptr)) stop = true;
  if (!stop) {
    SharedDofAngles snap;
    readSharedDofAnglesSnapshot(snap);
    if (!snap.valid[dof] || (micros() - snap.timestamp_us) > VEL_TEST_SNAP_STALE_US) {
      stop = true;                                           // lost / stale joint encoder -> de-power
    } else if (fabsf(snap.angles[dof] - fireangle2_ping_q_start) > FIREANGLE2_PING_MOTION_ABORT_DEG) {
      stop = true;                                           // the joint MOVED -> this is a NO-MOTION ping -> abort
      LOG_C1_ERROR("[A2PING] DOF " + String(dof) + " unexpected motion (" +
                   String(snap.angles[dof] - fireangle2_ping_q_start, 2) + " deg) -> abort + de-power");
    } else {
      ag->fireAngle2(fireangle2_ping_seed_A, FIREANGLE2_PING_MAXSPEED);
      an->fireAngle2(fireangle2_ping_seed_B, FIREANGLE2_PING_MAXSPEED);
      LKM_Motor::collectPair(ag, an, BURST_COLLECT_TIMEOUT_US);
      float resA = ag->getTrackedAngle() - fireangle2_ping_seed_A;   // motor drift from the seed (~0 expected)
      float resB = an->getTrackedAngle() - fireangle2_ping_seed_B;
      diag_hirate_capture_sample(dof, snap.angles[dof], fireangle2_ping_q_start, 0.0f,
                                 ag->motorTorqueCurrent, an->motorTorqueCurrent, resA, resB);
      return false;
    }
  }
  // STOP path: log the bus-diag counters (THE confirmation: rx_miss=0 -> collectPair works on 0xA4), then de-power.
  LOG_C1_INFO("[A2PING] tx_fail=" + String(LKM_Motor::txFailCount()) +
              " rx_miss=" + String(LKM_Motor::rxMissCount()) +
              " rx_overflow=" + String(LKM_Motor::rxOverflowCount()));
  stopAllMotors();
  safety_motor_power_disable();
  for (uint8_t d = 0; d < config.dof_count && d < MAX_DOFS; d++) {
    impedance_target[d].valid = false;
    dof_state[d] = DofState::IDLE;
  }
  return true;
}

// True only if `dof` can safely ENTER free-capture: the motor cache is warm (it is populated only by
// executeControlLoop), rev-tracking is bootstrapped on both tendons, and the DOF is HOLDING. The ENTER
// handler rejects otherwise, so free-capture never zero-torques with no/garbage angles.
bool JointController::isFreeCaptureReady(uint8_t dof) {
  if (dof >= MAX_DOFS) return false;
  LKM_Motor *ag = cached_agonist[dof];
  LKM_Motor *an = cached_antagonist[dof];
  if (ag == nullptr || an == nullptr) return false;                  // cold motor cache (DOF never ran control)
  if (!ag->isRevTrackInit() || !an->isRevTrackInit()) return false;  // rev-tracking not bootstrapped
  if (dof_state[dof] != DofState::HOLDING) return false;             // enter only from a held state
  return true;
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

// S2 CARRY choke point (MAJOR amendment). Abandon any pair the cross-cycle CARRY left in flight, so a
// motor-drive entry point that BYPASSES the control loop's injection resolve (stopAllMotors, pretension/
// release, recalc, the diag-loop arm*) can safely touch the motor bus without a carried pair's stale
// _fire_pending / RX frames corrupting the next transaction. Idempotent: a no-op when nothing is carried.
//   - core-affinity guard: s2_carry is owned by core1; core0 reaches stopAllMotors during startup when
//     carry is not running, so a core0 call is a no-op (+ wrong-core counter) — it must NOT flush core1's
//     motor bus or clear a slot core1 might be mid-writing.
//   - clears BOTH motors' _fire_pending as a miss + flushStaleRx(4) (drop the carried pair's now-ownerless
//     buffered replies) + clears s2_carry (gen++, single-authority) + counter.
void JointController::abandonCarriedPair() {
  if (get_core_num() != 1) { s2_wrong_core_count++; return; }
  if (s2_carry.dof < 0) return;                       // nothing carried — idempotent no-op
  const int8_t dof = s2_carry.dof;
  LKM_Motor *ag = (dof >= 0 && dof < MAX_DOFS) ? cached_agonist[dof] : nullptr;
  LKM_Motor *an = (dof >= 0 && dof < MAX_DOFS) ? cached_antagonist[dof] : nullptr;
  if (ag != nullptr) { ag->clearFire(); ag->flushStaleRx(4); }
  if (an != nullptr) { an->clearFire(); an->flushStaleRx(4); }
  s2_carry.dof = -1;
  s2_carry.gen++;
  s2_abandon_count++;
  LOG_C1_WARN("[S2] carried pair abandoned (DOF " + String((int)dof) + ")");
}

bool HOT_FUNC(JointController::executeControlLoop)() {
  // === PROFILING: Record cycle start time ===
  profiling_start_us = time_us_32();

  // === GMS bristle: measured inner-cycle dt (v2 timing fix) ===
  // The bristle integrates over REAL elapsed time; the configured inner_loop_period_us is wrong on
  // CAN-bound overruns (~7000us vs 4000us) and mistuned the v1 break-away phase (the bench coin-flip).
  // Use the measured interval between cycle starts, clamped to reject the first-cycle / timer-wrap garbage.
  {
    float gms_dt = (profiling_start_us - gms_prev_cycle_us) * 1e-6f;
    gms_dt_s = (gms_dt > 1e-3f && gms_dt < 20e-3f) ? gms_dt : (inner_loop_period_us * 1e-6f);
    gms_prev_cycle_us = profiling_start_us;
  }

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

  // ===== RUNAWAY-VELOCITY SAFETY (TEMPORARY, DISABLED 2026-06-28) =======================================
  // Was a joint-velocity ceiling that tripped the FULL e-stop to catch the fast self-oscillation EARLY while
  // probing it on the bench. DISABLED because it tripped on the operator's manual hand-vibration (~35 deg/s >
  // 30), so it can't coexist with probing the instability by hand; and the ROOT fix below (non-blocking
  // motor-CAN + PID measured-dt) is the real cure (it PREVENTS the oscillation rather than catching it).
  // Kept here under #if 0 so it can be re-enabled for a future supervised oscillation capture.
#if 0
  static constexpr float JOINT_VEL_ESTOP_DPS = 30.0f;
  for (uint8_t vdof = 0; vdof < config.dof_count; vdof++) {
    if (config.dofs[vdof].drive_type == DRIVE_ANTAGONISTIC_TENDON &&
        dof_snapshot.valid[vdof] && fabsf(dof_snapshot.velocities[vdof]) > JOINT_VEL_ESTOP_DPS) {
      LOG_C1_ERROR("[SAFETY] DOF " + String(vdof) + " OVERSPEED " + String(dof_snapshot.velocities[vdof], 1) +
                   " deg/s > " + String(JOINT_VEL_ESTOP_DPS, 0) + " deg/s — EMERGENCY STOP (runaway guard)");
      emergency_stop_requested = true;
    }
  }
#endif
  // =====================================================================================================

  // === Motor-CAN collect is now PER-DOF BURST-AWARE (fire pair -> collectPair), in the loop below ===
  // The old global "fire ALL 4, collect next cycle" OVERFLOWED the MCP2515's 2-deep RX buffer (4
  // simultaneous replies vs 2 buffers) AND could silently drop a 4th un-checked TX -> ~80% miss,
  // limping via constant 0x92 recover. A burst of <=2 outstanding per DOF respects the hardware.

  // Direct-drive DOFs use the motor's internal absolute encoder on Core1.
  // Override the snapshot locally so the rest of the control loop can stay
  // topology-agnostic.
  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    if (config.dofs[dof].drive_type != DRIVE_DIRECT_DRIVE) {
      continue;
    }

    const bool idle_probe_mode = (dof_state[dof] == DofState::IDLE);

    LKM_Motor *direct_motor = nullptr;
    int direct_motor_idx = -1;
    for (int i = 0; i < config.motor_count; i++) {
      if (config.motors[i].dof_index == dof &&
          config.motors[i].role == MOTOR_ROLE_DIRECT) {
        direct_motor = motors[i];
        direct_motor_idx = i;
        break;
      }
    }

    if (direct_motor == nullptr) {
      dof_snapshot.valid[dof] = false;
      dof_snapshot.velocities[dof] = 0.0f;
      direct_drive_invalid_streak[dof] = DIRECT_DRIVE_INVALID_STREAK_LIMIT;
      direct_drive_last_update_us[dof] = 0;
      direct_drive_next_probe_ms[dof] = 0;
      updateDirectDriveFeedback(dof, 0.0f, 0.0f, false);
      continue;
    }

    const uint32_t now_ms = millis();
    // Idle probe backoff must not leak into the first active cycle after
    // startup/recovery. Once the DOF leaves IDLE we need an immediate fresh
    // motor-internal read, otherwise the control loop can falsely fault on the
    // first HOLDING/MOVING iteration using an old idle-throttling deadline.
    if (!idle_probe_mode) {
      direct_drive_next_probe_ms[dof] = 0;
    }
    if (idle_probe_mode &&
        direct_drive_next_probe_ms[dof] != 0 &&
        (int32_t)(now_ms - direct_drive_next_probe_ms[dof]) < 0) {
      dof_snapshot.valid[dof] = false;
      dof_snapshot.velocities[dof] = 0.0f;
      continue;
    }

    const bool have_saved_reference = _saved_offsets[dof].valid;
    LKM_Motor::MultiAngleData raw_angle = direct_motor->getSingleAngleSync();
    if (isnan(raw_angle.angle)) {
      diag_note_motor_timeout(dof, direct_motor_idx >= 0 ? static_cast<uint8_t>(direct_motor_idx) : 0xFF);
      if (direct_drive_invalid_streak[dof] < 0xFF) {
        direct_drive_invalid_streak[dof]++;
      }
      direct_drive_next_probe_ms[dof] =
          now_ms + (have_saved_reference ? DIRECT_DRIVE_TIMEOUT_BACKOFF_MS
                                         : DIRECT_DRIVE_UNREFERENCED_PROBE_MS);
      if (direct_drive_invalid_streak[dof] < DIRECT_DRIVE_INVALID_STREAK_LIMIT &&
          direct_drive_last_update_us[dof] > 0) {
        dof_snapshot.angles[dof] = direct_drive_last_angle[dof];
        dof_snapshot.velocities[dof] = 0.0f;
        dof_snapshot.valid[dof] = true;
        updateDirectDriveFeedback(dof, direct_drive_last_angle[dof], 0.0f, true);
      } else {
        dof_snapshot.valid[dof] = false;
        dof_snapshot.velocities[dof] = 0.0f;
        direct_drive_last_update_us[dof] = 0;
        updateDirectDriveFeedback(dof, 0.0f, 0.0f, false);
      }
      continue;
    }

    if (!have_saved_reference) {
      direct_drive_invalid_streak[dof] = DIRECT_DRIVE_INVALID_STREAK_LIMIT;
      direct_drive_next_probe_ms[dof] = now_ms + DIRECT_DRIVE_UNREFERENCED_PROBE_MS;
      dof_snapshot.valid[dof] = false;
      dof_snapshot.velocities[dof] = 0.0f;
      direct_drive_last_update_us[dof] = 0;
      updateDirectDriveFeedback(dof, 0.0f, 0.0f, false);
      continue;
    }
    direct_drive_invalid_streak[dof] = 0;
    direct_drive_next_probe_ms[dof] = idle_probe_mode ? (now_ms + DIRECT_DRIVE_IDLE_PROBE_MS) : 0;

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
  // Wrap-safe phase (see outer_phase_counter decl). On a divisor change the phase
  // resets so the first cycle at the new divisor is outer-due — the on-change Ts
  // rescale inside the outer block then runs before the outer PID call.
  if (outer_phase_divisor != effective_divisor) {
    outer_phase_divisor = effective_divisor;
    outer_phase_counter = 0;
  }
  const uint16_t cycle_in_outer_phase = outer_phase_counter;  // 0..divisor-1, 0 = outer-due
  const bool outer_cycle_due = (cycle_in_outer_phase == 0);
  outer_phase_counter = (uint16_t)((outer_phase_counter + 1u) % effective_divisor);
  
  // ============================================================================================
  // PER-DOF PROCESSING — split into three phases so the motor-CAN pair round-trip can be
  // scheduled (O1 STRICT INTERLEAVE, 500Hz Stage 1 — design study DOF1_BENCH_RESULTS 2026-07-03):
  //
  //   computeDof(dof)         outer PID / map / governor / inner PID / safety guards —
  //                           everything up to (not including) the motor-CAN fire. Returns
  //                           false when the DOF is skipped this cycle (IDLE, faults, guards,
  //                           direct-drive handled inline): no fire, no drain for that DOF.
  //   fireDof(dof)            fire this DOF's <=2-frame pair (0xA1 torque, or 0xA4 in Loop2).
  //   drainConsumeDof(dof)    drain the pair's replies (deadline-aware timeout) + Phase-2
  //                           consume (rev-tracking anchor/watchdog/miss) + slack/holding
  //                           diagnostics + the per-DOF profiling tail.
  //
  // With sched_interleave_enabled OFF (boot default) the three phases run back-to-back per
  // DOF — the statement order is EXACTLY the historical serial loop (knob-OFF identical).
  //
  // With it ON and two processable tendon DOFs a,b the schedule becomes:
  //     compute(a) -> fire(a) -> compute(b) -> drain(a) -> fire(b) -> drain(b)
  // hiding DOF_b's control math inside pair_a's ~500-600us flight window. (v1 limitation:
  // nothing overlaps pair_b's flight — hiding that too needs the Stage-2 cross-cycle
  // deferred collect.)
  //
  // HARD INVARIANT (the MCP2515 has only 2 RX buffers): never more than ONE pair (2 frames)
  // outstanding on the motor CAN. Preserved BY CONSTRUCTION: the scheduler drains the pending
  // pair before the next fire, and computeDof calls collectPendingPair() before EVERY
  // motor-CAN operation it can issue (the 0x92 recovery round-trip — whose stale-RX flush
  // would otherwise DROP an in-flight pair's replies — direct-drive setTorque, and the
  // fault-path motor stops/cuts), so no branch can stack a second transaction on the bus.
  //
  // DATAFLOW INVARIANT: computing DOF_b before draining pair_a reads the same data as the
  // serial order — DOF_b's inner PID consumes getTrackedAngle() of ITS OWN motors (advanced
  // by DOF_b's own collect of the PREVIOUS cycle; pair_a's collect only touches DOF_a's
  // motors) and the joint-encoder snapshot taken once per cycle above. Only ORDER changes.
  //
  // Function-locals that cross a phase boundary live in DofPhaseCtx (per-DOF, on this frame's
  // stack); the phase bodies alias them by reference so the historical code text is unchanged
  // where possible. File-scope per-DOF arrays are untouched by the restructure. The DofPhaseCtx
  // TYPE is now hoisted to file scope (for the S2 CARRY single-slot copy); the per-cycle instances
  // still live here on the stack.
  // ============================================================================================
  DofPhaseCtx phase_ctx[MAX_DOFS] = {};
  int pending_fire_dof = -1;  // DOF with a fired, not-yet-drained pair (interleave only; serial keeps -1)
  // FATAL-1 (ctx clobber): the ctx the pending DOF's consume reads. In-cycle callers keep pending_ctx =
  // &phase_ctx[dof] (textually neutral — the serial path stays statement-identical). The S2-CARRY-injected
  // consume points it at &s2_carry.ctx so the carried DOF's OWN compute (which overwrites phase_ctx[dof]
  // this cycle) can never clobber the fire-time snapshot before it is consumed.
  DofPhaseCtx *pending_ctx = nullptr;
  // FATAL-2 gate: the carried/injected pair drains with the unconditional pre-sweep (its replies already
  // landed by drain time); in-cycle interleave pairs do NOT (bit-identity — their fire is ~us before the
  // collect). Set true ONLY at the injection resolve, cleared once the injected pair is consumed.
  bool pending_is_carried = false;

  // O10 (500Hz Stage 1): DEADLINE-AWARE collect timeout, shared by the in-slot drain and the
  // mid-compute collectPendingPair() so the timeout stays deadline-aware in BOTH schedule
  // orders. The fixed 1500us constant is 75% of a 2000us budget - one genuine miss would blow
  // the whole 500Hz cycle. Cap the wait to the remaining cycle budget minus a post-collect
  // reserve, but NEVER below the physical pair round-trip: a collect truncated under it
  // discards the fire's feedback entirely (miss -> zero_fire 0x92 recovery -> the recovery's
  // own bus time re-truncates the next collect = self-sustaining torque-hole loop, observed
  // live on the knee @500Hz 2026-07-03). Floor = 700us covers the full measured [LATHIST]
  // distribution (97% in 400-500us, tail <700us); a floored cycle takes a soft ~10% overrun
  // instead of losing motor feedback. At 250Hz (4000us budget) the floor never engages.
  auto collectDeadlineUs = [&]() HOT_LAMBDA_ATTR("collectDeadlineUs") -> uint32_t {
    uint32_t collect_deadline_us = BURST_COLLECT_TIMEOUT_US;
    const uint32_t elapsed_us = time_us_32() - profiling_start_us;
    const uint32_t POST_COLLECT_RESERVE_US = 250;
    const uint32_t COLLECT_FLOOR_US = 700;  // >= measured fire->reply p99.9 (LATHIST 2026-07-03)
    uint32_t budget_left = (inner_loop_period_us > elapsed_us + POST_COLLECT_RESERVE_US)
                               ? (uint32_t)inner_loop_period_us - elapsed_us - POST_COLLECT_RESERVE_US
                               : 0;
    if (budget_left < COLLECT_FLOOR_US) budget_left = COLLECT_FLOOR_US;
    if (budget_left < collect_deadline_us) collect_deadline_us = budget_left;
    return collect_deadline_us;
  };

  // Bus-level drain of the pending in-flight pair (interleave only — a no-op while serial or
  // with nothing pending, so the knob-OFF path is untouched). Only the RX drain happens here;
  // the pending DOF's consume/diag pass still runs at its scheduler slot (drainConsumeDof sees
  // c.collected and skips the second collect).
  auto collectPendingPair = [&]() HOT_LAMBDA_ATTR("collectPendingPair") {
    if (pending_fire_dof < 0) return;
    // FATAL-1: read the ctx the scheduler bound to this pending pair (phase_ctx[dof] in-cycle, or
    // &s2_carry.ctx for a carried/injected pair), never phase_ctx[pending_fire_dof] blindly.
    DofPhaseCtx &pc = pending_ctx ? *pending_ctx : phase_ctx[pending_fire_dof];
    if (pc.collected) return;
    // FATAL-2: a carried pair's replies are already buffered -> pre-sweep before the deadline WAIT.
    LKM_Motor::collectPair(pc.agonist, pc.antagonist, collectDeadlineUs(), pending_is_carried);
    pc.collected = true;
  };

  // --- PHASE 1: per-DOF control computation (everything before the motor-CAN fire) ---
  // Returns false when this DOF is skipped this cycle: the scheduler must then neither fire
  // nor drain it (a skipped DOF never leaves a pair in flight); other DOFs proceed normally.
  auto computeDof = [&](uint8_t dof) HOT_LAMBDA_ATTR("computeDof") -> bool {
    DofPhaseCtx &c = phase_ctx[dof];

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
      gms_z[dof] = 0.0f;                    // clear the GMS bristle -> cold-start on re-activation
      // Reset Phase2 tracking state
      wp_rev_track_init[dof] = false;
      wp_0x92_bootstrap_done[dof] = false;
      wp_watchdog_cycle_count[dof] = wpWatchdogSeed(dof);
      wp_a1_miss_count[dof] = 0;
      wp_last_a1_miss_log_ms[dof] = 0;
      wp_prev_torque_A[dof] = 0;
      wp_prev_torque_B[dof] = 0;
      prev_command_A[dof] = 0.0f;      // torque rate-limiter history: never carry a pre-IDLE
      prev_command_B[dof] = 0.0f;      // command into the next session's ramp (zero_fire replay)
      prev_command_direct[dof] = 0.0f;
      resetMotorFeedbackSnapshot(dof);
      // Reset per-DOF safety fault counters so they never carry across an IDLE
      // gap into the next session (a streak from a prior session must not let a
      // single fresh trip latch power off / cut torque).
      motor_range_trip_streak[dof] = 0;
      tendon_encoder_fault_streak[dof] = 0;
      // Drop the cached last-valid joint angle so a stale value from a prior session
      // can never seed a watchdog freeze in the next one (ITEM 4 cache hygiene).
      has_last_valid_joint_angle[dof] = false;
      return false; // Skip this DOF entirely (no fire, no drain this cycle)
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
    c.dof_start_us = dof_start_us;  // the per-DOF profiling tail closes in drainConsumeDof
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
      wp_watchdog_cycle_count[dof] = wpWatchdogSeed(dof);
      wp_a1_miss_count[dof] = 0;
      wp_last_a1_miss_log_ms[dof] = 0;
      wp_prev_torque_A[dof] = 0;
      wp_prev_torque_B[dof] = 0;
      resetMotorFeedbackSnapshot(dof);
      resetMovementSettleState(dof);

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
        // hold and stop chasing host updates until a fresh SET_IMPEDANCE arrives.
        // In Loop2, clear the 0xA4 latched-position path first; a torque cut alone
        // would not stop the motor's internal position controller.
        // Latch the local hold at the last VALID actual joint angle (ITEM 4). If the
        // current read is valid we use it (nominal — unchanged). If it is momentarily
        // invalid at the timeout instant, prefer the most recent cached valid actual
        // joint angle rather than getImpedanceHoldReference(): the segment reference
        // LEADS the joint mid-move, so freezing to it would command the joint forward
        // open-loop. Only if no valid joint angle has ever been seen do we fall back to
        // the prior behavior (hold reference) — there is no better estimate available.
        float q_curr_wd;
        if (dof_data.valid[dof]) {
          q_curr_wd = dof_data.angles[dof];
        } else if (has_last_valid_joint_angle[dof]) {
          q_curr_wd = last_valid_joint_angle[dof];
        } else {
          q_curr_wd = getImpedanceHoldReference(dof);
        }
        if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION) {
          collectPendingPair();  // motor-CAN op below: drain any in-flight pair first (<=1 pair invariant)
          stopDofMotors(dof);
          forceLoop1ActuationMode(dof);
          LOG_C1_WARN_F("[IMPEDANCE] DOF%d watchdog disabled Loop2 0xA4 latch before local hold",
                        dof);
        }
        freezeImpedanceToLocalHold(dof, q_curr_wd, t_now, true);
        cur_dof_state = DofState::HOLDING;
        diag_note_watchdog_timeout(dof, elapsed);
        // The dead host's feedforward must not survive it: zero the APPLIED tau_ff and
        // the core1 accumulator (the next seq0 restages from it) — otherwise a chained
        // session's first frame0-only transaction re-commits the stale learned bias.
        impedance_target[dof].tau_ff = 0;
        clearImpedanceTauFF(dof);
        {
          char fb[48];
          // Tail shortened (v2 review): the old text always overran the 119-byte log cap and
          // lost its closing paren; this renders complete.
          LOG_C1_WARN_F("[IMPEDANCE] DOF%d watchdog timeout (%lums > %lums) → LOCAL HOLD at "
                        "%s° (gated until next SET_IMPEDANCE)",
                        dof, (unsigned long)elapsed, (unsigned long)(uint32_t)impedance_watchdog_ms,
                        c1f(fb, q_curr_wd, 2));
        }
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
        wp_watchdog_cycle_count[dof] = wpWatchdogSeed(dof);
        wp_a1_miss_count[dof] = 0;
        wp_last_a1_miss_log_ms[dof] = 0;
        wp_prev_torque_A[dof] = 0;
        wp_prev_torque_B[dof] = 0;
        resetMotorFeedbackSnapshot(dof);
        resetMovementSettleState(dof);
        LOG_C1_INFO("[IMPEDANCE] DOF " + String(dof) + " bumpless init at " + String(q_init, 1) + "°");
      }
    }

    // Track impedance mode status for this DOF (used by both outer and inner loop)
    bool &impedance_active = c.impedance_active;  // crosses into drainConsumeDof (slack/diag gates)
    impedance_active = impedance_target[dof].valid && !impedance_target[dof].watchdog_timed_out;

    // === TENDON ENCODER-INVALID TORQUE-CUT — EVERY INNER CYCLE (ITEM 1) ===
    // The slam window is on the INNER loop: with a frozen last command the tendon is
    // driven open-loop into its mechanical stop every inner cycle. Evaluate the joint-
    // encoder validity, the fault streak, and the torque cut on EVERY inner cycle,
    // independent of outer_cycle_due — otherwise (outer_loop_divisor>1) the cut would be
    // deferred up to N inner cycles, widening that window. At the default divisor==1 this
    // ran every cycle already, so behavior is IDENTICAL; the streak threshold (3) now
    // counts inner cycles unconditionally (~12 ms @ 250 Hz) regardless of divisor.
    //
    // Scope: TENDON DOFs only. Direct-drive feedback faults keep their existing handling
    // inside the outer block (separate mechanism, already rate-limited). The global
    // encoder-fault e-stop (core0) remains the backstop for persistent loss.
    {
      const bool tendon_dof = !isDirectDriveDof(dof);
      if (dof_data.valid[dof]) {
        // ITEM 4 cache feed: remember the last real joint angle for the watchdog freeze.
        last_valid_joint_angle[dof] = dof_data.angles[dof];
        has_last_valid_joint_angle[dof] = true;
        // Valid cycle: DECAY the tendon fault counter by 1 (saturating at 0) — a flapping
        // encoder still climbs to the cut threshold, an isolated noise invalid hovers near 0.
        if (tendon_dof && tendon_encoder_fault_streak[dof] > 0) {
          tendon_encoder_fault_streak[dof]--;
        }
      } else if (tendon_dof) {
        // Tendon DOF with no valid feedback this inner cycle. Never keep driving on the
        // frozen last command: after a short run of bad cycles cut THIS DOF's motor torque
        // so the joint can't be driven open-loop into its mechanical stop (slam prevention).
        tendon_encoder_fault_streak[dof]++;
        if (tendon_encoder_fault_streak[dof] >= TENDON_ENCODER_FAULT_CUT_CYCLES) {
          collectPendingPair();  // motor-CAN ops below (torque cut / Loop2 latch): drain in-flight pair first
          if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION) {
            diag_note_encoder_invalid(dof);
            LOG_C1_ERROR("[Control] DOF " + String(dof) +
                         " encoder invalid in Loop2 — depowering to clear latched 0xA4 target");
            latchTerminalMotionFault(dof, "LOOP2_ENCODER_INVALID", this);
            return false;
          }
          for (int i = 0; i < config.motor_count; i++) {
            if (config.motors[i].dof_index == dof && motors[i] != nullptr) {
              motors[i]->setTorque(0);
            }
          }
          diag_note_encoder_invalid(dof);
          pid_reset_needed[dof] = true;  // (only consumed on an IDLE->active transition; kept for that path)
          // Make the cut's zero AUTHORITATIVE for the recovery too: without these, the rate limiter
          // stays anchored at the stale pre-fault command and the inner PID output equals its frozen
          // uprev, so the FIRST recovered cycle would re-fire ~pre-fault torque in one step (bypassing
          // torque_ramp) — a tension snap on a possibly-slack tendon (the zero_fire replay family).
          inner_pid_init_needed[dof] = true;   // bumpless inner-PID reinit at first valid motor data
          prev_command_A[dof] = 0.0f;          // recovery torque ramps 0 -> target at the limiter rate
          prev_command_B[dof] = 0.0f;
          gms_z[dof] = 0.0f;             // clear the GMS bristle so it cold-starts when feedback returns
          if (t_now - tendon_encoder_fault_log_ms[dof] > 250) {
            LOG_C1_ERROR("[Control] DOF " + String(dof) +
                         " encoder invalid — motor torque cut (open-loop drive prevented)");
            tendon_encoder_fault_log_ms[dof] = t_now;
          }
        } else {
          // (chatter log removed, v2 String pass 2026-07-06)
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
      
      const bool direct_drive_dof = isDirectDriveDof(dof);

      if (direct_drive_dof && direct_drive_feedback_fault_active[dof] && dof_data.valid[dof]) {
        float q_recovered = dof_data.angles[dof];
        direct_drive_feedback_fault_active[dof] = false;
        direct_drive_feedback_fault_log_ms[dof] = t_now;
        LOG_C1_WARN("[Control] DOF " + String(dof) +
                    " direct-drive feedback restored at " + String(q_recovered, 2) +
                    "° — startup required to re-arm movement");
      }

      // Read current angle from shared state (updated by Core0, or overridden locally for direct-drive)
      if (!dof_data.valid[dof]) {
        if (direct_drive_dof) {
          collectPendingPair();  // motor-CAN op below (zero torque): drain any in-flight pair first
          if (t_now - direct_drive_feedback_fault_log_ms[dof] > 250) {
            LOG_C1_ERROR("[Control] DOF " + String(dof) +
                         " direct-drive feedback lost — zero torque, clearing impedance, entering IDLE");
            direct_drive_feedback_fault_log_ms[dof] = t_now;
          }

          if (t_now - direct_drive_feedback_zero_torque_ms[dof] > 20) {
            for (int i = 0; i < config.motor_count; i++) {
              if (config.motors[i].dof_index == dof &&
                  config.motors[i].role == MOTOR_ROLE_DIRECT &&
                  motors[i] != nullptr) {
                motors[i]->setTorque(0);
                break;
              }
            }
            direct_drive_feedback_zero_torque_ms[dof] = t_now;
          }

          if (impedance_target[dof].valid) {
            clearImpedanceControlState(dof, this);
          }
          setMovementReadyForDof(dof, false);
          dof_state[dof] = DofState::IDLE;
          cur_dof_state = DofState::IDLE;
          prev_dof_state[dof] = DofState::IDLE;
          pid_reset_needed[dof] = true;
          inner_pid_init_needed[dof] = false;
          expected_velocity_cache[dof] = 0.0f;
          gms_z[dof] = 0.0f;               // clear the GMS bristle on a direct-drive feedback fault
          direct_drive_feedback_fault_active[dof] = true;
          return false;
        }

        // Tendon DOF with no valid feedback. The streak ++/decay and the open-loop
        // slam-prevention torque cut now run EVERY inner cycle in the dedicated block
        // before the outer loop (ITEM 1) so the cut is never deferred by outer_loop_divisor.
        // Here we only skip the outer-loop math for this cycle (no valid joint angle).
        return false;
      }
      // (Valid-cycle streak decay + the last-valid-angle cache are handled in the
      //  every-inner-cycle ITEM 1 block above.)
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

      if (!compliance_detection_enabled) {
        if (cs.compliance_active) {
          LOG_C1_INFO("[Compliance] DOF " + String(dof) + " DISABLED by host diagnostic");
        }
        cs.reset();
      } else if (!cs.compliance_active) {
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
#if MOTION_GUARD_V2
          bool mg2_stall_abort = false;
          const char *mg2_stall_kind = nullptr;
          if (motion_guard_mode != 0) {
            // Encoder-invalid cycles early-return upstream: blindness appears as a gap.
            Mg2State &g = mg2[dof];
            const uint32_t mg2_sraw = (g.stall_last_ms == 0 || t_now <= g.stall_last_ms)
                                          ? 4u : (t_now - g.stall_last_ms);
            const uint32_t mg2_sdt = (mg2_sraw > MG2_GAP_TICK_MS) ? 4u : mg2_sraw;
            if (mg2_sraw > MG2_STALL_GAP_RESET_MS) {
              // Segment/HOLDING/blindness boundary: previous-move anchors and integrators
              // are evidence about a DIFFERENT motion — start virgin (review: carried
              // i_stall + a stale anchor aborted healthy segments ~50ms in).
              g.stall_anchor_q = q_curr;
              g.stall_anchor_ms = t_now;
              g.stall_no_progress = false;
              g.i_stall_ms = 0.0f;
              g.err_ceil_ms = 0.0f;
            }
            g.stall_last_ms = t_now;
            if (g.stall_anchor_ms == 0) {
              g.stall_anchor_q = q_curr;
              g.stall_anchor_ms = t_now;
              g.stall_no_progress = false;
            } else if (t_now - g.stall_anchor_ms >= MG2_STALL_WINDOW_MS) {
              // Close the window. Progress = |displacement|, direction-agnostic (a joint
              // that MOVED is not mechanically stuck; commanded-direction reversals inside
              // a window must not fake a stall — review), scaled with commanded speed so
              // slow-creep grinding below the legacy velocity-ratio still reads as stalled.
              const float mg2_disp = fabsf(q_curr - g.stall_anchor_q);
              float mg2_prog_min = MG2_STALL_PROG_FRAC * fabsf(expected_velocity_deg_s) *
                                   ((float)MG2_STALL_WINDOW_MS * 0.001f);
              if (mg2_prog_min < MG2_STALL_PROGRESS_MIN_DEG) {
                mg2_prog_min = MG2_STALL_PROGRESS_MIN_DEG;
              }
              g.stall_no_progress = (mg2_disp < mg2_prog_min);
              g.stall_anchor_q = q_curr;
              g.stall_anchor_ms = t_now;
            }
            const bool mg2_effort = (t_now - g.effort_clamp_ms) < MG2_STALL_EFFORT_FRESH_MS;
            const bool mg2_err_armed = abs_error > (mg2_effort ? MG2_STALL_EFFORT_ERR_DEG
                                                               : MG2_STALL_ERR_FLOOR_DEG);
            if (g.stall_no_progress && mg2_err_armed) {
              // Corridor-railed no-progress = the cascade is grinding into an obstruction:
              // integrate x3 so a hard block aborts at ~300ms (legacy parity) instead of
              // 900ms of elastic wind-up (review F1 / TP-3).
              g.i_stall_ms += (mg2_effort ? MG2_EFFORT_ISTALL_GAIN : 1.0f) * (float)mg2_sdt;
              if (g.i_stall_ms > 4000.0f) g.i_stall_ms = 4000.0f;
            } else {
              g.i_stall_ms -= 2.0f * (float)mg2_sdt;   // fast leak on any progress
              if (g.i_stall_ms < 0.0f) g.i_stall_ms = 0.0f;
            }
            // ERR_CEIL rung: a fully blocked foot at sustained extreme error must not wind
            // the cascade for the whole integrator horizon (blocked-foot backstop).
            if (abs_error >= MG2_ERR_CEIL_DEG) {
              g.err_ceil_ms += (float)mg2_sdt;
            } else {
              g.err_ceil_ms = 0.0f;
            }
            if (g.i_stall_ms >= MG2_STALL_WARN_MS && t_now - g.last_stall_evt_ms >= 500u) {
              g.last_stall_evt_ms = t_now;
              char f1[48], f2[48];
              LOG_C1_WARN_F("[MG2] DOF %d stall ride warn istall=%sms disp=%sdeg", dof,
                            c1f(f1, g.i_stall_ms, 0),
                            c1f(f2, fabsf(q_curr - g.stall_anchor_q), 2));
              if (motion_guard_mode == 2) {
                SERIAL_C1_COM_LN_F("EVT:STALL_RIDE:DOF=%d", dof);
              }
            }
            if (g.err_ceil_ms >= MG2_ERR_CEIL_TRIP_MS) {
              mg2_stall_abort = true;
              mg2_stall_kind = "err-ceiling";
            } else if (g.i_stall_ms >= MG2_STALL_ABORT_MS) {
              mg2_stall_abort = true;
              mg2_stall_kind = "no-progress";
            }
            if (mg2_stall_abort && motion_guard_mode == 1) {
              if (t_now - g.last_sabort_evt_ms >= MG2_EVT_PERIOD_MS) {
                g.last_sabort_evt_ms = t_now;
                char f1[48];
                LOG_C1_WARN_F("[MG_SHADOW] DOF %d would-STALL_ABORT (%s) istall=%sms", dof,
                              mg2_stall_kind, c1f(f1, g.i_stall_ms, 0));
              }
              g.i_stall_ms = 0.0f;       // state-faithful to ACTIVE post-abort reset
              g.err_ceil_ms = 0.0f;
              g.stall_anchor_ms = 0;
              g.stall_no_progress = false;
            }
          }
          if (motion_guard_mode == 2) {
            cs.move_candidate_start_ms = 0;   // legacy hygiene while v2 owns (review)
            if (mg2_stall_abort) {
              {
                char f1[48];   // record WHICH rung fired (the generic compliance abort
                               // below doesn't know) — ACTIVE forensics need it
                LOG_C1_WARN_F("[MG2] DOF %d STALL abort rung=%s istall=%sms", dof,
                              mg2_stall_kind, c1f(f1, mg2[dof].i_stall_ms, 0));
              }
              compliance_should_activate = true;   // reuse the proven abort action below
              mg2[dof].i_stall_ms = 0.0f;
              mg2[dof].err_ceil_ms = 0.0f;
              mg2[dof].stall_anchor_ms = 0;
              mg2[dof].stall_no_progress = false;
            }
          } else {
#endif
          float expected_speed = fabs(expected_velocity_deg_s);
          bool stalled = (abs_error > move_error_threshold_deg) &&
                         (fabs(velocity_filtered[dof]) < expected_speed * move_velocity_ratio);
          if (stalled) {
            if (cs.move_candidate_start_ms == 0) {
              cs.move_candidate_start_ms = t_now;
            }
            if (t_now - cs.move_candidate_start_ms >= move_time_threshold_ms) {
              compliance_should_activate = true;
#if MOTION_GUARD_V2
              if (motion_guard_mode == 1 && !mg2_stall_abort) {
                char f1[48];
                LOG_C1_WARN_F("[MG_SHADOW] DOF %d legacy STALL fires, v2 would NOT (istall=%sms)",
                              dof, c1f(f1, mg2[dof].i_stall_ms, 0));
              }
#endif
            }
          } else {
            cs.move_candidate_start_ms = 0;
          }
#if MOTION_GUARD_V2
          }
#endif
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
          {
            char f1[48], f2[48], f3[48];
            LOG_C1_INFO_F("[Compliance] DOF %d ACTIVE mode=%s err=%sdeg exp_v=%sdeg/s act_v=%sdeg/s",
                          dof, mode_label, c1f(f1, error, 2), c1f(f2, expected_velocity_deg_s, 2),
                          c1f(f3, velocity_filtered[dof], 2));
          }

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

            char fb[48];
            LOG_C1_INFO_F("[Compliance] DOF %d STALL->HOLDING: trajectory aborted, holding at %sdeg",
                          dof, c1f(fb, q_curr, 2));

            // Notify host that trajectory was aborted due to stall
            // Host should stop sending targets and handle the situation
            SERIAL_C1_COM_LN_F("EVT:STALL_ABORT:DOF=%d:ANGLE=%s", dof, c1f(fb, q_curr, 2));
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
            const char *reason = release_by_error && release_by_velocity ? "HOLD_ERR+VEL"
                                 : (release_by_velocity ? "HOLD_VEL" : "HOLD_ERR");
            char f1[48], f2[48], f3[48];
            LOG_C1_INFO_F("[Compliance] DOF %d RELEASE reason=%s err=%sdeg exp_v=%sdeg/s act_v=%sdeg/s",
                          dof, reason, c1f(f1, error, 2), c1f(f2, expected_velocity_deg_s, 2),
                          c1f(f3, velocity_filtered[dof], 2));
          }
        } else {
          float expected_speed = fabs(expected_velocity_deg_s);
          bool release_by_error = (abs_error < move_error_threshold_deg);
          bool release_by_velocity = (fabs(velocity_filtered[dof]) >= expected_speed * move_velocity_ratio);
          if (release_by_error || release_by_velocity) {
            compliance_should_release = true;
            const char *reason = release_by_error && release_by_velocity ? "MOVE_ERR+VEL"
                                 : (release_by_velocity ? "MOVE_VEL" : "MOVE_ERR");
            char f1[48], f2[48], f3[48];
            LOG_C1_INFO_F("[Compliance] DOF %d RELEASE reason=%s err=%sdeg exp_v=%sdeg/s act_v=%sdeg/s",
                          dof, reason, c1f(f1, error, 2), c1f(f2, expected_velocity_deg_s, 2),
                          c1f(f3, velocity_filtered[dof], 2));
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

        // Check if oscillation is dangerous. In Loop2 the response is a TERMINAL
        // power-cut (the 0xA4 latch cannot be soft-stopped), so use a higher
        // amplitude threshold there: start-of-move ringing at legitimate 40+ dps
        // strokes reached the Loop1 threshold and must not brick the run.
        const bool osc_loop2 = (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION) &&
                               impedance_active;
        const float osc_threshold = osc_loop2 ? LOOP2_OSC_MIN_AMPLITUDE_DEG
                                              : OSC_MIN_AMPLITUDE_DEG;
        if (!od.oscillation_detected &&
            od.sign_change_count >= OSC_MIN_SIGN_CHANGES &&
            osc_amplitude >= osc_threshold) {
          
          // Folded to a single heap-free push (v2 String pass 2026-07-06): the 3-line String
          // banner heap-built in the stop chain; all fields preserved in order.
          {
            // Two pushes (review round: a single fold rendered 121-123 bytes > the 119-byte
            // core1 log cap — the threshold field was clipped and the cut could split the
            // UTF-8 degree sign). Split at the old line boundary keeps every field intact.
            char f1[48], f2[48];
            LOG_C1_ERROR_F("[OSCILLATION SAFETY] DOF %d DANGEROUS OSCILLATION DETECTED!"
                           "  Sign changes: %d in %lums",
                           dof, (int)od.sign_change_count,
                           (unsigned long)(now_ms - od.window_start_ms));
            LOG_C1_ERROR_F("  Amplitude: %s° (threshold: %s°)",
                           c1f(f1, osc_amplitude, 1), c1f(f2, OSC_MIN_AMPLITUDE_DEG, 1));
          }
          // Trigger emergency stop. In Loop2, a soft stop is not enough because the
          // motor-side 0xA4 position latch may keep holding after firmware skips torque.
          od.oscillation_detected = true;
          collectPendingPair();  // motor-CAN ops below (latch / stopAllMotors): drain in-flight pair first
          if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION && impedance_active) {
            latchTerminalMotionFault(dof, "LOOP2_OSCILLATION", this);
            return false;
          }
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
          
          return false; // Skip to next DOF
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
          // Heap-free build (2026-07-06): fires on every MOVING→HOLDING transition inside
          // the outer WP-PROF span. EVT: protocol format FROZEN (host parses it); c1f ==
          // String(float,2) byte-for-byte.
          LOG_C1_DEBUG_F("[Control] DOF %d transitioned MOVING → HOLDING", dof);

          // Send structured message for UI display
          {
            char fb[48];
            SERIAL_C1_COM_LN_F("EVT:HOLDING_TARGET:DOF=%d:ANGLE=%s", dof,
                               c1f(fb, holding_target, 2));
          }

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
      // - Always check joint limits in MOVING mode (every outer loop cycle = 250 Hz default)
      // - Check periodically in HOLDING mode (every 10 inner cycles ≈ 40ms at 250Hz/4ms)
      // NOTE: We do NOT check immediately when entering HOLDING because motors may still be settling
      // Reduced from 20 cycles (~80ms) to 10 cycles (~40ms) for faster detection during manual push
      bool should_check_safety = is_moving || (is_holding && safety_check_counter >= 10);
      
      if (should_check_safety) {
#if CONTROLLER_DEBUG
        uint32_t safety_start_us = time_us_32();
#endif
        // Heap-free (v2 pass): stack buffer — the String here was a malloc(1)+free through
        // the cross-core mutex EVERY moving outer cycle even with no violation.
        char safety_message[120];
        safety_message[0] = '\0';
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
            sizeof safety_message,
            check_motors,
            &safety_violation_type);
        if (!safety_ok && direct_drive_dof &&
            safety_violation_type == SAFETY_VIOLATION_MAPPING_LIMIT &&
            canDirectDriveRecoverTowardSafeRange(dof, q_curr, q_des)) {
          static uint32_t last_inward_recovery_log_ms[MAX_DOFS] = {0};
          if (t_now - last_inward_recovery_log_ms[dof] > 250) {
            float safe_min = 0.0f;
            float safe_max = 0.0f;
            getMappingSafeRange(dof, safe_min, safe_max);
            LOG_C1_WARN("[Safety] DOF " + String(dof) +
                        " outside conservative range [" + String(safe_min, 2) + ", " +
                        String(safe_max, 2) + "] at " + String(q_curr, 2) +
                        "° — allowing inward-only recovery toward " + String(q_des, 2) + "°");
            last_inward_recovery_log_ms[dof] = t_now;
          }
          safety_ok = true;
        }
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
          collectPendingPair();  // motor-CAN ops below (stop/latch): drain any in-flight pair first
          stopAllMotors();
          // Two pushes (v2 review): single-line prefix+message overran the 119-byte cap and
          // clipped the violation tail (motor safe-range numbers) — exactly the forensics
          // needed on a tendon-breakage stop. Marker text kept verbatim on line 1.
          LOG_C1_ERROR_F("[Safety] MOVEMENT STOPPED: (DOF %d)", dof);
          LOG_C1_ERROR_F("%s", safety_message);
          diag_note_safety_violation(dof, safety_violation_type);

          if (safety_violation_type == SAFETY_VIOLATION_MAPPING_LIMIT) {
            latchTerminalMotionFault(dof, "MAPPING_LIMIT", this);
            return false;
          }

          // === PERSISTENT MOTOR_RANGE ("possible tendon breakage") ESCALATION ===
          // A MOTOR_RANGE violation means a motor angle is outside its mapped safe
          // band while the joint encoder is fine — the classic broken/slack-tendon
          // signature. A single transient read still soft-stops (HOLDING) below, but
          // a SUSTAINED MOTOR_RANGE condition must NOT be allowed to re-drive: escalate
          // to the same latched hardware power-cut as the 0x000 host e-stop (handled at
          // the top of core1_loop: safety_motor_power_disable + diag_set_estop_latched,
          // requires PRETENSION recovery). We require MOTOR_RANGE_ESCALATE_CYCLES
          // CONSECUTIVE trips so a one-cycle glitch cannot latch power off.
          if (safety_violation_type == SAFETY_VIOLATION_MOTOR_RANGE) {
            if (motor_range_trip_streak[dof] < 0xFF) {
              motor_range_trip_streak[dof]++;
            }
            if (motor_range_trip_streak[dof] >= MOTOR_RANGE_ESCALATE_CYCLES) {
              LOG_C1_ERROR("[Safety] DOF " + String(dof) +
                           " PERSISTENT MOTOR_RANGE (" +
                           String(motor_range_trip_streak[dof]) +
                           " consecutive) — escalating to latched motor power-cut "
                           "(possible tendon breakage)");
              motor_range_trip_streak[dof] = 0;
              // Latch: handled by the e-stop block at the top of core1_loop.
              emergency_stop_requested = true;
              // Fall through to the soft-stop/HOLDING below as a belt-and-suspenders
              // stop for THIS cycle; the hardware latch lands on the next iteration.
            }
          }

          float hold_ref = dof_data.valid[dof] ? q_curr : getImpedanceHoldReference(dof);
          if (impedance_active) {
            clearImpedanceControlState(dof, this);
          }

          dof_hold_angle[dof] = hold_ref;
          dof_hold_time[dof] = t_now;
          dof_state[dof] = DofState::HOLDING;
          prev_dof_state[dof] = DofState::HOLDING;
          pid_reset_needed[dof] = true;

          // Other DOFs still get their turn (skip only this one this cycle)
          return false;
        }

        // Safety check passed this cycle: clear the MOTOR_RANGE escalation streak so
        // only CONSECUTIVE trips accumulate (a single transient never latches power off).
        motor_range_trip_streak[dof] = 0;
      }

      // === PASSIVE DIVERGENCE GUARD ===
#if MOTION_GUARD_V2
      // MG2-S v2 path (modes 1/2). Mode 1 SHADOW: compute + log would-actions only; the
      // legacy block below stays authoritative. Mode 2 ACTIVE: this path replaces legacy.
      // Encoder-invalid cycles never reach this point (per-DOF early return upstream), so
      // blindness appears here as a time GAP — handled by the gap rules below (no streak
      // laundering, no bulk crediting).
      if (motion_guard_mode != 0 && is_moving && !direct_drive_dof) {
        Mg2State &g = mg2[dof];
        const uint32_t mg2_raw_gap =
            (g.last_ms == 0 || t_now <= g.last_ms) ? 4u : (t_now - g.last_ms);
        const uint32_t mg2_dt = (mg2_raw_gap > MG2_GAP_TICK_MS) ? 4u : mg2_raw_gap;
        if (mg2_raw_gap > MG2_GAP_TICK_MS) {
          // Fresh observation after a gap (hold boundary / encoder blindness / mode
          // enable): nothing was observed meanwhile, so nothing may count as evidence.
          g.streak_ms = 0.0f;
          g.streak_samples = 0;
          g.limit_streak_ms = 0.0f;
          g.limit_streak_samples = 0;
          g.err_abs_prev = fabsf(error);
          for (uint8_t k = 0; k < 6; k++) g.err_hist_ms[k] = 0;  // stale-move ring entries
        }
        if (mg2_raw_gap > MG2_VREF_GAP_RESET_MS) {
          g.v_ref = 0.0f;   // fully decayed across long holds: the QS backstop must be
                            // LIVE for a slow move that follows a fast move (review F3)
        }
        g.last_ms = t_now;
        {
          const float mg2_abs_err = fabsf(error);
          const float mg2_v_exp = fabsf(expected_velocity_deg_s);
          float mg2_decay = 1.0f - (float)mg2_raw_gap / MG2_TAU_GATE_MS;  // wall-time decay
          if (mg2_decay < 0.0f) mg2_decay = 0.0f;
          g.v_ref *= mg2_decay;
          if (mg2_v_exp > g.v_ref) g.v_ref = mg2_v_exp;
          if (g.v_ref < MG2_VREF_FLOOR_DPS) g.v_ref = MG2_VREF_FLOOR_DPS;
          if (t_now - g.err_hist_ms[g.hist_idx] >= 16u) {
            g.hist_idx = (uint8_t)((g.hist_idx + 1u) % 6u);
            g.err_hist[g.hist_idx] = mg2_abs_err;
            g.err_hist_ms[g.hist_idx] = t_now;
          }
          float mg2_err_old = mg2_abs_err;
          uint32_t mg2_age_best = 0;
          for (uint8_t k = 0; k < 6; k++) {
            if (g.err_hist_ms[k] == 0) continue;
            uint32_t age = t_now - g.err_hist_ms[k];
            if (age >= 80u && age <= 220u && age > mg2_age_best) {
              mg2_age_best = age;
              mg2_err_old = g.err_hist[k];
            }
          }
          const bool mg2_growing = (mg2_age_best != 0)
                                       ? (mg2_abs_err > mg2_err_old + 0.10f)
                                       : (mg2_abs_err > g.err_abs_prev + 0.02f);
          g.err_abs_prev = mg2_abs_err;
          const float mg2_away = -(error > 0.0f ? 1.0f : -1.0f) * dof_data.velocities[dof];
          const bool mg2_qs = (g.v_ref <= MG2_QS_VREF_MAX_DPS);
          bool mg2_armed = false;
          if (mg2_abs_err >= MG2_MIN_ERR_DEG) {
            if (mg2_qs) {
              mg2_armed = (mg2_away >= MG2_GATE_FLOOR_DPS);   // legacy-parity backstop
            } else {
              float mg2_gate = MG2_GATE_K * g.v_ref;
              if (mg2_gate < MG2_GATE_FLOOR_DPS) mg2_gate = MG2_GATE_FLOOR_DPS;
              mg2_armed = (mg2_away >= mg2_gate) && mg2_growing;
            }
          }
          if (mg2_armed) {
            g.streak_ms += (float)mg2_dt;
            if (g.streak_samples < 0xFF) g.streak_samples++;
          } else if (mg2_abs_err >= MG2_MIN_ERR_DEG) {
            g.streak_ms -= (float)mg2_dt;   // leaky while error persists
            if (g.streak_ms < 0.0f) g.streak_ms = 0.0f;
            g.streak_samples = 0;
          } else {
            g.streak_ms = 0.0f;
            g.streak_samples = 0;
          }
          // L3d limit-approach fastpath: INDEPENDENT arm — a sub-gate runaway closing on a
          // mechanical stop never reaches the TRK gate (review), so this rung keeps its own
          // streak on {near limit, closing >=60dps, growing}.
          bool mg2_limit_closing = false;
          {
            float mg2_lo = 0.0f, mg2_hi = 0.0f;
            if (getMappingSafeRange(dof, mg2_lo, mg2_hi)) {
              const float mg2_v = dof_data.velocities[dof];
              mg2_limit_closing =
                  ((mg2_hi - q_curr) <= MG2_LIMIT_DIST_DEG && mg2_v >= MG2_LIMIT_VEL_DPS) ||
                  ((q_curr - mg2_lo) <= MG2_LIMIT_DIST_DEG && mg2_v <= -MG2_LIMIT_VEL_DPS);
            }
          }
          if (mg2_limit_closing && mg2_growing) {
            g.limit_streak_ms += (float)mg2_dt;
            if (g.limit_streak_samples < 0xFF) g.limit_streak_samples++;
          } else {
            g.limit_streak_ms = 0.0f;
            g.limit_streak_samples = 0;
          }
          bool mg2_terminal = false;
          bool mg2_limit_trip = false;
          const char *mg2_reason = nullptr;
          if (g.limit_streak_ms >= MG2_LIMIT_TRIP_MS && g.limit_streak_samples >= 2) {
            mg2_terminal = true;
            mg2_limit_trip = true;
            mg2_reason = "MG2 limit approach runaway";
          } else if (mg2_qs) {
            if (g.streak_ms >= MG2_QS_TRIP_MS && g.streak_samples >= 2) {
              mg2_terminal = true;
              mg2_reason = "MG2 QS divergence";
            }
          } else {
            if (mg2_away >= MG2_L3A_AWAY_DPS && mg2_abs_err >= MG2_L3A_ERR_DEG && mg2_growing &&
                g.streak_ms >= MG2_L3A_TRIP_MS && g.streak_samples >= 2) {
              mg2_terminal = true;
              mg2_reason = "MG2 fast runaway";
            }
            if (!mg2_terminal && mg2_armed && g.last_soft_ms != 0 &&
                (t_now - g.last_soft_ms) <= MG2_L3B_REARM_MS && g.streak_samples >= 2) {
              mg2_terminal = true;
              mg2_reason = "MG2 re-divergence after soft-hold";
            }
          }
          // Post-soft brake grace (review F2): after a soft-hold freezes the reference at
          // q_curr, a HEALTHY plant still carrying momentum shows growing error AND
          // away>=gate while it brakes out (motor pole ~25ms) — indistinguishable from a
          // runaway by error/away alone. Discriminator: a braking plant SHEDS away-speed;
          // a still-powered runaway keeps or exceeds its at-soft speed. The limit fastpath
          // is exempt (imminent hard-stop impact: cut power regardless).
          if (mg2_terminal && !mg2_limit_trip && g.last_soft_ms != 0 &&
              (t_now - g.last_soft_ms) < MG2_SOFT_GRACE_MS &&
              mg2_away <= g.away_at_soft + MG2_GRACE_AWAY_MARGIN_DPS) {
            mg2_terminal = false;
            mg2_reason = nullptr;
            g.streak_ms = 0.0f;      // re-observe: brake-out evidence must not pool into
            g.streak_samples = 0;    // a trip the instant the grace window closes
          }
          bool mg2_soft = (!mg2_terminal && !mg2_qs &&
                           g.streak_ms >= MG2_SOFT_TRIP_MS && g.streak_samples >= 2);
          if (mg2_soft) {   // L3c: repeated softs in the window escalate (not grace-gated:
                            // three softs in 2s IS the confirmed-runaway pattern)
            if (g.soft_window_start_ms == 0 ||
                t_now - g.soft_window_start_ms > MG2_L3C_WINDOW_MS) {
              g.soft_window_start_ms = t_now;
              g.soft_count = 0;
            }
            if ((uint8_t)(g.soft_count + 1u) >= MG2_L3C_SOFT_MAX) {
              mg2_terminal = true;
              mg2_reason = "MG2 repeated soft-holds";
              mg2_soft = false;
            }
          }
          if (mg2_terminal) {
            if (motion_guard_mode == 2) {
              collectPendingPair();
              stopAllMotors();
              {
                char f1[48], f2[48];
                LOG_C1_ERROR_F("[MG2] DOF %d TERMINAL: %s err=%s away=%s", dof, mg2_reason,
                               c1f(f1, error, 2), c1f(f2, mg2_away, 1));
              }
              diag_note_safety_violation(dof, SAFETY_VIOLATION_LIMIT);
              latchTerminalMotionFault(dof, mg2_reason, this);
              mg2ResetDof(dof);
              return false;
            }
            // SHADOW: state-faithful to ACTIVE — a terminal wipes the guard state; only
            // the log is rate-limited (review: bookkeeping must not live inside the gate).
            if (t_now - g.last_evt_ms >= MG2_EVT_PERIOD_MS) {
              g.last_evt_ms = t_now;
              char f1[48], f2[48];
              LOG_C1_WARN_F("[MG_SHADOW] DOF %d would-TERMINAL: %s err=%s away=%s", dof,
                            mg2_reason, c1f(f1, error, 2), c1f(f2, mg2_away, 1));
            }
            const uint32_t mg2_keep_evt = g.last_evt_ms;
            mg2ResetDof(dof);
            mg2[dof].last_evt_ms = mg2_keep_evt;
          } else if (mg2_soft) {
            if (motion_guard_mode == 2) {
              // L2 SOFT-HOLD: freeze the trajectory at the current pose — the proven
              // stall-abort action INCLUDING its same-cycle outer-PID rebase (review:
              // freezing the reference while keeping the wound accumulator lets the
              // movement bias keep driving the joint through the freeze). Joint stays
              // powered and held; host notified; escalation to terminal only via the
              // confirmed-runaway paths (grace discriminator above).
              if (metrics_tracking_enabled && dof < 3 && metrics_tracker[dof].tracking_active) {
                metrics_tracker[dof].aborted_by_stall = true;
                metrics_tracker[dof].abort_target_deg = q_des;
              }
              ImpedanceRollingSegment &seg = impedance_segment[dof];
              seg.q_goal_deg = q_curr; seg.q_start_deg = q_curr; seg.q_ref_deg = q_curr;
              seg.dq_ref_deg_s = 0.0f; seg.speed_abs_deg_s = 0.0f;
              seg.t_start_ms = t_now; seg.t_arrival_ms = t_now;
              seg.active = false; seg.initialized = true;
              impedance_target[dof].q_target_deg = q_curr;
              impedance_target[dof].dq_target_deg_s = 0.0f;
              PID *mg2_outer = getOuterPID(dof);
              if (mg2_outer) {
                mg2_outer->initializeState(q_curr, q_curr, 0.0f);  // discard movement bias,
              }                                                    // bumpless (legacy rebase)
              delta_theta[dof] = 0.0f;
              delta_theta_prev[dof] = 0.0f;
              dof_hold_angle[dof] = q_curr;
              dof_hold_time[dof] = t_now;
              dof_state[dof] = DofState::HOLDING;
              g.last_soft_ms = t_now;
              g.away_at_soft = mg2_away;
              g.soft_count++;
              g.streak_ms = 0.0f;
              g.streak_samples = 0;
              g.v_ref = MG2_VREF_FLOOR_DPS;  // post-soft re-arm runs through the QS
                                             // backstop; the brake-grace discriminator
                                             // separates coasting from powered runaway
              {
                char f1[48];
                LOG_C1_WARN_F("[MG2] DOF %d SOFT-HOLD at %sdeg (divergence ride-through)",
                              dof, c1f(f1, q_curr, 2));
              }
              SERIAL_C1_COM_LN_F("EVT:DIV_SOFT:DOF=%d", dof);
              diag_note_safety_violation(dof, SAFETY_VIOLATION_LIMIT);
              return false;
            }
            // SHADOW: same state trajectory as ACTIVE (floor v_ref, count softs, arm the
            // grace reference) so the campaign observes true post-soft re-arm dynamics;
            // no EVT on the COM stream in a logs-only mode.
            g.last_soft_ms = t_now;
            g.away_at_soft = mg2_away;
            g.soft_count++;
            g.streak_ms = 0.0f;
            g.streak_samples = 0;
            g.v_ref = MG2_VREF_FLOOR_DPS;
            if (t_now - g.last_evt_ms >= MG2_EVT_PERIOD_MS) {
              g.last_evt_ms = t_now;
              char f1[48], f2[48], f3[48];
              LOG_C1_WARN_F("[MG_SHADOW] DOF %d would-SOFT err=%s away=%s vref=%s", dof,
                            c1f(f1, error, 2), c1f(f2, mg2_away, 1), c1f(f3, g.v_ref, 0));
            }
          }
        }
      }
            // Legacy guard: authoritative in modes 0 and 1 (SHADOW); skipped in ACTIVE.
      if (motion_guard_mode != 2) {
#endif
      // Catch the failure class seen with inner_kd=0 before the joint reaches the
      // terminal mapping limit: substantial tracking error plus high velocity in the
      // direction that INCREASES that error. This avoids the old hand-vibration false
      // trips by requiring both large error and very high away-speed for consecutive
      // cycles.
      if (is_moving && !direct_drive_dof && dof_data.valid[dof] &&
          fabsf(error) >= DIVERGENCE_MIN_ERROR_DEG) {
        const float away_speed = -(error > 0.0f ? 1.0f : -1.0f) * dof_data.velocities[dof];
        if (away_speed >= DIVERGENCE_AWAY_SPEED_DEG_S) {
          if (divergence_guard_streak[dof] < 0xFF) {
            divergence_guard_streak[dof]++;
          }
        } else if (divergence_guard_streak[dof] > 0) {
          divergence_guard_streak[dof]--;
        }

        if (divergence_guard_streak[dof] >= DIVERGENCE_STOP_CYCLES) {
          collectPendingPair();  // motor-CAN ops below (stop/latch): drain any in-flight pair first
          stopAllMotors();
          LOG_C1_ERROR("[PASSIVE_GUARD] DOF " + String(dof) +
                       " divergence soft-stop: err=" + String(error, 2) +
                       " vel=" + String(dof_data.velocities[dof], 1) +
                       " away=" + String(away_speed, 1));
          diag_note_safety_violation(dof, SAFETY_VIOLATION_LIMIT);
          latchTerminalMotionFault(dof, "PASSIVE_GUARD divergence", this);
          divergence_guard_streak[dof] = 0;
          return false;
        }
      } else {
        divergence_guard_streak[dof] = 0;
      }
#if MOTION_GUARD_V2
      } else {
        divergence_guard_streak[dof] = 0;   // keep legacy state clean while v2 owns (ACTIVE)
      }
#endif
      
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

      // === UPDATE INNER LOOP SAMPLING PERIOD (same on-change pattern) ===
      // The inner PIDs are CONSTRUCTED with the preset motion.sampling_period (3000 us
      // on the tendon presets), which diverges from the real runtime period (4000 us
      // boot default): Ki*Ts ran ~25% low and Kd/Ts ~33% high vs the commanded gains.
      // This sync makes the host-commanded inner gains act on the REAL loop period
      // (first sync at boot, then on every 0x006 rate change).
      // NOTE for bench baselines: prior tuning absorbed the 3000-vs-4000 mismatch, so
      // the effective inner Ki/Kd shift when this first lands - re-baseline before A/Bs.
      static float last_inner_loop_dt = 0.0f;
      float inner_loop_dt = inner_loop_period_us / 1000000.0f;
      if (inner_loop_dt != last_inner_loop_dt) {
        setInnerLoopSamplingPeriod(inner_loop_dt);
        last_inner_loop_dt = inner_loop_dt;
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

        // Recorder ring (black box): sample the signals that drive the torque — joint angle,
        // outer output (dth), and the integral term — downsampled (~15 Hz at 250 Hz inner loop).
        static uint16_t diag_rec_div = 0;
        if (dof == 0) diag_rec_div++;
        if ((diag_rec_div % 16) == 0) {
          diag_record_control_sample(dof, q_curr, delta_theta[dof], outer_pid->last_ui, 0);
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
    c.agonist = agonist;        // fire/drain phases address this DOF's pair via the ctx
    c.antagonist = antagonist;
    PID *pid_agonist = cached_pid_agonist[dof];
    PID *pid_antagonist = cached_pid_antagonist[dof];
    int agonist_idx = cached_agonist_idx[dof];
    int antagonist_idx = cached_antagonist_idx[dof];

    // Get current position reference for inner loop theta_0 calculation
    float &theta_0_joint = c.theta_0_joint;  // crosses into drainConsumeDof (qDes diagnostics)

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
        return false;
      }

      if (!dof_data.valid[dof]) {
        return false;
      }

      float q_direct_curr = dof_data.angles[dof];
      float q_direct_ref = theta_0_joint;

      if (inner_pid_init_needed[dof] || inner_pid_reinit_after_impedance[dof]) {
        pid_direct->initializeState(q_direct_curr, q_direct_ref, 0.0f);
        inner_pid_init_needed[dof] = false;
        inner_pid_reinit_after_impedance[dof] = false;
      }

      float uff_direct = 0.0f;
      if (friction_ff_enabled) {
        // GMS break-away bristle FF (same model as the tendon path; see updateGmsFriction). On a
        // direct-drive DOF this is untested (the bristle was identified on the DOF1 tendon path) but it
        // is gated by friction_ff_enabled, which is OFF unless explicitly armed for a bench test.
        float Ts = gms_dt_s;  // measured inner-loop dt (v2 timing fix), not the configured period
        uff_direct = updateGmsFriction(dof, velocity_filtered[dof], expected_velocity_cache[dof],
                                       gmsFsForDirection(expected_velocity_cache[dof]),
                                       friction_ff_speed_thresh, Ts);
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

      float command_A = pid_direct->control(q_direct_ref, q_direct_curr, uff_direct, 1.0f, false,
                                            pid_measured_dt_enabled ? gms_dt_s : 0.0f);
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
      // (prev_command_direct is file-scope so IDLE/e-stop resets can zero it)
      command_A = constrain(command_A, -max_torque_direct, max_torque_direct);
      if (torque_ramp_time_ms > 0) {
        float rate = max_torque_direct * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);
        command_A = constrain(command_A,
                              prev_command_direct[dof] - rate,
                              prev_command_direct[dof] + rate);
      }
      prev_command_direct[dof] = command_A;

      collectPendingPair();  // direct-drive fires on the same motor CAN: drain any in-flight pair first
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
      return false;  // direct-drive fired inline above — nothing left for the pair fire/drain phases
    }

    if (agonist == nullptr || antagonist == nullptr) {
      // No motors for this DOF, skip
      return false;
    }
    
    // Compute theta_0 for motors using linear equations
    float &theta_0_agonist_motor = c.theta_0_agonist_motor;      // cross into fireDof (Loop2 spread clamp)
    float &theta_0_antagonist_motor = c.theta_0_antagonist_motor;
#if CONTROLLER_DEBUG
    uint32_t eq_start_us = time_us_32();
#endif
    // Live q0 (DOF0) from this cycle's sequence-locked snapshot for the bilinear DOF1 coupling.
    // If DOF0's reading is invalid this cycle, pass q0_nominal (center slice) so a dropped
    // encoder degrades to the 1D slice, never NaN. Ignored unless map_mode == MAP_BILINEAR.
    float q0_live = dof_data.valid[Q0_DOF] ? dof_data.angles[Q0_DOF]
                                           : linear_equations[dof].q0_nominal;
    bool equations_ok = calculateMotorAnglesWithEquations(dof, theta_0_joint, theta_0_joint,
                                                          theta_0_agonist_motor, theta_0_antagonist_motor,
                                                          q0_live);

    // === CASCADE SLOPE SCALING (opt-in knob, default OFF = legacy formula) ===
    // The cascade adds dth (JOINT degrees) unscaled 0.5/0.5 to MOTOR-degree refs; where the
    // local map slope S(q) varies (and differs per tendon) the joint-space loop gain runs
    // ~1/S(q) with a direction-flipping tension leak ~0.5*|1-S_B/S_A| (review P1, 2026-07-02;
    // offline analysis: matters at the RANGE EXTREMES, not mid-range). When enabled, scale the
    // per-tendon dth term by S_x/S_bar from a finite-difference eval of the SAME map.
    float slope_ratio_A = 1.0f;
    float slope_ratio_B = 1.0f;
    if (cascade_slope_scaling_enabled && equations_ok) {
      const float SLOPE_FD_DELTA_DEG = 0.5f;
      float a_lo, b_lo, a_hi, b_hi;
      if (calculateMotorAnglesWithEquations(dof, theta_0_joint - SLOPE_FD_DELTA_DEG,
                                            theta_0_joint - SLOPE_FD_DELTA_DEG, a_lo, b_lo, q0_live) &&
          calculateMotorAnglesWithEquations(dof, theta_0_joint + SLOPE_FD_DELTA_DEG,
                                            theta_0_joint + SLOPE_FD_DELTA_DEG, a_hi, b_hi, q0_live)) {
        const float s_a = (a_hi - a_lo) / (2.0f * SLOPE_FD_DELTA_DEG);
        const float s_b = (b_hi - b_lo) / (2.0f * SLOPE_FD_DELTA_DEG);
        const float s_bar = 0.5f * (s_a + s_b);
        // Sanity: finite, same-sign slopes of plausible magnitude; otherwise stay at 1.0
        // (legacy). Ratio authority is clamped to [0.5, 2.0] so a bad map segment cannot
        // more than halve/double the per-tendon dth term.
        if (isfinite(s_a) && isfinite(s_b) && isfinite(s_bar) &&
            fabsf(s_bar) > 0.2f && (s_a * s_bar) > 0.0f && (s_b * s_bar) > 0.0f) {
          slope_ratio_A = constrain(s_a / s_bar, 0.5f, 2.0f);
          slope_ratio_B = constrain(s_b / s_bar, 0.5f, 2.0f);
        }
      }
    }
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
      return false;
    }
    
    // Get cascade parameters
    float outer_kp, outer_ki, outer_kd;
    float &stiffness_ref = c.stiffness_ref;          // crosses into drainConsumeDof (slack/DIAG_HOLD)
    float &cascade_influence = c.cascade_influence;  // crosses into fireDof (Loop2 max spread)
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
    float &stiffness_ref_effective = c.stiffness_ref_effective;  // crosses into drainConsumeDof
    stiffness_ref_effective = stiffness_ref;
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

    if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION &&
        stiffness_ref_effective < LOOP2_MIN_STIFFNESS_REF_DEG) {
      if (t_now - loop2_stiffness_floor_log_ms[dof] > 500) {
        char f1[48], f2[48];
        LOG_C1_WARN_F("[LOOP2] DOF %d stiffness floor %s->%s", dof,
                      c1f(f1, stiffness_ref_effective, 1),
                      c1f(f2, LOOP2_MIN_STIFFNESS_REF_DEG, 1));
        loop2_stiffness_floor_log_ms[dof] = t_now;
      }
      stiffness_ref_effective = LOOP2_MIN_STIFFNESS_REF_DEG;
    }
    if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION &&
        stiffness_ref_effective > LOOP2_MAX_STIFFNESS_REF_DEG) {
      if (t_now - loop2_stiffness_ceiling_log_ms[dof] > 500) {
        char f1[48], f2[48];
        LOG_C1_WARN_F("[LOOP2] DOF %d stiffness ceiling %s->%s", dof,
                      c1f(f1, stiffness_ref_effective, 1),
                      c1f(f2, LOOP2_MAX_STIFFNESS_REF_DEG, 1));
        loop2_stiffness_ceiling_log_ms[dof] = t_now;
      }
      stiffness_ref_effective = LOOP2_MAX_STIFFNESS_REF_DEG;
    }
    
    // === DELTA_THETA INTERPOLATION ===
    // When outer_loop_divisor > 1, delta_theta updates every N cycles creating "steps".
    // To avoid vibrations from discontinuous reference changes, we linearly interpolate
    // between the previous and current delta_theta values based on where we are in the
    // outer loop period. When divisor = 1, alpha = 1.0 so no interpolation occurs.
    float &delta_theta_smooth = c.delta_theta_smooth;  // crosses into drainConsumeDof (HOLD_EVT/hi-rate)
    if (effective_divisor <= 1) {
      // No interpolation needed (outer and inner at same frequency)
      delta_theta_smooth = delta_theta[dof];
    } else {
      // Interpolate: cycle_in_outer goes from 0 to (divisor-1)
      // alpha goes from 1/divisor to 1.0 (we use new value immediately, blend out old)
      // Wrap-safe phase (was (cycle_count - 1) % divisor: int -1 at the %1000 wrap
      // cycle -> alpha = 0 -> one-cycle revert of delta_theta_smooth to the previous
      // outer output — a periodic kick present only at divisor > 1).
      int cycle_in_outer = (int)cycle_in_outer_phase;
      float alpha = (float)(cycle_in_outer + 1) / (float)effective_divisor;
      delta_theta_smooth = delta_theta_prev[dof] + alpha * (delta_theta[dof] - delta_theta_prev[dof]);
    }

    // === MAP-CORRIDOR GOVERNOR ===
    // Bound the *commanded* tendon displacement from the neutral map before the hard
    // per-motor safe-band clamp below. This preserves the antagonist geometry by
    // limiting delta_theta symmetrically, instead of independently clipping A/B refs.
    if (hasValidEquations(dof) && cascade_influence >= MAP_CORRIDOR_MIN_CASCADE_INFLUENCE) {
      float dth_min = 0.0f;
      float dth_max = 0.0f;
      bool corridor_ok = computeDeltaThetaCorridor(linear_equations[dof],
                                                   theta_0_agonist_motor,
                                                   theta_0_antagonist_motor,
                                                   stiffness_ref_effective,
                                                   cascade_influence,
                                                   MAP_CORRIDOR_SOFT_MARGIN_DEG,
                                                   dth_min,
                                                   dth_max,
                                                   slope_ratio_A, slope_ratio_B);
      bool soft_corridor = corridor_ok;
      if (!corridor_ok) {
        corridor_ok = computeDeltaThetaCorridor(linear_equations[dof],
                                                theta_0_agonist_motor,
                                                theta_0_antagonist_motor,
                                                stiffness_ref_effective,
                                                cascade_influence,
                                                0.0f,
                                                dth_min,
                                                dth_max,
                                                slope_ratio_A, slope_ratio_B);
      }

      bool stiffness_derated = false;
      float stiffness_derate_scale = 1.0f;
      float stiffness_before_corridor = stiffness_ref_effective;
      if (!corridor_ok) {
        const float floor_stiffness = fminf(stiffness_before_corridor,
                                            MAP_CORRIDOR_MIN_DERATED_STIFFNESS_DEG);
        const float scales[] = {0.75f, 0.50f, 0.30f};
        for (float scale : scales) {
          float candidate_stiffness = fmaxf(stiffness_before_corridor * scale,
                                            floor_stiffness);
          if (computeDeltaThetaCorridor(linear_equations[dof],
                                        theta_0_agonist_motor,
                                        theta_0_antagonist_motor,
                                        candidate_stiffness,
                                        cascade_influence,
                                        0.0f,
                                        dth_min,
                                        dth_max,
                                        slope_ratio_A, slope_ratio_B)) {
            stiffness_ref_effective = candidate_stiffness;
            stiffness_derated = true;
            stiffness_derate_scale = scale;
            corridor_ok = true;
            soft_corridor = false;
            break;
          }
        }
        if (!corridor_ok && floor_stiffness < stiffness_before_corridor) {
          float candidate_stiffness = floor_stiffness;
          if (computeDeltaThetaCorridor(linear_equations[dof],
                                        theta_0_agonist_motor,
                                        theta_0_antagonist_motor,
                                        candidate_stiffness,
                                        cascade_influence,
                                        0.0f,
                                        dth_min,
                                        dth_max,
                                        slope_ratio_A, slope_ratio_B)) {
            stiffness_ref_effective = candidate_stiffness;
            stiffness_derated = true;
            stiffness_derate_scale = candidate_stiffness / stiffness_before_corridor;
            corridor_ok = true;
            soft_corridor = false;
          }
        }
        if (!corridor_ok && t_now - map_corridor_last_log_ms[dof] >= MAP_CORRIDOR_LOG_PERIOD_MS) {
          char f1[48], f2[48], f3[48];
          LOG_C1_WARN_F("[MAP_CORRIDOR_DERATE] DOF %d no fit above stiffness floor=%s stiff=%s cascade=%s",
                        dof, c1f(f1, floor_stiffness, 1), c1f(f2, stiffness_before_corridor, 1),
                        c1f(f3, cascade_influence, 4));
          map_corridor_last_log_ms[dof] = t_now;
        }
      }

      if (corridor_ok) {
        float previous_derate_scale = map_corridor_last_derate_scale[dof];
        if (previous_derate_scale <= 0.0f) previous_derate_scale = 1.0f;
        if (fabsf(stiffness_derate_scale - previous_derate_scale) > 0.001f) {
          if (stiffness_derate_scale < 1.0f && map_corridor_derate_count[dof] < 0xFFFF) {
            map_corridor_derate_count[dof]++;
          }
          {
            char ln[120], f1[48], f2[48];
            int off = 0;
            c1cat(ln, sizeof ln, &off, "[MAP_CORRIDOR_DERATE] DOF %d scale=%s->%s", dof,
                  c1f(f1, previous_derate_scale, 2), c1f(f2, stiffness_derate_scale, 2));
            c1cat(ln, sizeof ln, &off, " stiff=%s->%s count=%d",
                  c1f(f1, stiffness_before_corridor, 1), c1f(f2, stiffness_ref_effective, 1),
                  (int)map_corridor_derate_count[dof]);
            LOG_C1_WARN_F("%s", ln);
          }
          map_corridor_last_derate_scale[dof] = stiffness_derate_scale;
        }

        const float dth_before_corridor = delta_theta_smooth;
        float dth_guarded = constrain(delta_theta_smooth, dth_min, dth_max);
        bool dth_clamped = fabsf(dth_guarded - dth_before_corridor) > 0.001f;
#if MOTION_GUARD_V2
        if (dth_clamped && motion_guard_mode != 0) {
          mg2[dof].effort_clamp_ms = t_now;   // MG2 stall effort branch (freshness-gated read)
        }
#endif
        bool near_corridor = dth_before_corridor <= (dth_min + MAP_CORRIDOR_NEAR_MARGIN_DEG) ||
                             dth_before_corridor >= (dth_max - MAP_CORRIDOR_NEAR_MARGIN_DEG);

        if (dth_clamped || near_corridor || stiffness_derated) {
          if (map_corridor_dwell[dof] < 0xFF) {
            map_corridor_dwell[dof]++;
          }
        } else if (map_corridor_dwell[dof] > 0) {
          map_corridor_dwell[dof]--;
        }

        if (dth_clamped) {
          delta_theta_smooth = dth_guarded;
          delta_theta[dof] = dth_guarded;
          delta_theta_prev[dof] = dth_guarded;
          // Sync the outer PID accumulator to the corridor limits. Overwriting delta_theta_*
          // alone leaves the PID's stored output (uprev) free to wind to its own +/-30 clamp
          // while the issued command is pinned at the (tighter) corridor edge — the same
          // stored-excess windup family the inner-PID back-calculation fix removed: after the
          // error reverses, the excess unwinds at Ki-only rate before the command can leave
          // the edge (seconds of continued pull toward the mechanical stop, worst at map ends).
          PID *outer_pid_corr = getOuterPID(dof);
          if (outer_pid_corr) {
            outer_pid_corr->clampAccumulator(dth_min, dth_max);
          }
        }

        if ((dth_clamped || stiffness_derated || map_corridor_dwell[dof] >= 20) &&
            t_now - map_corridor_last_log_ms[dof] >= MAP_CORRIDOR_LOG_PERIOD_MS) {
          {
            char ln[120], f1[48], f2[48];
            int off = 0;
            c1cat(ln, sizeof ln, &off, "[MAP_CORRIDOR] DOF %d dth=%s->%s", dof,
                  c1f(f1, dth_before_corridor, 2), c1f(f2, delta_theta_smooth, 2));
            c1cat(ln, sizeof ln, &off, " range=[%s,%s]",
                  c1f(f1, dth_min, 1), c1f(f2, dth_max, 1));
            c1cat(ln, sizeof ln, &off, " stiff=%s->%s",
                  c1f(f1, stiffness_before_corridor, 1), c1f(f2, stiffness_ref_effective, 1));
            c1cat(ln, sizeof ln, &off, "%s dwell=%d derates=%d",
                  soft_corridor ? " soft" : " hard", (int)map_corridor_dwell[dof],
                  (int)map_corridor_derate_count[dof]);
            LOG_C1_WARN_F("%s", ln);
          }
          map_corridor_last_log_ms[dof] = t_now;
        }
      }
    } else if (hasValidEquations(dof) && cascade_influence < MAP_CORRIDOR_MIN_CASCADE_INFLUENCE &&
               t_now - map_corridor_last_skip_log_ms[dof] >= 1000) {
      char f1[48], f2[48];
      LOG_C1_WARN_F("[MAP_CORRIDOR] DOF %d skipped: cascade_influence=%s < %s", dof,
                    c1f(f1, cascade_influence, 4),
                    c1f(f2, MAP_CORRIDOR_MIN_CASCADE_INFLUENCE, 4));
      map_corridor_last_skip_log_ms[dof] = t_now;
    }
    
    // Compute motor references using cascade control formula. slope_ratio_A/B are 1.0
    // unless cascade_slope_scaling_enabled (then S_x/S_bar from the live map, clamped).
    float &theta_A_ref = c.theta_A_ref;  // cross into fireDof (0xA4 target) / drainConsumeDof (HOLD_EVT)
    float &theta_B_ref = c.theta_B_ref;
    theta_A_ref = theta_0_agonist_motor +
                        cascade_influence * (0.5f * delta_theta_smooth * slope_ratio_A +
                                             0.5f * stiffness_ref_effective);
    theta_B_ref = theta_0_antagonist_motor +
                        cascade_influence * (0.5f * delta_theta_smooth * slope_ratio_B -
                                             0.5f * stiffness_ref_effective);

    // Record the co-contraction offset actually applied this cycle so fine-map capture can
    // subtract it (the cascade adds +off to the agonist ref / -off to the antagonist ref).
    // Stored as the NEUTRAL baseline → operating re-applies stiffness exactly once.
    if (dof < MAX_DOFS) {
      _cocontraction_offset[dof] = cascade_influence * 0.5f * stiffness_ref_effective;
    }

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
      // Live q0 (DOF0) for the bilinear DOF1 coupling (mirror the 1678/2563 sites); invalid DOF0 ->
      // q0_nominal (center slice), never NaN. Ignored unless map_mode == MAP_BILINEAR.
      float q0_slack = dof_data.valid[Q0_DOF] ? dof_data.angles[Q0_DOF]
                                              : linear_equations[dof].q0_nominal;
      if (calculateMotorAnglesWithEquations(dof, q_curr_inner, q_curr_inner, expected_A, expected_B,
                                            q0_slack)) {
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
            if (LOG_LEVEL >= 2) {
              char ln[120], f1[48], f2[48];
              int off = 0;
              c1cat(ln, sizeof ln, &off, "[AntiSlack] DOF %d Aref=%s Bref=%s", dof,
                    c1f(f1, theta_A_ref, 2), c1f(f2, theta_B_ref, 2));
              c1cat(ln, sizeof ln, &off, " expA=%s expB=%s",
                    c1f(f1, expected_A, 2), c1f(f2, expected_B, 2));
              LOG_C1_INFO_F("%s", ln);
            }
            last_anti_slack_log_ms[dof] = now_ms;
          }
        }
      }
    }

    // === PRE-SEND SAFE-BAND CLAMP OF COMMANDED MOTOR REFERENCE (ITEM 2) ===
    // Preventively clamp the commanded MOTOR-ANGLE references to the SAME per-tendon safe
    // band that the reactive MOTOR_RANGE check (checkMotorsInRange) compares the ACTUAL
    // motor angles against: linear_equations[dof].{agonist,antagonist}_safe_{min,max}
    // (map range ± MOTOR_SAFETY_MARGIN). A transient large delta_theta can briefly command
    // a ref past the agonist/antagonist safe range before the ~40 ms reactive check sees
    // it; clamping the SETPOINT here closes that open-loop excursion window.
    //
    // No-op in nominal operation: with the committed map the cascade refs stay inside the
    // band, so constrain() returns them unchanged — the inner PID setpoint is identical.
    // Gated on hasValidEquations() and a sane (min<max) band so a degenerate/zeroed map
    // (e.g. unmapped DOF) can never clamp a real command to 0 (such DOFs already cannot
    // reach the cascade — equations_ok is required above — this is belt-and-suspenders).
    if (hasValidEquations(dof)) {
      const float a_min = linear_equations[dof].agonist_safe_min;
      const float a_max = linear_equations[dof].agonist_safe_max;
      const float b_min = linear_equations[dof].antagonist_safe_min;
      const float b_max = linear_equations[dof].antagonist_safe_max;
      if (a_min < a_max) {
        theta_A_ref = constrain(theta_A_ref, a_min, a_max);
      }
      if (b_min < b_max) {
        theta_B_ref = constrain(theta_B_ref, b_min, b_max);
      }
    }

    // === PHASE 2: Motor angle acquisition ===
    // Bootstrap (first cycle): 0x92 provides absolute multi-turn angle for rev tracking init.
    // Normal cycles: use tracked angle from previous 0xA1 torque response (no CAN round-trip).
    // Watchdog (~1s): re-verify 0x92 to confirm tracking hasn't drifted.
    //
    // S2 SUBSTITUTE-FIRE (bit2): on a watchdog-DUE cycle, if bit2 is set AND bootstrap is done AND no
    // A1-miss recovery is pending, the DOF FIRES the 2-frame 0x92 pair INSTEAD of the torque pair — the
    // reply is carried and compared next cycle, motors hold the previous torque one cycle (ZOH). This maps
    // the watchdog-due condition ONLY to c.kind = PAIR_92 (which fireDof keys fire92() on), NOT to
    // need_0x92 (CRITICAL amendment): keeping need_0x92 FALSE makes compute take the tracked-angle branch
    // below and RUN IN FULL — outer PID, map, GMS, inner PID all advance at normal cadence. Setting
    // need_0x92 would wrongly skip PID/tracking and take the BLOCKING getMultiAnglePairPipelined read.
    // When bit2 is OFF, the watchdog-due condition sets need_0x92=true VERBATIM (the historical blocking
    // path) — bit-identical. The blocking 0x92 also stays for bootstrap + A1-miss recovery (rare) always.
    bool &recover_after_a1_miss = c.recover_after_a1_miss;  // cross into drainConsumeDof (re-anchor)
    bool &need_0x92 = c.need_0x92;
    recover_after_a1_miss = wp_a1_miss_count[dof] > 0;
    need_0x92 = !wp_0x92_bootstrap_done[dof] || recover_after_a1_miss;
    if (wp_0x92_bootstrap_done[dof] && !recover_after_a1_miss) {
      wp_watchdog_cycle_count[dof]++;
      if (wp_watchdog_cycle_count[dof] >= wp_watchdog_interval) {
        // Watchdog DUE. bit2 ON: substitute-fire (kind=PAIR_92, need_0x92 STAYS FALSE, compute runs full).
        // bit2 OFF: the historical blocking read (need_0x92=true) — statement-identical to the old tree.
        if (sched_sub92_enabled) {
          c.kind = PAIR_92;
        } else {
          need_0x92 = true;
        }
        wp_watchdog_cycle_count[dof] = 0;
      }
    }
    updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);

    PipelinedAngleData pipelined = {};
    MultiAngleData &data_A = c.data_A;  // 0x92 absolutes cross into drainConsumeDof (re-anchor)
    MultiAngleData &data_B = c.data_B;
    data_A.angle = NAN;
    data_B.angle = NAN;

#if CONTROLLER_DEBUG
    uint32_t can_start_us = time_us_32();
#endif
    if (need_0x92) {
      // 0x92 is a blocking round-trip on the motor CAN whose stale-RX flush would DROP an
      // in-flight pair's replies: drain the pending pair first (<=1 transaction outstanding).
      collectPendingPair();
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

    float &theta_A_curr = c.theta_A_curr;  // cross into fireDof (Loop2 latch seed)
    float &theta_B_curr = c.theta_B_curr;
    if (need_0x92) {
      // Bootstrap or watchdog cycle: 0x92 is source of truth
      theta_A_curr = data_A.angle;
      theta_B_curr = data_B.angle;
    } else {
      // Normal cycle: use tracked angle from 0xA1 (updated by setTorquePairPipelined)
      theta_A_curr = agonist->getTrackedAngle();
      theta_B_curr = antagonist->getTrackedAngle();
      if (isnan(theta_A_curr) || isnan(theta_B_curr)) {
        uint8_t reason = 0;
        if (isnan(theta_A_curr)) reason |= MOTOR_FEEDBACK_REASON_NAN_A;
        if (isnan(theta_B_curr)) reason |= MOTOR_FEEDBACK_REASON_NAN_B;
        diag_motor_feedback_snapshot[dof].invalid_reason = reason;
        updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
        diag_note_motor_feedback_invalid(dof, reason);
        if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION && impedance_active) {
          if (++loop2_feedback_fault_streak[dof] >= LOOP2_FEEDBACK_FAULT_STREAK_CYCLES) {
            collectPendingPair();  // latch stops ALL motors: drain any in-flight pair first
            latchTerminalMotionFault(dof, "LOOP2_MOTOR_FEEDBACK_NAN", this);
          }
          return false; // streak window: skip the cycle, motor holds its last valid latch
        }
        // Tracking not yet initialized — should not happen after bootstrap, skip DOF.
        return false;
      }
    }
    
    // === SANITY CHECK: Detect obviously invalid readings ===
    // Values outside ±100000° are clearly garbage (CAN corruption)
    bool invalid_A = (theta_A_curr < -100000.0f || theta_A_curr > 100000.0f || isnan(theta_A_curr));
    bool invalid_B = (theta_B_curr < -100000.0f || theta_B_curr > 100000.0f || isnan(theta_B_curr));
    
    if (invalid_A || invalid_B) {
      uint8_t reason = 0;
      if (isnan(theta_A_curr)) reason |= MOTOR_FEEDBACK_REASON_NAN_A;
      else if (invalid_A) reason |= MOTOR_FEEDBACK_REASON_RANGE_A;
      if (isnan(theta_B_curr)) reason |= MOTOR_FEEDBACK_REASON_NAN_B;
      else if (invalid_B) reason |= MOTOR_FEEDBACK_REASON_RANGE_B;
      diag_motor_feedback_snapshot[dof].invalid_reason = reason;
      updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
      diag_note_motor_feedback_invalid(dof, reason);
      if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION && impedance_active) {
        if (++loop2_feedback_fault_streak[dof] >= LOOP2_FEEDBACK_FAULT_STREAK_CYCLES) {
          collectPendingPair();  // latch stops ALL motors: drain any in-flight pair first
          latchTerminalMotionFault(dof, "LOOP2_MOTOR_FEEDBACK_RANGE", this);
        }
        return false; // streak window: skip the cycle, motor holds its last valid latch
      }
      static uint32_t last_invalid_log = 0;
      if (millis() - last_invalid_log > 100) { // Log max every 100ms
        LOG_C1_ERROR("[Control] DOF " + String(dof) + " INVALID CAN READ: A=" + 
                  String(theta_A_curr, 2) + " B=" + String(theta_B_curr, 2));
        last_invalid_log = millis();
      }
      // Skip this cycle entirely - don't send any torque command
      return false;
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
        diag_motor_feedback_snapshot[dof].invalid_reason = MOTOR_FEEDBACK_REASON_JUMP;
        updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
        diag_note_motor_feedback_invalid(dof, MOTOR_FEEDBACK_REASON_JUMP);

        LOG_C1_ERROR("[DIAG] DOF " + String(dof) + " MOTOR ANGLE JUMP (" +
                  String(recent_errors) + " errors in " + String(can_error_window_ms) + "ms)!");
        LOG_C1_ERROR("  Agonist: " + String(wp_last_theta_A[dof], 2) + " → " + String(theta_A_curr, 2) +
                  " (jump=" + String(jump_A, 2) + "°)");
        LOG_C1_ERROR("  Antagonist: " + String(wp_last_theta_B[dof], 2) + " → " + String(theta_B_curr, 2) +
                  " (jump=" + String(jump_B, 2) + "°)");

        if (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION && impedance_active) {
          if (++loop2_feedback_fault_streak[dof] >= LOOP2_FEEDBACK_FAULT_STREAK_CYCLES) {
            wp_canErrorTracker.clearErrors(dof);
            wp_first_read[dof] = true;
            collectPendingPair();  // latch stops ALL motors: drain any in-flight pair first
            latchTerminalMotionFault(dof, "LOOP2_MOTOR_FEEDBACK_JUMP", this);
          }
          return false; // streak window: skip the cycle, motor holds its last valid latch
        }

        // Trigger emergency stop if too many errors within time window
        if (wp_canErrorTracker.shouldStop(dof)) {
          LOG_C1_ERROR("[Control] DOF " + String(dof) + " - " + String(recent_errors) +
                    " CAN errors in " + String(can_error_window_ms) + "ms, EMERGENCY STOP!");
          collectPendingPair();  // motor-CAN op below (stopAllMotors): drain any in-flight pair first
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
          return false;
        }

        // Skip this cycle to avoid sending bad commands, use last known good values
        return false;
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
      // (chatter log removed, v2 String pass 2026-07-06)
      inner_pid_init_needed[dof] = false;
      inner_pid_reinit_after_impedance[dof] = false;
    }

    // === FRICTION FEEDFORWARD (now the GMS break-away bristle — see updateGmsFriction near the top) ===
    // NOTE the trapezoidal constant-FF description below is the OLD approach, retained as the cautionary
    // note on why it failed: it keyed off the COMMANDED velocity (constant through a stall), so it was a
    // DC bias that never broke the stick-slip. The live code now uses the measured-velocity bristle.
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
      // GMS break-away bristle FF (updateGmsFriction, defined near the top). v = RAW measured joint velocity (un-EMA, v2)
      // (the fix: the old constant FF used the COMMANDED velocity and never sensed the stall); vc =
      // commanded velocity; Ts = inner-loop period. Antisymmetric (uff_A=+, uff_B=-) -> zero net
      // co-contraction (never raises MEAN tendon tension); PEAK single-tendon tension is bounded by the
      // downstream per-motor torque saturation, NOT by this antisymmetry. friction_ff_torque is Fs.
      float Ts = gms_dt_s;  // measured inner-loop dt (v2 timing fix), not the configured period
      float tau_fric = updateGmsFriction(dof, dof_data.velocities[dof], expected_velocity_cache[dof],
                                         gmsFsForDirection(expected_velocity_cache[dof]),
                                         friction_ff_speed_thresh, Ts);
      uff_A = tau_fric;     // agonist: + break-away
      uff_B = -tau_fric;    // antagonist: - (tendon opposition)
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
    float &command_A = c.command_A;  // cross into fireDof (0xA1 torque) / drainConsumeDof (diag)
    float &command_B = c.command_B;

    // Root-fix A/B: feed the measured loop dt (gms_dt_s) to the PID time-scaled terms when enabled
    // (else 0 -> legacy fixed-Ts). Kills the derivative torque kick on an overrun-stretched cycle.
    float inner_dt_ovr = pid_measured_dt_enabled ? gms_dt_s : 0.0f;
    // Non-blocking-CAN bootstrap/recover: while rev-tracking is being (re-)anchored we DRIVE ZERO
    // torque for those ~2-3 cycles (decided policy — never a command against an un-anchored tracked
    // angle). In Loop1 the PID runs with its integrator frozen; in Loop2 the 0xA4 command is withheld
    // until rev-tracking is anchored.
    bool &zero_fire = c.zero_fire;      // crosses into fireDof (bootstrap/recover zero-torque pair)
    bool &loop2_active = c.loop2_active;  // crosses into fireDof/drainConsumeDof
    zero_fire = (!wp_rev_track_init[dof] || recover_after_a1_miss);
    loop2_active =
        (dof < MAX_DOFS) &&
        (inner_actuation_mode[dof] == INNER_ACTUATION_LOOP2_POSITION) &&
        impedance_active &&
        (config.dofs[dof].drive_type == DRIVE_ANTAGONISTIC_TENDON);

    if (loop2_active) {
      // Loop2 bypasses the firmware motor-torque PID and delegates each tendon to the motor's
      // internal 0xA4 position loop. Keep the torque PID accumulator out of the active path; core1
      // flags a bumpless reinit when the mode is changed back to Loop1.
      command_A = 0.0f;
      command_B = 0.0f;
    } else {
      if (loop2_target_valid[dof]) {
        resetLoop2ActuationState(dof);
      }
      if (impedance_active) {
        applyImpedanceInnerOverrides(dof, pid_agonist, pid_antagonist);
      }
      command_A = pid_agonist->control(theta_A_ref, theta_A_pid, uff_A, 1.0f, zero_fire, inner_dt_ovr);
      command_B = pid_antagonist->control(theta_B_ref, theta_B_pid, uff_B, 1.0f, zero_fire, inner_dt_ovr);
      if (zero_fire) { command_A = 0.0f; command_B = 0.0f; }
    }

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

      char f1[48], f2[48];
      LOG_C1_INFO_F("[Compliance] DOF %d torque_ratio %s -> %s ramp=%ums", dof,
                    c1f(f1, cs.torque_ratio_start, 2), c1f(f2, target_ratio, 2),
                    (unsigned)ramp_ms);
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
    // (prev_command_A/B are file-scope so IDLE/e-stop resets can zero them)

    // Step 1: Apply absolute torque limits FIRST
    command_A = constrain(command_A, -max_torque_A_effective, max_torque_A_effective);
    command_B = constrain(command_B, -max_torque_B_effective, max_torque_B_effective);

    // Step 2: Apply rate limiting on saturated values.
    // SKIPPED when zero_fire: the zero is AUTHORITATIVE (rev-tracking not anchored / 0xA1-miss
    // recovery must not drive), and rate-limiting from prev_command would re-inflate it back
    // toward the previous torque (the "designed zero" replay). Step 3 then stores 0, so any
    // post-recovery ramp restarts from zero instead of the stale pre-fault command.
    if (torque_ramp_time_ms > 0 && !zero_fire) {
      // Calculate max torque change per cycle
      // At 500 Hz with 100ms ramp time: 50 cycles to go 0→max
      // max_rate = max_torque / (ramp_time_ms / inner_loop_period_ms)
      //          = max_torque * inner_loop_period_us / (ramp_time_ms * 1000)
      float rate_A = max_torque_A_effective * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);
      float rate_B = max_torque_B_effective * inner_loop_period_us / (torque_ramp_time_ms * 1000.0f);

      command_A = constrain(command_A, prev_command_A[dof] - rate_A, prev_command_A[dof] + rate_A);
      command_B = constrain(command_B, prev_command_B[dof] - rate_B, prev_command_B[dof] + rate_B);
    }

    // Step 3: Store saturated+rate-limited value for next cycle.
    // S2 SUBSTITUTE-FIRE skip (bit2, CRITICAL amendment — this is the site where the advance actually is):
    // on a substitute cycle (kind==PAIR_92) NO torque frame is sent (the 0x92 pair is fired instead) and the
    // motors hold the previous 0xA1 torque (ZOH). The computed command is NOT wired to a motor, so it must
    // NOT become the rate limiter's ground truth — advancing prev_command here would let next cycle's ramp
    // start from a torque that was never applied, breaking the continuous-ramp invariant. Keep the ground
    // truth = the last FIRED torque. (compute still RAN in full — PID/GMS state advanced normally above;
    // only this rate-limiter history advance is skipped.) There is no separate wp_prev_torque advance in
    // this tree (it is reset-only in the fault/IDLE paths), so this is the sole guarded advance.
    if (c.kind != PAIR_92) {
      prev_command_A[dof] = command_A;
      prev_command_B[dof] = command_B;
    }
    
    // === DIAGNOSTIC: Log extreme torque commands ===
    if (fabs(command_A) >= max_torque_A_effective * 0.95f ||
        fabs(command_B) >= max_torque_B_effective * 0.95f) {
      static uint32_t last_torque_warn = 0;
      if (millis() - last_torque_warn > 500) { // Log max every 500ms
        // Folded to a single heap-free push (v2 String pass 2026-07-06): the 3-line String
        // trio heap-built on the hot path; all fields preserved in order.
        char ln[120], f1[48], f2[48], f3[48];
        int off = 0;
        c1cat(ln, sizeof ln, &off, "[DIAG] DOF %d HIGH TORQUE: A=%s B=%s (max=%s)", dof,
              c1f(f1, command_A, 0), c1f(f2, command_B, 0),
              c1f(f3, max_torque_A_effective, 0));
        c1cat(ln, sizeof ln, &off, "  refs: A=%s B=%s",
              c1f(f1, theta_A_ref, 2), c1f(f2, theta_B_ref, 2));
        c1cat(ln, sizeof ln, &off, "  curr: A=%s B=%s",
              c1f(f1, theta_A_curr, 2), c1f(f2, theta_B_curr, 2));
        LOG_C1_WARN_F("%s", ln);
        last_torque_warn = millis();
      }
    }
    
    return true;  // fully computed — the scheduler fires this DOF's pair next
  };

  // --- PHASE 2: fire this DOF's <=2-frame motor-CAN pair (0xA1 torque / 0xA4 Loop2) ---
  // Only called for a DOF whose computeDof returned true. The scheduler guarantees no other
  // pair is in flight when this runs (drain-before-fire), so firing 2 frames here can never
  // stack a third outstanding reply onto the MCP2515's 2 RX buffers.
  auto fireDof = [&](uint8_t dof) HOT_LAMBDA_ATTR("fireDof") {
    DofPhaseCtx &c = phase_ctx[dof];
    // MAJOR amendment (fire-time re-arm): re-arm the transaction state in the ctx AT FIRE TIME, as the
    // first statements. A snapshot must never carry a stale collected=true (a carried ctx with a leftover
    // true would skip every collect after the first carry). c.fire_us is set at the END of this lambda from
    // the post-fire fireTimestampUs() (the pair's single time authority; the scheduler tail reads the same
    // value into s2_carry.fire_us). c.kind is OWNED BY computeDof (PAIR_TORQUE default, or PAIR_92 on a bit2
    // substitute-fire cycle) — do NOT reset it here or the substitute-fire would be clobbered back to torque.
    c.collected = false;
    LKM_Motor *agonist = c.agonist;
    LKM_Motor *antagonist = c.antagonist;
    const bool zero_fire = c.zero_fire;
    const bool loop2_active = c.loop2_active;
    const float theta_0_agonist_motor = c.theta_0_agonist_motor;
    const float theta_0_antagonist_motor = c.theta_0_antagonist_motor;
    const float cascade_influence = c.cascade_influence;
    const float theta_A_ref = c.theta_A_ref;
    const float theta_B_ref = c.theta_B_ref;
    const float theta_A_curr = c.theta_A_curr;
    const float theta_B_curr = c.theta_B_curr;
    const float command_A = c.command_A;
    const float command_B = c.command_B;

    // Send inner actuation commands; the pair replies are collected in drainConsumeDof (in
    // the interleaved schedule possibly only after the next DOF's compute). Loop1 uses 0xA1
    // torque; Loop2 uses 0xA4 dual-position with the same collectPair reply discipline.
#if CONTROLLER_DEBUG
    c.torque_start_us = time_us_32();  // torque/diag micro-profile window closes in drainConsumeDof
#endif
    // === FIRE this DOF's pair; its <=2 replies are COLLECTED in drainConsumeDof (BURST-AWARE) ===
    // A burst of <=2 outstanding fits the MCP2515 2-RX / 3-TX buffers: fire BOTH motors (TX return is
    // checked inside fire* -> tx_fail), then poll-drain THIS pair's replies (short timeout). It is a
    // parallel pair round-trip (~1ms, both motors at once), well under the 4ms budget, and the buffer
    // never overflows. Bootstrap/recover always uses zero torque: never issue a 0xA4 position command
    // before rev-tracking has been anchored from 0x92 + encoder.
    if (c.kind == PAIR_92) {
      // S2 SUBSTITUTE-FIRE (bit2): fire the 2-frame 0x92 angle-read pair INSTEAD of the torque/position
      // pair — SAME <=2-frame bus footprint. The motors receive NO new actuation command, so they hold
      // their previous 0xA1 torque (Loop1) or 0xA4 latch (Loop2) for this one cycle (ZOH). The reply is
      // carried and consumed next cycle (drainConsumeDof, kind==PAIR_92 branch): |tracked - absolute| is
      // compared and the tracked angle advanced, exactly the drift-watchdog check — but non-blocking.
      agonist->fire92();
      antagonist->fire92();
    } else if (loop2_active && !zero_fire) {
      const float step_deg = constrain((float)loop2_target_step_x100[dof] * 0.01f,
                                       (float)LOOP2_MIN_TARGET_STEP_X100 * 0.01f,
                                       (float)LOOP2_MAX_TARGET_STEP_X100 * 0.01f);
      const uint16_t max_speed = (uint16_t)constrain((int)loop2_angle_max_speed[dof],
                                                    (int)LOOP2_MIN_ANGLE_MAXSPEED,
                                                    (int)LOOP2_MAX_ANGLE_MAXSPEED);
      if (!loop2_target_valid[dof]) {
        loop2_target_A[dof] = theta_A_curr;
        loop2_target_B[dof] = theta_B_curr;
        loop2_target_valid[dof] = true;
      }
      loop2_target_A[dof] = slewLoop2Target(loop2_target_A[dof], theta_A_ref, step_deg);
      loop2_target_B[dof] = slewLoop2Target(loop2_target_B[dof], theta_B_ref, step_deg);
      const float loop2_max_spread =
          fabsf(cascade_influence) * LOOP2_MAX_STIFFNESS_REF_DEG;
      if (loop2_max_spread > 0.001f) {
        const float a_offset = loop2_target_A[dof] - theta_0_agonist_motor;
        const float b_offset = loop2_target_B[dof] - theta_0_antagonist_motor;
        const float common_offset = 0.5f * (a_offset + b_offset);
        const float spread = a_offset - b_offset;
        const float clamped_spread = constrain(spread, -loop2_max_spread, loop2_max_spread);
        if (fabsf(clamped_spread - spread) > 0.001f) {
          loop2_target_A[dof] = theta_0_agonist_motor + common_offset + 0.5f * clamped_spread;
          loop2_target_B[dof] = theta_0_antagonist_motor + common_offset - 0.5f * clamped_spread;
          if (t_now - loop2_spread_clamp_log_ms[dof] > 500) {
            char f1[48], f2[48];
            LOG_C1_WARN_F("[LOOP2] DOF %d spread clamp %s->%s", dof,
                          c1f(f1, spread, 1), c1f(f2, clamped_spread, 1));
            loop2_spread_clamp_log_ms[dof] = t_now;
          }
        }
      }
      if (hasValidEquations(dof)) {
        const float a_min = linear_equations[dof].agonist_safe_min;
        const float a_max = linear_equations[dof].agonist_safe_max;
        const float b_min = linear_equations[dof].antagonist_safe_min;
        const float b_max = linear_equations[dof].antagonist_safe_max;
        if (a_min < a_max) {
          loop2_target_A[dof] = constrain(loop2_target_A[dof], a_min, a_max);
        }
        if (b_min < b_max) {
          loop2_target_B[dof] = constrain(loop2_target_B[dof], b_min, b_max);
        }
      }
      agonist->fireAngle2(loop2_target_A[dof], max_speed);
      antagonist->fireAngle2(loop2_target_B[dof], max_speed);
      // (chatter log removed, v2 String pass 2026-07-06: 1Hz 0xA4 status; loop2_last_log_ms now write-only)
    } else {
      agonist->fireTorque((int)command_A);
      antagonist->fireTorque((int)command_B);
    }
    // MINOR (single time authority): latch THIS fire's timestamp into the ctx (the agonist's _fire_t_us,
    // valid only after the fire above). All S2-CARRY age/staleness/stuck math derives from this value; the
    // scheduler tail copies the same fireTimestampUs() into s2_carry.fire_us for the carried pair.
    c.fire_us = agonist->fireTimestampUs();
  };

  // --- PHASE 3: drain this DOF's pair replies + Phase-2 consume + slack/holding diagnostics ---
  // Runs with no other pair in flight (its own pair is the only outstanding transaction, or it
  // was already bus-drained mid-compute by collectPendingPair()).
  // FATAL-1: the ctx is passed EXPLICITLY. In-cycle callers pass phase_ctx[dof] (textually neutral —
  // the serial path is statement-identical); the S2-CARRY-injected consume passes s2_carry.ctx so the
  // fire-time snapshot is read even though the carried DOF's own compute already overwrote phase_ctx[dof].
  // The file-scope per-DOF arrays (wp_*/diag_*/dof_state) are still indexed by the true dof — correct,
  // since the carried pair belongs to that same DOF.
  auto drainConsumeDof = [&](uint8_t dof, DofPhaseCtx &c, bool presweep = false) HOT_LAMBDA_ATTR("drainConsumeDof") {
    LKM_Motor *agonist = c.agonist;
    LKM_Motor *antagonist = c.antagonist;
    const bool impedance_active = c.impedance_active;
    const bool need_0x92 = c.need_0x92;
    const bool recover_after_a1_miss = c.recover_after_a1_miss;
    const bool loop2_active = c.loop2_active;
    const float theta_0_joint = c.theta_0_joint;
    const float stiffness_ref = c.stiffness_ref;
    const float stiffness_ref_effective = c.stiffness_ref_effective;
    const float delta_theta_smooth = c.delta_theta_smooth;
    const float theta_A_ref = c.theta_A_ref;
    const float theta_B_ref = c.theta_B_ref;
    const float command_A = c.command_A;
    const float command_B = c.command_B;
    MultiAngleData &data_A = c.data_A;
    MultiAngleData &data_B = c.data_B;
#if CONTROLLER_DEBUG
    uint32_t torque_start_us = c.torque_start_us;  // NOTE: under interleave the closed window may
    uint32_t dof_start_us = c.dof_start_us;        // include the next DOF's compute (overlap) —
#endif                                             // profiling-only; knob-OFF timing is unchanged.

    // O10 (500Hz Stage 1) DEADLINE-AWARE collect timeout — factored into collectDeadlineUs()
    // above (shared with the mid-compute collectPendingPair()), evaluated here at the same
    // point as before. c.collected: the pair may already have been bus-drained mid-flight by
    // collectPendingPair() when a later DOF's compute needed the bus; consume runs either way.
    // FATAL-2: presweep=true (S2-CARRY-injected pair only) reads the already-buffered replies before
    // the deadline WAIT; false (all in-cycle callers) is bit-identical to the historical collect.
    if (!c.collected) {
      LKM_Motor::collectPair(agonist, antagonist, collectDeadlineUs(), presweep);
      c.collected = true;
    }
    diag_motor_feedback_snapshot[dof].last_reply_mask =
        (agonist->lastReplyValid() ? 0x01 : 0x00) |
        (antagonist->lastReplyValid() ? 0x02 : 0x00);
    updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);

    // === S2 SUBSTITUTE-FIRE CONSUME (bit2, kind==PAIR_92) ===
    // The pair fired this DOF's 2-frame 0x92 read INSTEAD of a torque pair (the motors held ZOH). collectPair
    // routed the 0x92 replies into each motor's last92() (via parseMultiAngleReply). This branch does EXACTLY
    // the drift-watchdog check the blocking path did — compare |tracked - absolute| per motor and publish
    // watchdog_err_x100 — plus the MINOR-amendment tracked-angle advance from the just-validated absolute (so
    // the substitute + consume cycles don't feed the inner D/GMS the same repeated tracked sample). On a valid
    // in-threshold sample the tracking is advanced (measurement path only, rev-anchor untouched); on drift > 1°
    // ALWAYS escalate to the existing A1-miss blocking recovery (set wp_a1_miss_count so next cycle does the
    // zero_fire + synchronized 0x92 re-anchor) — the in-place re-anchor is DROPPED (the 0x92 sample is ~1-2
    // cycles skewed from the cached encoder). On a MISS: same A1-miss escalation. This consume RETURNS — the
    // downstream 0xA1 slack/HOLD/hi-rate telemetry is skipped for this one cycle (the motor-state fields Iq/
    // speed are stale ZOH; there is no fresh torque reply to report). bit2 OFF => kind is never PAIR_92 => this
    // block is dead and the consume below is statement-identical to the historical tree.
    if (c.kind == PAIR_92) {
      const bool got_A = agonist->lastReplyValid();
      const bool got_B = antagonist->lastReplyValid();
      if (got_A && got_B) {
        const MultiAngleData &abs_A = agonist->last92();
        const MultiAngleData &abs_B = antagonist->last92();
        if (!isnan(abs_A.angle) && !isnan(abs_B.angle)) {
          // getTrackedAngle() still reflects the tracking as of the substitute compute (ZOH held it — no
          // 0xA1 advanced it this cycle), so this is the genuine tracked-vs-absolute drift check.
          const float tracked_A = agonist->getTrackedAngle();
          const float tracked_B = antagonist->getTrackedAngle();
          const float err_A = fabs(tracked_A - abs_A.angle);
          const float err_B = fabs(tracked_B - abs_B.angle);
          diag_motor_feedback_snapshot[dof].watchdog_err_a_x100 = motorFeedbackErrX100(err_A);
          diag_motor_feedback_snapshot[dof].watchdog_err_b_x100 = motorFeedbackErrX100(err_B);
          diag_motor_feedback_snapshot[dof].invalid_reason = 0;
          updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
          // Heap-free build (2026-07-06): fires on every sub92 substitute cycle inside the
          // torq WP-PROF span — the String version added ~0.7-0.9ms to exactly the cycles
          // sub92 exists to keep short. Text identical (log-diff comparability).
          if (LOG_LEVEL >= 2) {
            char ln[120], f1[48], f2[48], f3[48];
            int off = 0;
            c1cat(ln, sizeof ln, &off, "[Watchdog92] DOF %d A: 0x92=%s tracked=%s err=%s", dof,
                  c1f(f1, abs_A.angle, 3), c1f(f2, tracked_A, 3), c1f(f3, err_A, 4));
            c1cat(ln, sizeof ln, &off, " | B: 0x92=%s tracked=%s err=%s",
                  c1f(f1, abs_B.angle, 3), c1f(f2, tracked_B, 3), c1f(f3, err_B, 4));
            LOG_C1_INFO_F("%s", ln);
          }
          if (err_A > 1.0f || err_B > 1.0f) {
            if (fabs(velocity_filtered[dof]) >= S2_SUB92_DRIFT_CHECK_MAX_VEL_DEG_S) {
              // MOVING: the substitute compare folds ~1 inner-period of real travel into err, so a
              // >1° reading here is (dominantly) motion skew, NOT tracking drift. DEFER — do not
              // escalate (that would false-drop torque mid-motion, the eyelet-slam-adjacent hazard)
              // and do not touch tracking. The ZOH continues; the next 0xA1 advances tracking; a
              // slower watchdog cycle re-checks; A1-miss detection independently guards real misses.
              s2_sub92_drift_deferred_count++;
            } else {
              // SLOW: err is genuine drift. ALWAYS escalate to the existing zero-fire blocking recovery
              // (do NOT re-anchor in place from a cycle-skewed substitute sample). Arming
              // wp_a1_miss_count makes the NEXT computeDof set recover_after_a1_miss -> need_0x92 ->
              // zero_fire + synchronized 0x92 re-anchor.
              if (wp_a1_miss_count[dof] < 255) wp_a1_miss_count[dof]++;
              char f1[48], f2[48], f3[48];
              LOG_C1_WARN_F("[Watchdog92] DOF %d DRIFT>1deg (errA=%s errB=%s v=%s"
                            ") -> escalate to blocking 0x92 re-anchor next cycle",
                            dof, c1f(f1, err_A, 2), c1f(f2, err_B, 2),
                            c1f(f3, velocity_filtered[dof], 1));
            }
          } else {
            // In-threshold: advance the tracked angle from the just-validated absolute (measurement path only,
            // rev-anchor math untouched) so the substitute + consume cycles don't repeat the same sample.
            agonist->setTrackedAngleFromAbsolute(abs_A.rawMotorAngle_centideg);
            antagonist->setTrackedAngleFromAbsolute(abs_B.rawMotorAngle_centideg);
          }
        }
      } else if (wp_0x92_bootstrap_done[dof]) {
        // Missed one/both 0x92 substitute replies: same discipline as an A1-miss — force the blocking 0x92
        // recovery next cycle (zero_fire + synchronized re-anchor). No torque was commanded this cycle (ZOH).
        if (wp_a1_miss_count[dof] < 255) wp_a1_miss_count[dof]++;
        diag_motor_feedback_snapshot[dof].invalid_reason = MOTOR_FEEDBACK_REASON_A1_MISS;
        updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
        uint32_t now_ms = millis();
        if (now_ms - wp_last_a1_miss_log_ms[dof] > 500) {
          LOG_C1_WARN_F("[Watchdog92] DOF %d missed 0x92 substitute reply (A=%d B=%d"
                        ") -> forcing blocking 0x92 recovery next cycle",
                        dof, got_A ? 1 : 0, got_B ? 1 : 0);
          wp_last_a1_miss_log_ms[dof] = now_ms;
        }
      }
      return;  // substitute cycle: no fresh torque reply -> skip the 0xA1 slack/HOLD/hi-rate downstream
    }

    // === PHASE 2: CONSUME this DOF's just-collected reply ===
    // Rev-tracking was advanced by collectPair; here we act on the collected state. Bootstrap/recover:
    // re-anchor from THIS cycle's 0x92 absolute (data_A/B, from the read phase) + the COLLECTED encoder
    // (both this cycle now). Watchdog (~1Hz): compare tracked vs 0x92, resync if drift > 1°. Missed reply
    // (counts rx_miss): force a 0x92 recovery next cycle.
    if (agonist->lastReplyValid() && antagonist->lastReplyValid()) {
      diag_motor_feedback_snapshot[dof].invalid_reason = 0;
      // All-clear for the Loop2 feedback-fault streak: this point is only reached when
      // the feedback checks passed AND both replies arrived (covers NaN/range/jump/miss).
      loop2_feedback_fault_streak[dof] = 0;
      if (!wp_rev_track_init[dof] || recover_after_a1_miss) {
        agonist->initRevTracking(data_A.rawMotorAngle_centideg, agonist->motorEncoderPosition);
        antagonist->initRevTracking(data_B.rawMotorAngle_centideg, antagonist->motorEncoderPosition);
        wp_rev_track_init[dof] = true;
        wp_0x92_bootstrap_done[dof] = true;
        wp_a1_miss_count[dof] = 0;
        updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
        if (recover_after_a1_miss) {
          LOG_C1_WARN("[Phase2] DOF " + String(dof) +
                      " re-anchored after missed motor command reply"
                      " (encA=" + String(agonist->motorEncoderPosition) +
                      " encB=" + String(antagonist->motorEncoderPosition) + ")");
        } else {
          LOG_C1_INFO("[Phase2] DOF " + String(dof) + " bootstrap done → tracked-only mode"
                      " (encA=" + String(agonist->motorEncoderPosition) +
                      " encB=" + String(antagonist->motorEncoderPosition) + ")");
        }
      } else if (need_0x92 && !isnan(data_A.angle) && !isnan(data_B.angle)) {
        // Watchdog cycle: compare tracked vs 0x92 absolute
        float tracked_A = agonist->getTrackedAngle();
        float tracked_B = antagonist->getTrackedAngle();
        float err_A = fabs(tracked_A - data_A.angle);
        float err_B = fabs(tracked_B - data_B.angle);
        diag_motor_feedback_snapshot[dof].watchdog_err_a_x100 = motorFeedbackErrX100(err_A);
        diag_motor_feedback_snapshot[dof].watchdog_err_b_x100 = motorFeedbackErrX100(err_B);
        updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);

        // Heap-free build (2026-07-06): blocking-watchdog cycles inside the torq WP-PROF
        // span (pre-knob / sub92-off) — same String tax as [Watchdog92]. Text identical.
        if (LOG_LEVEL >= 2) {
          char ln[120], f1[48], f2[48], f3[48];
          int off = 0;
          c1cat(ln, sizeof ln, &off, "[Watchdog] DOF %d A: 0x92=%s tracked=%s err=%s", dof,
                c1f(f1, data_A.angle, 3), c1f(f2, tracked_A, 3), c1f(f3, err_A, 4));
          c1cat(ln, sizeof ln, &off, " spd=%d iq=%d", (int)agonist->motorSpeed,
                (int)agonist->motorTorqueCurrent);
          LOG_C1_INFO_F("%s", ln);
          off = 0;
          c1cat(ln, sizeof ln, &off, "[Watchdog] DOF %d B: 0x92=%s tracked=%s err=%s", dof,
                c1f(f1, data_B.angle, 3), c1f(f2, tracked_B, 3), c1f(f3, err_B, 4));
          c1cat(ln, sizeof ln, &off, " spd=%d iq=%d", (int)antagonist->motorSpeed,
                (int)antagonist->motorTorqueCurrent);
          LOG_C1_INFO_F("%s", ln);
        }

        // Resync if drift exceeds 1° (output shaft)
        if (err_A > 1.0f || err_B > 1.0f) {
          agonist->initRevTracking(data_A.rawMotorAngle_centideg, agonist->motorEncoderPosition);
          antagonist->initRevTracking(data_B.rawMotorAngle_centideg, antagonist->motorEncoderPosition);
          LOG_C1_WARN("[Watchdog] DOF " + String(dof) + " RESYNC: errA=" +
                      String(err_A, 2) + "° errB=" + String(err_B, 2) + "°");
        }
      }
    } else if (wp_0x92_bootstrap_done[dof]) {
      // A previous-cycle 0xA1 reply was missed. Do not keep consuming stale tracked angles
      // indefinitely: force a 0x92 recovery on the very next cycle.
      if (wp_a1_miss_count[dof] < 255) {
        wp_a1_miss_count[dof]++;
      }
      diag_motor_feedback_snapshot[dof].invalid_reason = MOTOR_FEEDBACK_REASON_A1_MISS;
      updateMotorFeedbackSnapshot(dof, need_0x92, recover_after_a1_miss);
      uint32_t now_ms = millis();
      if (now_ms - wp_last_a1_miss_log_ms[dof] > 500) {
        LOG_C1_WARN_F("[Phase2] DOF %d missed motor command reply (A=%d B=%d"
                      ") → forcing 0x92 recovery next cycle",
                      dof, agonist->lastReplyValid() ? 1 : 0,
                      antagonist->lastReplyValid() ? 1 : 0);
        wp_last_a1_miss_log_ms[dof] = now_ms;
      }
      if (loop2_active) {
        if (++loop2_feedback_fault_streak[dof] >= LOOP2_FEEDBACK_FAULT_STREAK_CYCLES) {
          latchTerminalMotionFault(dof, "LOOP2_MOTOR_REPLY_MISS", this);
        }
        return; // streak window: single CAN reply misses no longer power-cut
      }
    }

    // Build the response view from this DOF's just-collected replies, so the downstream slack /
    // diagnostics / hi-rate code keeps working unchanged.
    PipelinedTorqueResponseData trResp = {};
    trResp.dataA.valid        = agonist->lastReplyValid();
    trResp.dataA.angle        = agonist->lastReplyValid() ? agonist->getTrackedAngle() : NAN;
    trResp.dataA.torqueCurrent = agonist->motorTorqueCurrent;
    trResp.dataA.motorSpeed   = agonist->motorSpeed;
    trResp.dataA.encoder      = agonist->motorEncoderPosition;
    trResp.dataA.temperature  = agonist->motorTemperature2;
    trResp.dataB.valid        = antagonist->lastReplyValid();
    trResp.dataB.angle        = antagonist->lastReplyValid() ? antagonist->getTrackedAngle() : NAN;
    trResp.dataB.torqueCurrent = antagonist->motorTorqueCurrent;
    trResp.dataB.motorSpeed   = antagonist->motorSpeed;
    trResp.dataB.encoder      = antagonist->motorEncoderPosition;
    trResp.dataB.temperature  = antagonist->motorTemperature2;

    // (FIRE + COLLECT already done above, before the CONSUME — burst-aware <=2 outstanding.)

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
                char f1[48], f2[48], f3[48];
                LOG_C1_WARN_F("⚠️ [SLACK] DOF %d %s tendon slack! iqA=%d iqB=%d ratio=%s"
                              " stiffBase=%s stiffEff=%s",
                              dof, side, (int)trResp.dataA.torqueCurrent,
                              (int)trResp.dataB.torqueCurrent, c1f(f1, ratio, 3),
                              c1f(f2, stiffness_ref, 1), c1f(f3, stiffness_ref_effective, 1));
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

            // Heap-free build (2026-07-06): the String-concat version of this line cost
            // ~1-2ms on core1 (exact-size realloc per '+', malloc/dtostrf/free per float,
            // IRQs-off cross-core malloc mutex) INSIDE the torq WP-PROF span — the dominant
            // hold-phase cycle spike. c1f/c1cat keep the output byte-identical incl. the
            // 119-char truncation point.
            {
              char ln[120], f1[48], f2[48], f3[48];
              int off = 0;
              c1cat(ln, sizeof ln, &off, "[HOLD_EVT] DOF%d qPrev=%s q=%s qDes=%s", dof,
                    c1f(f1, q_prev_hold, 3), c1f(f2, q_curr_hold_evt, 3),
                    c1f(f3, q_des_hold_evt, 3));
              c1cat(ln, sizeof ln, &off, " err=%s dq=%s vel=%s",
                    c1f(f1, error_hold_evt, 3), c1f(f2, q_curr_hold_evt - q_prev_hold, 3),
                    c1f(f3, velocity_filtered[dof], 3));
              c1cat(ln, sizeof ln, &off, " dth=%s kiS=%s frz=%d",
                    c1f(f1, delta_theta_smooth, 3), c1f(f2, last_outer_ki_scale_dbg[dof], 3),
                    last_outer_i_freeze_dbg[dof] ? 1 : 0);
              c1cat(ln, sizeof ln, &off, " rt=%s Aref=%s Bref=%s",
                    c1f(f1, last_retension_boost_dbg[dof], 2), c1f(f2, theta_A_ref, 3),
                    c1f(f3, theta_B_ref, 3));
              c1cat(ln, sizeof ln, &off, " cmdA=%s cmdB=%s iqA=%d iqB=%d",
                    c1f(f1, command_A, 1), c1f(f2, command_B, 1), iq_A_evt, iq_B_evt);
              c1cat(ln, sizeof ln, &off, " iqR=%s", c1f(f1, iq_ratio_evt, 3));
              LOG_C1_WARN_F("%s", ln);
            }
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

    // === MOTION DIAGNOSTIC LOG (TELEMETRY ONLY — control-neutral) ===
    // The [DIAG_HOLD] line below is GATED to a clean steady HOLDING (gate_holding &&
    // gate_low_velocity && gate_no_transition && ...), so during a step/sweep the host
    // re-reads a stale held line and is blind to the LIVE outer correction / motor effort.
    // This block emits the LIVE per-cycle diagnostic while the DOF is MOVING (hold gate
    // NOT satisfied), throttled to ~50 Hz, on a clearly distinct tag so it is never
    // confused with the gated holding ema. It ONLY READS existing per-cycle variables
    // (delta_theta_smooth, theta_0_joint, stiffness_ref_effective, velocity_filtered,
    // dof_data, trResp) and WRITES ONLY a log line — no control state is touched, no gate
    // that affects torque, no trajectory, no holding-ema computation. Reported dth is the
    // LIVE delta_theta_smooth, NOT the steady holding ema.
    {
      bool diag_move_active = (dof < 3) && impedance_active && dof_data.valid[dof] &&
                              (dof >= MAX_DOFS || dof_state[dof] != DofState::HOLDING);
      if (diag_move_active &&
          t_now - last_motion_diag_log[dof] > diag_move_period_ms) {
        // Motor-space geometric residual — same computation as the DIAG_HOLD block (read-only).
        float mv_expected_A = 0.0f, mv_expected_B = 0.0f;
        float mv_residual_A = 0.0f, mv_residual_B = 0.0f;
        float mv_q_joint = dof_data.angles[dof];
        float mv_q0 = dof_data.valid[Q0_DOF] ? dof_data.angles[Q0_DOF]
                                             : linear_equations[dof].q0_nominal;
        if (cached_motor_angles.valid[dof] &&
            calculateMotorAnglesWithEquations(dof, mv_q_joint, mv_q_joint,
                                              mv_expected_A, mv_expected_B, mv_q0)) {
          mv_residual_A = cached_motor_angles.agonist[dof] - mv_expected_A;
          mv_residual_B = cached_motor_angles.antagonist[dof] - mv_expected_B;
        }

        bool mv_iq_valid = trResp.dataA.valid && trResp.dataB.valid;
        int16_t mv_iq_A = mv_iq_valid ? trResp.dataA.torqueCurrent : 0;
        int16_t mv_iq_B = mv_iq_valid ? trResp.dataB.torqueCurrent : 0;

        // [DIAG_MOVE] mirrors the [DIAG_HOLD] field layout (q/ema/resA/resB/iqA/iqB) so the
        // host's existing DIAG regex parses it identically, but ema= carries the LIVE outer
        // correction (delta_theta_smooth), not the gated holding ema. Extra motion fields:
        // qDes (impedance reference this cycle), vel (filtered joint velocity), stiffEff.
        // Heap-free build (2026-07-06): 50Hz in motion inside the torq WP-PROF span — the
        // String version was the largest motion-phase spike contributor (~0.5-1.1ms/line).
        // FORMAT FROZEN (host DIAG regex); c1f == String(float,dp) byte-for-byte.
        if (LOG_LEVEL >= 2) {
          char ln[120], f1[48], f2[48], f3[48];
          int off = 0;
          c1cat(ln, sizeof ln, &off, "[DIAG_MOVE] DOF%d q=%s ema=%s resA=%s", dof,
                c1f(f1, mv_q_joint, 1), c1f(f2, delta_theta_smooth, 2),
                c1f(f3, mv_residual_A, 2));
          c1cat(ln, sizeof ln, &off, " resB=%s iqA=%d iqB=%d iqV=%d",
                c1f(f1, mv_residual_B, 2), mv_iq_A, mv_iq_B, mv_iq_valid ? 1 : 0);
          c1cat(ln, sizeof ln, &off, " qDes=%s vel=%s stiffEff=%s",
                c1f(f1, theta_0_joint, 2), c1f(f2, velocity_filtered[dof], 2),
                c1f(f3, stiffness_ref_effective, 1));
          LOG_C1_INFO_F("%s", ln);
        }

        last_motion_diag_log[dof] = t_now;
      }
    }

    // === HI-RATE WINDOWED MOTION CAPTURE (TELEMETRY ONLY — control-neutral) ===
    // Records EVERY control-loop cycle (~250 Hz) for the host-armed DOF into a dedicated buffer
    // (free of the 50 Hz [DIAG_MOVE] aliasing), downloaded later over CAN. Sources MIRROR the
    // [DIAG_MOVE] block above: q=dof_data.angles, qDes=theta_0_joint (impedance reference this
    // cycle), dth=delta_theta_smooth (LIVE outer correction), iqA/iqB=trResp.dataA/B.torqueCurrent,
    // resA/resB=cached motor angles vs calculateMotorAnglesWithEquations. The capture is GATED on
    // diag_hirate_is_capturing(dof) so when not armed the per-cycle cost is one bool read — no
    // residual/iq math, no logging. It ONLY reads existing live variables and appends to the
    // hi-rate buffer; it NEVER touches delta_theta, the cascade, gates, torque, the trajectory,
    // or the DiagRecord ring.
    if (diag_hirate_is_capturing(dof)) {
      float hr_expected_A = 0.0f, hr_expected_B = 0.0f;
      float hr_residual_A = 0.0f, hr_residual_B = 0.0f;
      float hr_q_joint = dof_data.valid[dof] ? dof_data.angles[dof] : theta_0_joint;
      float hr_q0 = dof_data.valid[Q0_DOF] ? dof_data.angles[Q0_DOF]
                                           : linear_equations[dof].q0_nominal;
      if (cached_motor_angles.valid[dof] &&
          calculateMotorAnglesWithEquations(dof, hr_q_joint, hr_q_joint,
                                            hr_expected_A, hr_expected_B, hr_q0)) {
        hr_residual_A = cached_motor_angles.agonist[dof] - hr_expected_A;
        hr_residual_B = cached_motor_angles.antagonist[dof] - hr_expected_B;
      }
      bool hr_iq_valid = trResp.dataA.valid && trResp.dataB.valid;
      int16_t hr_iq_A = hr_iq_valid ? trResp.dataA.torqueCurrent : 0;
      int16_t hr_iq_B = hr_iq_valid ? trResp.dataB.torqueCurrent : 0;
      diag_hirate_capture_sample(dof, hr_q_joint, theta_0_joint, delta_theta_smooth,
                                 hr_iq_A, hr_iq_B, hr_residual_A, hr_residual_B);
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
      // Live q0 (DOF0) from this cycle's snapshot for the bilinear DOF1 residual; invalid
      // DOF0 -> q0_nominal (center slice), never NaN. Ignored unless map_mode == MAP_BILINEAR.
      float q0_res = dof_data.valid[Q0_DOF] ? dof_data.angles[Q0_DOF]
                                            : linear_equations[dof].q0_nominal;
      if (cached_motor_angles.valid[dof] &&
          calculateMotorAnglesWithEquations(dof, q_joint, q_joint, expected_A_res, expected_B_res,
                                            q0_res)) {
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

      // Heap-free build (2026-07-06): 2Hz at hold inside the torq WP-PROF span — the String
      // version measured ~1.3ms/line and alone explains the pure-hold torq max (bench 1803).
      // FORMAT FROZEN (host DIAG regex); c1f == String(float,dp) byte-for-byte.
      if (LOG_LEVEL >= 2) {
        char ln[120], f1[48], f2[48], f3[48];
        int off = 0;
        c1cat(ln, sizeof ln, &off, "[DIAG_HOLD] DOF%d q=%s ema=%s resA=%s", dof,
              c1f(f1, q_joint, 1), c1f(f2, holding_dtheta_ema[dof], 2),
              c1f(f3, residual_A, 2));
        c1cat(ln, sizeof ln, &off, " resB=%s iqA=%d iqB=%d iqR=%s iqV=%d",
              c1f(f1, residual_B, 2), iq_A_diag, iq_B_diag, c1f(f2, iq_ratio, 2),
              iq_valid ? 1 : 0);
        c1cat(ln, sizeof ln, &off, " stiffBase=%s stiffEff=%s rt=%s",
              c1f(f1, stiffness_ref, 1), c1f(f2, stiffness_ref_effective, 1),
              c1f(f3, last_retension_boost_dbg[dof], 2));
        c1cat(ln, sizeof ln, &off, " trim=%s n=%u", c1f(f1, proposed_trim_deg[dof], 3),
              (unsigned)holding_ema_samples[dof]);
        LOG_C1_INFO_F("%s", ln);
      }

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
        // Map-health DRIFT classifier: |holding_dtheta_ema| (persistent outer-loop compensation at a
        // clean hold) as a fraction of the delta_theta rail = a measure of how wrong the feedforward
        // map is. NOTE: the EMA is gated to HOLDING and reset by motion, so this flags slow MAP DRIFT,
        // NOT the (motion) limit cycle — that is the raw-delta_theta saturation-dwell detector (Phase 1).
        // Packed into flags bits 2-3: 0=OK, 1=WARN, 2=DEGRADED. Thresholds are placeholders (tune on bench).
        constexpr float MAP_HEALTH_WARN_FRAC     = 0.25f;
        constexpr float MAP_HEALTH_DEGRADED_FRAC = 0.50f;
        float ema_frac = fabs(holding_dtheta_ema[dof]) / DEFAULT_MAX_DELTA_THETA;
        uint8_t map_health = (ema_frac >= MAP_HEALTH_DEGRADED_FRAC) ? 2u
                           : (ema_frac >= MAP_HEALTH_WARN_FRAC)     ? 1u : 0u;
        diag_flags |= (uint8_t)((map_health & 0x03u) << 2);  // bits 2-3
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
  };

  // --- SCHEDULER — serial vs interleave differ ONLY in the ORDER these calls are made ---
  // Latch the knob once per cycle: a host toggle can never flip the schedule between a fire
  // and its drain. With <2 configured DOFs the interleave order degenerates to serial anyway.
  const bool interleave_this_cycle = sched_interleave_enabled && (config.dof_count >= 2);
  // S2 CARRY (bit1): latch it the SAME once-per-cycle way. A deferred pair's collect happens past
  // its own fire slot, so even serial+carry needs the pre-fire stale-RX sweep (gate below).
  // GUARD: never carry on a joint with a DIRECT-DRIVE DOF. That DOF's preamble getSingleAngleSync()
  // (a blocking 0x94 read that flushes the RX buffers, BEFORE the injection resolve next cycle) would
  // drop the carried tendon pair's buffered replies AND stack a second outstanding transaction — the
  // <=2-outstanding proof requires the carried pair be the ONLY thing in flight through the preamble.
  // Pure-tendon joints (ankle 2-DOF, knee 1-DOF — the design's targets) have a no-op preamble. The
  // hip-hybrid keeps Stage-1 behavior. Injection still runs unconditionally to drain any stray carry.
  bool joint_has_direct_drive = false;
  for (uint8_t d = 0; d < config.dof_count; d++) {
    if (config.dofs[d].drive_type == DRIVE_DIRECT_DRIVE) { joint_has_direct_drive = true; break; }
  }
  const bool carry_this_cycle = sched_carry_enabled && !joint_has_direct_drive;

  // === S2 CARRY — INJECTION (RESOLVE point 1) ===
  // Runs UNCONDITIONALLY whenever a pair is carried (even if the knob was just turned OFF — no orphan
  // is possible). The carried pair re-enters as the Stage-1 pending pair, so ALL existing machinery
  // (pre-fire drain, fire veto, collect/consume, A1-miss recovery) drains+consumes it with no new
  // ordering logic. pending_is_carried routes its collect through the FATAL-2 pre-sweep.
  if (s2_carry.dof >= 0) {
    const int8_t cdof = s2_carry.dof;
    const uint32_t carry_age_us = time_us_32() - s2_carry.fire_us;
    const bool stale = carry_age_us > (uint32_t)(2u * (uint32_t)inner_loop_period_us);
    if (cdof >= 0 && cdof < config.dof_count && dof_state[cdof] == DofState::IDLE) {
      // (b) IDLE DISCARD: the DOF went IDLE while the pair was in flight (fault/e-stop/host stop).
      // Never re-command it — just bus-drain the pair (clear _fire_pending + the buffered replies)
      // and discard the consume. Uses the carried motors directly (cached_* still valid for the DOF).
      LKM_Motor *cag = (cdof < MAX_DOFS) ? cached_agonist[cdof] : nullptr;
      LKM_Motor *can2 = (cdof < MAX_DOFS) ? cached_antagonist[cdof] : nullptr;
      if (cag != nullptr && can2 != nullptr) {
        LKM_Motor::collectPair(cag, can2, collectDeadlineUs(), /*presweep=*/true);
      }
      s2_carry.dof = -1;
      s2_carry.gen++;
      s2_idle_discard_count++;
    } else {
      // (c) NOMINAL / (a) STALENESS BACKSTOP: re-inject as the Stage-1 pending pair. The existing
      // drain (before the next fire, or the tail) consumes it with the FATAL-2 pre-sweep; a miss
      // flows into the existing A1-miss recovery. Backstop (age > 2*period): additionally force the
      // per-DOF 0x92 watchdog DUE so the next cycle re-verifies the tracked angle early.
      pending_fire_dof = cdof;
      pending_ctx = &s2_carry.ctx;
      pending_is_carried = true;
      if (stale) {
        if (cdof >= 0 && cdof < MAX_DOFS) wp_watchdog_cycle_count[cdof] = wp_watchdog_interval;
        s2_stale_backstop_count++;
        LOG_C1_WARN("[S2] carry staleness backstop DOF " + String((int)cdof) +
                    " age=" + String(carry_age_us) + "us -> early 0x92 re-verify");
      } else {
        s2_injected_count++;
      }
      // Clear the slot now (single authority). pending_ctx still points at s2_carry.ctx, whose bytes
      // survive until the tail write point (this cycle's consume runs first), so the snapshot is safe.
      s2_carry.dof = -1;
      s2_carry.gen++;
    }
  }

  for (uint8_t dof = 0; dof < config.dof_count; dof++) {
    if (pending_fire_dof >= 0 && config.dofs[dof].drive_type == DRIVE_DIRECT_DRIVE) {
      // A direct-drive DOF fires inline INSIDE its compute (no separate fire/drain phases),
      // and collectPendingPair there only bus-drains the previous pair — the CONSUME half
      // (where a reply-miss streak latches a terminal fault) would still be deferred. The
      // serial order guaranteed the previous DOF's pair was fully consumed before this DOF
      // computed; restore that here so a latched fault gates the inline setTorque at this
      // DOF's IDLE guard. Costs the flight overlap only for tendon->direct transitions.
      const int prev_dof = pending_fire_dof;
      DofPhaseCtx &pdrain_ctx = pending_ctx ? *pending_ctx : phase_ctx[prev_dof];
      const bool pdrain_carried = pending_is_carried;
      pending_fire_dof = -1;
      pending_ctx = nullptr;
      pending_is_carried = false;
      drainConsumeDof((uint8_t)prev_dof, pdrain_ctx, pdrain_carried);
    }
    // SERIAL+CARRY (bit1 without bit0): "drain-before-next-COMPUTE" (MAJOR amendment). Plain serial
    // drains each DOF immediately after its fire; serial+carry instead leaves the pair pending after
    // fire (like interleave) and drains it HERE, at the top of the NEXT DOF's iteration BEFORE that
    // DOF computes — preserving each DOF's consume-before-next-compute freshness. (Interleave keeps
    // its post-compute drain to hide the flight; this block is a no-op under interleave since it
    // drains right after compute anyway, and under plain serial since pending stays -1.)
    if (!interleave_this_cycle && carry_this_cycle && pending_fire_dof >= 0) {
      const int prev_dof = pending_fire_dof;
      DofPhaseCtx &pdrain_ctx = pending_ctx ? *pending_ctx : phase_ctx[prev_dof];
      const bool pdrain_carried = pending_is_carried;
      pending_fire_dof = -1;
      pending_ctx = nullptr;
      pending_is_carried = false;
      drainConsumeDof((uint8_t)prev_dof, pdrain_ctx, pdrain_carried);
    }
    if (!computeDof(dof)) {
      // Skipped DOF (IDLE / fault / guard / direct-drive handled inline): nothing was fired
      // for it, so nothing is drained for it. A pair already in flight from a previous DOF
      // stays pending and is drained before the next fire (or after the loop).
      continue;
    }
    if (pending_fire_dof >= 0) {
      // Interleave (or an injected S2-CARRY pair): the previous/carried DOF's pair flew while this
      // DOF computed (or across the cycle boundary). Drain+consume it now, BEFORE firing this DOF —
      // never more than one pair outstanding on the motor CAN.
      const int prev_dof = pending_fire_dof;
      DofPhaseCtx &pdrain_ctx = pending_ctx ? *pending_ctx : phase_ctx[prev_dof];
      const bool pdrain_carried = pending_is_carried;
      pending_fire_dof = -1;
      pending_ctx = nullptr;
      pending_is_carried = false;
      drainConsumeDof((uint8_t)prev_dof, pdrain_ctx, pdrain_carried);
      if (emergency_stop_requested || dof_state[dof] == DofState::IDLE) {
        // The drain latched a terminal fault (stopAllMotors + motor power cut + all DOFs
        // forced IDLE) AFTER this DOF's compute passed its IDLE guard — the serial order
        // made that impossible because the drain ran before the next DOF's compute. Firing
        // the stale pair would re-command (or 0xA4-re-latch) a motor the fault path just
        // stopped, during the power-decay window. Veto it; next cycle computeDof exits IDLE.
        LOG_C1_ERROR("[SCHED] fire veto: terminal fault latched in mid-schedule drain; DOF " +
                     String(dof) + " pair not fired");
        continue;
      }
    }
    if ((interleave_this_cycle || carry_this_cycle) && phase_ctx[dof].agonist != nullptr) {
      // Frame-level guard the transaction invariant does not cover: late replies from a
      // previously TIMED-OUT collect (routine at 500Hz — the end-of-loop drain can be
      // deadline-truncated below the ~500-600us round-trip) are ownerless and sit in the
      // two MCP2515 RX buffers. Left there, this pair's unattended flight (overlapped by
      // the next compute) hard-drops the fresh replies -> rx_overflow -> miss -> zero_fire
      // chain across DOFs. Plain-serial mode needs no sweep: collectPair starts ~20us after
      // the fire and clears stale frames before any fresh reply arrives. But serial+carry
      // (bit1 without bit0) DOES need it — a deferred pair's collect happens past its own
      // fire slot (MAJOR amendment: || carry_this_cycle). Provably dead when both bits off.
      phase_ctx[dof].agonist->flushStaleRx(4);
    }
    fireDof(dof);
    if (interleave_this_cycle) {
      pending_fire_dof = dof;  // leave the pair in flight; the next DOF's compute overlaps it
      pending_ctx = &phase_ctx[dof];
      pending_is_carried = false;
    } else if (carry_this_cycle) {
      // SERIAL+CARRY: leave the pair pending (drained at the top of the next DOF's iteration, or
      // carried across the boundary at the tail if this is the last fired DOF).
      pending_fire_dof = dof;
      pending_ctx = &phase_ctx[dof];
      pending_is_carried = false;
    } else {
      drainConsumeDof(dof, phase_ctx[dof]);  // plain serial (boot default): EXACTLY the historical order
    }
  }
  if (pending_fire_dof >= 0) {
    // The last fired DOF still has a pair in flight (interleave: DOF_b; serial+carry: also reachable
    // when the loop leaves a pending pair — see below). Under bit1 on a carry-eligible cycle, DEFER
    // it across the cycle boundary (WRITE point) instead of draining; otherwise drain it here (v1).
    const int last_dof = pending_fire_dof;
    DofPhaseCtx &last_ctx = pending_ctx ? *pending_ctx : phase_ctx[last_dof];
    const bool last_carried = pending_is_carried;
    pending_fire_dof = -1;
    pending_ctx = nullptr;
    pending_is_carried = false;
    // === S2 CARRY — WRITE point (the ONLY one) ===
    // Carry-eligible = bit1 on AND this is a fresh in-cycle pair (NOT an already-carried/injected one:
    // never re-carry across two boundaries — that would let a pair age > 1 cycle). A bootstrap/recovery
    // pair (zero_fire) is drained in-cycle serial-style, never carried (its re-anchor must complete now).
    const bool carry_eligible = carry_this_cycle && !last_carried && !last_ctx.zero_fire &&
                                last_ctx.agonist != nullptr && last_ctx.antagonist != nullptr;
    if (carry_eligible) {
      // SINGLE-AUTHORITY TRIPWIRE: the slot must have been resolved by this cycle's injection
      // (which unconditionally clears dof to -1). If it is still occupied, a carry was orphaned
      // -> loud counter (never a hard halt on the control core). MUST read 0 in [S2DIAG].
      if (s2_carry.dof >= 0) {
        s2_orphan_count++;
        LOG_C1_ERROR("[S2DIAG] ORPHAN carry: slot occupied (dof=" + String(s2_carry.dof) +
                     ") at write point — injection missed a resolve");
      }
      // Snapshot the fire-time ctx into the single slot; leave the pair in flight. The next cycle's
      // injection re-enters it as the Stage-1 pending pair. fire_us = the pair's fireTimestampUs (set
      // in fireDof) — single time authority for all age/staleness math.
      s2_carry.ctx = last_ctx;
      s2_carry.dof = (int8_t)last_dof;
      s2_carry.kind = last_ctx.kind;
      s2_carry.fire_us = last_ctx.fire_us;
      s2_carry.gen++;
      s2_carried_count++;
    } else {
      drainConsumeDof((uint8_t)last_dof, last_ctx, last_carried);
    }
  }

  // Mark diagnostics as valid after processing all DOFs
  pid_diagnostics.last_update_ms = millis();
  pid_diagnostics.valid = true;
  
  // Reset safety check counter after processing all DOFs
  // This ensures all DOFs in HOLDING mode are checked in the same cycle
  // Using 10 inner cycles ≈ 40ms at the 250Hz/4ms default inner loop rate
  //
  // LATCH (outer-divisor fix, 2026-07-08): consume the counter only on an outer-due
  // cycle. The HOLDING checks live inside the outer-gated block, but this reset ran
  // unconditionally every inner cycle once >= 10; with the old cycle_count-derived
  // phase, counter >= 10 (cycle_count ≡ 0 mod 10) and outer-due (≡ 1 mod divisor)
  // were provably disjoint for divisors 2 and 4 — ALL periodic HOLDING safety checks
  // (joint/mapping limits, MOTOR_RANGE tendon-breakage escalation) silently never
  // ran. Latching keeps the counter pending until an outer-due cycle consumes it
  // (check cadence becomes 10..10+divisor-1 inner cycles). At divisor 1
  // outer_cycle_due is always true — behavior-identical.
  if (outer_cycle_due && safety_check_counter >= 10) {
    safety_check_counter = 0;
  }
  
  // === PROFILING: Calculate cycle time ===
  {
    uint32_t cycle_end_us = time_us_32();
    cycle_time_us_last = cycle_end_us - profiling_start_us;
    const uint32_t overrun_threshold_us = inner_loop_period_us + LOOP_OVERRUN_MARGIN_US;
    
    // Update max (reset every ~10 seconds)
    if (cycle_time_us_last > cycle_time_us_max) {
      cycle_time_us_max = cycle_time_us_last;
    }
    
    // Exponential moving average (α = 0.1 for smoothing)
    cycle_time_us_avg = (cycle_time_us_avg * 9 + cycle_time_us_last) / 10;

    // Treat loop overruns as burst diagnostics, not single-cycle noise.
    if (cycle_time_us_last > overrun_threshold_us) {
      if (loop_overrun_streak < 255) {
        ++loop_overrun_streak;
      }
      if (!loop_overrun_burst_active &&
          loop_overrun_streak >= LOOP_OVERRUN_CONSECUTIVE_LIMIT) {
        diag_note_loop_overrun();
        loop_overrun_burst_active = true;
      }
      // OVERRUN GUARD (backstop for the non-blocking motor-CAN root fix): the loop is meant to stay
      // on-time now (no in-loop ~5ms busy-wait). A SUSTAINED overrun means the fix failed or a new
      // in-loop block crept in -> e-stop to bound any building oscillation BEFORE it stresses the
      // tendon eyelet. The "consecutive" requirement means the isolated ~1/256-cycle 0x92 watchdog
      // read (streak resets to 0 the next cycle) cannot trip it; in normal post-fix operation the loop
      // never overruns, so this stays dormant. It catches the ROOT cause (overrun) — unlike the removed
      // velocity guard which caught the symptom and false-tripped on the operator's hand vibration.
      if (loop_overrun_streak >= LOOP_OVERRUN_ESTOP_LIMIT) {
        // Heap-free (dev pass 2026-07-07): this was the LAST String-mode core1 log — on the
        // worst possible path (sustained-overrun e-stop, i.e. exactly when the loop is late).
        LOG_C1_ERROR_F("[SAFETY] Sustained loop overrun (%u cycles, last=%luus) — EMERGENCY STOP (overrun guard)",
                       (unsigned)loop_overrun_streak, (unsigned long)cycle_time_us_last);
        emergency_stop_requested = true;
      }
    } else {
      loop_overrun_streak = 0;
      loop_overrun_burst_active = false;
    }

    // Log only when over budget (every 5 seconds = 2500 cycles @ 500Hz)
    // This avoids logging overhead during normal operation
    static uint16_t profiling_log_counter = 0;
    if (cycle_profiling_reset_pending) {
      cycle_profiling_reset_pending = false;
      profiling_log_counter = 0;
      cycle_time_us_max = 0;  // measurements from the old rate must not leak past a switch
#if CONTROLLER_DEBUG
      // Same leak for the [WP PROF] window: drop in-flight old-rate samples and latched maxima
      // so the first post-switch avg/max pair is pure new-rate.
      loop_micro_profile = LoopMicroProfile{};
#endif
    }
    profiling_log_counter++;
    if (profiling_log_counter >= 2500) {
      profiling_log_counter = 0;
      
      // Only log if we exceeded the budget during this period
      if (cycle_time_us_max > overrun_threshold_us) {
        LOG_C1_WARN_F("[PROFILING] OVER DIAG THRESHOLD! last=%luµs, avg=%luµs, max=%luµs "
                      "(budget=%uµs, threshold=%luµs)",
                      (unsigned long)cycle_time_us_last, (unsigned long)cycle_time_us_avg,
                      (unsigned long)cycle_time_us_max, (unsigned)inner_loop_period_us,
                      (unsigned long)overrun_threshold_us);
      }
      
      // Reset max for next period
      cycle_time_us_max = 0;
    }
  }

  // Drain at most one deferred [Metrics] line per cycle (outside every WP-PROF dof span).
  metricsDrainOnePending();

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
      // FULL-CYCLE wall-clock (executeControlLoop start->end). The per-bucket WP spans
      // above redistribute under S2 carry (drain moves to the next cycle), so they cannot
      // measure carry's true gain; this is the honest number. avg = 10-cycle EMA, max
      // reset per WP window below. Emitted every clean window (was PROFILING-over-threshold
      // only, i.e. invisible in a healthy run) — the carry A/B metric.
      LOG_C1_INFO_F("[CYCLETIME] avg=%luus max=%luus last=%luus",
                 (unsigned long)cycle_time_us_avg, (unsigned long)cycle_time_us_max,
                 (unsigned long)cycle_time_us_last);
      // Burst-aware motor-CAN health: cumulative since boot. PASS = rx_miss/tx_fail/rx_overflow
      // ~flat after bootstrap (a steady climb = the collect is still degrading to 0x92 recover).
      LOG_C1_INFO_F("[BUSDIAG] tx_fail=%lu rx_miss=%lu rx_overflow=%lu",
                 (unsigned long)LKM_Motor::txFailCount(),
                 (unsigned long)LKM_Motor::rxMissCount(),
                 (unsigned long)LKM_Motor::rxOverflowCount());
      // S2 CARRY health (cumulative since boot). G1 signal: carried==injected+stale nominally, abandon/
      // idle_discard ~0 in steady control, wrong_core==0 (a nonzero = a core0 call reached the choke).
      LOG_C1_INFO_F("[S2DIAG] carried=%lu injected=%lu stale=%lu idle_disc=%lu abandon=%lu wrong_core=%lu orphan=%lu drift_defer=%lu",
                 (unsigned long)s2_carried_count, (unsigned long)s2_injected_count,
                 (unsigned long)s2_stale_backstop_count, (unsigned long)s2_idle_discard_count,
                 (unsigned long)s2_abandon_count, (unsigned long)s2_wrong_core_count,
                 (unsigned long)s2_orphan_count, (unsigned long)s2_sub92_drift_deferred_count);
      // S2 CARRY residual-wait (G1 carry-correctness): of the carried collects, how many had to WAIT
      // past the pre-sweep (not_hidden) and the worst wait. not_hidden~0 = the carry fully hides the
      // flight (the target); a rising not_hidden means compute is too short / flight too long.
      LOG_C1_INFO_F("[S2WAIT] carried_collects=%lu not_hidden=%lu max_us=%lu",
                 (unsigned long)LKM_Motor::carryCollectCount(),
                 (unsigned long)LKM_Motor::carryWaitOverCount(),
                 (unsigned long)LKM_Motor::carryWaitMaxUs());
      {
        // Stage-1 instrumentation: fire->reply latency histogram (us). Sizes the interleave
        // flight windows + verifies whether the /INT-gated drain is actually asserting.
        const uint32_t *lh = LKM_Motor::latencyHistogram();
        LOG_C1_INFO_F("[LATHIST] <200:%lu <300:%lu <400:%lu <500:%lu <700:%lu <1000:%lu <1500:%lu ge1500:%lu",
                   (unsigned long)lh[0], (unsigned long)lh[1], (unsigned long)lh[2],
                   (unsigned long)lh[3], (unsigned long)lh[4], (unsigned long)lh[5],
                   (unsigned long)lh[6], (unsigned long)lh[7]);
      }
      // V2 snapshot: mirror the worst-case per-section times into the diag plane so the
      // loop profile is downloadable over CAN (FAULT_SNAPSHOT) without the USB serial.
      diag_note_loop_profile(loop_micro_profile.max_dof_us, loop_micro_profile.max_outer_us,
                             loop_micro_profile.max_eq_us, loop_micro_profile.max_can_us,
                             loop_micro_profile.max_pid_us, loop_micro_profile.max_torque_us,
                             loop_micro_profile.max_safety_us);
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
      cycle_time_us_max = 0;  // windowed full-cycle max (see [CYCLETIME] above)
      loop_micro_profile.samples = 0;
      loop_micro_profile.safety_samples = 0;
      loop_micro_profile.last_log_ms = now_ms;
    }
  }
#endif
  
  return any_movement;
}
