/**
 * @file PID.cpp
 * @brief Implementation of incremental PID controller
 * 
 * See PID.h for detailed documentation.
 */

#include "PID.h"
#include <hot_path.h>

// ===================================================================
// CONSTRUCTOR
// ===================================================================

/**
 * Initialize PID controller with specified parameters and reset state
 */
PID::PID(float Ts, float kp, float ki, float kd, float umax, float umin, float tau) {
  this->Ts   = Ts;
  this->kp   = kp;
  this->ki   = ki;
  this->kd   = kd;
  this->umax = umax;
  this->umin = umin;
  this->tau  = tau;

  // Initialize state to zero
  eprev[0]   = 0;
  eprev[1]   = 0;
  xprev[0]   = 0;
  xprev[1]   = 0;
  uprev      = 0;
  udfiltprev = 0;
}

// ===================================================================
// CONTROL COMPUTATION
// ===================================================================

/**
 * Compute PID control output using incremental algorithm
 * 
 * Algorithm steps:
 * 1. Calculate current error: e(k) = setpoint - measurement
 * 2. Compute proportional increment: Kp * [e(k) - e(k-1)]
 * 3. Compute integral term with anti-windup
 * 4. Compute filtered derivative term
 * 5. Sum all terms with feedforward
 * 6. Apply output saturation
 * 7. Update state for next iteration
 * 
 * Parameters:
 * - xsp: setpoint (desired value)
 * - x: current measured value
 * - uff: feedforward control term
 */
float HOT_FUNC(PID::control)(float xsp, float x, float uff, float ki_scale, bool freeze_integrator,
                   float dt_override) {
  // Step 1: Calculate error
  float e = xsp - x;

  // Measured-dt: when a positive dt_override (the real elapsed loop time, seconds) is
  // supplied, use it for the TIME-SCALED terms (integral, derivative, derivative filter)
  // instead of the fixed member Ts. When a cycle runs long — e.g. the real cycle is ~8ms
  // not 4ms after a loop overrun — fixed-Ts over-weights the derivative (divides the
  // legitimately larger second-difference by too small a dt -> a ~2x torque kick on the
  // back-to-back resume) and under-integrates. ts = the real dt fixes both. dt_override<=0
  // keeps the legacy fixed-Ts behaviour (and gates the runtime A/B flag at the call site).
  float ts = (dt_override > 0.0f) ? dt_override : Ts;

  // Step 2: Proportional term (incremental)
  // Uses error difference for velocity form — time-independent, no dt scaling.
  float up = kp * (e - eprev[0]);

  // Step 3: Integral term with anti-windup
  float ui = ki * ki_scale * ts * e;
  // Anti-windup: disable integration when output is saturated
  // This prevents integral windup during saturation
  if (freeze_integrator || uprev + uff >= umax || uprev + uff <= umin) {
    ui = 0;
  }

  // Step 4: Derivative term with low-pass filtering
  // Using DERIVATIVE ON MEASUREMENT (not error) to avoid setpoint kick
  // Raw derivative: -Kd/Ts * [x(k) - 2*x(k-1) + x(k-2)]
  // The negative sign is needed because we're differentiating x instead of e=(xsp-x)
  // When tracking a constant setpoint: d(xsp-x)/dt = -dx/dt, hence the sign change
  float ud = -kd / ts * (x - 2 * xprev[0] + xprev[1]);

  // Apply first-order low-pass filter to reduce noise
  // Filter equation: y(k) = α*y(k-1) + (1-α)*x(k), where α = tau/(tau+ts)
  float udfilt = tau / (tau + ts) * udfiltprev + ts / (tau + ts) * ud;

  // Store diagnostic outputs (for external monitoring)
  last_up = up;
  last_ui = ui;
  last_udfilt = udfilt;

  // Step 5: Calculate total PID output (incremental form)
  // u(k) = u(k-1) + ΔP + I + D_filtered + feedforward
  float u = uprev + up + ui + udfilt + uff;

  // Step 6: Output saturation (hard limits) — applied BEFORE the accumulator
  // is stored, so anti-windup is by back-calculation: uprev records the
  // SATURATED output, never the unbounded pre-clamp value.
  //
  // Storing the pre-clamp u into uprev (the previous behaviour) let the
  // accumulator wind far past [umin,umax], so the output stayed pinned at the
  // saturation limit for extra cycles after the error had already collapsed
  // (the controller had to subtract the accumulated excess before leaving
  // saturation). On the inner motor PID that is the "hold a max command
  // open-loop into the stop" tendon failure family; on the outer joint PID it
  // pins delta_theta at ±30 past error reversal (the DOF1 saturated-P "scatti"
  // limit cycle). Clamping first bounds the accumulator and removes both.
  if (u > umax)
    u = umax;
  else if (u < umin)
    u = umin;

  // Step 7: Update state for next iteration
  eprev[1]   = eprev[0]; // Shift error history
  eprev[0]   = e;
  xprev[1]   = xprev[0]; // Shift measurement history (for derivative on measurement)
  xprev[0]   = x;
  uprev      = u - uff;  // Store SATURATED output without feedforward (bounded -> no windup)
  udfiltprev = udfilt;   // Store filtered derivative

  return u;
}

// ===================================================================
// STATE MANAGEMENT
// ===================================================================

/**
 * Reset all internal state variables to zero
 *
 * Clears:
 * - Error history (eprev[])
 * - Measurement history (xprev[])
 * - Previous output (uprev)
 * - Filtered derivative (udfiltprev)
 *
 * This prevents:
 * - Transients from old state when starting new control
 * - Integral windup carryover
 * - Derivative spikes from stale measurement values
 */
void PID::reset() {
  eprev[0]   = 0;
  eprev[1]   = 0;
  xprev[0]   = 0;
  xprev[1]   = 0;
  uprev      = 0;
  udfiltprev = 0;
}

/**
 * Bumpless transfer: initialize PID state at current operating point
 *
 * Pre-loads internal state so the first control() call produces minimal
 * transient. On the first call after initializeState():
 *
 *   P = Kp * (e - eprev[0]) = Kp * (e - e) = 0           (no jump)
 *   D = -Kd/Ts * (x - 2*x + x) = 0                       (no kick)
 *   I = Ki * Ts * e                                        (only active term)
 *   u = output + 0 + I + 0 = output + small integral step (smooth)
 *
 * This is the standard bumpless transfer technique for activating a
 * controller on a process already at a known operating point.
 */
void PID::initializeState(float measurement, float setpoint, float output) {
  float e = setpoint - measurement;
  eprev[0]   = e;
  eprev[1]   = e;
  xprev[0]   = measurement;
  xprev[1]   = measurement;
  uprev      = output;
  udfiltprev = 0;
}

// ===================================================================
// PARAMETER TUNING
// ===================================================================

/**
 * Update all PID tuning parameters without resetting state (bumpless)
 *
 * In incremental PID form, changing gains mid-operation is inherently safe:
 * - uprev holds the accumulated output → preserved, no discontinuity
 * - eprev/xprev hold measurement history → still valid, no kick
 * - Only the NEXT increment uses the new gains
 *
 * Previously this called reset(), which zeroed xprev[] and caused a massive
 * derivative kick: -Kd/Ts * (x - 0 + 0) = -125 * 300° when changing gains
 * at runtime. This was the root cause of the "sobbalzo" (jolt) observed
 * when adjusting PID parameters from the UI while the joint was active.
 */
void PID::setTunings(float new_kp, float new_ki, float new_kd, float new_tau) {
  kp  = new_kp;
  ki  = new_ki;
  kd  = new_kd;
  tau = new_tau;

  // NOTE: No reset() — state is preserved for bumpless parameter change.
  // The incremental form naturally handles gain changes: the next control()
  // call simply uses the new gains for its increment calculation.
  // If a full state reset is needed, call reset() or initializeState() explicitly.
}
