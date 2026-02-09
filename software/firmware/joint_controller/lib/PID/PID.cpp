/**
 * @file PID.cpp
 * @brief Implementation of incremental PID controller
 * 
 * See PID.h for detailed documentation.
 */

#include "PID.h"

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
float PID::control(float xsp, float x, float uff) {
  // Step 1: Calculate error
  float e = xsp - x;

  // Step 2: Proportional term (incremental)
  // Uses error difference for velocity form
  float up = kp * (e - eprev[0]);

  // Step 3: Integral term with anti-windup
  float ui = ki * Ts * e;
  // Anti-windup: disable integration when output is saturated
  // This prevents integral windup during saturation
  if (uprev + uff >= umax || uprev + uff <= umin) {
    ui = 0;
  }

  // Step 4: Derivative term with low-pass filtering
  // Using DERIVATIVE ON MEASUREMENT (not error) to avoid setpoint kick
  // Raw derivative: -Kd/Ts * [x(k) - 2*x(k-1) + x(k-2)]
  // The negative sign is needed because we're differentiating x instead of e=(xsp-x)
  // When tracking a constant setpoint: d(xsp-x)/dt = -dx/dt, hence the sign change
  float ud = -kd / Ts * (x - 2 * xprev[0] + xprev[1]);
  
  // Apply first-order low-pass filter to reduce noise
  // Filter equation: y(k) = α*y(k-1) + (1-α)*x(k), where α = tau/(tau+Ts)
  float udfilt = tau / (tau + Ts) * udfiltprev + Ts / (tau + Ts) * ud;

  // Store diagnostic outputs (for external monitoring)
  last_up = up;
  last_ui = ui;
  last_udfilt = udfilt;

  // Step 5: Calculate total PID output (incremental form)
  // u(k) = u(k-1) + ΔP + I + D_filtered + feedforward
  float u = uprev + up + ui + udfilt + uff;

  // Step 6: Update state for next iteration
  eprev[1]   = eprev[0]; // Shift error history
  eprev[0]   = e;
  xprev[1]   = xprev[0]; // Shift measurement history (for derivative on measurement)
  xprev[0]   = x;
  uprev      = u - uff;  // Store output without feedforward
  udfiltprev = udfilt;   // Store filtered derivative

  // Step 7: Output saturation (hard limits)
  if (u > umax)
    u = umax;
  else if (u < umin)
    u = umin;

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
