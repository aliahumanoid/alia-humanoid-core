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
  // Raw derivative: Kd/Ts * [e(k) - 2*e(k-1) + e(k-2)]
  // This is the discrete second derivative (acceleration of error)
  float ud = kd / Ts * (e - 2 * eprev[0] + eprev[1]);
  
  // Apply first-order low-pass filter to reduce noise
  // Filter equation: y(k) = α*y(k-1) + (1-α)*x(k), where α = tau/(tau+Ts)
  float udfilt = tau / (tau + Ts) * udfiltprev + Ts / (tau + Ts) * ud;

  // Step 5: Calculate total PID output (incremental form)
  // u(k) = u(k-1) + ΔP + I + D_filtered + feedforward
  float u = uprev + up + ui + udfilt + uff;

  // Step 6: Update state for next iteration
  eprev[1]   = eprev[0]; // Shift error history
  eprev[0]   = e;
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
 * - Previous output (uprev)
 * - Filtered derivative (udfiltprev)
 * 
 * This prevents:
 * - Transients from old state when starting new control
 * - Integral windup carryover
 * - Derivative spikes from stale error values
 */
void PID::reset() {
  eprev[0]   = 0;
  eprev[1]   = 0;
  uprev      = 0;
  udfiltprev = 0;
}

// ===================================================================
// PARAMETER TUNING
// ===================================================================

/**
 * Update all PID tuning parameters atomically
 * 
 * Updates gains and derivative filter, then resets state to prevent
 * transients from parameter mismatch with old state values.
 * 
 * Note: State reset causes a momentary control discontinuity.
 * For bumpless parameter changes, consider gradual interpolation.
 */
void PID::setTunings(float new_kp, float new_ki, float new_kd, float new_tau) {
  kp  = new_kp;
  ki  = new_ki;
  kd  = new_kd;
  tau = new_tau;

  // Reset internal state to ensure smooth transition
  reset();
}
