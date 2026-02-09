/**
 * @file PID.h
 * @brief Incremental PID controller with anti-windup and derivative filtering
 * 
 * This class implements a discrete-time PID controller in incremental form,
 * optimized for real-time control systems.
 * 
 * Key Features:
 * - Incremental (velocity) form for better numerical stability
 * - Anti-windup mechanism to prevent integral saturation
 * - Low-pass filtered derivative term to reduce noise sensitivity
 * - Configurable output saturation limits
 * - Feedforward support for improved tracking
 * 
 * Algorithm:
 * u(k) = u(k-1) + Kp*[e(k)-e(k-1)] + Ki*Ts*e(k) + Kd_filtered + uff
 * 
 * Where:
 * - e(k) = setpoint - measurement (error)
 * - Kd_filtered uses a first-order low-pass filter with time constant tau
 * - Anti-windup disables integral term when output is saturated
 * 
 * Typical Usage:
 * @code
 * PID positionPID(0.01, 1.0, 0.5, 0.05, 100, -100, 0.02);
 * float control = positionPID.control(target, current, feedforward);
 * @endcode
 */

#ifndef PID_H
#define PID_H

class PID {
private:
  // ===================================================================
  // CONTROLLER PARAMETERS
  // ===================================================================
  
  float Ts;   ///< Sampling period (seconds)
  float kp;   ///< Proportional gain
  float ki;   ///< Integral gain
  float kd;   ///< Derivative gain
  float umax; ///< Upper saturation limit
  float umin; ///< Lower saturation limit
  float tau;  ///< Derivative filter time constant (seconds, 0 = no filtering)

  // ===================================================================
  // INTERNAL STATE
  // ===================================================================

  float eprev[2];   ///< Previous errors: [e(k-1), e(k-2)] (used for proportional increment)
  float xprev[2];   ///< Previous measurements: [x(k-1), x(k-2)] (used for derivative on measurement)
  float uprev;      ///< Previous controller output (without feedforward)
  float udfiltprev; ///< Previous filtered derivative term

public:
  // ===================================================================
  // DIAGNOSTIC OUTPUTS (written every control() call, read externally)
  // ===================================================================

  float last_up = 0;      ///< Last proportional increment: Kp * [e(k) - e(k-1)]
  float last_ui = 0;      ///< Last integral term: Ki * Ts * e(k) (0 when saturated)
  float last_udfilt = 0;  ///< Last filtered derivative term
  // ===================================================================
  // CONSTRUCTOR
  // ===================================================================
  
  /**
   * @brief Construct a new PID controller
   * 
   * @param Ts Sampling period in seconds (e.g., 0.01 for 100 Hz)
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param umax Upper output saturation limit (default: 1)
   * @param umin Lower output saturation limit (default: -1)
   * @param tau Derivative filter time constant in seconds (default: 0 = no filtering)
   *            Recommended: tau ≈ Ts to 10*Ts for noise reduction
   */
  PID(float Ts, float kp, float ki, float kd, float umax = 1, float umin = -1, float tau = 0);

  // ===================================================================
  // CONTROL METHODS
  // ===================================================================
  
  /**
   * @brief Compute PID control output
   *
   * Implements incremental PID with anti-windup and filtered derivative:
   * - Proportional: Kp * [e(k) - e(k-1)]
   * - Integral: Ki * Ts * e(k) (disabled when saturated)
   * - Derivative: Filtered -Kd/Ts * [x(k) - 2*x(k-1) + x(k-2)] (on measurement, not error)
   * - Feedforward: Direct pass-through for model-based control
   *
   * Note: Derivative is computed on MEASUREMENT (not error) to avoid spikes
   * when setpoint changes abruptly. This is the standard "derivative on measurement"
   * technique that provides identical behavior during tracking while eliminating
   * derivative kick on setpoint steps.
   *
   * @param xsp Setpoint (desired value)
   * @param x Current process variable (measurement)
   * @param uff Feedforward control term (default: 0)
   * @return Saturated control output in range [umin, umax]
   *
   * @note Call this method at regular intervals matching the sampling period Ts
   */
  float control(float xsp, float x, float uff = 0);
  
  /**
   * @brief Reset PID internal state to zero
   * 
   * Clears all error history and accumulated outputs.
   * Call this when:
   * - Starting a new control task
   * - Changing setpoint dramatically
   * - Recovering from a disturbance
   */
  void reset();

  // ===================================================================
  // PARAMETER ACCESS
  // ===================================================================
  
  /**
   * @brief Get proportional gain
   * @return Current Kp value
   */
  float getKp() const {
    return kp;
  }
  
  /**
   * @brief Get integral gain
   * @return Current Ki value
   */
  float getKi() const {
    return ki;
  }
  
  /**
   * @brief Get derivative gain
   * @return Current Kd value
   */
  float getKd() const {
    return kd;
  }
  
  /**
   * @brief Get derivative filter time constant
   * @return Current tau value (seconds)
   */
  float getTau() const {
    return tau;
  }

  /**
   * @brief Update derivative filter time constant (bumpless)
   *
   * Changes tau without resetting internal state, allowing smooth
   * real-time adjustment of derivative filtering. Unlike setTunings(),
   * this preserves the filtered derivative history for continuity.
   *
   * @param new_tau New time constant in seconds (0 = no filtering)
   */
  void setTau(float new_tau) {
    tau = new_tau;
  }

  /**
   * @brief Get sampling period
   * @return Current Ts value (seconds)
   */
  float getTs() const {
    return Ts;
  }

  /**
   * @brief Update sampling period
   *
   * Call this when the control loop frequency changes dynamically.
   * Does NOT reset internal state - the PID continues from its current state
   * with the new timing. This allows frequency changes during operation
   * without discontinuities.
   *
   * @param new_ts New sampling period in seconds
   *
   * @note If you also want to reset state, call reset() separately
   */
  void setSamplingPeriod(float new_ts) {
    Ts = new_ts;
  }

  /**
   * @brief Update PID tuning parameters
   * 
   * Updates all PID gains and resets internal state for smooth transition.
   * 
   * @param new_kp New proportional gain
   * @param new_ki New integral gain
   * @param new_kd New derivative gain
   * @param new_tau New derivative filter time constant (seconds)
   * 
   * @note Internal state is reset to prevent transients
   */
  void setTunings(float new_kp, float new_ki, float new_kd, float new_tau);
};

#endif
