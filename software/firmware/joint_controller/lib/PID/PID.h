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
  
  float eprev[2];   ///< Previous errors: [e(k-1), e(k-2)]
  float uprev;      ///< Previous controller output (without feedforward)
  float udfiltprev; ///< Previous filtered derivative term

public:
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
   * - Derivative: Filtered Kd/Ts * [e(k) - 2*e(k-1) + e(k-2)]
   * - Feedforward: Direct pass-through for model-based control
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
