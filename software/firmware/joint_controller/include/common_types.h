#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

/**
 * @brief Simple data structure for motor angle data
 * 
 * Used for encoder readings and motor position feedback.
 */
struct MultiAngleData {
  float angle;            // Actual angle in degrees (after reduction gear division)
  unsigned long waitTime; // Response time in microseconds
};

/**
 * @brief Basic PID controller gains (3-term)
 * 
 * Generic 3-term PID gains used in flash storage structures
 * and basic control loops. For motor control with filtering,
 * see MotorPIDParams in JointConfig.h which adds tau parameter.
 */
struct PIDGains {
  float kp; // Proportional gain
  float ki; // Integral gain
  float kd; // Derivative gain
};

#endif // COMMON_TYPES_H
