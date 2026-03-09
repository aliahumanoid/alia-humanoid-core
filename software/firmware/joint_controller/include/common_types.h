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
  int64_t rawMotorAngle_centideg; // Raw motor angle in 0.01° units (for revolution tracking init)
};

/**
 * @brief Pipelined dual-motor angle read result
 *
 * Used by getMultiAnglePairPipelined() to return both motor angles
 * from a single pipelined CAN transaction (send-send-wait).
 */
struct PipelinedAngleData {
  MultiAngleData dataA;
  MultiAngleData dataB;
  unsigned long totalTime;  // Total pipeline duration in microseconds
};

/**
 * @brief Motor torque command response data (from 0xA1 response)
 *
 * Contains motor state returned after a torque command:
 * temperature, torque current (iq), speed, and encoder position.
 * Used by setTorquePairPipelined() to combine torque send with angle read.
 */
struct TorqueResponseData {
  float angle;              // Tracked multi-turn angle in output degrees (NAN if tracking not init)
  int16_t torqueCurrent;    // Iq current feedback (raw, -2048 to 2048)
  int16_t motorSpeed;       // Motor speed in dps (raw motor shaft)
  int8_t temperature;       // Motor temperature in °C
  uint16_t encoder;         // Raw motor-side encoder value (range depends on resolution, 0-65535 for 18-bit)
  bool valid;               // True if response was received and parsed
};

/**
 * @brief Pipelined dual-motor torque send + response read result
 *
 * Used by setTorquePairPipelined() to send torque commands to both motors
 * and read back their state in a single pipelined CAN transaction.
 *
 * Phase 1 (shadow mode): used alongside 0x92 reads for revolution tracking
 * validation. Phase 2 (future): can replace separate getMultiAnglePairPipelined()
 * + setTorque() calls, reducing CAN transactions from 4 to 2 per cycle.
 */
struct PipelinedTorqueResponseData {
  TorqueResponseData dataA;
  TorqueResponseData dataB;
  uint32_t totalTime;       // Total operation time in µs
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
