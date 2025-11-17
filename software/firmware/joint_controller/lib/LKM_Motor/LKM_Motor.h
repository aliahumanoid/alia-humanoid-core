/**
 * @file LKM_Motor.h
 * @brief LKM series motor driver with CAN bus communication
 * 
 * This class provides complete control over LKM series motors via CAN bus.
 * It implements the full LKM protocol including:
 * 
 * MOTOR CONTROL:
 * - Position control (multi-loop, single-loop, incremental)
 * - Speed control (open-loop and within position commands)
 * - Torque control (direct Iq current control)
 * - Motor on/off/stop commands
 * 
 * ENCODER MANAGEMENT:
 * - Encoder reading (raw, single-loop, multi-loop)
 * - Encoder offset calibration (zero position setting)
 * - Encoder inversion (for motors mounted backwards)
 * - Reduction gear compensation
 * 
 * STATE MONITORING:
 * - Temperature, voltage, error states
 * - Current readings (torque current, phase currents)
 * - Speed and position feedback
 * 
 * PID CONFIGURATION:
 * - Read/write angle, speed, and Iq PID parameters
 * - RAM (temporary) and ROM (persistent) storage
 * 
 * ANGLE CALCULATION:
 * The class handles automatic angle transformation:
 * 1. Raw encoder reading (0-16383 → 0-360°)
 * 2. Apply reduction gear (motor angle / reduction ratio)
 * 3. Subtract encoder offset (for zero calibration)
 * 4. Apply inversion if configured (for reversed mounting)
 * 
 * Final angle = ((raw_angle / reduction) - offset) * invert_sign
 */

#ifndef LKM_MOTOR_H
#define LKM_MOTOR_H

#include <Arduino.h>
#include <initializer_list>
#include <mcp_can.h>
#include "mcp_can_guard.h"

// Use common type definition to avoid circular dependencies
#include <common_types.h>

// ===================================================================
// LKM_MOTOR CLASS
// ===================================================================

/**
 * @brief LKM motor driver with CAN bus interface
 *
 * Provides complete control interface for LKM series motors with:
 * - Position / speed / torque control modes
 * - State monitoring and error handling
 * - Encoder calibration and inversion support
 * - PID parameter configuration
 * - Multi-motor synchronized control
 */
class LKM_Motor {
public:
  // ---------------------------------------------------------------
  // TYPE DEFINITIONS
  // ---------------------------------------------------------------
  
  // Use the common MultiAngleData type instead of defining our own
  using MultiAngleData = ::MultiAngleData;

  // ---------------------------------------------------------------
  // INITIALIZATION
  // ---------------------------------------------------------------
  
  /**
   * @brief Constructor - configure motor instance
   * @param canInterface Pointer to the CAN interface (MCP_CAN)
   * @param motorID Motor CAN ID (1-32, corresponds to 0x140 + ID)
   * @param reductionGear Motor reduction ratio (e.g. 10.0 means 10:1 reduction)
   * @param invert If true, invert encoder direction (for reversed mounting)
   *
   * The reduction gear ratio is used to convert motor angles to output angles:
   * output_angle = motor_angle / reduction_gear
   * 
   * Encoder inversion flips all angles and torques to match the desired
   * positive direction when the motor is physically mounted backwards.
   */
  LKM_Motor(MCP_CAN *canInterface, unsigned int motorID, float reductionGear = 10.0,
            bool invert = false);

  /**
   * @brief Initialize motor (assumes CAN is already initialized)
   * 
   * Performs any necessary setup after construction.
   * The CAN bus must already be initialized before calling this.
   */
  void init();

  // ---------------------------------------------------------------
  // CONFIGURATION
  // ---------------------------------------------------------------
  
  /**
   * @brief Set motor CAN ID
   * @param id Motor ID (1-32)
   */
  void setMotorID(unsigned int id);
  
  /**
   * @brief Get motor CAN ID
   * @return Motor ID (1-32)
   */
  unsigned int getMotorID() const;
  
  /**
   * @brief Set reduction gear ratio
   * @param gear Reduction ratio (e.g. 10.0 for 10:1)
   */
  void setReductionGear(float gear);
  
  /**
   * @brief Get reduction gear ratio
   * @return Current reduction ratio
   */
  float getReductionGear() const;

  /**
   * @brief Set encoder inversion
   * @param invert If true, invert encoder positive direction
   *
   * When inversion is active:
   * - Read angles are multiplied by -1
   * - Position commands are multiplied by -1
   * - Torque commands are multiplied by -1
   */
  void setInvertEncoder(bool invert);

  // ---------------------------------------------------------------
  // BASIC MOTOR CONTROL
  // ---------------------------------------------------------------
  
  /**
   * @brief Enable motor (CAN command 0x88)
   * @return true if command sent successfully
   */
  bool motorOn();
  
  /**
   * @brief Disable motor (CAN command 0x80)
   * @return true if command sent successfully
   */
  bool motorOff();
  
  /**
   * @brief Stop motor immediately (CAN command 0x81)
   * @return true if command sent successfully
   */
  bool motorStop();
  
  /**
   * @brief Set motor speed (CAN command 0xA2)
   * @param speed Speed in degrees/second (°/s)
   * @return true if command sent successfully
   */
  bool setSpeed(float speed);

  /**
   * @brief Set motor torque (CAN command 0xA1)
   * @param torque Torque value (-2048 to 2048, approximately -33A to 33A)
   * @return true if command sent successfully
   *
   * When encoder inversion is active, the torque sign is automatically
   * flipped to maintain the intended direction.
   */
  bool setTorque(int torque);

  // ---------------------------------------------------------------
  // POSITION CONTROL
  // ---------------------------------------------------------------

  /**
   * @brief Multi-loop angle control (CAN command 0xA3)
   * @param angleControl Target angle in degrees (supports multiple rotations)
   * @return true if command sent successfully
   *
   * The angle is automatically processed through:
   * 1. Encoder offset application
   * 2. Encoder inversion (if active)
   * 3. Reduction ratio multiplication
   */
  bool sendMultiLoopAngle1Command(float angleControl);

  /**
   * @brief Multi-loop angle control with max speed (CAN command 0xA4)
   * @param angleControl Target angle in degrees (supports multiple rotations)
   * @param maxSpeed Maximum speed in 0.01 dps units
   * @return true if command sent successfully
   */
  bool sendMultiLoopAngle2Command(float angleControl, uint16_t maxSpeed);

  /**
   * @brief Single-loop angle control (CAN command 0xA5)
   * @param spinDirection Rotation direction (0=shortest, 1=CW, 2=CCW)
   * @param angleControl Target angle in degrees (0-360°)
   * @return true if command sent successfully
   *
   * Encoder inversion is applied automatically if configured.
   */
  bool sendSingleLoopAngle1Command(uint8_t spinDirection, float angleControl);

  /**
   * @brief Single-loop angle control with max speed (CAN command 0xA6)
   * @param spinDirection Rotation direction (0=shortest, 1=CW, 2=CCW)
   * @param maxSpeed Maximum speed in 0.01 dps units
   * @param angleControl Target angle in degrees (0-360°)
   * @return true if command sent successfully
   */
  bool sendSingleLoopAngle2Command(uint8_t spinDirection, uint16_t maxSpeed, float angleControl);

  /**
   * @brief Incremental angle control (CAN command 0xA7)
   * @param angleIncrement Angle increment in degrees (positive or negative)
   * @return true if command sent successfully
   *
   * Encoder inversion is applied automatically if configured.
   */
  bool sendIncrementAngle1Command(float angleIncrement);

  /**
   * @brief Incremental angle control with max speed (CAN command 0xA8)
   * @param angleIncrement Angle increment in degrees (positive or negative)
   * @param maxSpeed Maximum speed in 0.01 dps units
   * @return true if command sent successfully
   */
  bool sendIncrementAngle2Command(float angleIncrement, uint16_t maxSpeed);

  // ---------------------------------------------------------------
  // PID CONFIGURATION
  // ---------------------------------------------------------------
  
  /**
   * @brief Read PID parameters from motor (CAN command 0x30)
   * @return true if command sent successfully
   */
  bool readPIDParameters();
  
  /**
   * @brief Write PID parameters to RAM (CAN command 0x31)
   * @param anglePidKp Angle loop proportional gain
   * @param anglePidKi Angle loop integral gain
   * @param speedPidKp Speed loop proportional gain
   * @param speedPidKi Speed loop integral gain
   * @param iqPidKp Current loop proportional gain
   * @param iqPidKi Current loop integral gain
   * @return true if command sent successfully
   * 
   * RAM parameters are lost on power cycle.
   */
  bool writePIDParametersRAM(byte anglePidKp, byte anglePidKi, byte speedPidKp, byte speedPidKi,
                             byte iqPidKp, byte iqPidKi);
  
  /**
   * @brief Write PID parameters to ROM (CAN command 0x32)
   * @param anglePidKp Angle loop proportional gain
   * @param anglePidKi Angle loop integral gain
   * @param speedPidKp Speed loop proportional gain
   * @param speedPidKi Speed loop integral gain
   * @param iqPidKp Current loop proportional gain
   * @param iqPidKi Current loop integral gain
   * @return true if command sent successfully
   * 
   * ROM parameters persist across power cycles.
   */
  bool writePIDParametersROM(byte anglePidKp, byte anglePidKi, byte speedPidKp, byte speedPidKi,
                             byte iqPidKp, byte iqPidKi);

  /**
   * @brief Read motor acceleration from motor (CAN command 0x33)
   * @return true if command sent successfully
   */
  bool readAcceleration();

  /**
   * @brief Write acceleration to RAM (CAN command 0x34)
   * @param acceleration Acceleration value in dps²
   * @return true if command sent successfully
   */
  bool writeAccelerationRAM(int32_t acceleration);

  // ---------------------------------------------------------------
  // ENCODER OPERATIONS
  // ---------------------------------------------------------------
  
  /**
   * @brief Read raw encoder value (CAN command 0x90)
   * @return true if command sent successfully
   */
  bool readEncoder();
  
  /**
   * @brief Set encoder offset in ROM (CAN command 0x91)
   * @param encoderOffset Encoder offset value (0-16383)
   * @return true if command sent successfully
   */
  bool setEncoderOffsetROM(uint16_t encoderOffset);
  
  /**
   * @brief Set current position as zero in ROM (CAN command 0x19)
   * @return true if command sent successfully
   */
  bool setCurrentPositionAsZeroROM();

  /**
   * @brief Read multi-loop angle (CAN command 0x92)
   * @return true if command sent successfully
   */
  bool readMultiAngleLoop();
  
  /**
   * @brief Read single-loop angle (CAN command 0x94)
   * @return true if command sent successfully
   */
  bool readSingleAngleLoop();
  
  /**
   * @brief Clear accumulated angle (CAN command 0x95)
   * @return true if command sent successfully
   */
  bool clearAngleLoop();

  // ---------------------------------------------------------------
  // STATE MONITORING
  // ---------------------------------------------------------------
  
  /**
   * @brief Read motor state 1 (CAN command 0x9A)
   * @return true if command sent successfully
   * 
   * Updates: motorTemperature, motorVoltage, motorErrorState
   */
  bool readMotorState();
  
  /**
   * @brief Read motor state 2 (CAN command 0x9C)
   * @return true if command sent successfully
   * 
   * Updates: motorTemperature2, motorTorqueCurrent, motorSpeed, motorEncoderPosition
   */
  bool readMotorState2();
  
  /**
   * @brief Read motor state 3 (CAN command 0x9D)
   * @return true if command sent successfully
   * 
   * Updates: motorTemperature3, phaseACurrent, phaseBCurrent, phaseCCurrent
   */
  bool readMotorState3();
  
  /**
   * @brief Clear motor error flags (CAN command 0x9B)
   * @return true if command sent successfully
   */
  bool clearMotorErrors();

  // ---------------------------------------------------------------
  // MULTI-MOTOR CONTROL
  // ---------------------------------------------------------------
  
  /**
   * @brief Send torque commands to 4 motors simultaneously (CAN ID 0x280)
   * @param can CAN interface to use
   * @param iq1 Torque for motor 1 (-2048 to 2048)
   * @param iq2 Torque for motor 2 (-2048 to 2048)
   * @param iq3 Torque for motor 3 (-2048 to 2048)
   * @param iq4 Torque for motor 4 (-2048 to 2048)
   * @return true if command sent successfully
   * 
   * Allows synchronized torque control of 4 motors with a single CAN message.
   */
  static bool sendMultiMotorTorqueCommand(MCP_CAN *can, int16_t iq1, int16_t iq2, int16_t iq3,
                                          int16_t iq4);

  // ---------------------------------------------------------------
  // SYNCHRONOUS READ OPERATIONS
  // ---------------------------------------------------------------
  
  /**
   * @brief Read multi-loop angle synchronously (blocking)
   * @param applyOffset If true, apply encoder offset and inversion
   * @return MultiAngleData with angle in degrees and wait time in µs
   *
   * Behavior with applyOffset = true:
   * 1. Subtract encoder offset from raw angle
   * 2. Apply inversion if configured (multiply by -1)
   * 3. Apply reduction gear ratio
   *
   * Behavior with applyOffset = false:
   * - Return raw motor angle (no offset, no inversion)
   * - Used internally by zeroEncoderOffset()
   */
  MultiAngleData getMultiAngleSync(bool applyOffset = true);

  /**
   * @brief Read single-loop angle synchronously (blocking)
   * @return MultiAngleData with angle in degrees and wait time in µs
   *
   * Returns angle in 0-360° range with inversion and reduction applied.
   */
  MultiAngleData getSingleAngleSync();

  /**
   * @brief Read raw encoder value synchronously (blocking)
   * @return Raw encoder count (0-16383, corresponding to 0-360°)
   *
   * Returns the absolute encoder value without any processing:
   * - No offset applied
   * - No inversion applied
   * - No reduction gear applied
   */
  uint16_t getEncoderRawSync();

  // ---------------------------------------------------------------
  // CALIBRATION & UTILITIES
  // ---------------------------------------------------------------
  
  /**
   * @brief Set maximum torque limit
   * @param maxTorque Maximum torque value (-2048 to 2048)
   */
  void setMaxTorque(int16_t maxTorque);

  /**
   * @brief Zero encoder offset at current position
   *
   * Reads the current encoder position (raw, without offset/inversion)
   * and stores it as the new encoder offset. After this call, the
   * current position becomes the reference 0°.
   * 
   * This is the typical way to set the zero position during calibration.
   */
  void zeroEncoderOffset();

  /**
   * @brief Set encoder offset to define zero at a specific angle
   * @param actual_angle The angle (in degrees) that should be considered 0°
   *
   * Computes the encoder offset needed to make the specified angle
   * correspond to 0°. Useful when the zero position is known to be
   * at a specific angle value.
   * 
   * Example: if motor is at 45° and you call nonzeroEncoderOffset(45.0),
   * then 45° becomes the new 0° reference.
   */
  void nonzeroEncoderOffset(float actual_angle);

  /**
   * @brief Stop motor and wait for it to halt (blocking)
   * @param timeout_ms Maximum wait time in milliseconds
   * @return true if motor stopped successfully, false on timeout
   * 
   * Sends stop command and polls motor state until velocity reaches zero
   * or timeout expires.
   */
  bool motorStopSync(int timeout_ms = 1000);

  // ===================================================================
  // PUBLIC STATE VARIABLES
  // ===================================================================
  //
  // These variables hold the most recent readings from the motor.
  // They are updated when the corresponding read commands are called.
  // ===================================================================

  // ---------------------------------------------------------------
  // ENCODER CONFIGURATION
  // ---------------------------------------------------------------
  
  /**
   * @brief Encoder inversion flag
   *
   * When true, the encoder direction is inverted:
   * - Read angles are multiplied by -1
   * - Position commands are multiplied by -1
   * - Torque commands are multiplied by -1
   * 
   * Used when the motor is mounted backwards relative to the
   * desired positive rotation direction.
   */
  bool invertEncoder;

  /**
   * @brief Encoder offset in degrees
   *
   * This value is subtracted from the raw angle to obtain the relative angle.
   * 
   * Example: if offsetEncoder = 90°, then when the motor reads 90°,
   * the reported angle will be 0°.
   */
  float offsetEncoder;

  /**
   * @brief Set encoder offset
   * @param offset Offset value in degrees
   *
   * The offset is subtracted from raw angles in all read operations.
   */
  void setOffsetEncoder(float offset);

  /**
   * @brief Get current encoder offset
   * @return Current offset in degrees
   */
  float getOffsetEncoder() const;

  // ---------------------------------------------------------------
  // MOTOR STATE 1 (from command 0x9A)
  // ---------------------------------------------------------------
  
  int8_t motorTemperature;  ///< Motor temperature (°C)
  float motorVoltage;       ///< Motor supply voltage (V, 0.1V per LSB)
  uint8_t motorErrorState;  ///< Error state flags (bitfield)

  // ---------------------------------------------------------------
  // MOTOR STATE 2 (from command 0x9C)
  // ---------------------------------------------------------------
  
  int8_t motorTemperature2;      ///< Motor temperature (°C)
  int16_t motorTorqueCurrent;    ///< Torque current (Iq)
  int16_t motorSpeed;            ///< Motor speed (dps)
  uint16_t motorEncoderPosition; ///< Encoder position (0-16383)

  // ---------------------------------------------------------------
  // MOTOR STATE 3 (from command 0x9D)
  // ---------------------------------------------------------------
  
  int8_t motorTemperature3; ///< Motor temperature (°C)
  int16_t phaseACurrent;    ///< Phase A current (mA)
  int16_t phaseBCurrent;    ///< Phase B current (mA)
  int16_t phaseCCurrent;    ///< Phase C current (mA)

private:
  // ---------------------------------------------------------------
  // PRIVATE MEMBERS
  // ---------------------------------------------------------------
  
  MCP_CAN *_can;            ///< Pointer to CAN interface
  unsigned int _motorID;    ///< Motor CAN ID (1-32)
  float _reductionGear;     ///< Reduction ratio (e.g. 10.0 for 10:1)
};

#endif // LKM_MOTOR_H
