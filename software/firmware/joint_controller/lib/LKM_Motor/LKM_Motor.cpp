/**
 * @file LKM_Motor.cpp
 * @brief Implementation of LKM motor driver with CAN bus communication
 * 
 * See LKM_Motor.h for detailed documentation.
 */

#include "LKM_Motor.h"
#include <debug.h>

// ===================================================================
// GLOBAL VARIABLES
// ===================================================================

// Note: _maxTorque moved to LKM_Motor class member (was global, shared by all instances)

// ===================================================================
// CONSTRUCTOR & INITIALIZATION
// ===================================================================

/**
 * Constructor - initialize motor instance
 */
LKM_Motor::LKM_Motor(MCP_CAN *canInterface, unsigned int motorID, float reductionGear,
                     bool invert) {
  _can           = canInterface;
  _motorID       = motorID;
  _reductionGear = reductionGear;
  _maxTorque     = 2048;  // Default, overridden by setMaxTorque()
  invertEncoder  = invert;
  offsetEncoder  = 0.0;
}

/**
 * Initialize motor (CAN bus must be already initialized)
 */
void LKM_Motor::init() {
  LOG_C1_DEBUG("LKM_Motor initialized with ID: " + String(_motorID));
}

// ===================================================================
// CONFIGURATION - SETTERS & GETTERS
// ===================================================================

void LKM_Motor::setMotorID(unsigned int id) {
  _motorID = id;
}

unsigned int LKM_Motor::getMotorID() const {
  return _motorID;
}

void LKM_Motor::setOffsetEncoder(float offset) {
  offsetEncoder = offset;
}

float LKM_Motor::getOffsetEncoder() const {
  return offsetEncoder;
}

void LKM_Motor::setReductionGear(float gear) {
  _reductionGear = gear;
}

float LKM_Motor::getReductionGear() const {
  return _reductionGear;
}

void LKM_Motor::setInvertEncoder(bool invert) {
  invertEncoder = invert;
}

// ===================================================================
// BASIC MOTOR CONTROL
// ===================================================================

/**
 * Enable motor (CAN command 0x88)
 */
bool LKM_Motor::motorOn() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x88, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO("Command: Motor ON sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending Motor ON.");
  return false;
}

/**
 * Disable motor (CAN command 0x80)
 */
bool LKM_Motor::motorOff() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x80, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO("Command: Motor OFF sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending Motor OFF.");
  return false;
}

/**
 * Stop motor immediately (CAN command 0x81)
 */
bool LKM_Motor::motorStop() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x81, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO("Command: Motor STOP sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending Motor STOP.");
  return false;
}

/**
 * Set motor speed (CAN command 0xA2)
 */
bool LKM_Motor::setSpeed(float speed) {
  // Convert speed from deg/s to 0.01 dps
  int32_t speedCentesimi = static_cast<int32_t>(speed * 100.0);
  // Apply reduction ratio
  speedCentesimi         = static_cast<int32_t>(speedCentesimi * _reductionGear);
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA2, 0, 0, 0, 0, 0, 0, 0};
  buf[4]                 = speedCentesimi & 0xFF;
  buf[5]                 = (speedCentesimi >> 8) & 0xFF;
  buf[6]                 = (speedCentesimi >> 16) & 0xFF;
  buf[7]                 = (speedCentesimi >> 24) & 0xFF;
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO(String("SET_SPEED sent with value: ") + String(speedCentesimi));
    return true;
  }
  LOG_C1_ERROR("Error sending SET_SPEED.");
  return false;
}

/**
 * Set motor torque (CAN command 0xA1)
 * 
 * Torque value range: -2048 to 2048 (approximately -33A to 33A)
 * Torque is automatically limited to _maxTorque and inverted if encoder is inverted.
 */
bool LKM_Motor::setTorque(int torque) {
  // Limit torque to maximum value
  if (torque > _maxTorque) {
    torque = _maxTorque;
  } else if (torque < -_maxTorque) {
    torque = -_maxTorque;
  }

  // Invert torque direction if encoder is inverted
  if (invertEncoder) {
    torque = -torque;
  }

  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA1, 0, 0, 0, 0, 0, 0, 0};
  buf[4]                 = torque & 0xFF;
  buf[5]                 = (torque >> 8) & 0xFF;
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    // Optional debug: Serial.print("SET_TORQUE sent with value (limited): ");
    // SERIAL_C1_COM_LN(torque);
    return true;
  }
  LOG_C1_ERROR("Error sending SET_TORQUE.");
  return false;
}

/**
 * Set maximum torque limit
 */
void LKM_Motor::setMaxTorque(int16_t maxTorque) {
  _maxTorque = maxTorque;
}

// ===================================================================
// CALIBRATION & UTILITIES
// ===================================================================

/**
 * Zero encoder offset at current position
 */
void LKM_Motor::zeroEncoderOffset() {
  offsetEncoder = -getMultiAngleSync(false).angle;
  LOG_C1_INFO(String("Encoder offset set to: ") + String(offsetEncoder));
}

/**
 * Set encoder offset to define zero at a specific angle
 */
void LKM_Motor::nonzeroEncoderOffset(float actual_angle) {
  offsetEncoder = actual_angle - getMultiAngleSync(false).angle;
  LOG_C1_INFO(String("Encoder offset set to: ") + String(offsetEncoder));
}

/**
 * Stop motor and wait for it to halt (blocking)
 */
bool LKM_Motor::motorStopSync(int timeout_ms) {
  // Send stop command
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x81, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) != CAN_OK) {
    LOG_C1_ERROR("Error sending Motor STOP.");
    return false;
  }
  LOG_C1_INFO("Command: Motor STOP sent.");

  // Wait for motor to stop (poll speed until below threshold)
  unsigned long startTime   = millis();
  const int SPEED_THRESHOLD = 10; // Speed threshold for "stopped" (0.01 dps units)

  while (millis() - startTime < timeout_ms) {
    // Read motor state 2 (contains speed)
    unsigned char cmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
    if (_can->sendMsgBuf(targetID, 0, 8, cmd) == CAN_OK) {
      delay(1); // Small delay for motor response

      if (_can->checkReceive() == CAN_MSGAVAIL) {
        unsigned long canId;
        unsigned char len;
        unsigned char rcvBuf[8];
        if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
          if (rcvBuf[0] == 0x9C) {
            // Extract motor speed from response
            int16_t currentSpeed = ((int16_t)rcvBuf[5] << 8) | rcvBuf[4];

            // Check if speed is below threshold
            if (abs(currentSpeed) < SPEED_THRESHOLD) {
              LOG_C1_INFO("Motor successfully stopped.");
              return true;
            }
          }
        }
      }
    }
    delay(10); // Pause between polls
  }

  LOG_C1_WARN("Timeout: motor may not have fully stopped.");
  return false;
}

// ===================================================================
// STATE MONITORING
// ===================================================================

/**
 * Read motor state 1 (CAN command 0x9A)
 * Updates: motorTemperature, motorVoltage, motorErrorState
 */
bool LKM_Motor::readMotorState() {

  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x9A, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) != CAN_OK) {
    LOG_C1_ERROR("Error sending READ_STATE.");
    return false;
  }
  LOG_C1_INFO("READ_STATE command sent.");

  unsigned long startTime = millis();
  while (millis() - startTime < 200) {
    if (_can->checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      unsigned char len;
      unsigned char rcvBuf[8];
      if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
        if (rcvBuf[0] == 0x9A) {
          // Parse state 1 data
          motorTemperature    = (int8_t)rcvBuf[1];
          uint16_t voltageRaw = ((uint16_t)rcvBuf[4] << 8) | rcvBuf[3];
          motorVoltage        = voltageRaw / 10.0;
          motorErrorState     = rcvBuf[7];

          LOG_C1_DEBUG(String("READ_STATE: Temp=") + String(motorTemperature) + " °C, Voltage=" +
                    String(motorVoltage) + " V, Errors=" + String(motorErrorState, BIN));
          return true;
        }
      }
    }
  }
  LOG_C1_WARN("Timeout: no response from READ_STATE.");
  return false;
}

/**
 * Clear motor error flags (CAN command 0x9B)
 */
bool LKM_Motor::clearMotorErrors() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x9B, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO("CLEAR_ERRORS command sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending CLEAR_ERRORS.");
  return false;
}

/**
 * Read motor acceleration (CAN command 0x33)
 */
bool LKM_Motor::readAcceleration() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x33, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO("READ_ACCELERATION command sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending READ_ACCELERATION.");
  return false;
}

// ===================================================================
// POSITION CONTROL
// ===================================================================

/**
 * Send multi-loop angle control command (CAN command 0xA3)
 * @param angleControl Target angle in degrees (can be > 360° for multiple rotations)
 * @return true if the command was sent successfully
 *
 * Angle value processing order:
 * 1. Add offset (compensate mechanical zero)
 * 2. Apply inversion if enabled
 * 3. Multiply by reduction ratio
 * 4. Convert to hundredths of a degree for the CAN protocol
 */
bool LKM_Motor::sendMultiLoopAngle1Command(float angleControl) {
  int8_t invert                 = invertEncoder ? -1 : 1;
  int32_t angleControlCentesimi = static_cast<int32_t>((angleControl + (invert * offsetEncoder)) *
                                                       invert * _reductionGear * 100.0);

  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA3, 0, 0, 0, 0, 0, 0, 0};
  buf[4]                 = (uint8_t)(angleControlCentesimi & 0xFF);
  buf[5]                 = (uint8_t)((angleControlCentesimi >> 8) & 0xFF);
  buf[6]                 = (uint8_t)((angleControlCentesimi >> 16) & 0xFF);
  buf[7]                 = (uint8_t)((angleControlCentesimi >> 24) & 0xFF);
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO(String("ML_ANGLE1 sent: angleControl=") + String(angleControlCentesimi));
    return true;
  }
  LOG_C1_ERROR("Error sending ML_ANGLE1.");
  return false;
}

/**
 * Send multi-loop angle control with max speed (CAN command 0xA4)
 */
bool LKM_Motor::sendMultiLoopAngle2Command(float angleControl, uint16_t maxSpeed) {
  int8_t invert                 = invertEncoder ? -1 : 1;
  int32_t angleControlCentesimi = static_cast<int32_t>((angleControl + (invert * offsetEncoder)) *
                                                       invert * _reductionGear * 100.0);

  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA4, 0, 0, 0, 0, 0, 0, 0};
  buf[2]                 = (uint8_t)(maxSpeed & 0xFF);
  buf[3]                 = (uint8_t)((maxSpeed >> 8) & 0xFF);
  buf[4]                 = (uint8_t)(angleControlCentesimi & 0xFF);
  buf[5]                 = (uint8_t)((angleControlCentesimi >> 8) & 0xFF);
  buf[6]                 = (uint8_t)((angleControlCentesimi >> 16) & 0xFF);
  buf[7]                 = (uint8_t)((angleControlCentesimi >> 24) & 0xFF);
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO(String("ML_ANGLE2 sent: angleControl=") + String(angleControlCentesimi) +
             " maxSpeed=" + String(maxSpeed));
    return true;
  }
  LOG_C1_ERROR("Error sending ML_ANGLE2.");
  return false;
}

/**
 * Send single-loop angle control (CAN command 0xA5)
 */
bool LKM_Motor::sendSingleLoopAngle1Command(uint8_t spinDirection, float angleControl) {
  int32_t angleControlCentesimi = static_cast<int32_t>(angleControl * 100.0);
  // Apply reduction ratio
  angleControlCentesimi = static_cast<int32_t>(angleControlCentesimi * _reductionGear);
  if (invertEncoder) {
    angleControlCentesimi = -angleControlCentesimi;
  }
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA5, 0, 0, 0, 0, 0, 0, 0};
  buf[1]                 = spinDirection;
  buf[4]                 = (uint8_t)(angleControlCentesimi & 0xFF);
  buf[5]                 = (uint8_t)((angleControlCentesimi >> 8) & 0xFF);
  buf[6]                 = (uint8_t)((angleControlCentesimi >> 16) & 0xFF);
  buf[7]                 = (uint8_t)((angleControlCentesimi >> 24) & 0xFF);
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO(String("SL_ANGLE1 sent: spinDirection=") + String(spinDirection) +
             " angleControl=" + String(angleControlCentesimi));
    return true;
  }
  LOG_C1_ERROR("Error sending SL_ANGLE1.");
  return false;
}

/**
 * Send single-loop angle control with max speed (CAN command 0xA6)
 */
bool LKM_Motor::sendSingleLoopAngle2Command(uint8_t spinDirection, uint16_t maxSpeed,
                                            float angleControl) {
  int32_t angleControlCentesimi = static_cast<int32_t>(angleControl * 100.0);
  // Apply reduction ratio
  angleControlCentesimi = static_cast<int32_t>(angleControlCentesimi * _reductionGear);
  if (invertEncoder) {
    angleControlCentesimi = -angleControlCentesimi;
  }
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA6, 0, 0, 0, 0, 0, 0, 0};
  buf[1]                 = spinDirection;
  buf[2]                 = (uint8_t)(maxSpeed & 0xFF);
  buf[3]                 = (uint8_t)((maxSpeed >> 8) & 0xFF);
  buf[4]                 = (uint8_t)(angleControlCentesimi & 0xFF);
  buf[5]                 = (uint8_t)((angleControlCentesimi >> 8) & 0xFF);
  buf[6]                 = (uint8_t)((angleControlCentesimi >> 16) & 0xFF);
  buf[7]                 = (uint8_t)((angleControlCentesimi >> 24) & 0xFF);
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO(String("SL_ANGLE2 sent: spinDirection=") + String(spinDirection) +
             " maxSpeed=" + String(maxSpeed) + " angleControl=" + String(angleControlCentesimi));
    return true;
  }
  LOG_C1_ERROR("Error sending SL_ANGLE2.");
  return false;
}

/**
 * Send incremental angle control (CAN command 0xA7)
 */
bool LKM_Motor::sendIncrementAngle1Command(float angleIncrement) {
  int32_t angleIncrementCentesimi = static_cast<int32_t>(angleIncrement * _reductionGear * 100.0);
  if (invertEncoder) {
    angleIncrementCentesimi = -angleIncrementCentesimi;
  }
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA7, 0, 0, 0, 0, 0, 0, 0};
  buf[4]                 = (uint8_t)(angleIncrementCentesimi & 0xFF);
  buf[5]                 = (uint8_t)((angleIncrementCentesimi >> 8) & 0xFF);
  buf[6]                 = (uint8_t)((angleIncrementCentesimi >> 16) & 0xFF);
  buf[7]                 = (uint8_t)((angleIncrementCentesimi >> 24) & 0xFF);
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_INFO(String("INC_ANGLE1 sent: angleIncrement=") + String(angleIncrementCentesimi));
    return true;
  }
  LOG_C1_ERROR("ERROR sending INC_ANGLE1.");
  return false;
}

/**
 * Send incremental angle control with max speed (CAN command 0xA8)
 */
bool LKM_Motor::sendIncrementAngle2Command(float angleIncrement, uint16_t maxSpeed) {
  int32_t angleIncrementCentesimi = static_cast<int32_t>(angleIncrement * _reductionGear * 100.0);
  if (invertEncoder) {
    angleIncrementCentesimi = -angleIncrementCentesimi;
  }
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0xA8, 0, 0, 0, 0, 0, 0, 0};
  buf[2]                 = (uint8_t)(maxSpeed & 0xFF);
  buf[3]                 = (uint8_t)((maxSpeed >> 8) & 0xFF);
  buf[4]                 = (uint8_t)(angleIncrementCentesimi & 0xFF);
  buf[5]                 = (uint8_t)((angleIncrementCentesimi >> 8) & 0xFF);
  buf[6]                 = (uint8_t)((angleIncrementCentesimi >> 16) & 0xFF);
  buf[7]                 = (uint8_t)((angleIncrementCentesimi >> 24) & 0xFF);
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("INC_ANGLE2 sent: angleIncrement=" + String(angleIncrementCentesimi) +
              " maxSpeed=" + String(maxSpeed));
    return true;
  }
  LOG_C1_ERROR("ERROR sending INC_ANGLE2.");
  return false;
}

// ===================================================================
// PID CONFIGURATION
// ===================================================================

/**
 * Read PID parameters (CAN command 0x30)
 */
bool LKM_Motor::readPIDParameters() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x30, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("READ_PID command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending READ_PID.");
  return false;
}

/**
 * Write PID parameters to RAM (CAN command 0x31)
 */
bool LKM_Motor::writePIDParametersRAM(byte anglePidKp, byte anglePidKi, byte speedPidKp,
                                      byte speedPidKi, byte iqPidKp, byte iqPidKi) {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x31,       0,          anglePidKp, anglePidKi,
                            speedPidKp, speedPidKi, iqPidKp,    iqPidKi};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("WRITE_PID_RAM command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending WRITE_PID_RAM.");
  return false;
}

/**
 * Write PID parameters to ROM (CAN command 0x32)
 */
bool LKM_Motor::writePIDParametersROM(byte anglePidKp, byte anglePidKi, byte speedPidKp,
                                      byte speedPidKi, byte iqPidKp, byte iqPidKi) {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x32,       0,          anglePidKp, anglePidKi,
                            speedPidKp, speedPidKi, iqPidKp,    iqPidKi};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("WRITE_PID_ROM command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending WRITE_PID_ROM.");
  return false;
}

/**
 * Write acceleration to RAM (CAN command 0x34)
 */
bool LKM_Motor::writeAccelerationRAM(int32_t acceleration) {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x34, 0, 0, 0, 0, 0, 0, 0};
  buf[4]                 = acceleration & 0xFF;
  buf[5]                 = (acceleration >> 8) & 0xFF;
  buf[6]                 = (acceleration >> 16) & 0xFF;
  buf[7]                 = (acceleration >> 24) & 0xFF;
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("WRITE_ACCEL_RAM sent with value: " + String(acceleration));
    return true;
  }
  LOG_C1_ERROR("ERROR sending WRITE_ACCEL_RAM.");
  return false;
}

// ===================================================================
// ENCODER OPERATIONS
// ===================================================================

/**
 * Read raw encoder value (CAN command 0x90)
 */
bool LKM_Motor::readEncoder() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x90, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("READ_ENCODER command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending READ_ENCODER.");
  return false;
}

/**
 * Set encoder offset in ROM (CAN command 0x91)
 */
bool LKM_Motor::setEncoderOffsetROM(uint16_t encoderOffset) {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x91, 0, 0, 0, 0, 0, 0, 0};
  buf[6]                 = encoderOffset & 0xFF;
  buf[7]                 = (encoderOffset >> 8) & 0xFF;
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("SET_ENCODER_ROM sent with value: " + String(encoderOffset));
    return true;
  }
  LOG_C1_ERROR("ERROR sending SET_ENCODER_ROM.");
  return false;
}

/**
 * Set current position as zero in ROM (CAN command 0x19)
 */
bool LKM_Motor::setCurrentPositionAsZeroROM() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x19, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("SET_ZERO_POS_ROM command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending SET_ZERO_POS_ROM.");
  return false;
}

/**
 * Read multi-loop angle (CAN command 0x92)
 */
bool LKM_Motor::readMultiAngleLoop() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x92, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
  LOG_C1_INFO("READ_ML_ANGLE command sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending READ_ML_ANGLE.");
  return false;
}

/**
 * Read single-loop angle (CAN command 0x94)
 */
bool LKM_Motor::readSingleAngleLoop() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x94, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("READ_SL_ANGLE command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending READ_SL_ANGLE.");
  return false;
}

/**
 * Clear accumulated angle (CAN command 0x95)
 */
bool LKM_Motor::clearAngleLoop() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x95, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("CLEAR_ANGLE_LOOP command sent.");
    return true;
  }
  LOG_C1_ERROR("ERROR sending CLEAR_ANGLE_LOOP.");
  return false;
}

/**
 * Read motor state 2 (CAN command 0x9C)
 * Updates: motorTemperature2, motorTorqueCurrent, motorSpeed, motorEncoderPosition
 */
bool LKM_Motor::readMotorState2() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) != CAN_OK) {
  LOG_C1_ERROR("ERROR sending READ_STATE2.");
    return false;
  }
  LOG_C1_DEBUG("READ_STATE2 command sent.");

  unsigned long startTime = millis();
  while (millis() - startTime < 200) {
    if (_can->checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      unsigned char len;
      unsigned char rcvBuf[8];
      if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
        if (rcvBuf[0] == 0x9C) {
          // Parse state 2 data
          motorTemperature2    = (int8_t)rcvBuf[1];
          motorTorqueCurrent   = ((int16_t)rcvBuf[3] << 8) | rcvBuf[2];
          motorSpeed           = ((int16_t)rcvBuf[5] << 8) | rcvBuf[4];
          motorEncoderPosition = ((uint16_t)rcvBuf[7] << 8) | rcvBuf[6];

          LOG_C1_DEBUG("READ_STATE2: Temp=" + String(motorTemperature2) +
                    "°C TorqueCur=" + String(motorTorqueCurrent) +
                    " Speed=" + String(motorSpeed) +
                    "dps EncPos=" + String(motorEncoderPosition));
          return true;
        }
      }
    }
  }
  LOG_C1_WARN("Timeout: no response from READ_STATE2.");
  return false;
}

/**
 * Read motor state 3 (CAN command 0x9D)
 * Updates: motorTemperature3, phaseACurrent, phaseBCurrent, phaseCCurrent
 */
bool LKM_Motor::readMotorState3() {
  unsigned long targetID = 0x140 + _motorID;
  unsigned char buf[8]   = {0x9D, 0, 0, 0, 0, 0, 0, 0};
  if (_can->sendMsgBuf(targetID, 0, 8, buf) != CAN_OK) {
  LOG_C1_ERROR("ERROR sending READ_STATE3.");
    return false;
  }
  LOG_C1_DEBUG("READ_STATE3 command sent.");

  unsigned long startTime = millis();
  while (millis() - startTime < 200) {
    if (_can->checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      unsigned char len;
      unsigned char rcvBuf[8];
      if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
        if (rcvBuf[0] == 0x9D) {
          // Parse state 3 data
          motorTemperature3 = (int8_t)rcvBuf[1];
          phaseACurrent     = ((int16_t)rcvBuf[3] << 8) | rcvBuf[2];
          phaseBCurrent     = ((int16_t)rcvBuf[5] << 8) | rcvBuf[4];
          phaseCCurrent     = ((int16_t)rcvBuf[7] << 8) | rcvBuf[6];

          LOG_C1_DEBUG("READ_STATE3: Temp=" + String(motorTemperature3) +
                    "°C PhaseA=" + String(phaseACurrent) +
                    " PhaseB=" + String(phaseBCurrent) +
                    " PhaseC=" + String(phaseCCurrent));
          return true;
        }
      }
    }
  }
  LOG_C1_WARN("Timeout: no response from READ_STATE3.");
  return false;
}

// ===================================================================
// MULTI-MOTOR CONTROL
// ===================================================================

/**
 * Send torque commands to 4 motors simultaneously (CAN ID 0x280)
 * Static method for synchronized multi-motor control
 */
bool LKM_Motor::sendMultiMotorTorqueCommand(MCP_CAN *can, int16_t iq1, int16_t iq2, int16_t iq3,
                                            int16_t iq4) {
  unsigned char buf[8];
  buf[0] = iq1 & 0xFF;
  buf[1] = (iq1 >> 8) & 0xFF;
  buf[2] = iq2 & 0xFF;
  buf[3] = (iq2 >> 8) & 0xFF;
  buf[4] = iq3 & 0xFF;
  buf[5] = (iq3 >> 8) & 0xFF;
  buf[6] = iq4 & 0xFF;
  buf[7] = (iq4 >> 8) & 0xFF;
  // CAN ID for multi-motor commands is fixed at 0x280
  if (can->sendMsgBuf(0x280, 0, 8, buf) == CAN_OK) {
    LOG_C1_DEBUG("MULTI_TORQUE command sent.");
    return true;
  }
  LOG_C1_ERROR("Error sending MULTI_TORQUE.");
  return false;
}

// ===================================================================
// SYNCHRONOUS READ OPERATIONS
// ===================================================================

/**
 * Read raw encoder value synchronously (blocking)
 * Uses CAN command 0x90 to read 14-bit encoder position
 */
uint16_t LKM_Motor::getEncoderRawSync() {
  uint16_t rawEncoder    = 0;
  unsigned long targetID = 0x140 + _motorID;
  unsigned char cmd[8]   = {0x90, 0, 0, 0, 0, 0, 0, 0};

  if (_can->sendMsgBuf(targetID, 0, 8, cmd) != CAN_OK) {
    LOG_C1_ERROR("ERROR sending READ_ENCODER (raw).");
    return rawEncoder;
  }

  unsigned long startTime = millis();
  // Wait max 2 ms for response
  while (millis() - startTime < 2) {
    if (_can->checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      unsigned char len;
      unsigned char rcvBuf[8];
      if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
        if (rcvBuf[0] == 0x90) {
          // Extract raw encoder value from bytes 2-3
          rawEncoder = ((uint16_t)rcvBuf[3] << 8) | rcvBuf[2];

          return rawEncoder;
        }
      }
    }
  }
  LOG_C1_WARN("Timeout: no response from READ_ENCODER (raw).");
  return rawEncoder;
}

/**
 * Read single-loop angle synchronously (blocking)
 */
LKM_Motor::MultiAngleData LKM_Motor::getSingleAngleSync() {
  MultiAngleData data;
  data.angle    = 0.0;
  data.waitTime = 0;

  unsigned long targetID = 0x140 + _motorID;
  unsigned char cmd[8]   = {0x94, 0, 0, 0, 0, 0, 0, 0};

  if (_can->sendMsgBuf(targetID, 0, 8, cmd) != CAN_OK) {
    LOG_C1_ERROR("ERROR sending READ_SL_ANGLE.");
    return data;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 2) {
    if (_can->checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      unsigned char len;
      unsigned char rcvBuf[8];
      if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
        if (rcvBuf[0] == 0x94) {
          data.waitTime = millis() - startTime;
          // Build a 56-bit value from bytes 1-7
          uint64_t temp = ((uint64_t)rcvBuf[7] << 48) | ((uint64_t)rcvBuf[6] << 40) |
                          ((uint64_t)rcvBuf[5] << 32) | ((uint64_t)rcvBuf[4] << 24) |
                          ((uint64_t)rcvBuf[3] << 16) | ((uint64_t)rcvBuf[2] << 8) |
                          ((uint64_t)rcvBuf[1]);
          int64_t motorAngle = ((int64_t)temp << 8) >> 8;
          // motorAngle is in 0.01° units; convert to degrees and apply reduction
          data.angle = (motorAngle / 100.0) / _reductionGear;
          if (invertEncoder) {
            data.angle = -data.angle;
          }
          return data;
        }
      }
    }
  }
  LOG_C1_WARN("Timeout: no response from READ_SL_ANGLE.");
  return data;
}

/**
 * Read multi-loop angle synchronously (blocking)
 * 
 * Angle processing sequence:
 * 1. Read raw value from encoder (in hundredths of degree)
 * 2. Convert to degrees
 * 3. Apply reduction ratio
 * 4. If applyOffset is true:
 *    - Subtract offset
 *    - Apply inversion if active
 */
LKM_Motor::MultiAngleData LKM_Motor::getMultiAngleSync(bool applyOffset) {
  MultiAngleData data;
  data.angle    = NAN;  // Use NAN to indicate error (distinguishable from valid angles)
  data.waitTime = 0;

  unsigned long targetID = 0x140 + _motorID;
  unsigned long expectedResponseID = targetID;  // LKM motors respond with same ID
  unsigned char cmd[8]   = {0x92, 0, 0, 0, 0, 0, 0, 0};

  // Flush any stale messages in the RX buffer before sending new request.
  // This prevents reading old/stale data from previous failed requests.
  // 
  // NOTE: Stale messages are expected when running motor control at high frequency
  // (e.g., 100-500 Hz). The MCP2515 has only 2 RX buffers, and motor responses
  // can accumulate if not read in time. This flush mechanism ensures we always
  // get fresh data. Occasional garbage readings (e.g., -134140420096 degrees)
  // were observed without this, caused by reading stale/corrupted buffer data.
  int flushed = 0;
  while (_can->checkReceive() == CAN_MSGAVAIL && flushed < 5) {
    unsigned long dummyId;
    unsigned char dummyLen;
    unsigned char dummyBuf[8];
    _can->readMsgBuf(&dummyId, &dummyLen, dummyBuf);
    flushed++;
  }
  // Log only if we flushed more than expected (3+) and throttle to every 10 seconds
  if (flushed >= 3) {
    static uint32_t last_flush_log = 0;
    if (millis() - last_flush_log > 10000) {
      LOG_C1_WARN("[CAN] Flushed " + String(flushed) + " stale messages before motor " + String(_motorID) + " read");
      last_flush_log = millis();
    }
  }

  // Retry up to 2 times on failure
  const int MAX_RETRIES = 2;
  
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (_can->sendMsgBuf(targetID, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR("ERROR sending READ_ML_ANGLE (after " + String(MAX_RETRIES) + " retries).");
      }
      delayMicroseconds(100);  // Brief pause before retry
      continue;
    }

    unsigned long startTime = micros();
    // Wait for 2000 µs (2 ms)
    while (micros() - startTime < 2000) {
      if (_can->checkReceive() == CAN_MSGAVAIL) {
        unsigned long canId;
        unsigned char len;
        unsigned char rcvBuf[8];
        if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
          // IMPORTANT: Verify this is the response from OUR motor (correct CAN ID)
          // and the correct command response (0x92)
          if (canId == expectedResponseID && rcvBuf[0] == 0x92) {
            data.waitTime = micros() - startTime;
            // Build a 56-bit value from bytes 1-7
            uint64_t temp = ((uint64_t)rcvBuf[7] << 48) | ((uint64_t)rcvBuf[6] << 40) |
                            ((uint64_t)rcvBuf[5] << 32) | ((uint64_t)rcvBuf[4] << 24) |
                            ((uint64_t)rcvBuf[3] << 16) | ((uint64_t)rcvBuf[2] << 8) |
                            ((uint64_t)rcvBuf[1]);
            int64_t motorAngle = ((int64_t)temp << 8) >> 8;
            // motorAngle is in 0.01° units; convert to degrees and apply reduction
            data.angle = (motorAngle / 100.0) / _reductionGear;
            if (applyOffset) {
              data.angle = (data.angle - offsetEncoder) * (invertEncoder ? -1 : 1);
            } else {
              data.angle = data.angle * (invertEncoder ? -1 : 1);
            }
            return data;  // Success!
          }
          // Wrong CAN ID or command - this is a stale/wrong message, continue waiting
        }
      }
    }
    
    // Timeout on this attempt, will retry if attempts remain
    if (retry < MAX_RETRIES - 1) {
      delayMicroseconds(200);  // Brief pause before retry
    }
  }
  
  // All retries failed
  LOG_C1_WARN("Timeout: no response from READ_ML_ANGLE (motor " + String(_motorID) + ").");
  return data;  // Returns NAN angle to indicate error
}

/**
 * Pipelined dual-motor multi-loop angle read.
 *
 * Sends 0x92 to both motors back-to-back, then polls for both responses
 * in a single loop. Both motors must share the same CAN bus instance.
 */
PipelinedAngleData LKM_Motor::getMultiAnglePairPipelined(
    LKM_Motor *motorA, LKM_Motor *motorB) {
  PipelinedAngleData result;
  result.dataA.angle    = NAN;
  result.dataA.waitTime = 0;
  result.dataB.angle    = NAN;
  result.dataB.waitTime = 0;
  result.totalTime      = 0;

  MCP_CAN *can = motorA->_can;  // Shared bus
  unsigned long idA = 0x140 + motorA->_motorID;
  unsigned long idB = 0x140 + motorB->_motorID;
  unsigned char cmd[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};

  unsigned long t0 = micros();

  // --- Flush stale messages (single pass, shared bus) ---
  int flushed = 0;
  while (can->checkReceive() == CAN_MSGAVAIL && flushed < 5) {
    unsigned long dummyId;
    unsigned char dummyLen;
    unsigned char dummyBuf[8];
    can->readMsgBuf(&dummyId, &dummyLen, dummyBuf);
    flushed++;
  }
  if (flushed >= 3) {
    static uint32_t last_flush_log = 0;
    if (millis() - last_flush_log > 10000) {
      LOG_C1_WARN("[CAN PIPE] Flushed " + String(flushed) + " stale messages");
      last_flush_log = millis();
    }
  }

  const int MAX_RETRIES = 2;

  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    // --- Send command to motor A (non-blocking) ---
    if (can->sendMsgBufNoWait(idA, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR("[CAN PIPE] Send failed motor A (id " + String(motorA->_motorID) + ") after retries");
      }
      delayMicroseconds(100);
      continue;
    }

    // --- Send command to motor B (non-blocking, back-to-back) ---
    if (can->sendMsgBufNoWait(idB, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR("[CAN PIPE] Send failed motor B (id " + String(motorB->_motorID) + ") after retries");
      }
      delayMicroseconds(100);
      continue;
    }

    // --- Single poll loop for both responses ---
    bool gotA = false, gotB = false;

    while (micros() - t0 < 2500) {  // 2.5ms timeout from function start
      if (can->checkReceive() == CAN_MSGAVAIL) {
        unsigned long canId;
        unsigned char len;
        unsigned char rcvBuf[8];
        if (can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK && rcvBuf[0] == 0x92) {

          auto parseAngle = [&](LKM_Motor *motor, MultiAngleData &data) {
            data.waitTime = micros() - t0;
            uint64_t temp = ((uint64_t)rcvBuf[7] << 48) | ((uint64_t)rcvBuf[6] << 40) |
                            ((uint64_t)rcvBuf[5] << 32) | ((uint64_t)rcvBuf[4] << 24) |
                            ((uint64_t)rcvBuf[3] << 16) | ((uint64_t)rcvBuf[2] << 8) |
                            ((uint64_t)rcvBuf[1]);
            int64_t motorAngle = ((int64_t)temp << 8) >> 8;
            data.angle = (motorAngle / 100.0) / motor->_reductionGear;
            data.angle = (data.angle - motor->offsetEncoder) * (motor->invertEncoder ? -1 : 1);
          };

          if (canId == idA && !gotA) {
            parseAngle(motorA, result.dataA);
            gotA = true;
          } else if (canId == idB && !gotB) {
            parseAngle(motorB, result.dataB);
            gotB = true;
          }

          if (gotA && gotB) {
            result.totalTime = micros() - t0;
            return result;  // Both received
          }
        }
      }
    }

    // Timeout — retry if attempts remain
    if (retry < MAX_RETRIES - 1) {
      delayMicroseconds(200);
    }
  }

  // All retries failed
  result.totalTime = micros() - t0;
  static uint32_t last_pipe_timeout_log = 0;
  if (millis() - last_pipe_timeout_log > 5000) {
    LOG_C1_WARN("[CAN PIPE] Timeout: A=" + String(!isnan(result.dataA.angle) ? "OK" : "FAIL") +
                " B=" + String(!isnan(result.dataB.angle) ? "OK" : "FAIL"));
    last_pipe_timeout_log = millis();
  }
  return result;
}
