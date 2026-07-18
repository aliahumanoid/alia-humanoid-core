/**
 * @file LKM_Motor.cpp
 * @brief Implementation of LKM motor driver with CAN bus communication
 * 
 * See LKM_Motor.h for detailed documentation.
 */

#include "LKM_Motor.h"
#include <debug.h>
#include <hot_path.h>

namespace {

static bool shouldLogMotorReadFailure(uint32_t *last_log_ms, unsigned int motor_id,
                                      uint32_t throttle_ms = 1000) {
  const unsigned int idx = motor_id & 0xFF;
  const uint32_t now = millis();
  if (now - last_log_ms[idx] < throttle_ms) {
    return false;
  }
  last_log_ms[idx] = now;
  return true;
}

}  // namespace

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
  // Revolution tracking defaults
  _encoderCountsPerRev = 65536;  // 18-bit encoder (MG4005-i10 V2 default)
  _revolutions         = 0;
  _prevEncoder         = 0;
  _revTrackInit        = false;
}

/**
 * Initialize motor (CAN bus must be already initialized)
 */
void LKM_Motor::init() {
  LOG_C1_DEBUG_F("LKM_Motor initialized with ID: %u", _motorID);
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

void LKM_Motor::setEncoderCountsPerRev(uint32_t counts) {
  _encoderCountsPerRev = counts;
}

// ===================================================================
// REVOLUTION TRACKING
// ===================================================================

/**
 * Initialize revolution tracking from absolute 0x92 reading + 0xA1 encoder.
 *
 * Revolution tracking works in actuator/output-space:
 * - 0x92 provides a multi-turn motor-side angle in 0.01°
 * - 0xA1 encoder provides the single-turn actuator/output position
 *
 * We first convert the 0x92 angle into actuator degrees using the reduction gear,
 * then compute the full actuator revolutions consistent with the current encoder
 * position. Safe even if the motor moved slightly between the two reads, as long
 * as the actuator itself did not move more than ±180° between them.
 */
void LKM_Motor::initRevTracking(int64_t absMotorAngle_centideg, uint16_t currentEncoder) {
  double actuatorAngle_deg = (absMotorAngle_centideg / 100.0) / (double)_reductionGear;
  double encoderAngle_deg = (double)currentEncoder / (double)_encoderCountsPerRev * 360.0;

  // Compute actuator revolution count consistent with both readings
  _revolutions = (int32_t)round((actuatorAngle_deg - encoderAngle_deg) / 360.0);
  _prevEncoder = currentEncoder;
  _revTrackInit = true;

  // Verify consistency (rate-limited: log at most once per 2s per motor)
  double trackedAngle = (double)_revolutions * 360.0 + encoderAngle_deg;
  double error = fabs(trackedAngle - actuatorAngle_deg);
  if (error > 5.0) {
    static uint32_t lastLogMs[4] = {};  // up to 4 motors
    uint8_t idx = _motorID < 4 ? _motorID : 0;
    uint32_t now = millis();
    if (now - lastLogMs[idx] > 2000) {
      char f1[48], f2[48], f3[48];
      LOG_C1_WARN_F("[RevTrack] M%u init discrepancy: %s° (0x92=%s trk=%s)",
                    _motorID, c1f(f1, error, 2), c1f(f2, actuatorAngle_deg, 2),
                    c1f(f3, trackedAngle, 2));
      lastLogMs[idx] = now;
    }
  }
}

/**
 * Update tracked angle from new 0xA1 encoder reading.
 *
 * Detects wrap-arounds by checking if the encoder delta exceeds half
 * the encoder range. The tracked revolutions are actuator/output revolutions,
 * matching the single-turn actuator position encoded in the 0xA1 response.
 */
void HOT_FUNC(LKM_Motor::updateRevTracking)(uint16_t currentEncoder) {
  if (!_revTrackInit) return;

  int32_t halfRange = (int32_t)(_encoderCountsPerRev / 2);
  int32_t delta = (int32_t)currentEncoder - (int32_t)_prevEncoder;

  // Wrap detection
  if (delta > halfRange) {
    _revolutions--;  // Wrapped backward (e.g., 100 → 65400 for 18-bit)
  } else if (delta < -halfRange) {
    _revolutions++;  // Wrapped forward (e.g., 65400 → 100 for 18-bit)
  }

  _prevEncoder = currentEncoder;
}

/**
 * Get tracked angle in output degrees (same coordinate system as getMultiAngleSync).
 *
 * Combines actuator/output revolution count with the current single-turn
 * actuator position from 0xA1, then applies offset and inversion.
 */
float HOT_FUNC(LKM_Motor::getTrackedAngle)() const {
  float outputAngle = (float)_revolutions * 360.0f +
                      (float)_prevEncoder / (float)_encoderCountsPerRev * 360.0f;
  outputAngle = (outputAngle - offsetEncoder) * (invertEncoder ? -1.0f : 1.0f);
  return outputAngle;
}

void HOT_FUNC(LKM_Motor::setTrackedAngleFromAbsolute)(int64_t absMotorAngle_centideg) {
  // Measurement-path advance for S2 substitute-fire: correct the revolution count ONLY, from the just-
  // validated 0x92 absolute, keeping the live single-turn encoder (_prevEncoder) as the sub-revolution
  // source. Same _revolutions formula as initRevTracking() (whose rev-anchor use stays untouched).
  if (!_revTrackInit) return;  // nothing to advance until bootstrap has anchored the tracking
  double actuatorAngle_deg = (absMotorAngle_centideg / 100.0) / (double)_reductionGear;
  double encoderAngle_deg  = (double)_prevEncoder / (double)_encoderCountsPerRev * 360.0;
  _revolutions = (int32_t)round((actuatorAngle_deg - encoderAngle_deg) / 360.0);
}

bool LKM_Motor::isRevTrackInit() const {
  return _revTrackInit;
}

void LKM_Motor::resetRevTracking() {
  _revTrackInit = false;
  _revolutions = 0;
  _prevEncoder = 0;
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
    // (chatter log removed, v2 String pass 2026-07-06)
    return true;
  }
  LOG_C1_ERROR_F("Error sending Motor STOP.");
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
    // (chatter log removed, v2 String pass 2026-07-06)
    return true;
  }
  LOG_C1_ERROR_F("Error sending SET_SPEED.");
  return false;
}

/**
 * Set motor torque (CAN command 0xA1)
 * 
 * Torque value range: -2048 to 2048 (approximately -33A to 33A)
 * Torque is automatically limited to _maxTorque and inverted if encoder is inverted.
 */
bool HOT_FUNC(LKM_Motor::setTorque)(int torque) {
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
    return true;
  }
  LOG_C1_ERROR_F("Error sending SET_TORQUE.");
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
  char f1[48];
  LOG_C1_INFO_F("Encoder offset set to: %s", c1f(f1, offsetEncoder, 2));
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
    // (chatter log removed, v2 String pass 2026-07-06)
    return true;
  }
  LOG_C1_ERROR_F("Error sending ML_ANGLE2.");
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
  static uint32_t last_send_error_log_ms[256] = {0};
  static uint32_t last_timeout_log_ms[256] = {0};

  MultiAngleData data;
  data.angle    = NAN;
  data.waitTime = 0;
  data.rawMotorAngle_centideg = 0;

  unsigned long targetID = 0x140 + _motorID;
  unsigned long expectedResponseID = targetID;
  unsigned char cmd[8]   = {0x94, 0, 0, 0, 0, 0, 0, 0};

  int flushed = 0;
  while (_can->checkReceive() == CAN_MSGAVAIL && flushed < 5) {
    unsigned long dummyId;
    unsigned char dummyLen;
    unsigned char dummyBuf[8];
    _can->readMsgBuf(&dummyId, &dummyLen, dummyBuf);
    flushed++;
  }

  const int MAX_RETRIES = 2;
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (_can->sendMsgBuf(targetID, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1 &&
          shouldLogMotorReadFailure(last_send_error_log_ms, _motorID)) {
        LOG_C1_ERROR_F("ERROR sending READ_SL_ANGLE (after %d retries).", MAX_RETRIES);
      }
      delayMicroseconds(100);
      continue;
    }

    unsigned long startTime = micros();
    while (micros() - startTime < 2000) {
      if (_can->checkReceive() == CAN_MSGAVAIL) {
        unsigned long canId;
        unsigned char len;
        unsigned char rcvBuf[8];
        if (_can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK) {
          if (canId == expectedResponseID && rcvBuf[0] == 0x94) {
            data.waitTime = micros() - startTime;
            // 0x94 returns a single-turn absolute angle as a 32-bit unsigned
            // centidegree value in bytes 4..7 of the CAN payload.
            uint32_t motorAngle = ((uint32_t)rcvBuf[7] << 24) |
                                  ((uint32_t)rcvBuf[6] << 16) |
                                  ((uint32_t)rcvBuf[5] << 8)  |
                                  ((uint32_t)rcvBuf[4]);
            data.rawMotorAngle_centideg = static_cast<int64_t>(motorAngle);
            data.angle = (static_cast<float>(motorAngle) / 100.0f) / _reductionGear;
            if (invertEncoder) {
              data.angle = -data.angle;
            }
            return data;
          }
        }
      }
    }
  }

  if (shouldLogMotorReadFailure(last_timeout_log_ms, _motorID)) {
    LOG_C1_WARN_F("Timeout: no response from READ_SL_ANGLE (motor %u).", _motorID);
  }
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
  static uint32_t last_send_error_log_ms[256] = {0};
  static uint32_t last_timeout_log_ms[256] = {0};

  MultiAngleData data;
  data.angle    = NAN;  // Use NAN to indicate error (distinguishable from valid angles)
  data.waitTime = 0;
  data.rawMotorAngle_centideg = 0;

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
      LOG_C1_WARN_F("[CAN] Flushed %d stale messages before motor %u read", flushed, _motorID);
      last_flush_log = millis();
    }
  }

  // Retry up to 2 times on failure
  const int MAX_RETRIES = 2;
  
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (_can->sendMsgBuf(targetID, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1 &&
          shouldLogMotorReadFailure(last_send_error_log_ms, _motorID)) {
        LOG_C1_ERROR_F("ERROR sending READ_ML_ANGLE (after %d retries).", MAX_RETRIES);
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
            data.rawMotorAngle_centideg = motorAngle;
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
  if (shouldLogMotorReadFailure(last_timeout_log_ms, _motorID)) {
    LOG_C1_WARN_F("Timeout: no response from READ_ML_ANGLE (motor %u).", _motorID);
  }
  return data;  // Returns NAN angle to indicate error
}

/**
 * Pipelined dual-motor multi-loop angle read.
 *
 * Sends 0x92 to both motors back-to-back, then polls for both responses
 * in a single loop. Both motors must share the same CAN bus instance.
 */
// Parse an 0x92 multi-turn angle reply (DATA[1..7] = signed 56-bit motor-side centideg, little-endian)
// into output-space degrees + the raw centideg (for rev-tracking init). Factored out of the inline lambda
// in getMultiAnglePairPipelined() so a ROUTED 0x92 reply (S2 substitute-fire, collectPair) parses through
// the SAME conversion. Byte layout and math are verbatim from the historical lambda. waitTime = 0 (a
// carried/routed reply has no round-trip window; the pipelined caller overwrites it with micros()-t0).
void HOT_FUNC(LKM_Motor::parseMultiAngleReply)(LKM_Motor *motor, const unsigned char *buf, MultiAngleData &data) {
  data.waitTime = 0;
  uint64_t temp = ((uint64_t)buf[7] << 48) | ((uint64_t)buf[6] << 40) |
                  ((uint64_t)buf[5] << 32) | ((uint64_t)buf[4] << 24) |
                  ((uint64_t)buf[3] << 16) | ((uint64_t)buf[2] << 8) |
                  ((uint64_t)buf[1]);
  int64_t motorAngle = ((int64_t)temp << 8) >> 8;
  data.rawMotorAngle_centideg = motorAngle;
  data.angle = (motorAngle / 100.0) / motor->_reductionGear;
  data.angle = (data.angle - motor->offsetEncoder) * (motor->invertEncoder ? -1 : 1);
}

PipelinedAngleData LKM_Motor::getMultiAnglePairPipelined(
    LKM_Motor *motorA, LKM_Motor *motorB) {
  PipelinedAngleData result;
  result.dataA.angle    = NAN;
  result.dataA.waitTime = 0;
  result.dataA.rawMotorAngle_centideg = 0;
  result.dataB.angle    = NAN;
  result.dataB.waitTime = 0;
  result.dataB.rawMotorAngle_centideg = 0;
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
      LOG_C1_WARN_F("[CAN PIPE] Flushed %d stale messages", flushed);
      last_flush_log = millis();
    }
  }

  const int MAX_RETRIES = 2;

  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    // --- Send command to motor A (non-blocking) ---
    if (can->sendMsgBufNoWait(idA, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR_F("[CAN PIPE] Send failed motor A (id %u) after retries", motorA->_motorID);
      }
      delayMicroseconds(100);
      continue;
    }

    // --- Send command to motor B (non-blocking, back-to-back) ---
    if (can->sendMsgBufNoWait(idB, 0, 8, cmd) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR_F("[CAN PIPE] Send failed motor B (id %u) after retries", motorB->_motorID);
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
            parseMultiAngleReply(motor, rcvBuf, data);  // factored: same byte layout + conversion
            data.waitTime = micros() - t0;              // pipelined round-trip window (unchanged)
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
    LOG_C1_WARN_F("[CAN PIPE] Timeout: A=%s B=%s",
                  !isnan(result.dataA.angle) ? "OK" : "FAIL",
                  !isnan(result.dataB.angle) ? "OK" : "FAIL");
    last_pipe_timeout_log = millis();
  }
  return result;
}

// ===================================================================
// PIPELINED TORQUE SEND + RESPONSE READ
// ===================================================================

/**
 * Send torque commands to two motors and read their state responses
 * in a single pipelined CAN transaction.
 *
 * The 0xA1 command sends the torque value; the motor responds with its
 * current state (temperature, iq current, speed, encoder position).
 * Unlike fire-and-forget setTorque(), this captures the response data
 * for revolution tracking and motor state monitoring.
 *
 * Response format (LK CAN Protocol V2.35):
 *   DATA[0] = 0xA1 (command echo)
 *   DATA[1] = temperature (int8_t, 1°C/LSB)
 *   DATA[2-3] = iq torque current (int16_t, little-endian)
 *   DATA[4-5] = motor speed (int16_t, 1dps/LSB, little-endian)
 *   DATA[6-7] = encoder position (uint16_t, little-endian)
 *              14-bit: 0-16383, 15-bit: 0-32767, 18-bit: 0-65535
 *
 * If revolution tracking is initialized, the tracked angle is computed
 * from the encoder reading and stored in the response data.
 */
PipelinedTorqueResponseData LKM_Motor::setTorquePairPipelined(
    LKM_Motor *motorA, int torqueA,
    LKM_Motor *motorB, int torqueB) {

  PipelinedTorqueResponseData result;
  result.dataA.valid = false;
  result.dataA.angle = NAN;
  result.dataB.valid = false;
  result.dataB.angle = NAN;
  result.totalTime = 0;

  MCP_CAN *can = motorA->_can;  // Shared bus
  unsigned long idA = 0x140 + motorA->_motorID;
  unsigned long idB = 0x140 + motorB->_motorID;

  // Apply torque limits and inversion for motor A
  int ta = torqueA;
  if (ta > motorA->_maxTorque) ta = motorA->_maxTorque;
  else if (ta < -motorA->_maxTorque) ta = -motorA->_maxTorque;
  if (motorA->invertEncoder) ta = -ta;

  // Apply torque limits and inversion for motor B
  int tb = torqueB;
  if (tb > motorB->_maxTorque) tb = motorB->_maxTorque;
  else if (tb < -motorB->_maxTorque) tb = -motorB->_maxTorque;
  if (motorB->invertEncoder) tb = -tb;

  // Prepare CAN frames
  unsigned char bufA[8] = {0xA1, 0, 0, 0, 0, 0, 0, 0};
  bufA[4] = ta & 0xFF;
  bufA[5] = (ta >> 8) & 0xFF;

  unsigned char bufB[8] = {0xA1, 0, 0, 0, 0, 0, 0, 0};
  bufB[4] = tb & 0xFF;
  bufB[5] = (tb >> 8) & 0xFF;

  unsigned long t0 = micros();

  // --- Helper: drain any pending RX messages ---
  auto flushRx = [&](int maxDrain) {
    int n = 0;
    while (can->checkReceive() == CAN_MSGAVAIL && n < maxDrain) {
      unsigned long dId; unsigned char dLen; unsigned char dBuf[8];
      can->readMsgBuf(&dId, &dLen, dBuf);
      n++;
    }
    return n;
  };

  // --- Flush stale messages before first attempt ---
  int flushed = flushRx(5);
  if (flushed >= 3) {
    static uint32_t last_flush_log = 0;
    if (millis() - last_flush_log > 10000) {
      LOG_C1_WARN_F("[CAN TRQ PIPE] Flushed %d stale messages", flushed);
      last_flush_log = millis();
    }
  }

  const int MAX_RETRIES = 2;

  for (int retry = 0; retry < MAX_RETRIES; retry++) {

    // --- [P2 fix] Fresh timeout per retry attempt ---
    unsigned long t_retry = micros();

    // --- Send torque to motor A (non-blocking) ---
    if (can->sendMsgBufNoWait(idA, 0, 8, bufA) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR_F("[CAN TRQ PIPE] Send failed motor A (id %u)", motorA->_motorID);
      }
      delayMicroseconds(100);
      continue;
    }

    // --- Send torque to motor B (non-blocking, back-to-back) ---
    if (can->sendMsgBufNoWait(idB, 0, 8, bufB) != CAN_OK) {
      if (retry == MAX_RETRIES - 1) {
        LOG_C1_ERROR_F("[CAN TRQ PIPE] Send failed motor B (id %u)", motorB->_motorID);
      }
      // [P1 fix] Motor A was sent — drain its pending response before retrying
      // to prevent stale A response from pairing with fresh B response on next attempt
      delayMicroseconds(300);  // Allow A's response to arrive
      flushRx(3);
      continue;
    }

    // --- Single poll loop for both responses ---
    bool gotA = false, gotB = false;

    while (micros() - t_retry < 2500) {  // [P2 fix] per-retry 2.5ms window
      if (can->checkReceive() == CAN_MSGAVAIL) {
        unsigned long canId;
        unsigned char len;
        unsigned char rcvBuf[8];
        if (can->readMsgBuf(&canId, &len, rcvBuf) == CAN_OK && rcvBuf[0] == 0xA1) {

          auto parseResponse = [&](LKM_Motor *motor, TorqueResponseData &data) {
            data.temperature   = (int8_t)rcvBuf[1];
            data.torqueCurrent = ((int16_t)rcvBuf[3] << 8) | rcvBuf[2];
            data.motorSpeed    = ((int16_t)rcvBuf[5] << 8) | rcvBuf[4];
            data.encoder       = ((uint16_t)rcvBuf[7] << 8) | rcvBuf[6];
            data.valid = true;

            // Update revolution tracking if initialized
            if (motor->_revTrackInit) {
              motor->updateRevTracking(data.encoder);
              data.angle = motor->getTrackedAngle();
            } else {
              data.angle = NAN;
            }

            // Update motor state variables (same fields as readMotorState2)
            motor->motorTemperature2    = data.temperature;
            motor->motorTorqueCurrent   = data.torqueCurrent;
            motor->motorSpeed           = data.motorSpeed;
            motor->motorEncoderPosition = data.encoder;
          };

          if (canId == idA && !gotA) {
            parseResponse(motorA, result.dataA);
            gotA = true;
          } else if (canId == idB && !gotB) {
            parseResponse(motorB, result.dataB);
            gotB = true;
          }

          if (gotA && gotB) {
            result.totalTime = micros() - t0;
            return result;  // Both received
          }
        }
      }
    }

    // Timeout on this attempt — drain any partial responses before retrying
    // [P1 fix] Prevents stale partial response from corrupting next attempt
    if (retry < MAX_RETRIES - 1) {
      flushRx(3);
    }
  }

  // All retries failed
  result.totalTime = micros() - t0;
  static uint32_t last_trq_timeout_log = 0;
  if (millis() - last_trq_timeout_log > 5000) {
    LOG_C1_WARN_F("[CAN TRQ PIPE] Timeout: A=%s B=%s",
                  result.dataA.valid ? "OK" : "FAIL",
                  result.dataB.valid ? "OK" : "FAIL");
    last_trq_timeout_log = millis();
  }
  return result;
}

// ===================================================================
// NON-BLOCKING TORQUE (fire now, collect next cycle)
// ===================================================================

void HOT_FUNC(LKM_Motor::ingestTorqueReply)(const unsigned char *buf) {
  // Same parse as setTorquePairPipelined's parseResponse (minus the result-struct write).
  motorTemperature2    = (int8_t)buf[1];
  motorTorqueCurrent   = ((int16_t)buf[3] << 8) | buf[2];
  motorSpeed           = ((int16_t)buf[5] << 8) | buf[4];
  motorEncoderPosition = ((uint16_t)buf[7] << 8) | buf[6];
  if (_revTrackInit) {
    updateRevTracking(motorEncoderPosition);  // exactly once per reply, in arrival order
  }
}

// --- Bus-health diagnostic counters (cumulative since resetBusDiag()) ---
static uint32_t s_tx_fail = 0;
static uint32_t s_rx_miss = 0;
static uint32_t s_rx_overflow = 0;
static bool     s_ovr_prev = false;
// S2 CARRY correctness signal (G1): residual WAIT of a carried collect AFTER the pre-sweep. When the
// carry hides the flight, the replies are already buffered -> the pre-sweep gets both -> the WAIT loop
// is skipped -> residual ~0. A non-zero residual means the carry is NOT fully hiding the round-trip
// (compute too short / flight too long) — the direct carry-correctness number for the G1 bench gate.
static uint32_t s_carry_collect_count = 0;    // carried (presweep) collects total
static uint32_t s_carry_wait_over_count = 0;  // carried collects whose residual wait exceeded the threshold
static uint32_t s_carry_wait_max_us = 0;      // worst carried residual wait (us)
static const uint32_t S_CARRY_WAIT_OVER_US = 100;  // "not hidden" threshold (a hidden carry is ~a buffered read)
// 500Hz Stage-1 instrumentation: fire->reply latency histogram (us). Buckets:
// <200, <300, <400, <500, <700, <1000, <1500, >=1500. Resolves the design-study
// estimate spread (1450 vs 1800 us cycle) and sizes the Stage-2 drain windows.
static uint32_t s_lat_hist[8] = {0};
static const uint16_t HOT_DATA_ATTR("lat_edges") s_lat_edges[7] = {200, 300, 400, 500, 700, 1000, 1500};
static inline void HOT_FUNC(latRecord)(uint32_t dt_us) {
  int b = 7;
  for (int i = 0; i < 7; i++) { if (dt_us < s_lat_edges[i]) { b = i; break; } }
  s_lat_hist[b]++;
}
// O4a: MCP2515 /INT-gated draining. CANINTE RX0IE|RX1IE are set at init (mcp_can.cpp:588),
// so /INT is LOW while an RX buffer holds a frame. A GPIO read (~ns) replaces most ~4us SPI
// checkReceive polls. -1 = disabled (pure SPI poll, the legacy behavior). The SPI fallback
// every 16 idle spins runs regardless, so an unwired/non-asserting INT line can NEVER hang
// the drain - it only costs the old poll rate.
static int s_collect_int_pin = -1;
uint32_t LKM_Motor::txFailCount()     { return s_tx_fail; }
uint32_t LKM_Motor::rxMissCount()     { return s_rx_miss; }
const uint32_t *LKM_Motor::latencyHistogram() { return s_lat_hist; }
void LKM_Motor::resetLatencyHistogram() { for (int i = 0; i < 8; i++) s_lat_hist[i] = 0; }
uint32_t LKM_Motor::rxOverflowCount() { return s_rx_overflow; }
void     LKM_Motor::resetBusDiag()    { s_tx_fail = 0; s_rx_miss = 0; s_rx_overflow = 0; s_ovr_prev = false; }
uint32_t LKM_Motor::carryCollectCount()  { return s_carry_collect_count; }
uint32_t LKM_Motor::carryWaitOverCount() { return s_carry_wait_over_count; }
uint32_t LKM_Motor::carryWaitMaxUs()     { return s_carry_wait_max_us; }
void     LKM_Motor::resetCarryWaitStats(){ s_carry_collect_count = 0; s_carry_wait_over_count = 0; s_carry_wait_max_us = 0; }

bool HOT_FUNC(LKM_Motor::fireCommand)(const unsigned char *buf) {
  // Send a pre-built command frame non-blocking; record the outstanding transaction. CHECK the TX
  // return: with only 3 MCP2515 TX buffers, an un-checked back-to-back send can be silently dropped.
  _fire_canId = 0x140 + _motorID;
  _fire_cmd = buf[0];       // command byte (0xA1/0xA4/0x92) — collectPair routes the reply by this
  _last_reply_valid = false;
  if (_can->sendMsgBufNoWait(_fire_canId, 0, 8, (unsigned char *)buf) != CAN_OK) {
    _fire_pending = false;   // TX queue refused it -> no reply will come
    s_tx_fail++;
    return false;
  }
  _fire_t_us = micros();
  _fire_pending = true;
  return true;
}

bool HOT_FUNC(LKM_Motor::fireTorque)(int torque) {
  int t = torque;
  if (t > _maxTorque) t = _maxTorque;
  else if (t < -_maxTorque) t = -_maxTorque;
  if (invertEncoder) t = -t;
  unsigned char buf[8] = {0xA1, 0, 0, 0, 0, 0, 0, 0};
  buf[4] = t & 0xFF;
  buf[5] = (t >> 8) & 0xFF;
  return fireCommand(buf);
}

bool LKM_Motor::fireAngle2(float angle, uint16_t maxSpeed) {
  // Same conversion as sendMultiLoopAngle2Command (invert + offsetEncoder + reductionGear + centideg).
  int8_t invert = invertEncoder ? -1 : 1;
  int32_t c = (int32_t)((angle + (invert * offsetEncoder)) * invert * _reductionGear * 100.0);
  unsigned char buf[8] = {0xA4, 0, 0, 0, 0, 0, 0, 0};
  buf[2] = (uint8_t)(maxSpeed & 0xFF);
  buf[3] = (uint8_t)((maxSpeed >> 8) & 0xFF);
  buf[4] = (uint8_t)(c & 0xFF);
  buf[5] = (uint8_t)((c >> 8) & 0xFF);
  buf[6] = (uint8_t)((c >> 16) & 0xFF);
  buf[7] = (uint8_t)((c >> 24) & 0xFF);
  return fireCommand(buf);
}

bool HOT_FUNC(LKM_Motor::fire92)() {
  // 0x92 = multi-turn angle READ (no payload). Same non-blocking fire discipline as fireTorque/fireAngle2:
  // fireCommand records _fire_pending/_fire_t_us/_fire_canId and _fire_cmd=0x92, so collectPair/carry track
  // and route this transaction identically to a torque pair. The reply carries the same CAN id (0x140+ID)
  // as an 0xA1/0xA4 reply but with DATA[0]=0x92, so collectPair distinguishes it by _fire_cmd.
  unsigned char buf[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
  return fireCommand(buf);
}

int HOT_FUNC(LKM_Motor::flushStaleRx)(int maxDrain) {
  int n = 0;
  while (_can->checkReceive() == CAN_MSGAVAIL && n < maxDrain) {
    unsigned long dId; unsigned char dLen; unsigned char dBuf[8];
    _can->readMsgBuf(&dId, &dLen, dBuf);
    n++;
  }
  return n;
}

void LKM_Motor::setCollectIntPin(int pin) { s_collect_int_pin = pin; }

void HOT_FUNC(LKM_Motor::collectPair)(LKM_Motor *mA, LKM_Motor *mB, uint32_t timeout_us, bool presweep) {
  MCP_CAN *can = mA->_can;
  uint32_t t0 = micros();
  uint32_t idle_spin = 0;
  // Reply router (S2 sub92): a reply frame carries the same CAN id (0x140+ID) whether it answers an 0xA1
  // torque, an 0xA4 position, or an 0x92 angle-read; the ECHOED command byte (rxBuf[0]) distinguishes them.
  // Route a frame to a pending motor ONLY when its id AND its fired command byte (_fire_cmd) match. An 0x92
  // substitute-fire reply is parsed into _last92 (parseMultiAngleReply); an 0xA1/0xA4 reply goes through
  // ingestTorqueReply (rev-tracking advance) exactly as before. Bit-identical when no 0x92 was fired
  // (_fire_cmd is 0xA1/0xA4 so an 0x92 frame — which never appears then — could not match). Lambda has full
  // private access via the enclosing static member. Returns true if the frame was consumed by this motor.
  auto route = [](LKM_Motor *m, unsigned long rxId, const unsigned char *rxBuf) HOT_LAMBDA_ATTR("route") -> bool {
    if (!m->_fire_pending || rxId != m->_fire_canId || rxBuf[0] != m->_fire_cmd) return false;
    if (rxBuf[0] == 0x92) {
      parseMultiAngleReply(m, rxBuf, m->_last92);
    } else {
      m->ingestTorqueReply(rxBuf);
    }
    m->_fire_pending = false;
    m->_last_reply_valid = true;
    return true;
  };
  // S2 CARRY (FATAL-2): one UNCONDITIONAL non-blocking pass over the frames ALREADY in the RX buffers
  // BEFORE the deadline loop. A carried pair's replies landed while the loop preamble ran (age > flight),
  // so the deadline-shrunk WAIT below would otherwise time out to a spurious MISS without ever reading
  // them. Route them here first (same id filter as the main loop); the WAIT then only covers not-yet-
  // arrived frames. Bounded (<=4 frames = 2 RX buffers + slack) so a flooded bus can't hang the pass.
  if (presweep) {
    int swept = 0;
    while ((mA->_fire_pending || mB->_fire_pending) &&
           can->checkReceive() == CAN_MSGAVAIL && swept < 4) {
      swept++;
      unsigned long rxId; unsigned char len; unsigned char rxBuf[8];
      if (can->readMsgBuf(&rxId, &len, rxBuf) != CAN_OK) continue;
      if (rxBuf[0] != 0xA1 && rxBuf[0] != 0xA4 && rxBuf[0] != 0x92) continue;  // motor state / angle reply
      // NOTE: no latRecord() here. A pre-swept reply is (almost always) a CARRIED pair whose
      // _fire_t_us is from the PREVIOUS cycle, so micros()-_fire_t_us is the cross-cycle age
      // (~period+flight), not the fire->reply round-trip — it would inflate the LATHIST tail
      // that COLLECT_FLOOR_US is sized from. Carried-pair timing belongs in [S2DIAG], not LATHIST.
      // route() also matches _fire_cmd, so a carried 0x92 pair parses into _last92 (sub92) here.
      if (!route(mA, rxId, rxBuf)) route(mB, rxId, rxBuf);
    }
  }
  const uint32_t t_after_presweep = presweep ? micros() : 0;  // residual-wait start (carried collects only;
                                                              // no added read on the serial/non-carry path)
  // Only this ONE pair is outstanding (<=2), so the 2-deep MCP2515 RX buffer never overflows: drain
  // each reply as it arrives, routed by CAN id, until BOTH are in or the short timeout elapses.
  while ((mA->_fire_pending || mB->_fire_pending) && (micros() - t0) < timeout_us) {
    if (s_collect_int_pin >= 0 && HOT_DIGITAL_READ(s_collect_int_pin) == HIGH &&
        ((++idle_spin) & 0x0F) != 0) {
      continue;  // /INT high = no RX pending; SPI fallback poll every 16th spin regardless
    }
    if (can->checkReceive() != CAN_MSGAVAIL) continue;
    unsigned long rxId; unsigned char len; unsigned char rxBuf[8];
    if (can->readMsgBuf(&rxId, &len, rxBuf) != CAN_OK) continue;
    if (rxBuf[0] != 0xA1 && rxBuf[0] != 0xA4 && rxBuf[0] != 0x92) continue;  // motor state / angle reply
    if (route(mA, rxId, rxBuf)) {
      latRecord(micros() - mA->_fire_t_us);
    } else if (route(mB, rxId, rxBuf)) {
      latRecord(micros() - mB->_fire_t_us);
    }
  }
  // S2 carry residual-wait: how long the WAIT loop ran AFTER the pre-sweep, for carried collects only.
  // ~0 when the carry hid the flight (pre-sweep got both); >threshold = the carry did not fully hide.
  if (presweep) {
    const uint32_t residual = micros() - t_after_presweep;
    s_carry_collect_count++;
    if (residual > s_carry_wait_max_us) s_carry_wait_max_us = residual;
    if (residual > S_CARRY_WAIT_OVER_US) s_carry_wait_over_count++;
  }
  // Still pending after the short timeout = a genuine miss.
  if (mA->_fire_pending) { mA->_fire_pending = false; mA->_last_reply_valid = false; s_rx_miss++; }
  if (mB->_fire_pending) { mB->_fire_pending = false; mB->_last_reply_valid = false; s_rx_miss++; }
  // RX overflow (MCP2515 EFLG RX0OVR|RX1OVR, bits 6-7) — rising-edge count (flag not clearable here).
  const uint8_t RX_OVR_MASK = (1 << 6) | (1 << 7);
  bool ovr = (can->getError() & RX_OVR_MASK) != 0;
  if (ovr && !s_ovr_prev) s_rx_overflow++;
  s_ovr_prev = ovr;
}
