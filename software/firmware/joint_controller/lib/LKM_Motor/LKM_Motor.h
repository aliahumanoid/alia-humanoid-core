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

  /**
   * @brief Set motor encoder counts per revolution for revolution tracking
   * @param counts Encoder counts per revolution (65536 for 18-bit, 32768 for 15-bit, 16384 for 14-bit)
   *
   * Determines the wrap-around threshold for revolution tracking.
   * Must match the motor's actual encoder resolution (as reported in 0xA1 response).
   * Default is 65536 (18-bit encoder, used by MG4005/MG series).
   */
  void setEncoderCountsPerRev(uint32_t counts);

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

  /**
   * @brief Read multi-loop angles from two motors in a single pipelined CAN transaction
   * @param motorA Pointer to first motor (agonist)
   * @param motorB Pointer to second motor (antagonist)
   * @return PipelinedAngleData with both angles and total pipeline time
   *
   * Sends both 0x92 commands back-to-back, then polls for both responses
   * in a single loop. Saves ~400-500µs vs two sequential getMultiAngleSync() calls.
   * Both motors must share the same CAN bus.
   */
  static PipelinedAngleData getMultiAnglePairPipelined(
      LKM_Motor *motorA, LKM_Motor *motorB);

  /**
   * @brief Send torque commands to two motors and read back their state in one pipelined operation
   * @param motorA Pointer to first motor (agonist)
   * @param torqueA Torque command for motor A (before limiting/inversion)
   * @param motorB Pointer to second motor (antagonist)
   * @param torqueB Torque command for motor B (before limiting/inversion)
   * @return PipelinedTorqueResponseData with both motor states and tracked angles
   *
   * Sends 0xA1 torque commands to both motors and reads back their state
   * (temperature, iq current, speed, encoder position) in a single pipelined
   * CAN transaction — capturing response data that setTorque() discards.
   *
   * Phase 1 (shadow mode): called alongside getMultiAnglePairPipelined() for
   * revolution tracking validation. 0x92 remains the angle source of truth.
   * Phase 2 (after validation): can replace the separate 0x92 read + setTorque()
   * calls, reducing CAN transactions from 4 to 2 per cycle.
   *
   * Revolution tracking must be initialized via initRevTracking() before the
   * tracked angle in the response will be valid.
   */
  static PipelinedTorqueResponseData setTorquePairPipelined(
      LKM_Motor *motorA, int torqueA,
      LKM_Motor *motorB, int torqueB);

  // ---------------------------------------------------------------
  // BURST-AWARE MOTOR COMMANDS (fire a pair, collect ≤2 replies, repeat)
  // ---------------------------------------------------------------
  // The MCP2515 motor bus has only 2 RX + 3 TX buffers, so we NEVER keep more than ONE pair (2
  // motors) outstanding: the 250Hz loop fires a DOF's pair, poll-drains its ≤2 replies (short
  // timeout), then fires the next pair. This respects the hardware buffers (firing all 4 at once
  // overflows RX, and a 4th un-checked TX can be silently dropped), stays well under the 4ms budget
  // (a parallel pair round-trip, NOT the old per-pair busy-wait), and is GENERIC over the command
  // type — fireTorque (0xA1) and fireAngle2 (0xA4, Loop 2) both expect the SAME state reply, so
  // collectPair handles both. Rev-tracking is per-motor (routed by CAN id). The synchronous
  // setTorquePairPipelined() above stays for the off-loop consumers (free-capture, vel-test).

  /** Send the 0xA1 torque command non-blocking. Returns false (and counts tx_fail) if the MCP2515 TX
   *  queue refused it. On success records the outstanding transaction (lastReplyValid() reset false). */
  bool fireTorque(int torque);

  /** Send the 0xA4 position command (angle + maxSpeed) non-blocking — same transaction discipline as
   *  fireTorque, for the Loop 2 motor-position inner loop. Returns false (counts tx_fail) on TX refuse. */
  bool fireAngle2(float angle, uint16_t maxSpeed);

  /** Send the 0x92 multi-turn angle READ command non-blocking — SAME transaction discipline as fireTorque
   *  (records _fire_pending / _fire_t_us / _fire_canId / _fire_cmd=0x92) so collectPair/carry track it
   *  identically. S2 SUBSTITUTE-FIRE: on a watchdog-due cycle the DOF fires this INSTEAD of the torque pair;
   *  the reply is routed by collectPair to _last92 (via parseMultiAngleReply). Returns false (counts tx_fail)
   *  on TX refuse. The motor holds its previous 0xA1 torque for this cycle (ZOH — no torque frame is sent). */
  bool fire92();

  /** Last ROUTED 0x92 reply parsed by collectPair (S2 substitute-fire). Valid only after a fire92()+collect
   *  where lastReplyValid() is true. */
  const MultiAngleData &last92() const { return _last92; }

  /** Poll-drain the bus until BOTH motors of this pair have replied or timeout_us elapses. Only this
   *  pair is outstanding (<=2), so the 2-deep MCP2515 RX buffer never overflows. Ingests each reply
   *  (rev-tracking) as it arrives, in order; a still-pending motor at timeout is a miss. Reads the EFLG
   *  RX-overflow flag. Updates the tx_fail / rx_miss / rx_overflow diagnostic counters.
   *
   *  presweep (S2 CARRY, default false = bit-identical for the serial/interleave callers): run ONE
   *  UNCONDITIONAL non-blocking pass that reads+routes every CAN_MSGAVAIL frame already in the RX
   *  buffers BEFORE the deadline loop. A carried pair's replies have ALREADY landed by drain time
   *  (age > flight), so an age-based deadline computes ~0 and the plain path would declare a MISS
   *  without reading them -> permanent zero-fire recovery churn (FATAL-2). The pre-sweep consumes the
   *  buffered replies first; the deadline then governs only the residual WAIT for not-yet-arrived frames. */
  static void collectPair(LKM_Motor *mA, LKM_Motor *mB, uint32_t timeout_us, bool presweep = false);

  bool lastReplyValid() const { return _last_reply_valid; }  ///< was the last fire's reply collected?
  bool firePending() const { return _fire_pending; }         ///< is a command's reply still outstanding?
  void clearFire() { _fire_pending = false; }                ///< drop an outstanding fire (e-stop preempt)
  uint32_t fireTimestampUs() const { return _fire_t_us; }    ///< micros() latched at the last fire (S2 CARRY time authority)

  /// Drain and DISCARD any stale frames sitting in the MCP2515 RX buffers (bounded by
  /// maxDrain). Interleave-mode pre-fire sweep: late replies from a previously timed-out
  /// collect are ownerless (collectPair cleared _fire_pending on the miss) and would occupy
  /// both RX buffers through the next pair's unattended flight window, hard-dropping the
  /// fresh replies. Call ONLY when no fire is pending on this bus. Returns frames discarded.
  int flushStaleRx(int maxDrain);

  /// O4a: enable /INT-gated draining in collectPair (-1 = disabled, pure SPI poll).
  /// SPI fallback every 16 idle spins runs regardless, so a bad INT line cannot hang the drain.
  static void setCollectIntPin(int pin);
  /// Stage-1 instrumentation: fire->reply latency histogram, 8 buckets
  /// (<200,<300,<400,<500,<700,<1000,<1500,>=1500 us), cumulative.
  static const uint32_t *latencyHistogram();
  static void resetLatencyHistogram();

  // Bus-health diagnostic counters (cumulative since resetBusDiag()).
  static uint32_t txFailCount();
  static uint32_t rxMissCount();
  static uint32_t rxOverflowCount();
  static void resetBusDiag();

  // S2 carry residual-wait stats (G1 carry-correctness signal; cumulative). residual ~0 = carry hid
  // the flight; carryWaitOverCount/carryWaitMaxUs non-zero = the carry did not fully hide it.
  static uint32_t carryCollectCount();
  static uint32_t carryWaitOverCount();
  static uint32_t carryWaitMaxUs();
  static void resetCarryWaitStats();

  // ---------------------------------------------------------------
  // REVOLUTION TRACKING (for 0xA1 response-based angle reading)
  // ---------------------------------------------------------------

  /**
   * @brief Initialize revolution tracking from absolute position
   * @param absMotorAngle_centideg Absolute motor angle from 0x92 in 0.01° units (motor shaft)
   * @param currentEncoder Current single-turn encoder value from 0xA1 response
   *
   * Must be called once (typically on IDLE→MOVING transition) before
   * setTorquePairPipelined() can return valid tracked angles.
   * Uses the absolute 0x92 reading to determine the actuator/output
   * revolution count, and the 0xA1 encoder for sub-revolution actuator
   * position.
   */
  void initRevTracking(int64_t absMotorAngle_centideg, uint16_t currentEncoder);

  /**
   * @brief Update tracked angle from new 0xA1 encoder reading
   * @param currentEncoder New single-turn encoder value from 0xA1 response
   *
   * Detects encoder wrap-arounds (crossing 0/max boundary) and updates
   * the actuator/output revolution counter accordingly.
   */
  void updateRevTracking(uint16_t currentEncoder);

  /**
   * @brief Get the tracked multi-turn angle in output degrees
   * @return Angle with offset and inversion applied (same as getMultiAngleSync)
   *
   * Combines actuator/output revolution count with current encoder position,
   * then applies offset and inversion — identical output to 0x92 reading.
   */
  float getTrackedAngle() const;

  /**
   * @brief Snap the tracked angle to a freshly-measured 0x92 absolute (measurement path only).
   * @param absMotorAngle_centideg Absolute motor-side angle from a routed 0x92 reply, 0.01° units.
   *
   * S2 substitute-fire: on a within-threshold watchdog cycle the DOF fired 0x92 (not torque), so no fresh
   * 0xA1 encoder advanced the tracking this cycle. Correct ONLY the revolution count so getTrackedAngle()
   * reflects the just-validated absolute, keeping the ongoing single-turn wrap tracking (_prevEncoder) as the
   * sub-revolution source. This is the SAME revolution math as initRevTracking() but reusing the live encoder
   * (no re-anchor of _prevEncoder — the rev-anchor path is left untouched). Removes the 2-cycle repeated-sample
   * stutter (the substitute cycle + its consume cycle would otherwise feed the inner D/GMS the same angle).
   * No-op until tracking is initialized.
   */
  void setTrackedAngleFromAbsolute(int64_t absMotorAngle_centideg);

  /**
   * @brief Check if revolution tracking has been initialized
   */
  bool isRevTrackInit() const;

  /**
   * @brief Reset revolution tracking (e.g., on motor power cycle or state reset)
   */
  void resetRevTracking();

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
  uint16_t motorEncoderPosition; ///< Encoder position (motor-side, 0-65535 for 18-bit)

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
  int16_t _maxTorque;       ///< Per-instance torque limit (default 2048)

  // ---------------------------------------------------------------
  // REVOLUTION TRACKING STATE
  // ---------------------------------------------------------------

  uint32_t _encoderCountsPerRev; ///< Encoder counts per encoder/output revolution (65536 for 18-bit field, 32768 for 15-bit, 16384 for 14-bit)
  int32_t _revolutions;          ///< Tracked full actuator/output revolutions
  uint16_t _prevEncoder;         ///< Previous encoder reading from 0xA1 response
  bool _revTrackInit;            ///< Whether revolution tracking has been initialized

  // ---------------------------------------------------------------
  // BURST-AWARE COMMAND TRANSACTION STATE (fireTorque / fireAngle2 / collectPair)
  // ---------------------------------------------------------------
  bool _fire_pending = false;      ///< an 0xA1 fire is outstanding, awaiting its reply
  bool _last_reply_valid = false;  ///< the last fire's reply was collected (else missed)
  uint32_t _fire_t_us = 0;         ///< micros() at the last fire
  unsigned long _fire_canId = 0;   ///< CAN id the reply will carry (0x140 + motorID)
  uint8_t _fire_cmd = 0;           ///< command byte of the outstanding fire (0xA1/0xA4/0x92) — collectPair
                                   ///< routes the reply to the matching parser by this byte (S2 sub92)
  MultiAngleData _last92 = {};     ///< last ROUTED 0x92 reply parsed by collectPair (S2 substitute-fire); the
                                   ///< consume reads it instead of the blocking getMultiAnglePairPipelined result

  /** Parse a motor state reply (temp/iq/speed/encoder), store it, and advance rev-tracking. */
  void ingestTorqueReply(const unsigned char *buf);

  /** Parse an 0x92 multi-turn angle reply (DATA[1..7] = int56 motor-side centideg) into output-space.
   *  Factored out of getMultiAnglePairPipelined() so a ROUTED 0x92 reply (S2 substitute-fire, collectPair)
   *  parses through the exact same conversion (reductionGear / offsetEncoder / invertEncoder). Static: it
   *  reads only per-motor calibration and the 8-byte frame, with no bus/transaction side effects.
   *  waitTime is left 0 (the caller has no round-trip window for a carried reply). */
  static void parseMultiAngleReply(LKM_Motor *motor, const unsigned char *buf, MultiAngleData &data);

  /** Shared non-blocking send + transaction record for a pre-built command frame (0xA1 / 0xA4). */
  bool fireCommand(const unsigned char *buf);
};

#endif // LKM_MOTOR_H
