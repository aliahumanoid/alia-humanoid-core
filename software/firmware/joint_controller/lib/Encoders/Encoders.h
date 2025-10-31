/**
 * @file Encoders.h
 * @brief Multi-encoder SPI communication and angle management
 * 
 * This class manages communication with multiple absolute magnetic encoders
 * via a custom SPI protocol. It supports:
 * - Reading up to 6 encoders simultaneously via daisy-chained SPI
 * - Per-encoder angle inversion for different motor orientations
 * - Per-encoder offset adjustment for zero position calibration
 * - Data validation and sync sequence detection
 * - Selective zeroing (single encoder or all encoders)
 * 
 * HARDWARE SETUP:
 * Encoders are connected in a daisy-chain configuration using custom SPI
 * protocol. Each encoder board (joint_encoders firmware) sends its data
 * sequentially in a synchronized frame.
 * 
 * ANGLE CALCULATION:
 * Raw encoder counts are converted to angles in degrees, then:
 * 1. Optionally inverted (if encoder is mounted backwards)
 * 2. Offset adjusted for zero position calibration
 * 
 * Final angle = (raw_angle * invert_factor) + offset
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// ===================================================================
// CONSTANTS
// ===================================================================

// Maximum number of encoders supported in daisy-chain
#define ENCODER_COUNT 6

// ===================================================================
// SPI STATUS CODES
// ===================================================================

/**
 * SPI communication status codes returned by encoder boards
 */
const uint8_t STATUS_OK               = 0x01; // Data successfully loaded
const uint8_t STATUS_DATA_LOAD_FAILED = 0x02; // Failed to read encoder data

// ===================================================================
// ENCODER ZEROING COMMANDS
// ===================================================================

/**
 * Commands to zero encoder positions (sent via SPI)
 */
const uint8_t CMD_ZERO_ALL  = 0xAA; // Zero all encoders simultaneously
const uint8_t CMD_ZERO_DOF0 = 0x0A; // Zero only DOF 0 encoder
const uint8_t CMD_ZERO_DOF1 = 0x0B; // Zero only DOF 1 encoder
const uint8_t CMD_ZERO_DOF2 = 0x0C; // Zero only DOF 2 encoder

// ===================================================================
// ENCODERS CLASS
// ===================================================================

/**
 * @brief Multi-encoder manager with SPI communication
 * 
 * Manages a daisy-chain of up to 6 absolute magnetic encoders connected
 * via custom SPI protocol. Handles data synchronization, angle calculation,
 * and per-encoder configuration (inversion, offset).
 * 
 * TYPICAL USAGE:
 * ```
 * bool invert_flags[6] = {false, true, false, true, false, false};
 * Encoders encoders(RX_PIN, CS_PIN, SCK_PIN, TX_PIN, invert_flags);
 * encoders.begin();
 * 
 * // In main loop:
 * encoders.update();
 * bool valid;
 * float angle = encoders.getJointAngle(0, valid);
 * if (valid) {
 *   // Use angle
 * }
 * ```
 */
class Encoders {
public:
  // ---------------------------------------------------------------
  // INITIALIZATION
  // ---------------------------------------------------------------
  
  /**
   * @brief Constructor - configure encoder manager
   * 
   * @param rx_pin SPI MISO pin (receive data from encoders)
   * @param cs_pin SPI chip select pin (active low)
   * @param sck_pin SPI clock pin
   * @param tx_pin SPI MOSI pin (send commands to encoders)
   * @param encoder_invert Optional array of 6 bool flags for per-encoder inversion
   *                       (true = invert angle, false = normal)
   *                       Pass nullptr to disable all inversions
   */
  Encoders(int rx_pin, int cs_pin, int sck_pin, int tx_pin, bool *encoder_invert = nullptr);
  
  /**
   * @brief Initialize SPI communication
   * 
   * Must be called once before any encoder operations.
   * Sets up SPI peripheral and initializes internal state.
   */
  void begin();

  // ---------------------------------------------------------------
  // DATA ACQUISITION
  // ---------------------------------------------------------------
  
  /**
   * @brief Read latest encoder data via SPI
   * 
   * Performs a complete SPI transaction to read all encoder values.
   * Should be called periodically (e.g., every control loop iteration).
   * Updates internal encoder counts and data validity flag.
   */
  void update();
  
  /**
   * @brief Get raw encoder count for a specific encoder
   * 
   * @param encoder_index Encoder index (0-5)
   * @return Raw encoder count (typically 14-bit or 16-bit value)
   */
  int32_t getCount(uint8_t encoder_index);
  
  /**
   * @brief Get calibrated joint angle for a specific encoder
   * 
   * Returns angle in degrees, with inversion and offset applied.
   * 
   * @param encoder_index Encoder index (0-5)
   * @param isValid Output: set to true if data is valid, false if stale/invalid
   * @return Calibrated angle in degrees (accounting for inversion and offset)
   */
  float getJointAngle(uint8_t encoder_index, bool &isValid);
  
  /**
   * @brief Print all encoder counts to serial (debug)
   * 
   * Outputs raw counts for all encoders, useful for diagnostics.
   */
  void printCounts();

  // ---------------------------------------------------------------
  // CALIBRATION & CONFIGURATION
  // ---------------------------------------------------------------
  
  /**
   * @brief Zero encoder positions
   * 
   * Sends zero command to encoder boards to reset their reference position.
   * Can zero all encoders or a specific one.
   * 
   * @param encoder_index Encoder to zero (0-5), or -1 to zero all encoders
   */
  void zeroEncoders(int8_t encoder_index = -1);
  
  /**
   * @brief Set angular offset for a specific encoder
   * 
   * Adds a constant offset to the encoder reading for zero position adjustment.
   * Useful for calibrating mechanical zero positions.
   * 
   * @param encoder_index Encoder index (0-5)
   * @param offset Offset in degrees (added to final angle)
   */
  void setJointOffset(uint8_t encoder_index, float offset);
  
  /**
   * @brief Get current angular offset for an encoder
   * 
   * @param encoder_index Encoder index (0-5)
   * @return Current offset in degrees
   */
  float getJointOffset(uint8_t encoder_index) const;
  
  /**
   * @brief Set angle inversion flag for a specific encoder
   * 
   * Used when encoder is mounted backwards relative to joint rotation.
   * 
   * @param encoder_index Encoder index (0-5)
   * @param invert true to invert angle, false for normal
   */
  void setEncoderInvert(uint8_t encoder_index, bool invert);

  // ---------------------------------------------------------------
  // STATUS
  // ---------------------------------------------------------------
  
  /**
   * @brief Check if encoder data is valid
   * 
   * Data becomes invalid if:
   * - SPI communication fails
   * - Sync sequence not found in received data
   * - Checksum/validation errors
   * 
   * @return true if latest data is valid, false otherwise
   */
  bool isDataValid() const {
    return _dataValid;
  }

private:
  // ---------------------------------------------------------------
  // PRIVATE MEMBERS
  // ---------------------------------------------------------------
  
  // SPI pin configuration
  int _rx_pin;  // MISO
  int _cs_pin;  // Chip select
  int _sck_pin; // Clock
  int _tx_pin;  // MOSI
  
  // Encoder data
  int32_t _encoder_counts[ENCODER_COUNT]; // Raw encoder counts
  unsigned long _last_read_time;          // Timestamp of last successful read
  size_t syncIndex;                       // Index of sync sequence in SPI buffer
  
  // Per-encoder configuration
  bool _encoder_invert[ENCODER_COUNT] = {false}; // Inversion flags
  float _joint_offsets[ENCODER_COUNT];           // Angular offsets (degrees)
  
  // Data validity
  bool _dataValid = false; // True if last SPI read was successful

  // ---------------------------------------------------------------
  // PRIVATE METHODS
  // ---------------------------------------------------------------
  
  /**
   * @brief Find synchronization sequence in SPI buffer
   * 
   * Searches for the sync pattern that marks the start of valid encoder data.
   * Required for proper frame alignment in daisy-chain configuration.
   * 
   * @param buffer SPI receive buffer
   * @param bufferSize Size of buffer in bytes
   * @param syncIndex Output: index where sync sequence was found
   * @return true if sync sequence found, false otherwise
   */
  bool findSyncSequence(uint8_t *buffer, size_t bufferSize, size_t &syncIndex);
};

#endif // ENCODERS_H
