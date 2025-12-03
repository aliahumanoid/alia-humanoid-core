/**
 * @file DirectEncoders.h
 * @brief Direct MT6835 encoder reading without intermediate Pico
 * 
 * This class reads MT6835 magnetic encoders directly via SPI0, eliminating
 * the need for a separate encoder Pico. It provides:
 * - Direct SPI communication with up to 3 MT6835 encoders
 * - Multi-turn angle tracking
 * - Read validation with CRC check, spike detection, and status verification
 * - Automatic retry on failed reads
 * - Zero offset management
 * 
 * Hardware Setup:
 * - SPI0: MISO=GP16, SCK=GP18, MOSI=GP19
 * - CS pins: GP17 (encoder 1), GP20 (encoder 2), GP21 (encoder 3)
 * 
 * @see MagneticSensorMT6835.h for underlying sensor driver
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <MagneticSensorMT6835.h>
#include <global.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

// Maximum number of encoders supported
#define DIRECT_ENCODER_COUNT 3

// Read validation parameters (same as joint_encoders)
#define ENCODER_MAX_RPM 60.0f          // Max speed in RPM
#define ENCODER_VALIDATION_MARGIN 1.5f // Safety margin 50%
#define ENCODER_MAX_READ_ATTEMPTS 2    // Max read retry attempts
#define ENCODER_RETRY_DELAY_US 50      // Delay between retry attempts (µs)

// Flash storage constants
#define ENCODER_FLASH_MAGIC_NUMBER 0xE5C0FFEE    // Magic number for validation ("ESCOFFEE")
#define ENCODER_FLASH_STRUCT_VERSION 1            // Structure version
#define ENCODER_FLASH_TARGET_OFFSET (512 * 1024)  // 512KB offset (separate from other data)

/**
 * @brief Flash storage structure for encoder calibration data
 */
struct EncoderFlashData {
  uint32_t magic_number;       ///< Magic number for validation
  uint16_t version;            ///< Structure version
  uint16_t checksum;           ///< Checksum for data integrity
  float offsets[DIRECT_ENCODER_COUNT];  ///< Encoder zero offsets (radians)
};

/**
 * @class DirectEncoders
 * @brief Direct multi-encoder reader for MT6835 sensors
 * 
 * Reads up to 3 MT6835 magnetic encoders directly via SPI0.
 * Provides validated multi-turn angle tracking with error detection.
 */
class DirectEncoders {
public:
  /**
   * @brief Constructor
   * @param encoder_invert Array of inversion flags per encoder (optional)
   */
  DirectEncoders(bool *encoder_invert = nullptr);
  
  /**
   * @brief Initialize the encoder system
   * 
   * Sets up SPI0 and initializes all connected encoders.
   */
  void begin();
  
  /**
   * @brief Update encoder readings
   * 
   * Performs a single SPI transaction to read all connected encoders.
   * Call this once per control cycle (e.g., at 500Hz).
   */
  void update();
  
  /**
   * @brief Check if last update produced valid data
   * @return true if data is valid, false if read failed
   */
  bool isDataValid() const { return _dataValid; }
  
  /**
   * @brief Get the current angle for a specific encoder
   * @param encoder_index Encoder index (0-2)
   * @return Angle in degrees (with multi-turn tracking)
   */
  float getAngle(uint8_t encoder_index) const;
  
  /**
   * @brief Get raw angle in radians for a specific encoder
   * @param encoder_index Encoder index (0-2)
   * @return Raw angle in radians [0, 2π]
   */
  float getRawAngleRad(uint8_t encoder_index) const;
  
  /**
   * @brief Set zero offset for an encoder
   * @param encoder_index Encoder index (0-2)
   * @param offset_rad Zero offset in radians
   */
  void setOffset(uint8_t encoder_index, float offset_rad);
  
  /**
   * @brief Get zero offset for an encoder
   * @param encoder_index Encoder index (0-2)
   * @return Zero offset in radians
   */
  float getOffset(uint8_t encoder_index) const;
  
  /**
   * @brief Reset an encoder to zero at current position (CALL FROM CORE0 ONLY!)
   * @param encoder_index Encoder index (0-2)
   * @param save_to_flash If true, saves offset to flash memory (default: true)
   * @note This method accesses SPI - must only be called from Core0
   */
  void resetEncoder(uint8_t encoder_index, bool save_to_flash = true);
  
  /**
   * @brief Reset all encoders to zero at current position (CALL FROM CORE0 ONLY!)
   * @note This method accesses SPI - must only be called from Core0
   */
  void resetAllEncoders();
  
  /**
   * @brief Request encoder reset from Core1 (thread-safe)
   * @param encoder_index Encoder index (0-2), or 0xFF for all
   * @note Core0 will execute the reset on next update() cycle
   */
  void requestReset(uint8_t encoder_index);
  
  /**
   * @brief Process pending reset requests (CALL FROM CORE0 ONLY!)
   * @note Called automatically by update(), but can be called manually
   */
  void processPendingResets();
  
  /**
   * @brief Get error count for an encoder
   * @param encoder_index Encoder index (0-2)
   * @return Consecutive error count
   */
  uint32_t getErrorCount(uint8_t encoder_index) const;
  
  /**
   * @brief Check if an encoder is connected/enabled
   * @param encoder_index Encoder index (0-2)
   * @return true if connected, false otherwise
   */
  bool isEncoderConnected(uint8_t encoder_index) const;
  
  /**
   * @brief Set encoder connection status
   * @param encoder_index Encoder index (0-2)
   * @param connected Connection status
   */
  void setEncoderConnected(uint8_t encoder_index, bool connected);
  
  /**
   * @brief Get the status byte from last read
   * @param encoder_index Encoder index (0-2)
   * @return MT6835 status byte
   */
  uint8_t getStatus(uint8_t encoder_index) const;
  
  // =========================================================================
  // COMPATIBILITY METHODS (match old Encoders API)
  // =========================================================================
  
  /**
   * @brief Set encoder inversion flag (compatibility with old API)
   * @param encoder_index Encoder index (0-2)
   * @param invert true to invert angle
   */
  void setEncoderInvert(uint8_t encoder_index, bool invert);
  
  /**
   * @brief Get encoder inversion flag
   * @param encoder_index Encoder index (0-2)
   * @return true if inverted
   */
  bool getEncoderInvert(uint8_t encoder_index) const;
  
  /**
   * @brief Set joint offset (alias for setOffset, converts degrees to radians)
   * @param encoder_index Encoder index (0-2)
   * @param offset_deg Zero offset in degrees
   * @param save_to_flash If true, saves offset to flash memory (default: true)
   */
  void setJointOffset(uint8_t encoder_index, float offset_deg, bool save_to_flash = true);
  
  /**
   * @brief Get joint angle (compatibility method)
   * @param encoder_index Encoder index (0-2)
   * @param is_valid Output: true if reading is valid
   * @return Angle in degrees with multi-turn tracking
   */
  float getJointAngle(uint8_t encoder_index, bool &is_valid);
  
  /**
   * @brief Zero encoders (alias for resetEncoder)
   * @param encoder_index Encoder index to reset (0-2), or 0xFF for all
   */
  void zeroEncoders(uint8_t encoder_index = 0xFF);
  
  /**
   * @brief Get raw encoder count (compatibility method)
   * @param encoder_index Encoder index (0-2)
   * @return Raw count value (calculated from raw angle, MT6835 is 21-bit)
   */
  int32_t getCount(uint8_t encoder_index) const;
  
  // =========================================================================
  // FLASH STORAGE METHODS
  // =========================================================================
  
  /**
   * @brief Save encoder offsets to flash memory
   * @return true if save successful, false otherwise
   */
  bool saveOffsetsToFlash();
  
  /**
   * @brief Load encoder offsets from flash memory
   * @return true if load successful, false otherwise
   */
  bool loadOffsetsFromFlash();
  
  /**
   * @brief Check if flash data was loaded successfully
   * @return true if flash data is valid
   */
  bool isFlashDataValid() const { return _flashDataValid; }

private:
  // SPI object for encoder communication
  SPIClassRP2040 *_spi;
  
  // MT6835 sensor objects
  MagneticSensorMT6835 *_sensors[DIRECT_ENCODER_COUNT];
  
  // CS pins for each encoder
  int _cs_pins[DIRECT_ENCODER_COUNT];
  
  // Connection status
  bool _connected[DIRECT_ENCODER_COUNT];
  
  // Inversion flags
  bool _invert[DIRECT_ENCODER_COUNT];
  
  // Zero offsets (radians)
  float _offsets[DIRECT_ENCODER_COUNT];
  
  // Multi-turn tracking
  int32_t _turns[DIRECT_ENCODER_COUNT];
  float _last_angles[DIRECT_ENCODER_COUNT];  // Last angle in [0, 2π] for turn detection
  float _last_valid_angles[DIRECT_ENCODER_COUNT];  // Last valid angle for validation
  
  // Total angles in degrees (with multi-turn)
  float _total_angles_deg[DIRECT_ENCODER_COUNT];
  
  // Error tracking
  uint32_t _error_counts[DIRECT_ENCODER_COUNT];
  
  // Reset synchronization (for inter-core safety)
  volatile bool _skip_validation[DIRECT_ENCODER_COUNT];  // Skip spike validation after reset
  volatile bool _pending_reset[DIRECT_ENCODER_COUNT];    // Pending reset request from Core1
  volatile bool _pending_save_flash;                      // Pending flash save request
  
  // Data validity
  bool _dataValid;
  bool _flashDataValid;  // Track if flash data was loaded correctly
  
  // Timing
  unsigned long _last_read_us;
  
  /**
   * @brief Calculate checksum for flash data validation
   * @param data Pointer to data buffer
   * @param size Size of data in bytes
   * @return Checksum value
   */
  uint16_t calculateChecksum(const uint8_t *data, size_t size);
  
  /**
   * @brief Read and validate a single encoder
   * @param encoder_index Encoder index (0-2)
   * @param delta_us Time since last read in microseconds
   * @return Validated raw angle in radians, or last value on failure
   */
  float readEncoderWithValidation(uint8_t encoder_index, unsigned long delta_us);
};

// Global instance
extern DirectEncoders directEncoders;

