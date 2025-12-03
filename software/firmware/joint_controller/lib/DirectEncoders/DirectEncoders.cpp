/**
 * @file DirectEncoders.cpp
 * @brief Implementation of direct MT6835 encoder reading
 * 
 * See DirectEncoders.h for class documentation.
 */

#include "DirectEncoders.h"
#include <debug.h>

// External flag for flash operation synchronization with Core1
extern volatile bool flash_operation_in_progress;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// SPI settings for MT6835 (1 MHz for better signal integrity with bridge wiring)
// Reduced from 2 MHz to improve reliability over longer/bridged connections
static SPISettings directEncoderSPISettings(1000000, MSBFIRST, SPI_MODE3);

// ============================================================================
// CONSTRUCTOR
// ============================================================================

DirectEncoders::DirectEncoders(bool *encoder_invert) {
  // Store CS pins from global definitions
  _cs_pins[0] = ENCODER_CS_1;
  _cs_pins[1] = ENCODER_CS_2;
  _cs_pins[2] = ENCODER_CS_3;
  
  // Initialize state
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    _connected[i] = false;
    _invert[i] = (encoder_invert != nullptr) ? encoder_invert[i] : false;
    _offsets[i] = 0.0f;
    _turns[i] = 0;
    _last_angles[i] = 0.0f;
    _last_valid_angles[i] = 0.0f;
    _total_angles_deg[i] = 0.0f;
    _error_counts[i] = 0;
    _skip_validation[i] = false;   // Inter-core sync flag
    _pending_reset[i] = false;     // Pending reset from Core1
    _sensors[i] = nullptr;
  }
  
  _pending_save_flash = false;
  
  _dataValid = false;
  _flashDataValid = false;
  _last_read_us = 0;
  _spi = nullptr;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void DirectEncoders::begin() {
  LOG_INFO("=== DIRECT ENCODER INITIALIZATION ===");
  
  // Load saved offsets from flash FIRST (before SPI init)
  if (loadOffsetsFromFlash()) {
    LOG_INFO("Loaded encoder offsets from flash");
  } else {
    LOG_WARN("No saved encoder offsets found, using defaults (0)");
  }
  
  // Configure SPI0 pins
  SPI.setRX(ENCODER_SPI_MISO);
  SPI.setSCK(ENCODER_SPI_SCK);
  SPI.setTX(ENCODER_SPI_MOSI);
  SPI.begin();
  
  LOG_INFO_F("SPI0 configured: MISO=GP%d, SCK=GP%d, MOSI=GP%d", 
             ENCODER_SPI_MISO, ENCODER_SPI_SCK, ENCODER_SPI_MOSI);
  
  // Configure CS pins as outputs and set high (deselected)
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    pinMode(_cs_pins[i], OUTPUT);
    digitalWrite(_cs_pins[i], HIGH);
  }
  
  // Create and initialize sensor objects for connected encoders
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    if (_connected[i]) {
      _sensors[i] = new MagneticSensorMT6835(_cs_pins[i], directEncoderSPISettings);
      _sensors[i]->init(&SPI);
      _sensors[i]->checkcrc = true;  // Enable CRC checking
      LOG_INFO_F("Encoder %d: Initialized on CS=GP%d (offset=%.4f rad)", 
                 i + 1, _cs_pins[i], _offsets[i]);
      
      // Get initial reading
      float initial_angle = _sensors[i]->getSensorAngle();
      if (initial_angle >= 0) {
        _last_angles[i] = initial_angle;
        _last_valid_angles[i] = initial_angle;
        // Skip spike validation on first real read (offset might have changed)
        _skip_validation[i] = true;
      }
    } else {
      LOG_INFO_F("Encoder %d: Skipped (not connected)", i + 1);
    }
  }
  
  _last_read_us = micros();
  LOG_INFO("=================================");
}

// ============================================================================
// UPDATE
// ============================================================================

void DirectEncoders::update() {
  // Process any pending reset requests from Core1 FIRST
  processPendingResets();
  
  unsigned long now_us = micros();
  unsigned long delta_us = now_us - _last_read_us;
  _last_read_us = now_us;
  
  _dataValid = true;  // Assume valid, set false if any encoder fails
  
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    if (!_connected[i] || _sensors[i] == nullptr) {
      continue;
    }
    
    // Read with validation
    float raw_angle = readEncoderWithValidation(i, delta_us);
    
    // Apply offset and normalize to [0, 2π]
    float angle = fmod(raw_angle - _offsets[i], 2 * PI);
    if (angle < 0) angle += 2 * PI;
    
    // Apply inversion if needed
    if (_invert[i]) {
      angle = 2 * PI - angle;
    }
    
    // Multi-turn tracking: detect wrap-around
    float delta_angle = angle - _last_angles[i];
    
    if (delta_angle > PI) {
      _turns[i]--;  // Crossed zero going negative
    } else if (delta_angle < -PI) {
      _turns[i]++;  // Crossed zero going positive
    }
    
    _last_angles[i] = angle;
    
    // Compute total angle in degrees (with multi-turn)
    _total_angles_deg[i] = (_turns[i] * 360.0f) + (angle * (180.0f / PI));
  }
}

// ============================================================================
// VALIDATION
// ============================================================================

float DirectEncoders::readEncoderWithValidation(uint8_t encoder_index, unsigned long delta_us) {
  if (encoder_index >= DIRECT_ENCODER_COUNT || _sensors[encoder_index] == nullptr) {
    return 0.0f;
  }
  
  MagneticSensorMT6835 *sensor = _sensors[encoder_index];
  
  // Check if validation should be skipped (after reset/zero operation from Core1)
  bool skip_spike_check = _skip_validation[encoder_index];
  if (skip_spike_check) {
    _skip_validation[encoder_index] = false;  // Clear flag (one-shot)
    LOG_DEBUG("Encoder " + String(encoder_index + 1) + ": Skipping spike validation (reset sync)");
  }
  
  // Compute maximum allowed variation based on elapsed time
  float max_delta_rad = (ENCODER_MAX_RPM * 2.0f * PI / 60.0f) * 
                        (delta_us / 1000000.0f) * ENCODER_VALIDATION_MARGIN;
  
  float raw_angle = 0.0f;
  bool valid_reading = false;
  int attempts = 0;
  
  while (!valid_reading && attempts < ENCODER_MAX_READ_ATTEMPTS) {
    attempts++;
    raw_angle = sensor->getSensorAngle();
    
    // Check 1: Value in valid range [0, 2π]
    if (raw_angle < 0 || raw_angle > 2 * PI) {
      if (attempts == 1) {
        if (raw_angle == -1) {
          LOG_ERROR("Encoder " + String(encoder_index + 1) + ": CRC error - corrupted data on SPI");
        } else {
          LOG_ERROR_F("Encoder %d: Angle out of range = %.4f", encoder_index + 1, raw_angle);
        }
      }
      delayMicroseconds(ENCODER_RETRY_DELAY_US);
      continue;
    }
    
    // Check 2: Reasonable variation (skip if reset just happened)
    if (!skip_spike_check && _last_valid_angles[encoder_index] > 0 && delta_us > 0) {
      float delta = fabs(raw_angle - _last_valid_angles[encoder_index]);
      // Handle wrap-around at 2π
      if (delta > PI) {
        delta = 2 * PI - delta;
      }
      
      if (delta > max_delta_rad) {
        if (attempts == 1) {
          LOG_ERROR_F("Encoder %d: Spike detected. Delta = %.2f deg (max = %.2f deg)",
                      encoder_index + 1, delta * 180.0f / PI, max_delta_rad * 180.0f / PI);
        }
        delayMicroseconds(ENCODER_RETRY_DELAY_US);
        continue;
      }
    }
    
    // Check 3: Sensor status check
    uint8_t status = sensor->getStatus();
    if (status & (MT6835_STATUS_OVERSPEED | MT6835_STATUS_WEAKFIELD | MT6835_STATUS_UNDERVOLT)) {
      if (attempts == 1) {
        LOG_ERROR_F("Encoder %d: Status error = 0x%02X", encoder_index + 1, status);
      }
      delayMicroseconds(ENCODER_RETRY_DELAY_US);
      continue;
    }
    
    // If we are here, the reading is valid
    valid_reading = true;
  }
  
  if (valid_reading) {
    // Reset error counter on successful read
    if (_error_counts[encoder_index] > 0) {
      LOG_INFO("Encoder " + String(encoder_index + 1) + ": Reading recovered successfully");
    }
    _error_counts[encoder_index] = 0;
    _last_valid_angles[encoder_index] = raw_angle;
    return raw_angle;
  } else {
    // All attempts failed
    _error_counts[encoder_index]++;
    _dataValid = false;
    
    LOG_ERROR_F("Encoder %d: All attempts failed. Consecutive errors = %u", 
                encoder_index + 1, _error_counts[encoder_index]);
    
    // Return last valid value to avoid jumps
    return _last_valid_angles[encoder_index];
  }
}

// ============================================================================
// GETTERS
// ============================================================================

float DirectEncoders::getAngle(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return 0.0f;
  return _total_angles_deg[encoder_index];
}

float DirectEncoders::getRawAngleRad(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return 0.0f;
  return _last_angles[encoder_index];
}

void DirectEncoders::setOffset(uint8_t encoder_index, float offset_rad) {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return;
  _offsets[encoder_index] = offset_rad;
}

float DirectEncoders::getOffset(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return 0.0f;
  return _offsets[encoder_index];
}

void DirectEncoders::resetEncoder(uint8_t encoder_index, bool save_to_flash) {
  if (encoder_index >= DIRECT_ENCODER_COUNT || _sensors[encoder_index] == nullptr) return;
  
  // Set offset to current raw angle
  float current_raw = _sensors[encoder_index]->getSensorAngle();
  if (current_raw >= 0) {
    _offsets[encoder_index] = current_raw;
    _turns[encoder_index] = 0;
    _last_angles[encoder_index] = 0.0f;
    // CRITICAL: Reset last_valid_angles to current raw to avoid spike detection
    _last_valid_angles[encoder_index] = current_raw;
    _total_angles_deg[encoder_index] = 0.0f;
    _error_counts[encoder_index] = 0;
    
    // Signal Core0 to skip spike validation on next read (inter-core sync)
    _skip_validation[encoder_index] = true;
    
    LOG_INFO_F("Encoder %d: Reset to zero (offset = %.2f rad)", 
               encoder_index + 1, current_raw);
    
    // Save to flash if requested
    if (save_to_flash) {
      saveOffsetsToFlash();
    }
  }
}

void DirectEncoders::resetAllEncoders() {
  LOG_INFO("Resetting all encoders to zero...");
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    if (_connected[i]) {
      resetEncoder(i, false);  // Don't save each one individually
    }
  }
  // Save all offsets to flash at once
  saveOffsetsToFlash();
}

// ============================================================================
// INTER-CORE RESET REQUESTS (thread-safe)
// ============================================================================

void DirectEncoders::requestReset(uint8_t encoder_index) {
  if (encoder_index == 0xFF) {
    // Request reset for all connected encoders
    for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
      if (_connected[i]) {
        _pending_reset[i] = true;
      }
    }
    LOG_INFO("Reset requested for all encoders (will execute on Core0)");
  } else if (encoder_index < DIRECT_ENCODER_COUNT) {
    _pending_reset[encoder_index] = true;
    LOG_INFO_F("Reset requested for encoder %d (will execute on Core0)", encoder_index + 1);
  }
  _pending_save_flash = true;  // Save after reset
}

void DirectEncoders::processPendingResets() {
  bool any_reset = false;
  
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    if (_pending_reset[i] && _connected[i] && _sensors[i] != nullptr) {
      _pending_reset[i] = false;  // Clear flag first
      
      // Execute reset (this is safe because we're on Core0)
      float current_raw = _sensors[i]->getSensorAngle();
      if (current_raw >= 0) {
        _offsets[i] = current_raw;
        _turns[i] = 0;
        _last_angles[i] = 0.0f;
        _last_valid_angles[i] = current_raw;
        _total_angles_deg[i] = 0.0f;
        _error_counts[i] = 0;
        _skip_validation[i] = true;
        
        LOG_INFO_F("Encoder %d: Reset executed (offset = %.2f rad = %.2f deg)", 
                   i + 1, current_raw, current_raw * 180.0f / PI);
        any_reset = true;
      }
    }
  }
  
  // Save to flash if any reset happened and flash save is pending
  if (any_reset && _pending_save_flash) {
    _pending_save_flash = false;
    saveOffsetsToFlash();
  }
}

uint32_t DirectEncoders::getErrorCount(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return 0;
  return _error_counts[encoder_index];
}

bool DirectEncoders::isEncoderConnected(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return false;
  return _connected[encoder_index];
}

void DirectEncoders::setEncoderConnected(uint8_t encoder_index, bool connected) {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return;
  _connected[encoder_index] = connected;
}

uint8_t DirectEncoders::getStatus(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT || _sensors[encoder_index] == nullptr) return 0;
  return _sensors[encoder_index]->getStatus();
}

// ============================================================================
// COMPATIBILITY METHODS (match old Encoders API)
// ============================================================================

void DirectEncoders::setEncoderInvert(uint8_t encoder_index, bool invert) {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return;
  _invert[encoder_index] = invert;
  LOG_DEBUG("Encoder " + String(encoder_index + 1) + ": inversion set to " + String(invert ? "true" : "false"));
}

bool DirectEncoders::getEncoderInvert(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return false;
  return _invert[encoder_index];
}

void DirectEncoders::setJointOffset(uint8_t encoder_index, float offset_deg, bool save_to_flash) {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return;
  
  // Convert degrees to radians for internal storage
  float offset_rad = offset_deg * (PI / 180.0f);
  _offsets[encoder_index] = offset_rad;
  
  // CRITICAL: Reset last_valid_angles to avoid spike detection after offset change
  // Read current raw angle and set it as last valid
  if (_sensors[encoder_index] != nullptr) {
    float current_raw = _sensors[encoder_index]->getSensorAngle();
    if (current_raw >= 0) {
      _last_valid_angles[encoder_index] = current_raw;
      // Also reset turns and total angle
      _turns[encoder_index] = 0;
      _last_angles[encoder_index] = 0.0f;
      _total_angles_deg[encoder_index] = 0.0f;
      _error_counts[encoder_index] = 0;
      
      // Signal Core0 to skip spike validation on next read (inter-core sync)
      _skip_validation[encoder_index] = true;
    }
  }
  
  LOG_INFO_F("Encoder %d: offset set to %.2f deg (%.4f rad)", 
             encoder_index + 1, offset_deg, _offsets[encoder_index]);
  
  // Save to flash if requested
  if (save_to_flash) {
    saveOffsetsToFlash();
  }
}

float DirectEncoders::getJointAngle(uint8_t encoder_index, bool &is_valid) {
  if (encoder_index >= DIRECT_ENCODER_COUNT) {
    is_valid = false;
    return 0.0f;
  }
  
  // Check if this encoder is connected and has no errors
  is_valid = _connected[encoder_index] && (_error_counts[encoder_index] == 0);
  
  // Return the angle in degrees (already calculated with multi-turn)
  return _total_angles_deg[encoder_index];
}

void DirectEncoders::zeroEncoders(uint8_t encoder_index) {
  if (encoder_index == 0xFF) {
    resetAllEncoders();
  } else if (encoder_index < DIRECT_ENCODER_COUNT) {
    resetEncoder(encoder_index);
  }
}

int32_t DirectEncoders::getCount(uint8_t encoder_index) const {
  if (encoder_index >= DIRECT_ENCODER_COUNT) return 0;
  // Convert raw angle (radians) to encoder count
  // MT6835 is 21-bit (2^21 = 2097152 counts per revolution)
  // Count = (angle_rad / 2π) × 2097152 + turns × 2097152
  float raw_rad = _last_angles[encoder_index];
  int32_t single_turn_count = (int32_t)((raw_rad / (2.0f * PI)) * 2097152.0f);
  return _turns[encoder_index] * 2097152 + single_turn_count;
}

// ============================================================================
// FLASH STORAGE
// ============================================================================

uint16_t DirectEncoders::calculateChecksum(const uint8_t *data, size_t size) {
  uint16_t checksum = 0;
  for (size_t i = 0; i < size; i++) {
    checksum += data[i];
  }
  return checksum;
}

bool DirectEncoders::saveOffsetsToFlash() {
  EncoderFlashData data;
  
  // Set header fields
  data.magic_number = ENCODER_FLASH_MAGIC_NUMBER;
  data.version = ENCODER_FLASH_STRUCT_VERSION;
  
  // Copy current offsets
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    data.offsets[i] = _offsets[i];
  }
  
  // Calculate checksum (excluding header: magic, version, checksum)
  size_t header_size = sizeof(uint32_t) + sizeof(uint16_t) * 2;
  data.checksum = calculateChecksum((uint8_t *)&data + header_size, 
                                     sizeof(EncoderFlashData) - header_size);
  
  // Prepare for flash write
  uint8_t *data_ptr = (uint8_t *)&data;
  size_t data_size = sizeof(EncoderFlashData);
  size_t num_pages = (data_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
  
  // Signal Core1 to enter RAM wait loop before flash operations
  flash_operation_in_progress = true;
  delay(5);  // Give Core1 time to enter the wait loop
  
  // Disable interrupts for flash operations
  uint32_t ints = save_and_disable_interrupts();
  
  // Erase flash sector
  flash_range_erase(ENCODER_FLASH_TARGET_OFFSET, num_pages * FLASH_SECTOR_SIZE);
  
  // Create aligned buffer and write
  uint8_t flash_page[FLASH_PAGE_SIZE];
  memset(flash_page, 0xFF, FLASH_PAGE_SIZE);
  memcpy(flash_page, data_ptr, data_size);
  
  // Program the flash
  flash_range_program(ENCODER_FLASH_TARGET_OFFSET, flash_page, FLASH_PAGE_SIZE);
  
  restore_interrupts(ints);
  
  // Signal Core1 to resume normal operation
  flash_operation_in_progress = false;
  
  _flashDataValid = true;
  LOG_INFO("Encoder offsets saved to flash successfully!");
  LOG_INFO_F("  Offsets: %.4f, %.4f, %.4f rad", 
             data.offsets[0], data.offsets[1], data.offsets[2]);
  
  return true;
}

bool DirectEncoders::loadOffsetsFromFlash() {
  EncoderFlashData data;
  
  // Read from flash
  const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + ENCODER_FLASH_TARGET_OFFSET);
  memcpy(&data, flash_ptr, sizeof(EncoderFlashData));
  
  // Verify magic number
  if (data.magic_number != ENCODER_FLASH_MAGIC_NUMBER) {
    LOG_WARN("Flash: No valid encoder data found (magic number mismatch)");
    _flashDataValid = false;
    return false;
  }
  
  // Verify version
  if (data.version != ENCODER_FLASH_STRUCT_VERSION) {
    LOG_ERROR("Flash: Incompatible encoder data version!");
    _flashDataValid = false;
    return false;
  }
  
  // Verify checksum
  size_t header_size = sizeof(uint32_t) + sizeof(uint16_t) * 2;
  uint16_t calculated = calculateChecksum((uint8_t *)&data + header_size, 
                                           sizeof(EncoderFlashData) - header_size);
  if (calculated != data.checksum) {
    LOG_ERROR("Flash: Encoder data corrupted (checksum mismatch)!");
    _flashDataValid = false;
    return false;
  }
  
  // Load offsets
  for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
    _offsets[i] = data.offsets[i];
  }
  
  _flashDataValid = true;
  LOG_INFO("Encoder offsets loaded from flash successfully!");
  LOG_INFO_F("  Offsets: %.4f, %.4f, %.4f rad", 
             data.offsets[0], data.offsets[1], data.offsets[2]);
  
  return true;
}

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

// Global instance - will be configured in main.cpp
DirectEncoders directEncoders;
