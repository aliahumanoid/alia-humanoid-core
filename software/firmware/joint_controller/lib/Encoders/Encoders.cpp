/**
 * @file Encoders.cpp
 * @brief Implementation of multi-encoder SPI communication
 * 
 * See Encoders.h for detailed usage documentation.
 */

#include "Encoders.h"
#include <debug.h>
#include <SPI.h>

// ===================================================================
// SPI CONFIGURATION
// ===================================================================

// SPI communication settings (2 MHz, MSB first, mode 0)
SPISettings spi_settings(2000000, MSBFIRST, SPI_MODE0);

// ===================================================================
// SYNCHRONIZATION PROTOCOL
// ===================================================================

/**
 * Synchronization sequence for frame alignment
 * This pattern marks the start of valid encoder data in the SPI stream
 */
const uint8_t SYNC_SEQUENCE[]     = {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55};
const size_t SYNC_SEQUENCE_LENGTH = sizeof(SYNC_SEQUENCE);

// ===================================================================
// CONSTRUCTOR
// ===================================================================

Encoders::Encoders(int rx_pin, int cs_pin, int sck_pin, int tx_pin, bool *encoder_invert)
    : _rx_pin(rx_pin), _cs_pin(cs_pin), _sck_pin(sck_pin), _tx_pin(tx_pin) {

  // Initialize encoder counts to zero
  memset(_encoder_counts, 0, sizeof(_encoder_counts));
  _last_read_time = 0;
  _dataValid      = false;

  // Initialize per-encoder configuration
  for (int i = 0; i < ENCODER_COUNT; ++i) {
    _joint_offsets[i] = 0.0f;
    // Copy inversion flags if provided, otherwise default to false
    _encoder_invert[i] = (encoder_invert != nullptr) ? encoder_invert[i] : false;
  }
}

// ===================================================================
// INITIALIZATION
// ===================================================================

void Encoders::begin() {
  // Configure SPI pins
  SPI.setRX(_rx_pin);
  SPI.setCS(_cs_pin);
  SPI.setSCK(_sck_pin);
  SPI.setTX(_tx_pin);
  SPI.begin(true);

  // Perform initial reads to stabilize communication
  sleep_ms(100);
  update(); // First read to wake up encoders
  sleep_ms(100);
  update(); // Second read to ensure sync
  sleep_ms(100);
  update(); // Third read to validate data

  // Note: angle offsets (_joint_offsets) must be set after this initialization.
  // This is typically done in JointController::init() for each DOF that has
  // a zero_angle_offset configured.
}

// ===================================================================
// PRIVATE METHODS - SYNCHRONIZATION
// ===================================================================

bool Encoders::findSyncSequence(uint8_t *buffer, size_t bufferSize, size_t &syncIndex) {
  // Search for sync pattern in circular buffer
  for (size_t i = 0; i < bufferSize; ++i) {
    bool found = true;
    // Check if sync sequence matches at position i
    for (size_t j = 0; j < SYNC_SEQUENCE_LENGTH; ++j) {
      if (buffer[(i + j) % bufferSize] != SYNC_SEQUENCE[j]) {
        found = false;
        break;
      }
    }
    if (found) {
      syncIndex = i;
      return true;
    }
  }
  return false; // Sync sequence not found
}

// ===================================================================
// DATA ACQUISITION
// ===================================================================

void Encoders::update() {
  // Calculate buffer size: sync sequence + status byte + 4 bytes per encoder
  const size_t bufferSize = SYNC_SEQUENCE_LENGTH + 1 + 4 * ENCODER_COUNT;
  uint8_t tx_buffer[bufferSize];
  uint8_t recv_buffer[bufferSize];
  size_t syncIndex = 0;
  _dataValid       = true; // Assume valid, will be set to false on error

  // Prepare command buffer (zeros = read request)
  memset(tx_buffer, 0, bufferSize);

  // ---------------------------------------------------------------
  // Step 1: Perform SPI transaction
  // ---------------------------------------------------------------
  SPI.beginTransaction(spi_settings);
  delayMicroseconds(10); // Stabilize SPI lines before transfer
  SPI.transfer(tx_buffer, recv_buffer, bufferSize);
  delayMicroseconds(10); // Stabilize after transfer
  SPI.endTransaction();

  // Optional debug: print received buffer
  // Serial.print("Received buffer: ");
  // for (int i = 0; i < bufferSize; ++i) {
  //   Serial.print(recv_buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // ---------------------------------------------------------------
  // Step 2: Find synchronization sequence
  // ---------------------------------------------------------------
  if (!findSyncSequence(recv_buffer, bufferSize, syncIndex)) {
    LOG_ERROR("Synchronization sequence not found");
    _dataValid = false;
    return;
  }

  // Optional debug: print sync index
  // Serial.print("Sync found at index: ");
  // Serial.println(syncIndex);

  // ---------------------------------------------------------------
  // Step 3: Re-align buffer if sync is not at start
  // ---------------------------------------------------------------
  if (syncIndex != 0) {
    sleep_us(100);
    
    // Read extra bytes to re-align the stream
    uint8_t extra_buffer[syncIndex];
    SPI.beginTransaction(spi_settings);
    SPI.transfer(extra_buffer, syncIndex);
    SPI.endTransaction();

    sleep_us(10); // Small delay before re-reading
    
    // Read full buffer again after alignment
    SPI.beginTransaction(spi_settings);
    SPI.transfer(recv_buffer, bufferSize);
    SPI.endTransaction();

    // Verify sync sequence in new buffer
    if (!findSyncSequence(recv_buffer, bufferSize, syncIndex)) {
      LOG_ERROR("Synchronization sequence not found after re-alignment");
      _dataValid = false;
      return;
    }
  }

  // ---------------------------------------------------------------
  // Step 4: Read and validate status byte
  // ---------------------------------------------------------------
  size_t statusIndex = (syncIndex + SYNC_SEQUENCE_LENGTH) % bufferSize;
  uint8_t status     = recv_buffer[statusIndex];

  // Optional debug: print full buffer
  // Serial.print("Received buffer: ");
  // for (int i = 0; i < bufferSize; i++) {
  //   Serial.print(recv_buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Check if encoder boards loaded data successfully
  _dataValid = (status == STATUS_OK);

  if (!_dataValid) {
    LOG_ERROR("Invalid encoder data - load failed on remote device");
    return;
  }

  // ---------------------------------------------------------------
  // Step 5: Extract encoder counts from buffer
  // ---------------------------------------------------------------
  size_t dataStart = (statusIndex + 1) % bufferSize;

  // Read 4 bytes per encoder (32-bit count in big-endian format)
  for (int i = 0; i < ENCODER_COUNT; ++i) {
    _encoder_counts[i] = (recv_buffer[(dataStart + i * 4) % bufferSize] << 24) |
                         (recv_buffer[(dataStart + i * 4 + 1) % bufferSize] << 16) |
                         (recv_buffer[(dataStart + i * 4 + 2) % bufferSize] << 8) |
                         recv_buffer[(dataStart + i * 4 + 3) % bufferSize];
  }

  // Store timestamp of successful read
  _last_read_time = micros();
}

int32_t Encoders::getCount(uint8_t encoder_index) {
  // Auto-update if data is stale (>100 µs old)
  if (micros() - _last_read_time > 100) {
    update();
  }

  // Return 0 if data is invalid or index out of range
  if (!_dataValid || encoder_index >= ENCODER_COUNT) {
    return 0;
  }

  return _encoder_counts[encoder_index];
}

float Encoders::getJointAngle(uint8_t encoder_index, bool &isValid) {
  // Get raw encoder count
  long encoder_value = getCount(encoder_index);

  // Report data validity to caller
  isValid = _dataValid;

  if (!_dataValid) {
    return 0.0f;
  }

  // Convert from milli-degrees to degrees
  float angle = static_cast<float>(encoder_value) / 1000.0f;

  // Apply inversion if configured for this encoder
  if (encoder_index < ENCODER_COUNT && _encoder_invert[encoder_index]) {
    angle = -angle;
  }

  // Apply angular offset (for zero position calibration)
  if (encoder_index < ENCODER_COUNT) {
    float offset = _joint_offsets[encoder_index];
    if (offset != 0.0f) {
      // Optional debug output
      // Serial.printf("Applying offset %.2f to encoder %d (raw angle: %.2f)\n",
      //               offset, encoder_index, angle);
      angle += offset;
    }
  }

  return angle;
}

void Encoders::printCounts() {
  DBG_PRINT("Encoder counts: ");
  for (int i = 0; i < ENCODER_COUNT; ++i) {
    DBG_PRINT(_encoder_counts[i]);
    if (i < 3)
      DBG_PRINT(", ");
  }
  DBG_PRINTLN();
}

// ===================================================================
// CALIBRATION & CONFIGURATION
// ===================================================================

void Encoders::zeroEncoders(int8_t encoder_index) {
  const size_t bufferSize = SYNC_SEQUENCE_LENGTH + 1 + 4 * ENCODER_COUNT;
  uint8_t tx_buffer[bufferSize];
  uint8_t cmd = CMD_ZERO_ALL; // Default: zero all encoders

  // Select appropriate command based on encoder_index
  if (encoder_index >= 0 && encoder_index < ENCODER_COUNT) {
    switch (encoder_index) {
    case 0:
      cmd = CMD_ZERO_DOF0;
      break;
    case 1:
      cmd = CMD_ZERO_DOF1;
      break;
    case 2:
      cmd = CMD_ZERO_DOF2;
      break;
    default:
      // For indices > 2, keep CMD_ZERO_ALL
      // No specific commands supported for these encoders
      DBG_PRINTLN("Warning: selective zeroing supported only for channels 0-2");
      break;
    }
    DBG_PRINT("Selective zeroing of encoder ");
    DBG_PRINT(encoder_index);

  } else {
    DBG_PRINTLN("Zeroing all encoders");
  }

  // Fill buffer with command byte
  memset(tx_buffer, cmd, bufferSize);

  // Send zero command via SPI
  SPI.beginTransaction(spi_settings);
  SPI.transfer(tx_buffer, nullptr, bufferSize);
  SPI.endTransaction();

  // Clear in-memory count for the specific encoder
  if (encoder_index >= 0 && encoder_index < ENCODER_COUNT) {
    _encoder_counts[encoder_index] = 0;
  }
  
  // Wait for slave flash write to complete (encoder boards save zero position)
  sleep_ms(200);
  
  // Force a read to refresh encoder data
  update();

  // Note: this method only zeros physical counts on the encoder slave boards.
  // Angle offsets stored in _joint_offsets are not changed and will continue
  // to be applied in getJointAngle().
}

void Encoders::setEncoderInvert(uint8_t encoder_index, bool invert) {
  if (encoder_index < ENCODER_COUNT) {
    _encoder_invert[encoder_index] = invert;
  }
}

void Encoders::setJointOffset(uint8_t encoder_index, float offset) {
  if (encoder_index < ENCODER_COUNT) {
    _joint_offsets[encoder_index] = offset;
  }
}

float Encoders::getJointOffset(uint8_t encoder_index) const {
  if (encoder_index < ENCODER_COUNT) {
    return _joint_offsets[encoder_index];
  }
  return 0.0f;
}
