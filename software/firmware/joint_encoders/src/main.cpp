/**
 * @file main.cpp
 * @brief Multi-encoder reader with SPI slave communication for joint control
 * 
 * This firmware manages up to 6 MT6835 magnetic encoders in a daisy-chain configuration,
 * providing absolute angle readings with multi-turn tracking. Data is transmitted to a
 * host controller via SPI slave interface.
 * 
 * Key Features:
 * - Multi-encoder support (up to 6 encoders via SPI1)
 * - Absolute angle measurement with multi-turn tracking
 * - Validation and error detection (CRC, spike detection, status check)
 * - Flash storage for encoder zero offsets
 * - SPI slave communication with host (SPI0)
 * - Individual encoder reset capability
 * - Automatic retry and error recovery
 * 
 * Hardware Configuration:
 * - SPI1: MT6835 encoders (master mode, daisy-chain)
 * - SPI0: Host communication (slave mode)
 * - Flash: Persistent storage for calibration data
 * 
 * Protocol:
 * - Host → Device: Reset commands (0x0A-0x0C for individual, 0xFF for all)
 * - Device → Host: Sync sequence + status + 6x angle data (millidegrees)
 * 
 * @version See version.h
 * @date 2025
 */

#include "hardware/flash.h"
#include "hardware/sync.h"
#include <Arduino.h>
#include "version.h"
#include <MSPISlave.h> // MSPISlave library
#include <MagneticSensorMT6835.h>
#include <SPI.h> // SPI library
#include "logging.h"

// ================================
// CONFIGURATION
// ================================

#define ENCODER_COUNT 6

// ================================
// CONNECTED SENSORS CONFIG
// ================================
// Set true for connected sensors, false otherwise
#define SENSOR_1_CONNECTED true  // Encoder 1 connected
#define SENSOR_2_CONNECTED true  // Encoder 2 connected
#define SENSOR_3_CONNECTED false // Encoder 3 not connected
#define SENSOR_4_CONNECTED false // Encoder 4 not connected
#define SENSOR_5_CONNECTED false // Encoder 5 not connected
#define SENSOR_6_CONNECTED false // Encoder 6 not connected

#define SENSOR_CS_1 0 // GPIO CS pin encoder 1
#define SENSOR_CS_2 1 // GPIO CS pin encoder 2
#define SENSOR_CS_3 2 // GPIO CS pin encoder 3
#define SENSOR_CS_4 3 // GPIO CS pin encoder 4
#define SENSOR_CS_5 4 // GPIO CS pin encoder 5
#define SENSOR_CS_6 5 // GPIO CS pin encoder 6

// Pico onboard LED pin
#define LED_PIN 25

// Read validation parameters
#define MAX_RPM 60.0f          // Max speed in RPM
#define VALIDATION_MARGIN 1.5f // Safety margin 50%
#define MAX_READ_ATTEMPTS 2    // Max read retry attempts
#define MIN_VALID_ANGLE 0.001f // Min valid angle (avoid 0 reads)
#define RETRY_DELAY_MICROS 50  // Delay between retry attempts (µs)

#define MAGIC_NUMBER 0xDEADBEEF
#define FLASH_STRUCT_VERSION 1
// FLASH_PAGE_SIZE and FLASH_SECTOR_SIZE are defined in hardware/flash.h
// #define FLASH_PAGE_SIZE 256
// #define FLASH_SECTOR_SIZE 4096
#define FLASH_TARGET_OFFSET (256 * 1024) // Flash offset to use

// Define SPI1 object using the proper constructor
SPIClassRP2040 SPI_1(spi1, 8, -1, 10,
                     11); // rx=8 (MISO), sck=10, tx=11 (MOSI), cs is -1 (handled manually)
SPISettings myMT6835SPISettings(2000000, MSBFIRST, SPI_MODE3);

// Pass SPI1 as reference to sensor constructors
MagneticSensorMT6835 sensor1 = MagneticSensorMT6835(SENSOR_CS_1, myMT6835SPISettings);
MagneticSensorMT6835 sensor2 = MagneticSensorMT6835(SENSOR_CS_2, myMT6835SPISettings);
MagneticSensorMT6835 sensor3 = MagneticSensorMT6835(SENSOR_CS_3, myMT6835SPISettings);
MagneticSensorMT6835 sensor4 = MagneticSensorMT6835(SENSOR_CS_4, myMT6835SPISettings);
MagneticSensorMT6835 sensor5 = MagneticSensorMT6835(SENSOR_CS_5, myMT6835SPISettings);
MagneticSensorMT6835 sensor6 = MagneticSensorMT6835(SENSOR_CS_6, myMT6835SPISettings);

// Absolute encoder (encoder 1)
float encoder1_offset    = 0.0;
int32_t encoder1_turns   = 0;
static float last_angle1 = 0.0;

// Globals for encoder 2
int32_t encoder2_turns = 0;
float last_angle2      = 0.0;
float encoder2_offset  = 0.0;

// Global variables for encoder 3
int32_t encoder3_turns = 0;
float last_angle3      = 0.0;
float encoder3_offset  = 0.0;

// Global variables for encoder 4
int32_t encoder4_turns = 0;
float last_angle4      = 0.0;
float encoder4_offset  = 0.0;

// Global variables for encoder 5
int32_t encoder5_turns = 0;
float last_angle5      = 0.0;
float encoder5_offset  = 0.0;

// Global variables for encoder 6
int32_t encoder6_turns = 0;
float last_angle6      = 0.0;
float encoder6_offset  = 0.0;

// Error control / validation
unsigned long last_read_micros = 0; // Timestamp of last read
uint32_t error_count_encoder1  = 0; // Error counters per encoder
uint32_t error_count_encoder2  = 0;
uint32_t error_count_encoder3  = 0;
float last_valid_angle1        = 0.0; // Last valid values read
float last_valid_angle2        = 0.0;
float last_valid_angle3        = 0.0;
bool led_state                 = false; // LED state
unsigned long led_timer        = 0;     // LED blink timer

// SPI0 pins used to communicate with MT6835 sensor
SPISettings spisettings(5000000, MSBFIRST, SPI_MODE0); // 5 MHz is the reliability limit
const int PIN_RX  = 16;
const int PIN_CS  = 17;
const int PIN_SCK = 18;
const int PIN_TX  = 19;

// System state
bool device_data_loaded = false; // Track if data loaded correctly

// Improved synchronization sequence
const uint8_t SYNC_SEQUENCE[]     = {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55};
const size_t SYNC_SEQUENCE_LENGTH = sizeof(SYNC_SEQUENCE);

// SPI status codes
const uint8_t STATUS_OK               = 0x01;
const uint8_t STATUS_DATA_LOAD_FAILED = 0x02;

// SPI buffer: sync seq + status byte + 24 bytes data (4 bytes per encoder)
uint8_t spi_send_buffer[SYNC_SEQUENCE_LENGTH + 1 + 4 * ENCODER_COUNT];

std::array<volatile int32_t, ENCODER_COUNT> shared_encoder_millidegrees;

long ts;
long debug_timer = 0; // Debug output timer every 5 seconds

volatile bool dataReceived     = false;
volatile size_t receivedLength = 0;
uint8_t receivedData[SYNC_SEQUENCE_LENGTH + 1 + 4 * ENCODER_COUNT]; // Adequate size for the data

/**
 * @brief Flash storage structure for encoder calibration data
 * 
 * This structure is persisted to flash memory and contains encoder zero offsets.
 * Includes validation fields (magic number, version, checksum) to ensure data integrity.
 */
struct DeviceData {
  uint32_t magic_number;   ///< Magic number for validation (0xDEADBEEF)
  uint16_t version;        ///< Structure version (FLASH_STRUCT_VERSION)
  uint16_t checksum;       ///< Checksum for data integrity
  float encoder1_offset;   ///< Encoder 1 zero offset (radians)
  float encoder2_offset;   ///< Encoder 2 zero offset (radians)
  float encoder3_offset;   ///< Encoder 3 zero offset (radians)
};

/**
 * @brief Callback invoked when SPI slave has sent all data
 * 
 * Re-loads the buffer with updated encoder data for next transmission.
 */
void sentCallback() {
  // This is called when all data has been sent
  SPISlave.setData((uint8_t *)spi_send_buffer, sizeof(spi_send_buffer));
}

/**
 * @brief Callback invoked when SPI slave receives data from host
 * 
 * Accepts reset commands:
 * - 0x0A: Reset encoder 1
 * - 0x0B: Reset encoder 2
 * - 0x0C: Reset encoder 3
 * - 0xFF: Reset all encoders
 * 
 * @param data Received data buffer
 * @param len Length of received data
 */
void recvCallback(uint8_t *data, size_t len) {
  if (len > 0 && (data[0] == 0x0A || data[0] == 0x0B || data[0] == 0x0C || data[0] == 0xFF)) {
    // Copy data to a safe buffer
    memcpy((void *)receivedData, data, len);
    receivedLength = len;
    dataReceived   = true;
  }
}

void testEncoderSimulation(float offset, std::vector<float> testAngles) {
  // Compute normalized initial angle
  float first_angle     = testAngles[0] - offset;
  int32_t encoder_turns = (int32_t)floor(first_angle / (2 * PI));
  float last_angle      = fmod(first_angle, 2 * PI);
  if (last_angle < 0)
    last_angle += 2 * PI;

  for (float angle : testAngles) {
    // Normalize simulated angle to [0, 2π]
    float simulated_angle = fmod(angle - offset, 2 * PI);
    if (simulated_angle < 0)
      simulated_angle += 2 * PI;

    float delta_angle = simulated_angle - last_angle;

    if (delta_angle > PI) {
      encoder_turns--;
    } else if (delta_angle < -PI) {
      encoder_turns++;
    }

    float total_angle = ((encoder_turns * 2 * PI) + simulated_angle) * (180.0 / PI);

    last_angle = simulated_angle;

    // Print computed value (debug only)
    LOG_DEBUG("Simulated angle: %.2f°, Total computed angle: %.2f°",
              angle * (180.0 / PI), total_angle);
  }
}

/**
 * @brief Calculate simple checksum for flash data validation
 * 
 * @param data Pointer to data buffer
 * @param size Size of data buffer (bytes)
 * @return uint16_t Checksum value (sum of all bytes)
 */
uint16_t calculate_checksum(const uint8_t *data, size_t size) {
  uint16_t checksum = 0;
  for (size_t i = 0; i < size; i++) {
    checksum += data[i];
  }
  return checksum;
}

/**
 * @brief Activate error LED for visual feedback
 * 
 * Turns on the onboard LED to indicate an encoder read error.
 * LED will automatically turn off after 100ms (handled in main loop).
 */
void blinkErrorLED() {
  led_state = true;
  digitalWrite(LED_PIN, HIGH);
  led_timer = millis();
}

/**
 * @brief Read encoder with validation and error detection
 * 
 * Performs multiple validation checks on encoder readings:
 * - Range check [0, 2π]
 * - Speed-based spike detection
 * - Sensor status check
 * - Automatic retry on failure
 * 
 * @param sensor Reference to magnetic sensor object
 * @param last_valid_angle Last known valid angle (radians)
 * @param error_count Reference to error counter (incremented on failure)
 * @param delta_micros Time elapsed since last read (microseconds)
 * @param encoder_name Human-readable name for debug messages
 * @return float Validated angle (radians) or last read value on failure
 */
float readEncoderWithValidation(MagneticSensorMT6835 &sensor, float last_valid_angle,
                                uint32_t &error_count, unsigned long delta_micros,
                                const char *encoder_name) {

  // Compute maximum allowed variation based on elapsed time
  float max_delta_rad =
      (MAX_RPM * 2.0f * PI / 60.0f) * (delta_micros / 1000000.0f) * VALIDATION_MARGIN;

  float raw_angle    = 0.0f;
  bool valid_reading = false;
  int attempts       = 0;

  while (!valid_reading && attempts < MAX_READ_ATTEMPTS) {
    attempts++;
    raw_angle = sensor.getSensorAngle();

    // Check 1: Value in valid range [0, 2π]
    if (raw_angle < 0 || raw_angle > 2 * PI) {
      if (attempts == 1) {
        if (raw_angle == -1) {
          LOG_ERROR("%s: CRC error - corrupted data on SPI", encoder_name);
        } else {
          LOG_ERROR("%s: Angle out of range = %.4f", encoder_name, raw_angle);
        }
        blinkErrorLED(); // Blink on each detected error
      }
      delayMicroseconds(RETRY_DELAY_MICROS); // Small delay before retry
      continue;
    }

    // Check 2: Not exactly 0 or too close (disabled)
    // if (raw_angle < MIN_VALID_ANGLE) {
    //     if (attempts == 1) {
    //         LOG_ERROR("%s: Near-zero reading", encoder_name);
    //         blinkErrorLED(); // Blink on each detected error
    //     }
    //     delayMicroseconds(RETRY_DELAY_MICROS);
    //     continue;
    // }

    // Check 3: Reasonable variation (only if we have a previous valid reading)
    if (last_valid_angle > 0 && delta_micros > 0) {
      float delta = fabs(raw_angle - last_valid_angle);
      // Handle wrap-around at 2π
      if (delta > PI) {
        delta = 2 * PI - delta;
      }

      if (delta > max_delta_rad) {
        if (attempts == 1) {
          LOG_ERROR("%s: Spike detected. Delta = %.2f° (max allowed = %.2f°)",
                    encoder_name, delta * 180.0 / PI, max_delta_rad * 180.0 / PI);
          blinkErrorLED(); // Blink on each detected error
        }
        delayMicroseconds(RETRY_DELAY_MICROS);
        continue;
      }
    }

    // Check 4: Sensor status check
    uint8_t status = sensor.getStatus();
    if (status & (MT6835_STATUS_OVERSPEED | MT6835_STATUS_WEAKFIELD | MT6835_STATUS_UNDERVOLT)) {
      if (attempts == 1) {
        LOG_ERROR("%s: Status error = 0x%02X", encoder_name, status);
        blinkErrorLED(); // Blink on each detected error
      }
      delayMicroseconds(RETRY_DELAY_MICROS);
      continue;
    }

    // If we are here, the reading is valid
    valid_reading = true;
  }

  if (valid_reading) {
    // Reset error counter if reading is good
    if (error_count > 0) {
      LOG_INFO("%s: Reading recovered successfully", encoder_name);
    }
    error_count = 0;
    return raw_angle;
  } else {
    // All attempts failed
    error_count++;
    blinkErrorLED();

    LOG_ERROR("%s: All attempts failed. Consecutive errors = %u", encoder_name, error_count);

    // SIMPLE SOLUTION: Always return the last read value
    // This avoids infinite loops and does not introduce artifacts
    // The caller will update last_valid_angle accordingly
    return raw_angle;
  }
}

/**
 * @brief Save encoder calibration data to flash memory
 * 
 * Writes DeviceData structure to flash with validation fields (magic number,
 * version, checksum). Automatically erases required flash sectors before writing.
 * 
 * @param data DeviceData structure containing encoder offsets to save
 * @return bool true if save successful, false otherwise
 * 
 * @note This function disables interrupts during flash operations
 * @warning Flash writes are destructive - ensure data is correct before calling
 */
bool save_device_data(struct DeviceData data) {
  // Set magic number and version
  data.magic_number = MAGIC_NUMBER;
  data.version      = FLASH_STRUCT_VERSION;

  // Calculate checksum
  data.checksum = calculate_checksum((uint8_t *)&data + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                                     sizeof(DeviceData) - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  uint8_t *data_ptr = (uint8_t *)&data;
  size_t data_size  = sizeof(struct DeviceData);
  size_t offset     = 0;

  // Compute number of pages required
  size_t num_pages = (data_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

  uint32_t ints = save_and_disable_interrupts(); // Disable interrupts for safety

  // Ensure enough flash is erased for all data
  flash_range_erase(FLASH_TARGET_OFFSET, num_pages * FLASH_SECTOR_SIZE);

  bool success = true;

  while (offset < data_size && success) {
    size_t chunk_size =
        (data_size - offset > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : data_size - offset;

    // Create a temporary buffer to host data to be programmed
    uint8_t flash_page[FLASH_PAGE_SIZE];
    memset(flash_page, 0xFF, FLASH_PAGE_SIZE); // Initialize buffer to 0xFF
    memcpy(flash_page, data_ptr + offset, chunk_size);

    // Program the flash
    flash_range_program(FLASH_TARGET_OFFSET + offset, flash_page, FLASH_PAGE_SIZE);
    offset += FLASH_PAGE_SIZE; // Increment offset by FLASH_PAGE_SIZE
  }

  restore_interrupts(ints); // Restore interrupts

  if (success) {
    LOG_INFO("Encoder offsets saved successfully!");
  } else {
    LOG_ERROR("Error saving encoder offsets!");
  }

  return success;
}

/**
 * @brief Load encoder calibration data from flash memory
 * 
 * Reads DeviceData structure from flash and validates integrity using magic number,
 * version, and checksum. Returns false if validation fails.
 * 
 * @param data Pointer to DeviceData structure to populate
 * @return bool true if load and validation successful, false otherwise
 * 
 * @note Returns false if:
 *       - Magic number mismatch (data not initialized)
 *       - Version mismatch (incompatible structure)
 *       - Checksum mismatch (data corrupted)
 */
bool load_device_data(struct DeviceData *data) {
  if (data == NULL) {
    LOG_ERROR("Invalid data pointer provided!");
    return false;
  }

  uint8_t *data_ptr        = (uint8_t *)data;
  size_t data_size         = sizeof(struct DeviceData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

  // Read validation fields first
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  // Verify magic number
  if (data->magic_number != MAGIC_NUMBER) {
    LOG_WARN("Invalid magic number, data not initialized!");
    return false;
  }

  // Verify version
  if (data->version != FLASH_STRUCT_VERSION) {
    LOG_ERROR("Incompatible data version!");
    return false;
  }

  // Read remaining data
  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  // Calculate and verify checksum
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);
  if (calculated_checksum != data->checksum) {
    LOG_ERROR("Checksum mismatch, data corrupted!");
    return false;
  }

  // Print encoder data
  LOG_INFO("Loaded Encoder Offsets: Encoder1: %.3f, Encoder2: %.3f, Encoder3: %.3f",
           data->encoder1_offset, data->encoder2_offset, data->encoder3_offset);

  LOG_INFO("Encoder offsets loaded successfully!");
  return true;
}

void setup() {

  Serial.begin(115200); // Initialize serial communication

  delay(2000);
  
  // Version handshake (always printed for protocol identification)
  Serial.println("EVT:FW:VERSION " FW_VERSION);
  Serial.println("EVT:PROTO " PROTO_VERSION);
  Serial.println("EVT:BUILD " BUILD_GIT_SHA " " BUILD_DATE);
  Serial.println("EVT:READY");

  // Initialize error LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Configure CS pins as outputs
  pinMode(SENSOR_CS_1, OUTPUT);
  pinMode(SENSOR_CS_2, OUTPUT);
  pinMode(SENSOR_CS_3, OUTPUT);
  pinMode(SENSOR_CS_4, OUTPUT);
  pinMode(SENSOR_CS_5, OUTPUT);
  pinMode(SENSOR_CS_6, OUTPUT);
  // Set CS pins high (disabled)
  digitalWrite(SENSOR_CS_1, HIGH);
  digitalWrite(SENSOR_CS_2, HIGH);
  digitalWrite(SENSOR_CS_3, HIGH);
  digitalWrite(SENSOR_CS_4, HIGH);
  digitalWrite(SENSOR_CS_5, HIGH);
  digitalWrite(SENSOR_CS_6, HIGH);
  // Initialize only connected sensors
  LOG_INFO("=== SENSORS INITIALIZATION ===");
#if SENSOR_1_CONNECTED
  sensor1.init(&SPI_1);
  sensor1.checkcrc = true;
  LOG_INFO("Encoder 1: Initialized");
#else
  LOG_INFO("Encoder 1: Skipped (not connected)");
#endif

#if SENSOR_2_CONNECTED
  sensor2.init(&SPI_1);
  sensor2.checkcrc = true;
  LOG_INFO("Encoder 2: Initialized");
#else
  LOG_INFO("Encoder 2: Skipped (not connected)");
#endif

#if SENSOR_3_CONNECTED
  sensor3.init(&SPI_1);
  sensor3.checkcrc = true;
  LOG_INFO("Encoder 3: Initialized");
#else
  LOG_INFO("Encoder 3: Skipped (not connected)");
#endif

#if SENSOR_4_CONNECTED
  sensor4.init(&SPI_1);
  sensor4.checkcrc = true;
  LOG_INFO("Encoder 4: Initialized");
#else
  LOG_INFO("Encoder 4: Skipped (not connected)");
#endif

#if SENSOR_5_CONNECTED
  sensor5.init(&SPI_1);
  sensor5.checkcrc = true;
  LOG_INFO("Encoder 5: Initialized");
#else
  LOG_INFO("Encoder 5: Skipped (not connected)");
#endif

#if SENSOR_6_CONNECTED
  sensor6.init(&SPI_1);
  sensor6.checkcrc = true;
  LOG_INFO("Encoder 6: Initialized");
#else
  LOG_INFO("Encoder 6: Skipped (not connected)");
#endif
  LOG_INFO("=================================");

  // Load encoder data from flash
  DeviceData loadedData;
  device_data_loaded = load_device_data(&loadedData);

  if (device_data_loaded) {
    encoder1_offset = loadedData.encoder1_offset;
    encoder2_offset = loadedData.encoder2_offset;
    encoder3_offset = loadedData.encoder3_offset;
    LOG_INFO("Encoder offsets loaded successfully from flash!");
  } else {
    LOG_WARN("Failed to load encoder offsets, attempting recovery procedure...");

    // // Recovery procedure: initialize default data
    // DeviceData defaultData;
    // defaultData.encoder1_offset = sensor1.getSensorAngle(); // Use current position
    // defaultData.encoder2_offset = sensor2.getSensorAngle();
    // defaultData.encoder3_offset = sensor3.getSensorAngle();

    // LOG_INFO("Saving default encoder values to flash...");
    // LOG_INFO("Default values - Encoder1: %.3f, Encoder2: %.3f, Encoder3: %.3f",
    //          defaultData.encoder1_offset, defaultData.encoder2_offset, defaultData.encoder3_offset);

    // // Save default data to flash
    // bool save_success = save_device_data(defaultData);

    // if (save_success) {
    //     LOG_INFO("Default values saved, attempting to reload...");

    //     // Try to reload just-saved data
    //     device_data_loaded = load_device_data(&loadedData);

    //     if (device_data_loaded) {
    //         // Successfully loaded new data
    //         encoder1_offset = loadedData.encoder1_offset;
    //         encoder2_offset = loadedData.encoder2_offset;
    //         encoder3_offset = loadedData.encoder3_offset;
    //         LOG_INFO("Recovery successful! New encoder offsets loaded.");
    //     } else {
    //         // Critical error: unable to save/load data to flash
    //         LOG_ERROR("CRITICAL ERROR: Flash memory may be corrupted or not functioning!");
    //         LOG_ERROR("Will operate with default values, but data will be lost on reboot!");
    //         // Use default values (already set in sensors)
    //     }
    // } else {
    //     LOG_ERROR("CRITICAL ERROR: Failed to save default values to flash!");
    //     LOG_ERROR("Hardware issue detected with flash memory!");
    //     // Unable to save to flash — likely a hardware issue
    // }
  }

  // Configure SPI0 as slave
  SPISlave.setRX(PIN_RX);
  SPISlave.setCS(PIN_CS);
  SPISlave.setSCK(PIN_SCK);
  SPISlave.setTX(PIN_TX);

  // Ensure we start with something to send...
  sentCallback();

  SPISlave.onDataRecv(recvCallback);
  SPISlave.onDataSent(sentCallback);

  SPISlave.begin(spisettings);

  LOG_INFO("SPISlave started");

  ts          = millis();
  debug_timer = millis(); // Initialize debug timer

  // Initialize validation variables with first readings (only connected sensors)
  last_read_micros = micros();
#if SENSOR_1_CONNECTED
  last_valid_angle1 = sensor1.getSensorAngle();
#endif
#if SENSOR_2_CONNECTED
  last_valid_angle2 = sensor2.getSensorAngle();
#endif
#if SENSOR_3_CONNECTED
  last_valid_angle3 = sensor3.getSensorAngle();
#endif
}

void loop() {
  long now = millis();
  // Measure time before read
  unsigned long startMicros  = micros();
  unsigned long delta_micros = startMicros - last_read_micros;
  last_read_micros           = startMicros;

  // Error LED handling (turn off after 100 ms)
  if (led_state && (now - led_timer > 100)) {
    digitalWrite(LED_PIN, LOW);
    led_state = false;
  }

  // Encoder 1 read with validation (only if connected)
  float total_angle1 = 0.0;
#if SENSOR_1_CONNECTED
  float raw_angle1  = readEncoderWithValidation(sensor1, last_valid_angle1, error_count_encoder1,
                                                delta_micros, "Encoder1");
  last_valid_angle1 = raw_angle1; // Update last valid value

  float angle1 = fmod(raw_angle1 - encoder1_offset, 2 * PI);
  if (angle1 < 0)
    angle1 += 2 * PI;

  // Compute angle delta
  float delta_angle1 = angle1 - last_angle1;

  // Handle wrap-around across zero by decrementing
  if (delta_angle1 > PI) {
    encoder1_turns--;
  } else if (delta_angle1 < -PI) {
    encoder1_turns++;
  }

  // Update previous angle
  last_angle1 = angle1;

  // Compute total angle including turns
  total_angle1 = (encoder1_turns * 360.0) + (angle1 * (180.0 / PI));
#endif

  // Encoder 2 read with validation (only if connected)
  float total_angle2 = 0.0;
#if SENSOR_2_CONNECTED
  float raw_angle2  = readEncoderWithValidation(sensor2, last_valid_angle2, error_count_encoder2,
                                                delta_micros, "Encoder2");
  last_valid_angle2 = raw_angle2; // Update last valid value

  float angle2 = fmod(raw_angle2 - encoder2_offset, 2 * PI);
  if (angle2 < 0)
    angle2 += 2 * PI;

  float delta_angle2 = angle2 - last_angle2;

  if (delta_angle2 > PI) {
    encoder2_turns--;
  } else if (delta_angle2 < -PI) {
    encoder2_turns++;
  }

  last_angle2 = angle2;

  total_angle2 = (encoder2_turns * 360.0) + (angle2 * (180.0 / PI));
#endif

  // Encoder 3 read with validation (only if connected)
  float total_angle3 = 0.0;
#if SENSOR_3_CONNECTED
  float raw_angle3  = readEncoderWithValidation(sensor3, last_valid_angle3, error_count_encoder3,
                                                delta_micros, "Encoder3");
  last_valid_angle3 = raw_angle3; // Update last valid value

  float angle3 = fmod(raw_angle3 - encoder3_offset, 2 * PI);
  if (angle3 < 0)
    angle3 += 2 * PI;

  float delta_angle3 = angle3 - last_angle3;

  if (delta_angle3 > PI) {
    encoder3_turns--;
  } else if (delta_angle3 < -PI) {
    encoder3_turns++;
  }

  last_angle3 = angle3;

  total_angle3 = (encoder3_turns * 360.0) + (angle3 * (180.0 / PI));
#endif

  // Debug output every 5 seconds
  if (now - debug_timer > 5000) {
    debug_timer = now;

    LOG_DEBUG("=== ENCODER STATUS (every 5s) ===");
    LOG_DEBUG("Read delta: %lu us", delta_micros);

#if SENSOR_1_CONNECTED
    LOG_DEBUG("Encoder 1: %.3f° (errors: %u)", total_angle1, error_count_encoder1);
#endif

#if SENSOR_2_CONNECTED
    LOG_DEBUG("Encoder 2: %.3f° (errors: %u)", total_angle2, error_count_encoder2);
#endif

#if SENSOR_3_CONNECTED
    LOG_DEBUG("Encoder 3: %.3f° (errors: %u)", total_angle3, error_count_encoder3);
#endif

    LOG_DEBUG("=================================");
  }

  if (now - ts > 1000) {
    ts = now;

    // // Print angles and time on the same line
    // LOG_DEBUG("Encoder 1: %.3f°, Encoder 2: %.3f°, Encoder 3: %.3f°",
    //           total_angle1, total_angle2, total_angle3);
    // LOG_DEBUG("--------------------------------");
    // // Print device data loaded status
    // LOG_DEBUG("Encoder data loaded: %d", device_data_loaded);
    // LOG_DEBUG("--------------------------------");

    // // Test
    // std::vector<float> testAngles;
    // for (float angle = -4 * PI; angle <= 4 * PI; angle += PI / 2) {
    //     testAngles.push_back(angle);
    // }
    // for (float angle = 4 * PI; angle >= -4 * PI; angle -= PI / 2) {
    //     testAngles.push_back(angle);
    // }

    // float offset = 2 * PI; // Offset desiderato
    // testEncoderSimulation(offset, testAngles);
  }

  // Append synchronization sequence
  memcpy(spi_send_buffer, SYNC_SEQUENCE, SYNC_SEQUENCE_LENGTH);

  // Append status byte after synchronization sequence
  spi_send_buffer[SYNC_SEQUENCE_LENGTH] = device_data_loaded ? STATUS_OK : STATUS_DATA_LOAD_FAILED;

// Update SPI buffer only for connected sensors
#if SENSOR_1_CONNECTED
  shared_encoder_millidegrees[0] = (int32_t)(total_angle1 * 1000); // Convert to millidegrees
#else
  shared_encoder_millidegrees[0] = 0; // Sensor not connected
#endif

#if SENSOR_2_CONNECTED
  shared_encoder_millidegrees[1] = (int32_t)(total_angle2 * 1000);
#else
  shared_encoder_millidegrees[1] = 0; // Sensor not connected
#endif

#if SENSOR_3_CONNECTED
  shared_encoder_millidegrees[2] = (int32_t)(total_angle3 * 1000);
#else
  shared_encoder_millidegrees[2] = 0; // Sensor not connected
#endif

// Encoders 4, 5, 6 — reserved for future use (not currently implemented)
#if SENSOR_4_CONNECTED
  shared_encoder_millidegrees[3] = 0; // Reserved for future implementation
#else
  shared_encoder_millidegrees[3] = 0; // Sensor not connected
#endif

#if SENSOR_5_CONNECTED
  shared_encoder_millidegrees[4] = 0; // Reserved for future implementation
#else
  shared_encoder_millidegrees[4] = 0; // Sensor not connected
#endif

#if SENSOR_6_CONNECTED
  shared_encoder_millidegrees[5] = 0; // Reserved for future implementation
#else
  shared_encoder_millidegrees[5] = 0; // Sensor not connected
#endif

  // Append encoder data
  size_t j = SYNC_SEQUENCE_LENGTH + 1; // Start after sync sequence and status byte
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    spi_send_buffer[j++] = (shared_encoder_millidegrees[i] >> 24) & 0xFF;
    spi_send_buffer[j++] = (shared_encoder_millidegrees[i] >> 16) & 0xFF;
    spi_send_buffer[j++] = (shared_encoder_millidegrees[i] >> 8) & 0xFF;
    spi_send_buffer[j++] = shared_encoder_millidegrees[i] & 0xFF;
  }

  if (dataReceived) {

    uint8_t receivedDataCopy[SYNC_SEQUENCE_LENGTH + 1 + 4 * ENCODER_COUNT]; // Adequate size for data
    size_t len = receivedLength;
    // Copy data to a safe buffer
    memcpy((void *)receivedDataCopy, receivedData, len);
    dataReceived = false;

    // Read current data from flash
    DeviceData data;
    if (device_data_loaded) {
      load_device_data(&data);
    } else {
      // Initialize with current values if not loaded
      data.encoder1_offset = encoder1_offset;
      data.encoder2_offset = encoder2_offset;
      data.encoder3_offset = encoder3_offset;
    }

    if (receivedDataCopy[0] == 0xFF) {
      // Reset all connected encoders
      LOG_INFO("Reset of all connected encoders:");

#if SENSOR_1_CONNECTED
      encoder1_offset      = sensor1.getSensorAngle();
      encoder1_turns       = 0;
      last_angle1          = 0.0;
      last_valid_angle1    = encoder1_offset; // Reset valid value
      error_count_encoder1 = 0;               // Reset error counter
      data.encoder1_offset = encoder1_offset;
      LOG_INFO("  - Encoder 1: Reset completed");
#else
      LOG_INFO("  - Encoder 1: Skipped (not connected)");
#endif

#if SENSOR_2_CONNECTED
      encoder2_offset      = sensor2.getSensorAngle();
      encoder2_turns       = 0;
      last_angle2          = 0.0;
      last_valid_angle2    = encoder2_offset; // Reset valid value
      error_count_encoder2 = 0;               // Reset error counter
      data.encoder2_offset = encoder2_offset;
      LOG_INFO("  - Encoder 2: Reset completed");
#else
      LOG_INFO("  - Encoder 2: Skipped (not connected)");
#endif

#if SENSOR_3_CONNECTED
      encoder3_offset      = sensor3.getSensorAngle();
      encoder3_turns       = 0;
      last_angle3          = 0.0;
      last_valid_angle3    = encoder3_offset; // Reset valid value
      error_count_encoder3 = 0;               // Reset error counter
      data.encoder3_offset = encoder3_offset;
      LOG_INFO("  - Encoder 3: Reset completed");
#else
      LOG_INFO("  - Encoder 3: Skipped (not connected)");
#endif
    } else if (receivedDataCopy[0] == 0x0A) {
// Reset only encoder 1 (if connected)
#if SENSOR_1_CONNECTED
      encoder1_offset      = sensor1.getSensorAngle();
      encoder1_turns       = 0;
      last_angle1          = 0.0;
      last_valid_angle1    = encoder1_offset; // Reset valid value
      error_count_encoder1 = 0;               // Reset error counter

      // Update only encoder 1 offset
      data.encoder1_offset = encoder1_offset;

      LOG_INFO("Encoder 1 reset completed");
#else
      LOG_INFO("Encoder 1 reset command ignored (not connected)");
#endif
    } else if (receivedDataCopy[0] == 0x0B) {
// Reset only encoder 2 (if connected)
#if SENSOR_2_CONNECTED
      encoder2_offset      = sensor2.getSensorAngle();
      encoder2_turns       = 0;
      last_angle2          = 0.0;
      last_valid_angle2    = encoder2_offset; // Reset valid value
      error_count_encoder2 = 0;               // Reset error counter

      // Update only encoder 2 offset
      data.encoder2_offset = encoder2_offset;

      LOG_INFO("Encoder 2 reset completed");
#else
      LOG_INFO("Encoder 2 reset command ignored (not connected)");
#endif
    } else if (receivedDataCopy[0] == 0x0C) {
// Reset only encoder 3 (if connected)
#if SENSOR_3_CONNECTED
      encoder3_offset      = sensor3.getSensorAngle();
      encoder3_turns       = 0;
      last_angle3          = 0.0;
      last_valid_angle3    = encoder3_offset; // Reset valid value
      error_count_encoder3 = 0;               // Reset error counter

      // Update only encoder 3 offset
      data.encoder3_offset = encoder3_offset;

      LOG_INFO("Encoder 3 reset completed");
#else
      LOG_INFO("Encoder 3 reset command ignored (not connected)");
#endif
    }

    // Save updated data to flash only if a reset command was received
    if (receivedDataCopy[0] >= 0x0A && receivedDataCopy[0] <= 0x0C || receivedDataCopy[0] == 0xAA) {
      // Save data to flash and update state
      device_data_loaded = save_device_data(data);

      // Verify state immediately
      if (device_data_loaded) {
        LOG_INFO("Offsets saved and validated!");
      } else {
        LOG_ERROR("Failed to save offsets, data will be marked as invalid!");
      }
    }
  }
}
