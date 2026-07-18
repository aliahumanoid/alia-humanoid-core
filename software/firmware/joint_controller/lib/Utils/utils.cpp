/**
 * @file utils.cpp
 * @brief Implementation of utility functions
 * 
 * See utils.h for detailed documentation.
 */

#include "utils.h"
#include "IntercoreSync.h"
#include "debug.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <Arduino.h>
#include <hot_path.h>
#include <array>
#include <cstring>

/**
 * @brief Handshake with Core1 before flash operations.
 *
 * Signals Core1 to park in a RAM-resident wait loop, then waits for
 * Core1 to acknowledge.  Replaces the previous delay(5) with a proper
 * bidirectional handshake that is immune to Core1 timing jitter.
 *
 * Timeout (50 ms) aborts the flash write instead of risking an erase/program
 * while Core1 is still executing from XIP.
 */
static bool wait_for_core1_flash_ready(const char *context_label) {
  return begin_core1_flash_pause(context_label);
}

// Data format versions
static constexpr uint16_t PID_FLASH_VERSION            = 4;  ///< PID-only format version
static constexpr uint16_t LINEAR_EQ_FLASH_VERSION      = 5;  ///< Linear equations format version
static constexpr uint16_t SYSTEM_SETTINGS_FLASH_VERSION = 6;  ///< System settings format version
static constexpr uint16_t MOTOR_OFFSETS_FLASH_VERSION   = 7;  ///< Motor offsets format version
static constexpr uint16_t FINE_MAP_FLASH_VERSION        = 8;  ///< Fine-map (piecewise) format version
static constexpr uint16_t FINE_MAP_2D_FLASH_VERSION     = 9;  ///< Fine-map 2D (bilinear) format version

// Sub-offsets within the LINEAR_EQ sector (whole-sector RMW co-locates v5 and v8 records).
static constexpr uint32_t LINEAR_EQ_V5_SUBOFFSET = 0x000u;
static constexpr uint32_t FINE_MAP_V8_SUBOFFSET  = 0x400u; // 0x800 reserved for v9 2D
static constexpr uint32_t FINE_MAP_V9_SUBOFFSET  = 0x800u; // reserved (future 2D bilinear map)
// Compile-time layout guards: if any record grows past its sub-offset window, fail the build
// instead of silently corrupting the co-located record at flash-write time.
static_assert(sizeof(struct LinearEquationsDeviceData) <= FINE_MAP_V8_SUBOFFSET,
              "v5 linear-eq record overlaps the v8 fine-map sub-offset (0x400)");
static_assert(FINE_MAP_V8_SUBOFFSET + sizeof(struct FineMapDeviceDataV8) <= FINE_MAP_V9_SUBOFFSET,
              "v8 fine-map record overlaps the v9 (2D) reserved region (0x800)");
// (The weaker `FINE_MAP_V9_SUBOFFSET <= FLASH_SECTOR_SIZE` guard is implied by the next one.)
static_assert(FINE_MAP_V9_SUBOFFSET + sizeof(struct FineMap2DDeviceDataV9) <= FLASH_SECTOR_SIZE,
              "v9 2D record exceeds the 4KB sector");

// Time tracking globals for overflow handling
unsigned long overflow_count;  ///< Number of micros() overflows
unsigned long last_micros;     ///< Previous micros() reading

// ===================================================================
// HELPER FUNCTIONS
// ===================================================================

/**
 * @brief Calculate simple checksum for data integrity verification
 * 
 * Computes sum of all bytes in the data block. Simple but effective
 * for detecting corruption in flash storage.
 * 
 * @param data Pointer to data buffer
 * @param size Number of bytes to checksum
 * @return 16-bit checksum value (sum of all bytes)
 */
uint16_t calculate_checksum(const uint8_t *data, size_t size) {
  uint16_t checksum = 0;
  for (size_t i = 0; i < size; i++) {
    checksum += data[i];
  }
  return checksum;
}

/**
 * @brief Read and validate linear equations data from flash (internal helper)
 * 
 * Private function to read linear equations blob from specified flash offset.
 * Performs validation of magic number, version, and checksum.
 * 
 * @param flash_offset Absolute flash offset to read from
 * @param slot_label Human-readable name for error messages
 * @param[out] data Pointer to structure where data will be loaded
 * @param log_errors Whether to print error messages to Serial
 * @return true if data valid and loaded successfully, false otherwise
 */
static bool read_linear_equations_blob(uint32_t flash_offset, const char *slot_label,
                                       struct LinearEquationsDeviceData *data, bool log_errors) {
  if (data == NULL) {
    if (log_errors) {
      LOG_ERROR("Invalid equations pointer");
    }
    return false;
  }

  uint8_t *data_ptr        = reinterpret_cast<uint8_t *>(data);
  size_t data_size         = sizeof(struct LinearEquationsDeviceData);
  const uint8_t *flash_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + flash_offset);

  // Step 1: Read header to check magic number and version
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  if (data->magic_number != MAGIC_NUMBER) {
    if (log_errors) {
      LOG_DEBUG("Equations not found in slot " + String(slot_label) + " (missing magic number)");
    }
    return false;
  }

  if (data->version != LINEAR_EQ_FLASH_VERSION) {
    if (log_errors) {
      LOG_WARN("Slot " + String(slot_label) + ": version " + String(data->version) +
               " incompatible (expected " + String(LINEAR_EQ_FLASH_VERSION) + ")");

      if (data->version == PID_FLASH_VERSION) {
        LOG_DEBUG("Note: slot contains PID data. Ignoring for equations.");
      }
    }
    return false;
  }

  // Step 2: Full copy of data now that format is validated
  memcpy(data_ptr, flash_ptr, data_size);

  // Step 3: Verify checksum (excludes header: magic, version, checksum)
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    if (log_errors) {
      LOG_ERROR("Invalid equations checksum in slot " + String(slot_label));
    }
    return false;
  }

  return true;
}

static bool read_pid_blob(uint32_t flash_offset, const char *slot_label,
                          struct PIDOnlyDeviceData *data, bool log_errors) {
  if (data == NULL) {
    if (log_errors) {
      LOG_ERROR("Invalid PID pointer");
    }
    return false;
  }

  uint8_t *data_ptr        = reinterpret_cast<uint8_t *>(data);
  size_t data_size         = sizeof(struct PIDOnlyDeviceData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + flash_offset);

  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  if (data->magic_number != MAGIC_NUMBER) {
    if (log_errors) {
      LOG_DEBUG("No PID data found in slot " + String(slot_label) + " (missing magic number)");
    }
    return false;
  }

  if (data->version < PID_FLASH_VERSION || data->version >= LINEAR_EQ_FLASH_VERSION) {
    if (log_errors) {
      LOG_WARN("Slot " + String(slot_label) + ": unexpected PID version " + String(data->version) +
               " (expected " + String(PID_FLASH_VERSION) + ")");
    }
    return false;
  }

  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    if (log_errors) {
      LOG_ERROR("Invalid PID checksum in slot " + String(slot_label));
    }
    return false;
  }

  if (data->dof_count == 0 || data->dof_count > MAX_DOFS || data->motor_count > MAX_MOTORS) {
    if (log_errors) {
      LOG_ERROR("Invalid PID data in slot " + String(slot_label) + " - count out of range");
    }
    return false;
  }

  return true;
}

static bool read_system_settings_blob(uint32_t flash_offset, const char *slot_label,
                                      struct SystemSettingsData *data, bool log_errors) {
  if (data == NULL) {
    if (log_errors) {
      LOG_ERROR("Invalid system settings pointer");
    }
    return false;
  }

  uint8_t *data_ptr        = reinterpret_cast<uint8_t *>(data);
  size_t data_size         = sizeof(struct SystemSettingsData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + flash_offset);

  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  if (data->magic_number != MAGIC_NUMBER) {
    if (log_errors) {
      LOG_DEBUG("No system settings found in slot " + String(slot_label) + " (missing magic number)");
    }
    return false;
  }

  if (data->version != SYSTEM_SETTINGS_FLASH_VERSION) {
    if (log_errors) {
      LOG_WARN("Slot " + String(slot_label) + ": unexpected system settings version " +
               String(data->version) + " (expected " + String(SYSTEM_SETTINGS_FLASH_VERSION) + ")");
    }
    return false;
  }

  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    if (log_errors) {
      LOG_ERROR("Invalid system settings checksum in slot " + String(slot_label));
    }
    return false;
  }

  return true;
}

static bool read_motor_offsets_blob(uint32_t flash_offset, const char *slot_label,
                                    struct MotorOffsetsDeviceData *data, bool log_errors) {
  if (data == NULL) {
    if (log_errors) {
      LOG_ERROR("Invalid motor offsets pointer");
    }
    return false;
  }

  uint8_t *data_ptr        = reinterpret_cast<uint8_t *>(data);
  size_t data_size         = sizeof(struct MotorOffsetsDeviceData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + flash_offset);

  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  if (data->magic_number != MAGIC_NUMBER) {
    if (log_errors) {
      LOG_DEBUG("No motor offsets found in slot " + String(slot_label) + " (missing magic number)");
    }
    return false;
  }

  if (data->version != MOTOR_OFFSETS_FLASH_VERSION) {
    if (log_errors) {
      LOG_WARN("Slot " + String(slot_label) + ": unexpected motor offsets version " +
               String(data->version) + " (expected " + String(MOTOR_OFFSETS_FLASH_VERSION) + ")");
    }
    return false;
  }

  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    if (log_errors) {
      LOG_ERROR("Invalid motor offsets checksum in slot " + String(slot_label));
    }
    return false;
  }

  return true;
}

// ===================================================================
// FLASH STORAGE - PID DATA SAVE/LOAD
// ===================================================================

/**
 * Save PID-only configuration to flash
 * 
 * Compact storage format containing only PID gains and outer loop parameters.
 * Much smaller than full device data with calibration.
 * 
 * Flash write procedure:
 * 1. Populate metadata (magic, version, timestamp)
 * 2. Calculate checksum
 * 3. Disable interrupts for atomic write
 * 4. Erase required flash sectors
 * 5. Program data in 256-byte pages
 * 6. Restore interrupts
 * 7. Print confirmation
 */
void save_pid_only_data(struct PIDOnlyDeviceData data) {
  // Populate header metadata
  data.magic_number = MAGIC_NUMBER;
  data.version      = PID_FLASH_VERSION;  // Version with outer/cascade PID parameters
  data.timestamp    = millis();           // Save timestamp

  // Calculate checksum (excludes header)
  data.checksum =
      calculate_checksum((uint8_t *)&data + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         sizeof(PIDOnlyDeviceData) - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  uint8_t *data_ptr = (uint8_t *)&data;
  size_t data_size  = sizeof(struct PIDOnlyDeviceData);
  size_t offset     = 0;

  // Calculate number of sectors to erase (4KB each)
  size_t num_sectors = (data_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

  // Handshake: wait for Core1 to park in RAM before flash erase/program
  if (!wait_for_core1_flash_ready("PID save")) {
    LOG_ERROR("PID-only data save aborted: Core1 flash handshake failed");
    return;
  }
  
  // Atomic flash operation: disable interrupts during write
  uint32_t ints = save_and_disable_interrupts();

  // Erase flash sectors before programming
  flash_range_erase(FLASH_PID_OFFSET, num_sectors * FLASH_SECTOR_SIZE);

  // Program data in 256-byte pages
  while (offset < data_size) {
    size_t chunk_size =
        (data_size - offset > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : data_size - offset;

    // Prepare page buffer (0xFF for erased state)
    uint8_t flash_page[FLASH_PAGE_SIZE];
    memset(flash_page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(flash_page, data_ptr + offset, chunk_size);

    // Program flash page
    flash_range_program(FLASH_PID_OFFSET + offset, flash_page, FLASH_PAGE_SIZE);
    offset += FLASH_PAGE_SIZE;
  }

  // Restore interrupts
  restore_interrupts(ints);
  
  // Signal Core1 to resume normal operation
  end_core1_flash_pause();

  // Print confirmation
  LOG_INFO("PID-only data saved successfully!");
  LOG_DEBUG("Joint type: " + String(data.joint_type));
  LOG_DEBUG("DOF count: " + String(data.dof_count));
  LOG_DEBUG("Motor count: " + String(data.motor_count));
  LOG_DEBUG("Data size: " + String(data_size) + " bytes (very compact!)");
}

/**
 * Load PID-only configuration from flash
 * 
 * Reads and validates PID data with multiple checks:
 * - Magic number verification
 * - Version compatibility
 * - Checksum validation
 * - Range validation
 */
bool load_pid_only_data(struct PIDOnlyDeviceData *data) {
  if (data == NULL) {
    LOG_ERROR("Invalid data pointer provided!");
    return false;
  }

  bool loaded_from_legacy = false;
  if (!read_pid_blob(FLASH_PID_OFFSET, "PID", data, true)) {
    if (!read_pid_blob(FLASH_PID_OFFSET_LEGACY, "LEGACY_PID", data, true)) {
      return false;
    }
    loaded_from_legacy = true;
  }

  if (loaded_from_legacy) {
    LOG_WARN("PID data found only in legacy flash slot - migrating to top-of-flash NVM");
    save_pid_only_data(*data);
  }

  LOG_INFO("PID-only data loaded successfully!");
  LOG_DEBUG("Joint type: " + String(data->joint_type));
  LOG_DEBUG("DOF count: " + String(data->dof_count));
  LOG_DEBUG("Motor count: " + String(data->motor_count));
  LOG_DEBUG("Saved timestamp: " + String(data->timestamp));

  // Print PID configurations for all motors
  int valid_pid_count = 0;
  for (int motor = 0; motor < data->motor_count && motor < MAX_MOTORS; motor++) {
    if (data->pid_data[motor].kp != 0) {
      LOG_DEBUG("Motor " + String(motor) + " PID: kp=" + String(data->pid_data[motor].kp, 4) +
                ", ki=" + String(data->pid_data[motor].ki, 4) +
                ", kd=" + String(data->pid_data[motor].kd, 4));
      valid_pid_count++;
    }
  }
  LOG_DEBUG("Loaded " + String(valid_pid_count) + " valid PID configurations");

  return true;
}

// ===================================================================
// TIME MANAGEMENT
// ===================================================================

/**
 * Get high-resolution time with overflow handling
 * 
 * Tracks micros() overflow to provide continuous time measurement
 * beyond 2^32 microseconds (~71.6 minutes).
 */
float get_current_time() {
  unsigned long current_micros = micros();
  
  // Detect overflow (micros() wraps at 2^32)
  if (current_micros < last_micros) {
    overflow_count++;
  }
  last_micros = current_micros;
  
  // Calculate total microseconds including overflows
  unsigned long long total_micros =
      current_micros + ((unsigned long long)overflow_count * 4294967296ULL); // 2^32
  
  // Convert to seconds
  return total_micros / 1e6;
}

// ===================================================================
// INTERPOLATION
// ===================================================================

/**
 * Generic linear interpolation between two data arrays
 * 
 * Handles both increasing and decreasing sequences in data1.
 * Performs boundary clamping if target_value is out of range.
 * 
 * Algorithm:
 * 1. Determine if data1 is increasing or decreasing
 * 2. Find bracket indices where target_value falls
 * 3. Linearly interpolate between corresponding data2 values
 * 4. Clamp to boundaries if out of range
 */
float HOT_FUNC(interpolate_data)(float target_value, const float *data1, const float *data2, int size) {
  // Determine data1 sequence direction
  bool is_increasing = data1[0] < data1[size - 1];
  float result_value = 0.0f;
  int i              = 0;

  // Find bracket index: first element >= target_value (increasing)
  // or first element <= target_value (decreasing)
  if (is_increasing) {
    while (i < size && data1[i] < target_value) {
      i++;
    }
  } else {
    while (i < size && data1[i] > target_value) {
      i++;
    }
  }

  // Handle boundary cases and interpolation
  if (i == 0) {
    // target_value is below range (less than first element)
    // Clamp to first value
    result_value = data2[0];
  } else if (i >= size) {
    // target_value is above range (greater than last element)
    // Clamp to last value
    result_value = data2[size - 1];
  } else {
    // Linear interpolation between indices i-1 and i
    // Formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    result_value = data2[i - 1] + (target_value - data1[i - 1]) * (data2[i] - data2[i - 1]) /
                                      (data1[i] - data1[i - 1]);
  }

  return result_value;
}


// ===================================================================
// FLASH STORAGE - LINEAR EQUATIONS SAVE/LOAD
// ===================================================================

// Whole-sector RMW writer for the LINEAR_EQ sector. Reads the current sector into RAM, patches the
// v5, v8 and/or v9 record, then erases + reprograms the WHOLE sector in one window — so a v5 save
// never wipes a co-located v8 fine-map or v9 2D grid, and vice-versa. Pass nullptr for any record
// you are NOT updating.
static void write_linear_eq_sector(const struct LinearEquationsDeviceData *v5,
                                   const struct FineMapDeviceDataV8 *v8,
                                   const struct FineMap2DDeviceDataV9 *v9) {
  static uint8_t sector[FLASH_SECTOR_SIZE];  // .bss, not stack
  memcpy(sector, (const void *)(XIP_BASE + FLASH_LINEAR_EQ_OFFSET), FLASH_SECTOR_SIZE);
  if (v5) {
    struct LinearEquationsDeviceData rec = *v5;
    rec.magic_number = MAGIC_NUMBER;
    rec.version      = LINEAR_EQ_FLASH_VERSION;
    rec.timestamp    = millis();
    rec.checksum     = calculate_checksum((uint8_t *)&rec + 8, sizeof(rec) - 8);
    memcpy(sector + LINEAR_EQ_V5_SUBOFFSET, &rec, sizeof(rec));
  }
  if (v8) {
    struct FineMapDeviceDataV8 rec = *v8;
    rec.magic     = FINE_MAP_V8_MAGIC;
    rec.version   = FINE_MAP_FLASH_VERSION;
    rec.timestamp = millis();
    rec.checksum  = calculate_checksum((uint8_t *)&rec + 8, sizeof(rec) - 8);
    memcpy(sector + FINE_MAP_V8_SUBOFFSET, &rec, sizeof(rec));
  }
  if (v9) {
    struct FineMap2DDeviceDataV9 rec = *v9;
    rec.magic     = FINE_MAP_V9_MAGIC;
    rec.version   = FINE_MAP_2D_FLASH_VERSION;
    rec.timestamp = millis();
    rec.checksum  = calculate_checksum((uint8_t *)&rec + 8, sizeof(rec) - 8);
    memcpy(sector + FINE_MAP_V9_SUBOFFSET, &rec, sizeof(rec));
  }
  if (!wait_for_core1_flash_ready("linear-eq sector write")) {
    LOG_ERROR("Sector write aborted: Core1 flash handshake failed");
    return;
  }
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_LINEAR_EQ_OFFSET, FLASH_SECTOR_SIZE);
  for (size_t off = 0; off < FLASH_SECTOR_SIZE; off += FLASH_PAGE_SIZE) {
    flash_range_program(FLASH_LINEAR_EQ_OFFSET + off, sector + off, FLASH_PAGE_SIZE);
  }
  restore_interrupts(ints);
  end_core1_flash_pause();
}

/**
 * Save linear calibration equations to flash
 *
 * Similar procedure to PID save but stores motor-joint mapping equations
 * with calibration metadata (R², MSE, safety limits).
 *
 * The actual flash write goes through write_linear_eq_sector() (whole-sector RMW) so a co-located
 * v8 fine-map record in the same sector is preserved.
 */
void save_linear_equations_data(struct LinearEquationsDeviceData data) {
  // Populate header metadata (the writer re-sets these too, but keep them for the prints below).
  data.magic_number = MAGIC_NUMBER;
  data.version      = LINEAR_EQ_FLASH_VERSION;  // Version with safety limits
  data.timestamp    = millis();                 // Save timestamp

  size_t data_size = sizeof(struct LinearEquationsDeviceData);

  // Whole-sector read-modify-write: patches only the v5 record, preserves any v8/v9 records.
  write_linear_eq_sector(&data, nullptr, nullptr);

  // Print confirmation with equation details
  SERIAL_COM_LN("Linear equations saved to flash successfully!");
  SERIAL_COM("Joint type: ");
  SERIAL_COM_LN(data.joint_type);
  SERIAL_COM("DOF count: ");
  SERIAL_COM_LN(data.dof_count);
  SERIAL_COM("Motor count: ");
  SERIAL_COM_LN(data.motor_count);
  SERIAL_COM("Data size: ");
  SERIAL_COM(data_size);
  SERIAL_COM_LN(" bytes (ultra-compact!)");

  // Print summary of saved equations
  int valid_equations_count = 0;
  for (int dof = 0; dof < data.dof_count && dof < MAX_DOFS; dof++) {
    if (data.dof_equations[dof].calculated) {
      SERIAL_COM_LN("DOF " + String(dof) + " equations saved:");
      if (data.dof_equations[dof].agonist.valid) {
        SERIAL_COM_LN("  Agonist: y = " + String(data.dof_equations[dof].agonist.slope, 4) +
                       "*x + " + String(data.dof_equations[dof].agonist.intercept, 4) +
                       " (R^2=" + String(data.dof_equations[dof].agonist.r_squared, 3) + ")");
        valid_equations_count++;
      }
      if (data.dof_equations[dof].antagonist.valid) {
        SERIAL_COM_LN("  Antagonist: y = " + String(data.dof_equations[dof].antagonist.slope, 4) +
                       "*x + " + String(data.dof_equations[dof].antagonist.intercept, 4) +
                       " (R^2=" + String(data.dof_equations[dof].antagonist.r_squared, 3) + ")");
        valid_equations_count++;
      }
      SERIAL_COM_LN("  Joint limits: [" + String(data.dof_equations[dof].joint_safe_min, 2) +
                     "°, " + String(data.dof_equations[dof].joint_safe_max, 2) + "°]");
      SERIAL_COM_LN("  Agonist limits: [" + String(data.dof_equations[dof].agonist_safe_min, 2) +
                     "°, " + String(data.dof_equations[dof].agonist_safe_max, 2) + "°]");
      SERIAL_COM_LN("  Antagonist limits: [" +
                     String(data.dof_equations[dof].antagonist_safe_min, 2) + "°, " +
                     String(data.dof_equations[dof].antagonist_safe_max, 2) + "°]");
    }
  }
  SERIAL_COM_LN("Saved " + String(valid_equations_count) + " valid linear equations");
}

/**
 * Load linear calibration equations from flash
 * 
 * Uses helper function to read and validate equations blob.
 * Prints detailed equation summary including R² and MSE statistics.
 */
bool load_linear_equations_data(struct LinearEquationsDeviceData *data) {
  bool loaded_from_legacy = false;
  if (!read_linear_equations_blob(FLASH_LINEAR_EQ_OFFSET, "LINEAR_EQ", data, true)) {
    if (!read_linear_equations_blob(FLASH_LINEAR_EQ_OFFSET_LEGACY, "LEGACY_LINEAR_EQ", data, true)) {
      return false;
    }
    loaded_from_legacy = true;
  }

  // Further validate data
  if (data->dof_count == 0 || data->dof_count > MAX_DOFS || data->motor_count > MAX_MOTORS) {
    SERIAL_COM_LN("Invalid linear equations data - DOF or motor count out of range!");
    return false;
  }

  if (loaded_from_legacy) {
    LOG_WARN("Linear equations found only in legacy flash slot - migrating to top-of-flash NVM");
    save_linear_equations_data(*data);
  }

  SERIAL_COM_LN("Linear equations loaded successfully from flash!");
  SERIAL_COM("Joint type: ");
  SERIAL_COM_LN(data->joint_type);
  SERIAL_COM("DOF count: ");
  SERIAL_COM_LN(data->dof_count);
  SERIAL_COM("Motor count: ");
  SERIAL_COM_LN(data->motor_count);
  SERIAL_COM("Saved timestamp: ");
  SERIAL_COM_LN(data->timestamp);

  // Print loaded equations
  int valid_equations_count = 0;
  for (int dof = 0; dof < data->dof_count && dof < MAX_DOFS; dof++) {
    if (data->dof_equations[dof].calculated) {
      SERIAL_COM_LN("DOF " + String(dof) + " equations loaded:");
      if (data->dof_equations[dof].agonist.valid) {
        SERIAL_COM_LN("  Agonist: y = " + String(data->dof_equations[dof].agonist.slope, 4) +
                       "*x + " + String(data->dof_equations[dof].agonist.intercept, 4) +
                       " (R^2=" + String(data->dof_equations[dof].agonist.r_squared, 3) +
                       ", MSE=" + String(data->dof_equations[dof].agonist.mse, 3) + ")");
        valid_equations_count++;
      }
      if (data->dof_equations[dof].antagonist.valid) {
        SERIAL_COM_LN(
            "  Antagonist: y = " + String(data->dof_equations[dof].antagonist.slope, 4) + "*x + " +
            String(data->dof_equations[dof].antagonist.intercept, 4) +
            " (R^2=" + String(data->dof_equations[dof].antagonist.r_squared, 3) +
            ", MSE=" + String(data->dof_equations[dof].antagonist.mse, 3) + ")");
        valid_equations_count++;
      }
    }
  }
  SERIAL_COM_LN("Loaded " + String(valid_equations_count) + " valid linear equations");

  return true;
}

// ===================================================================
// FLASH STORAGE - FINE-MAP (PIECEWISE) SAVE/LOAD  (v8, co-located in LINEAR_EQ sector)
// ===================================================================

/**
 * Save the v8 fine-map record into the LINEAR_EQ sector (whole-sector RMW).
 *
 * Patches only the v8 sub-offset; any co-located v5 linear-eq record is preserved.
 */
void save_fine_map_data(const struct FineMapDeviceDataV8 *fm) {
  if (fm == NULL) {
    LOG_ERROR("Invalid fine-map pointer");
    return;
  }
  write_linear_eq_sector(nullptr, fm, nullptr);
  LOG_INFO("Fine-map (v8) saved to flash (DOF count " + String(fm->dof_count) + ")");
}

/**
 * @brief Read and validate the v8 fine-map blob from flash (internal helper)
 *
 * Mirrors read_linear_equations_blob: validates magic, version, and checksum.
 */
static bool read_fine_map_blob(struct FineMapDeviceDataV8 *data, bool log_errors) {
  if (data == NULL) {
    if (log_errors) {
      LOG_ERROR("Invalid fine-map pointer");
    }
    return false;
  }

  uint8_t *data_ptr = reinterpret_cast<uint8_t *>(data);
  size_t data_size  = sizeof(struct FineMapDeviceDataV8);
  const uint8_t *flash_ptr =
      reinterpret_cast<const uint8_t *>(XIP_BASE + FLASH_LINEAR_EQ_OFFSET + FINE_MAP_V8_SUBOFFSET);

  // Step 1: Read header to check magic number and version
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  if (data->magic != FINE_MAP_V8_MAGIC) {
    if (log_errors) {
      LOG_DEBUG("No fine-map found (missing magic number)");
    }
    return false;
  }

  if (data->version != FINE_MAP_FLASH_VERSION) {
    if (log_errors) {
      LOG_WARN("Fine-map version " + String(data->version) + " incompatible (expected " +
               String(FINE_MAP_FLASH_VERSION) + ")");
    }
    return false;
  }

  // Step 2: Full copy now that the format is validated
  memcpy(data_ptr, flash_ptr, data_size);

  // Step 3: Verify checksum (excludes the 8-byte header: magic, version, checksum)
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    if (log_errors) {
      LOG_ERROR("Invalid fine-map checksum");
    }
    return false;
  }

  return true;
}

/**
 * Load the v8 fine-map record from flash.
 *
 * @return true if a valid fine-map blob was found; false on absence/corruption (caller stays LINEAR).
 */
bool load_fine_map_data(struct FineMapDeviceDataV8 *data) {
  return read_fine_map_blob(data, true);
}

// ===================================================================
// FLASH STORAGE - FINE-MAP 2D (BILINEAR) SAVE/LOAD  (v9, co-located in LINEAR_EQ sector)
// ===================================================================

/**
 * Save the v9 2D (bilinear) record into the LINEAR_EQ sector (whole-sector RMW).
 *
 * Patches only the v9 sub-offset; any co-located v5 linear-eq and v8 fine-map records are preserved.
 */
void save_fine_map_2d_data(const struct FineMap2DDeviceDataV9 *v9) {
  if (v9 == NULL) {
    LOG_ERROR("Invalid fine-map 2D pointer");
    return;
  }
  write_linear_eq_sector(nullptr, nullptr, v9);
  LOG_INFO("Fine-map 2D (v9) saved to flash (grids " + String(v9->n_grids) + ")");
}

/**
 * @brief Save the v8 fine-map and the v9 2D grid in ONE whole-sector RMW (2026-07-10)
 *
 * Used by the fine-map save when it also drops a stale v9 grid slot: two separate
 * saves would erase+program the same 4KB sector twice (double wear) and leave a
 * non-atomic window (a crash after the v8 write but before the v9 drop persists the
 * exact stale-grid re-promotion bug the drop exists to fix). Pass nullptr for any
 * record you are not updating.
 */
void save_fine_map_and_2d(const struct FineMapDeviceDataV8 *v8,
                          const struct FineMap2DDeviceDataV9 *v9) {
  if (v8 == NULL && v9 == NULL) {
    LOG_ERROR("save_fine_map_and_2d: both records null");
    return;
  }
  write_linear_eq_sector(nullptr, v8, v9);
  LOG_INFO("Fine-map v8+v9 saved in one sector write");
}

/**
 * @brief Read and validate the v9 2D (bilinear) blob from flash (internal helper)
 *
 * Mirrors read_fine_map_blob: validates magic, version, and checksum.
 */
static bool read_fine_map_2d_blob(struct FineMap2DDeviceDataV9 *data, bool log_errors) {
  if (data == NULL) {
    if (log_errors) {
      LOG_ERROR("Invalid fine-map 2D pointer");
    }
    return false;
  }

  uint8_t *data_ptr = reinterpret_cast<uint8_t *>(data);
  size_t data_size  = sizeof(struct FineMap2DDeviceDataV9);
  const uint8_t *flash_ptr =
      reinterpret_cast<const uint8_t *>(XIP_BASE + FLASH_LINEAR_EQ_OFFSET + FINE_MAP_V9_SUBOFFSET);

  // Step 1: Read header to check magic number and version
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  if (data->magic != FINE_MAP_V9_MAGIC) {
    if (log_errors) {
      LOG_DEBUG("No fine-map 2D found (missing magic number)");
    }
    return false;
  }

  if (data->version != FINE_MAP_2D_FLASH_VERSION) {
    if (log_errors) {
      LOG_WARN("Fine-map 2D version " + String(data->version) + " incompatible (expected " +
               String(FINE_MAP_2D_FLASH_VERSION) + ")");
    }
    return false;
  }

  // Step 2: Full copy now that the format is validated
  memcpy(data_ptr, flash_ptr, data_size);

  // Step 3: Verify checksum (excludes the 8-byte header: magic, version, checksum)
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    if (log_errors) {
      LOG_ERROR("Invalid fine-map 2D checksum");
    }
    return false;
  }

  return true;
}

/**
 * Load the v9 2D (bilinear) record from flash.
 *
 * @return true if a valid 2D blob was found; false on absence/corruption (caller stays v5/v8).
 */
bool load_fine_map_2d_data(struct FineMap2DDeviceDataV9 *data) {
  return read_fine_map_2d_blob(data, true);
}

// ===================================================================
// FLASH STORAGE - SYSTEM SETTINGS SAVE/LOAD
// ===================================================================

/**
 * Save system settings to flash
 * 
 * Stores persistent system preferences including auto-start flag.
 */
void save_system_settings_data(struct SystemSettingsData data) {
  // Populate header metadata
  data.magic_number = MAGIC_NUMBER;
  data.version      = SYSTEM_SETTINGS_FLASH_VERSION;
  data.timestamp    = millis();

  // Calculate checksum (excludes header)
  data.checksum =
      calculate_checksum((uint8_t *)&data + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         sizeof(SystemSettingsData) - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  uint8_t *data_ptr = (uint8_t *)&data;
  size_t data_size  = sizeof(struct SystemSettingsData);
  size_t offset     = 0;

  // Calculate number of sectors to erase (4KB each)
  size_t num_sectors = (data_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

  // Handshake: wait for Core1 to park in RAM before flash erase/program
  if (!wait_for_core1_flash_ready("system settings save")) {
    LOG_ERROR("System settings save aborted: Core1 flash handshake failed");
    return;
  }

  // Atomic flash operation: disable interrupts during write
  uint32_t ints = save_and_disable_interrupts();

  // Erase flash sectors before programming
  flash_range_erase(FLASH_SYSTEM_SETTINGS_OFFSET, num_sectors * FLASH_SECTOR_SIZE);

  // Program data in 256-byte pages
  while (offset < data_size) {
    size_t chunk_size =
        (data_size - offset > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : data_size - offset;

    // Prepare page buffer (0xFF for erased state)
    uint8_t flash_page[FLASH_PAGE_SIZE];
    memset(flash_page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(flash_page, data_ptr + offset, chunk_size);

    // Program flash page
    flash_range_program(FLASH_SYSTEM_SETTINGS_OFFSET + offset, flash_page, FLASH_PAGE_SIZE);
    offset += FLASH_PAGE_SIZE;
  }

  // Restore interrupts
  restore_interrupts(ints);

  // Signal Core1 to resume normal operation
  end_core1_flash_pause();

  // Print confirmation
  LOG_INFO("System settings saved successfully!");
  LOG_DEBUG("Joint type: " + String(data.joint_type));
  LOG_DEBUG("Auto-start: " + String(data.auto_start_enabled ? "ENABLED" : "DISABLED"));
  if (data.auto_start_pretension > 0) {
    LOG_DEBUG("Auto-start pretension: " + String(data.auto_start_pretension, 1) + " (custom)");
  }
  if (data.auto_start_duration > 0) {
    LOG_DEBUG("Auto-start duration: " + String(data.auto_start_duration) + "ms (custom)");
  }
}

/**
 * Load system settings from flash
 * 
 * Reads and validates system settings with multiple checks.
 */
bool load_system_settings_data(struct SystemSettingsData *data) {
  if (data == NULL) {
    LOG_ERROR("Invalid data pointer provided!");
    return false;
  }

  bool loaded_from_legacy = false;
  if (!read_system_settings_blob(FLASH_SYSTEM_SETTINGS_OFFSET, "SYSTEM_SETTINGS", data, true)) {
    if (!read_system_settings_blob(FLASH_SYSTEM_SETTINGS_OFFSET_LEGACY, "LEGACY_SYSTEM_SETTINGS",
                                   data, true)) {
      return false;
    }
    loaded_from_legacy = true;
  }

  if (loaded_from_legacy) {
    LOG_WARN("System settings found only in legacy flash slot - migrating to top-of-flash NVM");
    save_system_settings_data(*data);
  }

  // Success - print loaded settings summary
  LOG_INFO("System settings loaded successfully!");
  LOG_DEBUG("Joint type: " + String(data->joint_type));
  LOG_DEBUG("Auto-start: " + String(data->auto_start_enabled ? "ENABLED" : "DISABLED"));

  return true;
}

// ===================================================================
// FLASH STORAGE - MOTOR OFFSETS SAVE/LOAD
// ===================================================================

/**
 * Save motor encoder offsets to flash
 * 
 * Stores offsets from recalculateMotorOffsets() so they can be
 * validated at next boot without requiring full recalibration.
 */
void save_motor_offsets_data(struct MotorOffsetsDeviceData data) {
  // Populate header metadata
  data.magic_number = MAGIC_NUMBER;
  data.version      = MOTOR_OFFSETS_FLASH_VERSION;
  data.timestamp    = millis();

  // Calculate checksum (excludes header)
  data.checksum =
      calculate_checksum((uint8_t *)&data + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         sizeof(MotorOffsetsDeviceData) - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  uint8_t *data_ptr = (uint8_t *)&data;
  size_t data_size  = sizeof(struct MotorOffsetsDeviceData);
  size_t offset     = 0;

  // Calculate number of sectors to erase (4KB each)
  size_t num_sectors = (data_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

  // Handshake: wait for Core1 to park in RAM before flash erase/program
  if (!wait_for_core1_flash_ready("motor offsets save")) {
    LOG_ERROR("Motor offsets save aborted: Core1 flash handshake failed");
    return;
  }

  // Atomic flash operation: disable interrupts during write
  uint32_t ints = save_and_disable_interrupts();

  // Erase flash sectors before programming
  flash_range_erase(FLASH_MOTOR_OFFSETS_OFFSET, num_sectors * FLASH_SECTOR_SIZE);

  // Program data in 256-byte pages
  while (offset < data_size) {
    size_t chunk_size =
        (data_size - offset > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : data_size - offset;

    uint8_t flash_page[FLASH_PAGE_SIZE];
    memset(flash_page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(flash_page, data_ptr + offset, chunk_size);

    flash_range_program(FLASH_MOTOR_OFFSETS_OFFSET + offset, flash_page, FLASH_PAGE_SIZE);
    offset += FLASH_PAGE_SIZE;
  }

  // Restore interrupts
  restore_interrupts(ints);

  // Signal Core1 to resume normal operation
  end_core1_flash_pause();

  LOG_INFO("Motor offsets saved to flash successfully!");
  LOG_DEBUG("Joint type: " + String(data.joint_type));
  LOG_DEBUG("DOF count: " + String(data.dof_count));
}

/**
 * Load motor encoder offsets from flash
 * 
 * Reads and validates saved motor offsets for boot-time validation.
 */
bool load_motor_offsets_data(struct MotorOffsetsDeviceData *data) {
  if (data == NULL) {
    LOG_ERROR("Invalid data pointer provided!");
    return false;
  }

  bool loaded_from_legacy = false;
  if (!read_motor_offsets_blob(FLASH_MOTOR_OFFSETS_OFFSET, "MOTOR_OFFSETS", data, true)) {
    if (!read_motor_offsets_blob(FLASH_MOTOR_OFFSETS_OFFSET_LEGACY, "LEGACY_MOTOR_OFFSETS", data,
                                 true)) {
      return false;
    }
    loaded_from_legacy = true;
  }

  if (loaded_from_legacy) {
    LOG_WARN("Motor offsets found only in legacy flash slot - migrating to top-of-flash NVM");
    save_motor_offsets_data(*data);
  }

  LOG_INFO("Motor offsets loaded from flash successfully!");
  LOG_DEBUG("Joint type: " + String(data->joint_type));
  LOG_DEBUG("DOF count: " + String(data->dof_count));

  return true;
}
