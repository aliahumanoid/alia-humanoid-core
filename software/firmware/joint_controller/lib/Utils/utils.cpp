/**
 * @file utils.cpp
 * @brief Implementation of utility functions
 * 
 * See utils.h for detailed documentation.
 */

#include "utils.h"
#include "debug.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <Arduino.h>
#include <array>
#include <cstring>

// External flags for flash operation synchronization with Core1
extern volatile bool flash_operation_in_progress;
extern volatile bool core1_flash_acknowledged;

/**
 * @brief Handshake with Core1 before flash operations.
 *
 * Signals Core1 to park in a RAM-resident wait loop, then waits for
 * Core1 to acknowledge.  Replaces the previous delay(5) with a proper
 * bidirectional handshake that is immune to Core1 timing jitter.
 *
 * Timeout (50 ms) covers the case where Core1 is not running yet
 * (boot) or is stuck — the flash operation proceeds anyway.
 */
static void wait_for_core1_flash_ready() {
  // Reset ack from any previous operation (safe: Core1 is not in the
  // wait loop because flash_operation_in_progress is still false)
  core1_flash_acknowledged = false;

  // Signal Core1 to enter RAM wait loop
  flash_operation_in_progress = true;

  // Wait for Core1 to confirm it is safely in RAM
  uint32_t start = millis();
  while (!core1_flash_acknowledged) {
    if (millis() - start > 50) {
      LOG_WARN("[FLASH] Core1 handshake timeout — proceeding");
      break;
    }
    tight_loop_contents();
  }
}

// ===================================================================
// FLASH MEMORY CONFIGURATION
// ===================================================================

// Flash storage layout in SPI flash memory
#define FLASH_STORAGE_BASE_OFFSET (256 * 1024)  ///< Start at 256 KB offset
#define FLASH_PID_OFFSET (FLASH_STORAGE_BASE_OFFSET)  ///< PID data at base
#define FLASH_LINEAR_EQ_OFFSET (FLASH_STORAGE_BASE_OFFSET + 64 * 1024)  ///< Equations at base + 64KB
#define FLASH_SYSTEM_SETTINGS_OFFSET (FLASH_STORAGE_BASE_OFFSET + 128 * 1024)  ///< System settings at base + 128KB
#define FLASH_MOTOR_OFFSETS_OFFSET (FLASH_STORAGE_BASE_OFFSET + 192 * 1024)   ///< Motor offsets at base + 192KB

// Data format versions
static constexpr uint16_t PID_FLASH_VERSION            = 4;  ///< PID-only format version
static constexpr uint16_t LINEAR_EQ_FLASH_VERSION      = 5;  ///< Linear equations format version
static constexpr uint16_t SYSTEM_SETTINGS_FLASH_VERSION = 6;  ///< System settings format version
static constexpr uint16_t MOTOR_OFFSETS_FLASH_VERSION   = 7;  ///< Motor offsets format version

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
  wait_for_core1_flash_ready();
  
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
  flash_operation_in_progress = false;

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

  uint8_t *data_ptr        = (uint8_t *)data;
  size_t data_size         = sizeof(struct PIDOnlyDeviceData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + FLASH_PID_OFFSET);

  // Step 1: Read header for validation
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  // Step 2: Verify magic number
  if (data->magic_number != MAGIC_NUMBER) {
    LOG_DEBUG("No PID data found in flash (magic number not found)");
    return false;
  }

  // Step 3: Verify version (must be PID format, not linear equations)
  if (data->version < PID_FLASH_VERSION || data->version >= LINEAR_EQ_FLASH_VERSION) {
    LOG_WARN("Unexpected PID data version found in flash: " + String(data->version) +
             " (expected " + String(PID_FLASH_VERSION) + ")");
    return false;
  }

  // Step 4: Read the rest of the data in chunks
  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  // Step 5: Verify checksum
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    LOG_ERROR("PID data checksum mismatch - data corrupted!");
    return false;
  }

  // Step 6: Validate data ranges
  if (data->dof_count == 0 || data->dof_count > MAX_DOFS || data->motor_count > MAX_MOTORS) {
    LOG_ERROR("Invalid PID data - DOF or motor count out of range!");
    return false;
  }

  // Success - print loaded data summary
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
float interpolate_data(float target_value, const float *data1, const float *data2, int size) {
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

/**
 * Save linear calibration equations to flash
 * 
 * Similar procedure to PID save but stores motor-joint mapping equations
 * with calibration metadata (R², MSE, safety limits).
 */
void save_linear_equations_data(struct LinearEquationsDeviceData data) {
  // Populate header metadata
  data.magic_number = MAGIC_NUMBER;
  data.version      = LINEAR_EQ_FLASH_VERSION;  // Version with safety limits
  data.timestamp    = millis();                 // Save timestamp

  // Calculate checksum (excludes header)
  data.checksum = calculate_checksum((uint8_t *)&data + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                                     sizeof(LinearEquationsDeviceData) - sizeof(uint32_t) -
                                         sizeof(uint16_t) * 2);

  uint8_t *data_ptr = (uint8_t *)&data;
  size_t data_size  = sizeof(struct LinearEquationsDeviceData);
  size_t offset     = 0;

  // Calculate number of sectors to erase
  size_t num_sectors = (data_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

  // Handshake: wait for Core1 to park in RAM before flash erase/program
  wait_for_core1_flash_ready();
  
  // Atomic flash operation
  uint32_t ints = save_and_disable_interrupts();

  // Erase and program flash
  flash_range_erase(FLASH_LINEAR_EQ_OFFSET, num_sectors * FLASH_SECTOR_SIZE);

  while (offset < data_size) {
    size_t chunk_size =
        (data_size - offset > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : data_size - offset;

    uint8_t flash_page[FLASH_PAGE_SIZE];
    memset(flash_page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(flash_page, data_ptr + offset, chunk_size);

    flash_range_program(FLASH_LINEAR_EQ_OFFSET + offset, flash_page, FLASH_PAGE_SIZE);
    offset += FLASH_PAGE_SIZE;
  }

  restore_interrupts(ints);
  
  // Signal Core1 to resume normal operation
  flash_operation_in_progress = false;

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
  if (!read_linear_equations_blob(FLASH_LINEAR_EQ_OFFSET, "LINEAR_EQ", data, true)) {
    return false;
  }

  // Further validate data
  if (data->dof_count == 0 || data->dof_count > MAX_DOFS || data->motor_count > MAX_MOTORS) {
    SERIAL_COM_LN("Invalid linear equations data - DOF or motor count out of range!");
    return false;
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
  wait_for_core1_flash_ready();

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
  flash_operation_in_progress = false;

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

  uint8_t *data_ptr        = (uint8_t *)data;
  size_t data_size         = sizeof(struct SystemSettingsData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + FLASH_SYSTEM_SETTINGS_OFFSET);

  // Step 1: Read header for validation
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  // Step 2: Verify magic number
  if (data->magic_number != MAGIC_NUMBER) {
    LOG_DEBUG("No system settings found in flash (magic number not found)");
    return false;
  }

  // Step 3: Verify version
  if (data->version != SYSTEM_SETTINGS_FLASH_VERSION) {
    LOG_WARN("Unexpected system settings version: " + String(data->version) +
             " (expected " + String(SYSTEM_SETTINGS_FLASH_VERSION) + ")");
    return false;
  }

  // Step 4: Read the rest of the data
  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  // Step 5: Verify checksum
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    LOG_ERROR("System settings checksum mismatch - data corrupted!");
    return false;
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
  wait_for_core1_flash_ready();

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
  flash_operation_in_progress = false;

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

  uint8_t *data_ptr        = (uint8_t *)data;
  size_t data_size         = sizeof(struct MotorOffsetsDeviceData);
  size_t offset            = 0;
  const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + FLASH_MOTOR_OFFSETS_OFFSET);

  // Step 1: Read header for validation
  memcpy(data_ptr, flash_ptr, sizeof(uint32_t) + sizeof(uint16_t) * 2);

  // Step 2: Verify magic number
  if (data->magic_number != MAGIC_NUMBER) {
    LOG_DEBUG("No motor offsets found in flash (magic number not found)");
    return false;
  }

  // Step 3: Verify version
  if (data->version != MOTOR_OFFSETS_FLASH_VERSION) {
    LOG_WARN("Unexpected motor offsets version: " + String(data->version) +
             " (expected " + String(MOTOR_OFFSETS_FLASH_VERSION) + ")");
    return false;
  }

  // Step 4: Read the rest of the data
  while (offset < data_size) {
    size_t chunk_size = (data_size - offset > 256) ? 256 : data_size - offset;
    memcpy(data_ptr + offset, flash_ptr + offset, chunk_size);
    offset += chunk_size;
  }

  // Step 5: Verify checksum
  uint16_t calculated_checksum =
      calculate_checksum(data_ptr + sizeof(uint32_t) + sizeof(uint16_t) * 2,
                         data_size - sizeof(uint32_t) - sizeof(uint16_t) * 2);

  if (calculated_checksum != data->checksum) {
    LOG_ERROR("Motor offsets checksum mismatch - data corrupted!");
    return false;
  }

  LOG_INFO("Motor offsets loaded from flash successfully!");
  LOG_DEBUG("Joint type: " + String(data->joint_type));
  LOG_DEBUG("DOF count: " + String(data->dof_count));

  return true;
}
