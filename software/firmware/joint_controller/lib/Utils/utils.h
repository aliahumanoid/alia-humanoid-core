/**
 * @file utils.h
 * @brief Utility functions for flash storage, timing, and interpolation
 * 
 * This module provides essential utility functions for the joint controller:
 * 
 * Flash Storage:
 * - Persistent storage of PID parameters, calibration equations, system settings,
 *   and motor offsets
 * - Current records live in dedicated top-of-flash sectors defined in flash_map.h
 * - Legacy offsets remain readable for one-time migration after reflashing
 * - Versioning and checksum validation for data integrity
 * - Automatic recovery from corrupted data
 * 
 * Flash Memory Layout:
 * - PID slot: FLASH_PID_OFFSET
 * - Linear equations slot: FLASH_LINEAR_EQ_OFFSET
 * - System settings slot: FLASH_SYSTEM_SETTINGS_OFFSET
 * - Motor offsets slot: FLASH_MOTOR_OFFSETS_OFFSET
 * - See flash_map.h for the authoritative layout contract
 * 
 * Data Versioning:
 * - Version 4: PID-only data (motor PID gains, outer loop PID)
 * - Version 5: Linear equations (motor-joint mapping, safety limits)
 * 
 * Time Management:
 * - High-resolution timing with overflow handling
 * - Supports continuous operation beyond 2^32 microseconds (~71 minutes)
 * 
 * Interpolation:
 * - Linear interpolation for motor-joint angle mapping
 * - Handles both increasing and decreasing data sequences
 */

#ifndef UTILS_H
#define UTILS_H

#include <global.h>
#include <flash_map.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string.h>

// ===================================================================
// FLASH STORAGE - PID DATA
// ===================================================================

/**
 * @brief Save PID configuration to flash memory
 * 
 * Stores compact PID-only data (no mapping/calibration) to dedicated
 * flash region. Includes motor PID gains and outer loop parameters.
 * 
 * Data structure:
 * - Magic number for validation (0x41534D50)
 * - Version 4 format
 * - Joint type, DOF count, motor count
 * - PID gains for all motors
 * - Outer loop PID parameters
 * - Checksum for integrity verification
 * 
 * @param data PID configuration structure to save
 * 
 * @note This operation erases and writes flash sectors. Do not call
 *       during time-critical operations. Takes ~100ms to complete.
 */
void save_pid_only_data(struct PIDOnlyDeviceData data);

/**
 * @brief Load PID configuration from flash memory
 * 
 * Reads and validates PID-only data from flash. Performs multiple
 * validation steps:
 * - Magic number verification
 * - Version compatibility check
 * - Checksum validation
 * - Range validation (DOF/motor counts)
 * 
 * @param[out] data Pointer to structure where loaded data will be stored
 * @return true if data loaded successfully, false if not found or invalid
 * 
 * @note Prints detailed diagnostic messages to Serial for debugging
 */
bool load_pid_only_data(struct PIDOnlyDeviceData *data);

// ===================================================================
// FLASH STORAGE - LINEAR EQUATIONS (CALIBRATION)
// ===================================================================

/**
 * @brief Save linear calibration equations to flash memory
 * 
 * Stores motor-joint mapping equations derived from auto-calibration.
 * Includes linear regression coefficients (slope, intercept) and
 * safety limits for each DOF.
 * 
 * Data structure:
 * - Magic number and version 5
 * - Joint configuration metadata
 * - Agonist/antagonist motor equations for each DOF
 * - R² and MSE statistics for equation quality
 * - Joint angle and motor angle safety limits
 * - Checksum for data integrity
 * 
 * @param data Linear equations structure to save
 * 
 * @note Stored in a dedicated top-of-flash slot separate from PID data
 */
void save_linear_equations_data(struct LinearEquationsDeviceData data);

/**
 * @brief Load linear calibration equations from flash memory
 * 
 * Reads and validates motor-joint mapping equations. Used to restore
 * calibration data after power cycle.
 * 
 * @param[out] data Pointer to structure where loaded equations will be stored
 * @return true if equations loaded successfully, false otherwise
 * 
 * @note Prints equation details (slope, intercept, R², MSE) and limits
 */
bool load_linear_equations_data(struct LinearEquationsDeviceData *data);

// ===================================================================
// FLASH STORAGE - FINE-MAP (PIECEWISE, v8)
// ===================================================================

/**
 * @brief Save the v8 fine-map record into the LINEAR_EQ sector (whole-sector RMW)
 *
 * Preserves any co-located v5 linear-eq record in the same sector.
 *
 * @param fm Fine-map structure to save
 */
void save_fine_map_data(const struct FineMapDeviceDataV8 *fm);

/**
 * @brief Load the v8 fine-map record from flash
 *
 * @param[out] data Pointer to structure where the loaded fine-map will be stored
 * @return true if a valid fine-map blob was found; false on absence/corruption
 */
bool load_fine_map_data(struct FineMapDeviceDataV8 *data);

// ===================================================================
// FLASH STORAGE - FINE-MAP 2D (BILINEAR, v9)
// ===================================================================

/**
 * @brief Save the v9 2D (bilinear) record into the LINEAR_EQ sector (whole-sector RMW)
 *
 * Preserves any co-located v5 linear-eq and v8 fine-map records in the same sector.
 *
 * @param v9 2D (bilinear) grid structure to save
 */
void save_fine_map_2d_data(const struct FineMap2DDeviceDataV9 *v9);

/**
 * @brief Load the v9 2D (bilinear) record from flash
 *
 * @param[out] data Pointer to structure where the loaded 2D grids will be stored
 * @return true if a valid 2D blob was found; false on absence/corruption
 */
bool load_fine_map_2d_data(struct FineMap2DDeviceDataV9 *data);

/**
 * @brief Save v8 fine-map + v9 2D grid in a single whole-sector RMW (see utils.cpp)
 */
void save_fine_map_and_2d(const struct FineMapDeviceDataV8 *v8,
                          const struct FineMap2DDeviceDataV9 *v9);

// ===================================================================
// FLASH STORAGE - SYSTEM SETTINGS
// ===================================================================

/**
 * @brief Save system settings to flash memory
 * 
 * Stores persistent system preferences including:
 * - Auto-start flag for boot-time initialization
 * - Pretension parameters for auto-start sequence
 * 
 * @param data System settings structure to save
 */
void save_system_settings_data(struct SystemSettingsData data);

/**
 * @brief Load system settings from flash memory
 * 
 * Reads and validates system settings. Returns false if no settings found
 * or data is corrupted (system will use default settings).
 * 
 * @param[out] data Pointer to structure where loaded settings will be stored
 * @return true if settings loaded successfully, false if not found or invalid
 */
bool load_system_settings_data(struct SystemSettingsData *data);

// ===================================================================
// FLASH STORAGE - MOTOR OFFSETS
// ===================================================================

/**
 * @brief Save motor encoder offsets to flash memory
 * 
 * Stores offsets calculated by recalculateMotorOffsets() for later
 * validation at boot. Enables skipping recalc when motors kept power.
 * 
 * @param data Motor offsets structure to save
 * 
 * @note Stored in a dedicated top-of-flash slot
 */
void save_motor_offsets_data(struct MotorOffsetsDeviceData data);

/**
 * @brief Load motor encoder offsets from flash memory
 * 
 * Reads and validates saved motor offsets. Used at boot to check
 * whether recalc_offset can be skipped.
 * 
 * @param[out] data Pointer to structure where loaded offsets will be stored
 * @return true if offsets loaded successfully, false if not found or invalid
 */
bool load_motor_offsets_data(struct MotorOffsetsDeviceData *data);

// ===================================================================
// TIME MANAGEMENT
// ===================================================================

/**
 * @brief Get high-resolution time with overflow handling
 * 
 * Returns elapsed time in seconds since system start, with automatic
 * handling of micros() overflow at 2^32 microseconds (~71.6 minutes).
 * 
 * Implementation:
 * - Tracks overflow count
 * - Combines overflow count with current micros()
 * - Returns total time as float in seconds
 * 
 * @return Time in seconds since system start (continuously increasing)
 * 
 * @note Must be called at least once per ~71 minutes to detect overflows
 */
float get_current_time();

// ===================================================================
// INTERPOLATION
// ===================================================================

/**
 * @brief Linear interpolation between two data arrays
 * 
 * Performs 1D linear interpolation to find corresponding value in data2
 * for a given target_value in data1. Handles both increasing and
 * decreasing data sequences.
 * 
 * Algorithm:
 * 1. Determine if data1 is increasing or decreasing
 * 2. Find bracket indices in data1 containing target_value
 * 3. Linearly interpolate corresponding values in data2
 * 4. Clamp to boundaries if target_value is out of range
 * 
 * Use cases:
 * - Motor angle → Joint angle conversion
 * - Joint angle → Motor angle conversion
 * - Time → Position trajectory interpolation
 * - General 1D lookup table interpolation
 * 
 * @param target_value Value to find in data1 (e.g., angle, time, position)
 * @param data1 Input array (independent variable, e.g., angles, timestamps)
 * @param data2 Output array (dependent variable, e.g., positions, values)
 * @param size Length of both arrays
 * @return Interpolated value from data2 corresponding to target_value
 * 
 * @note Returns boundary values if target_value is outside data1 range
 */
float interpolate_data(float target_value, const float *data1, const float *data2, int size);

#endif // UTILS_H
