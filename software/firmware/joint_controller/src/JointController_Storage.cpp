/**
 * @file JointController_Storage.cpp
 * @brief Flash storage implementation for JointController
 * 
 * This file contains the implementation of JointController methods related to:
 * - Loading/saving PID parameters to/from flash memory
 * - Loading/saving linear equations to/from flash memory
 * - Data validation and compatibility checks
 * 
 * These methods are part of the JointController class but separated here
 * for better code organization and maintainability.
 */

#include <JointController.h>
#include <debug.h>
#include <utils.h>
#include <algorithm>
#include "main_common.h"  // For shared_dof_angles

// ============================================================================
// FLASH STORAGE
// ============================================================================

// Load only PID parameters from flash (new lightweight system)
bool JointController::loadPIDDataFromFlash() {
  LOG_C1_INFO("Loading PID parameters from flash...");

  // Reset PIDs to default values before attempting load
  applyDefaultPidTunings(false);

  PIDOnlyDeviceData pid_data;
  if (!load_pid_only_data(&pid_data)) {
    LOG_C1_INFO("No PID data found in flash — using default values (kp=" +
             String(DEFAULT_INNER_LOOP_KP, 2) + ", ki=" + String(DEFAULT_INNER_LOOP_KI, 2) +
             ", kd=" + String(DEFAULT_INNER_LOOP_KD, 2) + ")");
    return false;
  }

  // Verify joint type matches
  if (pid_data.joint_type != config.joint_id) {
    LOG_C1_WARN("Joint type in flash (" + String(pid_data.joint_type) +
             ") differs from configuration (" + String(config.joint_id) + ")");
    // Continue anyway, might be compatible
  }

  // Verify DOF and motor counts match
  if (pid_data.dof_count != config.dof_count) {
    LOG_C1_ERROR("DOF count in flash (" + String(pid_data.dof_count) +
              ") differs from configuration (" + String(config.dof_count) + ")");
    LOG_C1_INFO("Applying default PID values for safety");
    return false;
  }

  if (pid_data.motor_count != config.motor_count) {
    LOG_C1_ERROR("Motor count in flash (" + String(pid_data.motor_count) +
              ") differs from configuration (" + String(config.motor_count) + ")");
    LOG_C1_INFO("Applying default PID values for safety");
    return false;
  }

  // Load PID data for each motor with safety checks
  LOG_C1_INFO("Applying PID parameters from flash...");
  bool pid_loading_enabled = true; // Flag to disable PID loading if necessary

  if (pid_loading_enabled) {
    for (int i = 0; i < config.motor_count && i < MAX_MOTORS; i++) {
      if (pid_data.pid_data[i].kp != 0) { // Only if there is valid PID data
        // Update PID parameters with safety checks
        if (pid_controllers[i] != nullptr) {
          // Validate PID parameters before applying
          float kp = pid_data.pid_data[i].kp;
          float ki = pid_data.pid_data[i].ki;
          float kd = pid_data.pid_data[i].kd;

          // Sanity checks on PID parameters
          if (isnan(kp) || isnan(ki) || isnan(kd) || kp < 0 || ki < 0 || kd < 0 || kp > 1000 ||
              ki > 1000 || kd > 1000) {
            LOG_C1_WARN("Invalid PID parameters for motor " + String(i) +
                     " — using configuration parameters");
            continue; // Skip this motor and use original parameters
          }

          // Get original tau safely
          float tau_original = config.motors[i].pid.tau;

          // Apply PID parameters safely
          pid_controllers[i]->setTunings(kp, ki, kd, tau_original);

          LOG_C1_INFO("PID loaded for motor " + String(i) + ": kp=" + String(kp, 4) +
                   ", ki=" + String(ki, 4) + ", kd=" + String(kd, 4) + ", tau=" +
                   String(tau_original, 4));
        }
      }
    }
  } else {
    LOG_C1_WARN("PID loading disabled - using configuration values");
  }

  LOG_C1_INFO("PID parameters loaded");

  // Load outer loop parameters
  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    float kp            = pid_data.outer_loop_kp[i];
    float ki            = pid_data.outer_loop_ki[i];
    float kd            = pid_data.outer_loop_kd[i];
    float stiffness_deg = pid_data.stiffness_ref_deg[i];
    float cascade       = pid_data.cascade_influence[i];

    if (!std::isfinite(kp) || kp <= 0.0f)
      kp = DEFAULT_OUTER_LOOP_KP;
    if (!std::isfinite(ki) || ki < 0.0f)
      ki = DEFAULT_OUTER_LOOP_KI;
    if (!std::isfinite(kd) || kd < 0.0f)
      kd = DEFAULT_OUTER_LOOP_KD;
    if (!std::isfinite(stiffness_deg) || stiffness_deg < 0.0f)
      stiffness_deg = DEFAULT_STIFFNESS_REF_DEG;
    if (!std::isfinite(cascade))
      cascade = DEFAULT_CASCADE_INFLUENCE;

    cascade = std::clamp(cascade, 0.0f, 1.0f);

    outer_loop_kp_values[i]     = kp;
    outer_loop_ki_values[i]     = ki;
    outer_loop_kd_values[i]     = kd;
    stiffness_ref_values[i]     = stiffness_deg;
    cascade_influence_values[i] = cascade;

    LOG_C1_INFO("Outer loop DOF " + String(i) + ": Kp=" + String(kp, 4) + ", Ki=" + String(ki, 4) +
             ", Kd=" + String(kd, 4));
    LOG_C1_INFO("  Stiffness=" + String(stiffness_deg, 4) + " deg, cascade influence=" +
             String(cascade * 100.0f, 1) + "%");
  }

  return true;
}

// Save only PID parameters to flash (new lightweight system)
bool JointController::savePIDDataToFlash() {
  LOG_C1_INFO("Saving PID parameters to flash...");

  PIDOnlyDeviceData pid_data = {};
  pid_data.joint_type        = config.joint_id;
  pid_data.dof_count         = config.dof_count;
  pid_data.motor_count       = config.motor_count;

  // Copy current PID parameters for all motors
  for (int i = 0; i < config.motor_count && i < MAX_MOTORS; i++) {
    if (pid_controllers[i] != nullptr) {
      pid_data.pid_data[i] = {pid_controllers[i]->getKp(), pid_controllers[i]->getKi(),
                              pid_controllers[i]->getKd()};
    } else {
      // Use configuration parameters if the controller is not available
      pid_data.pid_data[i] = {config.motors[i].pid.kp, config.motors[i].pid.ki,
                              config.motors[i].pid.kd};
    }
  }

  for (int i = 0; i < MAX_DOFS; i++) {
    if (i < config.dof_count) {
      pid_data.outer_loop_kp[i]     = outer_loop_kp_values[i];
      pid_data.outer_loop_ki[i]     = outer_loop_ki_values[i];
      pid_data.outer_loop_kd[i]     = outer_loop_kd_values[i];
      pid_data.stiffness_ref_deg[i] = stiffness_ref_values[i];
      pid_data.cascade_influence[i] = cascade_influence_values[i];
    } else {
      pid_data.outer_loop_kp[i]     = DEFAULT_OUTER_LOOP_KP;
      pid_data.outer_loop_ki[i]     = DEFAULT_OUTER_LOOP_KI;
      pid_data.outer_loop_kd[i]     = DEFAULT_OUTER_LOOP_KD;
      pid_data.stiffness_ref_deg[i] = DEFAULT_STIFFNESS_REF_DEG;
      pid_data.cascade_influence[i] = DEFAULT_CASCADE_INFLUENCE;
    }
  }

  // Save to flash
  save_pid_only_data(pid_data);

  LOG_C1_INFO("PID parameters saved to flash");
  return true;
}

// NEW: Save linear equations to flash
bool JointController::saveLinearEquationsToFlash() {
  LOG_C1_INFO("Saving linear equations to flash...");

  LinearEquationsDeviceData equations_data = {};
  equations_data.joint_type                = config.joint_id;
  equations_data.dof_count                 = config.dof_count;
  equations_data.motor_count               = config.motor_count;

  // Copy current linear equations for all DOFs
  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (linear_equations[i].calculated && linear_equations[i].limits_valid) {
      equations_data.dof_equations[i].calculated = 1;

      // Copy agonist data
      if (linear_equations[i].agonist.valid) {
        equations_data.dof_equations[i].agonist.slope     = linear_equations[i].agonist.slope;
        equations_data.dof_equations[i].agonist.intercept = linear_equations[i].agonist.intercept;
        equations_data.dof_equations[i].agonist.r_squared = linear_equations[i].agonist.r_squared;
        equations_data.dof_equations[i].agonist.mse       = linear_equations[i].agonist.mse;
        equations_data.dof_equations[i].agonist.valid     = 1;
      } else {
        equations_data.dof_equations[i].agonist.valid = 0;
      }

      // Copy antagonist data
      if (linear_equations[i].antagonist.valid) {
        equations_data.dof_equations[i].antagonist.slope = linear_equations[i].antagonist.slope;
        equations_data.dof_equations[i].antagonist.intercept =
            linear_equations[i].antagonist.intercept;
        equations_data.dof_equations[i].antagonist.r_squared =
            linear_equations[i].antagonist.r_squared;
        equations_data.dof_equations[i].antagonist.mse   = linear_equations[i].antagonist.mse;
        equations_data.dof_equations[i].antagonist.valid = 1;
      } else {
        equations_data.dof_equations[i].antagonist.valid = 0;
      }

      if (linear_equations[i].limits_valid) {
        equations_data.dof_equations[i].joint_safe_min   = linear_equations[i].joint_safe_min;
        equations_data.dof_equations[i].joint_safe_max   = linear_equations[i].joint_safe_max;
        equations_data.dof_equations[i].agonist_safe_min = linear_equations[i].agonist_safe_min;
        equations_data.dof_equations[i].agonist_safe_max = linear_equations[i].agonist_safe_max;
        equations_data.dof_equations[i].antagonist_safe_min =
            linear_equations[i].antagonist_safe_min;
        equations_data.dof_equations[i].antagonist_safe_max =
            linear_equations[i].antagonist_safe_max;
      } else {
        equations_data.dof_equations[i].joint_safe_min      = 0.0f;
        equations_data.dof_equations[i].joint_safe_max      = 0.0f;
        equations_data.dof_equations[i].agonist_safe_min    = 0.0f;
        equations_data.dof_equations[i].agonist_safe_max    = 0.0f;
        equations_data.dof_equations[i].antagonist_safe_min = 0.0f;
        equations_data.dof_equations[i].antagonist_safe_max = 0.0f;
      }
    } else {
      equations_data.dof_equations[i].calculated          = 0;
      equations_data.dof_equations[i].agonist.valid       = 0;
      equations_data.dof_equations[i].antagonist.valid    = 0;
      equations_data.dof_equations[i].joint_safe_min      = 0.0f;
      equations_data.dof_equations[i].joint_safe_max      = 0.0f;
      equations_data.dof_equations[i].agonist_safe_min    = 0.0f;
      equations_data.dof_equations[i].agonist_safe_max    = 0.0f;
      equations_data.dof_equations[i].antagonist_safe_min = 0.0f;
      equations_data.dof_equations[i].antagonist_safe_max = 0.0f;
    }
  }

  // Save to flash
  save_linear_equations_data(equations_data);

  LOG_C1_INFO("Linear equations saved to flash");
  return true;
}

// NEW: Load linear equations from flash
bool JointController::loadLinearEquationsFromFlash() {
  LOG_C1_INFO("Loading linear equations from flash...");

  LinearEquationsDeviceData equations_data;
  if (!load_linear_equations_data(&equations_data)) {
    LOG_C1_WARN("No linear equations found in flash - run auto-mapping");
    return false;
  }

  // Verify joint type matches
  if (equations_data.joint_type != config.joint_id) {
    LOG_C1_WARN("Joint type in flash (" + String(equations_data.joint_type) +
             ") differs from configuration (" + String(config.joint_id) + ")");
    // Continue anyway, might be compatible
  }

  // Verify DOF and motor counts match
  if (equations_data.dof_count != config.dof_count) {
    LOG_C1_ERROR("DOF count in flash (" + String(equations_data.dof_count) +
              ") differs from configuration (" + String(config.dof_count) + ")");
    return false;
  }

  if (equations_data.motor_count != config.motor_count) {
    LOG_C1_ERROR("Motor count in flash (" + String(equations_data.motor_count) +
              ") differs from configuration (" + String(config.motor_count) + ")");
    return false;
  }

  // Load equations for each DOF
  LOG_C1_INFO("Loading linear equations from flash...");
  int loaded_equations_count = 0;

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (equations_data.dof_equations[i].calculated) {
      linear_equations[i].calculated = true;
      linear_equations[i].dof_index  = i;

      linear_equations[i].joint_safe_min      = equations_data.dof_equations[i].joint_safe_min;
      linear_equations[i].joint_safe_max      = equations_data.dof_equations[i].joint_safe_max;
      linear_equations[i].agonist_safe_min    = equations_data.dof_equations[i].agonist_safe_min;
      linear_equations[i].agonist_safe_max    = equations_data.dof_equations[i].agonist_safe_max;
      linear_equations[i].antagonist_safe_min = equations_data.dof_equations[i].antagonist_safe_min;
      linear_equations[i].antagonist_safe_max = equations_data.dof_equations[i].antagonist_safe_max;
      linear_equations[i].limits_valid        = equations_data.dof_equations[i].agonist.valid &&
                                         equations_data.dof_equations[i].antagonist.valid;

      // Load agonist data
      if (equations_data.dof_equations[i].agonist.valid) {
        linear_equations[i].agonist.slope       = equations_data.dof_equations[i].agonist.slope;
        linear_equations[i].agonist.intercept   = equations_data.dof_equations[i].agonist.intercept;
        linear_equations[i].agonist.r_squared   = equations_data.dof_equations[i].agonist.r_squared;
        linear_equations[i].agonist.mse         = equations_data.dof_equations[i].agonist.mse;
        linear_equations[i].agonist.data_points = 0; // Not saved to flash
        linear_equations[i].agonist.valid       = true;
        loaded_equations_count++;

        LOG_C1_INFO("DOF " + String(i) + " agonist: y = " +
                 String(linear_equations[i].agonist.slope, 4) + "*x + " +
                 String(linear_equations[i].agonist.intercept, 4) + " (R^2=" +
                 String(linear_equations[i].agonist.r_squared, 3) + ")");
      } else {
        linear_equations[i].agonist.valid = false;
      }

      // Load antagonist data
      if (equations_data.dof_equations[i].antagonist.valid) {
        linear_equations[i].antagonist.slope = equations_data.dof_equations[i].antagonist.slope;
        linear_equations[i].antagonist.intercept =
            equations_data.dof_equations[i].antagonist.intercept;
        linear_equations[i].antagonist.r_squared =
            equations_data.dof_equations[i].antagonist.r_squared;
        linear_equations[i].antagonist.mse         = equations_data.dof_equations[i].antagonist.mse;
        linear_equations[i].antagonist.data_points = 0; // Not saved to flash
        linear_equations[i].antagonist.valid       = true;
        loaded_equations_count++;

        LOG_C1_INFO("DOF " + String(i) + " antagonist: y = " +
                 String(linear_equations[i].antagonist.slope, 4) + "*x + " +
                 String(linear_equations[i].antagonist.intercept, 4) + " (R^2=" +
                 String(linear_equations[i].antagonist.r_squared, 3) + ")");
      } else {
        linear_equations[i].antagonist.valid = false;
      }

      if (linear_equations[i].limits_valid) {
        LOG_C1_INFO("  Joint limits: [" + String(linear_equations[i].joint_safe_min, 2) + " deg, " +
                 String(linear_equations[i].joint_safe_max, 2) + " deg]");
        LOG_C1_INFO("  Agonist limits: [" + String(linear_equations[i].agonist_safe_min, 2) + " deg, " +
                 String(linear_equations[i].agonist_safe_max, 2) + " deg]");
        LOG_C1_INFO("  Antagonist limits: [" + String(linear_equations[i].antagonist_safe_min, 2) +
                 " deg, " + String(linear_equations[i].antagonist_safe_max, 2) + " deg]");
      } else {
        LOG_C1_WARN("  Safety limits unavailable - rerun auto-mapping");
      }
    } else {
      linear_equations[i].calculated       = false;
      linear_equations[i].agonist.valid    = false;
      linear_equations[i].antagonist.valid = false;
      linear_equations[i].limits_valid     = false;
    }
  }

  LOG_C1_INFO("Loaded " + String(loaded_equations_count) + " linear equations from flash");
  LOG_C1_INFO("Compact linear equations loaded - system ready for precise control");

  return loaded_equations_count > 0;
}

// ============================================================================
// MOTOR OFFSETS FLASH STORAGE
// ============================================================================

bool JointController::saveMotorOffsetsToFlash() {
  LOG_C1_INFO("Saving motor offsets to flash...");

  MotorOffsetsDeviceData offsets_data = {};
  offsets_data.joint_type = config.joint_id;
  offsets_data.dof_count  = config.dof_count;

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (_saved_offsets[i].valid) {
      offsets_data.dof_offsets[i].agonist_offset       = _saved_offsets[i].agonist_offset;
      offsets_data.dof_offsets[i].antagonist_offset     = _saved_offsets[i].antagonist_offset;
      offsets_data.dof_offsets[i].joint_angle_at_calib  = _saved_offsets[i].joint_angle_at_calib;
    }
  }

  save_motor_offsets_data(offsets_data);
  LOG_C1_INFO("Motor offsets saved to flash");
  return true;
}

bool JointController::loadMotorOffsetsFromFlash() {
  LOG_C1_INFO("Loading motor offsets from flash...");

  MotorOffsetsDeviceData offsets_data;
  if (!load_motor_offsets_data(&offsets_data)) {
    LOG_C1_INFO("No motor offsets found in flash");
    return false;
  }

  if (offsets_data.joint_type != config.joint_id) {
    LOG_C1_WARN("Joint type mismatch in motor offsets flash data");
    return false;
  }

  if (offsets_data.dof_count != config.dof_count) {
    LOG_C1_ERROR("DOF count mismatch in motor offsets flash data");
    return false;
  }

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    _saved_offsets[i].agonist_offset       = offsets_data.dof_offsets[i].agonist_offset;
    _saved_offsets[i].antagonist_offset     = offsets_data.dof_offsets[i].antagonist_offset;
    _saved_offsets[i].joint_angle_at_calib  = offsets_data.dof_offsets[i].joint_angle_at_calib;
    _saved_offsets[i].valid                 = true;

    LOG_C1_INFO("DOF " + String(i) + " saved offsets: agon=" +
             String(_saved_offsets[i].agonist_offset, 2) + " antag=" +
             String(_saved_offsets[i].antagonist_offset, 2) + " joint=" +
             String(_saved_offsets[i].joint_angle_at_calib, 2) + "°");
  }

  LOG_C1_INFO("Motor offsets loaded from flash successfully");
  return true;
}

JointController::OffsetValidationResult JointController::validateSavedOffsets(uint8_t dof_index) {
  OffsetValidationResult result = {false, 0.0f, 0.0f, false};

  if (dof_index >= config.dof_count) return result;

  // Check if we have saved offsets for this DOF
  if (!_saved_offsets[dof_index].valid) {
    LOG_C1_INFO("No saved offsets for DOF " + String(dof_index));
    return result;  // has_saved_data = false, valid = false
  }
  result.has_saved_data = true;

  // Check if linear equations are available (needed for expected angle calculation)
  if (!hasValidEquations(dof_index)) {
    LOG_C1_WARN("No linear equations for DOF " + String(dof_index) + " - cannot validate offsets");
    return result;
  }

  // Find agonist and antagonist motors for this DOF
  LKM_Motor *agonist_motor = nullptr;
  LKM_Motor *antagonist_motor = nullptr;

  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      if (config.motors[i].is_agonist) {
        agonist_motor = motors[i];
      } else {
        antagonist_motor = motors[i];
      }
    }
  }

  if (agonist_motor == nullptr || antagonist_motor == nullptr) {
    LOG_C1_ERROR("Motors not found for DOF " + String(dof_index));
    return result;
  }

  // Read raw motor angles (no offset applied) via CAN
  float raw_agonist = agonist_motor->getMultiAngleSync(false).angle;
  float raw_antagonist = antagonist_motor->getMultiAngleSync(false).angle;

  if (isnan(raw_agonist) || isnan(raw_antagonist)) {
    LOG_C1_ERROR("CAN timeout reading motors for DOF " + String(dof_index));
    return result;
  }

  // Read current joint angle (MT6835 absolute encoder - always valid)
  if (!shared_dof_angles.valid[dof_index]) {
    LOG_C1_ERROR("Encoder not valid for DOF " + String(dof_index));
    return result;
  }
  float joint_angle = shared_dof_angles.angles[dof_index];

  // Calculate expected motor angles using linear equations
  float expected_agonist, expected_antagonist;
  if (!calculateMotorAnglesWithEquations(dof_index, joint_angle, joint_angle,
                                          expected_agonist, expected_antagonist)) {
    LOG_C1_ERROR("Failed to calculate expected angles for DOF " + String(dof_index));
    return result;
  }

  // Apply saved offsets to raw angles and compare with expected
  // The offset relationship depends on invert_logic (same as recalculateMotorOffsets)
  bool invert_logic = config.dofs[dof_index].zero_mapping.auto_mapping_invert_direction;

  float calibrated_agonist, calibrated_antagonist;
  if (invert_logic) {
    // Inverted: raw + offset = calibrated
    calibrated_agonist    = raw_agonist + _saved_offsets[dof_index].agonist_offset;
    calibrated_antagonist = raw_antagonist + _saved_offsets[dof_index].antagonist_offset;
  } else {
    // Standard: raw - offset = calibrated
    calibrated_agonist    = raw_agonist - _saved_offsets[dof_index].agonist_offset;
    calibrated_antagonist = raw_antagonist - _saved_offsets[dof_index].antagonist_offset;
  }

  result.error_agonist_deg    = fabs(calibrated_agonist - expected_agonist);
  result.error_antagonist_deg = fabs(calibrated_antagonist - expected_antagonist);

  // Threshold: < 5° means motors kept power and offsets are valid
  // Typical values: < 1° if motors kept power, > 50° if they lost power
  const float OFFSET_VALID_THRESHOLD = 5.0f;
  result.valid = (result.error_agonist_deg < OFFSET_VALID_THRESHOLD &&
                  result.error_antagonist_deg < OFFSET_VALID_THRESHOLD);

  LOG_C1_INFO("DOF " + String(dof_index) + " offset validation: " +
           String(result.valid ? "VALID" : "NEEDS RECALC") +
           " (agon_err=" + String(result.error_agonist_deg, 2) +
           "° antag_err=" + String(result.error_antagonist_deg, 2) + "°)");

  return result;
}

JointController::OffsetValidationResult JointController::checkOffsetDriftFromCache(uint8_t dof_index) {
  OffsetValidationResult result = {false, 0.0f, 0.0f, false};

  if (dof_index >= config.dof_count) return result;

  // Need valid equations to calculate expected angles
  if (!hasValidEquations(dof_index)) return result;

  // Need valid cached motor angles (updated every control cycle by executeControlLoop)
  if (!cached_motor_angles.valid[dof_index]) return result;

  // Need valid joint encoder reading
  if (!shared_dof_angles.valid[dof_index]) return result;

  result.has_saved_data = true;  // We have live data to compare

  float joint_angle = shared_dof_angles.angles[dof_index];

  // Calculate expected motor angles from linear equations
  float expected_agonist, expected_antagonist;
  if (!calculateMotorAnglesWithEquations(dof_index, joint_angle, joint_angle,
                                          expected_agonist, expected_antagonist)) {
    return result;
  }

  // Cached motor angles are already calibrated (offset applied by the control loop)
  float actual_agonist = cached_motor_angles.agonist[dof_index];
  float actual_antagonist = cached_motor_angles.antagonist[dof_index];

  result.error_agonist_deg = fabs(actual_agonist - expected_agonist);
  result.error_antagonist_deg = fabs(actual_antagonist - expected_antagonist);

  const float DRIFT_THRESHOLD = 2.0f;
  result.valid = (result.error_agonist_deg < DRIFT_THRESHOLD &&
                  result.error_antagonist_deg < DRIFT_THRESHOLD);

  return result;
}

// Recalculate safe limits based on current equations and physical limits
bool JointController::recalculateSafeLimits() {
  LOG_C1_INFO("Recalculating safe limits from current equations...");
  
  int recalculated_count = 0;
  
  for (int dof = 0; dof < config.dof_count; dof++) {
    if (!linear_equations[dof].calculated || !linear_equations[dof].agonist.valid || 
        !linear_equations[dof].antagonist.valid) {
      LOG_C1_WARN("DOF " + String(dof) + ": No valid equations - skipping");
      continue;
    }
    
    // Get current mapping range from stored limits (before recalc)
    // We need to derive the original mapping range from the motor limits
    // Since motor limits are stored with MOTOR_SAFETY_MARGIN (20°), we can work backwards
    float old_joint_min = linear_equations[dof].joint_safe_min;
    float old_joint_max = linear_equations[dof].joint_safe_max;
    
    // Estimate mapping range from old safe limits (approximate)
    // The mapping range was approximately the safe range before the margin was removed
    float joint_mapping_min = old_joint_min;
    float joint_mapping_max = old_joint_max;
    
    // Calculate new safe limits using the updated algorithm (no margin on physical limits)
    const float MAPPING_EXTENSION_RATIO = 0.10f;
    
    float joint_range = joint_mapping_max - joint_mapping_min;
    float joint_extension = joint_range * MAPPING_EXTENSION_RATIO;
    float extended_joint_min = joint_mapping_min - joint_extension;
    float extended_joint_max = joint_mapping_max + joint_extension;
    
    // Use physical limits directly from config (no margin)
    float physical_min = config.dofs[dof].limits.min_angle;
    float physical_max = config.dofs[dof].limits.max_angle;
    
    // Safe range = intersection of extended mapping and physical limits
    float new_safe_min = max(extended_joint_min, physical_min);
    float new_safe_max = min(extended_joint_max, physical_max);
    
    if (new_safe_min > new_safe_max) {
      // If inverted, fall back to physical limits
      new_safe_min = physical_min;
      new_safe_max = physical_max;
    }
    
    LOG_C1_INFO("DOF " + String(dof) + ": Safe limits updated:");
    LOG_C1_INFO("  Old: [" + String(old_joint_min, 2) + "°, " + String(old_joint_max, 2) + "°]");
    LOG_C1_INFO("  New: [" + String(new_safe_min, 2) + "°, " + String(new_safe_max, 2) + "°]");
    LOG_C1_INFO("  Physical: [" + String(physical_min, 2) + "°, " + String(physical_max, 2) + "°]");
    
    linear_equations[dof].joint_safe_min = new_safe_min;
    linear_equations[dof].joint_safe_max = new_safe_max;
    linear_equations[dof].limits_valid = true;
    
    recalculated_count++;
  }
  
  if (recalculated_count == 0) {
    LOG_C1_ERROR("No equations available to recalculate limits");
    return false;
  }
  
  // Save updated equations to flash
  LOG_C1_INFO("Saving recalculated limits to flash...");
  if (saveLinearEquationsToFlash()) {
    LOG_C1_INFO("Safe limits recalculated and saved successfully!");
    return true;
  } else {
    LOG_C1_ERROR("Failed to save recalculated limits to flash");
    return false;
  }
}
