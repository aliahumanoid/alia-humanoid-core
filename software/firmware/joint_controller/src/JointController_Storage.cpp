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

// ============================================================================
// FLASH STORAGE
// ============================================================================

// Load only PID parameters from flash (new lightweight system)
bool JointController::loadPIDDataFromFlash() {
  LOG_INFO("Loading PID parameters from flash...");

  // Reset PIDs to default values before attempting load
  applyDefaultPidTunings(false);

  PIDOnlyDeviceData pid_data;
  if (!load_pid_only_data(&pid_data)) {
    LOG_INFO("No PID data found in flash — using default values (kp=" +
             String(DEFAULT_INNER_LOOP_KP, 2) + ", ki=" + String(DEFAULT_INNER_LOOP_KI, 2) +
             ", kd=" + String(DEFAULT_INNER_LOOP_KD, 2) + ")");
    return false;
  }

  // Verify joint type matches
  if (pid_data.joint_type != config.joint_id) {
    LOG_WARN("Joint type in flash (" + String(pid_data.joint_type) +
             ") differs from configuration (" + String(config.joint_id) + ")");
    // Continue anyway, might be compatible
  }

  // Verify DOF and motor counts match
  if (pid_data.dof_count != config.dof_count) {
    LOG_ERROR("DOF count in flash (" + String(pid_data.dof_count) +
              ") differs from configuration (" + String(config.dof_count) + ")");
    LOG_INFO("Applying default PID values for safety");
    return false;
  }

  if (pid_data.motor_count != config.motor_count) {
    LOG_ERROR("Motor count in flash (" + String(pid_data.motor_count) +
              ") differs from configuration (" + String(config.motor_count) + ")");
    LOG_INFO("Applying default PID values for safety");
    return false;
  }

  // Load PID data for each motor with safety checks
  LOG_INFO("Applying PID parameters from flash...");
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
            LOG_WARN("Invalid PID parameters for motor " + String(i) +
                     " — using configuration parameters");
            continue; // Skip this motor and use original parameters
          }

          // Get original tau safely
          float tau_original = config.motors[i].pid.tau;

          // Apply PID parameters safely
          pid_controllers[i]->setTunings(kp, ki, kd, tau_original);

          LOG_INFO("PID loaded for motor " + String(i) + ": kp=" + String(kp, 4) +
                   ", ki=" + String(ki, 4) + ", kd=" + String(kd, 4) + ", tau=" +
                   String(tau_original, 4));
        }
      }
    }
  } else {
    LOG_WARN("PID loading disabled - using configuration values");
  }

  LOG_INFO("PID parameters loaded");

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

    LOG_INFO("Outer loop DOF " + String(i) + ": Kp=" + String(kp, 4) + ", Ki=" + String(ki, 4) +
             ", Kd=" + String(kd, 4));
    LOG_INFO("  Stiffness=" + String(stiffness_deg, 4) + " deg, cascade influence=" +
             String(cascade * 100.0f, 1) + "%");
  }

  return true;
}

// Save only PID parameters to flash (new lightweight system)
bool JointController::savePIDDataToFlash() {
  LOG_INFO("Saving PID parameters to flash...");

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

  LOG_INFO("PID parameters saved to flash");
  return true;
}

// NEW: Save linear equations to flash
bool JointController::saveLinearEquationsToFlash() {
  LOG_INFO("Saving linear equations to flash...");

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

  LOG_INFO("Linear equations saved to flash");
  return true;
}

// NEW: Load linear equations from flash
bool JointController::loadLinearEquationsFromFlash() {
  LOG_INFO("Loading linear equations from flash...");

  LinearEquationsDeviceData equations_data;
  if (!load_linear_equations_data(&equations_data)) {
    LOG_WARN("No linear equations found in flash - run auto-mapping");
    return false;
  }

  // Verify joint type matches
  if (equations_data.joint_type != config.joint_id) {
    LOG_WARN("Joint type in flash (" + String(equations_data.joint_type) +
             ") differs from configuration (" + String(config.joint_id) + ")");
    // Continue anyway, might be compatible
  }

  // Verify DOF and motor counts match
  if (equations_data.dof_count != config.dof_count) {
    LOG_ERROR("DOF count in flash (" + String(equations_data.dof_count) +
              ") differs from configuration (" + String(config.dof_count) + ")");
    return false;
  }

  if (equations_data.motor_count != config.motor_count) {
    LOG_ERROR("Motor count in flash (" + String(equations_data.motor_count) +
              ") differs from configuration (" + String(config.motor_count) + ")");
    return false;
  }

  // Load equations for each DOF
  LOG_INFO("Loading linear equations from flash...");
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

        LOG_INFO("DOF " + String(i) + " agonist: y = " +
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

        LOG_INFO("DOF " + String(i) + " antagonist: y = " +
                 String(linear_equations[i].antagonist.slope, 4) + "*x + " +
                 String(linear_equations[i].antagonist.intercept, 4) + " (R^2=" +
                 String(linear_equations[i].antagonist.r_squared, 3) + ")");
      } else {
        linear_equations[i].antagonist.valid = false;
      }

      if (linear_equations[i].limits_valid) {
        LOG_INFO("  Joint limits: [" + String(linear_equations[i].joint_safe_min, 2) + " deg, " +
                 String(linear_equations[i].joint_safe_max, 2) + " deg]");
        LOG_INFO("  Agonist limits: [" + String(linear_equations[i].agonist_safe_min, 2) + " deg, " +
                 String(linear_equations[i].agonist_safe_max, 2) + " deg]");
        LOG_INFO("  Antagonist limits: [" + String(linear_equations[i].antagonist_safe_min, 2) +
                 " deg, " + String(linear_equations[i].antagonist_safe_max, 2) + " deg]");
      } else {
        LOG_WARN("  Safety limits unavailable - rerun auto-mapping");
      }
    } else {
      linear_equations[i].calculated       = false;
      linear_equations[i].agonist.valid    = false;
      linear_equations[i].antagonist.valid = false;
      linear_equations[i].limits_valid     = false;
    }
  }

  LOG_INFO("Loaded " + String(loaded_equations_count) + " linear equations from flash");
  LOG_INFO("Compact linear equations loaded - system ready for precise control");

  return loaded_equations_count > 0;
}

// Recalculate safe limits based on current equations and physical limits
bool JointController::recalculateSafeLimits() {
  LOG_INFO("Recalculating safe limits from current equations...");
  
  int recalculated_count = 0;
  
  for (int dof = 0; dof < config.dof_count; dof++) {
    if (!linear_equations[dof].calculated || !linear_equations[dof].agonist.valid || 
        !linear_equations[dof].antagonist.valid) {
      LOG_WARN("DOF " + String(dof) + ": No valid equations - skipping");
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
    
    LOG_INFO("DOF " + String(dof) + ": Safe limits updated:");
    LOG_INFO("  Old: [" + String(old_joint_min, 2) + "°, " + String(old_joint_max, 2) + "°]");
    LOG_INFO("  New: [" + String(new_safe_min, 2) + "°, " + String(new_safe_max, 2) + "°]");
    LOG_INFO("  Physical: [" + String(physical_min, 2) + "°, " + String(physical_max, 2) + "°]");
    
    linear_equations[dof].joint_safe_min = new_safe_min;
    linear_equations[dof].joint_safe_max = new_safe_max;
    linear_equations[dof].limits_valid = true;
    
    recalculated_count++;
  }
  
  if (recalculated_count == 0) {
    LOG_ERROR("No equations available to recalculate limits");
    return false;
  }
  
  // Save updated equations to flash
  LOG_INFO("Saving recalculated limits to flash...");
  if (saveLinearEquationsToFlash()) {
    LOG_INFO("Safe limits recalculated and saved successfully!");
    return true;
  } else {
    LOG_ERROR("Failed to save recalculated limits to flash");
    return false;
  }
}

