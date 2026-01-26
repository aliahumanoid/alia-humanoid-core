/**
 * @file JointController_LinearEq.cpp
 * @brief Linear equations and calibration implementation for JointController
 * 
 * This file contains the implementation of JointController methods related to:
 * - Linear regression calculations
 * - Motor-to-joint angle conversions using linear equations
 * - Joint-to-motor angle conversions
 * - Equation validation and storage
 * 
 * These methods are part of the JointController class but separated here
 * for better code organization and maintainability.
 */

#include <JointController.h>
#include <debug.h>
#include <cmath>

// ============================================================================
// LINEAR EQUATIONS & CALIBRATION
// ============================================================================

// Compute linear regression for a data set
// NOTE: This function is called from Core1 - NO Serial/LOG calls allowed!
LinearRegressionCoefficients
JointController::calculateLinearRegression(float *x_data, float *y_data, int data_count) {
  LinearRegressionCoefficients result = {0, 0, 0, 0, 0, false};

  if (data_count < 2 || x_data == nullptr || y_data == nullptr) {
    // Insufficient data - return invalid result silently (Core1 context)
    return result;
  }

  // Compute sums needed for linear regression
  float sum_x         = 0.0f;
  float sum_y         = 0.0f;
  float sum_xy        = 0.0f;
  float sum_x_squared = 0.0f;
  float sum_y_squared = 0.0f;

  for (int i = 0; i < data_count; i++) {
    sum_x += x_data[i];
    sum_y += y_data[i];
    sum_xy += x_data[i] * y_data[i];
    sum_x_squared += x_data[i] * x_data[i];
    sum_y_squared += y_data[i] * y_data[i];
  }

  // Compute coefficients for y = m x + b
  float denominator = data_count * sum_x_squared - sum_x * sum_x;

  if (fabs(denominator) < 1e-10) {
    // Denominator too small - return invalid result silently (Core1 context)
    return result;
  }

  result.slope     = (data_count * sum_xy - sum_x * sum_y) / denominator;
  result.intercept = (sum_y - result.slope * sum_x) / data_count;

  // Compute R^2 and MSE
  float y_mean = sum_y / data_count;
  float ss_tot = 0.0f; // Total sum of squares
  float ss_res = 0.0f; // Residual sum of squares

  for (int i = 0; i < data_count; i++) {
    float y_pred   = result.slope * x_data[i] + result.intercept;
    float residual = y_data[i] - y_pred;

    ss_res += residual * residual;
    ss_tot += (y_data[i] - y_mean) * (y_data[i] - y_mean);
  }

  // Compute R^2
  if (ss_tot > 1e-10) {
    result.r_squared = 1.0f - (ss_res / ss_tot);
  } else {
    result.r_squared = 0.0f;
  }

  // Compute MSE (Mean Square Error)
  result.mse = ss_res / data_count;

  result.data_points = data_count;
  result.valid       = true;

  return result;
}

// Compute linear equations for all DOFs using linear regression
// NOTE: This function is called from Core1 - NO Serial/LOG calls allowed!
// All logging is deferred to Core0 after completion to avoid Serial conflicts.
bool JointController::calculateLinearEquationsFromMappingData() {
  bool all_calculated = true;

  for (int dof = 0; dof < config.dof_count; dof++) {
    // Use dof_mappings directly as in transferAutoMappingData
    DofMappingData_t &mapping_data     = dof_mappings[dof];
    linear_equations[dof].limits_valid = false;

    if (mapping_data.flag != 1 || mapping_data.size <= 1) {
      // Data not available for this DOF
      linear_equations[dof].calculated       = false;
      linear_equations[dof].agonist.valid    = false;
      linear_equations[dof].antagonist.valid = false;
      all_calculated                         = false;
      continue;
    }

    // Compute regression for agonist motor
    linear_equations[dof].agonist = calculateLinearRegression(
        mapping_data.joint_data, mapping_data.agonist_data, mapping_data.size);

    // Compute regression for antagonist motor
    linear_equations[dof].antagonist = calculateLinearRegression(
        mapping_data.joint_data, mapping_data.antagonist_data, mapping_data.size);

    // Verify successful computations
    if (linear_equations[dof].agonist.valid && linear_equations[dof].antagonist.valid) {
      linear_equations[dof].calculated = true;

      // Compute and store safe limits for joint and motors
      // 
      // JOINT LIMITS LOGIC:
      // - operating_min/max define the desired operational range
      // - If operating_* == 0, use physical limits (min_angle/max_angle)
      // - Physical limits are absolute mechanical boundaries
      // 
      // MOTOR LIMITS LOGIC:
      // - Extrapolate from mapping data + margin to allow motors to reach joint extremes
      // - Even if mapping doesn't cover full range, motors need to go there
      const float MAPPING_EXTENSION_RATIO = 0.15f; // Increased from 0.10 to better cover extremes
      const float MOTOR_SAFETY_MARGIN     = 50.0f; // Margin for motors (increased from 30 to cover extended joint range 0-110°)

      // Determine joint safe limits from config
      float physical_min = config.dofs[dof].limits.min_angle;
      float physical_max = config.dofs[dof].limits.max_angle;
      float operating_min = config.dofs[dof].limits.operating_min;
      float operating_max = config.dofs[dof].limits.operating_max;
      
      // Use operating limits if specified, otherwise use physical limits
      // operating_min/max == 0 means "use physical limits"
      if (operating_min == 0.0f && operating_max == 0.0f) {
        linear_equations[dof].joint_safe_min = physical_min;
        linear_equations[dof].joint_safe_max = physical_max;
      } else {
        // Clamp operating limits to physical limits (safety)
        linear_equations[dof].joint_safe_min = max(operating_min, physical_min);
        linear_equations[dof].joint_safe_max = min(operating_max, physical_max);
      }
      
      // Note: MAPPING_EXTENSION_RATIO defined above but not currently used for joint limits
      // (retained for potential future motor limit extrapolation based on joint range)
      (void)MAPPING_EXTENSION_RATIO; // Suppress unused variable warning

      // Compute limits for motors with margin
      float agonist_min    = mapping_data.agonist_data[0];
      float agonist_max    = mapping_data.agonist_data[0];
      float antagonist_min = mapping_data.antagonist_data[0];
      float antagonist_max = mapping_data.antagonist_data[0];

      for (int i = 1; i < mapping_data.size; i++) {
        agonist_min    = min(agonist_min, mapping_data.agonist_data[i]);
        agonist_max    = max(agonist_max, mapping_data.agonist_data[i]);
        antagonist_min = min(antagonist_min, mapping_data.antagonist_data[i]);
        antagonist_max = max(antagonist_max, mapping_data.antagonist_data[i]);
      }

      linear_equations[dof].agonist_safe_min    = agonist_min - MOTOR_SAFETY_MARGIN;
      linear_equations[dof].agonist_safe_max    = agonist_max + MOTOR_SAFETY_MARGIN;
      linear_equations[dof].antagonist_safe_min = antagonist_min - MOTOR_SAFETY_MARGIN;
      linear_equations[dof].antagonist_safe_max = antagonist_max + MOTOR_SAFETY_MARGIN;

      if (linear_equations[dof].agonist_safe_min > linear_equations[dof].agonist_safe_max) {
        float temp                             = linear_equations[dof].agonist_safe_min;
        linear_equations[dof].agonist_safe_min = linear_equations[dof].agonist_safe_max;
        linear_equations[dof].agonist_safe_max = temp;
      }
      if (linear_equations[dof].antagonist_safe_min > linear_equations[dof].antagonist_safe_max) {
        float temp                                = linear_equations[dof].antagonist_safe_min;
        linear_equations[dof].antagonist_safe_min = linear_equations[dof].antagonist_safe_max;
        linear_equations[dof].antagonist_safe_max = temp;
      }

      linear_equations[dof].limits_valid = true;

    } else {
      // Failed to compute equations for this DOF
      linear_equations[dof].calculated       = false;
      linear_equations[dof].agonist.valid    = false;
      linear_equations[dof].antagonist.valid = false;
      linear_equations[dof].limits_valid     = false;
      all_calculated                         = false;
    }
  }

  // Signal Core0 to save equations to flash and print summary
  // Flash save is handled by Core0 to avoid crash (Serial conflicts)
  if (all_calculated) {
    _pending_flash_save = true;
  }

  return all_calculated;
}

// Get linear equations for a specific DOF
DofLinearEquations *JointController::getLinearEquations(uint8_t dof_index) {

  if (dof_index >= config.dof_count) {
    return nullptr;
  }

  if (!linear_equations[dof_index].calculated) {
    return nullptr;
  }

  return &linear_equations[dof_index];
}

bool JointController::hasValidEquations(uint8_t dof_index) const {
  if (dof_index >= config.dof_count) {
    return false;
  }

  return linear_equations[dof_index].calculated && linear_equations[dof_index].agonist.valid &&
         linear_equations[dof_index].antagonist.valid && linear_equations[dof_index].limits_valid;
}

// NEW: Compute motor angle using linear equations (alternate method)
// Version with separate inputs for agonist and antagonist
bool JointController::calculateMotorAnglesWithEquations(uint8_t dof_index,
                                                        float agonist_joint_angle,
                                                        float antagonist_joint_angle,
                                                        float &agonist_angle,
                                                        float &antagonist_angle) {
  // Validate DOF index
  if (dof_index >= config.dof_count) {
    return false;
  }

  // Ensure linear equations are computed
  if (!linear_equations[dof_index].calculated || !linear_equations[dof_index].agonist.valid ||
      !linear_equations[dof_index].antagonist.valid || !linear_equations[dof_index].limits_valid) {
    return false;
  }

  // Compute angles using linear equations: y = m x + b
  agonist_angle = linear_equations[dof_index].agonist.slope * agonist_joint_angle +
                  linear_equations[dof_index].agonist.intercept;

  antagonist_angle = linear_equations[dof_index].antagonist.slope * antagonist_joint_angle +
                     linear_equations[dof_index].antagonist.intercept;

  return true;
}

// NEW: Compute joint angle using inverse linear equations (motor → joint)
// Unified version with separate inputs for agonist and antagonist
bool JointController::calculateJointAnglesWithEquations(uint8_t dof_index,
                                                        float agonist_motor_angle,
                                                        float antagonist_motor_angle,
                                                        float &agonist_joint_angle,
                                                        float &antagonist_joint_angle) {
  // Validate DOF index
  if (dof_index >= config.dof_count) {
    return false;
  }

  // Ensure linear equations are computed
  if (!linear_equations[dof_index].calculated || !linear_equations[dof_index].limits_valid) {
    return false;
  }

  // Compute joint angle from agonist motor
  bool agonist_success = false;
  if (linear_equations[dof_index].agonist.valid) {
    // Ensure slope is non-zero (avoid division by zero)
    if (fabs(linear_equations[dof_index].agonist.slope) >= 1e-10) {
      // Invert the equation: if y = m x + b, then x = (y - b) / m
      // where y = agonist_motor_angle, x = agonist_joint_angle
      agonist_joint_angle = (agonist_motor_angle - linear_equations[dof_index].agonist.intercept) /
                            linear_equations[dof_index].agonist.slope;
      agonist_success = true;
    }
  }

  // Compute joint angle from antagonist motor
  bool antagonist_success = false;
  if (linear_equations[dof_index].antagonist.valid) {
    // Ensure slope is non-zero (avoid division by zero)
    if (fabs(linear_equations[dof_index].antagonist.slope) >= 1e-10) {
      // Invert the equation: if y = m x + b, then x = (y - b) / m
      // where y = antagonist_motor_angle, x = antagonist_joint_angle
      antagonist_joint_angle =
          (antagonist_motor_angle - linear_equations[dof_index].antagonist.intercept) /
          linear_equations[dof_index].antagonist.slope;
      antagonist_success = true;
    }
  }

  // Return true only if both calculations succeeded
  return agonist_success && antagonist_success;
}

