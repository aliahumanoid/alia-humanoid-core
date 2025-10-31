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

// NEW: Compute linear regression for a data set
LinearRegressionCoefficients
JointController::calculateLinearRegression(float *x_data, float *y_data, int data_count) {
  LinearRegressionCoefficients result = {0, 0, 0, 0, 0, false};

  if (data_count < 2 || x_data == nullptr || y_data == nullptr) {
    LOG_ERROR("Insufficient data for linear regression");
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
    LOG_ERROR("Denominator too small in linear regression");
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

// NEW: Compute linear equations for all DOFs using linear regression
bool JointController::calculateLinearEquationsFromMappingData() {
  LOG_INFO("=== LINEAR EQUATIONS COMPUTATION ON PICO ===");
  LOG_INFO("Analyzing raw mapping data with linear regression");

  bool all_calculated = true;

  for (int dof = 0; dof < config.dof_count; dof++) {
    // Use dof_mappings directly as in transferAutoMappingData
    DofMappingData_t &mapping_data     = dof_mappings[dof];
    linear_equations[dof].limits_valid = false;

    if (mapping_data.flag != 1 || mapping_data.size <= 1) {
      LOG_ERROR("ERROR: Mapping data not available for DOF " + String(dof));
      linear_equations[dof].calculated       = false;
      linear_equations[dof].agonist.valid    = false;
      linear_equations[dof].antagonist.valid = false;
      all_calculated                         = false;
      continue;
    }

    LOG_INFO("\n--- COMPUTATION FOR DOF " + String(dof) + " (" + String(config.dofs[dof].name) + ") ---");
    LOG_INFO("Number of data points: " + String(mapping_data.size));

    // Compute regression for agonist motor
    linear_equations[dof].agonist = calculateLinearRegression(
        mapping_data.joint_data, mapping_data.agonist_data, mapping_data.size);

    // Compute regression for antagonist motor
    linear_equations[dof].antagonist = calculateLinearRegression(
        mapping_data.joint_data, mapping_data.antagonist_data, mapping_data.size);

    // Verify successful computations
    if (linear_equations[dof].agonist.valid && linear_equations[dof].antagonist.valid) {
      linear_equations[dof].calculated = true;

      // Print detailed results
      LOG_INFO("AGONIST MOTOR:");
      LOG_INFO("  Equation: y = " + String(linear_equations[dof].agonist.slope, 6) +
               " * x + " + String(linear_equations[dof].agonist.intercept, 6));
      LOG_INFO("  R^2 = " + String(linear_equations[dof].agonist.r_squared, 6));
      LOG_INFO("  MSE = " + String(linear_equations[dof].agonist.mse, 6));
      LOG_INFO("  Points: " + String(linear_equations[dof].agonist.data_points));

      LOG_INFO("ANTAGONIST MOTOR:");
      LOG_INFO("  Equation: y = " + String(linear_equations[dof].antagonist.slope, 6) +
               " * x + " + String(linear_equations[dof].antagonist.intercept, 6));
      LOG_INFO("  R^2 = " + String(linear_equations[dof].antagonist.r_squared, 6));
      LOG_INFO("  MSE = " + String(linear_equations[dof].antagonist.mse, 6));
      LOG_INFO("  Points: " + String(linear_equations[dof].antagonist.data_points));

      // Compute validity range for equations
      float min_joint_angle = mapping_data.joint_data[0];
      float max_joint_angle = mapping_data.joint_data[0];
      for (int i = 1; i < mapping_data.size; i++) {
        if (mapping_data.joint_data[i] < min_joint_angle) {
          min_joint_angle = mapping_data.joint_data[i];
        }
        if (mapping_data.joint_data[i] > max_joint_angle) {
          max_joint_angle = mapping_data.joint_data[i];
        }
      }

      Serial.println("VALID RANGE: [" + String(min_joint_angle, 2) + "°, " +
                     String(max_joint_angle, 2) + "°]");

      // Verification test: compute mean error over a few test points
      float total_error_agonist    = 0.0f;
      float total_error_antagonist = 0.0f;
      int test_points              = min(5, mapping_data.size); // Test on first 5 points

      LOG_INFO("ACCURACY CHECK (first " + String(test_points) + " points):");
      for (int i = 0; i < test_points; i++) {
        float joint_angle       = mapping_data.joint_data[i];
        float actual_agonist    = mapping_data.agonist_data[i];
        float actual_antagonist = mapping_data.antagonist_data[i];

        float predicted_agonist = linear_equations[dof].agonist.slope * joint_angle +
                                  linear_equations[dof].agonist.intercept;
        float predicted_antagonist = linear_equations[dof].antagonist.slope * joint_angle +
                                     linear_equations[dof].antagonist.intercept;

        float error_agonist    = fabs(actual_agonist - predicted_agonist);
        float error_antagonist = fabs(actual_antagonist - predicted_antagonist);

        total_error_agonist += error_agonist;
        total_error_antagonist += error_antagonist;

        Serial.println("  Point " + String(i + 1) + ": Joint=" + String(joint_angle, 2) + "°");
        Serial.println("    Agonist: actual=" + String(actual_agonist, 2) + "°, pred=" +
                       String(predicted_agonist, 2) + "°, err=" + String(error_agonist, 3) + "°");
        Serial.println("    Antagonist: actual=" + String(actual_antagonist, 2) +
                       "°, pred=" + String(predicted_antagonist, 2) +
                       "°, err=" + String(error_antagonist, 3) + "°");
      }

      LOG_INFO("MEAN ERROR:");
      Serial.println("  Agonist: " + String(total_error_agonist / test_points, 3) + "°");
      Serial.println("  Antagonist: " + String(total_error_antagonist / test_points, 3) + "°");

      // Compute and store safe limits for joint and motors
      const float PHYSICAL_SAFETY_MARGIN  = 1.0f;  // Margin on physical joint limits
      const float MAPPING_EXTENSION_RATIO = 0.10f; // Dynamic extension based on data
      const float MOTOR_SAFETY_MARGIN     = 20.0f; // Margin for motors

      float joint_mapping_min = mapping_data.joint_data[0];
      float joint_mapping_max = mapping_data.joint_data[0];
      for (int i = 1; i < mapping_data.size; i++) {
        joint_mapping_min = min(joint_mapping_min, mapping_data.joint_data[i]);
        joint_mapping_max = max(joint_mapping_max, mapping_data.joint_data[i]);
      }

      float joint_range        = joint_mapping_max - joint_mapping_min;
      float joint_extension    = joint_range * MAPPING_EXTENSION_RATIO;
      float extended_joint_min = joint_mapping_min - joint_extension;
      float extended_joint_max = joint_mapping_max + joint_extension;

      float physical_min = config.dofs[dof].limits.min_angle + PHYSICAL_SAFETY_MARGIN;
      float physical_max = config.dofs[dof].limits.max_angle - PHYSICAL_SAFETY_MARGIN;

      linear_equations[dof].joint_safe_min = max(extended_joint_min, physical_min);
      linear_equations[dof].joint_safe_max = min(extended_joint_max, physical_max);

      if (linear_equations[dof].joint_safe_min > linear_equations[dof].joint_safe_max) {
        // If the resulting interval is inverted, fall back to physical limits with margin
        linear_equations[dof].joint_safe_min = physical_min;
        linear_equations[dof].joint_safe_max = physical_max;
      }

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

      LOG_INFO("SAVED LIMITS:");
      Serial.println("  Joint safe: [" + String(linear_equations[dof].joint_safe_min, 2) + ", " +
                     String(linear_equations[dof].joint_safe_max, 2) + "]°");
      Serial.println("  Agonist safe: [" + String(linear_equations[dof].agonist_safe_min, 2) +
                     ", " + String(linear_equations[dof].agonist_safe_max, 2) + "]°");
      Serial.println("  Antagonist safe: [" +
                     String(linear_equations[dof].antagonist_safe_min, 2) + ", " +
                     String(linear_equations[dof].antagonist_safe_max, 2) + "]°");

    } else {
      LOG_ERROR("ERROR: Unable to compute linear equations for DOF " + String(dof));
      linear_equations[dof].calculated       = false;
      linear_equations[dof].agonist.valid    = false;
      linear_equations[dof].antagonist.valid = false;
      linear_equations[dof].limits_valid     = false;
      all_calculated                         = false;
    }
  }

  if (all_calculated) {
    LOG_INFO("\n=== SUMMARY OF COMPUTED EQUATIONS ===");
    for (int dof = 0; dof < config.dof_count; dof++) {
      if (linear_equations[dof].calculated) {
        Serial.println("DOF " + String(dof) + " (" + String(config.dofs[dof].name) + "):");
        Serial.println("  Agonist: y = " + String(linear_equations[dof].agonist.slope, 4) +
                       "*x + " + String(linear_equations[dof].agonist.intercept, 4) +
                       " (R²=" + String(linear_equations[dof].agonist.r_squared, 3) + ")");
        Serial.println("  Antagonist: y = " + String(linear_equations[dof].antagonist.slope, 4) +
                       "*x + " + String(linear_equations[dof].antagonist.intercept, 4) +
                       " (R²=" + String(linear_equations[dof].antagonist.r_squared, 3) + ")");
      }
    }
    Serial.println("======================================");
  }

  // NEW: Automatically save equations to flash if all have been computed
  if (all_calculated) {
    LOG_INFO("\n=== AUTOMATIC FLASH SAVE ===");
    if (saveLinearEquationsToFlash()) {
      Serial.println("✓ Equations lineari saved automaticamente in flash");
      LOG_INFO("\xE2\x9C\x93 System ready for standalone use without Pi5");
      LOG_INFO("\xE2\x9C\x93 At next boot the equations will be loaded automatically");
    } else {
      LOG_ERROR("\xE2\x9C\x97 ERROR in automatic saving of equations");
    }
    Serial.println("==========================================");
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

