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
#include <utils.h>  // free interpolate_data() used by bilinear_eval
#include <cmath>
#include <hot_path.h>

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

  // Reject non-finite fits. The denominator guard above only covers the x (joint
  // encoder) variance: with a dead/unpowered motor bus the y samples are NaN, the
  // fit propagates NaN through slope/intercept/mse while R^2 collapses to 0, and
  // the result would still be marked valid and PERSISTED (bench 2026-07-02: the
  // no-24V knee mapping saved "y = nan*x + nan" to flash as valid equations,
  // clobbering the map slot). Silent return - Core1 context, no LOG allowed.
  if (!isfinite(result.slope) || !isfinite(result.intercept) || !isfinite(result.mse)) {
    return result; // result.valid is still false from the initializer
  }

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

bool HOT_FUNC(JointController::hasValidEquations)(uint8_t dof_index) const {
  if (dof_index >= config.dof_count) {
    return false;
  }

  return linear_equations[dof_index].calculated && linear_equations[dof_index].agonist.valid &&
         linear_equations[dof_index].antagonist.valid && linear_equations[dof_index].limits_valid;
}

// Bilinear lookup over a row-major (q0 x q1) grid: bracket q0 among the M rows (clamp at ends),
// interpolate each bracketing row over q1 (interpolate_data, verbatim), then blend in q0. A
// degenerate M==1 grid is bit-for-bit the 1D piecewise eval (same primitive, same clamp).
static float HOT_FUNC(bilinear_eval)(const float *table, const float *q0_axis, const float *q1_axis,
                           int M, int N, float q0, float q1) {
  int r0 = 0, r1 = 0;
  float t = 0.0f;
  if (M > 1) {
    if (q0 <= q0_axis[0]) {            r0 = r1 = 0; }
    else if (q0 >= q0_axis[M - 1]) {   r0 = r1 = M - 1; }
    else {
      r1 = 1;
      while (r1 < M - 1 && q0_axis[r1] < q0) r1++;
      r0 = r1 - 1;
      float denom = q0_axis[r1] - q0_axis[r0];
      t = (denom > 1e-9f) ? (q0 - q0_axis[r0]) / denom : 0.0f;
      if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    }
  }
  float v0 = interpolate_data(q1, q1_axis, table + r0 * N, N);
  float v1 = (r1 == r0) ? v0 : interpolate_data(q1, q1_axis, table + r1 * N, N);
  return v0 + t * (v1 - v0);
}

// Inverse of bilinear_eval: invert each bracketing row (motor->q1 via interpolate_data, the row is
// commit-monotone) then blend the two q1 results in JOINT space. q_other = q0.
static float inverse_bilinear(const float *table, const float *q0_axis, const float *q1_axis,
                              int M, int N, float q0, float motor) {
  int r0 = 0, r1 = 0; float t = 0.0f;
  if (M > 1) {
    if (q0 <= q0_axis[0]) { r0 = r1 = 0; }
    else if (q0 >= q0_axis[M - 1]) { r0 = r1 = M - 1; }
    else {
      r1 = 1; while (r1 < M - 1 && q0_axis[r1] < q0) r1++;
      r0 = r1 - 1;
      float denom = q0_axis[r1] - q0_axis[r0];
      t = (denom > 1e-9f) ? (q0 - q0_axis[r0]) / denom : 0.0f;
      if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    }
  }
  float j0 = interpolate_data(motor, table + r0 * N, q1_axis, N);
  float j1 = (r1 == r0) ? j0 : interpolate_data(motor, table + r1 * N, q1_axis, N);
  return j0 + t * (j1 - j0);
}

// Compute motor angle from joint angle (forward map).
// Mode LINEAR: y = m*x + b. Mode PIECEWISE: linear interpolation over captured points.
// Mode BILINEAR: 2D (q0,q1) grid lookup (q_other = live q0; NAN -> eq.q0_nominal slice).
// Version with separate inputs for agonist and antagonist
bool HOT_FUNC(JointController::calculateMotorAnglesWithEquations)(uint8_t dof_index,
                                                        float agonist_joint_angle,
                                                        float antagonist_joint_angle,
                                                        float &agonist_angle,
                                                        float &antagonist_angle,
                                                        float q_other) {
  // Validate DOF index
  if (dof_index >= config.dof_count) {
    return false;
  }

  DofLinearEquations &eq = linear_equations[dof_index];

  // BILINEAR (2D) map: row-major (q0 x q1) grid lookup. q_other carries the live q0;
  // NAN falls back to the captured-slice q0_nominal (center slice). On any invalid grid
  // this returns false so the caller's equations_ok==false fallback keeps motion safe.
  if (eq.map_mode == MAP_BILINEAR) {
    DofGridData_t &g = dof_grids[dof_index];
    if (!eq.bl_valid || !g.bl_valid || g.grid_m < 1 || g.grid_n < MIN_FINE_POINTS) {
      return false;  // invalid grid -> caller's equations_ok==false fallback (must be safe)
    }
    float q0 = isnan(q_other) ? eq.q0_nominal : q_other;
    agonist_angle    = bilinear_eval(g.agonist, g.q0_axis, g.q1_axis, g.grid_m, g.grid_n, q0,
                                     agonist_joint_angle);
    antagonist_angle = bilinear_eval(g.antagonist, g.q0_axis, g.q1_axis, g.grid_m, g.grid_n, q0,
                                     antagonist_joint_angle);
    return true;
  }

  // PIECEWISE (fine) map: interpolate over captured points (sorted by joint angle).
  if (eq.map_mode == MAP_PIECEWISE) {
    DofMappingData_t &m = dof_mappings[dof_index];
    if (!eq.pw_valid || m.flag != 1 || m.size < MIN_FINE_POINTS) {
      return false;
    }
    agonist_angle = interpolate_data(agonist_joint_angle, m.joint_data, m.agonist_data, m.size);
    antagonist_angle =
        interpolate_data(antagonist_joint_angle, m.joint_data, m.antagonist_data, m.size);
    return true;
  }

  // LINEAR map: ensure equations are computed
  if (!eq.calculated || !eq.agonist.valid || !eq.antagonist.valid || !eq.limits_valid) {
    return false;
  }

  // Compute angles using linear equations: y = m x + b
  agonist_angle = eq.agonist.slope * agonist_joint_angle + eq.agonist.intercept;
  antagonist_angle = eq.antagonist.slope * antagonist_joint_angle + eq.antagonist.intercept;

  return true;
}

// NEW: Compute joint angle using inverse linear equations (motor → joint)
// Unified version with separate inputs for agonist and antagonist
bool JointController::calculateJointAnglesWithEquations(uint8_t dof_index,
                                                        float agonist_motor_angle,
                                                        float antagonist_motor_angle,
                                                        float &agonist_joint_angle,
                                                        float &antagonist_joint_angle,
                                                        float q_other) {
  // Validate DOF index
  if (dof_index >= config.dof_count) {
    return false;
  }

  // BILINEAR (2D) inverse map: invert each bracketing q0-row (motor -> q1, the row is commit-
  // monotone) then blend in JOINT space. q_other carries the live q0; NAN -> eq.q0_nominal slice.
  if (linear_equations[dof_index].map_mode == MAP_BILINEAR) {
    DofLinearEquations &eq = linear_equations[dof_index];
    DofGridData_t &g = dof_grids[dof_index];
    if (!eq.bl_valid || !g.bl_valid || g.grid_m < 1 || g.grid_n < MIN_FINE_POINTS) return false;
    float q0 = isnan(q_other) ? eq.q0_nominal : q_other;
    agonist_joint_angle    = inverse_bilinear(g.agonist, g.q0_axis, g.q1_axis, g.grid_m, g.grid_n,
                                              q0, agonist_motor_angle);
    antagonist_joint_angle = inverse_bilinear(g.antagonist, g.q0_axis, g.q1_axis, g.grid_m,
                                              g.grid_n, q0, antagonist_motor_angle);
    return true;
  }

  // PIECEWISE (fine) map: inverse interpolation motor → joint. Requires the motor
  // arrays to be monotonic vs joint (enforced at commit) so the bracket search is valid.
  if (linear_equations[dof_index].map_mode == MAP_PIECEWISE) {
    DofMappingData_t &m = dof_mappings[dof_index];
    if (!linear_equations[dof_index].pw_valid || m.flag != 1 || m.size < MIN_FINE_POINTS) {
      return false;
    }
    agonist_joint_angle = interpolate_data(agonist_motor_angle, m.agonist_data, m.joint_data, m.size);
    antagonist_joint_angle =
        interpolate_data(antagonist_motor_angle, m.antagonist_data, m.joint_data, m.size);
    return true;
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

