/**
 * @file JointController_AutoMapping.cpp
 * @brief Automatic calibration (auto-mapping) implementation for JointController
 * 
 * This file contains the implementation of JointController methods related to:
 * - Starting and stopping automatic mapping/calibration
 * - Moving between mapping points
 * - Acquiring calibration data at each point
 * - Applying torques and speeds during calibration
 * - Verifying position stability and completion
 * - Transferring acquired data to mapping structures
 * 
 * These methods are part of the JointController class but separated here
 * for better code organization and maintainability.
 */

#include <JointController.h>
#include <commands.h>
#include <debug.h>
#include <utils.h>
#include <algorithm>
#include <hardware/sync.h>  // __dmb() cross-core memory barrier (pico-sdk)
#include "main_common.h"
#include <hot_path.h>  // For shared_dof_angles

namespace {
bool isActiveAutoMappingDof(const AutoMappingState_t &state, uint8_t dof_index) {
  for (int i = 0; i < state.active_dof_count; i++) {
    if (state.active_dof_indices[i] == dof_index) {
      return true;
    }
  }
  return false;
}
}

// External variables for inter-core communication
// Note: emergency_stop_requested, buffer_ready, active_buffer, pending_command_type
// are declared in main_common.h (included above)

// ============================================================================
// AUTOMATIC CALIBRATION (AUTO-MAPPING)
// ============================================================================

// Start automatic mapping for all DOFs of the joint
bool JointController::startAutoMapping(AutoMappingState_t &auto_mapping_state,
                                       float tensioning_torque, float *steps, int settle_time_ms) {
  // S2 CARRY choke point (MAJOR amendment): auto-mapping is a SEPARATE state machine that replaces
  // executeControlLoop (and thus the carry-injection resolve) and drives the motor bus. Abandon any
  // carried pair before it starts, so a carried pair's replies aren't orphaned. Core-affinity guarded.
  abandonCarriedPair();
  // Validate input parameters
  if (config.dof_count == 0) {
    LOG_C1_ERROR("No DOF configured for this joint");
    return false;
  }

  if (config.motor_count == 0) {
    LOG_C1_ERROR("No motors configured for this joint");
    return false;
  }

  uint8_t active_dof_indices[MAX_DOFS] = {0};
  int active_dof_count = 0;
  for (int i = 0; i < config.dof_count; i++) {
    if (dofSupportsAutoMapping(i)) {
      active_dof_indices[active_dof_count++] = static_cast<uint8_t>(i);
      continue;
    }

    if (config.dofs[i].drive_type == DRIVE_DIRECT_DRIVE) {
      LOG_C1_INFO("Skipping auto mapping for DOF " + String(i) + " (" +
                  String(config.dofs[i].name) + ") - direct-drive DOF");
      continue;
    }

    LOG_C1_ERROR("Auto mapping not supported for DOF " + String(i) + " (" +
                 String(config.dofs[i].name) + ")");
    return false;
  }

  if (active_dof_count == 0) {
    LOG_C1_ERROR("No tendon DOFs available for auto mapping");
    return false;
  }

  // Verify that each DOF has at least 2 motors (agonist and antagonist)
  for (int idx = 0; idx < active_dof_count; idx++) {
    int i = active_dof_indices[idx];
    int motor_count_for_dof = 0;
    bool has_agonist = false, has_antagonist = false;

    for (int j = 0; j < config.motor_count; j++) {
      if (config.motors[j].dof_index == i) {
        motor_count_for_dof++;
        if (config.motors[j].is_agonist) {
          has_agonist = true;
        } else {
          has_antagonist = true;
        }
      }
    }

    if (motor_count_for_dof < 2 || !has_agonist || !has_antagonist) {
      LOG_C1_ERROR("DOF " + String(i) + " does not have correctly configured agonist/antagonist motors");
      return false;
    }
  }

  // Reset counters before starting
  auto_mapping_state.consecutive_encoder_errors = 0;
  auto_mapping_state.last_valid_reading         = millis();
  LOG_C1_DEBUG("Initializing auto mapping counters");

  // Verify if auto‑mapping is already active
  if (auto_mapping_state.active) {
    LOG_C1_ERROR("Auto mapping already in progress");
    return false;
  }

  // Initialize mapping state
  memset(&auto_mapping_state, 0, sizeof(AutoMappingState_t));

  // IMPORTANT: Completely reset existing mapping structures to avoid conflicts
  for (int i = 0; i < config.dof_count; i++) {
    dof_mappings[i].size = 0;
    dof_mappings[i].flag = 0;
    // Optionally, reset data for safety
    memset(dof_mappings[i].agonist_data, 0, sizeof(dof_mappings[i].agonist_data));
    memset(dof_mappings[i].antagonist_data, 0, sizeof(dof_mappings[i].antagonist_data));
    memset(dof_mappings[i].joint_data, 0, sizeof(dof_mappings[i].joint_data));
  }
  LOG_C1_DEBUG("DofMappingData_t structures reset for new mapping");

  // Set first_call flag to ensure array initialization
  auto_mapping_state.first_call = true;

  // Reset counters after memset
  auto_mapping_state.consecutive_encoder_errors = 0;
  auto_mapping_state.last_valid_reading         = millis();
  auto_mapping_state.acquired_points_count      = 0; // Initialize acquired points counter

  // Set number of DOFs and motors
  auto_mapping_state.dof_count   = config.dof_count;
  auto_mapping_state.active_dof_count = active_dof_count;
  for (int i = 0; i < active_dof_count; i++) {
    auto_mapping_state.active_dof_indices[i] = active_dof_indices[i];
  }
  auto_mapping_state.motor_count = config.motor_count;

  // Set settle time
  auto_mapping_state.settle_time_ms =
      (settle_time_ms > 0) ? settle_time_ms :
      config.dofs[auto_mapping_state.active_dof_indices[0]].zero_mapping.auto_mapping_settle_time;

  // Set tensioning torque and active motor speed
  float applied_tensioning_torque =
      (tensioning_torque != 0) ? tensioning_torque :
      config.dofs[auto_mapping_state.active_dof_indices[0]].zero_mapping.tensioning_torque;

  // Get motor speed from configuration
  float motor_speed       = config.dofs[auto_mapping_state.active_dof_indices[0]].zero_mapping.auto_mapping_speed;
  float resistance_torque = config.dofs[auto_mapping_state.active_dof_indices[0]].zero_mapping.auto_mapping_resistance_torque;

  LOG_C1_INFO("Starting auto mapping - active DOFs: " + String(auto_mapping_state.active_dof_count) +
           ", motors: " + String(auto_mapping_state.motor_count) +
           ", tensioning torque: " + String(applied_tensioning_torque) +
           ", motor speed: " + String(motor_speed) + " deg/s" +
           ", resistance torque: " + String(resistance_torque));

  // Set limits and steps for each DOF
  auto_mapping_state.total_points = 1; // Initialize to 1 and multiply for each DOF

  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    // Use specific limits for auto-mapping instead of general limits
    auto_mapping_state.min_angles[i] = config.dofs[i].zero_mapping.auto_mapping_min_angle;
    auto_mapping_state.max_angles[i] = config.dofs[i].zero_mapping.auto_mapping_max_angle;

    // Verify that mapping limits are within joint's general limits
    if (auto_mapping_state.min_angles[i] < config.dofs[i].limits.min_angle) {
      LOG_C1_WARN("auto_mapping_min_angle (" + String(auto_mapping_state.min_angles[i]) +
               ") is below joint min limit (" + String(config.dofs[i].limits.min_angle) +
               ") for DOF " + String(i) + ". Using joint limit.");
      auto_mapping_state.min_angles[i] = config.dofs[i].limits.min_angle;
    }

    if (auto_mapping_state.max_angles[i] > config.dofs[i].limits.max_angle) {
      LOG_C1_WARN("auto_mapping_max_angle (" + String(auto_mapping_state.max_angles[i]) +
               ") is above joint max limit (" + String(config.dofs[i].limits.max_angle) +
               ") for DOF " + String(i) + ". Using joint limit.");
      auto_mapping_state.max_angles[i] = config.dofs[i].limits.max_angle;
    }

    // Set step from configuration or parameters
    if (steps != nullptr && steps[i] > 0) {
      auto_mapping_state.step_sizes[i] = steps[i];
    } else {
      auto_mapping_state.step_sizes[i] = config.dofs[i].zero_mapping.auto_mapping_step;
      // Ensure step is reasonable (minimum 0.5 degrees)
      if (auto_mapping_state.step_sizes[i] < 0.5f) {
        auto_mapping_state.step_sizes[i] = 0.5f;
      }
    }

    // Calculate number of points for this DOF
    // Must match the actual iteration: start at min, increment by step, stop when > max.
    // floor() ensures the count matches the loop behavior (ceil caused overcounting).
    int dof_points = (int)floor((auto_mapping_state.max_angles[i] - auto_mapping_state.min_angles[i]) /
                                auto_mapping_state.step_sizes[i]) +
                     1;

    // Update total points (cartesian product)
    auto_mapping_state.total_points *= dof_points;

    // Verify total number of points does not exceed memory capacity
    if (auto_mapping_state.total_points > MAX_MAPPING_DATA_SIZE) {
      LOG_C1_ERROR("Total number of points (" + String(auto_mapping_state.total_points) +
                ") exceeds maximum capacity (" + String(MAX_MAPPING_DATA_SIZE) + ")");
      LOG_C1_WARN("Reduce mapping step or angle range");
      return false;
    }

    // Start from minimum angle
    auto_mapping_state.target_angles[i] = auto_mapping_state.min_angles[i];

    LOG_C1_DEBUG("DOF " + String(i) + ": mapping range [" + String(auto_mapping_state.min_angles[i]) +
              ", " + String(auto_mapping_state.max_angles[i]) + "] (joint limits: [" +
              String(config.dofs[i].limits.min_angle) + ", " +
              String(config.dofs[i].limits.max_angle) + "]) step " +
              String(auto_mapping_state.step_sizes[i]) + ", points " + String(dof_points));
  }

  // Reset counters and set flags
  auto_mapping_state.current_point     = 0;
  auto_mapping_state.stability_count   = 0;
  auto_mapping_state.position_reached  = false;
  auto_mapping_state.settle_start_time = 0;

  // Initialize applied torques
  for (int i = 0; i < auto_mapping_state.motor_count; i++) {
    int motor_dof = config.motors[i].dof_index;
    if (!isActiveAutoMappingDof(auto_mapping_state, motor_dof)) {
      auto_mapping_state.applied_torques[i] = 0.0f;
      continue;
    }

    // Initial setting: tensioning torques based on motor role (agonist/antagonist)
    if (config.motors[i].is_agonist) {
      auto_mapping_state.applied_torques[i] = applied_tensioning_torque;
    } else {
      auto_mapping_state.applied_torques[i] = -applied_tensioning_torque;
    }
  }

  // Stop all motors before starting for safety
  LOG_C1_INFO("Stopping all motors before starting auto mapping");
  stopAllMotors();
  sleep_ms(500); // Brief pause to ensure motors stop completely

  // Apply initial torques to all motors
  LOG_C1_INFO("Applying initial torques for auto mapping");
  for (int i = 0; i < auto_mapping_state.motor_count; i++) {
    if (!isActiveAutoMappingDof(auto_mapping_state, config.motors[i].dof_index)) {
      motors[i]->setTorque(0);
      LOG_C1_DEBUG("  Motor " + String(i) + ": skipped (DOF " +
                   String(config.motors[i].dof_index) + " not in auto-mapping set)");
      continue;
    }
    motors[i]->setTorque(auto_mapping_state.applied_torques[i]);
    LOG_C1_DEBUG("  Motor " + String(i) + ": torque " +
              String(auto_mapping_state.applied_torques[i]));
  }

  // Activate mapping
  auto_mapping_state.active = true;

  // Print mapping info
  LOG_C1_INFO("Auto mapping started — Total points: " + String(auto_mapping_state.total_points) +
           ", settle time: " + String(auto_mapping_state.settle_time_ms) + "ms");

  return true;
}

// Move to next mapping point
bool JointController::moveToNextMappingPoint(AutoMappingState_t &auto_mapping_state) {
  // Print detailed information about current point and transition to next
  LOG_C1_DEBUG("\n----------------------------------------------------------");
  LOG_C1_DEBUG("MAPPING POINT CHANGE — Current point: " + String(auto_mapping_state.current_point));
  LOG_C1_DEBUG("Current position:");
  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    // Use shared_dof_angles (updated by Core0)
    float current_angle = shared_dof_angles.valid[i] ? shared_dof_angles.angles[i] : 0.0f;
    LOG_C1_DEBUG("  DOF " + String(i) + ": " + String(current_angle, 2) +
              "° (target was: " + String(auto_mapping_state.target_angles[i], 2) + "°)");
  }

  // Reset of PID controllers no longer needed because PID is not used for automatic mapping

  // Pause before proceeding to next point
  const int TRANSITION_DELAY_MS = 2000; // 2 seconds pause
  LOG_C1_DEBUG("Waiting " + String(TRANSITION_DELAY_MS) + "ms before proceeding...");
  sleep_ms(TRANSITION_DELAY_MS);

  // Multidimensional increment, like nested counters
  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    // Increment target angle for this DOF
    auto_mapping_state.target_angles[i] += auto_mapping_state.step_sizes[i];

    // If we haven't exceeded the maximum for this DOF, we've found the next point
    if (auto_mapping_state.target_angles[i] <= auto_mapping_state.max_angles[i]) {
      // Target updated, continue with new point
      LOG_C1_DEBUG("New target:");
      for (int jdx = 0; jdx < auto_mapping_state.active_dof_count; jdx++) {
        int j = auto_mapping_state.active_dof_indices[jdx];
        LOG_C1_DEBUG("  DOF " + String(j) + ": " + String(auto_mapping_state.target_angles[j], 2) +
                  "°");
      }

      // Explicit reset of all movement flags for each DOF
      for (int kdx = 0; kdx < auto_mapping_state.active_dof_count; kdx++) {
        int k = auto_mapping_state.active_dof_indices[kdx];
        auto_mapping_state.dof_movement_initialized[k] = false;
        auto_mapping_state.last_direction[k]           = 0;
        auto_mapping_state.direction_check_counter[k]  = 0;
        auto_mapping_state.debug_counter[k]            = 0;
        auto_mapping_state.dof_reach_logged[k]         = false;
      }
      LOG_C1_DEBUG("Explicit reset of movement flags for all DOFs");

      // Also reset global timeout counter
      auto_mapping_state.last_valid_reading = millis();

      LOG_C1_DEBUG("----------------------------------------------------------\n");
      return true;
    }

    // Reset this DOF to minimum and move to next DOF
    auto_mapping_state.target_angles[i] = auto_mapping_state.min_angles[i];
    LOG_C1_DEBUG("Reset DOF " + String(i) + " to minimum: " +
              String(auto_mapping_state.min_angles[i], 2) + "°");
  }

  // If we get here, we've completed all DOFs
  LOG_C1_INFO("All points explored");
  LOG_C1_DEBUG("----------------------------------------------------------\n");
  return false;
}

// Acquire current point in automatic mapping
bool JointController::acquireCurrentPoint(AutoMappingState_t &auto_mapping_state) {
  // Verify that there's space in the array to save the point
  if (auto_mapping_state.acquired_points_count >= MAX_MAPPING_DATA_SIZE) {
    LOG_C1_ERROR("Acquired points array full");
    return false;
  }

  // Populate structure with current data
  auto_mapping_state.last_sample.timestamp = millis();

  // Acquire DOF angles from shared state (updated by Core0)
  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    if (!shared_dof_angles.valid[i]) {
      LOG_C1_ERROR("Acquisition failed — invalid encoder reading for DOF " + String(i));
      return false;
    }
    auto_mapping_state.last_sample.dof_angles[i] = shared_dof_angles.angles[i];
  }

  // Acquire motor angles
  for (int i = 0; i < auto_mapping_state.motor_count; i++) {
    auto_mapping_state.last_sample.motor_angles[i] = motors[i]->getMultiAngleSync().angle;
  }

  // Save point to acquired points array
  auto_mapping_state.acquired_points[auto_mapping_state.acquired_points_count] =
      auto_mapping_state.last_sample;
  auto_mapping_state.acquired_points_count++;

  // Print acquired point data
  String point_data = "AUTO_MAP_POINT:" + String(auto_mapping_state.current_point) + ":";

  // Add DOF angles
  point_data += "DOF=[";
  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    point_data += String(auto_mapping_state.last_sample.dof_angles[i], 2);
    if (idx < auto_mapping_state.active_dof_count - 1)
      point_data += ",";
  }
  point_data += "]:";

  // Add motor angles
  point_data += "MOTORS=[";
  for (int i = 0; i < auto_mapping_state.motor_count; i++) {
    point_data += String(auto_mapping_state.last_sample.motor_angles[i], 2);
    if (i < auto_mapping_state.motor_count - 1)
      point_data += ",";
  }
  point_data += "]";

  // INFO (not DEBUG) so the paired raw grid point (q0,q1 + all motor angles) is always
  // logged during auto-mapping — needed to recover the 2D joint↔motor coupling that the
  // per-DOF sorting in the saved map throws away.
  LOG_C1_INFO(point_data);

  // Calculate and print progress
  float progress =
      (float)auto_mapping_state.current_point / auto_mapping_state.total_points * 100.0f;
  if (auto_mapping_state.current_point % 10 == 0 || (int)progress % 5 == 0) {
    LOG_C1_INFO("AUTO_MAP_PROGRESS:" + String(progress, 1) + "% - " +
             String(auto_mapping_state.current_point) + "/" +
             String(auto_mapping_state.total_points) + " points");
  }

  return true;
}

// Helper function for interpolation (wrapper to global interpolate_data)
float HOT_FUNC(JointController::interpolate_data)(float target_value, float *data1, float *data2, int size) {
  return ::interpolate_data(target_value, data1, data2, size);
}

// ============================================================================
// FINE REMAP ("command and record")
// ============================================================================
// Operator commands positions under normal impedance/cascade control and records
// the settled (joint, agonist, antagonist) sample at each. On commit the points
// become a per-DOF piecewise (non-linear) map evaluated by calculateMotorAngles...

// Max joint speed (deg/s) under which a recorded sample is considered settled.
static constexpr float FINE_SETTLE_MAX_DPS = 2.0f;
// Minimum joint-angle separation (deg) between consecutive recorded points.
static constexpr float FINE_MIN_SEPARATION_DEG = 0.5f;
// Required fraction of the DOF safe range that captured points must span.
static constexpr float FINE_MIN_COVERAGE = 0.85f;

// Strictly monotonic: every step has the same nonzero sign and magnitude >= min_step.
static bool fineIsMonotonic(const float *y, int n, float min_step) {
  if (n < 2) return false;
  bool increasing = (y[n - 1] > y[0]);
  for (int i = 1; i < n; i++) {
    float d = y[i] - y[i - 1];
    if (increasing && d < min_step) return false;
    if (!increasing && d > -min_step) return false;
  }
  return true;
}

bool JointController::startFineCapture(uint8_t dof_index) {
  if (dof_index >= config.dof_count) return false;
  // Piecewise agonist/antagonist map applies to tendon DOFs only.
  if (config.dofs[dof_index].drive_type == DRIVE_DIRECT_DRIVE) {
    LOG_C1_ERROR("FINE_CAPTURE_FAIL: direct-drive DOF " + String(dof_index) + " not supported");
    return false;
  }
  _fine_capture_active = true;
  _fine_capture_dof    = dof_index;
  _fc_size             = 0;
  LOG_C1_INFO("FINE_CAPTURE_START:" + String(dof_index));
  return true;
}

bool JointController::recordFinePoint() {
  if (!_fine_capture_active) return false;
  uint8_t dof = _fine_capture_dof;
  if (_fc_size >= MAX_MAPPING_DATA_SIZE) {
    LOG_C1_WARN("FINE_POINT_REJECT: buffer full (" + String(MAX_MAPPING_DATA_SIZE) + ")");
    return false;
  }
  // Freshness of joint + motor readings
  if (!shared_dof_angles.valid[dof] || !cached_motor_angles.valid[dof]) {
    LOG_C1_WARN("FINE_POINT_REJECT: stale reading dof=" + String(dof));
    return false;
  }
  // Quasi-static gate: only record while settled
  float vel = shared_dof_angles.velocities[dof];
  if (fabs(vel) > FINE_SETTLE_MAX_DPS) {
    LOG_C1_WARN("FINE_POINT_REJECT: not settled |v|=" + String(fabs(vel), 1) + " deg/s");
    return false;
  }
  float joint     = shared_dof_angles.angles[dof];
  float ag_meas   = cached_motor_angles.agonist[dof];
  float anta_meas = cached_motor_angles.antagonist[dof];
  // Remove the co-contraction the cascade applied this cycle so the stored map is the NEUTRAL
  // baseline (cascade adds +off to the agonist ref / -off to the antagonist ref; operating then
  // re-applies stiffness exactly once instead of double-counting it).
  float off  = (dof < MAX_DOFS) ? _cocontraction_offset[dof] : 0.0f;
  float ag   = ag_meas - off;
  float anta = anta_meas + off;
  // Reject near-duplicate joint positions
  for (int i = 0; i < _fc_size; i++) {
    if (fabs(_fc_joint[i] - joint) < FINE_MIN_SEPARATION_DEG) {
      LOG_C1_WARN("FINE_POINT_REJECT: duplicate joint=" + String(joint, 2));
      return false;
    }
  }
  _fc_joint[_fc_size]      = joint;
  _fc_agonist[_fc_size]    = ag;
  _fc_antagonist[_fc_size] = anta;
  _fc_size++;
  // Stream to host (AG/ANTA are the NEUTRAL stored values, co-contraction removed).
  LOG_C1_INFO("FINE_MAP_POINT:" + String(dof) + ":IDX=" + String(_fc_size) +
              ":JOINT=" + String(joint, 3) + ":AG=" + String(ag, 3) + ":ANTA=" + String(anta, 3));
  LOG_C1_INFO("FINE_COCO:dof=" + String(dof) + " off=" + String(off, 3) +
              " ag_raw=" + String(ag_meas, 2) + " anta_raw=" + String(anta_meas, 2));
  return true;
}

void JointController::stopFineCapture() {
  _fine_capture_active = false;
  LOG_C1_INFO("FINE_CAPTURE_STOP:" + String(_fine_capture_dof) + " points=" + String(_fc_size));
}

void JointController::abortFineCapture() {
  _fine_capture_active = false;
  _fc_size             = 0;
  LOG_C1_INFO("FINE_CAPTURE_ABORT");
}

bool JointController::commitFineCapture(uint8_t dof_index) {
  if (dof_index >= config.dof_count) return false;
  // COMMIT ENDS THE CAPTURE SESSION ON EVERY OUTCOME (2026-07-10): while the session is
  // active, getMappingSafeRange returns the WIDE physical band for this DOF (ratchet fix).
  // The flag used to clear only on SUCCESS — a FINE_COMMIT_FAIL (monotonicity/span) left
  // the wide safety band latched for the rest of the session, and the host's single-frame
  // FINE abort is droppable. Fail-closed: clear first; a failed commit means re-run the
  // capture (startFineCapture re-arms), never continue on a half-validated session.
  _fine_capture_active = false;
  if (dof_index != _fine_capture_dof) {
    LOG_C1_ERROR("FINE_COMMIT_FAIL: dof mismatch");
    return false;
  }
  if (_fc_size < MIN_FINE_POINTS) {
    LOG_C1_ERROR("FINE_COMMIT_FAIL: too few points (" + String(_fc_size) + "<" +
                 String(MIN_FINE_POINTS) + ")");
    return false;
  }

  // Work on a local copy and validate BEFORE touching the live map (dof_mappings).
  int n = _fc_size;
  float j[MAX_MAPPING_DATA_SIZE], a[MAX_MAPPING_DATA_SIZE], b[MAX_MAPPING_DATA_SIZE];
  for (int i = 0; i < n; i++) {
    j[i] = _fc_joint[i];
    a[i] = _fc_agonist[i];
    b[i] = _fc_antagonist[i];
  }
  // Sort by ascending joint angle (same approach as transferAutoMappingData)
  for (int i = 0; i < n - 1; i++) {
    for (int k = 0; k < n - i - 1; k++) {
      if (j[k] > j[k + 1]) {
        float t;
        t = j[k]; j[k] = j[k + 1]; j[k + 1] = t;
        t = a[k]; a[k] = a[k + 1]; a[k + 1] = t;
        t = b[k]; b[k] = b[k + 1]; b[k + 1] = t;
      }
    }
  }

  // The fine map DEFINES its own operating range = the captured (validated-monotonic) span.
  // The DOF safe range is narrowed to this span (set below) so the control loop never commands
  // a joint angle outside the mapped region — e.g. DOF1's everted antagonist fold is excluded by
  // capturing only up to the monotonic limit. Gate only on a sensible minimum span so a sliver
  // capture is still rejected.
  float phys_lo    = config.dofs[dof_index].limits.min_angle;
  float phys_hi    = config.dofs[dof_index].limits.max_angle;
  float phys_range = phys_hi - phys_lo;
  float covered    = j[n - 1] - j[0];
  const float FINE_MIN_SPAN_FRACTION = 0.5f;  // captured span must cover >=50% of physical range (was 0.6f, but
                                              // DOF1's reachable non-fold range ~28deg = 56% of the 50deg physical, so
                                              // the 60% gate was unreachable + ratcheted the safe-band narrow; an eyelet
                                              // replacement needs a clean recalibration -> 0.5 lets DOF1 commit again)
  if (phys_range > 1.0f && covered < FINE_MIN_SPAN_FRACTION * phys_range) {
    LOG_C1_ERROR("FINE_COMMIT_FAIL: span " + String(covered, 1) + "/" + String(phys_range, 1) +
                 " deg (<50% of range)");
    return false;
  }
  // New operating range = captured span, clamped to physical limits.
  float lo    = max(j[0], phys_lo);
  float hi    = min(j[n - 1], phys_hi);
  float range = hi - lo;

  // Monotonicity (required so the inverse map is well-defined)
  const float MIN_STEP = 0.01f;
  if (!fineIsMonotonic(a, n, MIN_STEP)) {
    LOG_C1_ERROR("FINE_COMMIT_FAIL: agonist not monotonic");
    return false;
  }
  if (!fineIsMonotonic(b, n, MIN_STEP)) {
    LOG_C1_ERROR("FINE_COMMIT_FAIL: antagonist not monotonic");
    return false;
  }

  // Validated — install as the DOF's piecewise map.
  DofMappingData_t &m = dof_mappings[dof_index];
  for (int i = 0; i < n; i++) {
    m.joint_data[i]      = j[i];
    m.agonist_data[i]    = a[i];
    m.antagonist_data[i] = b[i];
  }
  m.size = n;
  m.flag = 1;

  // Recompute motor safe limits from the new points (mirror the linear path).
  const float MOTOR_SAFETY_MARGIN = 50.0f;
  float amin = a[0], amax = a[0], bmin = b[0], bmax = b[0];
  for (int i = 1; i < n; i++) {
    amin = min(amin, a[i]); amax = max(amax, a[i]);
    bmin = min(bmin, b[i]); bmax = max(bmax, b[i]);
  }
  DofLinearEquations &eq = linear_equations[dof_index];
  eq.joint_safe_min      = lo;
  eq.joint_safe_max      = hi;
  eq.agonist_safe_min    = amin - MOTOR_SAFETY_MARGIN;
  eq.agonist_safe_max    = amax + MOTOR_SAFETY_MARGIN;
  eq.antagonist_safe_min = bmin - MOTOR_SAFETY_MARGIN;
  eq.antagonist_safe_max = bmax + MOTOR_SAFETY_MARGIN;
  eq.limits_valid        = true;
  eq.map_mode            = MAP_PIECEWISE;
  eq.pw_valid            = true;
  _fine_capture_active   = false;

  // span=lo..hi is parsed by the host (calibrate --require-span gate): a capture that
  // silently lost its endpoint targets to settle-SKIPs must not be saved as the new map.
  LOG_C1_INFO("FINE_COMMIT_OK:" + String(dof_index) + " points=" + String(n) + " coverage=" +
              String(covered, 1) + "/" + String(range, 1) + " deg span=" + String(lo, 2) +
              ".." + String(hi, 2));
  return true;
}

// ============================================================================
// GRID CAPTURE (2D bilinear map) — STEP 2
//
// The host drives a per-row 1D fine capture (REUSING startFineCapture/recordFinePoint/
// stopFineCapture which fill the _fc_* scratch), then calls recordGridRow to harvest that
// settled row at a q0 coordinate. Each row is RESAMPLED onto a shared q1_axis so the grid is
// rectangular. The in-progress grid is kept bl_valid=false; only a successful commitGridCapture
// makes it usable (MAP_BILINEAR). Nothing here runs unless the host explicitly drives it.
// ============================================================================

bool JointController::startGridCapture(uint8_t dof) {
  if (dof >= config.dof_count) return false;
  // Bilinear agonist/antagonist grid applies to tendon DOFs only (mirror startFineCapture).
  if (config.dofs[dof].drive_type == DRIVE_DIRECT_DRIVE) {
    LOG_C1_ERROR("GRID_CAPTURE_FAIL: direct-drive DOF " + String(dof) + " not supported");
    return false;
  }
  // The captured DOF must be a COUPLED DOF, never the q0 coupling SOURCE itself: a bilinear grid for
  // Q0_DOF would use that DOF's own angle as its coupling axis (self-referential, nonsensical). This
  // rejects a wrong host command that would publish a bilinear map on DOF0 with DOF0 as q0 — the
  // dedicated capture_grid script targets the coupled DOF, but the firmware endpoint must enforce it.
  if (dof == Q0_DOF) {
    LOG_C1_ERROR("GRID_CAPTURE_FAIL: DOF " + String(dof) + " is the q0 coupling source (Q0_DOF=" +
                 String(Q0_DOF) + ") — cannot capture a bilinear grid for the coupling axis itself");
    return false;
  }
  // Build into the SCRATCH grid; the live dof_grids[dof] is untouched until a successful commit
  // (so an abort or a torn capture can never affect a previously-committed live grid).
  memset(&_grid_scratch, 0, sizeof(_grid_scratch));
  _grid_scratch.grid_m   = 0;
  _grid_scratch.grid_n   = 0;
  _grid_scratch.bl_valid = false;  // in-progress grid must NOT be usable
  _grid_capture_active   = true;
  _grid_dof              = dof;
  LOG_C1_INFO("GRID_CAPTURE_START:" + String(dof));
  return true;
}

bool JointController::recordGridRow(uint8_t dof, uint8_t row, float q0) {
  if (!_grid_capture_active) {
    LOG_C1_ERROR("GRID_ROW_FAIL: no active grid capture");
    return false;
  }
  if (dof != _grid_dof) {
    LOG_C1_ERROR("GRID_ROW_FAIL: dof mismatch");
    return false;
  }
  if (row >= GRID_M_MAX) {
    LOG_C1_ERROR("GRID_ROW_FAIL: row " + String(row) + ">=" + String(GRID_M_MAX));
    return false;
  }
  if (_fc_size < MIN_FINE_POINTS) {
    LOG_C1_ERROR("GRID_ROW_FAIL: too few scratch points (" + String(_fc_size) + "<" +
                 String(MIN_FINE_POINTS) + ")");
    return false;
  }

  // Copy the 1D scratch to a local row and sort ascending by joint (mirror commitFineCapture).
  int n = _fc_size;
  float j[MAX_MAPPING_DATA_SIZE], a[MAX_MAPPING_DATA_SIZE], b[MAX_MAPPING_DATA_SIZE];
  for (int i = 0; i < n; i++) {
    j[i] = _fc_joint[i];
    a[i] = _fc_agonist[i];
    b[i] = _fc_antagonist[i];
  }
  for (int i = 0; i < n - 1; i++) {
    for (int k = 0; k < n - i - 1; k++) {
      if (j[k] > j[k + 1]) {
        float t;
        t = j[k]; j[k] = j[k + 1]; j[k + 1] = t;
        t = a[k]; a[k] = a[k + 1]; a[k + 1] = t;
        t = b[k]; b[k] = b[k + 1]; b[k + 1] = t;
      }
    }
  }

  // Monotonicity of the raw row (required so the per-row q1 interpolation is well-defined).
  const float MIN_STEP = 0.01f;
  if (!fineIsMonotonic(a, n, MIN_STEP)) {
    LOG_C1_ERROR("GRID_ROW_FAIL: agonist not monotonic (row=" + String(row) + ")");
    return false;
  }
  if (!fineIsMonotonic(b, n, MIN_STEP)) {
    LOG_C1_ERROR("GRID_ROW_FAIL: antagonist not monotonic (row=" + String(row) + ")");
    return false;
  }

  DofGridData_t &g = _grid_scratch;

  // STRICT-APPEND: a row may only be written if it is the next contiguous row. This guarantees the
  // scratch grid is always row-contiguous (no holes) and forbids re-recording an earlier row (which
  // would change the stride / leave stale rows), closing the row-contiguity and row-0-re-record
  // blockers. (The very first row therefore must be row 0, which establishes the q1_axis.)
  if (row != g.grid_m) {
    LOG_C1_ERROR("GRID_ROW_FAIL: expected row " + String(g.grid_m) + ", got " + String(row));
    return false;
  }

  // Establish q1_axis on row 0: downsample the sorted joint[] to grid_n points keeping BOTH
  // endpoints (same even-spacing-with-endpoints index formula as saveFineMapToFlash). grid_n is
  // LOCKED after row 0 — a later row whose resample would change grid_n is REJECTED (the grid must
  // stay rectangular on the shared q1_axis).
  if (row == 0) {
    int gn = (n < GRID_N_MAX) ? n : GRID_N_MAX;
    g.grid_n = (uint8_t)gn;
    for (int c = 0; c < gn; c++) {
      int idx = (gn == 1) ? 0
                          : (int)(((long)c * (long)(n - 1) + (gn - 1) / 2) / (gn - 1));
      if (idx > n - 1) idx = n - 1;
      g.q1_axis[c] = j[idx];
    }
  } else {
    // grid_n is locked after row 0: the new row must not imply a different column count.
    int gn_new = (n < GRID_N_MAX) ? n : GRID_N_MAX;
    if (gn_new != g.grid_n) {
      LOG_C1_ERROR("GRID_ROW_FAIL: grid_n locked at " + String(g.grid_n) + ", row implies " +
                   String(gn_new));
      return false;
    }
  }

  // Resample THIS row onto the shared q1_axis (every row, including row 0) via piecewise-linear
  // interpolation over the raw (sorted) row. Clamps at the row's own endpoints.
  int N = g.grid_n;
  for (int c = 0; c < N; c++) {
    g.agonist[row * N + c]    = interpolate_data(g.q1_axis[c], j, a, n);
    g.antagonist[row * N + c] = interpolate_data(g.q1_axis[c], j, b, n);
  }

  g.q0_axis[row] = q0;
  g.grid_m = (uint8_t)(row + 1);  // contiguous by strict-append

  LOG_C1_INFO("GRID_ROW_OK:" + String(dof) + " row=" + String(row) + " q0=" + String(q0, 3) +
              " n=" + String(g.grid_n));
  return true;
}

// BLENDED-ROW MONOTONICITY GUARD (the hysteresis discriminator):
// The eval blends adjacent q0-rows linearly. If a static coupling surface exists, every blend
// between two monotonic rows stays monotonic. If the coupling is HYSTERETIC (state-dependent), an
// interior blend folds back (non-monotonic). Test the interior fractions of every adjacent q0-row
// pair. Returns true iff all blends stay monotonic. Called at capture-time (commitGridCapture) and
// at boot-time (loadGridFromFlash, defense-in-depth against flash corruption).
bool JointController::gridBlendedRowsMonotonic(const DofGridData_t &g) {
  const int M = g.grid_m;
  const int N = g.grid_n;
  const float MIN_STEP = 0.01f;
  const float blend_t[3] = {0.25f, 0.5f, 0.75f};
  float blended_ag[GRID_N_MAX];
  float blended_anta[GRID_N_MAX];
  for (int r0 = 0; r0 < M - 1; r0++) {
    int r1 = r0 + 1;
    for (int ti = 0; ti < 3; ti++) {
      float t = blend_t[ti];
      for (int c = 0; c < N; c++) {
        blended_ag[c]   = (1.0f - t) * g.agonist[r0 * N + c]    + t * g.agonist[r1 * N + c];
        blended_anta[c] = (1.0f - t) * g.antagonist[r0 * N + c] + t * g.antagonist[r1 * N + c];
      }
      if (!fineIsMonotonic(blended_ag, N, MIN_STEP) ||
          !fineIsMonotonic(blended_anta, N, MIN_STEP)) {
        return false;
      }
    }
  }
  return true;
}

// PER-ROW MONOTONICITY GUARD: every STORED raw row (agonist + antagonist) must itself be strictly
// monotonic. The blended-row guard tests blends BETWEEN rows; this tests each row in isolation so a
// single corrupt/non-monotonic row is caught even if blends happen to pass. Run at commit (sanity of
// the captured rows) and at load (defense-in-depth against flash bit-rot).
bool JointController::gridRowsMonotonic(const DofGridData_t &g) {
  const int M = g.grid_m;
  const int N = g.grid_n;
  const float MIN_STEP = 0.01f;
  for (int r = 0; r < M; r++) {
    if (!fineIsMonotonic(g.agonist + r * N, N, MIN_STEP) ||
        !fineIsMonotonic(g.antagonist + r * N, N, MIN_STEP)) {
      return false;
    }
  }
  return true;
}

// Atomically publish a validated grid into the live slot. Demote map_mode first so the control loop
// stops using the (about-to-be-overwritten) grid, write the grid+limits, barrier, then promote.
void JointController::publishBilinearGrid(uint8_t dof, const DofGridData_t &src, float q0_nominal) {
  DofLinearEquations &eq = linear_equations[dof];
  uint8_t prev_mode = eq.map_mode;
  eq.map_mode = (prev_mode == MAP_PIECEWISE) ? MAP_PIECEWISE : MAP_LINEAR; // demote away from BILINEAR
  __dmb();                                   // ensure the demote is visible before we overwrite the grid
  dof_grids[dof] = src;                       // ~859B copy (now safe: control loop not in BILINEAR branch)

  // Recompute motor safe limits over ALL grid_m*grid_n cells (mirror the fine path: min-margin /
  // max+margin). q0_nominal is supplied by the caller (commit: captured center row; load: stored
  // q0_nominal) — both a real swept-range q0.
  const int M = src.grid_m;
  const int N = src.grid_n;
  const float MOTOR_SAFETY_MARGIN = 50.0f;
  float amin = src.agonist[0], amax = src.agonist[0];
  float bmin = src.antagonist[0], bmax = src.antagonist[0];
  for (int idx = 0; idx < M * N; idx++) {
    amin = min(amin, src.agonist[idx]);    amax = max(amax, src.agonist[idx]);
    bmin = min(bmin, src.antagonist[idx]); bmax = max(bmax, src.antagonist[idx]);
  }
  eq.joint_safe_min      = src.q1_axis[0];
  eq.joint_safe_max      = src.q1_axis[N - 1];
  eq.agonist_safe_min    = amin - MOTOR_SAFETY_MARGIN;
  eq.agonist_safe_max    = amax + MOTOR_SAFETY_MARGIN;
  eq.antagonist_safe_min = bmin - MOTOR_SAFETY_MARGIN;
  eq.antagonist_safe_max = bmax + MOTOR_SAFETY_MARGIN;
  eq.q0_nominal          = q0_nominal;
  eq.limits_valid        = true;
  eq.calculated          = true;

  dof_grids[dof].bl_valid = true;
  eq.bl_valid = true;
  __dmb();                                   // ensure grid+limits are visible before promoting
  eq.map_mode = MAP_BILINEAR;                 // promote LAST
}

bool JointController::commitGridCapture(uint8_t dof) {
  if (!_grid_capture_active) {
    LOG_C1_ERROR("GRID_COMMIT_FAIL: no active grid capture");
    return false;
  }
  if (dof != _grid_dof) {
    LOG_C1_ERROR("GRID_COMMIT_FAIL: dof mismatch");
    return false;
  }
  // Validate the SCRATCH grid; the live dof_grids[dof] is untouched until publishBilinearGrid below.
  DofGridData_t &g = _grid_scratch;
  if (g.grid_m < 2) {  // need >=2 rows for a real 2D grid
    LOG_C1_ERROR("GRID_COMMIT_FAIL: too few rows (" + String(g.grid_m) + "<2)");
    return false;
  }
  if (g.grid_n < MIN_FINE_POINTS) {
    LOG_C1_ERROR("GRID_COMMIT_FAIL: too few cols (" + String(g.grid_n) + "<" +
                 String(MIN_FINE_POINTS) + ")");
    return false;
  }

  const int M = g.grid_m;
  const int N = g.grid_n;

  // q0_axis must be strictly ascending (rows recorded at increasing q0 sweep points).
  for (int r = 1; r < M; r++) {
    if (!(g.q0_axis[r] > g.q0_axis[r - 1])) {
      LOG_C1_ERROR("GRID_COMMIT_FAIL: q0 axis not monotonic");
      return false;
    }
  }

  // PER-ROW MONOTONICITY GUARD: each stored raw row must itself be strictly monotonic (defense vs a
  // single bad row that blends could still pass).
  if (!gridRowsMonotonic(g)) {
    LOG_C1_ERROR("GRID_COMMIT_FAIL: a stored row is non-monotonic");
    return false;  // leave bl_valid=false
  }

  // BLENDED-ROW MONOTONICITY GUARD (the hysteresis discriminator): reject and shelve 2D rather than
  // ship an unstable inverse map if any interior blend of adjacent q0-rows folds back. Same guard is
  // re-run at boot from loadGridFromFlash (defense-in-depth against flash corruption).
  if (!gridBlendedRowsMonotonic(g)) {
    LOG_C1_ERROR("GRID_COMMIT_FAIL: blended row non-monotonic -> coupling is HYSTERETIC, not a "
                 "static surface; shelve 2D");
    return false;  // leave bl_valid=false
  }

  // Validated — ATOMIC-PUBLISH the scratch grid into the live slot (cross-core ordered: demote map
  // mode, write grid+limits, barrier, promote). q0_nominal = the captured CENTER row.
  g.bl_valid = true;
  publishBilinearGrid(dof, g, g.q0_axis[g.grid_m / 2]);
  _grid_capture_active   = false;

  LOG_C1_INFO("GRID_COMMIT_OK:" + String(dof) + " M=" + String(M) + " N=" + String(N));
  return true;
}

void JointController::abortGridCapture() {
  // Discard the scratch only — a previously-committed LIVE grid must survive an abort, so we never
  // touch dof_grids[dof] here.
  _grid_capture_active = false;
  _grid_scratch.bl_valid = false;
  LOG_C1_INFO("GRID_CAPTURE_ABORT");
}

// REMOVED: obsolete testPidDirection function
// Now we use static parameters agonist_drives_positive_movement from config_presets.h

// Apply necessary torques to reach target position
void JointController::applyTorquesForTargetPosition(AutoMappingState_t &auto_mapping_state) {
  // For each DOF, determine which motor must move at constant speed and which must
  // apply resistance torque
  bool all_dofs_reached = true;

  // Initialize arrays on first call
  if (auto_mapping_state.first_call) {
    auto_mapping_state.first_call = false;

    // Initialize all arrays to zero
    for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
      int i = auto_mapping_state.active_dof_indices[idx];
      auto_mapping_state.dof_movement_initialized[i] = false;
      auto_mapping_state.last_direction[i]           = 0;
      auto_mapping_state.last_agonist_value[i]       = 0.0f;
      auto_mapping_state.last_antagonist_value[i]    = 0.0f;
      auto_mapping_state.direction_check_counter[i]  = 0;
      auto_mapping_state.previous_angle[i]           = 0.0f;
      auto_mapping_state.initial_error[i]            = 0.0f;
      auto_mapping_state.debug_counter[i]            = 0;
      auto_mapping_state.dof_reach_logged[i]         = false;
    }
  }

  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    // Get current DOF angle from shared state (updated by Core0)
    if (!shared_dof_angles.valid[i]) {
      LOG_C1_ERROR("Invalid encoder reading for DOF " + String(i));
      continue;
    }
    float current_angle = shared_dof_angles.angles[i];

    // Target angle for this DOF
    float target_angle = auto_mapping_state.target_angles[i];

    // Calculate error (difference between target and current position)
    float angle_error = target_angle - current_angle;

    // Initialize previous_angle and initial_error the first time for this DOF
    if (!auto_mapping_state.dof_movement_initialized[i]) {
      auto_mapping_state.previous_angle[i] = current_angle;
      auto_mapping_state.initial_error[i]  = angle_error;
    }

    // Find motors associated with this DOF
    int agonist_index    = -1;
    int antagonist_index = -1;

    // Find indices of agonist and antagonist motors for this DOF
    for (int j = 0; j < auto_mapping_state.motor_count; j++) {
      if (config.motors[j].dof_index == i) {
        if (config.motors[j].is_agonist) {
          agonist_index = j;
        } else {
          antagonist_index = j;
        }
      }
    }

    // Verify that both motors were found
    if (agonist_index == -1 || antagonist_index == -1) {
    LOG_C1_ERROR("Agonist or antagonist motors not found for DOF " + String(i));
      continue;
    }

    // Get control parameters from configuration
    float motor_speed        = config.dofs[i].zero_mapping.auto_mapping_speed;
    float resistance_torque  = config.dofs[i].zero_mapping.auto_mapping_resistance_torque;
    float position_threshold = config.dofs[i].zero_mapping.position_threshold;

    // Verify we're going in the right direction - use DOF-specific counters
    auto_mapping_state.direction_check_counter[i]++;

    // Every 100000 cycles, verify we're moving in the right direction
    if (auto_mapping_state.direction_check_counter[i] >= 100000) {
      auto_mapping_state.direction_check_counter[i] = 0;

      // Calculate angle variation
      float angle_change                   = current_angle - auto_mapping_state.previous_angle[i];
      auto_mapping_state.previous_angle[i] = current_angle;

      // Debug to see actual values
      SERIAL_C1_COM("DOF ");
      SERIAL_C1_COM(i);
      SERIAL_C1_COM(": initial_error=");
      SERIAL_C1_COM(auto_mapping_state.initial_error[i]);
      SERIAL_C1_COM(", angle_change=");
      SERIAL_C1_COM(angle_change);
      SERIAL_C1_COM(", current=");
      SERIAL_C1_COM(current_angle);
      SERIAL_C1_COM(", product=");
      SERIAL_C1_COM_LN(angle_change * auto_mapping_state.initial_error[i]);

      // Verify we haven't exceeded DOF limits
      if (!isAngleInLimits(i, current_angle)) {
        LOG_C1_ERROR("Angle out of limits for DOF " + String(i));
        LOG_C1_DEBUG(String("Angle: ") + String(current_angle) + ", Min: " +
                  String(config.dofs[i].limits.min_angle) + ", Max: " +
                  String(config.dofs[i].limits.max_angle));

        // Stop motors and set error flag
        stopAllMotors();
        auto_mapping_state.active = false;
        return;
      }
    }

    // Verify if this DOF has reached its target
    bool dof_reached = fabs(angle_error) <= position_threshold;

    if (dof_reached) {
      // This DOF has reached the target, stop its motors if not already stopped
      if (auto_mapping_state.dof_movement_initialized[i]) {
        motors[agonist_index]->motorStop();
        motors[antagonist_index]->motorStop();
        auto_mapping_state.dof_movement_initialized[i] =
            false; // Reset initialization flag

        // Logging and flag to avoid oscillations
        if (!auto_mapping_state.dof_reach_logged[i]) {
          auto_mapping_state.dof_reach_logged[i] = true;
          LOG_C1_INFO("DOF " + String(i) +
                   " reached target. Motors definitively stopped.");
        }
      }
      // DOF reached, doesn't influence all_dofs_reached flag (considered reached)
    } else {
      // IMPORTANT: If the DOF has already reached the target once (dof_reach_logged = true),
      // consider DOF as reached to avoid oscillations, even if it temporarily exits
      // the threshold
      if (auto_mapping_state.dof_reach_logged[i]) {
        // DOF previously reached, keep motors stopped to avoid oscillations
        if (auto_mapping_state.dof_movement_initialized[i]) {
          motors[agonist_index]->motorStop();
          motors[antagonist_index]->motorStop();
          auto_mapping_state.dof_movement_initialized[i] = false;
        }
        // DOF still considered reached (doesn't influence all_dofs_reached)
        // Skip rest of movement logic for this DOF
        continue;
      }

      // This DOF hasn't reached the target for the first time yet
      all_dofs_reached = false;

      // Determine error direction: 1 = increase angle, -1 = decrease angle
      int current_direction = (angle_error > 0) ? 1 : -1;

      // Send commands only if necessary:
      // 1. When the DOF has not yet been initialized
      // 2. When direction has changed
      bool needs_update = (!auto_mapping_state.dof_movement_initialized[i] ||
                           (current_direction != auto_mapping_state.last_direction[i]));

      if (needs_update) {
        // Store new direction
        auto_mapping_state.last_direction[i]           = current_direction;
        auto_mapping_state.dof_movement_initialized[i] = true;

        // Save initial error for this DOF
        auto_mapping_state.initial_error[i] = angle_error;

        // Decide which motor must move and which must resist based on error direction
        // NEW: Check if inversion logic is requested
        bool invert_logic = config.dofs[i].zero_mapping.auto_mapping_invert_direction;
        
        // Effective direction is flipped if invert_logic is true
        // current_direction > 0 means "increase angle"
        // If inverted: treat "increase angle" as "decrease angle" logic
        bool increase_angle_logic = (current_direction > 0) ^ invert_logic;

        if (increase_angle_logic) {
          // We need to increase DOF angle (target > current) [OR inverted logic]
          // Stop motors first to avoid conflicting commands
          motors[agonist_index]->motorStop();
          motors[antagonist_index]->motorStop();
          delay(5); // Brief pause to stabilize

          // Antagonist moves, agonist resists
          motors[antagonist_index]->setSpeed(-motor_speed); // Positive direction
          motors[agonist_index]->setTorque(resistance_torque);

          // Store values
          auto_mapping_state.last_agonist_value[i] = resistance_torque;
          auto_mapping_state.last_antagonist_value[i] =
              -motor_speed; // For simplicity, store the speed

          // Save applied values for debug
          auto_mapping_state.applied_torques[antagonist_index] = 0; // Speed, not torque
          auto_mapping_state.applied_torques[agonist_index]    = resistance_torque;

          SERIAL_C1_COM_LN("DOF " + String(i) + ": Changed direction, now " + 
                         (invert_logic ? "decreasing (inverted logic)" : "increasing") + " angle");
        } else {
          // We need to decrease DOF angle (target < current) [OR inverted logic]
          // Stop motors first to avoid conflicting commands
          motors[agonist_index]->motorStop();
          motors[antagonist_index]->motorStop();
          delay(5); // Brief pause to stabilize

          // Agonist moves, antagonist resists
          motors[agonist_index]->setSpeed(motor_speed); // Negative direction
          motors[antagonist_index]->setTorque(-resistance_torque);

          // Store values
          auto_mapping_state.last_agonist_value[i] =
              motor_speed; // For simplicity, store the speed
          auto_mapping_state.last_antagonist_value[i] = -resistance_torque;

          // Save applied values for debug
          auto_mapping_state.applied_torques[agonist_index]    = 0; // Speed, not torque
          auto_mapping_state.applied_torques[antagonist_index] = -resistance_torque;

          SERIAL_C1_COM_LN("DOF " + String(i) + ": Changed direction, now " + 
                         (invert_logic ? "increasing (inverted logic)" : "decreasing") + " angle");
        }
      }

      // Debug info every 500000 cycles - use DOF-specific counter
      auto_mapping_state.debug_counter[i]++;
      if (auto_mapping_state.debug_counter[i] >= 500000) {
        auto_mapping_state.debug_counter[i] = 0;
        SERIAL_C1_COM_LN("DOF " + String(i) + ": target=" + String(target_angle, 2) + ", current=" +
                       String(current_angle, 2) + ", error=" + String(angle_error, 2));

        // Direction determines which values to print
        if (auto_mapping_state.last_direction[i] > 0) {
          SERIAL_C1_COM_LN("  Antagonist (motor " + String(antagonist_index) +
                         ") moving at speed " + String(-motor_speed) +
                         ", Agonist (motor " + String(agonist_index) +
                         ") resisting with torque " + String(resistance_torque));
        } else {
          SERIAL_C1_COM_LN("  Agonist (motor " + String(agonist_index) +
                         ") moving at speed " + String(motor_speed) +
                         ", Antagonist (motor " + String(antagonist_index) +
                         ") resisting with torque " + String(-resistance_torque));
        }
      }
    }
  }

  // Update global flag if all DOFs have reached position
  auto_mapping_state.position_reached = all_dofs_reached;
}

// Verify whether the current position is stable
bool JointController::isPositionReached(AutoMappingState_t &auto_mapping_state) {
  bool all_reached = true;

  // Check stability for each DOF using shared state (updated by Core0)
  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int i = auto_mapping_state.active_dof_indices[idx];
    if (!shared_dof_angles.valid[i]) {
      return false; // Invalid encoder reading
    }
    float current_angle = shared_dof_angles.angles[i];

    // Calculate error relative to target
    float error = fabs(auto_mapping_state.target_angles[i] - current_angle);

    // Get position threshold from configuration
    float position_threshold = config.dofs[i].zero_mapping.position_threshold;

    // Check whether the error is within the threshold
    if (error > position_threshold) {
      all_reached = false;
      break;
    }
  }

  return all_reached;
}

// Stop automatic mapping
bool JointController::stopAutoMapping(AutoMappingState_t &auto_mapping_state) {
  // Deactivate auto mapping
  auto_mapping_state.active = false;
  
  // Reset counters
  resetAutoMappingCounters(auto_mapping_state);

  // Stop all motors
  stopAllMotors();

  LOG_C1_INFO("Auto mapping stopped manually");

  return true;
}

// Reset counters used in auto mapping
void JointController::resetAutoMappingCounters(AutoMappingState_t &auto_mapping_state) {
  // Reset counters in auto_mapping_state structure
  auto_mapping_state.consecutive_encoder_errors = 0;
  auto_mapping_state.last_valid_reading         = millis();
  LOG_C1_DEBUG("Auto mapping counters reset");
}

// Update automatic mapping state
int JointController::updateAutoMapping(AutoMappingState_t &auto_mapping_state) {
  // Using class members instead of static variables

  // Check if commands are incoming with new system
  if (emergency_stop_requested) {
    // Stop all motors immediately and abort mapping
    LOG_C1_ERROR("EMERGENCY STOP received during auto mapping — immediate abort");
    stopAllMotors();
    auto_mapping_state.active = false;
    resetAutoMappingCounters(auto_mapping_state);
    return 3; // Error/interruption
  }

  if (buffer_ready[active_buffer]) {
    uint8_t cmd = pending_command_type;
    if (cmd == CMD_STOP || cmd == CMD_STOP_AUTO_MAPPING) {
      // Stop all motors immediately and abort mapping
      LOG_C1_WARN("STOP command received during auto mapping — immediate abort");
      stopAllMotors();
      auto_mapping_state.active = false;
      resetAutoMappingCounters(auto_mapping_state);
      return 3; // Error/interruption
    }
  }

  // Verify whether mapping is active
  if (!auto_mapping_state.active) {
    resetAutoMappingCounters(auto_mapping_state);
    return 3; // Error
  }

  // Global safety timeout - abort if stuck for too long
  unsigned long time_diff = millis() - auto_mapping_state.last_valid_reading;
  if (time_diff > 5000) { // 5 seconds without valid readings
    SERIAL_C1_COM_LN("!!!!! SAFETY TIMEOUT TRIGGERED !!!!!");
    LOG_C1_ERROR("Safety timeout during auto mapping — aborting");
    SERIAL_C1_COM("Last valid timestamp: ");
    SERIAL_C1_COM_LN(auto_mapping_state.last_valid_reading);
    SERIAL_C1_COM("Current timestamp: ");
    SERIAL_C1_COM_LN(millis());
    SERIAL_C1_COM("Difference: ");
    SERIAL_C1_COM_LN(time_diff);
    stopAllMotors();
    auto_mapping_state.active = false;
    resetAutoMappingCounters(auto_mapping_state);
    return 3; // Error/interruption
  }
  
  // If position not yet reached, apply necessary torques
  if (!auto_mapping_state.position_reached) {
    // Verify whether the position is now reached
    bool position_check_ok = true;

    // Verify that all encoders are valid using shared state (updated by Core0)
    for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
      int i = auto_mapping_state.active_dof_indices[idx];
      if (!shared_dof_angles.valid[i]) {
        position_check_ok = false;
        auto_mapping_state.consecutive_encoder_errors++;

        // If there are too many consecutive errors, abort
        if (auto_mapping_state.consecutive_encoder_errors > 10) {
          LOG_C1_ERROR("Too many consecutive encoder read errors — aborting auto mapping");
          stopAllMotors();
          auto_mapping_state.active = false;
          resetAutoMappingCounters(auto_mapping_state);
          return 3; // Error
        }

        LOG_C1_WARN("Encoder read error during position check, attempt " +
                 String(auto_mapping_state.consecutive_encoder_errors) + "/10");

        // Continue to next cycle to give another chance
        return 0;
      }
    }

    // If encoders are valid, check if we've reached the position
    if (position_check_ok) {
      // Reset error counter if we get here
      auto_mapping_state.consecutive_encoder_errors = 0;
      auto_mapping_state.last_valid_reading         = millis();

      auto_mapping_state.position_reached = isPositionReached(auto_mapping_state);

      if (auto_mapping_state.position_reached) {
        // Position reached - acquire point IMMEDIATELY without stopping
        // The actual angle is recorded (even if slightly different from target)
        // Linear regression works with any (motor_angle, joint_angle) pairs
        LOG_C1_INFO("Position reached for point " + String(auto_mapping_state.current_point) +
                 " - acquiring on-the-fly");
      } else {
        // Position not yet reached, apply necessary torques
        applyTorquesForTargetPosition(auto_mapping_state);

        return 0; // In progress
      }
    }
  }

  // Position reached - acquire point immediately (no settle time needed)
  if (auto_mapping_state.position_reached) {
    // Acquire point on-the-fly - motors keep moving
    if (acquireCurrentPoint(auto_mapping_state)) {
      // Point acquired successfully, reset error counter
      auto_mapping_state.consecutive_encoder_errors = 0;
      auto_mapping_state.last_valid_reading         = millis();

      // Increment counter
      auto_mapping_state.current_point++;

      // Verify whether mapping is complete
      if (auto_mapping_state.current_point >= auto_mapping_state.total_points) {
        // Mapping completed. STOP the motors FIRST: the final point is acquired
        // on-the-fly while the approach SET_SPEED is still LATCHED in the motors,
        // so without this stop they keep driving past the mapped range into the
        // mechanical stop (bench 2026-07-02: knee overran 95 -> 100.3+ deg until
        // the operator cut power). Same latched-command family as the Loop2 0xA4
        // teardown hardening.
        stopAllMotors();
        auto_mapping_state.active = false;
        resetAutoMappingCounters(auto_mapping_state); // Reset at end of mapping
        return 2;                                     // Completed
      }

      // Move to next point
      if (!moveToNextMappingPoint(auto_mapping_state)) {
        // Error moving to next point — stop the motors: the previous approach
        // SET_SPEED is still latched (same hole as the completion path above).
        stopAllMotors();
        auto_mapping_state.active = false;
        resetAutoMappingCounters(auto_mapping_state); // Reset on error
        return 3;                                     // Error
      }

      // Reset position flags for new point
      auto_mapping_state.position_reached = false;

      return 1; // Point acquired
    } else {
      // Error acquiring point
      auto_mapping_state.consecutive_encoder_errors++;
      if (auto_mapping_state.consecutive_encoder_errors > 5) {
        LOG_C1_ERROR("Too many consecutive errors acquiring point — aborting");
        stopAllMotors();
        auto_mapping_state.active = false;
        resetAutoMappingCounters(auto_mapping_state);
        return 3; // Error
      }

      LOG_C1_WARN("Point acquisition error, attempt " +
               String(auto_mapping_state.consecutive_encoder_errors) + "/5");

      // Don't abort immediately, try again
      return 0;
    }
  }

  // Position not yet reached, continue moving
  return 0; // In progress
}

// Transfer data from automatic mapping to DofMappingData_t structures
// IMPORTANT NOTE: These are RAW temporary data that will be replaced
// by PROCESSED (interpolated and extrapolated) data sent by Pi5
// NOTE: This function is called from Core1 - NO Serial/LOG calls allowed!
bool JointController::transferAutoMappingData(const AutoMappingState_t &auto_mapping_state) {
  // Verify that there's data to transfer
  if (auto_mapping_state.acquired_points_count <= 0) {
    return false;
  }

  // For each DOF, extract data and populate it in DofMappingData_t structure
  for (int idx = 0; idx < auto_mapping_state.active_dof_count; idx++) {
    int dof = auto_mapping_state.active_dof_indices[idx];
    DofMappingData_t &mapping_data = dof_mappings[dof];

    // A fresh coarse auto-map INVALIDATES any prior refined map for this DOF: the piecewise
    // fine-map and the bilinear grid were captured on the OLD mechanics (e.g. pre-tendon-swap).
    // This demote is RAM-ONLY (this cycle stops using the refined branches; demote FIRST,
    // barrier, then overwrite the raw buffer - publishBilinearGrid ordering). The FLASH side
    // is handled by core0 on AUTO_MAP_COMPLETE: after the v5 save it calls
    // invalidateRefinedMapsInFlash() so a reboot cannot re-promote the stale maps.
    linear_equations[dof].map_mode = MAP_LINEAR;
    __dmb();
    linear_equations[dof].pw_valid = false;
    if (dof < MAX_DOFS) {
      dof_grids[dof].bl_valid = false;
    }

    // Reset structure
    mapping_data.size = 0;
    mapping_data.flag = 0;

    // Find motors associated with this DOF
    int agonist_motor_index    = -1;
    int antagonist_motor_index = -1;

    for (int i = 0; i < config.motor_count; i++) {
      if (config.motors[i].dof_index == dof) {
        if (config.motors[i].is_agonist) {
          agonist_motor_index = i;
        } else {
          antagonist_motor_index = i;
        }
      }
    }

    if (agonist_motor_index == -1 || antagonist_motor_index == -1) {
      // Motors not found for DOF - skip silently (Core1 context)
      continue;
    }

    // Transfer data point by point
    for (int point = 0;
         point < auto_mapping_state.acquired_points_count && point < MAX_MAPPING_DATA_SIZE;
         point++) {
      const AutoMapPoint_t &acquired_point = auto_mapping_state.acquired_points[point];

      // Copy DOF data
      mapping_data.joint_data[point]      = acquired_point.dof_angles[dof];
      mapping_data.agonist_data[point]    = acquired_point.motor_angles[agonist_motor_index];
      mapping_data.antagonist_data[point] = acquired_point.motor_angles[antagonist_motor_index];

      mapping_data.size++;
    }

    // Sort data by increasing joint angle (simple bubble sort)
    for (int i = 0; i < mapping_data.size - 1; i++) {
      for (int j = 0; j < mapping_data.size - i - 1; j++) {
        if (mapping_data.joint_data[j] > mapping_data.joint_data[j + 1]) {
          // Swap joint_data
          float temp_joint               = mapping_data.joint_data[j];
          mapping_data.joint_data[j]     = mapping_data.joint_data[j + 1];
          mapping_data.joint_data[j + 1] = temp_joint;

          // Swap agonist_data
          float temp_agonist               = mapping_data.agonist_data[j];
          mapping_data.agonist_data[j]     = mapping_data.agonist_data[j + 1];
          mapping_data.agonist_data[j + 1] = temp_agonist;

          // Swap antagonist_data
          float temp_antagonist               = mapping_data.antagonist_data[j];
          mapping_data.antagonist_data[j]     = mapping_data.antagonist_data[j + 1];
          mapping_data.antagonist_data[j + 1] = temp_antagonist;
        }
      }
    }

    // Set flag to indicate that data is ready to send
    mapping_data.flag = 1;
  }

  return true;
}
