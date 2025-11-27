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

// External variables for inter-core communication
extern volatile bool emergency_stop_requested;
extern volatile bool buffer_ready[2];
extern volatile int active_buffer;
extern volatile uint8_t pending_command_type;

// ============================================================================
// AUTOMATIC CALIBRATION (AUTO-MAPPING)
// ============================================================================

// Start automatic mapping for all DOFs of the joint
bool JointController::startAutoMapping(AutoMappingState_t &auto_mapping_state,
                                       float tensioning_torque, float *steps, int settle_time_ms) {
  // Validate input parameters
  if (config.dof_count == 0) {
    LOG_ERROR("No DOF configured for this joint");
    return false;
  }

  if (config.motor_count == 0) {
    LOG_ERROR("No motors configured for this joint");
    return false;
  }

  // Verify that each DOF has at least 2 motors (agonist and antagonist)
  for (int i = 0; i < config.dof_count; i++) {
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
      LOG_ERROR("DOF " + String(i) + " does not have correctly configured agonist/antagonist motors");
      return false;
    }
  }

  // Reset counters before starting
  auto_mapping_state.consecutive_encoder_errors = 0;
  auto_mapping_state.last_valid_reading         = millis();
  LOG_DEBUG("Initializing auto mapping counters");

  // Verify if auto‑mapping is already active
  if (auto_mapping_state.active) {
    LOG_ERROR("Auto mapping already in progress");
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
  LOG_DEBUG("DofMappingData_t structures reset for new mapping");

  // Set first_call flag to ensure array initialization
  auto_mapping_state.first_call = true;

  // Reset counters after memset
  auto_mapping_state.consecutive_encoder_errors = 0;
  auto_mapping_state.last_valid_reading         = millis();
  auto_mapping_state.acquired_points_count      = 0; // Initialize acquired points counter

  // Set number of DOFs and motors
  auto_mapping_state.dof_count   = config.dof_count;
  auto_mapping_state.motor_count = config.motor_count;

  // Set settle time
  auto_mapping_state.settle_time_ms =
      (settle_time_ms > 0) ? settle_time_ms : config.dofs[0].zero_mapping.auto_mapping_settle_time;

  // Set tensioning torque and active motor speed
  float applied_tensioning_torque =
      (tensioning_torque != 0) ? tensioning_torque : config.dofs[0].zero_mapping.tensioning_torque;

  // Get motor speed from configuration
  float motor_speed       = config.dofs[0].zero_mapping.auto_mapping_speed;
  float resistance_torque = config.dofs[0].zero_mapping.auto_mapping_resistance_torque;

  LOG_INFO("Starting auto mapping - DOF: " + String(auto_mapping_state.dof_count) +
           ", motors: " + String(auto_mapping_state.motor_count) +
           ", tensioning torque: " + String(applied_tensioning_torque) +
           ", motor speed: " + String(motor_speed) + " deg/s" +
           ", resistance torque: " + String(resistance_torque));

  // Set limits and steps for each DOF
  auto_mapping_state.total_points = 1; // Initialize to 1 and multiply for each DOF

  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    // Use specific limits for auto-mapping instead of general limits
    auto_mapping_state.min_angles[i] = config.dofs[i].zero_mapping.auto_mapping_min_angle;
    auto_mapping_state.max_angles[i] = config.dofs[i].zero_mapping.auto_mapping_max_angle;

    // Verify that mapping limits are within joint's general limits
    if (auto_mapping_state.min_angles[i] < config.dofs[i].limits.min_angle) {
      LOG_WARN("auto_mapping_min_angle (" + String(auto_mapping_state.min_angles[i]) +
               ") is below joint min limit (" + String(config.dofs[i].limits.min_angle) +
               ") for DOF " + String(i) + ". Using joint limit.");
      auto_mapping_state.min_angles[i] = config.dofs[i].limits.min_angle;
    }

    if (auto_mapping_state.max_angles[i] > config.dofs[i].limits.max_angle) {
      LOG_WARN("auto_mapping_max_angle (" + String(auto_mapping_state.max_angles[i]) +
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
    int dof_points = ceil((auto_mapping_state.max_angles[i] - auto_mapping_state.min_angles[i]) /
                          auto_mapping_state.step_sizes[i]) +
                     1;

    // Update total points (cartesian product)
    auto_mapping_state.total_points *= dof_points;

    // Verify total number of points does not exceed memory capacity
    if (auto_mapping_state.total_points > MAX_MAPPING_DATA_SIZE) {
      LOG_ERROR("Total number of points (" + String(auto_mapping_state.total_points) +
                ") exceeds maximum capacity (" + String(MAX_MAPPING_DATA_SIZE) + ")");
      LOG_WARN("Reduce mapping step or angle range");
      return false;
    }

    // Start from minimum angle
    auto_mapping_state.target_angles[i] = auto_mapping_state.min_angles[i];

    LOG_DEBUG("DOF " + String(i) + ": mapping range [" + String(auto_mapping_state.min_angles[i]) +
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
    // Initial setting: tensioning torques based on motor role (agonist/antagonist)
    if (config.motors[i].is_agonist) {
      // Agonist motors
      auto_mapping_state.applied_torques[i] = applied_tensioning_torque;
    } else {
      // Antagonist motors
      auto_mapping_state.applied_torques[i] = -applied_tensioning_torque;
    }
  }

  // Stop all motors before starting for safety
  LOG_INFO("Stopping all motors before starting auto mapping");
  stopAllMotors();
  sleep_ms(500); // Brief pause to ensure motors stop completely

  // Apply initial torques to all motors
  LOG_INFO("Applying initial torques for auto mapping");
  for (int i = 0; i < auto_mapping_state.motor_count; i++) {
    motors[i]->setTorque(auto_mapping_state.applied_torques[i]);
    LOG_DEBUG("  Motor " + String(i) + ": torque " +
              String(auto_mapping_state.applied_torques[i]));
  }

  // Activate mapping
  auto_mapping_state.active = true;

  // Print mapping info
  LOG_INFO("Auto mapping started — Total points: " + String(auto_mapping_state.total_points) +
           ", settle time: " + String(auto_mapping_state.settle_time_ms) + "ms");

  return true;
}

// Move to next mapping point
bool JointController::moveToNextMappingPoint(AutoMappingState_t &auto_mapping_state) {
  // Print detailed information about current point and transition to next
  LOG_DEBUG("\n----------------------------------------------------------");
  LOG_DEBUG("MAPPING POINT CHANGE — Current point: " + String(auto_mapping_state.current_point));
  LOG_DEBUG("Current position:");
  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    bool isValid;
    float current_angle = getCurrentAngle(i, isValid);
    LOG_DEBUG("  DOF " + String(i) + ": " + String(current_angle, 2) +
              "° (target was: " + String(auto_mapping_state.target_angles[i], 2) + "°)");
  }

  // Reset of PID controllers no longer needed because PID is not used for automatic mapping

  // Pause before proceeding to next point
  const int TRANSITION_DELAY_MS = 2000; // 2 seconds pause
  LOG_DEBUG("Waiting " + String(TRANSITION_DELAY_MS) + "ms before proceeding...");
  sleep_ms(TRANSITION_DELAY_MS);

  // Multidimensional increment, like nested counters
  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    // Increment target angle for this DOF
    auto_mapping_state.target_angles[i] += auto_mapping_state.step_sizes[i];

    // If we haven't exceeded the maximum for this DOF, we've found the next point
    if (auto_mapping_state.target_angles[i] <= auto_mapping_state.max_angles[i]) {
      // Target updated, continue with new point
      LOG_DEBUG("New target:");
      for (int j = 0; j < auto_mapping_state.dof_count; j++) {
        LOG_DEBUG("  DOF " + String(j) + ": " + String(auto_mapping_state.target_angles[j], 2) +
                  "°");
      }

      // Explicit reset of all movement flags for each DOF
      for (int k = 0; k < auto_mapping_state.dof_count; k++) {
        auto_mapping_state.dof_movement_initialized[k] = false;
        auto_mapping_state.last_direction[k]           = 0;
        auto_mapping_state.direction_check_counter[k]  = 0;
        auto_mapping_state.debug_counter[k]            = 0;
        auto_mapping_state.dof_reach_logged[k]         = false;
      }
      LOG_DEBUG("Explicit reset of movement flags for all DOFs");

      // Also reset global timeout counter
      auto_mapping_state.last_valid_reading = millis();

      LOG_DEBUG("----------------------------------------------------------\n");
      return true;
    }

    // Reset this DOF to minimum and move to next DOF
    auto_mapping_state.target_angles[i] = auto_mapping_state.min_angles[i];
    LOG_DEBUG("Reset DOF " + String(i) + " to minimum: " +
              String(auto_mapping_state.min_angles[i], 2) + "°");
  }

  // If we get here, we've completed all DOFs
  LOG_INFO("All points explored");
  LOG_DEBUG("----------------------------------------------------------\n");
  return false;
}

// Acquire current point in automatic mapping
bool JointController::acquireCurrentPoint(AutoMappingState_t &auto_mapping_state) {
  // Verify that there's space in the array to save the point
  if (auto_mapping_state.acquired_points_count >= MAX_MAPPING_DATA_SIZE) {
    LOG_ERROR("Acquired points array full");
    return false;
  }

  // Populate structure with current data
  auto_mapping_state.last_sample.timestamp = millis();

  // Acquire DOF angles
  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    bool isValid;
    auto_mapping_state.last_sample.dof_angles[i] = getCurrentAngle(i, isValid);

    if (!isValid) {
      LOG_ERROR("Acquisition failed — invalid encoder reading for DOF " + String(i));
      return false;
    }
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
  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    point_data += String(auto_mapping_state.last_sample.dof_angles[i], 2);
    if (i < auto_mapping_state.dof_count - 1)
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

  LOG_DEBUG(point_data);

  // Calculate and print progress
  float progress =
      (float)auto_mapping_state.current_point / auto_mapping_state.total_points * 100.0f;
  if (auto_mapping_state.current_point % 10 == 0 || (int)progress % 5 == 0) {
    LOG_INFO("AUTO_MAP_PROGRESS:" + String(progress, 1) + "% - " +
             String(auto_mapping_state.current_point) + "/" +
             String(auto_mapping_state.total_points) + " points");
  }

  return true;
}

// Helper function for interpolation (wrapper to global interpolate_data)
float JointController::interpolate_data(float target_value, float *data1, float *data2, int size) {
  return ::interpolate_data(target_value, data1, data2, size);
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
    for (int i = 0; i < auto_mapping_state.dof_count; i++) {
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

  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    // Get current DOF angle
    bool isValid;
    float current_angle = getCurrentAngle(i, isValid);

    if (!isValid) {
    LOG_ERROR("Invalid encoder reading for DOF " + String(i));
      continue;
    }

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
    LOG_ERROR("Agonist or antagonist motors not found for DOF " + String(i));
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
      Serial.print("DOF ");
      Serial.print(i);
      Serial.print(": initial_error=");
      Serial.print(auto_mapping_state.initial_error[i]);
      Serial.print(", angle_change=");
      Serial.print(angle_change);
      Serial.print(", current=");
      Serial.print(current_angle);
      Serial.print(", product=");
      Serial.println(angle_change * auto_mapping_state.initial_error[i]);

      // Verify we haven't exceeded DOF limits
      if (!isAngleInLimits(i, current_angle)) {
        LOG_ERROR("Angle out of limits for DOF " + String(i));
        LOG_DEBUG(String("Angle: ") + String(current_angle) + ", Min: " +
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
          LOG_INFO("DOF " + String(i) +
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

          Serial.println("DOF " + String(i) + ": Changed direction, now " + 
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

          Serial.println("DOF " + String(i) + ": Changed direction, now " + 
                         (invert_logic ? "increasing (inverted logic)" : "decreasing") + " angle");
        }
      }

      // Debug info every 500000 cycles - use DOF-specific counter
      auto_mapping_state.debug_counter[i]++;
      if (auto_mapping_state.debug_counter[i] >= 500000) {
        auto_mapping_state.debug_counter[i] = 0;
        Serial.println("DOF " + String(i) + ": target=" + String(target_angle, 2) + ", current=" +
                       String(current_angle, 2) + ", error=" + String(angle_error, 2));

        // Direction determines which values to print
        if (auto_mapping_state.last_direction[i] > 0) {
          Serial.println("  Antagonist (motor " + String(antagonist_index) +
                         ") moving at speed " + String(-motor_speed) +
                         ", Agonist (motor " + String(agonist_index) +
                         ") resisting with torque " + String(resistance_torque));
        } else {
          Serial.println("  Agonist (motor " + String(agonist_index) +
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

  // Check stability for each DOF
  for (int i = 0; i < auto_mapping_state.dof_count; i++) {
    bool isValid;
    float current_angle = getCurrentAngle(i, isValid);

    if (!isValid) {
      return false; // Invalid encoder reading
    }

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

  LOG_INFO("Auto mapping stopped manually");

  return true;
}

// Reset counters used in auto mapping
void JointController::resetAutoMappingCounters(AutoMappingState_t &auto_mapping_state) {
  // Reset counters in auto_mapping_state structure
  auto_mapping_state.consecutive_encoder_errors = 0;
  auto_mapping_state.last_valid_reading         = millis();
  LOG_DEBUG("Auto mapping counters reset");
}

// Update automatic mapping state
int JointController::updateAutoMapping(AutoMappingState_t &auto_mapping_state) {
  // Using class members instead of static variables

  // Check if commands are incoming with new system
  if (emergency_stop_requested) {
    // Stop all motors immediately and abort mapping
    LOG_ERROR("EMERGENCY STOP received during auto mapping — immediate abort");
    stopAllMotors();
    auto_mapping_state.active = false;
    resetAutoMappingCounters(auto_mapping_state);
    return 3; // Error/interruption
  }

  if (buffer_ready[active_buffer]) {
    uint8_t cmd = pending_command_type;
    if (cmd == CMD_STOP || cmd == CMD_STOP_AUTO_MAPPING) {
      // Stop all motors immediately and abort mapping
      LOG_WARN("STOP command received during auto mapping — immediate abort");
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
    Serial.println("!!!!! SAFETY TIMEOUT TRIGGERED !!!!!");
    LOG_ERROR("Safety timeout during auto mapping — aborting");
    Serial.print("Last valid timestamp: ");
    Serial.println(auto_mapping_state.last_valid_reading);
    Serial.print("Current timestamp: ");
    Serial.println(millis());
    Serial.print("Difference: ");
    Serial.println(time_diff);
    stopAllMotors();
    auto_mapping_state.active = false;
    resetAutoMappingCounters(auto_mapping_state);
    return 3; // Error/interruption
  }
  
  // If position not yet reached, apply necessary torques
  if (!auto_mapping_state.position_reached) {
    // Verify whether the position is now reached
    bool position_check_ok = true;

    // Verify that all encoders are valid
    for (int i = 0; i < auto_mapping_state.dof_count; i++) {
      bool isValid;
      getCurrentAngle(i, isValid);
      if (!isValid) {
        position_check_ok = false;
        auto_mapping_state.consecutive_encoder_errors++;

        // If there are too many consecutive errors, abort
        if (auto_mapping_state.consecutive_encoder_errors > 10) {
          LOG_ERROR("Too many consecutive encoder read errors — aborting auto mapping");
          stopAllMotors();
          auto_mapping_state.active = false;
          resetAutoMappingCounters(auto_mapping_state);
          return 3; // Error
        }

        LOG_WARN("Encoder read error during position check, attempt " +
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
        // Position reached, stop motors immediately
        stopAllMotors();

        // Start counting for stabilization
        auto_mapping_state.settle_start_time = millis();
        LOG_INFO("Position reached for point " + String(auto_mapping_state.current_point) +
                 ", motors stopped, waiting " + String(auto_mapping_state.settle_time_ms) + "ms");
      } else {
        // Position not yet reached, apply necessary torques
        applyTorquesForTargetPosition(auto_mapping_state);

        return 0; // In progress
      }
    }
  }

  // Position reached, verify whether settle time has passed
  if (millis() - auto_mapping_state.settle_start_time >= auto_mapping_state.settle_time_ms) {

    // Before acquiring point, apply pretension torques to both motors
    LOG_INFO("Applying pretension torques for point acquisition");
    for (int i = 0; i < auto_mapping_state.dof_count; i++) {
      // Find motors for this DOF
      int agonist_index    = -1;
      int antagonist_index = -1;

      for (int j = 0; j < auto_mapping_state.motor_count; j++) {
        if (config.motors[j].dof_index == i) {
          if (agonist_index == -1) {
            agonist_index = j;
          } else {
            antagonist_index = j;
            break;
          }
        }
      }

      if (agonist_index != -1 && antagonist_index != -1) {
        // Get pretension torque from configuration
        float pretension_torque = config.dofs[i].zero_mapping.tensioning_torque;

        // Apply opposite torques to create tension
        motors[agonist_index]->setTorque(pretension_torque);
        motors[antagonist_index]->setTorque(-pretension_torque);

        LOG_DEBUG("  DOF " + String(i) + ": agonist torque=" + String(pretension_torque) +
                  ", antagonist torque=" + String(-pretension_torque));
      }
    }

    // Wait briefly to stabilize with new torques
    const int PRETENSION_STABILIZE_MS = 500;
    LOG_DEBUG("Waiting " + String(PRETENSION_STABILIZE_MS) +
              "ms for stabilization with pretension torques");
    sleep_ms(PRETENSION_STABILIZE_MS);

    // Stabilization time completed, acquire point
    if (acquireCurrentPoint(auto_mapping_state)) {
      // Point acquired successfully, reset error counter
      auto_mapping_state.consecutive_encoder_errors = 0;
      auto_mapping_state.last_valid_reading         = millis();

      // Increment counter
      auto_mapping_state.current_point++;

      // Verify whether mapping is complete
      if (auto_mapping_state.current_point >= auto_mapping_state.total_points) {
        // Mapping completed
        auto_mapping_state.active = false;
        resetAutoMappingCounters(auto_mapping_state); // Reset at end of mapping
        return 2;                                     // Completed
      }

      // Move to next point
      if (!moveToNextMappingPoint(auto_mapping_state)) {
        // Error moving to next point
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
        LOG_ERROR("Too many consecutive errors acquiring point — aborting");
        stopAllMotors();
        auto_mapping_state.active = false;
        resetAutoMappingCounters(auto_mapping_state);
        return 3; // Error
      }

      LOG_WARN("Point acquisition error, attempt " +
               String(auto_mapping_state.consecutive_encoder_errors) + "/5");

      // Don't abort immediately, try again
      return 0;
    }
  }

  // Still waiting for stabilization
  return 0; // In progress
}

// Transfer data from automatic mapping to DofMappingData_t structures
// IMPORTANT NOTE: These are RAW temporary data that will be replaced
// by PROCESSED (interpolated and extrapolated) data sent by Pi5
bool JointController::transferAutoMappingData(const AutoMappingState_t &auto_mapping_state) {
  LOG_INFO("Transferring auto-mapping data...");
  LOG_DEBUG("NOTE: These are RAW data to be replaced by processed data from Pi5");

  // Verify that there's data to transfer
  if (auto_mapping_state.acquired_points_count <= 0) {
    LOG_ERROR("ERROR: No auto-mapping data available to transfer");
    return false;
  }

  Serial.println("Transferring " + String(auto_mapping_state.acquired_points_count) +
                 " raw points for " + String(auto_mapping_state.dof_count) + " DOF");

  // For each DOF, extract data and populate it in DofMappingData_t structure
  for (int dof = 0; dof < auto_mapping_state.dof_count; dof++) {
    DofMappingData_t &mapping_data = dof_mappings[dof];

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
      LOG_ERROR("Motors not found for DOF " + String(dof));
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

    Serial.println("DOF " + String(dof) + ": transferred " + String(mapping_data.size) +
                   " points (sorted)");
    LOG_INFO("  Joint angle range: " + String(mapping_data.joint_data[0], 2) + " -> " +
                   String(mapping_data.joint_data[mapping_data.size - 1], 2));
    LOG_INFO("  Agonist motor range: " + String(mapping_data.agonist_data[0], 2) + " -> " +
                   String(mapping_data.agonist_data[mapping_data.size - 1], 2));
    LOG_INFO("  Antagonist motor range: " + String(mapping_data.antagonist_data[0], 2) +
                   " -> " + String(mapping_data.antagonist_data[mapping_data.size - 1], 2));

    // CORRECTION: Set flag to indicate that data is ready to send
    mapping_data.flag = 1;
    LOG_DEBUG("DOF " + String(dof) + " flag set to 1 - data ready to send");
  }

  LOG_INFO("Data transfer completed successfully");
  return true;
}

