/**
 * @file core1.cpp
 * @brief Core1 execution loop - Movement control and hardware access
 *
 * This file contains the Core1 main loop responsible for:
 * - Processing movement commands from Core0 via inter-core buffers
 * - Executing hardware operations (motor control, auto-mapping, calibration)
 * - Handling emergency stops with immediate motor shutdown
 * - Managing smooth transitions between movements
 * - Communicating results back to Core0 via shared data structures
 *
 * Core1 runs in parallel with Core0, which handles serial communication.
 *
 * IMPORTANT: Core1 has exclusive access to motor hardware to prevent conflicts.
 *
 * @see main_common.h for shared data structures and global variables
 * @see JointController.h for movement and control methods
 */

#include "main_common.h"

// ============================================================================
// DUAL CAN BUS ARCHITECTURE
// ============================================================================
// J4 CAN_Servo (Motor CAN): GP9=CS, GP13=INT - Motor commands via LKM_Motor
// J5 CAN_Controller (Host CAN): GP8=CS, GP14=INT - Host commands (TimeSync, Waypoints)
// Both share SPI1 (GP10=SCK, GP11=MOSI, GP12=MISO) with different CS pins
// ============================================================================

// CAN ID ranges (from CAN_CONTROL_PROTOCOL.md - Priority-Optimized)
// Priority Level 0 (Highest): Emergency
#define CAN_ID_EMERGENCY_STOP 0x000

// Priority Level 1: System Control
#define CAN_ID_TIME_SYNC 0x002

// Priority Level 2: Motor Control (0x140-0x280) - handled by LKM_Motor library

// Priority Level 3: Trajectory Commands
#define CAN_ID_WAYPOINT_BASE 0x300  // 0x300-0x31F for waypoints (NEW: was 0x010)

// Priority Level 4: Status Feedback
#define CAN_ID_STATUS_BASE 0x400    // 0x400-0x4FF for status (NEW: was 0x200)

// Time synchronization state
// We store the host timestamp and local timestamp at sync time.
// To convert host time to local time: local = host - host_at_sync + local_at_sync
static volatile bool clock_synced = false;
static volatile uint32_t sync_host_ms = 0;   // Host timestamp at sync
static volatile uint32_t sync_local_ms = 0;  // Local millis() at sync

// Startup safety: require minimum uptime before accepting waypoints
// This prevents stale messages from executing after reset
static const uint32_t MIN_UPTIME_FOR_WAYPOINTS_MS = 2000;  // 2 seconds

/**
 * @brief Convert host timestamp to local time
 * @param host_time_ms Timestamp in host time reference
 * @return Equivalent timestamp in local millis() reference
 * 
 * Formula: local = (host_time - sync_host) + sync_local
 * This avoids overflow issues with large offsets.
 */
uint32_t hostTimeToLocal(uint32_t host_time_ms) {
  if (!clock_synced) {
    // Not synchronized - assume host time IS local time (for testing)
    return host_time_ms;
  }
  // Calculate relative offset from sync point
  // Using signed arithmetic to handle both directions
  int32_t delta_from_sync = (int32_t)(host_time_ms - sync_host_ms);
  return sync_local_ms + delta_from_sync;
}

/**
 * @brief Get current time in local milliseconds
 * @return Current local time (millis())
 */
uint32_t getAbsoluteTimeMs() {
  return millis();
}

/**
 * @brief Check if clock is synchronized with host
 */
bool isClockSynced() {
  return clock_synced;
}

/**
 * @brief Handle Time Sync frame from host
 * @param data CAN frame data (8 bytes)
 * @param len Frame length
 */
void handleTimeSyncFrame(const uint8_t *data, uint8_t len) {
  if (len < 8) {
    LOG_WARN("[CAN] Time Sync frame too short (" + String(len) + " bytes)");
    return;
  }

  // Parse timestamp (uint32_t, little-endian)
  uint32_t t_host_ms = 0;
  memcpy(&t_host_ms, data, sizeof(uint32_t));

  const uint32_t t_local = millis();
  sync_host_ms = t_host_ms;
  sync_local_ms = t_local;
  clock_synced = true;

  LOG_INFO("[CAN] Time sync: host=" + String(t_host_ms) + " local=" + String(t_local) + " (synced)");
}

/**
 * @brief Handle Waypoint frame from host
 * @param id CAN ID (encodes joint and DOF)
 * @param data CAN frame data (8 bytes)
 * @param len Frame length
 */
void handleWaypointFrame(uint32_t id, const uint8_t *data, uint8_t len) {
  if (len < 8) {
    LOG_WARN("[CAN] Waypoint frame too short (" + String(len) + " bytes)");
    return;
  }

  if (!clock_synced) {
    LOG_WARN("[CAN] Waypoint dropped: clock not synchronized");
    return;
  }
  
  // SAFETY: Require minimum uptime before accepting waypoints
  // This prevents stale messages from MCP2515 buffer from executing after reset
  if (millis() < MIN_UPTIME_FOR_WAYPOINTS_MS) {
    LOG_WARN("[CAN] Waypoint dropped: system startup (uptime=" + String(millis()) + "ms < " + 
             String(MIN_UPTIME_FOR_WAYPOINTS_MS) + "ms)");
    return;
  }

  // Parse waypoint (from CAN_CONTROL_PROTOCOL.md)
  struct {
    uint8_t dof_index;
    int16_t target_angle;    // 0.01° resolution
    uint32_t t_arrival_ms;   // Absolute arrival time
    uint8_t mode;            // LINEAR/SMOOTH
  } __attribute__((packed)) waypoint;

  memcpy(&waypoint, data, sizeof(waypoint));

  // Verify DOF index
  if (waypoint.dof_index >= waypoint_buffers_get_dof_count()) {
    LOG_WARN("[CAN] Waypoint DOF " + String(waypoint.dof_index) + " exceeds configured DOFs");
    return;
  }
  
  // SAFETY: Verify system is ready for movement (linear equations + calibrated offsets)
  if (active_joint_controller != nullptr && !active_joint_controller->isSystemReadyForMovement()) {
    LOG_ERROR("[CAN] Waypoint REJECTED: System not ready - run recalcOffset first!");
    return;
  }

  // Convert host timestamp to local time
  uint32_t t_arrival_local = hostTimeToLocal(waypoint.t_arrival_ms);
  
  // Convert to WaypointEntry
  WaypointEntry entry{};
  entry.dof_index = waypoint.dof_index;
  entry.target_angle_deg = static_cast<float>(waypoint.target_angle) / 100.0f;
  entry.t_arrival_ms = t_arrival_local;  // Store in local time reference
  entry.mode = waypoint.mode;

  // Check if this is the first waypoint for this DOF (transition from IDLE to MOVING)
  WaypointState current_state = waypoint_buffer_state(waypoint.dof_index);
  bool is_first_waypoint = (current_state == WaypointState::IDLE);

  // Push to buffer
  if (!waypoint_buffer_push(waypoint.dof_index, entry)) {
    LOG_WARN("[CAN] Waypoint buffer full for DOF " + String(waypoint.dof_index));
    return;
  }

  // Initialize movement if first waypoint OR resuming from HOLDING
  // In both cases, we need to set prev_angle/prev_time to current values
  bool needs_init = is_first_waypoint || (current_state == WaypointState::HOLDING);
  
  if (needs_init && active_joint_controller != nullptr) {
    // Get current joint angle as starting point
    bool is_valid = false;
    float current_angle = active_joint_controller->getCurrentAngle(waypoint.dof_index, is_valid);
    
    if (is_valid) {
      uint32_t t_now = getAbsoluteTimeMs();
      
      // === COMPREHENSIVE SAFETY CHECK ===
      // Checks: velocity limits, angle limits, time validity, mapping limits
      String safety_violation;
      if (!active_joint_controller->checkWaypointSafety(waypoint.dof_index, current_angle, 
                                                        entry.target_angle_deg, entry.t_arrival_ms, 
                                                        t_now, safety_violation)) {
        // Safety check failed - log error and trigger emergency stop
        LOG_ERROR("[CAN SAFETY] " + safety_violation);
        emergency_stop_requested = true;
        return; // Drop waypoint
      }
      
      // Safety check passed - initialize/update movement start point
      waypoint_buffer_set_prev(waypoint.dof_index, current_angle, t_now);
      waypoint_buffer_set_state(waypoint.dof_index, WaypointState::MOVING);
      
      if (is_first_waypoint) {
        LOG_INFO("[CAN] DOF " + String(waypoint.dof_index) + " transitioned IDLE → MOVING (start=" + 
                 String(current_angle, 3) + "° at t=" + String(t_now) + " ms)");
      } else {
        LOG_INFO("[CAN] DOF " + String(waypoint.dof_index) + " resumed HOLDING → MOVING (start=" + 
                 String(current_angle, 3) + "° at t=" + String(t_now) + " ms)");
      }
    } else {
      LOG_WARN("[CAN] Cannot read current angle for DOF " + String(waypoint.dof_index) + ", waypoint queued but state unchanged");
    }
  }

  uint32_t t_now = millis();
  int32_t time_until_arrival = (int32_t)entry.t_arrival_ms - (int32_t)t_now;
  LOG_INFO("[CAN] Waypoint queued DOF=" + String(waypoint.dof_index) + " angle=" + String(entry.target_angle_deg, 3) +
           " deg t_arrival=" + String(entry.t_arrival_ms) + " (in " + String(time_until_arrival) + " ms)");
}

/**
 * @brief Poll Host CAN bus for commands from Jetson/Host
 * 
 * This function polls the dedicated Host CAN bus (J5 CAN_Controller) for:
 * - Time Sync commands
 * - Waypoint commands
 * - Emergency Stop commands
 * 
 * The Motor CAN bus (J4 CAN_Servo) is handled separately by LKM_Motor.
 * Both share SPI1 but have different CS pins (GP8 for Host, GP9 for Motor).
 * 
 * Called from core1_loop() every iteration.
 */
void pollHostCan() {
  extern MCP_CAN CAN_HOST;  // Host CAN bus (separate from motor CAN)

  // Check if messages are available on Host CAN
  if (CAN_HOST.checkReceive() != CAN_MSGAVAIL) {
    return;
  }

  // Process up to 10 messages per poll to avoid blocking
  uint8_t msg_count = 0;
  while (CAN_HOST.checkReceive() == CAN_MSGAVAIL && msg_count < 10) {
    unsigned long rx_id = 0;
    unsigned char len = 0;
    unsigned char buf[8] = {0};

    if (CAN_HOST.readMsgBuf(&rx_id, &len, buf) != CAN_OK) {
      // Read error - skip this message
      break;
    }

    // Dispatch based on CAN ID
    if (rx_id == CAN_ID_TIME_SYNC) {
      handleTimeSyncFrame(buf, len);
    } else if (rx_id == CAN_ID_EMERGENCY_STOP) {
      LOG_WARN("[CAN_HOST] RX EMERGENCY_STOP frame");
      emergency_stop_requested = true;
    } else if (rx_id >= CAN_ID_WAYPOINT_BASE && rx_id < CAN_ID_STATUS_BASE) {
      handleWaypointFrame(rx_id, buf, len);
    }
    // Note: Motor responses (0x140+) are on the Motor CAN bus, not here

    msg_count++;
  }
}

/**
 * @brief Core1 main execution loop
 *
 * This function runs continuously on Core1 and processes movement commands
 * received from Core0 via inter-core communication buffers.
 *
 * Execution flow:
 * 1. Check for emergency stop requests (highest priority)
 * 2. Wait for new command in active buffer
 * 3. Validate command target (joint ID)
 * 4. Execute appropriate hardware operation
 * 5. Signal completion/error to Core0 via shared_data_ext
 *
 * Supported commands:
 * - CMD_STOP: Emergency stop all motors
 * - CMD_PRETENSION / CMD_PRETENSION_ALL: Apply tensioning torque
 * - CMD_RELEASE / CMD_RELEASE_ALL: Release tensioning torque
 * - CMD_SET_ZERO_CURRENT_POS: Set current position as zero
 * - CMD_MOVE_MULTI_DOF: Execute coordinated multi-DOF movement
 * - CMD_RECALC_OFFSET: Recalculate motor offset calibration
 * - CMD_START_AUTO_MAPPING: Start automatic joint calibration
 * - CMD_STOP_AUTO_MAPPING: Stop automatic joint calibration
 */
void core1_loop() {
  MovementResult last_movement_result; // To track last movement result

  // Timing for waypoint control @ 500 Hz (same as moveMultiDOF_cascade)
  const uint64_t SAMPLING_PERIOD_US = 2000; // 2 ms = 500 Hz
  uint64_t next_time = 0; // Will be initialized on first waypoint
  bool timing_initialized = false;

  while (true) {
    // === POLL HOST CAN BUS ===
    // Poll dedicated Host CAN (J5) for TimeSync, Waypoints, Emergency Stop
    // Motor CAN (J4) is handled by LKM_Motor during motor operations
    pollHostCan();

    // === WAYPOINT-BASED MOVEMENT ===
    // Execute waypoint trajectory for all DOFs (if waypoints available)
    // This runs @ 500 Hz with precise timing (outer loop @ 100 Hz, inner loop @ 500 Hz)
    bool waypoint_active = false;
    if (active_joint_controller != nullptr) {
      waypoint_active = active_joint_controller->executeWaypointMovement();
    }

    // === TIMING: Wait for next cycle @ 500 Hz ===
    // Only wait if waypoint control is active (to maintain precise timing)
    // If no waypoints, loop runs as fast as possible for responsive command handling
    if (waypoint_active) {
      // Initialize timing on first active waypoint
      if (!timing_initialized) {
        next_time = time_us_64() + SAMPLING_PERIOD_US;
        timing_initialized = true;
      }
      
      busy_wait_until(next_time);
      next_time += SAMPLING_PERIOD_US;
    } else {
      // Reset timing when waypoints stop (for next activation)
      timing_initialized = false;
    }

    // === EMERGENCY STOP CHECK ===
    if (emergency_stop_requested) {
      LOG_INFO("Core1: Emergency stop requested");

      // Stop all motors immediately
      // No SPI1 conflicts possible - Core1 has exclusive CAN access
      if (active_joint_controller != nullptr) {
        active_joint_controller->stopAllMotors();
        LOG_INFO("Core1: All motors stopped");
        
        // Clear all waypoint buffers to exit waypoint control loop
        for (uint8_t dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
          waypoint_buffer_clear(dof);
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
        }
        LOG_INFO("Core1: Waypoint buffers cleared");
      }

      // Reset flag
      emergency_stop_requested = false;
      smooth_transition_active = false;

      LOG_INFO("Core1: Emergency stop flag cleared");

      // Notify core0
      if (shared_data_ext.flag == 0) {
        shared_data_ext.flag = CMD1_END_MOVE;
        strcpy(shared_data_ext.message, "EMERGENCY STOP EXECUTED");
      }

      Serial.println("EMERGENCY STOP EXECUTED");
      continue;
    }

    // ============================================================================
    // AUTO-MAPPING CONTINUOUS PROCESSING (MUST RUN BEFORE COMMAND CHECK!)
    // ============================================================================
    // If auto-mapping is active, update the state machine continuously
    if (auto_mapping_state.active && active_joint_controller != nullptr) {
      int status = active_joint_controller->updateAutoMapping(auto_mapping_state);

      // Handle update result
      switch (status) {
      case 0: // In progress
        // Continue processing in next cycle
        break;

      case 1: // New point acquired
        // Continue processing - no flag needed for intermediate points
        break;

      case 2: // Mapping completed
        LOG_INFO("AUTO_MAP_COMPLETE: Auto-mapping completed successfully");

        // Transfer acquired data to DofMappingData_t structures
        if (active_joint_controller->transferAutoMappingData(auto_mapping_state)) {
          LOG_INFO("Auto-mapping data transferred successfully");

          // Compute linear equations from the raw mapping data just transferred
          LOG_INFO("Computing linear equations on Pico from raw data...");
          if (active_joint_controller->calculateLinearEquationsFromMappingData()) {
            LOG_INFO("PICO_LINEAR_EQUATIONS: Linear equations computed successfully");
            LOG_INFO("Compare these results with the Pi5 output for validation");
          } else {
            LOG_ERROR("ERROR: Unable to compute linear equations on Pico");
          }
        } else {
          LOG_ERROR("ERROR: Unable to transfer auto-mapping data");
        }

        // Signal completion
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Auto mapping completed successfully");
          shared_data_ext.flag = CMD1_AUTO_MAP_COMPLETE;
        }
        break;

      case 3: // Error
        LOG_ERROR("AUTO_MAP_ERROR: Error during auto-mapping");
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Auto-mapping error");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
        break;
      }
    }

    // === NEW COMMAND CHECK ===
    if (!buffer_ready[active_buffer]) {
      // No command available, wait
      sleep_us(100);
      continue;
    }

    // Read the command from the active buffer
    uint8_t command  = pending_command_type;
    command_data_ext = command_buffer[active_buffer]; // Atomic copy of structure

    // Reset buffer flag
    buffer_ready[active_buffer] = false;

    // Extract the components from the command data
    uint8_t joint_id  = command_data_ext.joint_id;
    uint8_t dof_index = command_data_ext.dof_index;

    // Save active controller and DOFs for this cycle
    current_joint_id  = joint_id;
    current_dof_index = dof_index;

    // Ensure the command targets the active joint
    if (joint_id != ACTIVE_JOINT && joint_id != 0) {
      // Not a command for this joint
      if (shared_data_ext.flag == 0) {
        shared_data_ext.flag = CMD1_FAIL_MOVE;
        strcpy(shared_data_ext.message, "ERROR: Command targeted to a different joint");
        // print joint id
        LOG_ERROR("Command targeted to a different joint: " + String(joint_id));
      }
      continue;
    }

    // Use the active controller
    JointController *controller = active_joint_controller;
    if (controller == nullptr) {
      // No active controller
      if (shared_data_ext.flag == 0) {
        shared_data_ext.flag = CMD1_FAIL_MOVE;
        strcpy(shared_data_ext.message, "ERROR: Controller not initialized");
      }
      continue;
    }

    // Execute appropriate command - only commands requiring hardware access
    switch (command) {
    case CMD_STOP:
      // Stop all motors of the controller
      controller->stopAllMotors();
      break;

    case CMD_PRETENSION: {
      // Pretension the motors of the specific DOF
      // Check for inverted logic (e.g. Knee joint)
      bool invert = controller->getConfig().dofs[dof_index].zero_mapping.auto_mapping_invert_direction;
      if (invert) {
        // Inverted logic: use RELEASE instead of PRETENSION
        controller->release(dof_index, command_data_ext.torque, command_data_ext.duration);
      } else {
        // Standard logic
        controller->pretension(dof_index, command_data_ext.torque, command_data_ext.duration);
      }
      break;
    }

    case CMD_PRETENSION_ALL:
      // Pretension all DOFs of the joint
      // Note: This assumes all DOFs share the same logic or controller handles it.
      // Ideally iterate through DOFs, but for now relying on standard implementation.
      // TODO: If mixed DOFs exist (inverted/standard), this needs per-DOF loop here.
      controller->pretensionAll();
      break;

    case CMD_RELEASE: {
      // Release the motors of the specific DOF
      // Check for inverted logic
      bool invert = controller->getConfig().dofs[dof_index].zero_mapping.auto_mapping_invert_direction;
      if (invert) {
        // Inverted logic: use PRETENSION instead of RELEASE
        controller->pretension(dof_index, command_data_ext.torque, command_data_ext.duration);
      } else {
        // Standard logic
        controller->release(dof_index, command_data_ext.torque, command_data_ext.duration);
      }
      break;
    }

    case CMD_RELEASE_ALL:
      // Release all DOFs of the joint with the configured parameters
      controller->releaseAll();
      break;

    case CMD_SET_ZERO_CURRENT_POS:
      // Set current position as zero for the specific DOF
      if (controller->setZeroCurrentPos(dof_index)) {
        // Signal the end of the zeroing procedure
        shared_data_ext.dof_index = dof_index;
        shared_data_ext.flag      = CMD1_END_ZERO;
        strcpy(shared_data_ext.message, "Current position set as zero for DOF");
      } else {
        // Error during zeroing
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "ERROR: Failed to set zero position");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
      }
      break;

    case CMD_MOVE_MULTI_DOF: {
      // SAFETY CHECK: Ensure the system is ready for movement
      if (!controller->isSystemReadyForMovement()) {
        // Block movement if linear equations are unavailable or offsets are not calibrated
        if (shared_data_ext.flag == 0) {
          strcpy(
              shared_data_ext.message,
              "SAFETY ERROR: System not ready - missing linear equations or uncalibrated offsets");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
        break;
      }

      // CRITICAL FIX: Wait for Core0 to reset flag before starting new movement
      // This prevents race condition where flag is still set from previous movement
      // Normal case: flag reset happens in < 1ms, timeout is just safety net
      uint32_t wait_counter = 0;
      const uint32_t MAX_WAIT_CYCLES = 500; // 50ms timeout (500 * 100µs)
      while (shared_data_ext.flag != 0 && wait_counter < MAX_WAIT_CYCLES) {
        sleep_us(100);
        wait_counter++;
      }

      if (shared_data_ext.flag != 0) {
        // Timeout: Core0 didn't reset flag in time - indicates serious issue
        LOG_ERROR("Core0 flag reset timeout - forcing reset to prevent deadlock");
        shared_data_ext.flag = 0;
      }

      // Determine whether this is a smooth transition move
      bool is_smooth_transition =
          (last_movement_result.exit_code == MOVEMENT_TRANSITION) || smooth_transition_active;

      if (is_smooth_transition) {
        LOG_INFO("SMOOTH TRANSITION ACTIVE - Motors already in motion");
      }

      // Execute the movement
      MovementResult move_multi_result = controller->moveMultiDOF_cascade(
          command_data_ext.target_angles, command_data_ext.active_dofs_mask,
          command_data_ext.path_type, command_data_ext.sync_strategy, command_data_ext.speed,
          command_data_ext.acceleration,
          controller->getConfig()
              .dofs[0]
              .motion.path_steps,   // number of steps from first DOF configuration
          controller->getConfig()
              .dofs[0]
              .motion.sampling_period, // sampling period from configuration
          false,  
          command_data_ext.torque > 0 ? command_data_ext.torque
                                      : 1500, // maximum torque customizable
          is_smooth_transition                // parameter to indicate smooth transition
      );

      // Prepare response data
      shared_data_ext.joint_id  = joint_id;
      shared_data_ext.dof_index = 0xFF; // Special value to indicate multi-DOF movement

      // NEW HANDLING: Interpret movement exit code
      switch (move_multi_result.exit_code) {
      case MOVEMENT_COMPLETED:
        // Movement completed successfully, position maintained
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Movement completed successfully");
          shared_data_ext.flag = CMD1_END_MOVE;
        }
        break;

      case MOVEMENT_TRANSITION:
        // SMOOTH TRANSITION: movement ended by transition, motors kept active
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "TRANSITION: Ready for seamless next movement");
          shared_data_ext.flag =
              CMD1_END_MOVE; // Signal completion to allow new command
        }
        // The motors remain active thanks to the code in moveMultiDOF
        break;

      case MOVEMENT_STOPPED:
        // Movement stopped by STOP command
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Movement stopped by STOP command");
          shared_data_ext.flag = CMD1_END_MOVE;
        }
        break;

      case MOVEMENT_CONTROL_COMMAND:
        // Movement interrupted by joint control command
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Movement interrupted by joint control command");
          shared_data_ext.flag = CMD1_END_MOVE;
        }
        break;

      case MOVEMENT_ERROR:
      case MOVEMENT_SAFETY_STOP:
      default:
        // Error in movement or safety stop
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Movement interrupted by error");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
        break;
      }

      // Save result for future transitions
      last_movement_result = move_multi_result;

      // Update smooth transition flag
      smooth_transition_active = (move_multi_result.exit_code == MOVEMENT_TRANSITION);
    } break;  

    
    case CMD_RECALC_OFFSET:
      // Recalculate the offsets of the motors for the specific DOF
      if (controller->recalculateMotorOffsets(
              dof_index,
              // Use parameters from command if provided, otherwise configuration defaults
              command_data_ext.recalc_offset_torque > 0
                  ? command_data_ext.recalc_offset_torque
                  : controller->getConfig().dofs[dof_index].zero_mapping.recalc_offset_torque,
              command_data_ext.recalc_offset_duration > 0
                  ? command_data_ext.recalc_offset_duration
                  : controller->getConfig().dofs[dof_index].zero_mapping.recalc_offset_duration)) {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Offsets recalculated successfully");
          shared_data_ext.flag = CMD1_END_MOVE;
        }
      } else {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Error recalculating offsets");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
      }
      break;

    case CMD_START_AUTO_MAPPING:
      // Start automatic mapping for the joint
      if (controller->startAutoMapping(auto_mapping_state, command_data_ext.tensioning_torque,
                                       command_data_ext.auto_mapping_steps,
                                       command_data_ext.auto_mapping_settle_time)) {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Automatic mapping started");
          shared_data_ext.flag = CMD1_AUTO_MAP_PROGRESS;
        }
      } else {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Error starting automatic mapping");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
      }
      break;

    case CMD_STOP_AUTO_MAPPING:
      // Stop automatic mapping
      if (controller->stopAutoMapping(auto_mapping_state)) {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Automatic mapping stopped");
          shared_data_ext.flag = CMD1_END_MOVE;
        }
      } else {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Error stopping automatic mapping");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
      }
      break;

    default:
      // Unrecognized command or not manageable on core1
      if (shared_data_ext.flag == 0) {
        strcpy(shared_data_ext.message, "Command cannot be handled on core1");
        shared_data_ext.flag = CMD1_FAIL_MOVE;
      }
      break;
    }

    // Short pause between cycles
    sleep_us(100);
  }
}

