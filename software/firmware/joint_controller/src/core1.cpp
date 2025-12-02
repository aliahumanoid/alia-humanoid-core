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
#define CAN_ID_WAYPOINT_BASE 0x300       // 0x300-0x31F for single-DOF waypoints (legacy)
#define CAN_ID_MULTI_DOF_WAYPOINT_BASE 0x380  // 0x380-0x39F for multi-DOF waypoints (optimized)

// Priority Level 4: Status Feedback
#define CAN_ID_STATUS_BASE 0x400    // 0x400-0x4FF for status (NEW: was 0x200)

// Sentinel value for unused DOF in Multi-DOF waypoint
#define MULTI_DOF_UNUSED 0x7FFF

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
 * @brief Handle Multi-DOF Waypoint frame from host (optimized format)
 * 
 * This is the recommended format for production use, as it sends all DOFs
 * of a joint in a single CAN frame, reducing bus traffic by 66%.
 * 
 * Format (8 bytes):
 *   Byte 0-1: int16_t dof0_angle (0.01° resolution, 0x7FFF = unused)
 *   Byte 2-3: int16_t dof1_angle (0.01° resolution, 0x7FFF = unused)
 *   Byte 4-5: int16_t dof2_angle (0.01° resolution, 0x7FFF = unused)
 *   Byte 6-7: uint16_t t_offset_ms (offset from last time sync)
 * 
 * CAN ID: 0x380 + joint_id (this controller responds to its own joint_id)
 * 
 * @param id CAN ID (0x380 + joint_id)
 * @param data CAN frame data (8 bytes)
 * @param len Frame length
 * 
 * @see CAN_SYSTEM_ARCHITECTURE.md section 4.2.4
 */
void handleMultiDofWaypointFrame(uint32_t id, const uint8_t *data, uint8_t len) {
  if (len < 8) {
    LOG_WARN("[CAN] Multi-DOF Waypoint frame too short (" + String(len) + " bytes)");
    return;
  }

  if (!clock_synced) {
    LOG_WARN("[CAN] Multi-DOF Waypoint dropped: clock not synchronized");
    return;
  }
  
  // SAFETY: Require minimum uptime before accepting waypoints
  if (millis() < MIN_UPTIME_FOR_WAYPOINTS_MS) {
    LOG_WARN("[CAN] Multi-DOF Waypoint dropped: system startup");
    return;
  }
  
  // SAFETY: Verify system is ready for movement
  if (active_joint_controller != nullptr && !active_joint_controller->isSystemReadyForMovement()) {
    LOG_ERROR("[CAN] Multi-DOF Waypoint REJECTED: System not ready - run recalcOffset first!");
    return;
  }

  // Parse Multi-DOF waypoint
  // Format matches single-DOF behavior: t_offset_ms is relative to current time
  // This allows the same JS logic to work for both formats
  struct {
    int16_t dof0_angle;    // 0.01° resolution, 0x7FFF = unused
    int16_t dof1_angle;    // 0.01° resolution, 0x7FFF = unused
    int16_t dof2_angle;    // 0.01° resolution, 0x7FFF = unused
    uint16_t t_offset_ms;  // Offset from CURRENT host time (same as single-DOF arrival_offset)
  } __attribute__((packed)) multi_wp;

  memcpy(&multi_wp, data, sizeof(multi_wp));

  // Calculate absolute arrival time from offset
  // Like single-DOF: the host sends (current_host_time + offset), we convert to local
  // But here we receive just the offset, so we add it to current sync reference
  // To match single-DOF behavior: arrival_time = host_now + offset
  // Since we don't have host_now, we use: local_now + offset (clocks are synced)
  uint32_t t_now = getAbsoluteTimeMs();
  uint32_t t_arrival_local = t_now + multi_wp.t_offset_ms;

  // Get DOF count for this joint
  uint8_t dof_count = waypoint_buffers_get_dof_count();
  
  // Array of angle values for easy iteration
  int16_t angles[3] = {multi_wp.dof0_angle, multi_wp.dof1_angle, multi_wp.dof2_angle};
  
  // Process each DOF
  uint8_t queued_count = 0;
  for (uint8_t dof = 0; dof < 3 && dof < dof_count; dof++) {
    // Skip unused DOFs (sentinel value 0x7FFF)
    if (angles[dof] == MULTI_DOF_UNUSED) {
      continue;
    }
    
    // Create waypoint entry
    WaypointEntry entry{};
    entry.dof_index = dof;
    entry.target_angle_deg = static_cast<float>(angles[dof]) / 100.0f;
    entry.t_arrival_ms = t_arrival_local;
    entry.mode = 0;  // LINEAR interpolation
    
    // Check current state for this DOF
    WaypointState current_state = waypoint_buffer_state(dof);
    bool is_first_waypoint = (current_state == WaypointState::IDLE);
    bool needs_init = is_first_waypoint || (current_state == WaypointState::HOLDING);
    
    // Initialize movement if needed
    if (needs_init && active_joint_controller != nullptr) {
      bool is_valid = false;
      float current_angle = active_joint_controller->getCurrentAngle(dof, is_valid);
      
      if (is_valid) {
        // Safety check
        String safety_violation;
        if (!active_joint_controller->checkWaypointSafety(dof, current_angle, 
                                                          entry.target_angle_deg, entry.t_arrival_ms, 
                                                          t_now, safety_violation)) {
          LOG_ERROR("[CAN SAFETY] Multi-DOF DOF" + String(dof) + ": " + safety_violation);
          emergency_stop_requested = true;
          return;
        }
        
        waypoint_buffer_set_prev(dof, current_angle, t_now);
        waypoint_buffer_set_state(dof, WaypointState::MOVING);
        
        if (is_first_waypoint) {
          LOG_DEBUG("[CAN] DOF " + String(dof) + " IDLE → MOVING (multi-DOF)");
        }
      }
    }
    
    // Push to buffer
    if (waypoint_buffer_push(dof, entry)) {
      queued_count++;
    } else {
      LOG_WARN("[CAN] Multi-DOF buffer full for DOF " + String(dof));
    }
  }
  
  // Log summary (throttled to avoid serial bottleneck)
  static uint16_t multi_dof_log_counter = 0;
  multi_dof_log_counter++;
  if (multi_dof_log_counter >= 50) {
    LOG_INFO("[CAN] Multi-DOF: " + String(queued_count) + " DOFs queued, t_offset=" + 
             String(multi_wp.t_offset_ms) + "ms, t_arrival=" + String(t_arrival_local));
    multi_dof_log_counter = 0;
  }
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
  
  // Debug: log that we received something on Host CAN
  static uint32_t last_rx_log = 0;
  if (millis() - last_rx_log > 1000) {  // Log max once per second
    LOG_INFO("[CAN_HOST] Message available on J5");
    last_rx_log = millis();
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
    } else if (rx_id >= CAN_ID_MULTI_DOF_WAYPOINT_BASE && rx_id < CAN_ID_STATUS_BASE) {
      // Multi-DOF Waypoint (0x380-0x39F) - optimized format, all DOFs in one frame
      handleMultiDofWaypointFrame(rx_id, buf, len);
    } else if (rx_id >= CAN_ID_WAYPOINT_BASE && rx_id < CAN_ID_MULTI_DOF_WAYPOINT_BASE) {
      // Single-DOF Waypoint (0x300-0x37F) - legacy format, one DOF per frame
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

    case CMD_CAN_DIAG: {
      // CAN Bus Diagnostic Test - Motor CAN only (Host CAN disabled)
      LOG_INFO("=== CAN BUS DIAGNOSTIC TEST ===");
      
      extern MCP_CAN CAN;  // Motor CAN (J4)
      
      bool all_ok = true;
      int motors_responding = 0;
      int motors_failed = 0;
      int send_errors = 0;
      bool loopback_ok = false;
      
      // Test 1: MCP2515 SPI communication
      LOG_INFO("[DIAG] Step 1: MCP2515 SPI Check");
      LOG_INFO("[DIAG] Motor CAN (J4): CS=GP9, INT=GP13");
      LOG_INFO("[DIAG] Host CAN (J5): DISABLED for debugging");
      
      // Check CS pin state
      LOG_INFO("[DIAG] CS pin GP9 state: " + String(digitalRead(9)));
      
      // Test 1b: MCP2515 Loopback Test (internal, no bus needed)
      LOG_INFO("[DIAG] Step 1b: MCP2515 LOOPBACK Test");
      {
        // Switch to loopback mode
        if (CAN.setMode(MCP_LOOPBACK) == CAN_OK) {
          LOG_INFO("[DIAG]   Loopback mode enabled");
          
          // Send a test message
          unsigned char testData[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};
          unsigned long testId = 0x7FF;  // Use max standard ID
          
          byte sendResult = CAN.sendMsgBuf(testId, 0, 8, testData);
          if (sendResult == CAN_OK) {
            LOG_INFO("[DIAG]   TX in loopback: OK");
            
            // Wait for message to loop back
            delay(10);
            
            // Try to receive the looped message
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
              unsigned long rxId;
              unsigned char rxLen;
              unsigned char rxBuf[8];
              
              if (CAN.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
                // Verify received data matches sent data
                bool dataMatch = (rxId == testId) && (rxLen == 8);
                for (int i = 0; i < 8 && dataMatch; i++) {
                  if (rxBuf[i] != testData[i]) dataMatch = false;
                }
                
                if (dataMatch) {
                  LOG_INFO("[DIAG]   ✓ LOOPBACK OK - SPI + MCP2515 working!");
                  LOG_INFO("[DIAG]     Received ID=0x" + String(rxId, HEX) + 
                           " Data=" + String(rxBuf[0], HEX) + "," + String(rxBuf[1], HEX) + 
                           "," + String(rxBuf[2], HEX) + "," + String(rxBuf[3], HEX));
                  loopback_ok = true;
                } else {
                  LOG_ERROR("[DIAG]   ✗ LOOPBACK DATA MISMATCH");
                  LOG_ERROR("[DIAG]     Expected ID=0x7FF, got ID=0x" + String(rxId, HEX));
                }
              } else {
                LOG_ERROR("[DIAG]   ✗ LOOPBACK RX read failed");
              }
            } else {
              LOG_ERROR("[DIAG]   ✗ LOOPBACK RX: No message received");
              LOG_ERROR("[DIAG]     MCP2515 internal issue or SPI problem");
            }
          } else {
            LOG_ERROR("[DIAG]   ✗ LOOPBACK TX failed (code " + String(sendResult) + ")");
          }
          
          // Return to normal mode for bus tests
          if (CAN.setMode(MCP_NORMAL) == CAN_OK) {
            LOG_INFO("[DIAG]   Returned to normal mode");
          } else {
            LOG_ERROR("[DIAG]   ✗ Failed to return to normal mode!");
            all_ok = false;
          }
        } else {
          LOG_ERROR("[DIAG]   ✗ Failed to enter loopback mode - SPI issue?");
          all_ok = false;
        }
        delay(20);
      }
      
      // Test 2: Raw CAN TX test (requires bus connection)
      LOG_INFO("[DIAG] Step 2: CAN Bus TX Test (Motor ID 1)");
      {
        unsigned char testCmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};  // READ_STATUS
        unsigned long testId = 0x141;  // Motor ID 1
        
        LOG_INFO("[DIAG] Sending to CAN ID 0x" + String(testId, HEX) + "...");
        
        byte sendResult = CAN.sendMsgBuf(testId, 0, 8, testCmd);
        if (sendResult == CAN_OK) {
          LOG_INFO("[DIAG]   ✓ CAN TX OK - ACK received from bus");
        } else if (sendResult == CAN_SENDMSGTIMEOUT) {
          LOG_ERROR("[DIAG]   ✗ CAN TX TIMEOUT (code " + String(sendResult) + ")");
          LOG_ERROR("[DIAG]     NO ACK = no device responding on CAN bus");
          send_errors++;
          all_ok = false;
        } else if (sendResult == CAN_GETTXBFTIMEOUT) {
          LOG_ERROR("[DIAG]   ✗ CAN TX BUFFER TIMEOUT (code " + String(sendResult) + ")");
          LOG_ERROR("[DIAG]     MCP2515 TX buffer busy - SPI issue?");
          send_errors++;
          all_ok = false;
        } else {
          LOG_ERROR("[DIAG]   ✗ CAN TX ERROR code: " + String(sendResult));
          send_errors++;
          all_ok = false;
        }
        delay(20);
      }
      
      // Test 3: Try second motor
      LOG_INFO("[DIAG] Step 3: CAN Bus TX Test (Motor ID 2)");
      {
        unsigned char testCmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
        unsigned long testId = 0x142;  // Motor ID 2
        
        LOG_INFO("[DIAG] Sending to CAN ID 0x" + String(testId, HEX) + "...");
        
        byte sendResult = CAN.sendMsgBuf(testId, 0, 8, testCmd);
        if (sendResult == CAN_OK) {
          LOG_INFO("[DIAG]   ✓ CAN TX OK");
        } else {
          LOG_ERROR("[DIAG]   ✗ CAN TX ERROR code: " + String(sendResult));
          send_errors++;
        }
        delay(20);
      }
      
      // Test 4: Motor angle read via LKM_Motor
      LOG_INFO("[DIAG] Step 4: Motor Communication Test");
      
      if (controller != nullptr) {
        const JointConfig& cfg = controller->getConfig();
        LOG_INFO("[DIAG] Testing " + String(cfg.motor_count) + " motors for: " + String(cfg.name));
        
        for (uint8_t m = 0; m < cfg.motor_count; m++) {
          uint8_t motor_id = cfg.motors[m].id;
          String motor_name = String(cfg.motors[m].name);
          
          LOG_INFO("[DIAG] Motor " + String(m) + " (ID=" + String(motor_id) + ", " + motor_name + ")");
          
          LKM_Motor* motor = controller->getMotor(m);
          if (motor != nullptr) {
            LKM_Motor::MultiAngleData data = motor->getMultiAngleSync(false);
            
            if (data.waitTime > 0) {
              LOG_INFO("[DIAG]   ✓ Response in " + String(data.waitTime) + "µs: " + 
                       String(data.angle, 2) + "°");
              motors_responding++;
            } else {
              LOG_ERROR("[DIAG]   ✗ NO RESPONSE (waitTime=0)");
              motors_failed++;
              all_ok = false;
            }
          } else {
            LOG_ERROR("[DIAG]   ✗ Motor object NULL");
            motors_failed++;
            all_ok = false;
          }
          delay(50);
        }
      }
      
      // Summary
      LOG_INFO("[DIAG] === SUMMARY ===");
      LOG_INFO("[DIAG] Loopback test: " + String(loopback_ok ? "PASS" : "FAIL"));
      LOG_INFO("[DIAG] TX errors: " + String(send_errors));
      LOG_INFO("[DIAG] Motors: " + String(motors_responding) + "/" + 
               String(motors_responding + motors_failed) + " responding");
      
      if (all_ok && motors_responding > 0) {
        LOG_INFO("[DIAG] ✓ CAN TESTS PASSED");
        snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                 "CAN OK: %d motors", motors_responding);
        shared_data_ext.flag = CMD1_END_MOVE;
      } else {
        LOG_ERROR("[DIAG] ✗ CAN TESTS FAILED");
        if (!loopback_ok) {
          LOG_ERROR("[DIAG] → LOOPBACK FAILED = SPI or MCP2515 issue");
          LOG_ERROR("[DIAG]   Check SPI wiring: SCK(GP10), MOSI(GP11), MISO(GP12), CS(GP9)");
        } else if (send_errors > 0) {
          LOG_ERROR("[DIAG] → LOOPBACK OK but TX FAILED = Transceiver or bus issue");
          LOG_ERROR("[DIAG]   1. CAN transceiver powered? (5V or 3.3V)");
          LOG_ERROR("[DIAG]   2. TXCAN/RXCAN pins connected?");
          LOG_ERROR("[DIAG]   3. CAN_H/CAN_L to bus?");
          LOG_ERROR("[DIAG]   4. Termination 120Ω present?");
          LOG_ERROR("[DIAG]   5. Other device on bus powered?");
        } else {
          LOG_ERROR("[DIAG] → TX OK but motors not responding");
          LOG_ERROR("[DIAG]   1. Motor IDs correct?");
          LOG_ERROR("[DIAG]   2. Motors in error state?");
        }
        snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                 "CAN FAIL: LB=%s TX_ERR=%d M=%d/%d", 
                 loopback_ok ? "OK" : "FAIL", send_errors, 
                 motors_responding, motors_responding + motors_failed);
        shared_data_ext.flag = CMD1_FAIL_MOVE;
      }
      
      LOG_INFO("=== END CAN DIAGNOSTIC ===");
    } break;

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

