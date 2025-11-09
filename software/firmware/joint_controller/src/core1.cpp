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

  while (true) {
    // === EMERGENCY STOP CHECK ===
    if (emergency_stop_requested) {
      // Immediate stop of all motors
      if (active_joint_controller != nullptr) {
        active_joint_controller->stopAllMotors();
      }

      // Reset flag
      emergency_stop_requested = false;
      smooth_transition_active = false;

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

    case CMD_PRETENSION:
      // Pretension the motors of the specific DOF
      controller->pretension(dof_index, command_data_ext.torque, command_data_ext.duration);
      break;

    case CMD_PRETENSION_ALL:
      // Pretension all DOFs of the joint with the configured parameters
      controller->pretensionAll();
      break;

    case CMD_RELEASE:
      // Release the motors of the specific DOF (opposite torque compared to pretensioning)
      controller->release(dof_index, command_data_ext.torque, command_data_ext.duration);
      break;

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

