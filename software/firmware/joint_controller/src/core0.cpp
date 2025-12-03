/**
 * @file core0.cpp
 * @brief Core0 execution loop - Serial communication and command dispatch
 *
 * This file contains the Core0 main loop responsible for:
 * - Processing incoming serial commands from host
 * - Parsing commands with CommandParser
 * - Dispatching hardware commands to Core1 via inter-core buffers
 * - Handling commands that don't require hardware access (queries, configuration)
 * - Streaming data to host (encoder values, mapping data, movement samples)
 * - Managing auto-mapping data protocol
 *
 * Core0 runs in parallel with Core1 (see core1.cpp), which handles hardware operations.
 *
 * IMPORTANT: Core0 must NOT access motor hardware directly to prevent conflicts.
 *
 * @see main_common.h for shared data structures and global variables
 * @see core1.cpp for hardware operations and movement execution
 * @see CommandParser.h for command parsing logic
 */

#include "main_common.h"

// ============================================================================
// HELPER FUNCTIONS - Serial & Mapping Data
// ============================================================================

/**
 * @brief Convert joint ID to serial name string
 * @param joint_id Numeric joint ID (from JointConfig.h)
 * @return Pointer to serial name string (e.g. "KNEE_LEFT")
 */
const char *jointIdToSerialName(uint8_t joint_id) {
  switch (joint_id) {
  case JOINT_KNEE_LEFT:
    return SERIAL_JOINT_KNEE_LEFT;
  case JOINT_KNEE_RIGHT:
    return SERIAL_JOINT_KNEE_RIGHT;
  case JOINT_ANKLE_LEFT:
    return SERIAL_JOINT_ANKLE_LEFT;
  case JOINT_ANKLE_RIGHT:
    return SERIAL_JOINT_ANKLE_RIGHT;
  case JOINT_HIP_LEFT:
    return SERIAL_JOINT_HIP_LEFT;
  case JOINT_HIP_RIGHT:
    return SERIAL_JOINT_HIP_RIGHT;
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Clear all movement samples from queue
 * 
 * Removes all pending movement samples from the inter-core queue.
 * Used to reset the queue before starting a new movement logging session.
 */
void clearMovementSampleQueue() {
  MovementSample tmp;
  while (queue_try_remove(&movement_sample_queue, &tmp)) {
  }
}

/**
 * @brief Flush movement samples to serial output
 * 
 * Sends all collected movement samples to the host via serial.
 * Samples are organized per DOF and sent in CSV format for analysis.
 * 
 * Format: EVT:MOVEMENT_SAMPLE_HEADER(joint_id,dof_count)
 * Followed by: EVT:DOF{dof}_SAMPLE(time_ms,target_angle,actual_angle,error,torque)
 * 
 * Only flushes if movement_sample_stream_done is true.
 */
void flushMovementSamples() {
  if (!movement_sample_stream_done) {
    return;
  }

  // Count samples per DOF first (without removing from queue)
  int dof_counts[MAX_DOFS] = {0};
  int total_samples = movement_sample_queue.element_count;
  
  Serial.println("DBG: flushMovementSamples() - queue has " + String(total_samples) + " samples");
  
  // Simple approach: just send everything in queue order
  // The Python parser will organize by DOF
  if (total_samples > 0) {
    Serial.println("EVT:MOVEMENT_SAMPLE_HEADER(" + String(movement_sample_joint_id) + "," + String(2) + ")");
    Serial.flush();  // Force immediate send
    delay(50);  // Wait for buffer to drain
    
    // Now drain queue and send samples
    MovementSample sample;
    int sent_count = 0;
    
    while (queue_try_remove(&movement_sample_queue, &sample)) {
      
      // Send sample immediately
      Serial.print("EVT:DOF" + String(sample.dof) + "_SAMPLE(");
      Serial.print(String(sample.index) + ",");
      Serial.print(String(sample.joint_target, 4) + ",");
      Serial.print(String(sample.joint_actual, 4) + ",");
      Serial.print(String(sample.motor_agonist_curr, 4) + ",");
      Serial.print(String(sample.motor_antagonist_curr, 4) + ",");
      Serial.print(String(sample.motor_agonist_ref, 4) + ",");
      Serial.print(String(sample.motor_antagonist_ref, 4) + ",");
      Serial.print(String(sample.torque_agonist, 4) + ",");
      Serial.print(String(sample.torque_antagonist, 4));
      Serial.println(")");
      
      sent_count++;
      
      // Every 5 samples, flush and delay
      if (sent_count % 5 == 0) {
        Serial.flush();
        delay(20);
      }
    }
    
    Serial.flush();
    delay(50);
    Serial.println("EVT:MOVEMENT_SAMPLES_END");
    Serial.flush();
    
    Serial.println("DBG: Sent " + String(sent_count) + " samples");
  }

  if (movement_sample_overflow) {
    LOG_WARN("Movement sample queue overflowed - some samples lost");
  }

  movement_sample_stream_active = false;
  movement_sample_stream_done   = false;
  movement_sample_joint_id      = 0;
  movement_sample_overflow      = false;
}

// ============================================================================
// NOTE: CAN polling has been moved to Core1 (see core1.cpp)
// Core1 now handles ALL CAN communication (Host + Motor) to avoid SPI1 conflicts
// ============================================================================
// CORE0 MAIN LOOP - Serial Communication & Command Dispatch
// ============================================================================

/**
 * @brief Core0 main execution loop
 * 
 * This function runs continuously on Core0 and handles all serial communication
 * with the host. It processes incoming commands, dispatches hardware operations
 * to Core1, and streams data back to the host.
 * 
 * Main responsibilities:
 * 1. Initialize Core1 (one-time setup)
 * 2. Receive and parse serial commands
 * 3. Handle non-hardware commands directly (queries, status, etc.)
 * 4. Dispatch hardware commands to Core1 via double-buffered inter-core communication
 * 5. Monitor Core1 command completion and report results
 * 6. Stream data to host (encoders, mapping data, movement samples)
 * 7. Handle auto-mapping state machine and data protocol
 * 
 * Command protocol:
 * - All commands must start with "CMD:" prefix
 * - Commands are parsed by CommandParser
 * - Hardware commands are sent to Core1 via command_buffer[]
 * - Core1 signals completion via shared_data_ext.flag
 * 
 * @see core1.cpp for hardware command execution
 * @see CommandParser.h for command parsing logic
 * @see shared_data.h for inter-core communication structures
 */

// ============================================================================
// SHARED DOF ANGLES UPDATE
// ============================================================================

/**
 * @brief Update shared DOF angles from encoders (called every Core0 cycle)
 * 
 * This function provides a single point of encoder reading for the entire system.
 * All components (Core1 waypoint control, UI encoder display, etc.) should read
 * from shared_dof_angles instead of calling encoder functions directly.
 * 
 * Benefits:
 * - Single SPI read per cycle (efficiency)
 * - Consistent values across all consumers
 * - Velocity calculation with proper dt
 * - Deterministic timing
 * 
 * NOTE: Encoder reading is throttled to ~500Hz (every 2ms) to reduce SPI bus stress
 * and avoid "Synchronization sequence not found" errors. The control loops run at
 * 100Hz (outer) / 500Hz (inner), so 500Hz encoder updates are sufficient.
 * 
 * DIRECT ENCODER READING: Uses DirectEncoders class to read MT6835 sensors
 * directly via SPI0, without intermediate encoder Pico.
 */
void updateSharedDofAngles() {
  static uint32_t last_update_us = 0;
  static uint32_t last_encoder_read_us = 0;
  
  // Throttle encoder reads to ~500Hz (every 2000us = 2ms)
  // This reduces SPI bus stress and prevents sync errors
  static const uint32_t ENCODER_READ_INTERVAL_US = 2000;
  
  JointController *controller = active_joint_controller;
  if (controller == nullptr) {
    return;
  }
  
  uint32_t now_us = time_us_32();
  
  // Skip encoder read if not enough time has passed
  if (last_encoder_read_us > 0 && (now_us - last_encoder_read_us) < ENCODER_READ_INTERVAL_US) {
    return;  // Keep previous values, they're still fresh enough
  }
  
  // Consecutive error counters for emergency stop (one per DOF)
  static uint8_t consecutive_errors[MAX_DOFS] = {0};
  static const uint8_t ENCODER_ERROR_THRESHOLD = 25;  // 25 cycles @ 500Hz = 50ms
  
  // Update encoder data from hardware (direct SPI to MT6835)
  directEncoders.update();
  last_encoder_read_us = now_us;
  
  // Calculate time delta for velocity
  float dt_s = (last_update_us > 0) ? (now_us - last_update_us) / 1000000.0f : 0.0f;
  
  uint8_t dof_count = controller->getConfig().dof_count;
  shared_dof_angles.dof_count = dof_count;
  
  for (uint8_t dof = 0; dof < dof_count; dof++) {
    // Check if this encoder had a valid read
    bool is_valid = directEncoders.isEncoderConnected(dof) && 
                    (directEncoders.getErrorCount(dof) == 0);
    
    if (is_valid) {
      // Get angle directly from DirectEncoders (already in degrees with multi-turn)
      float new_angle = directEncoders.getAngle(dof);
      
      // Reset error counter on successful read
      consecutive_errors[dof] = 0;
      
      // Calculate velocity if we have a previous reading
      if (dt_s > 0.0001f && shared_dof_angles.valid[dof]) {
        float angle_diff = new_angle - shared_dof_angles.angles[dof];
        shared_dof_angles.velocities[dof] = angle_diff / dt_s;
      } else {
        shared_dof_angles.velocities[dof] = 0.0f;
      }
      
      shared_dof_angles.angles[dof] = new_angle;
      shared_dof_angles.valid[dof] = true;
    } else {
      // Increment error counter for this specific DOF
      consecutive_errors[dof]++;
      shared_dof_angles.valid[dof] = false;
      shared_dof_angles.velocities[dof] = 0.0f;
      
      // Check for emergency stop condition per DOF
      if (consecutive_errors[dof] >= ENCODER_ERROR_THRESHOLD && !emergency_stop_requested) {
        emergency_stop_requested = true;
        LOG_ERROR("[SAFETY] EMERGENCY STOP: Encoder DOF " + String(dof) + 
                  " failed " + String(ENCODER_ERROR_THRESHOLD) + " consecutive reads");
      }
    }
  }
  
  // Mark remaining DOFs as invalid
  for (uint8_t dof = dof_count; dof < MAX_DOFS; dof++) {
    shared_dof_angles.valid[dof] = false;
    consecutive_errors[dof] = 0;  // Reset unused DOFs
  }
  
  shared_dof_angles.timestamp_us = now_us;
  shared_dof_angles.updated = true;
  last_update_us = now_us;
}

// ============================================================================
// CORE0 MAIN LOOP
// ============================================================================

void core0_main_loop() {

#pragma region Init Core1 and SharedData
  // Start the second core if it is not already running
  if (init_prg) {
    multicore_launch_core1(core1_loop);
    init_prg = false;
  }
#pragma endregion

  // Update shared DOF angles from encoders (single read point for entire system)
  updateSharedDofAngles();

  // NOTE: CAN polling has been moved to Core1 to avoid SPI1 conflicts
  // Core1 now handles all CAN communication (Host + Motor)

#pragma region Receive SerialData
  // check for incoming serial data:
  if (Serial.available() > 0) {

    // read the incoming byte:
    char c = Serial.read();
    // add to string array until enter is pressed
    if (c != '\n' && c != '\r') {
      // Add character to command buffer
      size_t len = strlen(command);
      if (len < sizeof(command) - 1) {
        command[len]     = c;
        command[len + 1] = '\0';
      }
    } else if (strlen(command) > 0) {
      // print the value of the incoming byte:
      Serial.println(command);

      // Check if command has correct prefix
      if (strncmp(command, "CMD:", 4) == 0) {
        // Valid command: remove the prefix and determine the type
        char actual_command[96];
        strcpy(actual_command, command + 4); // Skip "CMD:"

        // First, try to recognize it as a simple command (e.g., STATUS, STOP)
        {
          int simple_cmd_id = getCommandId(actual_command);
          
          if (simple_cmd_id != CMD_UNKNOWN && 
              (simple_cmd_id == CMD_STATUS || simple_cmd_id == CMD_STOP)) {
            // Handle simple commands that don't use multi-joint format
            switch (simple_cmd_id) {
              case CMD_STATUS: {
                // Get joint status
                if (active_joint_controller != nullptr) {
                  bool ready = active_joint_controller->isSystemReadyForMovement();
                  Serial.println("RSP:STATUS(" + String(ACTIVE_JOINT) + "," +
                                 (ready ? "READY" : "NOT_READY") + ")");
                } else {
                  Serial.println("RSP:ERROR: Controller not initialized");
                }
                break;
              }
              
              case CMD_STOP: {
                // Emergency stop - forward to Core1
                emergency_stop_requested = true;
                Serial.println("RSP:STOP");
                break;
              }
            }
          }
          // Try to parse as multi-joint format command
          else {
            // Make a copy for parsing (parseCommand uses strtok which modifies the string)
            char cmd_copy[96];
            strncpy(cmd_copy, actual_command, sizeof(cmd_copy) - 1);
            cmd_copy[sizeof(cmd_copy) - 1] = '\0';
            
            ParsedCommand parsed_cmd;
            bool is_new_format = command_parser.parseCommand(cmd_copy, parsed_cmd);

            if (!is_new_format) {
              // Unrecognized command format
              LOG_ERROR("Unrecognized command format: " + String(actual_command));
            } else {
              // Command is in the new (multi-joint) format

              // If the command targets a specific joint, verify it is the active joint
              if (parsed_cmd.joint_id != 0 && parsed_cmd.joint_id != ACTIVE_JOINT) {
                Serial.println("RSP:ERROR: Command targeted to joint " + String(parsed_cmd.joint_id) +
                               ", but this device controls joint " + String(ACTIVE_JOINT));
              } else {
                // Set the joint ID to the active one (if it was 0 or already correct)
                parsed_cmd.joint_id = ACTIVE_JOINT;

                if (parsed_cmd.command == CMD_UNKNOWN) {
                  Serial.println("RSP:ERROR: Unsupported command: " +
                                 String(parsed_cmd.original_command));
                } else {
                  // Debug command execution
                  LOG_DEBUG("CMD: " + String(parsed_cmd.original_command) + " → ID:" + String(parsed_cmd.command));
                  LOG_DEBUG("    Joint:" + String(parsed_cmd.joint_id) + " DOF:" + String(parsed_cmd.dof_index) + 
                            " AllDOF:" + String(parsed_cmd.all_dofs) + " Params:" + String(parsed_cmd.param_count));

                  // Populate command data
                  command_parser.populateCommandData(parsed_cmd, command_data_ext);

                  // Handle commands that don't require hardware access
                  bool handled_on_core0 = false;

                  // Check if the command can be handled directly on core0
                  switch (parsed_cmd.command) {
                    case CMD_STOP_MEASURING: {
                      measuring_data_ext.flag = 0;
                      handled_on_core0        = true;
                      LOG_INFO("Measurement stopped");
                      break;
                    }

                    case CMD_START_MEASURING: {
                      measuring_data_ext.flag      = 1;
                      measuring_data_ext.joint_id  = ACTIVE_JOINT;
                      measuring_data_ext.dof_index = parsed_cmd.dof_index;
                      handled_on_core0             = true;
                      LOG_INFO("Measurement started for DOF " + String(parsed_cmd.dof_index));
                      break;
                    }

                    // Note: CMD_GET_VERSION and CMD_GET_ANGLES commands do not exist in commands.h
                    // Version information is sent automatically during setup() as EVT:FW:VERSION

                    case CMD_GET_PID: {
                      // Get PID parameters for specified DOF and motor
                      if (active_joint_controller != nullptr && parsed_cmd.dof_index < MAX_DOFS) {
                        uint8_t motor_index = 0;

                        if (parsed_cmd.param_count > 0) {
                          motor_index = static_cast<uint8_t>(parsed_cmd.params[0]);
                        } else {
                          // Fallback: parse motor_index directly from command string
                          const char *cmd_segment = strstr(parsed_cmd.original_command,
                                                            SERIAL_CMD_GET_PID ":");
                          if (cmd_segment != nullptr) {
                            cmd_segment += strlen(SERIAL_CMD_GET_PID) + 1;
                            motor_index = static_cast<uint8_t>(atoi(cmd_segment));
                          } else {
                            LOG_WARN("GET_PID: failed to parse motor index");
                          }
                        }

                        if (motor_index == 1 || motor_index == 2) {
                          float kp, ki, kd, tau;
                          if (active_joint_controller->getPid(parsed_cmd.dof_index, motor_index, kp, ki, kd, tau)) {
                            Serial.println("EVT:PID:" + String(parsed_cmd.dof_index) + ":" + String(motor_index) + ":" +
                                           String(kp, 6) + ":" + String(ki, 6) + ":" +
                                           String(kd, 6) + ":" + String(tau, 6));
                          } else {
                            Serial.println("RSP:ERROR: Failed to get PID parameters");
                          }
                        } else {
                          Serial.println("RSP:ERROR: Invalid or missing motor index (expected 1 or 2)");
                        }
                      } else {
                        Serial.println("RSP:ERROR: Controller not initialized or invalid DOF");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_SET_PID: {
                      // Set PID parameters for specified DOF and motor
                      if (active_joint_controller != nullptr && parsed_cmd.dof_index < MAX_DOFS) {
                        uint8_t motor_index = parsed_cmd.params[0]; // 1=agonist, 2=antagonist
                        if ((motor_index == 1 || motor_index == 2) && parsed_cmd.param_count >= 5) {
                          float kp  = parsed_cmd.params[1];
                          float ki  = parsed_cmd.params[2];
                          float kd  = parsed_cmd.params[3];
                          float tau = parsed_cmd.params[4];
                          if (active_joint_controller->setPid(parsed_cmd.dof_index, motor_index, kp, ki, kd, tau)) {
                            Serial.println("RSP:PID_SET_OK(" + String(ACTIVE_JOINT) + "," +
                                           String(parsed_cmd.dof_index) + "," + String(motor_index) + ")");
                          } else {
                            Serial.println("RSP:ERROR: Failed to set PID parameters");
                          }
                        } else {
                          Serial.println("RSP:ERROR: Invalid parameters (expected motor_index,kp,ki,kd,tau)");
                        }
                      } else {
                        Serial.println("RSP:ERROR: Controller not initialized or invalid DOF");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_GET_PID_OUTER: {
                      // Get outer loop (cascade) PID parameters for specified DOF
                      if (active_joint_controller != nullptr && parsed_cmd.dof_index < MAX_DOFS) {
                        float kp, ki, kd, stiffness_deg, cascade_influence;
                        if (active_joint_controller->getOuterLoopParameters(parsed_cmd.dof_index, kp, ki, kd,
                                                                             stiffness_deg, cascade_influence)) {
                          // Format: EVT:PID_OUTER:<DOF>:<KP>:<KI>:<KD>:<STIFFNESS>:<CASCADE>
                          Serial.println("EVT:PID_OUTER:" + String(parsed_cmd.dof_index) + ":" +
                                         String(kp, 6) + ":" + String(ki, 6) + ":" + String(kd, 6) + ":" +
                                         String(stiffness_deg, 6) + ":" + String(cascade_influence, 6));
                        } else {
                          Serial.println("RSP:ERROR: Failed to get outer loop PID parameters");
                        }
                      } else {
                        Serial.println("RSP:ERROR: Controller not initialized or invalid DOF");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_SET_PID_OUTER: {
                      // Set outer loop (cascade) PID parameters for specified DOF
                      if (active_joint_controller != nullptr && parsed_cmd.dof_index < MAX_DOFS) {
                        if (parsed_cmd.param_count >= 5) {
                          float kp                = parsed_cmd.params[0];
                          float ki                = parsed_cmd.params[1];
                          float kd                = parsed_cmd.params[2];
                          float stiffness_deg     = parsed_cmd.params[3];
                          float cascade_influence = parsed_cmd.params[4];
                          if (active_joint_controller->setOuterLoopParameters(parsed_cmd.dof_index, kp, ki, kd,
                                                                               stiffness_deg, cascade_influence)) {
                            Serial.println("RSP:PID_OUTER_SET_OK(" + String(ACTIVE_JOINT) + "," +
                                           String(parsed_cmd.dof_index) + ")");
                          } else {
                            Serial.println("RSP:ERROR: Failed to set outer loop PID parameters");
                          }
                        } else {
                          Serial.println("RSP:ERROR: Invalid parameters (expected kp,ki,kd,stiffness,influence)");
                        }
                      } else {
                        Serial.println("RSP:ERROR: Controller not initialized or invalid DOF");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_SAVE_PID: {
                      // Save PID parameters to flash memory
                      if (active_joint_controller != nullptr) {
                        if (active_joint_controller->savePIDDataToFlash()) {
                          Serial.println("RSP:PID_SAVED(" + String(ACTIVE_JOINT) + ")");
                          LOG_INFO("PID parameters saved to flash for joint " + String(ACTIVE_JOINT));
                        } else {
                          Serial.println("RSP:ERROR: Failed to save PID parameters to flash");
                          LOG_ERROR("Failed to save PID parameters for joint " + String(ACTIVE_JOINT));
                        }
                      } else {
                        Serial.println("RSP:ERROR: Controller not initialized");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_LOAD_PID: {
                      // Load PID parameters from flash memory
                      if (active_joint_controller != nullptr) {
                        if (active_joint_controller->loadPIDDataFromFlash()) {
                          Serial.println("RSP:PID_LOADED(" + String(ACTIVE_JOINT) + ")");
                          LOG_INFO("PID parameters loaded from flash for joint " + String(ACTIVE_JOINT));
                        } else {
                          Serial.println("RSP:ERROR: Failed to load PID parameters from flash");
                          LOG_ERROR("Failed to load PID parameters for joint " + String(ACTIVE_JOINT));
                        }
                      } else {
                        Serial.println("RSP:ERROR: Controller not initialized");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_START_TEST_ENCODER: {
                      // Activate encoder test
                      encoder_test_active    = true;
                      encoder_test_joint_id  = parsed_cmd.joint_id;
                      encoder_test_dof_index = parsed_cmd.dof_index;
                      encoder_test_all_dofs  = parsed_cmd.all_dofs;
                      last_encoder_test_time = millis();
                      handled_on_core0       = true;

                      if (encoder_test_all_dofs) {
                        LOG_INFO("Encoder test enabled for ALL joint DOFs");
                      } else {
                        LOG_INFO("Encoder test enabled for DOF " + String(parsed_cmd.dof_index));
                      }
                      break;
                    }

                    case CMD_STOP_TEST_ENCODER: {
                      // Deactivate encoder test
                      encoder_test_active   = false;
                      encoder_test_all_dofs = false;
                      handled_on_core0      = true;
                      LOG_INFO("Encoder test disabled");
                      break;
                    }

                    case CMD_GET_MOVEMENT_DATA: {
                      // Flush movement samples to serial on-demand
                      LOG_INFO("Sending movement data to host (on-demand request)");
                      // Enable stream done flag to allow flush (required by flushMovementSamples)
                      movement_sample_stream_done = true;
                      flushMovementSamples();
                      handled_on_core0 = true;
                      break;
                    }

                    default:
                      // Command requires hardware access - will be dispatched to core1
                      break;
                  }

                // If command was not handled on core0, send it to core1
                if (!handled_on_core0) {
                  // Find next free buffer
                  int next_buffer = (active_buffer + 1) % 2;

                  // Wait if buffer is still being processed
                  while (buffer_ready[next_buffer]) {
                    sleep_us(100);
                  }

                  // Copy command data to buffer
                  command_buffer[next_buffer] = command_data_ext;
                  pending_command_type        = parsed_cmd.command;

                  // Signal buffer ready (atomic)
                  buffer_ready[next_buffer] = true;
                  active_buffer             = next_buffer;

                  LOG_DEBUG("Command dispatched to core1: " + String(parsed_cmd.command));
                }
              }  // Close the 'else' block for parsed_cmd.command == CMD_UNKNOWN
            }    // Close the 'else' block for parsed_cmd.joint_id compatibility
          }      // Close the 'else' block for is_new_format parsing
          }      // Close the 'else' block for multi-joint format parsing (line 261)
        }        // Close the "else" block for SYNC command check (line 233)
      } else {  // Close the "if (CMD:)" block (line 198)
        // Command without correct prefix
        Serial.println("RSP:ERROR: Commands must start with CMD: prefix");
      }

      // reset command buffer
      command[0] = '\0';
    }
  }
#pragma endregion

#pragma region Check SharedData from Core1
  // Check if core1 has completed a command
  if (shared_data_ext.flag != 0) {
    // Process result based on flag
    switch (shared_data_ext.flag) {
    case CMD1_END_MOVE:
      Serial.println("RSP:MOVE_COMPLETE(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      break;

    case CMD1_FAIL_MOVE:
      Serial.println("RSP:MOVE_FAILED(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      break;

    case CMD1_END_ZERO:
      Serial.println("RSP:ZERO_COMPLETE(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.dof_index) + ")");
      break;

    case CMD1_AUTO_MAP_PROGRESS:
      Serial.println("RSP:AUTO_MAP_PROGRESS(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      break;

    case CMD1_AUTO_MAP_COMPLETE:
      Serial.println("RSP:AUTO_MAP_COMPLETE(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      // Signal that mapping data is ready to be sent
      auto_mapping_data_ready_to_send = true;
      break;

    default:
      Serial.println("RSP:UNKNOWN_FLAG(" + String(shared_data_ext.flag) + ")");
      break;
    }

    // Reset flag (command processed)
    shared_data_ext.flag = 0;
  }
#pragma endregion

#pragma region Streaming Data

  // Stream encoder data if measuring is active (uses shared_dof_angles)
  if (measuring_data_ext.flag == 1) {
    uint8_t dof = measuring_data_ext.dof_index;
    if (dof < shared_dof_angles.dof_count && shared_dof_angles.valid[dof]) {
      Serial.println("EVT:ANGLE(" + String(ACTIVE_JOINT) + "," + String(dof) + "," +
                     String(shared_dof_angles.angles[dof], 4) + ")");
    }
    delay(50); // Throttle streaming to ~20Hz
  }

  // Send auto-mapping data if ready
  if (auto_mapping_data_ready_to_send) {
    LOG_DEBUG("Auto-mapping data ready - preparing to send to Pi5...");
    JointController *controller = active_joint_controller;

    if (controller == nullptr) {
      LOG_ERROR("Controller not available");
    } else {
      // NEW CHECK: Verify if there is data to send
      bool has_mapping_data = false;
      int total_arrays      = 0;

      for (int dof = 0; dof < controller->getConfig().dof_count; dof++) {
        DofMappingData_t *mapping_data = controller->getMappingData(dof);
        LOG_DEBUG("DOF " + String(dof) +
                  " - mapping_data pointer: " + String((unsigned long)mapping_data, HEX));

        if (mapping_data != nullptr) {
          LOG_DEBUG("DOF " + String(dof) + " - flag: " + String(mapping_data->flag) +
                    ", size: " + String(mapping_data->size));

          if (mapping_data->flag == 1) {
            has_mapping_data = true;
            total_arrays += 3; // Each DOF contributes 3 arrays (agonist, antagonist, joint)
            LOG_DEBUG("DOF " + String(dof) + " - DATA READY to send");
          } else {
            LOG_DEBUG("DOF " + String(dof) + " - data NOT ready (flag=" + String(mapping_data->flag) + ")");
          }
        } else {
          LOG_DEBUG("DOF " + String(dof) + " - mapping_data is NULL");
        }
      }

      LOG_DEBUG("has_mapping_data = " + String(has_mapping_data) +
                ", total_arrays = " + String(total_arrays));

      if (has_mapping_data) {
        LOG_DEBUG("Starting mapping data send to Pi5...");

        // Find first DOF with ready data to determine size
        int reference_size = 0;
        for (int dof = 0; dof < controller->getConfig().dof_count; dof++) {
          DofMappingData_t *mapping_data = controller->getMappingData(dof);
          if (mapping_data != nullptr && mapping_data->flag == 1) {
            reference_size = mapping_data->size;
            break;
          }
        }

        // Print general information
        LOG_DEBUG("Mapping Data Size: " + String(reference_size));
        LOG_DEBUG("DOF Count: " + String(controller->getConfig().dof_count));

        // Send command with size and DOF count (according to MAPPING_DATA protocol)
        Serial.println("EVT:MAPPING_DATA(" + String(reference_size) + "," +
                       String(controller->getConfig().dof_count) + ")");

        // Send data for each DOF with extended format
        // Format: {prefix}{dof_index}_{array_index}_{data}
        // Prefixes: a=joint, b=agonist, c=antagonist (according to MAPPING_DATA_PROTOCOL.md)
        for (int i = 0; i < reference_size; i++) {
          String line_data = "";

          for (int dof = 0; dof < controller->getConfig().dof_count; dof++) {
            DofMappingData_t *mapping_data = controller->getMappingData(dof);
            if (mapping_data != nullptr && mapping_data->flag == 1) {
              // a = Joint angle for specified DOF
              line_data += "a" + String(dof) + "_" + String(i) + "_" +
                           String(mapping_data->joint_data[i]) + "|";
              // b = Agonist motor angle for specified DOF
              line_data += "b" + String(dof) + "_" + String(i) + "_" +
                           String(mapping_data->agonist_data[i]) + "|";
              // c = Antagonist motor angle for specified DOF
              line_data += "c" + String(dof) + "_" + String(i) + "_" +
                           String(mapping_data->antagonist_data[i]);

              // Add separator if this is not the last DOF
              if (dof < controller->getConfig().dof_count - 1) {
                line_data += "|";
              }
            }
          }

          Serial.println("EVT:" + line_data);
        }

        // Mapping data is no longer saved to flash
        // Only PID parameters are persisted via JointController
        LOG_DEBUG("MAPPING_DATA_SENT_RAW: Raw mapping data sent to Pi5 for processing");
        LOG_DEBUG("NOTE: Mapping data NOT saved to flash (too large)");
        LOG_DEBUG("NOTE: Only PID parameters are persisted for future sessions");

        // Set flag to wait for processed data from Pi5
        auto_mapping_data_ready_to_send = false; // Data sent, now awaiting response

        LOG_DEBUG("NOTE: Raw data sent - movement always controlled by isSystemReadyForMovement()");

        // Reset flags for all DOFs (temporary - will be reset when processed data arrives)
        for (int dof = 0; dof < controller->getConfig().dof_count; dof++) {
          DofMappingData_t *mapping_data = controller->getMappingData(dof);
          if (mapping_data != nullptr && mapping_data->flag == 1) {
            mapping_data->flag = 0;
          }
        }

        LOG_DEBUG("WAITING_PROCESSED_DATA: Waiting for enhanced data from Pi5...");
      }
    }
  }

  // NOTE: Movement samples are NO LONGER flushed automatically.
  // They remain in the queue until explicitly requested via GET_MOVEMENT_DATA command.
  // This avoids race conditions with other serial messages.
  // flushMovementSamples();  // DISABLED - use GET_MOVEMENT_DATA command instead
#pragma endregion

#pragma region TestEncoder
  // Encoder test handling - uses shared_dof_angles (updated by updateSharedDofAngles)
  if (encoder_test_active) {
    static unsigned long last_encoder_print_time = 0;
    // Send encoder data every 200ms
    if (millis() - last_encoder_print_time > 200) {
      last_encoder_print_time = millis();

      // Use shared DOF angles (already updated by updateSharedDofAngles at start of loop)
      JointController *controller = active_joint_controller;

      if (controller != nullptr && shared_dof_angles.dof_count > 0) {
        // Handle ALL case (all DOFs) or single DOF
        if (encoder_test_all_dofs) {
          // Send data for all DOFs of the joint
          for (int dof = 0; dof < shared_dof_angles.dof_count; dof++) {
            // Get encoder channel for this DOF (for raw count)
            uint8_t encoder_channel = controller->getConfig().dofs[dof].encoder_channel;
            int32_t encoder_count = directEncoders.getCount(encoder_channel);

            if (shared_dof_angles.valid[dof]) {
              // Build output message for this DOF using shared angles
              char buffer[100];
              snprintf(buffer, sizeof(buffer), "EVT:ENCODER_DATA:DOF=%d:ANGLE=%.2f:COUNT=%ld", 
                       dof, shared_dof_angles.angles[dof], encoder_count);
              Serial.println(buffer);
            } else {
              char buffer[100];
              snprintf(buffer, sizeof(buffer), "EVT:ENCODER_DATA:DOF=%d:ERROR=Invalid encoder data", dof);
              Serial.println(buffer);
            }
          }
        } else {
          // Single DOF handling
          uint8_t encoder_channel = controller->getConfig().dofs[encoder_test_dof_index].encoder_channel;
          int32_t encoder_count = directEncoders.getCount(encoder_channel);

          if (shared_dof_angles.valid[encoder_test_dof_index]) {
            // Build output message using shared angles
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "EVT:ENCODER_DATA:DOF=%d:ANGLE=%.2f:COUNT=%ld",
                     encoder_test_dof_index, shared_dof_angles.angles[encoder_test_dof_index], encoder_count);
            Serial.println(buffer);
          } else {
            Serial.println("EVT:ENCODER_DATA:ERROR=Invalid encoder data");
          }
        }
      } else {
        Serial.println("EVT:ENCODER_DATA:ERROR=Controller not found");
        encoder_test_active = false; // Disable test on error
      }
    }
  }
#pragma endregion
}

