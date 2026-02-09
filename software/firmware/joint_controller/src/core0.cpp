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
  int total_samples = queue_get_level(&movement_sample_queue);
  
  LOG_DEBUG("flushMovementSamples() - queue has " + String(total_samples) + " samples");
  
  // Simple approach: just send everything in queue order
  // The Python parser will organize by DOF
  if (total_samples > 0) {
    uint8_t dof_count = (active_joint_controller != nullptr) ? active_joint_controller->getConfig().dof_count : 0;
    SERIAL_COM_LN("EVT:MOVEMENT_SAMPLE_HEADER(" + String(movement_sample_joint_id) + "," + String(dof_count) + ")");
    Serial.flush();  // Force immediate send
    delay(50);  // Wait for buffer to drain
    
    // Now drain queue and send samples
    MovementSample sample;
    int sent_count = 0;
    
    while (queue_try_remove(&movement_sample_queue, &sample)) {
      
      // Send sample immediately
      SERIAL_COM("EVT:DOF" + String(sample.dof) + "_SAMPLE(");
      SERIAL_COM(String(sample.index) + ",");
      SERIAL_COM(String(sample.joint_target, 4) + ",");
      SERIAL_COM(String(sample.joint_actual, 4) + ",");
      SERIAL_COM(String(sample.motor_agonist_curr, 4) + ",");
      SERIAL_COM(String(sample.motor_antagonist_curr, 4) + ",");
      SERIAL_COM(String(sample.motor_agonist_ref, 4) + ",");
      SERIAL_COM(String(sample.motor_antagonist_ref, 4) + ",");
      SERIAL_COM(String(sample.torque_agonist, 4) + ",");
      SERIAL_COM(String(sample.torque_antagonist, 4));
      SERIAL_COM_LN(")");
      
      sent_count++;
      
      // Every 5 samples, flush and delay
      if (sent_count % 5 == 0) {
        Serial.flush();
        delay(20);
      }
    }
    
    Serial.flush();
    delay(50);
    SERIAL_COM_LN("EVT:MOVEMENT_SAMPLES_END");
    Serial.flush();
    
    LOG_DEBUG("Sent " + String(sent_count) + " samples");
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
// CORE1 LOG QUEUE DRAIN (print Core1 messages safely from Core0)
// ============================================================================

/**
 * @brief Push a log message to the Core1 log queue (non-blocking)
 *
 * Safe to call from any core. If queue is full, message is dropped silently.
 * Called by LOG_C1_* macros defined in debug.h.
 */
void core1LogPush(uint8_t level, const char* msg) {
  Core1LogEntry entry;
  entry.level = level;
  strncpy(entry.msg, msg, CORE1_LOG_MSG_SIZE - 1);
  entry.msg[CORE1_LOG_MSG_SIZE - 1] = '\0';
  queue_try_add(&core1_log_queue, &entry);
}

/**
 * @brief Drain Core1 log queue and print messages to Serial
 *
 * Core1 cannot safely call Serial.print (USB CDC deadlock on RP2350).
 * Instead, Core1 pushes log entries to a lock-free queue, and Core0
 * drains up to 4 entries per cycle to avoid blocking its own loop.
 */
void drainCore1LogQueue() {
  static const char* const prefixes[] = {"ERROR: ", "WARN: ", "INFO: ", "DBG: ", ""};
  Core1LogEntry entry;
  uint8_t count = 0;
  while (count < 4 && queue_try_remove(&core1_log_queue, &entry)) {
    if (SERIAL_CONNECTED()) {
      uint8_t lvl = entry.level;
      if (lvl > C1_LOG_LEVEL_COM) lvl = C1_LOG_LEVEL_COM;
      const char* prefix = prefixes[lvl];
      if (prefix[0] != '\0') Serial.print(prefix);
      Serial.println(entry.msg);
    }
    count++;
  }
}

// ============================================================================
// NOTE: CAN polling has been moved to Core1 (see core1.cpp)
// Core1 now handles ALL CAN communication (Host + Motor) to avoid SPI1 conflicts
// ============================================================================

// ============================================================================
// STARTUP SEQUENCE (shared by Serial CMD and CAN 0x009)
// ============================================================================

/**
 * @brief Push a startup status event to the CAN event queue
 *
 * Core1 drains this queue and sends CAN frames (0x490 + joint_id).
 * Non-blocking: drops event silently if queue is full.
 */
static void pushStartupEvent(uint8_t event_type, uint8_t dof_index,
                              uint8_t reason_code, uint16_t elapsed_ms) {
  StartupStatusEvent evt;
  evt.event_type = event_type;
  evt.dof_index = dof_index;
  evt.reason_code = reason_code;
  evt.elapsed_ms = elapsed_ms;
  queue_try_add(&startup_event_queue, &evt);
}

// Startup event types
#define STARTUP_EVT_BEGIN      0
#define STARTUP_EVT_DOF_READY  1
#define STARTUP_EVT_DOF_FAILED 2
#define STARTUP_EVT_COMPLETE   3
#define STARTUP_EVT_FAILED     4

// Startup reason codes
#define STARTUP_REASON_OK                0
#define STARTUP_REASON_NO_CONTROLLER     1
#define STARTUP_REASON_NO_EQUATIONS      2
#define STARTUP_REASON_ENCODER_TIMEOUT   3
#define STARTUP_REASON_POSITION_RANGE    4
#define STARTUP_REASON_RECALC_ERROR      5
#define STARTUP_REASON_GLOBAL_TIMEOUT    6

/**
 * @brief Execute startup sequence (recalc_offset for all DOFs, then HOLDING)
 *
 * This function is the shared implementation for:
 * - Serial CMD_STARTUP_SEQUENCE
 * - CAN 0x009 startup command
 * - Auto-start at boot
 *
 * Architecture: Core0 performs safety checks (no CAN access), then delegates
 * recalc_offset to Core1 via CMD_RECALC_OFFSET command buffer.
 * All Motor CAN access stays on Core1, preventing SPI1 conflicts.
 *
 * @param custom_torque  Pretension torque override (0 = use config default)
 * @param custom_duration  Pretension duration override (0 = use config default)
 * @return true if startup completed successfully, false on failure
 */
static bool executeStartupSequence(int16_t custom_torque, int16_t custom_duration) {
  const uint32_t STARTUP_TIMEOUT_MS = 30000;
  uint32_t startup_start_time = millis();

  // SAFETY CHECK 0: Controller initialized
  if (active_joint_controller == nullptr) {
    SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=NO_CONTROLLER");
    pushStartupEvent(STARTUP_EVT_FAILED, 0, STARTUP_REASON_NO_CONTROLLER,
                     (uint16_t)(millis() - startup_start_time));
    return false;
  }

  // SAFETY CHECK 1: Linear equations loaded for all DOFs
  bool equations_ok = true;
  for (uint8_t dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
    if (!active_joint_controller->hasValidEquations(dof)) {
      LOG_ERROR("DOF " + String(dof) + ": linear equations not available");
      equations_ok = false;
    }
  }
  if (!equations_ok) {
    SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=NO_EQUATIONS");
    LOG_ERROR("Startup sequence failed: run auto-mapping or load equations from flash first");
    pushStartupEvent(STARTUP_EVT_FAILED, 0, STARTUP_REASON_NO_EQUATIONS,
                     (uint16_t)(millis() - startup_start_time));
    return false;
  }

  // SAFETY CHECK 2: Wait for encoder readings to be valid
  LOG_INFO("Waiting for encoder readings to stabilize...");
  const int MAX_ENCODER_WAIT_MS = 2000;
  const int ENCODER_CHECK_INTERVAL_MS = 100;
  int encoder_wait_ms = 0;
  bool encoders_valid = false;

  while (encoder_wait_ms < MAX_ENCODER_WAIT_MS) {
    if (millis() - startup_start_time > STARTUP_TIMEOUT_MS) {
      SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=GLOBAL_TIMEOUT");
      LOG_ERROR("Startup sequence timed out during encoder wait");
      pushStartupEvent(STARTUP_EVT_FAILED, 0, STARTUP_REASON_GLOBAL_TIMEOUT,
                       (uint16_t)(millis() - startup_start_time));
      return false;
    }

    bool all_valid = true;
    for (uint8_t dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
      if (!shared_dof_angles.valid[dof]) {
        all_valid = false;
        break;
      }
    }
    if (all_valid) {
      encoders_valid = true;
      break;
    }
    delay(ENCODER_CHECK_INTERVAL_MS);
    encoder_wait_ms += ENCODER_CHECK_INTERVAL_MS;
  }

  if (!encoders_valid) {
    SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=ENCODER_TIMEOUT");
    LOG_ERROR("Startup sequence failed: encoder readings not valid after " + String(MAX_ENCODER_WAIT_MS) + "ms");
    pushStartupEvent(STARTUP_EVT_FAILED, 0, STARTUP_REASON_ENCODER_TIMEOUT,
                     (uint16_t)(millis() - startup_start_time));
    return false;
  }
  LOG_INFO("Encoder readings valid after " + String(encoder_wait_ms) + "ms");

  // SAFETY CHECK 3: Verify joint positions are within physical limits
  LOG_INFO("Checking joint position limits...");
  bool positions_ok = true;
  const float POSITION_MARGIN = 5.0f;
  const float POSITION_HARD_MARGIN = 15.0f;

  for (uint8_t dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
    if (!shared_dof_angles.valid[dof]) {
      LOG_ERROR("DOF " + String(dof) + " encoder invalid during position check");
      positions_ok = false;
      break;
    }
    float current_angle = shared_dof_angles.angles[dof];
    float phys_min = active_joint_controller->getConfig().dofs[dof].limits.min_angle;
    float phys_max = active_joint_controller->getConfig().dofs[dof].limits.max_angle;

    if (current_angle < (phys_min - POSITION_HARD_MARGIN) ||
        current_angle > (phys_max + POSITION_HARD_MARGIN)) {
      LOG_ERROR("DOF " + String(dof) + " position " + String(current_angle, 1) +
                "° is FAR outside physical limits [" + String(phys_min, 1) +
                ", " + String(phys_max, 1) + "]");
      positions_ok = false;
      break;
    }

    if (current_angle < (phys_min - POSITION_MARGIN) ||
        current_angle > (phys_max + POSITION_MARGIN)) {
      LOG_WARN("DOF " + String(dof) + " position " + String(current_angle, 1) +
               "° is outside physical limits [" + String(phys_min, 1) +
               ", " + String(phys_max, 1) + "] - proceeding with caution");
    } else if (current_angle < phys_min || current_angle > phys_max) {
      LOG_INFO("DOF " + String(dof) + " at " + String(current_angle, 1) +
               "° (slightly outside limits, acceptable)");
    } else {
      LOG_INFO("DOF " + String(dof) + " at " + String(current_angle, 1) +
               "° (within limits [" + String(phys_min, 1) + ", " + String(phys_max, 1) + "])");
    }
  }

  if (!positions_ok) {
    SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=POSITION_OUT_OF_RANGE");
    LOG_ERROR("Startup sequence failed: joint position too far outside limits");
    LOG_ERROR("Manually move joint to safe position before retrying");
    pushStartupEvent(STARTUP_EVT_FAILED, 0, STARTUP_REASON_POSITION_RANGE,
                     (uint16_t)(millis() - startup_start_time));
    return false;
  }
  LOG_INFO("Position limits verified");
  SERIAL_COM_LN("EVT:STARTUP_POSITIONS_OK(" + String(ACTIVE_JOINT) + ")");

  if (millis() - startup_start_time > STARTUP_TIMEOUT_MS) {
    SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=GLOBAL_TIMEOUT");
    LOG_ERROR("Startup sequence timed out before recalc");
    pushStartupEvent(STARTUP_EVT_FAILED, 0, STARTUP_REASON_GLOBAL_TIMEOUT,
                     (uint16_t)(millis() - startup_start_time));
    return false;
  }

  LOG_INFO("Starting startup sequence for joint " + String(ACTIVE_JOINT) + "...");
  SERIAL_COM_LN("EVT:STARTUP_BEGIN(" + String(ACTIVE_JOINT) + ")");
  pushStartupEvent(STARTUP_EVT_BEGIN, 0, STARTUP_REASON_OK,
                   (uint16_t)(millis() - startup_start_time));

  // Dispatch CMD_RECALC_OFFSET to Core1 for each DOF sequentially
  bool all_success = true;
  uint8_t last_successful_dof = 0;

  for (uint8_t dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
    if (millis() - startup_start_time > STARTUP_TIMEOUT_MS) {
      LOG_ERROR("Startup sequence timed out at DOF " + String(dof));
      SERIAL_COM_LN("EVT:STARTUP_DOF_FAILED(" + String(ACTIVE_JOINT) + "):DOF=" + String(dof) + ":REASON=TIMEOUT");
      pushStartupEvent(STARTUP_EVT_DOF_FAILED, dof, STARTUP_REASON_GLOBAL_TIMEOUT,
                       (uint16_t)(millis() - startup_start_time));
      all_success = false;
      break;
    }

    LOG_INFO("Running recalc_offset for DOF " + String(dof) + " (delegated to Core1)...");
    SERIAL_COM_LN("EVT:STARTUP_DOF_BEGIN(" + String(ACTIVE_JOINT) + "):DOF=" + String(dof));

    int pretension = (custom_torque > 0) ? (int)custom_torque : 0;
    int duration = (custom_duration > 0) ? (int)custom_duration : 0;

    // Clear shared_data_ext.flag before dispatching
    shared_data_ext.flag = 0;

    // Dispatch CMD_RECALC_OFFSET to Core1 via double-buffered command system
    int next_buf = (active_buffer + 1) % 2;
    while (buffer_ready[next_buf]) { sleep_us(100); }

    command_buffer[next_buf].joint_id = ACTIVE_JOINT;
    command_buffer[next_buf].dof_index = dof;
    command_buffer[next_buf].recalc_offset_torque = pretension;
    command_buffer[next_buf].recalc_offset_duration = duration;
    pending_command_type = CMD_RECALC_OFFSET;
    buffer_ready[next_buf] = true;
    active_buffer = next_buf;

    // Poll for Core1 completion while keeping encoders alive
    bool dof_timeout = false;
    while (shared_data_ext.flag == 0) {
      updateSharedDofAngles();

      if (emergency_stop_requested) {
        LOG_WARN("Emergency stop detected during startup polling");
        dof_timeout = true;
        break;
      }

      if (millis() - startup_start_time > STARTUP_TIMEOUT_MS) {
        dof_timeout = true;
        emergency_stop_requested = true;
        break;
      }
      delay(10);
    }

    if (dof_timeout) {
      LOG_ERROR("Startup timed out waiting for Core1 recalc on DOF " + String(dof));
      SERIAL_COM_LN("EVT:STARTUP_DOF_FAILED(" + String(ACTIVE_JOINT) + "):DOF=" + String(dof) + ":REASON=TIMEOUT");
      pushStartupEvent(STARTUP_EVT_DOF_FAILED, dof, STARTUP_REASON_GLOBAL_TIMEOUT,
                       (uint16_t)(millis() - startup_start_time));
      shared_data_ext.flag = 0;
      all_success = false;
      break;
    }

    // Process Core1 result
    if (shared_data_ext.flag == CMD1_END_MOVE) {
      last_successful_dof = dof;
      SERIAL_COM_LN("EVT:STARTUP_DOF_READY(" + String(ACTIVE_JOINT) + "):DOF=" + String(dof));
      LOG_INFO("DOF " + String(dof) + " recalc complete: " + String(shared_data_ext.message));
      pushStartupEvent(STARTUP_EVT_DOF_READY, dof, STARTUP_REASON_OK,
                       (uint16_t)(millis() - startup_start_time));
    } else {
      LOG_ERROR("recalc_offset failed for DOF " + String(dof) + ": " + String(shared_data_ext.message));
      SERIAL_COM_LN("EVT:STARTUP_DOF_FAILED(" + String(ACTIVE_JOINT) + "):DOF=" + String(dof) + ":REASON=RECALC");
      pushStartupEvent(STARTUP_EVT_DOF_FAILED, dof, STARTUP_REASON_RECALC_ERROR,
                       (uint16_t)(millis() - startup_start_time));
      all_success = false;
    }

    shared_data_ext.flag = 0;
    if (!all_success) break;
  }

  // RECOVERY: If failed partway through, request emergency stop
  if (!all_success) {
    emergency_stop_requested = true;
    delay(50);
    SERIAL_COM_LN("RSP:STARTUP_FAILED(" + String(ACTIVE_JOINT) + "):REASON=RECALC_ERROR:LAST_OK_DOF=" + String(last_successful_dof));
    LOG_ERROR("Startup sequence failed - emergency stop sent");
    pushStartupEvent(STARTUP_EVT_FAILED, last_successful_dof, STARTUP_REASON_RECALC_ERROR,
                     (uint16_t)(millis() - startup_start_time));
    return false;
  }

  // Save motor offsets to flash
  if (active_joint_controller->isPendingOffsetsSave()) {
    active_joint_controller->saveMotorOffsetsToFlash();
    active_joint_controller->clearPendingOffsetsSave();
    LOG_INFO("Motor offsets saved to flash after startup");
  }

  // === Inject HOLDING waypoints for all DOFs ===
  // After recalc, motors are stopped (torque 0). Injecting a waypoint at the
  // current angle transitions each DOF from IDLE → MOVING → HOLDING, so the PID
  // takes over and holds position under closed-loop control.
  // Safe to push from Core0: Core1 is idle (no active commands/waypoints).
  updateSharedDofAngles();  // Refresh angles right before injection
  uint32_t t_now_hold = millis();
  uint8_t dof_count_hold = active_joint_controller->getConfig().dof_count;

  for (uint8_t dof = 0; dof < dof_count_hold; dof++) {
    if (shared_dof_angles.valid[dof]) {
      float current_angle = shared_dof_angles.angles[dof];

      waypoint_buffer_set_prev(dof, current_angle, t_now_hold);

      WaypointEntry hold_wp{};
      hold_wp.dof_index = dof;
      hold_wp.target_angle_deg = current_angle;
      hold_wp.t_arrival_ms = t_now_hold + 100;  // 100ms — PID ramps up gently
      hold_wp.mode = 0;

      waypoint_buffer_push(dof, hold_wp);
      waypoint_buffer_set_state(dof, WaypointState::MOVING);

      LOG_INFO("DOF " + String(dof) + " entering HOLDING at " +
               String(current_angle, 1) + "\xC2\xB0");
    }
  }

  uint32_t total_time_ms = millis() - startup_start_time;
  SERIAL_COM_LN("RSP:STARTUP_COMPLETE(" + String(ACTIVE_JOINT) + "):TIME_MS=" + String(total_time_ms));
  LOG_INFO("Startup sequence complete in " + String(total_time_ms) + "ms — holding position");
  pushStartupEvent(STARTUP_EVT_COMPLETE, 0, STARTUP_REASON_OK, (uint16_t)total_time_ms);
  return true;
}

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
 * 500Hz (inner), with the outer loop running every outer_loop_divisor cycles, so
 * 500Hz encoder updates are sufficient.
 * 
 * DIRECT ENCODER READING: Uses DirectEncoders class to read MT6835 sensors
 * directly via SPI0, without intermediate encoder Pico.
 */
void updateSharedDofAngles() {
  static uint32_t last_update_us = 0;
  static uint32_t last_encoder_read_us = 0;
  
  // Synchronize encoder reads with inner loop period for consistent control timing
  // This ensures encoder data is fresh for each control cycle
  // Minimum 500µs (2kHz max) for SPI bus safety
  uint32_t read_interval = inner_loop_period_us;
  if (read_interval < 500) read_interval = 500;  // Minimum 500µs
  
  JointController *controller = active_joint_controller;
  if (controller == nullptr) {
    return;
  }
  
  uint32_t now_us = time_us_32();
  
  // Skip encoder read if not enough time has passed
  if (last_encoder_read_us > 0 && (now_us - last_encoder_read_us) < read_interval) {
    return;  // Keep previous values, they're still fresh enough
  }

  // Consecutive error counters for emergency stop (one per DOF)
  static uint16_t consecutive_errors[MAX_DOFS] = {0};
  // Calculate threshold in cycles from configurable ms value
  // Example: 100ms with 2000µs read interval = 50 cycles
  uint16_t error_threshold_cycles = (encoder_error_threshold_ms * 1000) / read_interval;
  if (error_threshold_cycles < 5) error_threshold_cycles = 5;  // Minimum 5 cycles for safety
  
  // Update encoder data from hardware (direct SPI to MT6835)
  directEncoders.update();
  last_encoder_read_us = now_us;
  
  // Calculate time delta for velocity
  float dt_s = (last_update_us > 0) ? (now_us - last_update_us) / 1000000.0f : 0.0f;
  
  uint8_t dof_count = controller->getConfig().dof_count;
  shared_dof_angles.dof_count = dof_count;

  // Begin sequence-locked write
  __atomic_fetch_add(&shared_dof_angles.seq, 1, __ATOMIC_ACQ_REL);
  
  for (uint8_t dof = 0; dof < dof_count; dof++) {
    // Use encoder_channel from config (may differ from dof index depending on wiring)
    uint8_t enc_ch = controller->getConfig().dofs[dof].encoder_channel;
    
    // Check if this encoder had a valid read
    bool is_valid = directEncoders.isEncoderConnected(enc_ch) && 
                    (directEncoders.getErrorCount(enc_ch) == 0);
    
    if (is_valid) {
      // Get angle directly from DirectEncoders (already in degrees with multi-turn)
      float new_angle = directEncoders.getAngle(enc_ch);
      
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
      if (consecutive_errors[dof] >= error_threshold_cycles && !emergency_stop_requested) {
        emergency_stop_requested = true;
        LOG_ERROR("[SAFETY] EMERGENCY STOP: Encoder DOF " + String(dof) +
                  " failed " + String(error_threshold_cycles) + " consecutive reads (" +
                  String(encoder_error_threshold_ms) + "ms)");
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

  // End sequence-locked write
  __atomic_fetch_add(&shared_dof_angles.seq, 1, __ATOMIC_ACQ_REL);
}

// ============================================================================
// CORE0 MAIN LOOP
// ============================================================================

void core0_main_loop() {

#pragma region Init Core1 and SharedData
  // Start the second core if it is not already running
  // Using 8KB stack (default is 4KB) to prevent stack overflow in
  // executeWaypointMovement() which has large stack frame from cascade control,
  // metrics tracking, compliance detection, and String-based LOG_INFO calls.
  static uint32_t core1_stack[2048];  // 8KB (2048 × 4 bytes)
  if (init_prg) {
    multicore_launch_core1_with_stack(core1_loop, core1_stack, sizeof(core1_stack));
    init_prg = false;
  }
#pragma endregion

  // LED mirrors motor power gate state (24V): ON = motors powered, OFF = power cut
  // Rate-limited to ~2Hz to avoid unnecessary GPIO writes
  {
    static uint32_t last_led_update_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_led_update_ms >= 500) {
      last_led_update_ms = now_ms;
      digitalWrite(LED_BUILTIN, safety_is_motor_power_enabled() ? HIGH : LOW);
    }
  }

  // Update shared DOF angles from encoders (single read point for entire system)
  updateSharedDofAngles();

  // Drain Core1 log queue — print queued messages safely from Core0
  drainCore1LogQueue();

  // NOTE: CAN polling has been moved to Core1 to avoid SPI1 conflicts
  // Core1 now handles all CAN communication (Host + Motor)

#pragma region Joint Identification Broadcast
  // Periodic EVT:JOINT emission when identification broadcast is active
  if (identify_broadcast_active) {
    uint32_t now = millis();
    
    // Check if broadcast duration has expired
    if (now - identify_broadcast_start_ms >= IDENTIFY_BROADCAST_DURATION_MS) {
      identify_broadcast_active = false;
      LOG_INFO("[IDENTIFY] Broadcast ended");
    } 
    // Check if it's time to emit
    else if (now - identify_broadcast_last_emit_ms >= IDENTIFY_BROADCAST_INTERVAL_MS) {
      identify_broadcast_last_emit_ms = now;
      
      // Emit joint identification event
      SERIAL_COM("EVT:JOINT ");
      SERIAL_COM(ACTIVE_JOINT);
      SERIAL_COM(" ");
      SERIAL_COM_LN(ACTIVE_JOINT_CONFIG.name);
    }
  }
#pragma endregion

#pragma region CAN Startup Sequence Poll
  // Poll CAN-triggered startup sequence flag (set by Core1 on CAN 0x009 or auto-start)
  if (can_startup_requested) {
    can_startup_requested = false;
    LOG_INFO("[CAN] Startup sequence requested: torque=" + String(can_startup_torque) +
             " duration=" + String(can_startup_duration));
    executeStartupSequence(can_startup_torque, can_startup_duration);
  }
#pragma endregion

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
      SERIAL_COM_LN(command);

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
                  SERIAL_COM_LN("RSP:STATUS(" + String(ACTIVE_JOINT) + "," +
                                 (ready ? "READY" : "NOT_READY") + ")");
                } else {
                  SERIAL_COM_LN("RSP:ERROR: Controller not initialized");
                }
                break;
              }
              
              case CMD_STOP: {
                // Emergency stop - forward to Core1
                emergency_stop_requested = true;
                SERIAL_COM_LN("RSP:STOP");
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
                SERIAL_COM_LN("RSP:ERROR: Command targeted to joint " + String(parsed_cmd.joint_id) +
                               ", but this device controls joint " + String(ACTIVE_JOINT));
              } else {
                // Set the joint ID to the active one (if it was 0 or already correct)
                parsed_cmd.joint_id = ACTIVE_JOINT;

                if (parsed_cmd.command == CMD_UNKNOWN) {
                  SERIAL_COM_LN("RSP:ERROR: Unsupported command: " +
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
                            SERIAL_COM_LN("EVT:PID:" + String(parsed_cmd.dof_index) + ":" + String(motor_index) + ":" +
                                           String(kp, 6) + ":" + String(ki, 6) + ":" +
                                           String(kd, 6) + ":" + String(tau, 6));
                          } else {
                            SERIAL_COM_LN("RSP:ERROR: Failed to get PID parameters");
                          }
                        } else {
                          SERIAL_COM_LN("RSP:ERROR: Invalid or missing motor index (expected 1 or 2)");
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized or invalid DOF");
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
                            SERIAL_COM_LN("RSP:PID_SET_OK(" + String(ACTIVE_JOINT) + "," +
                                           String(parsed_cmd.dof_index) + "," + String(motor_index) + ")");
                          } else {
                            SERIAL_COM_LN("RSP:ERROR: Failed to set PID parameters");
                          }
                        } else {
                          SERIAL_COM_LN("RSP:ERROR: Invalid parameters (expected motor_index,kp,ki,kd,tau)");
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized or invalid DOF");
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
                          SERIAL_COM_LN("EVT:PID_OUTER:" + String(parsed_cmd.dof_index) + ":" +
                                         String(kp, 6) + ":" + String(ki, 6) + ":" + String(kd, 6) + ":" +
                                         String(stiffness_deg, 6) + ":" + String(cascade_influence, 6));
                        } else {
                          SERIAL_COM_LN("RSP:ERROR: Failed to get outer loop PID parameters");
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized or invalid DOF");
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
                            SERIAL_COM_LN("RSP:PID_OUTER_SET_OK(" + String(ACTIVE_JOINT) + "," +
                                           String(parsed_cmd.dof_index) + ")");
                          } else {
                            SERIAL_COM_LN("RSP:ERROR: Failed to set outer loop PID parameters");
                          }
                        } else {
                          SERIAL_COM_LN("RSP:ERROR: Invalid parameters (expected kp,ki,kd,stiffness,influence)");
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized or invalid DOF");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_SAVE_PID: {
                      // Save PID parameters to flash memory
                      if (active_joint_controller != nullptr) {
                        if (active_joint_controller->savePIDDataToFlash()) {
                          SERIAL_COM_LN("RSP:PID_SAVED(" + String(ACTIVE_JOINT) + ")");
                          LOG_INFO("PID parameters saved to flash for joint " + String(ACTIVE_JOINT));
                        } else {
                          SERIAL_COM_LN("RSP:ERROR: Failed to save PID parameters to flash");
                          LOG_ERROR("Failed to save PID parameters for joint " + String(ACTIVE_JOINT));
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_LOAD_PID: {
                      // Load PID parameters from flash memory
                      if (active_joint_controller != nullptr) {
                        if (active_joint_controller->loadPIDDataFromFlash()) {
                          SERIAL_COM_LN("RSP:PID_LOADED(" + String(ACTIVE_JOINT) + ")");
                          LOG_INFO("PID parameters loaded from flash for joint " + String(ACTIVE_JOINT));
                        } else {
                          SERIAL_COM_LN("RSP:ERROR: Failed to load PID parameters from flash");
                          LOG_ERROR("Failed to load PID parameters for joint " + String(ACTIVE_JOINT));
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_RECALC_SAFE_LIMITS: {
                      // Recalculate safe limits from current equations using updated algorithm
                      if (active_joint_controller != nullptr) {
                        if (active_joint_controller->recalculateSafeLimits()) {
                          SERIAL_COM_LN("RSP:SAFE_LIMITS_RECALCULATED(" + String(ACTIVE_JOINT) + ")");
                        } else {
                          SERIAL_COM_LN("RSP:SAFE_LIMITS_RECALC_FAILED(" + String(ACTIVE_JOINT) + ")");
                          LOG_ERROR("Failed to recalculate safe limits for joint " + String(ACTIVE_JOINT));
                        }
                      } else {
                        SERIAL_COM_LN("RSP:ERROR: Controller not initialized");
                      }
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_SET_AUTO_START: {
                      // Enable or disable auto-start on boot (persistent setting)
                      // Parse ENABLED parameter (1=enable, 0=disable) from original command
                      // Command format: JOINT:DOF:SET_AUTO_START:ENABLED=1
                      bool enable = false;
                      const char* cmd_str = parsed_cmd.original_command;
                      char *enabled_str = strstr(cmd_str, "ENABLED=");
                      if (enabled_str != nullptr) {
                        enable = (atoi(enabled_str + 8) != 0);
                      }
                      
                      // Update system settings
                      system_settings.auto_start_enabled = enable ? 1 : 0;
                      system_settings.joint_type = ACTIVE_JOINT;
                      
                      // Parse optional custom parameters
                      char *torque_str = strstr(cmd_str, "TORQUE=");
                      if (torque_str != nullptr) {
                        system_settings.auto_start_pretension = atof(torque_str + 7);
                      }
                      char *duration_str = strstr(cmd_str, "DURATION=");
                      if (duration_str != nullptr) {
                        system_settings.auto_start_duration = atoi(duration_str + 9);
                      }
                      
                      // Save to flash
                      save_system_settings_data(system_settings);
                      system_settings_loaded = true;
                      
                      SERIAL_COM_LN("RSP:AUTO_START_SET(" + String(ACTIVE_JOINT) + "):ENABLED=" + String(enable ? 1 : 0));
                      LOG_INFO("Auto-start " + String(enable ? "ENABLED" : "DISABLED") + " for joint " + String(ACTIVE_JOINT));
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_GET_AUTO_START: {
                      // Query current auto-start setting
                      SERIAL_COM_LN("RSP:AUTO_START(" + String(ACTIVE_JOINT) + "):ENABLED=" + 
                                     String(system_settings.auto_start_enabled ? 1 : 0) +
                                     ":TORQUE=" + String(system_settings.auto_start_pretension, 1) +
                                     ":DURATION=" + String(system_settings.auto_start_duration));
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_CASCADE_SPEED_SCALING: {
                      // Set velocity-dependent stiffness scaling parameters
                      // Format: JOINT:DOF:CASCADE_SPEED_SCALING:ENABLED=1:MIN=0.3:LOW=3.0:HIGH=15.0
                      const char* cmd_str = parsed_cmd.original_command;

                      char *en_str = strstr(cmd_str, "ENABLED=");
                      if (en_str != nullptr) {
                        cascade_speed_scaling_enabled = (atoi(en_str + 8) != 0);
                      }
                      char *min_str = strstr(cmd_str, "MIN=");
                      if (min_str != nullptr) {
                        float val = atof(min_str + 4);
                        if (val >= 0.0f && val <= 1.0f) cascade_min_factor = val;
                      }
                      char *low_str = strstr(cmd_str, "LOW=");
                      if (low_str != nullptr) {
                        float val = atof(low_str + 4);
                        if (val >= 0.0f && val <= 100.0f) cascade_speed_low = val;
                      }
                      char *high_str = strstr(cmd_str, "HIGH=");
                      if (high_str != nullptr) {
                        float val = atof(high_str + 5);
                        if (val >= 0.0f && val <= 100.0f) cascade_speed_high = val;
                      }

                      // EMA filter parameters (optional, in same command)
                      char *ema_en_str = strstr(cmd_str, "EMA_EN=");
                      if (ema_en_str != nullptr) {
                        motor_ema_enabled = (atoi(ema_en_str + 7) != 0);
                      }
                      char *ema_a_str = strstr(cmd_str, "EMA_ALPHA=");
                      if (ema_a_str != nullptr) {
                        float val = atof(ema_a_str + 10);
                        if (val >= 0.05f && val <= 1.0f) motor_ema_alpha = val;
                      }

                      // Inner PID tau scaling parameters (optional, in same command)
                      char *tau_en_str = strstr(cmd_str, "TAU_EN=");
                      if (tau_en_str != nullptr) {
                        inner_tau_scaling_enabled = (atoi(tau_en_str + 7) != 0);
                      }
                      char *tau_h_str = strstr(cmd_str, "TAU_HIGH=");
                      if (tau_h_str != nullptr) {
                        float val = atof(tau_h_str + 9);
                        if (val >= 0.005f && val <= 0.2f) inner_tau_high = val;
                      }
                      char *tau_s_str = strstr(cmd_str, "TAU_SPEED=");
                      if (tau_s_str != nullptr) {
                        float val = atof(tau_s_str + 10);
                        if (val >= 0.1f && val <= 100.0f) inner_tau_speed_threshold = val;
                      }

                      // Joint angle EMA filter parameters (optional, in same command)
                      char *jema_en_str = strstr(cmd_str, "JEMA_EN=");
                      if (jema_en_str != nullptr) {
                        joint_ema_enabled = (atoi(jema_en_str + 8) != 0);
                      }
                      char *jema_a_str = strstr(cmd_str, "JEMA_ALPHA=");
                      if (jema_a_str != nullptr) {
                        float val = atof(jema_a_str + 11);
                        if (val >= 0.05f && val <= 1.0f) joint_ema_alpha = val;
                      }
                      char *jema_s_str = strstr(cmd_str, "JEMA_SPEED=");
                      if (jema_s_str != nullptr) {
                        float val = atof(jema_s_str + 11);
                        if (val >= 0.1f && val <= 100.0f) joint_ema_speed_threshold = val;
                      }

                      SERIAL_COM_LN("RSP:CASCADE_SPEED_SCALING:ENABLED=" +
                                     String(cascade_speed_scaling_enabled ? 1 : 0) +
                                     ":MIN=" + String(cascade_min_factor, 2) +
                                     ":LOW=" + String(cascade_speed_low, 1) +
                                     ":HIGH=" + String(cascade_speed_high, 1) +
                                     ":EMA_EN=" + String(motor_ema_enabled ? 1 : 0) +
                                     ":EMA_ALPHA=" + String(motor_ema_alpha, 2) +
                                     ":TAU_EN=" + String(inner_tau_scaling_enabled ? 1 : 0) +
                                     ":TAU_HIGH=" + String(inner_tau_high, 4) +
                                     ":TAU_SPEED=" + String(inner_tau_speed_threshold, 1) +
                                     ":JEMA_EN=" + String(joint_ema_enabled ? 1 : 0) +
                                     ":JEMA_ALPHA=" + String(joint_ema_alpha, 2) +
                                     ":JEMA_SPEED=" + String(joint_ema_speed_threshold, 1));
                      LOG_INFO("[CASCADE] Speed scaling: en=" + String(cascade_speed_scaling_enabled) +
                               " min=" + String(cascade_min_factor, 2) +
                               " low=" + String(cascade_speed_low, 1) +
                               " high=" + String(cascade_speed_high, 1) +
                               " ema_en=" + String(motor_ema_enabled) +
                               " ema_a=" + String(motor_ema_alpha, 2) +
                               " tau_en=" + String(inner_tau_scaling_enabled) +
                               " tau_h=" + String(inner_tau_high, 4) +
                               " tau_spd=" + String(inner_tau_speed_threshold, 1) +
                               " jema_en=" + String(joint_ema_enabled) +
                               " jema_a=" + String(joint_ema_alpha, 2) +
                               " jema_spd=" + String(joint_ema_speed_threshold, 1));
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_STARTUP_SEQUENCE: {
                      // Manual startup sequence via Serial
                      // Delegates to shared executeStartupSequence() (also used by CAN 0x009 and auto-start)
                      int pretension = system_settings.auto_start_pretension > 0
                          ? (int)system_settings.auto_start_pretension : 0;
                      int duration = system_settings.auto_start_duration > 0
                          ? system_settings.auto_start_duration : 0;
                      executeStartupSequence(pretension, duration);
                      handled_on_core0 = true;
                      break;
                    }

                    case CMD_START_TEST_ENCODER: {
                      // Activate encoder streaming via CAN ONLY (handled by Core1)
                      // Serial streaming is disabled to avoid conflicts and data corruption
                      encoder_stream_can_active = true;
                      encoder_stream_last_send_us = time_us_32() - 100000; // Force immediate first send
                      
                      // DISABLE Serial streaming to avoid conflicts with CAN data
                      encoder_test_active    = false;
                      encoder_test_joint_id  = parsed_cmd.joint_id;
                      encoder_test_dof_index = parsed_cmd.dof_index;
                      encoder_test_all_dofs  = parsed_cmd.all_dofs;
                      last_encoder_test_time = millis();
                      handled_on_core0       = true;

                      if (encoder_test_all_dofs) {
                        LOG_INFO("Encoder streaming enabled via CAN @ 50Hz for ALL DOFs");
                      } else {
                        LOG_INFO("Encoder streaming enabled via CAN @ 50Hz for DOF " + String(parsed_cmd.dof_index));
                      }
                      break;
                    }

                    case CMD_STOP_TEST_ENCODER: {
                      // Deactivate CAN encoder streaming
                      encoder_stream_can_active = false;
                      encoder_test_active   = false;
                      encoder_test_all_dofs = false;
                      handled_on_core0      = true;
                      LOG_INFO("Encoder streaming disabled");
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

                    case CMD_SET_ZERO_CURRENT_POS: {
                      // Set zero: joint encoder on Core0, motor encoders delegated to Core1
                      JointController *ctrl = active_joint_controller;
                      if (ctrl != nullptr) {
                        uint8_t dof = parsed_cmd.dof_index;
                        const JointConfig& cfg = ctrl->getConfig();
                        if (dof < cfg.dof_count) {
                          uint8_t enc_channel = cfg.dofs[dof].encoder_channel;
                          float zero_offset = cfg.dofs[dof].zero_mapping.zero_angle_offset;
                          
                          // 1. Reset joint encoder (MT6835) - Core0
                          directEncoders.requestReset(enc_channel, zero_offset);
                          
                          LOG_INFO_F("Set Zero: DOF %d → joint encoder target %.2f°", dof, zero_offset);
                          
                          // 2. Delegate motor encoder zeroing to Core1 (requires CAN access)
                          int next_buffer = (active_buffer + 1) % 2;
                          command_buffer[next_buffer].joint_id = parsed_cmd.joint_id;
                          command_buffer[next_buffer].dof_index = dof;
                          pending_command_type = CMD_ZERO_MOTOR_ENCODERS;
                          buffer_ready[next_buffer] = true;
                          active_buffer = next_buffer;
                          
                          LOG_DEBUG("Delegating motor encoder zero to Core1");
                          
                          // Signal completion (Core1 will do motor zeroing async)
                          shared_data_ext.dof_index = dof;
                          shared_data_ext.flag = CMD1_END_ZERO;
                          strcpy(shared_data_ext.message, "Zero position set");
                        } else {
                          LOG_ERROR("Invalid DOF index for Set Zero");
                        }
                      }
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
        SERIAL_COM_LN("RSP:ERROR: Commands must start with CMD: prefix");
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
      SERIAL_COM_LN("RSP:MOVE_COMPLETE(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      // Check if motor offsets need saving to flash (after recalc_offset)
      if (active_joint_controller != nullptr && active_joint_controller->isPendingOffsetsSave()) {
        if (active_joint_controller->saveMotorOffsetsToFlash()) {
          LOG_INFO("Motor offsets saved to flash after recalc");
        } else {
          LOG_WARN("Failed to save motor offsets to flash");
        }
        active_joint_controller->clearPendingOffsetsSave();
      }
      break;

    case CMD1_FAIL_MOVE:
      SERIAL_COM_LN("RSP:MOVE_FAILED(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      break;

    case CMD1_END_ZERO:
      SERIAL_COM_LN("RSP:ZERO_COMPLETE(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.dof_index) + ")");
      break;

    case CMD1_AUTO_MAP_PROGRESS:
      SERIAL_COM_LN("RSP:AUTO_MAP_PROGRESS(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      break;

    case CMD1_AUTO_MAP_COMPLETE:
      SERIAL_COM_LN("RSP:AUTO_MAP_COMPLETE(" + String(shared_data_ext.joint_id) + "," +
                     String(shared_data_ext.message) + ")");
      // Signal that mapping data is ready to be sent
      auto_mapping_data_ready_to_send = true;
      
      // Check if Core1 requested a flash save for linear equations
      if (active_joint_controller != nullptr && active_joint_controller->isPendingFlashSave()) {
        // Print equation summary (safe to do from Core0)
        SERIAL_COM_LN("=== LINEAR EQUATIONS SUMMARY ===");
        for (int dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
          DofLinearEquations *eq = active_joint_controller->getLinearEquations(dof);
          if (eq != nullptr && eq->calculated) {
            SERIAL_COM_LN("DOF " + String(dof) + ":");
            SERIAL_COM_LN("  Agonist: y = " + String(eq->agonist.slope, 4) +
                           "*x + " + String(eq->agonist.intercept, 4) +
                           " (R²=" + String(eq->agonist.r_squared, 3) + ")");
            SERIAL_COM_LN("  Antagonist: y = " + String(eq->antagonist.slope, 4) +
                           "*x + " + String(eq->antagonist.intercept, 4) +
                           " (R²=" + String(eq->antagonist.r_squared, 3) + ")");
            SERIAL_COM_LN("  Joint range: [" + String(eq->joint_safe_min, 1) + ", " +
                           String(eq->joint_safe_max, 1) + "]°");
          }
        }
        SERIAL_COM_LN("================================");
        
        // Save equations to flash (from Core0 - thread safe)
        if (active_joint_controller->saveLinearEquationsToFlash()) {
          SERIAL_COM_LN("RSP:LINEAR_EQUATIONS_SAVED(" + String(shared_data_ext.joint_id) + ")");
        } else {
          SERIAL_COM_LN("RSP:LINEAR_EQUATIONS_SAVE_FAILED(" + String(shared_data_ext.joint_id) + ")");
        }
        active_joint_controller->clearPendingFlashSave();
      }
      break;

    default:
      SERIAL_COM_LN("RSP:UNKNOWN_FLAG(" + String(shared_data_ext.flag) + ")");
      break;
    }

    // Reset flag (command processed)
    shared_data_ext.flag = 0;
  }
#pragma endregion

#pragma region Streaming Data

  // Stream encoder data if measuring is active (uses shared_dof_angles)
  // NOTE: Suspend measuring streaming during movement to avoid Serial conflicts with Core1
  if (measuring_data_ext.flag == 1 && !movement_in_progress) {
    uint8_t dof = measuring_data_ext.dof_index;
    if (dof < shared_dof_angles.dof_count && shared_dof_angles.valid[dof]) {
      SERIAL_COM_LN("EVT:ANGLE(" + String(ACTIVE_JOINT) + "," + String(dof) + "," +
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
        SERIAL_COM_LN("EVT:MAPPING_DATA(" + String(reference_size) + "," +
                       String(controller->getConfig().dof_count) + ")");

        // Send data for each DOF with extended format
        // Format: {prefix}{dof_index}_{array_index}_{data}
        // Prefixes: a=joint, b=agonist, c=antagonist (according to MAPPING_DATA_PROTOCOL.md)
        // Uses fixed stack buffer instead of String concatenation to avoid heap fragmentation
        {
          char line_buf[256];  // Fixed buffer — avoids hundreds of String allocs
          int dof_count = controller->getConfig().dof_count;

          for (int i = 0; i < reference_size; i++) {
            int pos = 0;
            pos += snprintf(line_buf + pos, sizeof(line_buf) - pos, "EVT:");

            for (int dof = 0; dof < dof_count; dof++) {
              DofMappingData_t *mapping_data = controller->getMappingData(dof);
              if (mapping_data != nullptr && mapping_data->flag == 1) {
                pos += snprintf(line_buf + pos, sizeof(line_buf) - pos,
                                "a%d_%d_%.4f|b%d_%d_%.4f|c%d_%d_%.4f",
                                dof, i, mapping_data->joint_data[i],
                                dof, i, mapping_data->agonist_data[i],
                                dof, i, mapping_data->antagonist_data[i]);
                if (dof < dof_count - 1) {
                  pos += snprintf(line_buf + pos, sizeof(line_buf) - pos, "|");
                }
              }
            }

            SERIAL_COM_LN(line_buf);
          }
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
  // NOTE: Suspend encoder streaming during movement to avoid Serial conflicts with Core1
  if (encoder_test_active && !movement_in_progress) {
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
              SERIAL_COM_LN(buffer);
            } else {
              char buffer[100];
              snprintf(buffer, sizeof(buffer), "EVT:ENCODER_DATA:DOF=%d:ERROR=Invalid encoder data", dof);
              SERIAL_COM_LN(buffer);
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
            SERIAL_COM_LN(buffer);
          } else {
            SERIAL_COM_LN("EVT:ENCODER_DATA:ERROR=Invalid encoder data");
          }
        }
      } else {
        SERIAL_COM_LN("EVT:ENCODER_DATA:ERROR=Controller not found");
        encoder_test_active = false; // Disable test on error
      }
    }
  }
#pragma endregion
}

