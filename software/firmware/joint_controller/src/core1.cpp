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
#include "hardware/sync.h"

// RAM-resident function for waiting during flash operations
// This MUST be in RAM because flash is inaccessible during erase/program
static void __not_in_flash_func(wait_for_flash_complete)(void) {
  // Signal Core0 that we are safely parked in RAM
  core1_flash_acknowledged = true;
  while (flash_operation_in_progress) {
    tight_loop_contents();  // CPU-friendly busy wait
  }
  // Reset acknowledgment for next flash operation
  core1_flash_acknowledged = false;
}

// ============================================================================
// DUAL CAN BUS ARCHITECTURE
// ============================================================================
// J4 CAN_Servo (Motor CAN): GP9=CS, GP13=INT - Motor commands via LKM_Motor
// J5 CAN_Controller (Host CAN): GP8=CS, GP14=INT - Host commands (TimeSync, Waypoints)
// Both share SPI1 (GP10=SCK, GP11=MOSI, GP12=MISO) with different CS pins
// ============================================================================

// CAN ID ranges (from CAN_SYSTEM_ARCHITECTURE.md - Priority-Optimized)
// Priority Level 0 (Highest): Emergency
#define CAN_ID_EMERGENCY_STOP 0x000

// Priority Level 1: System Control
#define CAN_ID_TIME_SYNC 0x002
#define CAN_ID_ENCODER_STREAM_CTRL 0x003  // Encoder streaming control (start/stop)
#define CAN_ID_PID_DIAG_CTRL 0x004        // PID diagnostics streaming control
#define CAN_ID_INTERPOLATION_MODE 0x005   // Waypoint interpolation mode (linear/smooth)
#define CAN_ID_LOOP_FREQUENCY 0x006       // Control loop frequencies (inner/outer)
#define CAN_ID_PID_DIAG_FREQ 0x007        // PID diagnostics stream frequency
#define CAN_ID_IDENTIFY_REQUEST 0x008     // Joint identification request (broadcast)
#define CAN_ID_STARTUP_SEQUENCE 0x009     // Startup sequence command (Host → Controller)

// Priority Level 2: Motor Control (0x140-0x280) - handled by LKM_Motor library

// Priority Level 3: Trajectory Commands
#define CAN_ID_MULTI_DOF_WAYPOINT_BASE 0x380  // 0x380-0x39F for multi-DOF waypoints

// Priority Level 4: Status Feedback
#define CAN_ID_STATUS_BASE 0x400    // 0x400-0x4FF for status (NEW: was 0x200)
#define CAN_ID_ENCODER_STREAM_DATA 0x410  // Encoder streaming data (Controller → Host)
#define CAN_ID_PID_DIAG_DATA 0x420        // PID diagnostics (target, error) per joint
#define CAN_ID_PID_TORQUE_DATA 0x430      // PID torque commands per joint
#define CAN_ID_MOVEMENT_METRICS 0x440     // Movement metrics (per DOF, sent on HOLDING)
#define CAN_ID_SMOOTHNESS_METRICS 0x460   // Smoothness/oscillation metrics (per DOF)
#define CAN_ID_PID_INNER_TERMS 0x470      // Inner PID P/I/D/FF breakdown (optional)
#define CAN_ID_PID_OUTER_TERMS 0x480      // Outer PID P/I/D/output breakdown (optional)
#define CAN_ID_STARTUP_STATUS 0x490       // Startup status events (Controller → Host)
#define CAN_ID_JOINT_ANNOUNCE 0x4A0       // Joint announce/discovery (Controller → Host)
#define CAN_ID_GET_ENCODER_OFFSETS 0x00A  // Request encoder offsets (Host → Controller)
#define CAN_ID_SET_ZERO 0x00B             // Set-zero command (Host → Controller)
#define CAN_ID_PRETENSION 0x00C           // Pretension single DOF (Host → Controller)
#define CAN_ID_PRETENSION_ALL 0x00D       // Pretension all DOFs (Host → Controller)
#define CAN_ID_RELEASE 0x00E              // Release single DOF (Host → Controller)
#define CAN_ID_RELEASE_ALL 0x00F          // Release all DOFs (Host → Controller)
#define CAN_ID_RECALC_OFFSET 0x010        // Recalculate motor offsets (Host → Controller)
#define CAN_ID_SAVE_PID 0x011             // Save PID to flash (Host → Controller)
#define CAN_ID_LOAD_PID 0x012             // Load PID from flash (Host → Controller)
#define CAN_ID_SET_PID 0x013              // Set inner PID params (Host → Controller, multi-frame)
#define CAN_ID_SET_PID_OUTER 0x014        // Set outer PID params (Host → Controller, multi-frame)
#define CAN_ID_CASCADE_SPEED_SCALING 0x015 // Set cascade speed scaling (Host → Controller)
#define CAN_ID_START_AUTO_MAPPING 0x016   // Start auto-mapping (Host → Controller)
#define CAN_ID_STOP_AUTO_MAPPING 0x017    // Stop auto-mapping (Host → Controller)
#define CAN_ID_SAVE_LINEAR_EQ 0x018       // Save linear equations to flash (Host → Controller)
#define CAN_ID_LOAD_LINEAR_EQ 0x019       // Load linear equations from flash (Host → Controller)
#define CAN_ID_SET_AUTO_START 0x01A       // Set auto-start on boot (Host → Controller)
#define CAN_ID_WP_REANCHOR_INTERVAL 0x01B // Set waypoint re-anchor interval (Host → Controller)
#define CAN_ID_ENCODER_OFFSETS_DATA 0x4B0 // Encoder offsets response (Controller → Host, + joint_id)
#define CAN_ID_ZERO_COMPLETE 0x4C0        // Zero complete notification (Controller → Host, + joint_id)

// Encoder streaming configuration
#define ENCODER_STREAM_INTERVAL_US 20000  // 20ms = 50Hz (reduced for SLCAN compatibility)

// PID diagnostics streaming - configurable frequency
// Default: 50ms = 20Hz, can be changed via CAN for high-frequency training data
#define PID_DIAG_DEFAULT_INTERVAL_US 50000    // 50ms = 20Hz (normal monitoring)
volatile uint32_t pid_diag_interval_us = PID_DIAG_DEFAULT_INTERVAL_US;

// Interpolation modes
#define INTERPOLATION_LINEAR 0   // Linear interpolation (step response)
#define INTERPOLATION_COSINE 1   // S-curve cosine (smooth motion)

// Global interpolation mode (set via CAN command, used by waypoint execution)
volatile uint8_t waypoint_interpolation_mode = INTERPOLATION_LINEAR;

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
    LOG_C1_WARN("[CAN] Time Sync frame too short (" + String(len) + " bytes)");
    return;
  }

  // Parse timestamp (uint32_t, little-endian)
  uint32_t t_host_ms = 0;
  memcpy(&t_host_ms, data, sizeof(uint32_t));

  const uint32_t t_local = millis();
  sync_host_ms = t_host_ms;
  sync_local_ms = t_local;
  clock_synced = true;

  LOG_C1_INFO("[CAN] Time sync: host=" + String(t_host_ms) + " local=" + String(t_local) + " (synced)");
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
  // Extract joint_id from CAN ID (0x380 + joint_id)
  uint8_t waypoint_joint_id = id - CAN_ID_MULTI_DOF_WAYPOINT_BASE;
  
  // CRITICAL: Only process waypoints for THIS joint
  // Multiple controllers share the same CAN bus, each must filter by joint_id
  if (waypoint_joint_id != ACTIVE_JOINT) {
    // Not for this joint, ignore silently
    return;
  }
  
  if (len < 8) {
    LOG_C1_WARN("[CAN] Multi-DOF Waypoint frame too short (" + String(len) + " bytes)");
    return;
  }

  if (!clock_synced) {
    LOG_C1_WARN("[CAN] Multi-DOF Waypoint dropped: clock not synchronized");
    return;
  }
  
  // SAFETY: Require minimum uptime before accepting waypoints.
  // Wrap-safe: latch after first pass (check only relevant at boot, avoids
  // millis() wrap at ~49.71 days re-triggering the guard).
  static bool uptime_check_passed = false;
  if (!uptime_check_passed) {
    if (millis() < MIN_UPTIME_FOR_WAYPOINTS_MS) {
      LOG_C1_WARN("[CAN] Multi-DOF Waypoint dropped: system startup");
      return;
    }
    uptime_check_passed = true;
  }

  // SAFETY: Drop waypoints during startup injection (Core0 writing to buffer)
  if (__atomic_load_n(&startup_injecting_waypoints, __ATOMIC_ACQUIRE)) {
    LOG_C1_WARN("[CAN] Multi-DOF Waypoint dropped: startup injection in progress");
    return;
  }

  // SAFETY: Verify system is ready for movement
  if (active_joint_controller != nullptr && !active_joint_controller->isSystemReadyForMovement()) {
    LOG_C1_ERROR("[CAN] Multi-DOF Waypoint REJECTED: System not ready - run recalcOffset first!");
    return;
  }

  // Parse Multi-DOF waypoint
  // Format: 3× int16 angles (0.01° resolution, 0x7FFF = unused) + uint16 t_offset_ms
  // t_offset_ms is the desired arrival time relative to batch start (not compensated).
  // The firmware anchors all offsets to a single local timestamp captured at the
  // first waypoint of each batch, eliminating per-frame millis() jitter.
  struct {
    int16_t dof0_angle;    // 0.01° resolution, 0x7FFF = unused
    int16_t dof1_angle;    // 0.01° resolution, 0x7FFF = unused
    int16_t dof2_angle;    // 0.01° resolution, 0x7FFF = unused
    uint16_t t_offset_ms;  // Offset from batch start (host sends original, uncompensated)
  } __attribute__((packed)) multi_wp;

  memcpy(&multi_wp, data, sizeof(multi_wp));

  uint32_t t_now = getAbsoluteTimeMs();

  // --- Batch anchor timing ---
  // The host sends t_offset_ms as the desired arrival time relative to
  // batch start (uncompensated).  The firmware anchors all offsets to a
  // single local timestamp captured when the first waypoint of a batch
  // arrives, giving exact inter-WP spacing regardless of per-frame
  // millis() jitter.
  //
  // Anchor reset triggers:
  //   1. Any DOF transitioning IDLE/HOLDING → MOVING (new movement)
  //   2. Candidate arrival in the past (new streaming chunk whose
  //      t_offsets restart from a small lead value)
  static uint32_t batch_anchor_local_ms = 0;
  static bool batch_anchor_valid = false;

  // Check if any active DOF in this frame needs init (new batch)
  bool is_new_batch = false;
  {
    uint8_t dc = waypoint_buffers_get_dof_count();
    int16_t raw_angles[3] = {multi_wp.dof0_angle, multi_wp.dof1_angle, multi_wp.dof2_angle};
    for (uint8_t d = 0; d < 3 && d < dc; d++) {
      if (raw_angles[d] == MULTI_DOF_UNUSED) continue;
      WaypointState st = waypoint_buffer_state(d);
      if (st == WaypointState::IDLE || st == WaypointState::HOLDING) {
        is_new_batch = true;
        break;
      }
    }
  }

  if (is_new_batch || !batch_anchor_valid) {
    batch_anchor_local_ms = t_now;
    batch_anchor_valid = true;
    if (is_new_batch) {
      wp_reanchor_reset_all();
    }
  }

  // Candidate arrival based on current anchor
  uint32_t t_arrival_local = batch_anchor_local_ms + multi_wp.t_offset_ms;

  // If the candidate arrival is already in the past, this is a new
  // streaming chunk whose t_offsets are relative to "now" — re-anchor.
  if ((int32_t)(t_now - t_arrival_local) > 0) {
    batch_anchor_local_ms = t_now;
    t_arrival_local = t_now + multi_wp.t_offset_ms;
    // Reset re-anchor corrections: old corrections were relative to
    // the previous batch_anchor, no longer valid with the new anchor.
    wp_reanchor_reset_all();
  }

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
    // Monotonicity enforcement: ensure arrival times strictly increase per DOF.
    // Uses last_pushed_time (tail of queue), not prev_time (consumed/interpolation ref),
    // to catch out-of-order insertions relative to already-queued waypoints.
    uint32_t last_push_t = waypoint_buffer_last_pushed_time(dof);
    uint32_t arrival_ms = t_arrival_local;
    // Wrap-safe: signed difference detects backwards timestamps across uint32_t overflow
    if (last_push_t > 0 && (int32_t)(arrival_ms - last_push_t) <= 0) {
      arrival_ms = last_push_t + 1;
    }
    entry.t_arrival_ms = arrival_ms;
    entry.mode = 0;  // LINEAR interpolation
    
    // Check current state for this DOF
    WaypointState current_state = waypoint_buffer_state(dof);
    bool is_first_waypoint = (current_state == WaypointState::IDLE);
    bool needs_init = is_first_waypoint || (current_state == WaypointState::HOLDING);
    
    // Initialize movement if needed
    if (needs_init && active_joint_controller != nullptr) {
      bool is_valid = shared_dof_angles.valid[dof];
      float current_angle = shared_dof_angles.angles[dof];

      if (is_valid) {
        String safety_violation;
        if (!active_joint_controller->checkWaypointSafety(dof, current_angle,
                                                          entry.target_angle_deg, entry.t_arrival_ms,
                                                          t_now, safety_violation)) {
          LOG_C1_ERROR("[CAN SAFETY] Multi-DOF DOF" + String(dof) + ": " + safety_violation);
          emergency_stop_requested = true;
          return;
        }

        waypoint_buffer_set_prev(dof, current_angle, t_now);
        waypoint_buffer_set_state(dof, WaypointState::MOVING);

        if (is_first_waypoint) {
          LOG_C1_DEBUG("[CAN] DOF " + String(dof) + " IDLE → MOVING (multi-DOF)");
        }
      } else {
        // Encoder not valid during init — skip this DOF entirely
        LOG_C1_WARN("[CAN] Multi-DOF DOF" + String(dof) + " waypoint dropped: encoder not valid");
        continue;
      }
    } else if (active_joint_controller != nullptr) {
      // In-stream waypoint (already MOVING) — lightweight angle validation
      if (!active_joint_controller->isAngleInLimits(dof, entry.target_angle_deg)) {
        LOG_C1_ERROR("[CAN SAFETY] Multi-DOF DOF" + String(dof) +
                     " in-stream waypoint outside physical limits: " +
                     String(entry.target_angle_deg, 2) + " deg");
        emergency_stop_requested = true;
        return;
      }
      if (!active_joint_controller->isAngleInMappingLimits(dof, entry.target_angle_deg)) {
        LOG_C1_WARN("[CAN] Multi-DOF DOF" + String(dof) +
                    " in-stream waypoint outside mapping limits, skipped");
        continue;
      }

      // In-stream velocity check: compare against last pushed waypoint
      uint32_t last_t = waypoint_buffer_last_pushed_time(dof);
      float last_angle = waypoint_buffer_last_pushed_angle(dof);
      if (last_t > 0) {
        float dt_s = (float)(arrival_ms - last_t) / 1000.0f;
        if (dt_s > 0.001f) {
          float vel_deg_s = fabsf(entry.target_angle_deg - last_angle) / dt_s;

          // HARD CAP: 150 deg/s — matches host-side validation in
          // waypoint_types.py.  Batch-anchor timing eliminates the
          // per-frame millis() jitter that previously required a 20%
          // margin (was 180).  All arrival times are now computed from
          // a single reference point, so inter-WP dt is exact.
          const float ABSOLUTE_MAX_VELOCITY_DEG_S = 150.0f;
          if (vel_deg_s > ABSOLUTE_MAX_VELOCITY_DEG_S) {
            LOG_C1_ERROR("[CAN SAFETY] Multi-DOF DOF" + String(dof) +
                         " in-stream velocity " + String(vel_deg_s, 1) +
                         " deg/s exceeds hard limit");
            emergency_stop_requested = true;
            return;
          }

          // Per-DOF max_speed with 1.5x emergency margin
          float max_speed_rad_s = active_joint_controller->getConfig().dofs[dof].motion.max_speed;
          float max_speed_deg_s = max_speed_rad_s * RAD_TO_DEG;
          if (vel_deg_s > max_speed_deg_s * 1.5f) {
            LOG_C1_ERROR("[CAN SAFETY] Multi-DOF DOF" + String(dof) +
                         " in-stream velocity " + String(vel_deg_s, 1) +
                         " deg/s exceeds 1.5x max_speed " +
                         String(max_speed_deg_s, 1) + " deg/s");
            emergency_stop_requested = true;
            return;
          }
        }
      }
    }

    // Push to buffer
    if (waypoint_buffer_push(dof, entry)) {
      queued_count++;
    } else {
      LOG_C1_WARN("[CAN] Multi-DOF buffer full for DOF " + String(dof));
    }
  }
  
  // Log summary (throttled to avoid serial bottleneck)
  static uint16_t multi_dof_log_counter = 0;
  multi_dof_log_counter++;
  if (multi_dof_log_counter >= 50) {
    LOG_C1_INFO("[CAN] Multi-DOF: " + String(queued_count) + " DOFs queued, t_offset=" + 
             String(multi_wp.t_offset_ms) + "ms, t_arrival=" + String(t_arrival_local));
    multi_dof_log_counter = 0;
  }
}

/**
 * @brief Send encoder data via CAN for high-frequency streaming
 * 
 * Sends all DOF angles in a single CAN frame (same format as Multi-DOF Waypoint).
 * Called from core1_loop() at ~200Hz when streaming is active.
 * 
 * Frame format (8 bytes):
 * - Bytes 0-1: int16_t dof0_angle (0.01° resolution)
 * - Bytes 2-3: int16_t dof1_angle (0.01° resolution, 0x7FFF if invalid)
 * - Bytes 4-5: int16_t dof2_angle (0.01° resolution, 0x7FFF if invalid)
 * - Bytes 6-7: uint16_t t_offset_ms (ms since last time sync)
 */
void sendEncoderStreamData() {
  if (!encoder_stream_can_active) return;
  
  // Skip if Host CAN is suspended (SPI1 bus conflict avoidance)
  if (suspend_host_can_polling) return;
  
  // Check timing - only send every ENCODER_STREAM_INTERVAL_US (5ms = 200Hz)
  uint32_t now_us = time_us_32();
  if ((now_us - encoder_stream_last_send_us) < ENCODER_STREAM_INTERVAL_US) {
    return;
  }
  encoder_stream_last_send_us = now_us;
  
  extern MCP_CAN CAN_HOST;
  
  // Build encoder data frame (same format as Multi-DOF Waypoint for consistency)
  struct __attribute__((packed)) {
    int16_t dof0_angle;
    int16_t dof1_angle;
    int16_t dof2_angle;
    uint16_t t_offset_ms;
  } frame;
  
  // Read angles from shared_dof_angles (updated by Core0)
  // NOTE: shared_dof_angles.angles[] are ALREADY in degrees (from DirectEncoders)
  for (uint8_t dof = 0; dof < 3; dof++) {
    if (dof < MAX_DOFS && shared_dof_angles.valid[dof]) {
      // Convert to 0.01° resolution (same as waypoint format)
      float angle_deg = shared_dof_angles.angles[dof];  // Already in degrees
      int16_t angle_int = (int16_t)(angle_deg * 100.0f);
      
      if (dof == 0) frame.dof0_angle = angle_int;
      else if (dof == 1) frame.dof1_angle = angle_int;
      else frame.dof2_angle = angle_int;
    } else {
      // Mark as invalid/unused
      if (dof == 0) frame.dof0_angle = MULTI_DOF_UNUSED;
      else if (dof == 1) frame.dof1_angle = MULTI_DOF_UNUSED;
      else frame.dof2_angle = MULTI_DOF_UNUSED;
    }
  }
  
  // Timestamp: ms since sync (or just millis() if not synced)
  frame.t_offset_ms = (uint16_t)(millis() & 0xFFFF);
  
  // Send via Host CAN - use joint-specific CAN ID (0x410 + joint_id)
  // This allows filtering by joint when multiple controllers are on the bus
  uint32_t can_id = CAN_ID_ENCODER_STREAM_DATA + ACTIVE_JOINT;
  uint8_t result = CAN_HOST.sendMsgBuf(can_id, 0, sizeof(frame), (uint8_t*)&frame);
  
  // Debug log (throttled to 10s to reduce log spam)
  static uint32_t last_debug_log = 0;
  static uint32_t frame_count = 0;
  static uint32_t error_count = 0;
  frame_count++;
  if (result != CAN_OK) error_count++;
  
  if (millis() - last_debug_log > 10000) {
    // Only log if there were errors - silence is golden for normal operation
    if (error_count > 0) {
      LOG_C1_WARN("[CAN] Encoder stream: " + String(error_count) + "/" + String(frame_count) + " errors");
    }
    frame_count = 0;
    error_count = 0;
    last_debug_log = millis();
  }
}

/**
 * @brief Send PID diagnostic data via CAN for tuning
 * 
 * Sends two CAN frames:
 * - Frame 1 (0x420 + joint): target and error for each DOF
 * - Frame 2 (0x430 + joint): torque commands for each DOF pair
 * 
 * Called from core1_loop() at ~20Hz when diagnostic streaming is active.
 */
void sendPIDDiagStreamData() {
  if (!pid_diag_stream_active) return;
  if (!pid_diagnostics.valid) return;
  
  // Check timing - only send at configured interval (configurable via CAN)
  static uint32_t last_send_us = 0;
  uint32_t now_us = time_us_32();
  if ((now_us - last_send_us) < pid_diag_interval_us) {
    return;
  }
  last_send_us = now_us;
  
  extern MCP_CAN CAN_HOST;
  
  // Frame 1: Target and Error (0x420 + joint)
  // Format: [target0, target1, error0, error1] - 8 bytes for 2 DOFs
  struct __attribute__((packed)) {
    int16_t target0;   // Target DOF0 (°×100)
    int16_t target1;   // Target DOF1 (°×100)
    int16_t error0;    // Error DOF0 (°×100)
    int16_t error1;    // Error DOF1 (°×100)
  } frame1;
  
  frame1.target0 = pid_diagnostics.target_deg_x100[0];
  frame1.target1 = pid_diagnostics.target_deg_x100[1];
  frame1.error0 = pid_diagnostics.error_deg_x100[0];
  frame1.error1 = pid_diagnostics.error_deg_x100[1];
  
  uint32_t can_id_1 = CAN_ID_PID_DIAG_DATA + ACTIVE_JOINT;
  CAN_HOST.sendMsgBuf(can_id_1, 0, sizeof(frame1), (uint8_t*)&frame1);
  
  // Frame 2: Torque commands (0x430 + joint)
  // Format: [torqueA0, torqueB0, torqueA1, torqueB1] - 8 bytes for 2 DOFs
  struct __attribute__((packed)) {
    int16_t torque_A0;  // Torque agonist DOF0
    int16_t torque_B0;  // Torque antagonist DOF0
    int16_t torque_A1;  // Torque agonist DOF1
    int16_t torque_B1;  // Torque antagonist DOF1
  } frame2;
  
  frame2.torque_A0 = pid_diagnostics.torque_A[0];
  frame2.torque_B0 = pid_diagnostics.torque_B[0];
  frame2.torque_A1 = pid_diagnostics.torque_A[1];
  frame2.torque_B1 = pid_diagnostics.torque_B[1];
  
  uint32_t can_id_2 = CAN_ID_PID_TORQUE_DATA + ACTIVE_JOINT;
  CAN_HOST.sendMsgBuf(can_id_2, 0, sizeof(frame2), (uint8_t*)&frame2);

  // Frame 3 & 4 (OPTIONAL): PID term breakdown — only when enabled
  // Zero overhead when disabled (single bool check)
  if (pid_diag_terms_enabled && pid_diagnostics.pid_terms_valid) {
    // Frame 3: Inner PID terms (0x470 + joint)
    // Format: [P, I, D, FF] — agonist motor, DOF 0
    struct __attribute__((packed)) {
      int16_t p_term;
      int16_t i_term;
      int16_t d_term;
      int16_t ff_term;
    } frame3;
    frame3.p_term  = pid_diagnostics.inner_p_term;
    frame3.i_term  = pid_diagnostics.inner_i_term;
    frame3.d_term  = pid_diagnostics.inner_d_term;
    frame3.ff_term = pid_diagnostics.inner_ff_term;
    CAN_HOST.sendMsgBuf(CAN_ID_PID_INNER_TERMS + ACTIVE_JOINT, 0, sizeof(frame3), (uint8_t*)&frame3);

    // Frame 4: Outer PID terms (0x480 + joint)
    // Format: [P, I, D, output_x100] — joint PID, DOF 0
    struct __attribute__((packed)) {
      int16_t p_term;
      int16_t i_term;
      int16_t d_term;
      int16_t output_x100;
    } frame4;
    frame4.p_term      = pid_diagnostics.outer_p_term;
    frame4.i_term      = pid_diagnostics.outer_i_term;
    frame4.d_term      = pid_diagnostics.outer_d_term;
    frame4.output_x100 = pid_diagnostics.outer_output;
    CAN_HOST.sendMsgBuf(CAN_ID_PID_OUTER_TERMS + ACTIVE_JOINT, 0, sizeof(frame4), (uint8_t*)&frame4);
  }
}

/**
 * @brief Send movement metrics for a specific DOF via CAN
 * 
 * Called when metrics_ready[dof] is true (set when movement enters HOLDING).
 * Sends 2 CAN frames with complete movement performance data.
 * 
 * Frame format (0x440 + joint*3 + dof):
 * Frame 1: [rise_time, settle_time, overshoot, sse] (8 bytes)
 * Frame 2: [max_torque_A, max_torque_B, duration, flags, dof] (8 bytes)
 */
void sendMovementMetrics(uint8_t dof) {
  if (dof >= 3 || !metrics_ready[dof]) return;
  
  extern MCP_CAN CAN_HOST;
  MovementMetrics& m = last_movement_metrics[dof];
  
  // Frame 1: Timing and accuracy metrics
  struct __attribute__((packed)) {
    uint16_t rise_time_ms;     // Rise time in ms
    uint16_t settling_time_ms; // Settling time in ms
    int16_t overshoot_x100;    // Overshoot % × 100
    int16_t sse_x100;          // Steady-state error × 100
  } frame1;
  
  frame1.rise_time_ms = m.rise_time_ms;
  frame1.settling_time_ms = m.settling_time_ms;
  frame1.overshoot_x100 = m.overshoot_x100;
  frame1.sse_x100 = m.sse_x100;
  
  // CAN ID: 0x440 + joint*3 + dof (unique per joint per DOF)
  uint32_t can_id_1 = CAN_ID_MOVEMENT_METRICS + ACTIVE_JOINT * 3 + dof;
  CAN_HOST.sendMsgBuf(can_id_1, 0, sizeof(frame1), (uint8_t*)&frame1);
  
  // Frame 2: Torque and movement info (use next ID)
  struct __attribute__((packed)) {
    int16_t max_torque_A;        // Peak torque agonist
    int16_t max_torque_B;        // Peak torque antagonist
    uint16_t duration_ms;        // Total movement duration
    uint8_t dof_index;           // DOF index
    uint8_t flags;               // Status flags
  } frame2;
  
  frame2.max_torque_A = m.max_torque_A;
  frame2.max_torque_B = m.max_torque_B;
  frame2.duration_ms = m.movement_duration_ms;
  frame2.dof_index = m.dof_index;
  frame2.flags = m.flags;
  
  // Use ID + 0x10 offset for second frame
  uint32_t can_id_2 = CAN_ID_MOVEMENT_METRICS + 0x10 + ACTIVE_JOINT * 3 + dof;
  CAN_HOST.sendMsgBuf(can_id_2, 0, sizeof(frame2), (uint8_t*)&frame2);
  
  // Frame 3: Smoothness/oscillation metrics (new)
  struct __attribute__((packed)) {
    int16_t rms_error_x100;      // RMS error × 100
    int16_t jitter_x100;         // Jitter × 100
    uint8_t oscillation_count;   // Zero-crossings (capped at 255)
    uint8_t score_rms;           // RMS score (0-100)
    uint8_t score_jitter;        // Jitter score (0-100)
    uint8_t score_smoothness;    // Overall smoothness score (0-100)
  } frame3;
  
  frame3.rms_error_x100 = m.rms_error_x100;
  frame3.jitter_x100 = m.jitter_x100;
  frame3.oscillation_count = (m.oscillation_count > 255) ? 255 : (uint8_t)m.oscillation_count;
  frame3.score_rms = m.score_rms;
  frame3.score_jitter = m.score_jitter;
  frame3.score_smoothness = m.score_smoothness;
  
  // Use ID 0x460 + joint*3 + dof for third frame
  uint32_t can_id_3 = CAN_ID_SMOOTHNESS_METRICS + ACTIVE_JOINT * 3 + dof;
  CAN_HOST.sendMsgBuf(can_id_3, 0, sizeof(frame3), (uint8_t*)&frame3);
  
  // Clear the ready flag
  metrics_ready[dof] = false;
  
  LOG_C1_INFO("[CAN] Metrics sent for DOF " + String(dof) + ": rise=" + String(m.rise_time_ms) + 
           "ms, settle=" + String(m.settling_time_ms) + "ms, smooth=" + String(m.score_smoothness) + "/100");
}

/**
 * @brief Check and send any pending movement metrics
 * Called from core1_loop() to dispatch metrics when ready
 */
void checkAndSendMetrics() {
  for (uint8_t dof = 0; dof < 3; dof++) {
    if (metrics_ready[dof]) {
      sendMovementMetrics(dof);
      // NOTE: Offset drift check on HOLDING disabled — was causing firmware freeze.
      // TODO: investigate root cause and re-enable with proper safeguards.
    }
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

  // Check if Host CAN polling is suspended (e.g., during startup sequence)
  // This prevents SPI1 bus conflicts with Motor CAN operations on Core0
  if (suspend_host_can_polling) {
    return;
  }

  // Check if messages are available on Host CAN
  if (CAN_HOST.checkReceive() != CAN_MSGAVAIL) {
    return;
  }
  
  // Debug: log Host CAN activity (throttled to 30s to reduce spam)
  static uint32_t last_rx_log = 0;
  static uint32_t rx_count = 0;
  rx_count++;
  if (millis() - last_rx_log > 30000) {
    LOG_C1_DEBUG("[CAN_HOST] " + String(rx_count) + " messages received in 30s");
    rx_count = 0;
    last_rx_log = millis();
  }

  // Process up to 3 messages per poll to limit jitter on the control loop.
  // At 500Hz loop rate, 3 msgs/cycle = 1500 msgs/s throughput (well above
  // the ~300 msgs/s needed for 100Hz waypoints × 3 DOFs).
  // Remaining messages are picked up in the next cycle (2ms later).
  uint8_t msg_count = 0;
  while (CAN_HOST.checkReceive() == CAN_MSGAVAIL && msg_count < 3) {
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
      LOG_C1_WARN("[CAN_HOST] RX EMERGENCY_STOP frame");
      emergency_stop_requested = true;
    } else if (rx_id == CAN_ID_ENCODER_STREAM_CTRL) {
      // Encoder streaming control: byte 0 = 0x01 start, 0x00 stop
      if (len >= 1) {
        bool start = (buf[0] == 0x01);
        encoder_stream_can_active = start;
        if (start) {
          // Reset timer to ensure immediate first send
          encoder_stream_last_send_us = time_us_32() - ENCODER_STREAM_INTERVAL_US - 1;
          LOG_C1_INFO("[CAN_HOST] Encoder streaming STARTED @ 50Hz");
        } else {
          LOG_C1_INFO("[CAN_HOST] Encoder streaming STOPPED");
        }
      }
    } else if (rx_id == CAN_ID_PID_DIAG_CTRL) {
      // PID diagnostics streaming control:
      // byte 0 = 0x01 start / 0x00 stop
      // byte 1 = 0x01 enable P/I/D terms breakdown / 0x00 disable (optional)
      if (len >= 1) {
        bool start = (buf[0] == 0x01);
        pid_diag_stream_active = start;
        // Parse optional terms enable flag (byte 1)
        if (len >= 2) {
          pid_diag_terms_enabled = (buf[1] == 0x01);
        } else {
          pid_diag_terms_enabled = false;  // Default off if not specified
        }
        if (start) {
          uint32_t freq_hz = 1000000 / pid_diag_interval_us;
          LOG_C1_INFO("[CAN_HOST] PID diagnostics streaming STARTED @ " + String(freq_hz) + "Hz"
                      + (pid_diag_terms_enabled ? " (P/I/D terms ON)" : ""));
        } else {
          pid_diag_terms_enabled = false;  // Always disable terms when stopping
          LOG_C1_INFO("[CAN_HOST] PID diagnostics streaming STOPPED");
        }
      }
    } else if (rx_id == CAN_ID_INTERPOLATION_MODE) {
      // Interpolation mode control: byte 0 = mode (0=linear, 1=cosine)
      if (len >= 1) {
        uint8_t mode = buf[0];
        if (mode <= INTERPOLATION_COSINE) {
          waypoint_interpolation_mode = mode;
          const char* mode_name = (mode == INTERPOLATION_LINEAR) ? "LINEAR (step)" : "COSINE (smooth)";
          LOG_C1_INFO("[CAN_HOST] Interpolation mode set to: " + String(mode_name));
        } else {
          LOG_C1_WARN("[CAN_HOST] Invalid interpolation mode: " + String(mode));
        }
      }
    } else if (rx_id == CAN_ID_LOOP_FREQUENCY) {
      // Loop frequency control: 
      // byte 0-1: inner_loop_period_us (uint16_t, little-endian)
      // byte 2: outer_loop_divisor (uint8_t)
      if (len >= 3) {
        uint16_t new_inner_period = buf[0] | (buf[1] << 8);
        uint8_t new_outer_div = buf[2];
        
        // Validate: inner period 500-10000µs (100Hz-2000Hz), divisor 1-20
        if (new_inner_period >= 500 && new_inner_period <= 10000 && 
            new_outer_div >= 1 && new_outer_div <= 20) {
          inner_loop_period_us = new_inner_period;
          outer_loop_divisor = new_outer_div;
          
          float inner_freq = 1000000.0f / new_inner_period;
          float outer_freq = inner_freq / new_outer_div;
          LOG_C1_INFO("[CAN_HOST] Loop frequencies updated: inner=" + String(inner_freq, 1) + 
                   "Hz, outer=" + String(outer_freq, 1) + "Hz");
        } else {
          LOG_C1_WARN("[CAN_HOST] Invalid loop frequency: inner_period=" + String(new_inner_period) + 
                   "µs, outer_div=" + String(new_outer_div));
        }
      }
    } else if (rx_id == CAN_ID_PID_DIAG_FREQ) {
      // PID diagnostics frequency control:
      // byte 0: frequency in Hz (10-200 Hz)
      if (len >= 1) {
        uint8_t freq_hz = buf[0];
        
        // Validate: 10-200 Hz
        if (freq_hz >= 10 && freq_hz <= 200) {
          pid_diag_interval_us = 1000000 / freq_hz;
          LOG_C1_INFO("[CAN_HOST] PID diagnostics frequency set to: " + String(freq_hz) + 
                   "Hz (" + String(pid_diag_interval_us / 1000) + "ms)");
        } else {
          LOG_C1_WARN("[CAN_HOST] Invalid PID diag frequency: " + String(freq_hz) + 
                   "Hz (valid: 10-200Hz)");
        }
      }
    } else if (rx_id == CAN_ID_IDENTIFY_REQUEST) {
      // Joint identification broadcast request
      // All controllers start emitting EVT:JOINT on Serial + CAN announce periodically
      identify_broadcast_active = true;
      identify_broadcast_start_ms = millis();
      identify_broadcast_last_emit_ms = 0;  // Force immediate first emit
      identify_can_announce_last_ms = 0;     // Force immediate first CAN announce
      LOG_C1_INFO("[CAN_HOST] Joint identification broadcast STARTED (3s)");
    } else if (rx_id == CAN_ID_STARTUP_SEQUENCE) {
      // Startup sequence request via CAN (replaces serial CMD_STARTUP_SEQUENCE)
      // Frame: [joint_id, reserved, torque_lo, torque_hi, duration_lo, duration_hi, 0, 0]
      if (len >= 6) {
        uint8_t joint_id = buf[0];
        if (joint_id == ACTIVE_JOINT) {
          int16_t torque = (int16_t)(buf[2] | (buf[3] << 8));
          int16_t duration = (int16_t)(buf[4] | (buf[5] << 8));

          // Set flags for Core0 to execute (startup runs on Core0)
          can_startup_torque = torque;
          can_startup_duration = duration;
          can_startup_joint_id = joint_id;
          can_startup_requested = true;

          LOG_C1_INFO("[CAN_HOST] Startup sequence requested: torque=" + String(torque) +
                      " duration=" + String(duration));
        }
        // Silently ignore if not for this joint
      } else {
        LOG_C1_WARN("[CAN_HOST] Startup sequence frame too short: " + String(len) + " bytes");
      }
    } else if (rx_id == CAN_ID_GET_ENCODER_OFFSETS) {
      // Encoder offsets query: byte 0 = joint_id
      if (len >= 1 && buf[0] == ACTIVE_JOINT) {
        uint32_t resp_id = CAN_ID_ENCODER_OFFSETS_DATA + ACTIVE_JOINT;
        for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
          if (directEncoders.isEncoderConnected(i)) {
            float off = directEncoders.getOffset(i);
            uint8_t frame[8] = {0};
            frame[0] = (uint8_t)i;  // encoder index
            memcpy(&frame[4], &off, sizeof(float));  // offset as float32
            CAN_HOST.sendMsgBuf(resp_id, 0, 8, frame);
          }
        }
        LOG_C1_INFO("[CAN_HOST] Encoder offsets sent (" + String(DIRECT_ENCODER_COUNT) + " enc)");
      }
    } else if (rx_id == CAN_ID_SET_ZERO) {
      // Set-zero command: byte 0 = joint_id, byte 1 = dof_index
      if (len >= 2 && buf[0] == ACTIVE_JOINT) {
        can_set_zero_dof_index = buf[1];
        can_set_zero_requested = true;
        LOG_C1_INFO("[CAN_HOST] Set-zero requested: DOF=" + String(buf[1]));
      }
    } else if (rx_id == CAN_ID_PRETENSION) {
      // Pretension single DOF: [joint_id, dof_index, torque_lo, torque_hi, 0,0,0,0]
      if (len >= 2 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        uint8_t dof = buf[1];
        int16_t torque = (len >= 4) ? (int16_t)(buf[2] | (buf[3] << 8)) : 0;
        if (!safety_is_motor_power_enabled()) {
          safety_motor_power_enable();
        }
        bool invert = active_joint_controller->getConfig().dofs[dof].zero_mapping.auto_mapping_invert_direction;
        if (invert) {
          active_joint_controller->release(dof, torque, 0);
        } else {
          active_joint_controller->pretension(dof, torque, 0);
        }
        LOG_C1_INFO("[CAN_HOST] Pretension DOF=" + String(dof) + " torque=" + String(torque));
      }
    } else if (rx_id == CAN_ID_PRETENSION_ALL) {
      // Pretension all DOFs: [joint_id, 0,0,0,0,0,0,0]
      if (len >= 1 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        if (!safety_is_motor_power_enabled()) {
          safety_motor_power_enable();
        }
        active_joint_controller->pretensionAll();
        LOG_C1_INFO("[CAN_HOST] Pretension ALL");
      }
    } else if (rx_id == CAN_ID_RELEASE) {
      // Release single DOF: [joint_id, dof_index, 0,0,0,0,0,0]
      if (len >= 2 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        uint8_t dof = buf[1];
        int16_t torque = (len >= 4) ? (int16_t)(buf[2] | (buf[3] << 8)) : 0;
        bool invert = active_joint_controller->getConfig().dofs[dof].zero_mapping.auto_mapping_invert_direction;
        if (invert) {
          active_joint_controller->pretension(dof, torque, 0);
        } else {
          active_joint_controller->release(dof, torque, 0);
        }
        LOG_C1_INFO("[CAN_HOST] Release DOF=" + String(dof));
      }
    } else if (rx_id == CAN_ID_RELEASE_ALL) {
      // Release all DOFs: [joint_id, 0,0,0,0,0,0,0]
      if (len >= 1 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        active_joint_controller->releaseAll();
        LOG_C1_INFO("[CAN_HOST] Release ALL");
      }
    } else if (rx_id == CAN_ID_RECALC_OFFSET) {
      // Recalculate motor offsets: [joint_id, dof_index, torque_lo, torque_hi, duration_lo, duration_hi, 0, 0]
      if (len >= 2 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        uint8_t dof = buf[1];
        int16_t torque = (len >= 4) ? (int16_t)(buf[2] | (buf[3] << 8)) : 0;
        int16_t duration = (len >= 6) ? (int16_t)(buf[4] | (buf[5] << 8)) : 0;
        const JointConfig& cfg = active_joint_controller->getConfig();
        float actual_torque = (torque > 0) ? torque : cfg.dofs[dof].zero_mapping.recalc_offset_torque;
        int actual_duration = (duration > 0) ? duration : cfg.dofs[dof].zero_mapping.recalc_offset_duration;
        LOG_C1_INFO("[CAN_HOST] Recalc offset DOF=" + String(dof) +
                    " torque=" + String(actual_torque) + " dur=" + String(actual_duration));
        active_joint_controller->recalculateMotorOffsets(dof, actual_torque, actual_duration);
      }
    } else if (rx_id == CAN_ID_SAVE_PID) {
      // Save PID to flash: [joint_id, 0,0,0,0,0,0,0] — Core0 handler (flash access)
      if (len >= 1 && buf[0] == ACTIVE_JOINT) {
        can_save_pid_requested = true;
        LOG_C1_INFO("[CAN_HOST] Save PID to flash requested");
      }
    } else if (rx_id == CAN_ID_LOAD_PID) {
      // Load PID from flash: [joint_id, 0,0,0,0,0,0,0] — Core0 handler (flash access)
      if (len >= 1 && buf[0] == ACTIVE_JOINT) {
        can_load_pid_requested = true;
        LOG_C1_INFO("[CAN_HOST] Load PID from flash requested");
      }
    } else if (rx_id == CAN_ID_SET_PID) {
      // Set inner PID: multi-frame accumulator
      // Frame: [joint_id, dof, motor, seq, float32_value(4)]
      // seq: 0=Kp, 1=Ki, 2=Kd, 3=Tau (triggers setPid)
      if (len >= 8 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        static uint8_t spid_dof = 0, spid_motor = 0;
        static float spid_kp = 0, spid_ki = 0, spid_kd = 0;
        uint8_t dof = buf[1];
        uint8_t motor = buf[2];
        uint8_t seq = buf[3];
        float val;
        memcpy(&val, &buf[4], sizeof(float));
        switch (seq) {
          case 0: spid_dof = dof; spid_motor = motor; spid_kp = val; break;
          case 1: spid_ki = val; break;
          case 2: spid_kd = val; break;
          case 3: {
            float tau = val;
            if (active_joint_controller->setPid(spid_dof, spid_motor, spid_kp, spid_ki, spid_kd, tau)) {
              LOG_C1_INFO("[CAN_HOST] SET_PID OK: DOF=" + String(spid_dof) + " motor=" + String(spid_motor) +
                          " Kp=" + String(spid_kp, 4) + " Ki=" + String(spid_ki, 4) +
                          " Kd=" + String(spid_kd, 4) + " Tau=" + String(tau, 4));
            } else {
              LOG_C1_WARN("[CAN_HOST] SET_PID FAILED: DOF=" + String(spid_dof) + " motor=" + String(spid_motor));
            }
            break;
          }
        }
      }
    } else if (rx_id == CAN_ID_SET_PID_OUTER) {
      // Set outer PID: multi-frame accumulator
      // Frame: [joint_id, dof, seq, 0, float32_value(4)]
      // seq: 0=Kp, 1=Ki, 2=Kd, 3=stiffness, 4=influence (triggers setOuterLoopParameters)
      if (len >= 8 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        static uint8_t opid_dof = 0;
        static float opid_kp = 0, opid_ki = 0, opid_kd = 0, opid_stiffness = 0;
        uint8_t dof = buf[1];
        uint8_t seq = buf[2];
        float val;
        memcpy(&val, &buf[4], sizeof(float));
        switch (seq) {
          case 0: opid_dof = dof; opid_kp = val; break;
          case 1: opid_ki = val; break;
          case 2: opid_kd = val; break;
          case 3: opid_stiffness = val; break;
          case 4: {
            float influence = val;
            if (active_joint_controller->setOuterLoopParameters(opid_dof, opid_kp, opid_ki, opid_kd,
                                                                 opid_stiffness, influence)) {
              LOG_C1_INFO("[CAN_HOST] SET_PID_OUTER OK: DOF=" + String(opid_dof) +
                          " Kp=" + String(opid_kp, 4) + " Ki=" + String(opid_ki, 4) +
                          " Kd=" + String(opid_kd, 4) + " stiff=" + String(opid_stiffness, 2) +
                          " infl=" + String(influence, 2));
            } else {
              LOG_C1_WARN("[CAN_HOST] SET_PID_OUTER FAILED: DOF=" + String(opid_dof));
            }
            break;
          }
        }
      }
    } else if (rx_id == CAN_ID_CASCADE_SPEED_SCALING) {
      // Cascade speed scaling: per-parameter frames
      // Frame: [joint_id, param_id, 0, 0, float32_value(4)]
      // param_id: 0=enabled, 1=min, 2=low, 3=high, 4=ema_en, 5=ema_alpha,
      //           6=tau_en, 7=tau_high, 8=tau_speed, 9=jema_en, 10=jema_alpha,
      //           11=jema_speed, 12=fric_en, 13=fric_torque, 14=fric_speed, 0xFF=apply (log)
      if (len >= 6 && buf[0] == ACTIVE_JOINT) {
        uint8_t param_id = buf[1];
        float val;
        memcpy(&val, &buf[2], sizeof(float));
        switch (param_id) {
          case 0:  cascade_speed_scaling_enabled = (val != 0.0f); break;
          case 1:  if (val >= 0.0f && val <= 1.0f) cascade_min_factor = val; break;
          case 2:  if (val >= 0.0f && val <= 100.0f) cascade_speed_low = val; break;
          case 3:  if (val >= 0.0f && val <= 100.0f) cascade_speed_high = val; break;
          case 4:  motor_ema_enabled = (val != 0.0f); break;
          case 5:  if (val >= 0.05f && val <= 1.0f) motor_ema_alpha = val; break;
          case 6:  inner_tau_scaling_enabled = (val != 0.0f); break;
          case 7:  if (val >= 0.005f && val <= 0.2f) inner_tau_high = val; break;
          case 8:  if (val >= 0.1f && val <= 100.0f) inner_tau_speed_threshold = val; break;
          case 9:  joint_ema_enabled = (val != 0.0f); break;
          case 10: if (val >= 0.05f && val <= 1.0f) joint_ema_alpha = val; break;
          case 11: if (val >= 0.1f && val <= 100.0f) joint_ema_speed_threshold = val; break;
          case 12: friction_ff_enabled = (val != 0.0f); break;
          case 13: if (val >= 0.0f && val <= 100.0f) friction_ff_torque = val; break;
          case 14: if (val >= 0.1f && val <= 50.0f) friction_ff_speed_thresh = val; break;
          case 0xFF:
            LOG_C1_INFO("[CAN_HOST] CASCADE_SPEED_SCALING applied: en=" +
                        String(cascade_speed_scaling_enabled) + " min=" + String(cascade_min_factor, 2) +
                        " low=" + String(cascade_speed_low, 1) + " high=" + String(cascade_speed_high, 1));
            break;
        }
      }
    } else if (rx_id == CAN_ID_START_AUTO_MAPPING) {
      // Start auto-mapping: [joint_id, reserved, 0,0,0,0,0,0]
      // Maps ALL DOFs simultaneously (Cartesian product). Byte 1 reserved for future use.
      // Uses firmware config defaults (torque=0, steps=nullptr, settle=0 → config values)
      if (len >= 1 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        LOG_C1_INFO("[CAN_HOST] START_AUTO_MAPPING (all DOFs)");
        if (active_joint_controller->startAutoMapping(auto_mapping_state, 0, nullptr, 0)) {
          LOG_C1_INFO("[CAN_HOST] Auto-mapping started successfully");
        } else {
          LOG_C1_ERROR("[CAN_HOST] Failed to start auto-mapping");
        }
      }
    } else if (rx_id == CAN_ID_STOP_AUTO_MAPPING) {
      // Stop auto-mapping: [joint_id, 0,0,0,0,0,0,0]
      if (len >= 1 && buf[0] == ACTIVE_JOINT && active_joint_controller != nullptr) {
        LOG_C1_INFO("[CAN_HOST] STOP_AUTO_MAPPING");
        if (active_joint_controller->stopAutoMapping(auto_mapping_state)) {
          LOG_C1_INFO("[CAN_HOST] Auto-mapping stopped");
        } else {
          LOG_C1_ERROR("[CAN_HOST] Failed to stop auto-mapping");
        }
      }
    } else if (rx_id == CAN_ID_SAVE_LINEAR_EQ) {
      // Save linear equations to flash: [joint_id, 0,0,0,0,0,0,0]
      // Core0 executes flash write via volatile flag
      if (len >= 1 && buf[0] == ACTIVE_JOINT) {
        LOG_C1_INFO("[CAN_HOST] SAVE_LINEAR_EQ requested");
        can_save_linear_eq_requested = true;
      }
    } else if (rx_id == CAN_ID_LOAD_LINEAR_EQ) {
      // Load linear equations from flash: [joint_id, 0,0,0,0,0,0,0]
      // Core0 executes flash read via volatile flag
      if (len >= 1 && buf[0] == ACTIVE_JOINT) {
        LOG_C1_INFO("[CAN_HOST] LOAD_LINEAR_EQ requested");
        can_load_linear_eq_requested = true;
      }
    } else if (rx_id == CAN_ID_SET_AUTO_START) {
      // Set auto-start: [joint_id, enabled, torque_lo, torque_hi, dur_lo, dur_hi, 0, 0]
      // Core0 executes flash save via volatile flag + params
      if (len >= 2 && buf[0] == ACTIVE_JOINT) {
        can_auto_start_enabled = buf[1];
        if (len >= 6) {
          int16_t torque;
          uint16_t duration;
          memcpy(&torque, &buf[2], sizeof(int16_t));
          memcpy(&duration, &buf[4], sizeof(uint16_t));
          can_auto_start_torque = torque;
          can_auto_start_duration = duration;
        } else {
          can_auto_start_torque = 0;
          can_auto_start_duration = 0;
        }
        LOG_C1_INFO("[CAN_HOST] SET_AUTO_START en=" + String(can_auto_start_enabled));
        can_set_auto_start_requested = true;
      }
    } else if (rx_id == CAN_ID_WP_REANCHOR_INTERVAL) {
      // Re-anchor interval: byte 0-1 = uint16_t interval
      // 0 = disabled, otherwise clamp to [1, WAYPOINT_BUFFER_DEPTH].
      if (len >= 2) {
        uint16_t requested_interval = buf[0] | (buf[1] << 8);
        uint16_t applied_interval = requested_interval;

        if (requested_interval > WAYPOINT_BUFFER_DEPTH) {
          applied_interval = WAYPOINT_BUFFER_DEPTH;
          LOG_C1_WARN("[CAN_HOST] WP re-anchor interval clamped: " +
                      String(requested_interval) + " -> " +
                      String((uint16_t)WAYPOINT_BUFFER_DEPTH));
        }

        wp_reanchor_interval = applied_interval;
        LOG_C1_INFO("[CAN_HOST] WP re-anchor interval set to: " + String(applied_interval) +
                    (applied_interval == 0 ? " (disabled)" : " WPs"));
      }
    } else if (rx_id >= CAN_ID_MULTI_DOF_WAYPOINT_BASE && rx_id < CAN_ID_STATUS_BASE) {
      // Multi-DOF Waypoint (0x380-0x39F) - all DOFs in one frame
      handleMultiDofWaypointFrame(rx_id, buf, len);
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
 * - Movement: via CAN waypoints (executeWaypointMovement)
 * - CMD_RECALC_OFFSET: Recalculate motor offset calibration
 * - CMD_START_AUTO_MAPPING: Start automatic joint calibration
 * - CMD_STOP_AUTO_MAPPING: Stop automatic joint calibration
 */
void core1_loop() {
  // NOTE: multicore_lockout_victim_init() was removed because it interferes
  // with core1 startup. Flash operations now use a simpler approach:
  // Core0 waits for Core1 to be in a safe state before flash write.
  
  // Timing for waypoint control (configurable via inner_loop_period_us)
  // Default: 2000µs = 500Hz
  uint64_t next_time = 0; // Will be initialized on first waypoint
  bool timing_initialized = false;

  while (true) {
    // === CHECK FOR FLASH OPERATION ===
    // If Core0 is performing a flash operation, wait in RAM until complete
    // This prevents Core1 from crashing when XIP is disabled during flash erase/program
    if (flash_operation_in_progress) {
      wait_for_flash_complete();
    }
    
    // === POLL HOST CAN BUS ===
    // Poll dedicated Host CAN (J5) for TimeSync, Waypoints, Emergency Stop
    // Motor CAN (J4) is handled by LKM_Motor during motor operations
    pollHostCan();

    // === EMERGENCY STOP CHECK (immediately after CAN poll) ===
    // Must run BEFORE any motor commands to ensure zero-delay stop
    if (emergency_stop_requested) {
      LOG_C1_INFO("Core1: Emergency stop requested");

      // Cut motor power at hardware level (Rev B: <10µs via MOSFET gate)
      safety_motor_power_disable();

      // Stop all motors via CAN (software stop — belt-and-suspenders with HW cutoff)
      if (active_joint_controller != nullptr) {
        active_joint_controller->stopAllMotors();
        LOG_C1_INFO("Core1: All motors stopped");
        
        // Clear all waypoint buffers to exit waypoint control loop
        for (uint8_t dof = 0; dof < active_joint_controller->getConfig().dof_count; dof++) {
          waypoint_buffer_clear(dof);
          waypoint_buffer_set_state(dof, WaypointState::IDLE);
        }
        LOG_C1_INFO("Core1: Waypoint buffers cleared");
      }

      // Reset flag
      emergency_stop_requested = false;

      LOG_C1_INFO("Core1: Emergency stop flag cleared");

      // Notify core0
      if (shared_data_ext.flag == 0) {
        shared_data_ext.flag = CMD1_END_MOVE;
        strcpy(shared_data_ext.message, "EMERGENCY STOP EXECUTED");
      }

      SERIAL_C1_COM_LN("EMERGENCY STOP EXECUTED");
      continue;
    }

    // === ENCODER STREAMING VIA CAN ===
    // Send encoder data at 200Hz if streaming is active
    // This reads from shared_dof_angles (updated by Core0)
    sendEncoderStreamData();
    
    // === PID DIAGNOSTICS STREAMING VIA CAN ===
    // Send PID target/error/torque data at 20Hz for tuning
    sendPIDDiagStreamData();

    // === ENCODER OFFSET NOTIFICATION VIA CAN ===
    // Core0 sets this flag after zero (saveOffsetsToFlash) or boot (loadOffsetsFromFlash)
    if (can_encoder_offsets_notify) {
      can_encoder_offsets_notify = false;

      uint32_t resp_id = CAN_ID_ENCODER_OFFSETS_DATA + ACTIVE_JOINT;
      uint8_t enc_sent = 0;
      for (int i = 0; i < DIRECT_ENCODER_COUNT; i++) {
        if (directEncoders.isEncoderConnected(i)) {
          float off = directEncoders.getOffset(i);
          uint8_t frame[8] = {0};
          frame[0] = (uint8_t)i;  // encoder index
          memcpy(&frame[4], &off, sizeof(float));
          CAN_HOST.sendMsgBuf(resp_id, 0, 8, frame);
          enc_sent++;
        }
      }

      // Send zero-complete summary frame on 0x4C0 + joint_id
      uint32_t summary_id = CAN_ID_ZERO_COMPLETE + ACTIVE_JOINT;
      uint8_t summary[8] = {0xFF, 0, 0, enc_sent, 0, 0, 0, 0};
      CAN_HOST.sendMsgBuf(summary_id, 0, 8, summary);

      LOG_C1_INFO("[CAN_HOST] Encoder offsets notified via CAN (" + String(enc_sent) + " enc)");
    }

    // === MOVEMENT METRICS VIA CAN ===
    // DISABLED: CAN_HOST.sendMsgBuf on SPI1 conflicts with Motor CAN reads
    // in the same Core1 cycle, causing SPI1 deadlock during HOLDING.
    // TODO: serialize Host CAN sends to avoid SPI1 bus conflict with Motor CAN.
    // checkAndSendMetrics();

    // === CAN JOINT ANNOUNCE (Discovery) ===
    // When identify broadcast is active, send announce frames on CAN HOST
    // Core1 handles all CAN HOST I/O to avoid SPI1 conflicts with Core0
    if (identify_broadcast_active) {
      uint32_t now_ms = millis();
      if (now_ms - identify_can_announce_last_ms >= IDENTIFY_BROADCAST_INTERVAL_MS) {
        identify_can_announce_last_ms = now_ms;

        uint8_t announce[8] = {0};
        announce[0] = ACTIVE_JOINT;                    // joint_id
        announce[1] = ACTIVE_JOINT_CONFIG.dof_count;   // DOF count
        announce[2] = ACTIVE_JOINT_CONFIG.motor_count; // Motor count
        announce[3] = (active_joint_controller != nullptr &&
                       active_joint_controller->isSystemReadyForMovement()) ? 0x01 : 0x00;
        // FW_VERSION is "X.Y.Z" string; extract major.minor.patch
        announce[4] = (uint8_t)(FW_VERSION[0] - '0');
        announce[5] = (uint8_t)(FW_VERSION[2] - '0');
        announce[6] = (uint8_t)(FW_VERSION[4] - '0');
        announce[7] = clock_synced ? 0x01 : 0x00;

        CAN_HOST.sendMsgBuf(CAN_ID_JOINT_ANNOUNCE + ACTIVE_JOINT, 0, 8, announce);
      }
    }

    // === STARTUP STATUS EVENTS VIA CAN ===
    // Drain startup event queue (produced by Core0) and send as CAN frames
    {
      StartupStatusEvent evt;
      while (queue_try_remove(&startup_event_queue, &evt)) {
        uint8_t frame[8] = {0};
        frame[0] = evt.event_type;
        frame[1] = evt.dof_index;
        frame[2] = evt.reason_code;
        frame[3] = (uint8_t)(evt.elapsed_ms & 0xFF);
        frame[4] = (uint8_t)((evt.elapsed_ms >> 8) & 0xFF);

        CAN_HOST.sendMsgBuf(CAN_ID_STARTUP_STATUS + ACTIVE_JOINT, 0, 8, frame);
      }
    }

    // === WAYPOINT-BASED MOVEMENT ===
    // Execute waypoint trajectory for all DOFs (if waypoints available)
    // This runs @ 500 Hz with precise timing (outer loop every outer_loop_divisor cycles)
    bool waypoint_active = false;
    if (active_joint_controller != nullptr && safety_is_motor_power_enabled()) {
      waypoint_active = active_joint_controller->executeWaypointMovement();
    }

    // Signal Core0 to suspend Serial streaming during active MOVING only.
    // HOLDING is safe for serial — Core1 doesn't log when buffer is empty.
    movement_in_progress = waypoint_active;

    // === TIMING: Wait for next cycle (configurable frequency) ===
    // PID needs 500Hz both in MOVING and HOLDING (gains tuned for 2ms period).
    // waypoint_active covers MOVING; check HOLDING (non-IDLE) separately.
    bool pid_timing_needed = waypoint_active;
    if (!pid_timing_needed && active_joint_controller != nullptr) {
      uint8_t dof_count = active_joint_controller->getConfig().dof_count;
      for (uint8_t d = 0; d < dof_count; d++) {
        if (waypoint_buffer_state(d) != WaypointState::IDLE) {
          pid_timing_needed = true;
          break;
        }
      }
    }

    if (pid_timing_needed) {
      if (!timing_initialized) {
        next_time = time_us_64() + inner_loop_period_us;
        timing_initialized = true;
      }

      busy_wait_until(next_time);
      next_time += inner_loop_period_us;
    } else {
      timing_initialized = false;
    }

    // === WATCHDOG KICK ===
    // Must run every iteration to prove the control loop is alive.
    // Rev B: kicks external MAX6369 (rate-limited internally to ~200ms)
    // If this stops: external WDT cuts motor power after ~1s, internal WDT resets MCU.
    safety_watchdog_kick();

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
        // NOTE: No Serial/LOG calls here to avoid conflicts with Core0
        
        // Transfer acquired data to DofMappingData_t structures
        if (active_joint_controller->transferAutoMappingData(auto_mapping_state)) {
          // Compute linear equations from the raw mapping data just transferred
          // (This function sets _pending_flash_save flag for Core0 to handle)
          active_joint_controller->calculateLinearEquationsFromMappingData();
        }

        // Signal completion - Core0 will handle logging and flash save
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "Auto mapping completed successfully");
          shared_data_ext.flag = CMD1_AUTO_MAP_COMPLETE;
        }
        break;

      case 3: // Error
        // NOTE: No Serial/LOG calls here to avoid conflicts with Core0
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

    // Reset buffer flag IMMEDIATELY to free buffer for next command
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
        LOG_C1_ERROR("Command targeted to a different joint: " + String(joint_id));
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

    // Guard: reject motor-moving commands when power gate is disabled (after e-stop)
    // Only PRETENSION can re-enable power. STOP, zero-setting and diagnostics always allowed.
    if (!safety_is_motor_power_enabled() &&
        command != CMD_STOP &&
        command != CMD_PRETENSION &&
        command != CMD_PRETENSION_ALL &&
        command != CMD_SET_ZERO_CURRENT_POS &&
        command != CMD_ZERO_MOTOR_ENCODERS &&
        command != CMD_STOP_AUTO_MAPPING &&
        command != CMD_CAN_DIAG &&
        command != CMD_CHECK_OFFSETS) {
      if (shared_data_ext.flag == 0) {
        shared_data_ext.flag = CMD1_FAIL_MOVE;
        strcpy(shared_data_ext.message, "ERROR: Motor power disabled. Send PRETENSION to recover.");
      }
      LOG_C1_WARN("Command rejected: motor power disabled after emergency stop");
      continue;
    }

    // Execute appropriate command - only commands requiring hardware access
    switch (command) {
    case CMD_STOP:
      // Stop all motors of the controller
      controller->stopAllMotors();
      break;

    case CMD_PRETENSION: {
      // Re-enable motor power if it was cut by emergency stop (Rev B HW gate)
      if (!safety_is_motor_power_enabled()) {
        safety_motor_power_enable();
      }
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
      // Re-enable motor power if it was cut by emergency stop (Rev B HW gate)
      if (!safety_is_motor_power_enabled()) {
        safety_motor_power_enable();
      }
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
      // Legacy: Now handled by Core0, but keep for compatibility
      if (controller->setZeroCurrentPos(dof_index)) {
        shared_data_ext.dof_index = dof_index;
        shared_data_ext.flag      = CMD1_END_ZERO;
        strcpy(shared_data_ext.message, "Current position set as zero for DOF");
      } else {
        if (shared_data_ext.flag == 0) {
          strcpy(shared_data_ext.message, "ERROR: Failed to set zero position");
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
      }
      break;

    case CMD_ZERO_MOTOR_ENCODERS:
      // Zero motor encoder offsets for a specific DOF (delegated from Core0)
      // This is called AFTER Core0 has reset the joint encoder
      for (int m = 0; m < controller->getConfig().motor_count; m++) {
        if (controller->getConfig().motors[m].dof_index == dof_index) {
          LKM_Motor* motor = controller->getMotor(m);
          if (motor != nullptr) {
            motor->zeroEncoderOffset();
          }
        }
      }
      LOG_C1_DEBUG_F("Motor encoders zeroed for DOF %d", dof_index);
      break;

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
        // Motors intentionally NOT stopped — pretension torque maintained to prevent
        // gravity-induced movement gap. Core0 injects HOLDING waypoints immediately
        // after all DOFs complete, and PID bumpless transfer takes over seamlessly.
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

    case CMD_APPLY_SAVED_OFFSETS: {
      // Validate saved offsets against current motor positions, apply if valid
      LOG_C1_INFO("=== SMART STARTUP: DOF " + String(dof_index) + " ===");
      JointController::OffsetValidationResult vr = controller->validateSavedOffsets(dof_index);
      // validateSavedOffsets already logs: VALID/NEEDS RECALC + error values

      if (vr.valid) {
        // Offsets still valid — apply from flash without full recalc
        if (controller->applySavedOffsetsToMotors(dof_index)) {
          // applySavedOffsetsToMotors logs: offsets applied + post-apply verification
          if (shared_data_ext.flag == 0) {
            snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                     "Saved offsets applied (err: %.1f/%.1f deg)",
                     vr.error_agonist_deg, vr.error_antagonist_deg);
            shared_data_ext.flag = CMD1_END_MOVE;
          }
        } else {
          LOG_C1_ERROR("applySavedOffsetsToMotors failed for DOF " + String(dof_index));
          if (shared_data_ext.flag == 0) {
            strcpy(shared_data_ext.message, "Failed to apply saved offsets");
            shared_data_ext.flag = CMD1_FAIL_MOVE;
          }
        }
      } else {
        // Offsets invalid or no data — signal caller to fall back to full recalc
        LOG_C1_INFO("DOF " + String(dof_index) + " requires full recalc: " +
                 String(vr.has_saved_data ? "offset drift detected" : "no saved data"));
        if (shared_data_ext.flag == 0) {
          snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                   "OFFSETS_INVALID: %s (err: %.1f/%.1f deg)",
                   vr.has_saved_data ? "drift" : "no_data",
                   vr.error_agonist_deg, vr.error_antagonist_deg);
          shared_data_ext.flag = CMD1_FAIL_MOVE;
        }
      }
    } break;

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
      LOG_C1_INFO("=== CAN BUS DIAGNOSTIC TEST ===");
      
      bool all_ok = true;
      int motors_responding = 0;
      int motors_failed = 0;
      int send_errors = 0;
      bool loopback_ok = false;
      
      // Test 1: MCP2515 SPI communication
      LOG_C1_INFO("[DIAG] Step 1: MCP2515 SPI Check");
      LOG_C1_INFO("[DIAG] Motor CAN (J4): CS=GP9, INT=GP13");
      LOG_C1_INFO("[DIAG] Host CAN (J5): DISABLED for debugging");
      
      // Check CS pin state
      LOG_C1_INFO("[DIAG] CS pin GP9 state: " + String(digitalRead(9)));
      
      // Test 1b: MCP2515 Loopback Test (internal, no bus needed)
      LOG_C1_INFO("[DIAG] Step 1b: MCP2515 LOOPBACK Test");
      {
        const unsigned char testData[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};
        loopback_ok = can_loopback_test(CAN, "[DIAG] Motor CAN", 0x7FF, testData);
        if (!loopback_ok) all_ok = false;
        delay(20);
      }
      
      // Test 2: Raw CAN TX test (requires bus connection)
      LOG_C1_INFO("[DIAG] Step 2: CAN Bus TX Test (Motor ID 1)");
      {
        unsigned char testCmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};  // READ_STATUS
        unsigned long testId = 0x141;  // Motor ID 1
        
        LOG_C1_INFO("[DIAG] Sending to CAN ID 0x" + String(testId, HEX) + "...");
        
        byte sendResult = CAN.sendMsgBuf(testId, 0, 8, testCmd);
        if (sendResult == CAN_OK) {
          LOG_C1_INFO("[DIAG]   ✓ CAN TX OK - ACK received from bus");
        } else if (sendResult == CAN_SENDMSGTIMEOUT) {
          LOG_C1_ERROR("[DIAG]   ✗ CAN TX TIMEOUT (code " + String(sendResult) + ")");
          LOG_C1_ERROR("[DIAG]     NO ACK = no device responding on CAN bus");
          send_errors++;
          all_ok = false;
        } else if (sendResult == CAN_GETTXBFTIMEOUT) {
          LOG_C1_ERROR("[DIAG]   ✗ CAN TX BUFFER TIMEOUT (code " + String(sendResult) + ")");
          LOG_C1_ERROR("[DIAG]     MCP2515 TX buffer busy - SPI issue?");
          send_errors++;
          all_ok = false;
        } else {
          LOG_C1_ERROR("[DIAG]   ✗ CAN TX ERROR code: " + String(sendResult));
          send_errors++;
          all_ok = false;
        }
        delay(20);
      }
      
      // Test 3: Try second motor
      LOG_C1_INFO("[DIAG] Step 3: CAN Bus TX Test (Motor ID 2)");
      {
        unsigned char testCmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
        unsigned long testId = 0x142;  // Motor ID 2
        
        LOG_C1_INFO("[DIAG] Sending to CAN ID 0x" + String(testId, HEX) + "...");
        
        byte sendResult = CAN.sendMsgBuf(testId, 0, 8, testCmd);
        if (sendResult == CAN_OK) {
          LOG_C1_INFO("[DIAG]   ✓ CAN TX OK");
        } else {
          LOG_C1_ERROR("[DIAG]   ✗ CAN TX ERROR code: " + String(sendResult));
          send_errors++;
        }
        delay(20);
      }
      
      // Test 4: Motor angle read via LKM_Motor
      LOG_C1_INFO("[DIAG] Step 4: Motor Communication Test");
      
      if (controller != nullptr) {
        const JointConfig& cfg = controller->getConfig();
        LOG_C1_INFO("[DIAG] Testing " + String(cfg.motor_count) + " motors for: " + String(cfg.name));
        
        for (uint8_t m = 0; m < cfg.motor_count; m++) {
          uint8_t motor_id = cfg.motors[m].id;
          String motor_name = String(cfg.motors[m].name);
          
          LOG_C1_INFO("[DIAG] Motor " + String(m) + " (ID=" + String(motor_id) + ", " + motor_name + ")");
          
          LKM_Motor* motor = controller->getMotor(m);
          if (motor != nullptr) {
            LKM_Motor::MultiAngleData data = motor->getMultiAngleSync(false);
            
            if (data.waitTime > 0) {
              LOG_C1_INFO("[DIAG]   ✓ Response in " + String(data.waitTime) + "µs: " + 
                       String(data.angle, 2) + "°");
              motors_responding++;
            } else {
              LOG_C1_ERROR("[DIAG]   ✗ NO RESPONSE (waitTime=0)");
              motors_failed++;
              all_ok = false;
            }
          } else {
            LOG_C1_ERROR("[DIAG]   ✗ Motor object NULL");
            motors_failed++;
            all_ok = false;
          }
          delay(50);
        }
      }
      
      // Summary
      LOG_C1_INFO("[DIAG] === SUMMARY ===");
      LOG_C1_INFO("[DIAG] Loopback test: " + String(loopback_ok ? "PASS" : "FAIL"));
      LOG_C1_INFO("[DIAG] TX errors: " + String(send_errors));
      LOG_C1_INFO("[DIAG] Motors: " + String(motors_responding) + "/" + 
               String(motors_responding + motors_failed) + " responding");
      
      if (all_ok && motors_responding > 0) {
        LOG_C1_INFO("[DIAG] ✓ CAN TESTS PASSED");
        snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                 "CAN OK: %d motors", motors_responding);
        shared_data_ext.flag = CMD1_END_MOVE;
      } else {
        LOG_C1_ERROR("[DIAG] ✗ CAN TESTS FAILED");
        if (!loopback_ok) {
          LOG_C1_ERROR("[DIAG] → LOOPBACK FAILED = SPI or MCP2515 issue");
          LOG_C1_ERROR("[DIAG]   Check SPI wiring: SCK(GP10), MOSI(GP11), MISO(GP12), CS(GP9)");
        } else if (send_errors > 0) {
          LOG_C1_ERROR("[DIAG] → LOOPBACK OK but TX FAILED = Transceiver or bus issue");
          LOG_C1_ERROR("[DIAG]   1. CAN transceiver powered? (5V or 3.3V)");
          LOG_C1_ERROR("[DIAG]   2. TXCAN/RXCAN pins connected?");
          LOG_C1_ERROR("[DIAG]   3. CAN_H/CAN_L to bus?");
          LOG_C1_ERROR("[DIAG]   4. Termination 120Ω present?");
          LOG_C1_ERROR("[DIAG]   5. Other device on bus powered?");
        } else {
          LOG_C1_ERROR("[DIAG] → TX OK but motors not responding");
          LOG_C1_ERROR("[DIAG]   1. Motor IDs correct?");
          LOG_C1_ERROR("[DIAG]   2. Motors in error state?");
        }
        snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                 "CAN FAIL: LB=%s TX_ERR=%d M=%d/%d", 
                 loopback_ok ? "OK" : "FAIL", send_errors, 
                 motors_responding, motors_responding + motors_failed);
        shared_data_ext.flag = CMD1_FAIL_MOVE;
      }
      
      LOG_C1_INFO("=== END CAN DIAGNOSTIC ===");
    } break;

    case CMD_CHECK_OFFSETS: {
      // Validate saved motor offsets against current motor positions
      // Used by UI to determine if recalc_offset is needed before startup
      bool all_valid = true;
      bool has_any_data = false;
      char detail_buf[80];

      for (uint8_t dof = 0; dof < controller->getConfig().dof_count; dof++) {
        JointController::OffsetValidationResult vr = controller->validateSavedOffsets(dof);

        // Emit per-DOF status as EVT line (parsed by host)
        const char* status_str = !vr.has_saved_data ? "NO_DATA" : (vr.valid ? "VALID" : "NEEDED");
        if (vr.has_saved_data) has_any_data = true;
        if (!vr.valid) all_valid = false;

        snprintf(detail_buf, sizeof(detail_buf),
                 "EVT:RECALC_STATUS(%d,%d,%s,%.2f,%.2f)",
                 ACTIVE_JOINT, dof, status_str,
                 vr.error_agonist_deg, vr.error_antagonist_deg);
        SERIAL_C1_COM_LN(detail_buf);
      }

      // Emit firmware safe limits per DOF (from linear equations, if available)
      for (uint8_t dof = 0; dof < controller->getConfig().dof_count; dof++) {
        DofLinearEquations *eq = controller->getLinearEquations(dof);
        if (eq != nullptr && eq->calculated && eq->limits_valid) {
          snprintf(detail_buf, sizeof(detail_buf),
                   "EVT:SAFE_LIMITS(%d,%d,%.2f,%.2f)",
                   ACTIVE_JOINT, dof, eq->joint_safe_min, eq->joint_safe_max);
          SERIAL_C1_COM_LN(detail_buf);
        }
      }

      // Summary result
      const char* summary = !has_any_data ? "NO_DATA" : (all_valid ? "ALL_VALID" : "RECALC_NEEDED");
      if (shared_data_ext.flag == 0) {
        snprintf(shared_data_ext.message, sizeof(shared_data_ext.message),
                 "CHECK_OFFSETS:%s", summary);
        shared_data_ext.flag = CMD1_END_MOVE;
      }
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
