/**
 * @file main_common.h
 * @brief Common definitions, includes, and global variables shared between
 *        main.cpp, core0.cpp, and core1.cpp
 * 
 * This file contains:
 * - All library includes
 * - Active joint configuration
 * - Global variables for hardware (CAN, encoders)
 * - Shared data structures for inter-core communication
 * - Function declarations for helper functions
 * 
 * Structure:
 * - main.cpp: Entry point + setup()
 * - core0.cpp: loop() - Serial communication, command parsing
 * - core1.cpp: core1_loop() - Movement execution, hardware control
 */

#ifndef MAIN_COMMON_H
#define MAIN_COMMON_H

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================

#include <DirectEncoders.h>
#include <LKM_Motor.h>
#include "pico/multicore.h"
#include "version.h"
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// Multi-joint support includes (before legacy system)
#include <CommandParser.h>
#include <JointConfig.h>
#include <JointController.h>
#include <commands.h> // Before global.h to avoid conflicts
#include <config_presets.h>
#include <shared_data.h>
#include <waypoint_buffer.h>

// Legacy support includes
#include <PID.h>
#include <debug.h>
#include <global.h> // After commands.h to avoid conflicts
#include <path.h>
#include "pico/util/queue.h"
#include <utils.h>
#include <vector>

// ============================================================================
// ACTIVE JOINT CONFIGURATION
// ============================================================================

// Set the joint type for this PICO board
// Possible values: JOINT_KNEE_LEFT, JOINT_KNEE_RIGHT, JOINT_ANKLE_LEFT,
// JOINT_ANKLE_RIGHT, JOINT_HIP_LEFT, JOINT_HIP_RIGHT
#define ACTIVE_JOINT JOINT_KNEE_RIGHT
//#define ACTIVE_JOINT JOINT_ANKLE_RIGHT

// CAN ID assignment scheme for motors:
// - IDs always start from 1
// - First DOF: ID 1 agonist, ID 2 antagonist
// - Second DOF: ID 3 agonist, ID 4 antagonist
// - Third DOF (hip only): ID 5 agonist, ID 6 antagonist
//
// Example for a 2‑DOF joint (ankle):
// - DOF 0: ID 1, 2
// - DOF 1: ID 3, 4
//
// Example for a 3‑DOF joint (hip):
// - DOF 0: ID 1, 2
// - DOF 1: ID 3, 4
// - DOF 2: ID 5, 6

// Get active joint configuration
extern const JointConfig &ACTIVE_JOINT_CONFIG;

// ============================================================================
// GLOBAL HARDWARE OBJECTS
// ============================================================================

// Init program flag
extern bool init_prg;

// Command buffer
extern char command[100];

// CAN bus controller for motor control (J4 CAN_Servo - SPI1)
extern MCP_CAN CAN;

// CAN bus controller for host commands (J5 CAN_Controller - SPI1 shared, different CS)
extern MCP_CAN CAN_HOST;

// Direct encoder reading (MT6835 sensors via SPI0)
// Replaces the old encoder1 (Encoders class that used SPI slave)
extern DirectEncoders directEncoders;

// ============================================================================
// SYSTEM STATE VARIABLES
// ============================================================================

// Flash operation synchronization
// Core0 sets this before flash operations, Core1 checks and waits in RAM
extern volatile bool flash_operation_in_progress;

// Movement in progress flag
// Core1 sets this during movement execution to pause Serial streaming on Core0
extern volatile bool movement_in_progress;

// Time offset for synchronization
extern float time_offset;

// Encoder test flags (Serial)
extern bool encoder_test_active;
extern uint8_t encoder_test_joint_id;
extern uint8_t encoder_test_dof_index;
extern bool encoder_test_all_dofs;
extern unsigned long last_encoder_test_time;

// Encoder streaming via CAN (high-frequency, Core1)
extern volatile bool encoder_stream_can_active;
extern volatile uint32_t encoder_stream_last_send_us;

// ============================================================================
// CONTROL LOOP TIMING (configurable)
// ============================================================================

/**
 * @brief Control loop timing parameters
 * 
 * Inner loop: Motor PID, runs at base frequency (500 Hz default)
 * Outer loop: Joint PID, runs at reduced frequency (100 Hz default)
 * 
 * These can be adjusted via CAN for tuning experiments.
 * Changes take effect immediately.
 */
extern volatile uint16_t inner_loop_period_us;  // Inner loop period in µs (default: 2000 = 500Hz)
extern volatile uint8_t outer_loop_divisor;     // Outer loop runs every N inner cycles (default: 1 = 500Hz)
extern volatile uint16_t torque_ramp_time_ms;   // Time for torque to go 0→max (default: 100ms, 0=disabled)
extern volatile uint16_t encoder_error_threshold_ms;  // Time before encoder error triggers emergency stop (default: 100ms)
// NOTE: SPI encoder read interval now uses inner_loop_period_us for automatic synchronization

// CAN error detection: time-window based (more robust than consecutive count)
// Emergency stop triggers if can_error_threshold errors occur within can_error_window_ms
extern volatile uint16_t can_error_window_ms;     // Time window for CAN error detection (default: 50ms)
extern volatile uint8_t can_error_threshold;       // Number of errors in window before emergency stop (default: 5)

// ============================================================================
// COMPLIANCE CONTROL (Deflection/Stall, Anti-Slack, Soft Hold)
// ============================================================================

/**
 * @brief Recovery policy when compliance ends
 */
enum ComplianceRecoveryPolicy : uint8_t {
  RECOVERY_RETURN_TO_TARGET = 0,  // Return to original target
  RECOVERY_STAY_AT_CURRENT = 1,   // Teach mode: stay at current position
  RECOVERY_RAMP_BACK = 2          // Slowly return to target
};

// Expected velocity from current waypoint segment (deg/s)
extern volatile float expected_velocity_deadband_deg_s;  // <= this => treat as holding

// HOLDING deflection detection (expected velocity ~ 0)
extern volatile float hold_error_threshold_deg;          // |error| > this
extern volatile uint16_t hold_time_threshold_ms;         // Duration to confirm
extern volatile float hold_release_threshold_deg;        // Hysteresis for release
extern volatile float hold_release_velocity_deg_s;       // |velocity| < this for release (settling)
extern volatile float hold_release_max_velocity_deg_s;   // Max |velocity| for error-based release
extern volatile uint16_t hold_release_time_ms;           // Duration to confirm release

// MOVING stall detection (expected velocity > deadband)
extern volatile float move_error_threshold_deg;          // |error| > this
extern volatile float move_velocity_ratio;               // |v_actual| < ratio * |v_expected|
extern volatile uint16_t move_time_threshold_ms;         // Duration to confirm

// Actual velocity filtering (from encoder)
extern volatile uint8_t velocity_filter_samples;         // Moving average window

// Anti-slack clamp
extern volatile float anti_slack_margin_deg;             // Motor angle margin
extern volatile bool anti_slack_enabled;                 // Enable anti-slack

// Soft hold (compliance) torque reduction
extern volatile float soft_hold_torque_ratio;            // Reduce max torque to this ratio
extern volatile float min_tension_torque;                // Absolute minimum torque
extern volatile uint16_t soft_hold_ramp_down_ms;         // Ramp time entering compliance
extern volatile uint16_t soft_hold_ramp_up_ms;           // Ramp time leaving compliance
extern volatile bool soft_hold_enabled;                  // Enable soft hold

// Recovery parameters
extern volatile ComplianceRecoveryPolicy recovery_policy;
extern volatile uint16_t recovery_ramp_back_ms;          // For RECOVERY_RAMP_BACK

/**
 * @brief Per-DOF compliance state tracking
 */
struct ComplianceState {
  // Detection state
  bool compliance_active;
  uint32_t hold_candidate_start_ms;
  uint32_t move_candidate_start_ms;
  uint32_t release_candidate_start_ms;
  float stall_entry_angle_deg;
  float original_target_deg;

  // Torque ramp state
  float torque_ratio_current;
  float torque_ratio_start;
  float torque_ratio_target;
  uint32_t torque_ramp_start_ms;
  uint16_t torque_ramp_duration_ms;
  bool torque_ramp_active;

  // Statistics
  uint32_t stall_count;
  uint32_t last_stall_ms;

  void reset() {
    compliance_active = false;
    hold_candidate_start_ms = 0;
    move_candidate_start_ms = 0;
    release_candidate_start_ms = 0;
    stall_entry_angle_deg = 0.0f;
    original_target_deg = 0.0f;
    torque_ratio_current = 1.0f;
    torque_ratio_start = 1.0f;
    torque_ratio_target = 1.0f;
    torque_ramp_start_ms = 0;
    torque_ramp_duration_ms = 0;
    torque_ramp_active = false;
    // Do not reset statistics
  }
};

extern ComplianceState compliance_state[MAX_DOFS];

// ============================================================================
// CAN ERROR TRACKER (shared utility for Movement and Waypoint loops)
// ============================================================================

/**
 * @brief Time-window based CAN error tracker
 *
 * Tracks CAN read errors using a circular buffer of timestamps.
 * More robust than consecutive error counting - tolerates brief EMI glitches.
 * Emergency stop triggers only if threshold errors occur within time window.
 *
 * Usage:
 *   CANErrorTracker tracker;
 *   tracker.recordError(dof_idx);
 *   if (tracker.shouldStop(dof_idx)) { ... emergency stop ... }
 */
#define CAN_ERROR_HISTORY_SIZE 8

class CANErrorTracker {
public:
  // Record an error for a DOF (stores timestamp in circular buffer)
  void recordError(uint8_t dof_idx) {
    if (dof_idx >= MAX_DOFS) return;
    uint32_t now = millis();
    error_timestamps[dof_idx][error_head[dof_idx]] = now;
    error_head[dof_idx] = (error_head[dof_idx] + 1) % CAN_ERROR_HISTORY_SIZE;
  }

  // Count errors within the configured time window
  uint8_t countRecentErrors(uint8_t dof_idx) const {
    if (dof_idx >= MAX_DOFS) return 0;
    uint32_t now = millis();
    uint32_t cutoff = (now > can_error_window_ms) ? (now - can_error_window_ms) : 0;
    uint8_t count = 0;
    for (int i = 0; i < CAN_ERROR_HISTORY_SIZE; i++) {
      if (error_timestamps[dof_idx][i] > cutoff) {
        count++;
      }
    }
    return count;
  }

  // Check if error threshold exceeded (should trigger emergency stop)
  bool shouldStop(uint8_t dof_idx) const {
    return countRecentErrors(dof_idx) >= can_error_threshold;
  }

  // Clear error history for a DOF (call after emergency stop)
  void clearErrors(uint8_t dof_idx) {
    if (dof_idx >= MAX_DOFS) return;
    for (int i = 0; i < CAN_ERROR_HISTORY_SIZE; i++) {
      error_timestamps[dof_idx][i] = 0;
    }
    error_head[dof_idx] = 0;
  }

  // Clear all error history
  void clearAll() {
    for (uint8_t dof = 0; dof < MAX_DOFS; dof++) {
      clearErrors(dof);
    }
  }

private:
  uint32_t error_timestamps[MAX_DOFS][CAN_ERROR_HISTORY_SIZE] = {{0}};
  uint8_t error_head[MAX_DOFS] = {0};
};

// ============================================================================
// PID DIAGNOSTICS DATA (for tuning/debugging)
// ============================================================================

/**
 * @brief Diagnostic data from PID control loop
 * 
 * Updated by Core1 during waypoint execution.
 * Read by Core1 for CAN diagnostic streaming.
 * All angles in degrees * 100 (int16_t for CAN efficiency).
 */
struct PIDDiagnostics {
  volatile int16_t target_deg_x100[3];    // Target angle per DOF (°×100)
  volatile int16_t error_deg_x100[3];     // PID error per DOF (°×100)
  volatile int16_t torque_A[3];           // Torque command agonist per DOF
  volatile int16_t torque_B[3];           // Torque command antagonist per DOF
  volatile uint32_t last_update_ms;       // Timestamp of last update
  volatile bool valid;                    // Data valid flag
};

extern volatile PIDDiagnostics pid_diagnostics;
extern volatile bool pid_diag_stream_active;  // Enable diagnostic streaming

// ============================================================================
// MOTOR ANGLE CACHE (for safety checks without redundant CAN reads)
// ============================================================================

/**
 * @brief Cached motor angles from the control loop
 * 
 * Updated by the waypoint controller during each control cycle.
 * Used by checkMotorsInRange() to avoid redundant CAN reads which
 * were causing ~2ms delays per motor during safety checks.
 * 
 * This reduces safety check overhead from 4000µs to ~50µs.
 */
struct CachedMotorAngles {
    volatile float agonist[MAX_DOFS];      // Last read agonist angle per DOF (degrees)
    volatile float antagonist[MAX_DOFS];   // Last read antagonist angle per DOF (degrees)
    volatile bool valid[MAX_DOFS];          // True if angles have been read at least once
    volatile uint32_t last_update_ms;       // Timestamp of last update
};

extern volatile CachedMotorAngles cached_motor_angles;

// ============================================================================
// MOVEMENT METRICS (for PID tuning evaluation)
// ============================================================================

/**
 * @brief Performance metrics calculated during movement execution
 * 
 * These metrics are computed per-DOF during waypoint execution and
 * sent via CAN when the DOF enters HOLDING state.
 * Used for PID tuning evaluation and optimization.
 */
struct MovementMetrics {
  // Timing metrics (milliseconds)
  uint16_t rise_time_ms;          // Time to reach 90% of target
  uint16_t settling_time_ms;      // Time to enter ±0.5° band permanently
  
  // Accuracy metrics (scaled: degrees × 100 for int16_t)
  int16_t overshoot_x100;         // Maximum overshoot (% × 100, e.g., 250 = 2.5%)
  int16_t sse_x100;               // Steady-state error (°×100)
  int16_t max_error_x100;         // Maximum absolute error during movement (°×100)
  
  // Torque metrics
  int16_t max_torque_A;           // Peak torque agonist
  int16_t max_torque_B;           // Peak torque antagonist
  uint16_t torque_integral;       // Sum of |torque| (energy proxy, saturated)
  
  // Movement info
  int16_t start_angle_x100;       // Starting angle (°×100)
  int16_t target_angle_x100;      // Target angle (°×100)
  uint16_t movement_duration_ms;  // Total movement time
  
  // Status
  uint8_t dof_index;              // Which DOF this is for
  uint8_t flags;                  // Bit flags: 0=valid, 1=overshoot_detected, 2=timeout
};

/**
 * @brief Runtime tracking state for metrics calculation (per DOF)
 */
struct MetricsTracker {
  // State
  bool tracking_active;           // Currently tracking a movement
  uint32_t movement_start_ms;     // When movement actually starts (first waypoint t_arrival)
  uint32_t tracking_init_ms;      // When tracking was initialized (for timeout detection)
  float start_angle_deg;          // Angle at movement start
  float target_angle_deg;         // Target angle to reach
  float movement_direction;       // +1 or -1 (sign of target - start)
  
  // Rise time tracking
  bool reached_90_percent;        // Flag for rise time detection
  uint32_t rise_time_ms;          // When we reached 90%
  
  // Overshoot tracking
  float max_overshoot_deg;        // Maximum overshoot in degrees
  bool overshoot_detected;        // True if we went past target
  
  // Settling tracking
  bool in_settling_band;          // Currently within ±0.5°
  uint32_t settling_enter_ms;     // When we entered the band
  uint32_t settling_time_ms;      // Final settling time (0 if not yet settled)
  uint8_t settling_stable_count;  // Consecutive cycles in band
  
  // Error tracking
  float max_error_deg;            // Maximum absolute error seen
  float sse_accumulator;          // Accumulator for SSE calculation
  uint16_t sse_sample_count;      // Number of samples in HOLDING
  
  // Torque tracking
  int16_t max_torque_A;           // Peak torque agonist
  int16_t max_torque_B;           // Peak torque antagonist
  uint32_t torque_integral;       // Accumulated |torque|
  
  // Reset for new movement
  // t_arrival_ms: when the first waypoint will be executed (actual movement start)
  void reset(float start, float target, uint32_t t_arrival_ms = 0) {
    tracking_active = true;
    tracking_init_ms = millis();
    // Use the first waypoint's arrival time as movement start, not when waypoint was received
    movement_start_ms = (t_arrival_ms > 0) ? t_arrival_ms : millis();
    start_angle_deg = start;
    target_angle_deg = target;
    movement_direction = (target > start) ? 1.0f : -1.0f;
    
    reached_90_percent = false;
    rise_time_ms = 0;
    
    max_overshoot_deg = 0.0f;
    overshoot_detected = false;
    
    in_settling_band = false;
    settling_enter_ms = 0;
    settling_time_ms = 0;
    settling_stable_count = 0;
    
    max_error_deg = 0.0f;
    sse_accumulator = 0.0f;
    sse_sample_count = 0;
    
    max_torque_A = 0;
    max_torque_B = 0;
    torque_integral = 0;
  }
  
  // Finalize and build metrics struct
  MovementMetrics finalize() {
    MovementMetrics m;
    m.dof_index = 0; // Will be set by caller
    m.rise_time_ms = (uint16_t)min(rise_time_ms, 65535UL);
    m.settling_time_ms = (uint16_t)min(settling_time_ms, 65535UL);
    
    // Calculate overshoot as percentage of movement range
    float range = fabs(target_angle_deg - start_angle_deg);
    float overshoot_pct = (range > 0.01f) ? (max_overshoot_deg / range * 100.0f) : 0.0f;
    m.overshoot_x100 = (int16_t)(overshoot_pct * 100.0f);
    
    // SSE: average error in HOLDING
    float sse = (sse_sample_count > 0) ? (sse_accumulator / sse_sample_count) : 0.0f;
    m.sse_x100 = (int16_t)(sse * 100.0f);
    
    m.max_error_x100 = (int16_t)(max_error_deg * 100.0f);
    m.max_torque_A = max_torque_A;
    m.max_torque_B = max_torque_B;
    m.torque_integral = (uint16_t)min(torque_integral / 100, 65535UL); // Scale down
    
    m.start_angle_x100 = (int16_t)(start_angle_deg * 100.0f);
    m.target_angle_x100 = (int16_t)(target_angle_deg * 100.0f);
    m.movement_duration_ms = (uint16_t)min(millis() - movement_start_ms, 65535UL);
    
    m.flags = 0;
    if (tracking_active) m.flags |= 0x01; // Valid
    if (overshoot_detected) m.flags |= 0x02;
    
    return m;
  }
};

// Metrics trackers (one per DOF)
extern MetricsTracker metrics_tracker[3];

// Last computed metrics (one per DOF, updated when entering HOLDING)
extern MovementMetrics last_movement_metrics[3];
extern volatile bool metrics_ready[3];  // Flag to signal new metrics available

// Flag to enable/disable metrics tracking (default: enabled)
// Set to false for critical timing tests or to reduce overhead
extern volatile bool metrics_tracking_enabled;

// Auto-mapping state
extern AutoMappingState_t auto_mapping_state;

// Control flag for sending mapping data
extern bool auto_mapping_data_ready_to_send;

// ============================================================================
// MULTI-JOINT SUPPORT DATA STRUCTURES
// ============================================================================

// Data structures for multi‑joint support
extern shared_data_extended_t shared_data_ext;
extern command_data_extended_t command_data_ext;
extern measuring_data_extended_t measuring_data_ext;

// Command parser
extern CommandParser command_parser;

// Active joint controller
extern JointController *active_joint_controller;

// ============================================================================
// MOVEMENT SAMPLE LOGGING
// ============================================================================

using MovementSample = movement_sample_t;

extern queue_t movement_sample_queue;
extern volatile bool movement_sample_stream_active;
extern volatile bool movement_sample_stream_done;
extern volatile uint8_t movement_sample_joint_id;
extern volatile bool movement_sample_overflow;
extern uint16_t movement_sample_counters[MAX_DOFS];

// Movement sample helper functions
const char *jointIdToSerialName(uint8_t joint_id);
void clearMovementSampleQueue();
void flushMovementSamples();

// ============================================================================
// SHARED DOF ANGLES (Updated by Core0, read by Core1 and UI)
// ============================================================================

/**
 * @brief Centralized DOF angle state updated by Core0 every cycle
 * 
 * This structure provides a single source of truth for DOF angles.
 * Core0 reads from encoders and updates this structure.
 * Core1 and other components read from here instead of directly from encoders.
 * 
 * Benefits:
 * - Single SPI read per cycle (efficiency)
 * - Consistent values across all consumers in the same cycle
 * - Deterministic timing for control loops
 */
struct SharedDofAngles {
  float angles[MAX_DOFS];           // Validated angles in degrees
  float velocities[MAX_DOFS];       // Calculated velocities in deg/s
  uint32_t timestamp_us;            // Microsecond timestamp of last update
  bool valid[MAX_DOFS];             // Per-DOF validity flag
  volatile bool updated;            // Flag set by Core0, cleared by Core1
  uint8_t dof_count;                // Active DOF count
};

extern SharedDofAngles shared_dof_angles;

// Function to update shared DOF angles (called by Core0)
void updateSharedDofAngles();

// ============================================================================
// INTER-CORE COMMUNICATION (CORE0 <-> CORE1)
// ============================================================================

// Double‑buffered command passing
extern command_data_extended_t command_buffer[2];
extern volatile int active_buffer;
extern volatile bool buffer_ready[2];
extern volatile uint8_t pending_command_type;

// Separate emergency flag for extra safety
extern volatile bool emergency_stop_requested;

// Smooth transition flag
extern volatile bool smooth_transition_active;

// Array of active controllers accessible from core1 (indices 1..6 for joints)
extern JointController *active_controllers[7]; // Index 0 not used

// Active controller state for current cycle
extern uint8_t current_joint_id;
extern uint8_t current_dof_index;



// ============================================================================
// SETUP FUNCTION (called by main.cpp)
// ============================================================================

void setup_common();

// ============================================================================
// CORE LOOPS (implemented in core0.cpp and core1.cpp)
// ============================================================================

void core0_main_loop();  // Core0 loop - serial communication (implemented in core0.cpp)
void core1_loop();       // Core1 loop - hardware operations (implemented in core1.cpp)

#endif // MAIN_COMMON_H

