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
extern volatile uint16_t encoder_read_interval_us;    // SPI encoder read interval in µs (default: 2000 = 500Hz)

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
// NOTCH FILTER CONFIGURATION (for resonance suppression)
// ============================================================================

/**
 * @brief Notch filter configuration for torque output filtering
 * 
 * The notch filter can eliminate specific resonance frequencies from
 * the torque control loop. By default it is bypassed (disabled).
 * 
 * Typical resonance frequencies: 5-15 Hz depending on joint geometry.
 * Configure via CAN command or Serial for tuning experiments.
 */
struct NotchFilterConfig {
    volatile bool enabled;           // Filter enabled (default: false = bypass)
    volatile float center_freq_hz;   // Center frequency to eliminate (default: 8.0 Hz)
    volatile float quality;          // Q factor: 0.8=wide, 0.99=narrow (default: 0.90)
    volatile bool config_changed;    // Flag to signal reconfiguration needed
};

extern volatile NotchFilterConfig notch_filter_config;

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

