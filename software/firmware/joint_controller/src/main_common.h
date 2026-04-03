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
#include <atomic>
#include <SPI.h>
#include <mcp_can.h>

// Multi-joint support includes (before legacy system)
#include <CommandParser.h>
#include <JointConfig.h>
#include <JointController.h>
#include <commands.h> // Before global.h to avoid conflicts
#include <config_presets.h>
#include <shared_data.h>
#include <safety_system.h>
// Legacy support includes
#include <PID.h>
#include <debug.h>
#include <global.h> // After commands.h to avoid conflicts
#include <path.h>
#include "pico/util/queue.h"
#include <utils.h>
#include <vector>

// ============================================================================
// RUNTIME JOINT PROFILE / BOARD MODE
// ============================================================================

// The board identity now comes from persisted flash settings.
// If no valid profile is provisioned, the firmware enters an unprovisioned
// safe mode (`JOINT_NONE`) and exposes only identity/provisioning paths.

// Cross-chip CAN bus test: bridge J4 CAN_H↔J5 CAN_H and J4 CAN_L↔J5 CAN_L
// externally, then enable this to verify both transceivers through the physical bus.
// Disable for normal operation (motors on J4, host on J5 are separate buses).
//#define CAN_CROSS_CHIP_TEST


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

// ============================================================================
// GLOBAL HARDWARE OBJECTS
// ============================================================================

// Init program flag
extern bool init_prg;

// Command buffer
extern char command[100];

// CAN bus controller for motor control (J4 CAN_Servo - SPI1)
extern MCP_CAN CAN;

// CAN loopback test utility (used by setup and CMD_CAN_DIAG)
bool can_loopback_test(MCP_CAN &can, const char* label,
                       unsigned long test_id, const unsigned char test_data[8]);

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
// Core1 sets this to confirm it has entered the RAM wait loop
extern volatile bool core1_flash_acknowledged;

// Movement in progress flag
// Core1 sets this during movement execution to pause Serial streaming on Core0
extern volatile bool movement_in_progress;

// Host CAN polling suspend flag
// Set by Core0 during critical SPI operations (motor access, recalc) to prevent
// SPI bus conflicts with Host CAN on the shared SPI1 bus
extern volatile bool suspend_host_can_polling;

// System settings (loaded from flash at boot)
extern SystemSettingsData system_settings;
extern bool system_settings_loaded;

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

// Core1 → Core0 flag: request safe limits emission via Serial
extern volatile bool emit_safe_limits_requested;

// Joint identification broadcast (triggered via CAN, emitted on Serial + CAN)
extern volatile bool identify_broadcast_active;
extern volatile uint32_t identify_broadcast_start_ms;
extern const uint32_t IDENTIFY_BROADCAST_DURATION_MS;
extern const uint32_t IDENTIFY_BROADCAST_INTERVAL_MS;
extern volatile uint32_t identify_broadcast_last_emit_ms;
extern volatile uint32_t identify_can_announce_last_ms;  // CAN announce timing (Core1)

// CAN-triggered startup sequence (Core1 sets flag, Core0 executes)
extern volatile bool can_startup_requested;
extern volatile uint8_t can_startup_joint_id;
extern volatile int16_t can_startup_torque;    // 0 = use config default
extern volatile int16_t can_startup_duration;  // 0 = use config default

// CAN-triggered set-zero (Core1 sets flags, Core0 executes)
extern volatile bool can_set_zero_requested;
extern volatile uint8_t can_set_zero_dof_index;

// CAN encoder offset notification (Core0 sets flag, Core1 sends CAN frames)
extern volatile bool can_encoder_offsets_notify;

// CAN-triggered PID flash operations (Core1 sets flag, Core0 executes)
extern volatile bool can_save_pid_requested;
extern volatile bool can_load_pid_requested;

// CAN-triggered linear equations flash ops (Core1 sets flag, Core0 executes)
extern volatile bool can_save_linear_eq_requested;
extern volatile bool can_load_linear_eq_requested;

// CAN-triggered auto-start setting (Core1 sets params + flag, Core0 executes flash save)
extern volatile bool can_set_auto_start_requested;
extern volatile uint8_t can_auto_start_enabled;
extern volatile int16_t can_auto_start_torque;
extern volatile uint16_t can_auto_start_duration;

// Startup status event queue (Core0 produces, Core1 consumes and sends via CAN)
struct StartupStatusEvent {
  uint8_t event_type;   // 0=BEGIN, 1=DOF_READY, 2=DOF_FAILED, 3=COMPLETE, 4=FAILED
  uint8_t dof_index;
  uint8_t reason_code;  // 0=OK, 1=NO_CONTROLLER, 2=NO_EQUATIONS, 3=ENCODER_TIMEOUT,
                        // 4=POSITION_OUT_OF_RANGE, 5=RECALC_ERROR, 6=GLOBAL_TIMEOUT
  uint16_t elapsed_ms;
};

#define STARTUP_EVENT_QUEUE_DEPTH 8
extern queue_t startup_event_queue;

// Diagnostic event queue (Core1 producers → Core1 CAN sender)
struct DiagnosticEventNoticePending {
  uint8_t event_code;
  uint8_t flags;
  uint8_t source_kind;
  uint8_t source_index;
  uint8_t detail0;
  uint8_t detail1;
};

#define DIAG_EVENT_QUEUE_DEPTH 16
extern queue_t diag_event_notice_queue;

// ============================================================================
// CONTROL LOOP TIMING (configurable)
// ============================================================================

/**
 * @brief Control loop timing parameters
 * 
 * Inner loop: Motor PID, runs at base frequency (500 Hz default)
 * Outer loop: Joint PID, runs every N inner cycles (default matches inner at 500 Hz)
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
// Motor angle jump detection threshold expressed as max speed (°/s).
// Per-cycle threshold is computed automatically: jump_threshold_dps * inner_loop_period_us / 1e6
// Default 10000°/s → 20° per cycle @ 500Hz (~1.4× motor max of 7200°/s).
extern volatile float can_motor_jump_threshold_dps;

// Oscillation detection: safety feature that detects dangerous oscillations and triggers emergency stop
// Emergency stop triggers if osc_min_sign_changes sign reversals occur within osc_window_ms
// AND the oscillation amplitude exceeds osc_min_amplitude_deg
extern volatile uint16_t osc_window_ms;              // Time window for sign change counting (default: 500ms)
extern volatile uint8_t osc_min_sign_changes;         // Min sign changes to trigger (default: 4 = 2 full cycles)
extern volatile float osc_min_amplitude_deg;          // Min amplitude to be dangerous (default: 3.0°)
extern volatile float osc_min_error_to_check;         // Skip check if error below this (default: 1.0°)

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

// Expected velocity from current movement segment (deg/s)
extern volatile float expected_velocity_deadband_deg_s;  // <= this => treat as holding
extern volatile float outer_hold_ki_scale;               // Final Ki scale after HOLDING ramp
extern volatile uint16_t outer_hold_ki_ramp_ms;         // Ramp Ki scale from 1.0 → outer_hold_ki_scale
extern volatile float outer_hold_integral_freeze_error_deg;      // Freeze I if |error| below this in HOLDING
extern volatile float outer_hold_integral_freeze_velocity_deg_s; // Freeze I if |velocity| below this in HOLDING
extern volatile bool retension_probe_enabled;            // Periodic local re-tension probe during clean HOLDING
extern volatile float retension_probe_boost_deg;         // Temporary boost added to stiffness_ref during probe
extern volatile uint16_t retension_probe_pulse_ms;       // Probe pulse duration
extern volatile uint16_t retension_probe_start_delay_ms; // Delay after entering HOLDING before probe
extern volatile uint16_t retension_probe_post_ms;        // Post-pulse observation window
extern volatile uint16_t retension_probe_repeat_ms;      // Periodic re-arm interval while HOLDING stays clean
extern volatile float retension_probe_min_hold_q_deg;    // Minimum |joint angle| to probe (0 = any pose)
extern volatile uint16_t diag_hold_min_samples;          // Minimum gated samples before DIAG_HOLD emission
extern volatile uint16_t diag_hold_ema_settled_samples;  // Minimum gated samples before EMA is considered settled
extern volatile uint16_t diag_hold_period_ms;            // Period of DIAG_HOLD emission while gated
extern volatile uint16_t trim_dry_run_min_samples;       // Minimum gated samples before proposed trim updates
extern volatile uint16_t trim_dry_run_period_ms;         // Minimum time between proposed trim updates

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

// Friction feedforward compensation (overcomes static friction at low speeds)
// At low velocity, tendon systems exhibit stick-slip: the joint stalls until PID
// accumulates enough error to overcome static friction, then snaps forward.
// This feedforward adds a small torque in the direction of motion to pre-load
// against static friction, reducing the stall duration and overshoot at break-free.
extern volatile bool friction_ff_enabled;                 // Enable friction feedforward
extern volatile float friction_ff_torque;                 // Torque magnitude (motor units)
extern volatile float friction_ff_speed_thresh;           // Below this speed (deg/s): apply FF

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
// CAN ERROR TRACKER (shared utility for control loop)
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
  // Uses unsigned subtraction (now - timestamp) which handles millis() wrap correctly
  uint8_t countRecentErrors(uint8_t dof_idx) const {
    if (dof_idx >= MAX_DOFS) return 0;
    uint32_t now = millis();
    uint8_t count = 0;
    for (int i = 0; i < CAN_ERROR_HISTORY_SIZE; i++) {
      uint32_t ts = error_timestamps[dof_idx][i];
      if (ts != 0 && (now - ts) < can_error_window_ms) {
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
 * Written AND read exclusively on Core1 (control loop + CAN streaming).
 * No cross-core access — volatile not needed on members.
 * All angles in degrees * 100 (int16_t for CAN efficiency).
 */
struct PIDDiagnostics {
  int16_t target_deg_x100[3];    // Target angle per DOF (°×100)
  int16_t error_deg_x100[3];     // PID error per DOF (°×100)
  int16_t torque_A[3];           // Torque command agonist per DOF
  int16_t torque_B[3];           // Torque command antagonist per DOF
  // Inner PID contribution breakdown (agonist motor, DOF 0 only)
  int16_t inner_p_term;          // Proportional increment
  int16_t inner_i_term;          // Integral term
  int16_t inner_d_term;          // Filtered derivative term
  int16_t inner_ff_term;         // Feedforward term
  // Outer PID contribution breakdown (joint PID, DOF 0 only)
  int16_t outer_p_term;          // Proportional increment
  int16_t outer_i_term;          // Integral term
  int16_t outer_d_term;          // Filtered derivative term
  int16_t outer_output;          // delta_theta output (°×100)
  bool pid_terms_valid;          // PID terms breakdown populated
  uint32_t last_update_ms;       // Timestamp of last update
  bool valid;                    // Data valid flag
};

extern PIDDiagnostics pid_diagnostics;
extern volatile bool pid_diag_stream_active;   // Cross-core: CAN sets, Core1 reads
extern volatile bool pid_diag_terms_enabled;   // Cross-core: enable P/I/D breakdown streaming

// ============================================================================
// HOLDING DIAGNOSTICS (Phase 1 slack/bias data for UI streaming)
// ============================================================================

/**
 * @brief Diagnostic data captured during gated HOLDING, streamed via CAN to host.
 *
 * Written by Core0 (control loop) at diag_hold_period_ms cadence during clean HOLDING.
 * Read by Core1 and sent as CAN frame 0x4D0+joint_id.
 * All signals per SLACK_DETECTION_AND_TENSION_TRIM.md Phase 1.
 */
struct DiagHoldData {
    int16_t ema_x100;           // delta_theta EMA (°×100)
    int16_t residual_A_x100;    // motor residual agonist (°×100)
    int16_t residual_B_x100;    // motor residual antagonist (°×100)
    int16_t iq_A;               // torque current agonist (raw)
    int16_t iq_B;               // torque current antagonist (raw)
    int16_t stiffness_x10;      // stiffness_ref (°×10)
    int16_t tension_trim_x100;  // tension_trim_deg (°×100) — Phase 2, signed
    uint8_t dof;                // DOF index
    uint8_t flags;              // bit0=iq_valid, bit1=ema_settled, bit2-7=reserved
    volatile uint32_t seq;      // sequence counter: Core0 increments before+after write
                                // Core1 reads seq, copies, reads seq again — retry if mismatch
};

extern DiagHoldData diag_hold_data[MAX_DOFS];

// ============================================================================
// RETENSION PROBE RESULT (generic active-sensing output for host policy)
// ============================================================================

/**
 * @brief Probe summary captured after an active stiffness pulse during HOLDING.
 *
 * Written by Core0 when a probe window completes. Read by Core1 and sent as a
 * small CAN telemetry burst so the host can log and interpret it generically,
 * without baking joint-specific trim policy into firmware.
 */
struct RetensionProbeResultData {
    int16_t q_x100;               // Joint angle at probe (°×100)
    int16_t base_stiffness_x10;   // Baseline stiffness before temporary probe boost (°×10)
    int16_t pre_ratio_x1000;      // iq ratio before pulse (×1000)
    int16_t dur_ratio_x1000;      // iq ratio during pulse (×1000)
    int16_t delta_ratio_x1000;    // dur_ratio - pre_ratio (×1000)
    int16_t recruit_norm_x1000;   // Weak-side recruitment normalized by baseline max (×1000)
    uint16_t effort_pre;          // Baseline total |Iq| effort (raw)
    int16_t boost_x10;            // Probe boost amplitude (°×10)
    uint16_t pulse_ms;            // Probe pulse duration (ms)
    uint8_t dof;                  // DOF index
    uint8_t flags;                // bit0=weak_side_is_B, bit1-7 reserved
    uint8_t class_code;           // Heuristic classification (host may ignore)
    uint8_t min_samples;          // Minimum samples collected across pre/during/post windows
    volatile uint32_t seq;        // Sequence counter for cross-core consistent snapshot
};

extern RetensionProbeResultData retension_probe_result_data[MAX_DOFS];

// Reset session-local diagnostics (proposed_trim, EMA, samples) for a DOF.
// Defined in JointController_ControlLoop.cpp, callable from core1 E-Stop path.
void resetDiagHoldState(uint8_t dof);

// ============================================================================
// DIAGNOSTIC PLANE (CAN-first observability)
// ============================================================================

enum DiagFaultCode : uint8_t {
  DIAG_FAULT_HOST_CAN_WARN = 0,
  DIAG_FAULT_MOTOR_CAN_WARN = 1,
  DIAG_FAULT_LOOP_OVERRUN = 2,
  DIAG_FAULT_HOST_WATCHDOG_TIMEOUT = 3,
  DIAG_FAULT_ENCODER_INVALID = 4,
  DIAG_FAULT_ENCODER_STALE = 5,
  DIAG_FAULT_MOTOR_TIMEOUT = 6,
  DIAG_FAULT_SAFETY_LIMIT = 7,
  DIAG_FAULT_MAPPING_LIMIT = 8,
  DIAG_FAULT_MOTOR_RANGE = 9,
  DIAG_FAULT_STARTUP_FAILED = 10,
  DIAG_FAULT_CONFIG_INVALID = 11,
  DIAG_FAULT_FLASH_ERROR = 12,
  DIAG_FAULT_BAD_COMMAND = 13,
  DIAG_FAULT_ESTOP_LATCHED = 14,
  DIAG_FAULT_INTERNAL_ERROR = 15,
  DIAG_FAULT_COUNT,
};
static_assert(DIAG_FAULT_COUNT <= 16, "Diagnostic fault mask exceeds 16-bit wire format");

enum DiagEventCode : uint8_t {
  DIAG_EVENT_BOOT_COMPLETE = 0x01,
  DIAG_EVENT_READY_ASSERTED = 0x02,
  DIAG_EVENT_READY_CLEARED = 0x03,
  DIAG_EVENT_STARTUP_BEGIN = 0x04,
  DIAG_EVENT_STARTUP_COMPLETE = 0x05,
  DIAG_EVENT_STARTUP_FAILED = 0x06,
  DIAG_EVENT_WATCHDOG_WARNING = 0x07,
  DIAG_EVENT_WATCHDOG_TIMEOUT = 0x08,
  DIAG_EVENT_ESTOP_ASSERTED = 0x09,
  DIAG_EVENT_ESTOP_CLEARED = 0x0A,
  DIAG_EVENT_FAULT_SET = 0x0B,
  DIAG_EVENT_FAULT_CLEARED = 0x0C,
  DIAG_EVENT_ENCODER_INVALID = 0x0D,
  DIAG_EVENT_MOTOR_TIMEOUT = 0x0E,
  DIAG_EVENT_LOOP_OVERRUN_BURST = 0x0F,
  DIAG_EVENT_SNAPSHOT_FROZEN = 0x10,
  DIAG_EVENT_SNAPSHOT_AVAILABLE = 0x11,
};

enum DiagEventSourceKind : uint8_t {
  DIAG_SRC_GLOBAL = 0,
  DIAG_SRC_DOF = 1,
  DIAG_SRC_MOTOR = 2,
  DIAG_SRC_HOST_CAN = 3,
  DIAG_SRC_MOTOR_CAN = 4,
  DIAG_SRC_STARTUP = 5,
  DIAG_SRC_CONFIG = 6,
  DIAG_SRC_SAFETY = 7,
};

void diag_init_boot_reason();
void diag_set_startup_in_progress(bool active);
void diag_set_estop_latched(bool latched);
void diag_note_watchdog_timeout(uint8_t dof, uint32_t elapsed_ms);
void diag_note_loop_overrun();
void diag_note_motor_timeout(uint8_t dof, uint8_t motor_index);
void diag_note_encoder_invalid(uint8_t dof);
void diag_note_safety_violation(uint8_t dof, SafetyViolationType violation_type);
void diag_note_bad_command(uint8_t source_kind, uint8_t source_index);

// ============================================================================
// MOTOR ANGLE CACHE (for safety checks without redundant CAN reads)
// ============================================================================

/**
 * @brief Cached motor angles from the control loop
 * 
 * Written AND read exclusively on Core1 (impedance control + safety checks).
 * No cross-core access — volatile not needed.
 * Eliminates ~2ms CAN read delay per motor during safety checks.
 */
struct CachedMotorAngles {
    float agonist[MAX_DOFS];      // Last read agonist angle per DOF (degrees)
    float antagonist[MAX_DOFS];   // Last read antagonist angle per DOF (degrees)
    bool valid[MAX_DOFS];          // True if angles have been read at least once
    uint32_t last_update_ms;       // Timestamp of last update
};

extern CachedMotorAngles cached_motor_angles;

// ============================================================================
// MOVEMENT METRICS (for PID tuning evaluation)
// ============================================================================

/**
 * @brief Performance metrics calculated during movement execution
 * 
 * These metrics are computed per-DOF during movement execution and
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
  
  // Smoothness/Oscillation metrics (tracking quality during MOVING)
  int16_t rms_error_x100;         // RMS tracking error during MOVING (°×100)
  uint16_t oscillation_count;     // Zero-crossings of error during MOVING (sign changes)
  int16_t jitter_x100;            // RMS of error derivative (°×100) - high = vibrations
  
  // Smoothness scores (0-100, higher = better)
  uint8_t score_rms;              // RMS error score: 100 if <0.1°, 0 if >2°
  uint8_t score_oscillation;      // Oscillation score: 100 if 0, decreases with count
  uint8_t score_jitter;           // Jitter score: 100 if <0.02°, 0 if >0.3°
  uint8_t score_smoothness;       // Overall smoothness (weighted average)
  
  // Movement info
  int16_t start_angle_x100;       // Starting angle (°×100)
  int16_t target_angle_x100;      // Target angle (°×100)
  uint16_t movement_duration_ms;  // Total movement time
  
  // Status
  uint8_t dof_index;              // Which DOF this is for
  uint8_t flags;                  // Bit flags: 0=valid, 1=overshoot_detected, 2=timeout, 3=stall_aborted
};

/**
 * @brief Runtime tracking state for metrics calculation (per DOF)
 */
struct MetricsTracker {
  // State
  bool tracking_active;           // Currently tracking a movement
  uint32_t movement_start_ms;     // When movement actually starts (first target t_arrival)
  uint32_t tracking_init_ms;      // When tracking was initialized (for timeout detection)
  float start_angle_deg;          // Angle at movement start
  float target_angle_deg;         // Target angle to reach
  float movement_direction;       // +1 or -1 (sign of target - start)
  bool aborted_by_stall;          // True if movement aborted due to stall
  float abort_target_deg;         // Desired target at stall abort
  
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
  
  // Oscillation/Smoothness tracking (during MOVING phase)
  float error_sum_sq;             // Sum of squared errors (for RMS)
  uint32_t moving_sample_count;   // Number of samples during MOVING
  uint16_t zero_crossings;        // Count of sign changes in error
  float prev_error_sign;          // Previous error sign (-1, 0, +1)
  float error_deriv_sum_sq;       // Sum of squared (error - prev_error) for jitter
  float prev_error_deg;           // Previous error value for derivative
  bool prev_error_valid;          // True after first sample (for derivative)
  
  // Torque tracking
  int16_t max_torque_A;           // Peak torque agonist
  int16_t max_torque_B;           // Peak torque antagonist
  uint32_t torque_integral;       // Accumulated |torque|
  
  // Reset for new movement
  // t_arrival_ms: when the first target will be executed (actual movement start)
  void reset(float start, float target, uint32_t t_arrival_ms = 0) {
    tracking_active = true;
    tracking_init_ms = millis();
    // Use the first target's arrival time as movement start, not when target was received
    movement_start_ms = (t_arrival_ms > 0) ? t_arrival_ms : millis();
    start_angle_deg = start;
    target_angle_deg = target;
    movement_direction = (target > start) ? 1.0f : -1.0f;
    aborted_by_stall = false;
    abort_target_deg = 0.0f;
    
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
    
    // Oscillation/Smoothness reset
    error_sum_sq = 0.0f;
    moving_sample_count = 0;
    zero_crossings = 0;
    prev_error_sign = 0.0f;
    error_deriv_sum_sq = 0.0f;
    prev_error_deg = 0.0f;
    prev_error_valid = false;
    
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
    
    // Oscillation/Smoothness metrics (calculated from MOVING phase data)
    // RMS error: sqrt(sum_sq / n)
    float rms_error = (moving_sample_count > 0) ? sqrtf(error_sum_sq / moving_sample_count) : 0.0f;
    m.rms_error_x100 = (int16_t)(rms_error * 100.0f);
    
    // Zero-crossings: direct count
    m.oscillation_count = zero_crossings;
    
    // Jitter: RMS of error derivative (indicates high-frequency vibrations)
    // Note: we have (n-1) derivative samples for n error samples
    uint32_t deriv_samples = (moving_sample_count > 1) ? (moving_sample_count - 1) : 0;
    float jitter = (deriv_samples > 0) ? sqrtf(error_deriv_sum_sq / deriv_samples) : 0.0f;
    m.jitter_x100 = (int16_t)(jitter * 100.0f);
    
    // === SMOOTHNESS SCORES (0-100, higher = better) ===
    // RMS Score: 100 if rms < 0.1°, 0 if rms > 2°, linear interpolation
    const float RMS_EXCELLENT = 0.1f;  // 100 points
    const float RMS_POOR = 2.0f;       // 0 points
    if (rms_error <= RMS_EXCELLENT) {
      m.score_rms = 100;
    } else if (rms_error >= RMS_POOR) {
      m.score_rms = 0;
    } else {
      m.score_rms = (uint8_t)(100.0f * (RMS_POOR - rms_error) / (RMS_POOR - RMS_EXCELLENT));
    }
    
    // Oscillation Score: 100 if 0 crossings, decreases exponentially
    // Score = 100 * exp(-osc/5), so 5 crossings = 37, 10 crossings = 14
    const float OSC_DECAY = 5.0f;
    m.score_oscillation = (uint8_t)(100.0f * expf(-((float)zero_crossings) / OSC_DECAY));
    
    // Jitter Score: 100 if jitter < 0.02°, 0 if jitter > 0.3°, linear
    const float JITTER_EXCELLENT = 0.02f;  // 100 points
    const float JITTER_POOR = 0.30f;       // 0 points
    if (jitter <= JITTER_EXCELLENT) {
      m.score_jitter = 100;
    } else if (jitter >= JITTER_POOR) {
      m.score_jitter = 0;
    } else {
      m.score_jitter = (uint8_t)(100.0f * (JITTER_POOR - jitter) / (JITTER_POOR - JITTER_EXCELLENT));
    }
    
    // Overall Smoothness Score: weighted average (rms 40%, osc 30%, jitter 30%)
    m.score_smoothness = (uint8_t)(0.4f * m.score_rms + 0.3f * m.score_oscillation + 0.3f * m.score_jitter);
    
    m.start_angle_x100 = (int16_t)(start_angle_deg * 100.0f);
    m.target_angle_x100 = (int16_t)(target_angle_deg * 100.0f);
    m.movement_duration_ms = (uint16_t)min(millis() - movement_start_ms, 65535UL);
    
    m.flags = 0;
    if (tracking_active) m.flags |= 0x01; // Valid
    if (overshoot_detected) m.flags |= 0x02;
    if (aborted_by_stall) m.flags |= 0x08;
    
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
  volatile uint32_t seq;           // Sequence lock for cross-core consistency
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

// Read a consistent snapshot of shared DOF angles (sequence lock)
inline void readSharedDofAnglesSnapshot(SharedDofAngles &out) {
  uint32_t seq_start = 0;
  uint32_t seq_end = 0;
  do {
    seq_start = __atomic_load_n(&shared_dof_angles.seq, __ATOMIC_ACQUIRE);
    if (seq_start & 1U) {
      continue;  // Writer in progress
    }
    out = shared_dof_angles;
    seq_end = __atomic_load_n(&shared_dof_angles.seq, __ATOMIC_ACQUIRE);
  } while (seq_start != seq_end || (seq_end & 1U));
}

// ============================================================================
// CORE1 LOG QUEUE (lock-free Core1 → Core0 logging)
// ============================================================================
// Serial.print from Core1 causes USB CDC cross-core deadlock on RP2350.
// All Core1 logging goes through this lock-free queue; Core0 drains and prints.

#define CORE1_LOG_MSG_SIZE 120   // Max message length (including null terminator)
#define CORE1_LOG_QUEUE_DEPTH 32 // Number of entries (~4KB total)

struct Core1LogEntry {
  uint8_t level;                    // 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=COM (protocol)
  char msg[CORE1_LOG_MSG_SIZE];     // Null-terminated message
};

extern queue_t core1_log_queue;

// Drain Core1 log queue and print to Serial (called from Core0 main loop)
void drainCore1LogQueue();

// ============================================================================
// INTER-CORE COMMUNICATION (CORE0 <-> CORE1)
// ============================================================================

// Double‑buffered command passing
extern command_data_extended_t command_buffer[2];
extern volatile int active_buffer;
extern volatile bool buffer_ready[2];
extern volatile uint8_t pending_command_type;

// Separate emergency flag for extra safety (atomic for cross-core access)
extern std::atomic<bool> emergency_stop_requested;

// Array of active controllers accessible from core1 (indices 1..6 for joints)
extern JointController *active_controllers[9]; // Index 0 not used, joint IDs 1..8

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

// ============================================================================
// DOF CONTROL STATE
// ============================================================================

/**
 * @brief Per-DOF control state
 *
 * Tracks whether each DOF is idle, actively moving (impedance segment),
 * or holding position. Standalone per-DOF state for the impedance controller.
 *
 * Cross-core contract: Core0 writes during startup sequence (motors stopped,
 * control loop not active for this DOF). Core1 writes during normal operation
 * (watchdog timeout, IMPEDANCE_CTRL disable, safety transitions). Reads by
 * Core1 control loop every cycle. Volatile ensures compiler does not cache
 * stale values across loop iterations.
 *
 * On RP2350 (ARM Cortex-M33), aligned ≤32-bit reads/writes are naturally
 * atomic. The startup sequence writes happen while Core1 sees dof_state==IDLE
 * (control loop skips IDLE DOFs), so no torn-read risk in practice.
 */
enum class DofState : uint8_t {
  IDLE = 0,
  MOVING,
  HOLDING
};

extern volatile DofState dof_state[MAX_DOFS];
extern volatile float dof_hold_angle[MAX_DOFS];     // Hold reference angle (degrees)
extern volatile uint32_t dof_hold_time[MAX_DOFS];   // Timestamp of last state update

// ============================================================================
// IMPEDANCE CONTROL (Scenario B — SET_IMPEDANCE CAN command)
// ============================================================================

/**
 * @brief Impedance target from Jetson (received via SET_IMPEDANCE CAN command)
 *
 * Received as 1 to 4 CAN frames (accumulator pattern on 0x01D).
 * Written by Core1 CAN handler, read by Core1 control loop.
 * No cross-core access — volatile only on last_update_ms for watchdog.
 */
struct ImpedanceTarget {
  float q_target_deg;      // Desired joint position (degrees)
  float dq_target_deg_s;   // Requested cruise speed magnitude (deg/s)
  float stiffness_deg;     // Co-contraction stiffness (degrees)
  float kp;                // Position gain (outer PID Kp)
  float ki;                // Position integral gain (outer PID Ki)
  float kd;                // Velocity gain (outer PID Kd)
  float kp_inner;          // Inner PID Kp override (applied to both motors)
  float ki_inner;          // Inner PID Ki override (applied to both motors)
  float kd_inner;          // Inner PID Kd override (applied to both motors)
  int16_t tau_ff;          // Feedforward torque (raw motor units)
  uint32_t last_update_ms; // Timestamp of last SET_IMPEDANCE (for watchdog)
  bool valid;              // True after first complete SET_IMPEDANCE received
  bool watchdog_timed_out; // Host stream timed out; firmware is holding locally
};

/**
 * @brief Rolling local segment generated from the latest SET_IMPEDANCE goal.
 *
 * Instead of tracking the raw q_target as an immediate step, the firmware
 * generates a single overwrite-only local segment. Each new command starts a
 * new segment from the current q_ref and reaches q_goal at the requested
 * cruise speed. This provides smooth interpolation without maintaining a
 * queue.
 */
struct ImpedanceRollingSegment {
  float q_goal_deg;        // Latest commanded goal
  float q_start_deg;       // Segment start used for interpolation
  float q_ref_deg;         // Last evaluated local reference
  float dq_ref_deg_s;      // Last evaluated local reference speed
  float speed_abs_deg_s;   // Cruise speed magnitude for this segment
  uint32_t t_start_ms;     // Segment start time
  uint32_t t_arrival_ms;   // Segment end time
  bool active;             // True while q_ref is still moving towards q_goal
  bool initialized;        // True after first valid segment/hold has been created
};

/**
 * @brief Backup of original inner PID gains for restore on impedance mode exit.
 *
 * When a DOF enters impedance mode, the original inner PID Kp/Ki/Kd are saved
 * so they can be restored when impedance mode ends (via disable,
 * watchdog timeout, or E-Stop).
 */
struct InnerPidBackup {
  float kp;
  float ki;
  float kd;
  bool saved;
};

extern InnerPidBackup inner_pid_backup[MAX_DOFS][2];  // [dof][0=agonist, 1=antagonist]

/**
 * @brief Backup of outer loop parameters before entering impedance mode.
 *
 * SET_IMPEDANCE overrides outer Kp/Ki/Kd/stiffness on the cascade PID.
 * The original values must be restored when leaving impedance mode so
 * holding behavior retains its tuned gains.
 */
struct OuterLoopBackup {
  float kp;
  float ki;
  float kd;
  float stiffness_deg;
  float cascade_influence;
  bool saved;
};

extern OuterLoopBackup outer_loop_backup[MAX_DOFS];

// Flag: inner PID needs bumpless reinitialization after impedance parameter override.
// Set by core1 (restoreInnerPidGains), consumed by control loop.
extern volatile bool inner_pid_reinit_after_impedance[MAX_DOFS];

// Per-DOF impedance targets (written by CAN handler on Core1)
extern ImpedanceTarget impedance_target[MAX_DOFS];

// Per-DOF rolling impedance segments (written by CAN handler, evaluated in control loop)
extern ImpedanceRollingSegment impedance_segment[MAX_DOFS];

// Impedance watchdog timeout (ms). If no SET_IMPEDANCE arrives within this
// period, the DOF transitions to HOLDING at current position.
// Configurable via IMPEDANCE_CTRL (0x01E) sub_cmd=0x02.
extern volatile uint32_t impedance_watchdog_ms;

// Rolling-impedance helper API shared by CAN handling and the control loop.
void resetImpedanceSegment(uint8_t dof);
bool evaluateImpedanceSegment(uint8_t dof, uint32_t now_ms, float &q_ref_deg, float &dq_ref_deg_s);
float getImpedanceHoldReference(uint8_t dof);
void restoreInnerPidGains(uint8_t dof, JointController *jc);
void restoreOuterLoopParameters(uint8_t dof, JointController *jc);

#endif // MAIN_COMMON_H
