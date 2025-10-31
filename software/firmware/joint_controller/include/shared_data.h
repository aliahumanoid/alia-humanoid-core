/**
 * @file shared_data.h
 * @brief Inter-core communication and runtime data structures
 * 
 * This file defines data structures used for:
 * 1. Communication between RP2350 cores (core0 and core1)
 * 2. Runtime movement and mapping data
 * 3. Auto-calibration state management
 * 4. Movement synchronization and exit codes
 * 
 * STRUCTURE ORGANIZATION:
 * - Size constants for movement/mapping arrays
 * - Movement data structures (trajectories, samples)
 * - Mapping data structures (raw and processed)
 * - Status flags and exit codes
 * - Auto-mapping state and point storage
 * - Inter-core communication structures
 * 
 * Note: Uses MAX_DOFS and MAX_MOTORS from JointConfig.h
 */

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <JointConfig.h>
#include <common_types.h>
#include <Arduino.h>

// ===================================================================
// SIZE CONSTANTS
// ===================================================================

// Maximum array dimensions for movement and mapping data
#ifndef MAX_MOVEMENT_DATA_SIZE
#define MAX_MOVEMENT_DATA_SIZE 1000
#endif
#ifndef MAX_MAPPING_DATA_SIZE
#define MAX_MAPPING_DATA_SIZE 100
#endif

// ===================================================================
// MOVEMENT DATA STRUCTURES
// ===================================================================

/**
 * @brief Movement trajectory data for a single DOF
 * 
 * Contains arrays of target angles, torques, and timing information
 * for executing coordinated movements. Used by core1 for trajectory execution.
 */
struct DofMovementData_t {
  float alpha_next[MAX_MOVEMENT_DATA_SIZE];
  float gamma_curr[MAX_MOVEMENT_DATA_SIZE];
  float theta_curr[MAX_MOVEMENT_DATA_SIZE];
  float gamma_next[MAX_MOVEMENT_DATA_SIZE];
  float theta_next[MAX_MOVEMENT_DATA_SIZE];
  float cycle_time[MAX_MOVEMENT_DATA_SIZE];
  float gamma_torque[MAX_MOVEMENT_DATA_SIZE];
  float theta_torque[MAX_MOVEMENT_DATA_SIZE];
  int size;
  uint8_t flag;  // Status flag (0=not ready, 1=ready)
};

/**
 * @brief Single movement sample for logging/debugging
 * 
 * Captures a snapshot of joint and motor states during movement execution.
 * Used for trajectory analysis and debugging.
 */
struct movement_sample_t {
  uint8_t dof;
  uint16_t index;
  float joint_target;
  float joint_actual;
  float motor_agonist_curr;
  float motor_antagonist_curr;
  float motor_agonist_ref;
  float motor_antagonist_ref;
  float torque_agonist;
  float torque_antagonist;
};

// ===================================================================
// MAPPING DATA STRUCTURES
// ===================================================================

/**
 * @brief Raw mapping data from auto-calibration (single DOF)
 * 
 * Contains unprocessed motor-joint angle relationships collected during
 * automatic mapping. This data is sent to the host (Pi5) for processing.
 * 
 * Note: agonist = primary motor, antagonist = opposing motor
 * (previously named extensor/flexor in legacy code)
 */
struct DofMappingData_t {
  float agonist_data[MAX_MAPPING_DATA_SIZE];    // Agonist motor angles
  float antagonist_data[MAX_MAPPING_DATA_SIZE]; // Antagonist motor angles
  float joint_data[MAX_MAPPING_DATA_SIZE];      // Joint angles
  int size;                                     // Number of valid points
  uint8_t flag;                                 // Status flag
};

// ===================================================================
// STATUS FLAGS & EXIT CODES
// ===================================================================

/**
 * @brief Status flags for inter-core communication
 * 
 * Used to signal completion or progress of operations from core1 to core0.
 */
#define CMD1_END_ZERO 2           // End of zero search procedure
#define CMD1_END_MOVE 3           // End of movement
#define CMD1_FAIL_MOVE 4          // Movement failure
#define CMD1_AUTO_MAP_PROGRESS 8  // Automatic mapping progress
#define CMD1_AUTO_MAP_COMPLETE 10 // Automatic mapping completed

/**
 * @brief Movement exit codes for smooth command transitions
 * 
 * Determines how motors should behave when a movement ends:
 * - Hold position vs stop motors
 * - Graceful transition vs immediate stop
 */
enum MovementExitCode {
  MOVEMENT_COMPLETED       = 0, // Completed successfully, hold position
  MOVEMENT_STOPPED         = 1, // Stopped by STOP command
  MOVEMENT_TRANSITION      = 2, // Transition to new movement (do NOT stop motors)
  MOVEMENT_CONTROL_COMMAND = 3, // Joint control command (stop motors)
  MOVEMENT_ERROR           = 4, // Error during movement (stop motors)
  MOVEMENT_SAFETY_STOP     = 5  // Safety stop due to limits or encoder (stop motors)
};

/**
 * @brief Movement result structure
 * 
 * Encapsulates the outcome of a movement command, including exit code,
 * debug message, and motor stop decision. Used for centralized movement
 * termination logic.
 */
struct MovementResult {
  MovementExitCode exit_code; // Exit reason
  String message;             // Descriptive message for debug/log
  bool should_stop_motors;    // Whether motors should be stopped

  // Constructors for ease of use
  MovementResult() : exit_code(MOVEMENT_COMPLETED), message(""), should_stop_motors(true) {}

  MovementResult(MovementExitCode code, const String &msg) : exit_code(code), message(msg) {
    // Automatically determine whether to stop motors based on exit code
    should_stop_motors = (code != MOVEMENT_TRANSITION);
  }

  MovementResult(MovementExitCode code, const String &msg, bool stop_motors)
      : exit_code(code), message(msg), should_stop_motors(stop_motors) {}
};

// ===================================================================
// AUTO-MAPPING STRUCTURES
// ===================================================================

/**
 * @brief Single calibration point from auto-mapping
 * 
 * Stores DOF and motor angles at a specific time during automatic calibration.
 */
struct AutoMapPoint_t {
  float dof_angles[MAX_DOFS];     // DOF angles
  float motor_angles[MAX_MOTORS]; // Motor angles
  uint32_t timestamp;             // Acquisition timestamp (ms)
};

/**
 * @brief Auto-mapping state machine and data storage
 * 
 * Comprehensive state tracking for automatic motor-joint calibration.
 * Manages:
 * - Multi-dimensional grid exploration
 * - Position stabilization and settling
 * - Sample acquisition and storage
 * - Torque application and direction control
 * - Error detection and recovery
 * 
 * Used exclusively during auto-mapping operations on core1.
 */
struct AutoMappingState_t {
  bool active;                       // Whether auto mapping is active
  float target_angles[MAX_DOFS];     // Target angles for current position
  float min_angles[MAX_DOFS];        // Min angles per DOF
  float max_angles[MAX_DOFS];        // Max angles per DOF
  float step_sizes[MAX_DOFS];        // Step per DOF
  int dof_count;                     // Joint DOF count
  int motor_count;                   // Joint motor count
  int current_point;                 // Current point index
  int total_points;                  // Total points to acquire
  uint32_t settle_start_time;        // Stabilization phase start time
  int settle_time_ms;                // Stabilization time (ms)
  int stability_count;               // Stability check counter
  bool position_reached;             // Whether target position was reached
  float applied_torques[MAX_MOTORS]; // Applied motor torques
  AutoMapPoint_t last_sample;        // Last acquired sample

  // Array to save all points acquired during mapping
  AutoMapPoint_t acquired_points[MAX_MAPPING_DATA_SIZE]; // All acquired points
  int acquired_points_count;                             // Number of points actually acquired

  // Fields migrated from previous static vars in applyTorquesForTargetPosition
  bool first_call;                         // First‑call flag
  bool dof_movement_initialized[MAX_DOFS]; // Init flag per DOF
  int last_direction[MAX_DOFS];            // Last movement direction per DOF
  float last_agonist_value[MAX_DOFS];      // Last agonist value
  float last_antagonist_value[MAX_DOFS];   // Last antagonist value

  // Arrays for direction control per DOF
  int direction_check_counter[MAX_DOFS]; // Direction check counter
  float previous_angle[MAX_DOFS];        // Previous angle for direction check
  float initial_error[MAX_DOFS];         // Initial error per DOF
  int debug_counter[MAX_DOFS];           // Debug print counter
  bool dof_reach_logged[MAX_DOFS];       // Logged flag for reaching target

  // Error counters and read validity (moved from JointController)
  int consecutive_encoder_errors;   // Consecutive encoder errors counter
  unsigned long last_valid_reading; // Timestamp of last valid reading (ms)
};

// ===================================================================
// INTER-CORE COMMUNICATION
// ===================================================================

/**
 * @brief Extended shared data from core1 to core0
 * 
 * Used to report operation results and status from the control core (core1)
 * to the communication core (core0). Includes message buffer for debug/log.
 */
struct shared_data_extended_t {
  uint8_t flag;               // Status flag (see CMD1_* defines)
  uint8_t joint_id;           // Active joint ID
  uint8_t dof_index;          // Active DOF index
  MultiAngleData *motor_data; // Motor encoder data (runtime-sized array)
  char message[100];          // Info/debug message
};

/**
 * @brief Command data from core0 to core1
 * 
 * Contains parameters for all movement and calibration commands.
 * Sent from the communication core (core0) to the control core (core1).
 * Supports both single-DOF and multi-DOF operations.
 */
struct command_data_extended_t {
  uint8_t joint_id;   // Target joint ID
  uint8_t dof_index;  // Target DOF index
  float angle;        // Target angle
  float speed;        // Max speed
  float acceleration; // Acceleration time
  int path_type;      // Path type
  int torque;         // Pretension torque
  int duration;       // Pretension duration (ms)

  // Multi-DOF movement params
  float target_angles[MAX_DOFS]; // Target angles per DOF (degrees)
  uint8_t active_dofs_mask;      // Active DOFs bitmask (bit 0=DOF0, bit 1=DOF1, ...)
  int sync_strategy;             // Sync strategy (0=none, 1=duration, 2=velocity)

  // Params for recalculateMotorOffsets
  int recalc_offset_torque;   // Torque for offset recalculation
  int recalc_offset_duration; // Duration for offset recalculation (ms)

  // Params for auto-mapping
  float tensioning_torque;            // Tensioning torque for auto-mapping
  float auto_mapping_steps[MAX_DOFS]; // Step size per DOF (degrees)
  int auto_mapping_settle_time;       // Settle time (ms)
};

/**
 * @brief Measurement data sharing structure
 * 
 * Used for diagnostic measurements and motor output logging.
 * Currently used for specialized measurement commands.
 */
struct measuring_data_extended_t {
  uint8_t flag;         // Status flag
  uint8_t joint_id;     // Measured joint ID
  uint8_t dof_index;    // Measured DOF index
  float *motor_outputs; // Motor outputs (runtime-sized array)
};

#endif // SHARED_DATA_H
