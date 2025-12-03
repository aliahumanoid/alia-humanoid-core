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

// Time offset for synchronization
extern float time_offset;

// Encoder test flags
extern bool encoder_test_active;
extern uint8_t encoder_test_joint_id;
extern uint8_t encoder_test_dof_index;
extern bool encoder_test_all_dofs;
extern unsigned long last_encoder_test_time;

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

