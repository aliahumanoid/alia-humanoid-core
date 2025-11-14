/**
 * @file commands.h
 * @brief Command definitions for multi-joint system
 * 
 * This file defines all command IDs, joint identifiers, and serial protocol
 * constants for the joint controller firmware.
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include <string.h>

// ============================================================================
// COMMAND IDs
// ============================================================================
// Command ID allocation blocks (10 IDs per category):
//   0-9:   System Control
//  10-19:  Motor Control
//  20-29:  Movement
//  30-39:  Calibration & Mapping
//  40-49:  PID Control
//  50-59:  Measurement & Testing
//  255:    Special (UNKNOWN)

// --- System Control Commands (0-9) ---
#ifndef CMD_STOP
#define CMD_STOP 0
#endif

#ifndef CMD_STATUS
#define CMD_STATUS 2
#endif

// Reserved: 3-9 for future system commands

// --- Motor Control Commands (10-19) ---
#ifndef CMD_PRETENSION
#define CMD_PRETENSION 10
#endif

#ifndef CMD_PRETENSION_ALL
#define CMD_PRETENSION_ALL 11
#endif

#ifndef CMD_RELEASE
#define CMD_RELEASE 12
#endif

#ifndef CMD_RELEASE_ALL
#define CMD_RELEASE_ALL 13
#endif

// Reserved: 14-19 for future motor control commands

// --- Movement Commands (20-29) ---
#ifndef CMD_MOVE_MULTI_DOF
#define CMD_MOVE_MULTI_DOF 20  // Coordinated multi-DOF movement
#endif

// Reserved: 21-29 for future movement commands

// --- Calibration & Mapping Commands (30-39) ---
#ifndef CMD_SET_ZERO_CURRENT_POS
#define CMD_SET_ZERO_CURRENT_POS 30
#endif

#ifndef CMD_RECALC_OFFSET
#define CMD_RECALC_OFFSET 31
#endif

#ifndef CMD_START_AUTO_MAPPING
#define CMD_START_AUTO_MAPPING 32
#endif

#ifndef CMD_STOP_AUTO_MAPPING
#define CMD_STOP_AUTO_MAPPING 33
#endif

#ifndef CMD_SAVE_LINEAR_EQUATIONS
#define CMD_SAVE_LINEAR_EQUATIONS 34
#endif

#ifndef CMD_LOAD_LINEAR_EQUATIONS
#define CMD_LOAD_LINEAR_EQUATIONS 35
#endif

// Reserved: 36-39 for future calibration commands

// --- PID Control Commands (40-49) ---
#ifndef CMD_SET_PID
#define CMD_SET_PID 40
#endif

#ifndef CMD_GET_PID
#define CMD_GET_PID 41
#endif

#ifndef CMD_SET_PID_OUTER
#define CMD_SET_PID_OUTER 42
#endif

#ifndef CMD_GET_PID_OUTER
#define CMD_GET_PID_OUTER 43
#endif

#ifndef CMD_SAVE_PID
#define CMD_SAVE_PID 44
#endif

#ifndef CMD_LOAD_PID
#define CMD_LOAD_PID 45
#endif

// Reserved: 46-49 for future PID commands

// --- Measurement & Testing Commands (50-59) ---
#ifndef CMD_START_MEASURING
#define CMD_START_MEASURING 50
#endif

#ifndef CMD_STOP_MEASURING
#define CMD_STOP_MEASURING 51
#endif

#ifndef CMD_START_TEST_ENCODER
#define CMD_START_TEST_ENCODER 52
#endif

#ifndef CMD_STOP_TEST_ENCODER
#define CMD_STOP_TEST_ENCODER 53
#endif

#ifndef CMD_GET_MOVEMENT_DATA
#define CMD_GET_MOVEMENT_DATA 54
#endif

// Reserved: 55-59 for future measurement commands

// --- Special Commands ---
#ifndef CMD_UNKNOWN
#define CMD_UNKNOWN 255
#endif

// ============================================================================
// JOINT IDs
// ============================================================================

#define JOINT_NONE 0

// Knees
#define JOINT_KNEE_LEFT 1
#define JOINT_KNEE_RIGHT 2

// Ankles
#define JOINT_ANKLE_LEFT 3
#define JOINT_ANKLE_RIGHT 4

// Hips
#define JOINT_HIP_LEFT 5
#define JOINT_HIP_RIGHT 6

// ============================================================================
// MOVEMENT PARAMETERS
// ============================================================================

// Path types for trajectory generation
#ifndef PATH_LINEAR
#define PATH_LINEAR 0
#endif

#ifndef PATH_TRIG
#define PATH_TRIG 1
#endif

#ifndef PATH_QUAD
#define PATH_QUAD 2
#endif

// Synchronization strategies for multi-DOF moves
#define SYNC_NONE 0       // No synchronization
#define SYNC_DURATION 1   // Synchronize by duration
#define SYNC_VELOCITY 2   // Synchronize by velocity

// ============================================================================
// SERIAL PROTOCOL DEFINITIONS
// ============================================================================

// --- Joint Identifiers ---
#define SERIAL_JOINT_KNEE_LEFT "KNEE_LEFT"
#define SERIAL_JOINT_KNEE_RIGHT "KNEE_RIGHT"
#define SERIAL_JOINT_ANKLE_LEFT "ANKLE_LEFT"
#define SERIAL_JOINT_ANKLE_RIGHT "ANKLE_RIGHT"
#define SERIAL_JOINT_HIP_LEFT "HIP_LEFT"
#define SERIAL_JOINT_HIP_RIGHT "HIP_RIGHT"

// Special DOF token
#define SERIAL_DOF_ALL "ALL"

// --- Command Strings (System Control) ---
#define SERIAL_CMD_STOP "STOP"
#define SERIAL_CMD_STATUS "STATUS"
#define SERIAL_CMD_CONFIG "CONFIG"

// --- Command Strings (Motor Control) ---
#define SERIAL_CMD_PRETENSION "PRETENSION"
#define SERIAL_CMD_PRETENSION_ALL "PRETENSION_ALL"
#define SERIAL_CMD_RELEASE "RELEASE"
#define SERIAL_CMD_RELEASE_ALL "RELEASE_ALL"

// --- Command Strings (Movement) ---
#define SERIAL_CMD_MOVE_MULTI_DOF "MOVE_MULTI_DOF"

// --- Command Strings (Calibration & Mapping) ---
#define SERIAL_CMD_SET_ZERO_CURRENT_POS "SET_ZERO_CURRENT_POS"
#define SERIAL_CMD_RECALC_OFFSET "RECALC_OFFSET"
#define SERIAL_CMD_START_AUTO_MAPPING "START_AUTO_MAPPING"
#define SERIAL_CMD_STOP_AUTO_MAPPING "STOP_AUTO_MAPPING"
#define SERIAL_CMD_SAVE_LINEAR_EQUATIONS "SAVE_LINEAR_EQUATIONS"
#define SERIAL_CMD_LOAD_LINEAR_EQUATIONS "LOAD_LINEAR_EQUATIONS"

// --- Command Strings (PID Control) ---
#define SERIAL_CMD_SET_PID "SET_PID"
#define SERIAL_CMD_GET_PID "GET_PID"
#define SERIAL_CMD_SET_PID_OUTER "SET_PID_OUTER"
#define SERIAL_CMD_GET_PID_OUTER "GET_PID_OUTER"
#define SERIAL_CMD_SAVE_PID "SAVE_PID"
#define SERIAL_CMD_LOAD_PID "LOAD_PID"

// --- Command Strings (Measurement & Testing) ---
#define SERIAL_CMD_START_MEASURE "START_MEASURE"
#define SERIAL_CMD_STOP_MEASURE "STOP_MEASURE"
#define SERIAL_CMD_START_TEST_ENCODER "START_TEST_ENCODER"
#define SERIAL_CMD_STOP_TEST_ENCODER "STOP_TEST_ENCODER"
#define SERIAL_CMD_GET_MOVEMENT_DATA "GET_MOVEMENT_DATA"

// ============================================================================
// PARAMETER NAMES
// ============================================================================

// --- Movement Parameters ---
#define SERIAL_PARAM_SPEED "SPEED"
#define SERIAL_PARAM_ACCEL "ACCEL"
#define SERIAL_PARAM_PATH "PATH"
#define SERIAL_PARAM_TORQUE "TORQUE"
#define SERIAL_PARAM_DURATION "DURATION"
#define SERIAL_PARAM_SYNC "SYNC"

// --- PID Parameters ---
#define SERIAL_PARAM_PID_MOTOR "MOTOR"  // 1=agonist, 2=antagonist
#define SERIAL_PARAM_PID_KP "KP"        // Proportional
#define SERIAL_PARAM_PID_KI "KI"        // Integral
#define SERIAL_PARAM_PID_KD "KD"        // Derivative
#define SERIAL_PARAM_PID_TAU "TAU"      // Time constant

// --- Calibration Parameters ---
#define SERIAL_PARAM_RECALC_TORQUE "RECALC_TORQUE"
#define SERIAL_PARAM_RECALC_DURATION "RECALC_DURATION"

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Map joint name string to joint ID
 * @param joint_name String identifier (e.g., "KNEE_LEFT")
 * @return Joint ID constant or JOINT_NONE if not found
 */
inline uint8_t getJointId(const char *joint_name) {
  // Knees
  if (strcmp(joint_name, SERIAL_JOINT_KNEE_LEFT) == 0) {
    return JOINT_KNEE_LEFT;
  } else if (strcmp(joint_name, SERIAL_JOINT_KNEE_RIGHT) == 0) {
    return JOINT_KNEE_RIGHT;
  }
  // Ankles
  else if (strcmp(joint_name, SERIAL_JOINT_ANKLE_LEFT) == 0) {
    return JOINT_ANKLE_LEFT;
  } else if (strcmp(joint_name, SERIAL_JOINT_ANKLE_RIGHT) == 0) {
    return JOINT_ANKLE_RIGHT;
  }
  // Hips
  else if (strcmp(joint_name, SERIAL_JOINT_HIP_LEFT) == 0) {
    return JOINT_HIP_LEFT;
  } else if (strcmp(joint_name, SERIAL_JOINT_HIP_RIGHT) == 0) {
    return JOINT_HIP_RIGHT;
  }

  return JOINT_NONE;
}

/**
 * @brief Map command name string to command ID
 * @param cmd_name String identifier (e.g., "STOP", "MOVE_MULTI_DOF")
 * @return Command ID constant or CMD_UNKNOWN if not found
 */
inline uint8_t getCommandId(const char *cmd_name) {
  // System Control
  if (strcmp(cmd_name, SERIAL_CMD_STOP) == 0) {
    return CMD_STOP;
  } else if (strcmp(cmd_name, SERIAL_CMD_STATUS) == 0) {
    return CMD_STATUS;
  }
  // Motor Control
  else if (strcmp(cmd_name, SERIAL_CMD_PRETENSION) == 0) {
    return CMD_PRETENSION;
  } else if (strcmp(cmd_name, SERIAL_CMD_PRETENSION_ALL) == 0) {
    return CMD_PRETENSION_ALL;
  } else if (strcmp(cmd_name, SERIAL_CMD_RELEASE) == 0) {
    return CMD_RELEASE;
  } else if (strcmp(cmd_name, SERIAL_CMD_RELEASE_ALL) == 0) {
    return CMD_RELEASE_ALL;
  }
  // Movement
  else if (strcmp(cmd_name, SERIAL_CMD_MOVE_MULTI_DOF) == 0) {
    return CMD_MOVE_MULTI_DOF;
  }
  // Calibration & Mapping
  else if (strcmp(cmd_name, SERIAL_CMD_SET_ZERO_CURRENT_POS) == 0) {
    return CMD_SET_ZERO_CURRENT_POS;
  } else if (strcmp(cmd_name, SERIAL_CMD_RECALC_OFFSET) == 0) {
    return CMD_RECALC_OFFSET;
  } else if (strcmp(cmd_name, SERIAL_CMD_START_AUTO_MAPPING) == 0) {
    return CMD_START_AUTO_MAPPING;
  } else if (strcmp(cmd_name, SERIAL_CMD_STOP_AUTO_MAPPING) == 0) {
    return CMD_STOP_AUTO_MAPPING;
  } else if (strcmp(cmd_name, SERIAL_CMD_SAVE_LINEAR_EQUATIONS) == 0) {
    return CMD_SAVE_LINEAR_EQUATIONS;
  } else if (strcmp(cmd_name, SERIAL_CMD_LOAD_LINEAR_EQUATIONS) == 0) {
    return CMD_LOAD_LINEAR_EQUATIONS;
  }
  // PID Control
  else if (strcmp(cmd_name, SERIAL_CMD_SET_PID) == 0) {
    return CMD_SET_PID;
  } else if (strcmp(cmd_name, SERIAL_CMD_GET_PID) == 0) {
    return CMD_GET_PID;
  } else if (strcmp(cmd_name, SERIAL_CMD_SET_PID_OUTER) == 0) {
    return CMD_SET_PID_OUTER;
  } else if (strcmp(cmd_name, SERIAL_CMD_GET_PID_OUTER) == 0) {
    return CMD_GET_PID_OUTER;
  } else if (strcmp(cmd_name, SERIAL_CMD_SAVE_PID) == 0) {
    return CMD_SAVE_PID;
  } else if (strcmp(cmd_name, SERIAL_CMD_LOAD_PID) == 0) {
    return CMD_LOAD_PID;
  }
  // Measurement & Testing
  else if (strcmp(cmd_name, SERIAL_CMD_START_MEASURE) == 0) {
    return CMD_START_MEASURING;
  } else if (strcmp(cmd_name, SERIAL_CMD_STOP_MEASURE) == 0) {
    return CMD_STOP_MEASURING;
  } else if (strcmp(cmd_name, SERIAL_CMD_START_TEST_ENCODER) == 0) {
    return CMD_START_TEST_ENCODER;
  } else if (strcmp(cmd_name, SERIAL_CMD_STOP_TEST_ENCODER) == 0) {
    return CMD_STOP_TEST_ENCODER;
  } else if (strcmp(cmd_name, SERIAL_CMD_GET_MOVEMENT_DATA) == 0) {
    return CMD_GET_MOVEMENT_DATA;
  }

  return CMD_UNKNOWN;
}

#endif // COMMANDS_H
