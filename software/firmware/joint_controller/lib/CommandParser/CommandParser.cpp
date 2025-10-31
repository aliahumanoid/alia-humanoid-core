/**
 * @file CommandParser.cpp
 * @brief Implementation of serial command parsing and parameter extraction
 * 
 * See CommandParser.h for detailed usage documentation.
 */

#include "CommandParser.h"
#include <string.h>

// ===================================================================
// CONSTRUCTOR
// ===================================================================

CommandParser::CommandParser() {
  // Stateless parser - no initialization needed
}

// ===================================================================
// PRIVATE METHODS
// ===================================================================

/**
 * Find a named parameter in the command string (format: "PARAM_NAME=value")
 */
bool CommandParser::findParam(const char *cmd, const char *param_name, float &value) {
  // Build search pattern: "param_name="
  char param_prefix[32];
  sprintf(param_prefix, "%s=", param_name);

  // Search for the pattern in the command string
  const char *param_start = strstr(cmd, param_prefix);
  if (param_start == NULL) {
    return false; // Parameter not found
  }

  // Advance past the "param_name=" prefix
  param_start += strlen(param_prefix);

  // Convert the value string to float
  char *end_ptr;
  value = strtof(param_start, &end_ptr);

  // Verify conversion was successful (end_ptr moved forward)
  return end_ptr != param_start;
}

// ===================================================================
// PUBLIC METHODS - PARSING
// ===================================================================

/**
 * Parse command string into structured components
 * 
 * Format: "JOINT:DOF:COMMAND:PARAM1:PARAM2:..."
 * Example: "ankle:0:MOVE_MULTI_DOF:45.0:90.0:30.0:7:1:100.0:500.0:0"
 */
bool CommandParser::parseCommand(const char *cmd, ParsedCommand &result) {
  // Initialize result structure
  memset(&result, 0, sizeof(ParsedCommand));
  strncpy(result.original_command, cmd, sizeof(result.original_command) - 1);

  // Temporary buffers for string tokens
  char joint_str[16]   = {0};
  char dof_str[16]     = {0};
  char command_str[32] = {0};

  // ---------------------------------------------------------------
  // Step 1: Extract JOINT identifier
  // ---------------------------------------------------------------
  char *token = strtok((char *)cmd, ":");
  if (token == NULL) {
    return false; // Malformed command
  }
  strncpy(joint_str, token, sizeof(joint_str) - 1);

  // ---------------------------------------------------------------
  // Step 2: Extract DOF index (or "ALL")
  // ---------------------------------------------------------------
  token = strtok(NULL, ":");
  if (token == NULL) {
    return false; // Malformed command
  }
  strncpy(dof_str, token, sizeof(dof_str) - 1);

  // ---------------------------------------------------------------
  // Step 3: Extract COMMAND name
  // ---------------------------------------------------------------
  token = strtok(NULL, ":");
  if (token == NULL) {
    return false; // Malformed command
  }
  strncpy(command_str, token, sizeof(command_str) - 1);

  // ---------------------------------------------------------------
  // Step 4: Map joint name to numeric ID
  // ---------------------------------------------------------------
  result.joint_id = getJointId(joint_str);
  if (result.joint_id == JOINT_NONE) {
    return false; // Unknown joint
  }

  // ---------------------------------------------------------------
  // Step 5: Parse DOF (numeric index or "ALL")
  // ---------------------------------------------------------------
  if (strcmp(dof_str, SERIAL_DOF_ALL) == 0) {
    result.all_dofs  = true;
    result.dof_index = 0; // Default to first DOF when "ALL" specified
  } else {
    result.all_dofs  = false;
    result.dof_index = atoi(dof_str);
  }

  // ---------------------------------------------------------------
  // Step 6: Map command name to numeric ID
  // ---------------------------------------------------------------
  result.command = getCommandId(command_str);

  // ---------------------------------------------------------------
  // Step 7: Extract positional parameters (up to 8)
  // ---------------------------------------------------------------
  token = strtok(NULL, ":");
  while (token != NULL && result.param_count < 8) {
    // Check if this token is a named parameter (contains '=')
    char *equals_sign = strchr(token, '=');
    if (equals_sign != NULL) {
      // Named parameter - skip here, handled separately below
    } else {
      // Positional parameter - convert to float and store
      result.params[result.param_count++] = atof(token);
    }

    token = strtok(NULL, ":");
  }

  // ---------------------------------------------------------------
  // Step 8: Extract optional named parameters
  // ---------------------------------------------------------------
  // Note: These override positional params if both are provided
  float value;
  if (findParam(cmd, SERIAL_PARAM_SPEED, value)) {
    result.params[result.param_count++] = value;
  }
  if (findParam(cmd, SERIAL_PARAM_ACCEL, value)) {
    result.params[result.param_count++] = value;
  }
  if (findParam(cmd, SERIAL_PARAM_PATH, value)) {
    result.params[result.param_count++] = value;
  }
  if (findParam(cmd, SERIAL_PARAM_TORQUE, value)) {
    result.params[result.param_count++] = value;
  }
  if (findParam(cmd, SERIAL_PARAM_DURATION, value)) {
    result.params[result.param_count++] = value;
  }
  if (findParam(cmd, SERIAL_PARAM_SYNC, value)) {
    result.params[result.param_count++] = value;
  }

  return true;
}

// ===================================================================
// PUBLIC METHODS - DATA POPULATION
// ===================================================================

/**
 * Populate command_data structure from parsed command
 * 
 * Maps generic parameters to command-specific fields based on command type.
 * Each case documents its expected parameter order.
 */
void CommandParser::populateCommandData(const ParsedCommand &result,
                                        command_data_extended_t &cmd_data) {
  // ---------------------------------------------------------------
  // Copy basic command identifiers
  // ---------------------------------------------------------------
  cmd_data.joint_id  = result.joint_id;
  cmd_data.dof_index = result.dof_index;

  // ---------------------------------------------------------------
  // Initialize all fields to safe defaults
  // ---------------------------------------------------------------
  cmd_data.angle                    = 0.0f;
  cmd_data.speed                    = 0.0f;
  cmd_data.acceleration             = 0.0f;
  cmd_data.path_type                = PATH_TRIG;
  cmd_data.torque                   = 0;
  cmd_data.duration                 = 0;
  
  cmd_data.recalc_offset_torque     = 0;
  cmd_data.recalc_offset_duration   = 0;
  cmd_data.tensioning_torque        = 0.0f;
  cmd_data.auto_mapping_settle_time = 0;

  // ---------------------------------------------------------------
  // Populate command-specific fields based on command type
  // ---------------------------------------------------------------
  switch (result.command) {
  case CMD_MOVE_MULTI_DOF:
    // -----------------------------------------------------------
    // Multi-DOF coordinated movement
    // -----------------------------------------------------------
    // Expected format: JOINT:ALL:MOVE_MULTI_DOF:ANGLE0:ANGLE1:ANGLE2:MASK:SYNC:SPEED:ACCEL:PATH
    // 
    // Parameters:
    //   [0-2]: Target angles for DOF 0, 1, 2 (degrees)
    //   [3]:   Active DOFs bitmask (e.g., 7=0b111 = all 3 DOFs, 5=0b101 = DOF 0&2)
    //   [4]:   Sync strategy (0=none, 1=duration, 2=velocity)
    //   [5]:   Max speed (degrees/sec)
    //   [6]:   Acceleration time (ms)
    //   [7]:   Path type (0=linear, 1=trig)
    
    cmd_data.active_dofs_mask = 0; // Will be set below
    cmd_data.sync_strategy    = 0; // Default: no synchronization

    // Extract target angles for each DOF (params 0-2)
    for (int i = 0; i < MAX_DOFS && i < result.param_count; i++) {
      cmd_data.target_angles[i] = result.params[i];
    }

    // Extract active DOFs bitmask (param 3)
    if (result.param_count > 3) {
      cmd_data.active_dofs_mask = (uint8_t)result.params[3];
    } else {
      // Default: enable all DOFs if mask not specified
      for (int i = 0; i < MAX_DOFS; i++) {
        cmd_data.active_dofs_mask |= (1 << i);
      }
    }

    // Extract synchronization strategy (param 4)
    if (result.param_count > 4) {
      cmd_data.sync_strategy = (int)result.params[4];
    }

    // Extract max speed (param 5)
    if (result.param_count > 5) {
      cmd_data.speed = result.params[5];
    }

    // Extract acceleration time (param 6)
    if (result.param_count > 6) {
      cmd_data.acceleration = result.params[6];
    }

    // Extract path type (param 7)
    if (result.param_count > 7) {
      cmd_data.path_type = (int)result.params[7];
    }
    break;

  case CMD_PRETENSION:
  case CMD_RELEASE:
    // -----------------------------------------------------------
    // Pretension/Release commands
    // -----------------------------------------------------------
    // Expected format: JOINT:DOF:PRETENSION:TORQUE:DURATION
    // Parameters:
    //   [0]: Torque value
    //   [1]: Duration (ms)
    
    if (result.param_count > 0) {
      cmd_data.torque = (int)result.params[0];
    }
    if (result.param_count > 1) {
      cmd_data.duration = (int)result.params[1];
    }
    break;

  case CMD_START_AUTO_MAPPING:
    // -----------------------------------------------------------
    // Automatic calibration mapping
    // -----------------------------------------------------------
    // Expected format: JOINT:ALL:START_AUTO_MAPPING:TORQUE:STEP0:STEP1:STEP2:SETTLE_TIME
    // Parameters:
    //   [0]:   Tensioning torque
    //   [1-3]: Step size per DOF (degrees)
    //   [4]:   Settle time (ms)
    
    if (result.param_count > 0) {
      cmd_data.tensioning_torque = result.params[0];
    }
    // Steps per DOF
    if (result.param_count > 1) {
      cmd_data.auto_mapping_steps[0] = result.params[1];
    }
    if (result.param_count > 2) {
      cmd_data.auto_mapping_steps[1] = result.params[2];
    }
    if (result.param_count > 3) {
      cmd_data.auto_mapping_steps[2] = result.params[3];
    }
    if (result.param_count > 4) {
      cmd_data.auto_mapping_settle_time = (int)result.params[4];
    }
    break;

  case CMD_RECALC_OFFSET:
    // -----------------------------------------------------------
    // Recalculate motor offsets
    // -----------------------------------------------------------
    // Expected format: JOINT:DOF:RECALC_OFFSET:TORQUE:DURATION
    // Parameters:
    //   [0]: Torque for recalculation
    //   [1]: Duration (ms)
    
    if (result.param_count > 0) {
      cmd_data.recalc_offset_torque = (int)result.params[0];
    }
    if (result.param_count > 1) {
      cmd_data.recalc_offset_duration = (int)result.params[1];
    }
    break;
  }

  // ---------------------------------------------------------------
  // Override with optional named parameters if present
  // ---------------------------------------------------------------
  // Named parameters take precedence over positional ones.
  // This allows flexible command syntax like:
  // "JOINT:DOF:COMMAND:angle=90.0:speed=100.0"
  
  float value;
  if (findParam(result.original_command, SERIAL_PARAM_SPEED, value)) {
    cmd_data.speed = value;
  }
  if (findParam(result.original_command, SERIAL_PARAM_ACCEL, value)) {
    cmd_data.acceleration = value;
  }
  if (findParam(result.original_command, SERIAL_PARAM_PATH, value)) {
    cmd_data.path_type = (int)value;
  }
  if (findParam(result.original_command, SERIAL_PARAM_TORQUE, value)) {
    cmd_data.torque = (int)value;
  }
  if (findParam(result.original_command, SERIAL_PARAM_DURATION, value)) {
    cmd_data.duration = (int)value;
  }

  // Named parameters for motor offset recalculation
  if (findParam(result.original_command, SERIAL_PARAM_RECALC_TORQUE, value)) {
    cmd_data.recalc_offset_torque = (int)value;
  }
  if (findParam(result.original_command, SERIAL_PARAM_RECALC_DURATION, value)) {
    cmd_data.recalc_offset_duration = (int)value;
  }
}
