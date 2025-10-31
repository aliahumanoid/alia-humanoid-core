/**
 * @file CommandParser.h
 * @brief Serial command parsing and parameter extraction
 * 
 * This class handles parsing of incoming serial commands and extraction
 * of parameters. It supports:
 * - Command string tokenization (joint_id, dof_index, command_name)
 * - Positional parameter extraction (up to 8 float parameters)
 * - Named parameter extraction (key=value pairs)
 * - Multi-DOF command detection (all_dofs flag)
 * - Population of command_data_extended_t for inter-core communication
 * 
 * COMMAND FORMAT:
 * Standard format: "joint_id dof_index COMMAND_NAME param1 param2 ..."
 * Named params:    "joint_id dof_index COMMAND_NAME angle=90.0 speed=100.0"
 * All DOFs:        "joint_id all COMMAND_NAME ..."
 * 
 * Example: "0 0 MOVE_MULTI_DOF 45.0 90.0 100.0 500.0 1"
 * 
 * WORKFLOW:
 * 1. parseCommand() - Parse raw string into ParsedCommand struct
 * 2. populateCommandData() - Convert ParsedCommand to command_data_extended_t
 * 3. Send command_data to core1 for execution
 */

#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <commands.h>
#include <shared_data.h>
#include <Arduino.h>

/**
 * @brief Parsed command result structure
 * 
 * Contains all components extracted from a command string.
 * Used as an intermediate format before populating command_data_extended_t.
 * 
 * Fields:
 * - joint_id: Target joint identifier (0-255)
 * - dof_index: Target DOF within joint (0-255, or 255 for all DOFs)
 * - command: Numeric command ID (from commands.h)
 * - params[8]: Positional float parameters extracted from command
 * - param_count: Number of valid parameters in params array
 * - all_dofs: True if command targets all DOFs of the joint
 * - original_command: Copy of raw command string for debugging/logging
 */
struct ParsedCommand {
  uint8_t joint_id;           // Target joint ID (0-255)
  uint8_t dof_index;          // Target DOF index (0-255, 255=all)
  uint8_t command;            // Command ID (see commands.h)
  float params[8];            // Positional parameters (max 8)
  uint8_t param_count;        // Number of valid parameters
  bool all_dofs;              // Whether command applies to all DOFs
  char original_command[100]; // Original command string (debug/log)
};

/**
 * @brief Command parsing and parameter extraction class
 * 
 * Stateless parser for serial command strings. Converts text commands
 * into structured data suitable for inter-core communication.
 * 
 * USAGE EXAMPLE:
 * ```
 * CommandParser parser;
 * ParsedCommand parsed;
 * command_data_extended_t cmd_data;
 * 
 * if (parser.parseCommand("0 0 MOVE_MULTI_DOF 45.0 90.0", parsed)) {
 *   parser.populateCommandData(parsed, cmd_data);
 *   // Send cmd_data to core1
 * }
 * ```
 */
class CommandParser {
private:
  /**
   * @brief Find and extract a named parameter from command string
   * 
   * Searches for "param_name=value" pattern in the command string.
   * Used for commands with named parameters (e.g., "angle=90.0").
   * 
   * @param cmd Full command string to search
   * @param param_name Parameter name to find (without '=')
   * @param value Output: extracted float value
   * @return true if parameter found and parsed successfully
   */
  bool findParam(const char *cmd, const char *param_name, float &value);

public:
  /**
   * @brief Constructor - initializes the parser
   * 
   * No configuration needed, parser is stateless.
   */
  CommandParser();

  /**
   * @brief Parse a serial command string into structured components
   * 
   * Extracts joint_id, dof_index, command name, and parameters from
   * a space-separated command string. Handles both positional and
   * named parameters.
   * 
   * Expected format: "joint_id dof_index COMMAND_NAME [params...]"
   * - joint_id: 0-255
   * - dof_index: 0-255 or "all" for all DOFs
   * - COMMAND_NAME: String matching SERIAL_CMD_* from commands.h
   * - params: Up to 8 float values (space-separated)
   * 
   * @param cmd Input command string (null-terminated)
   * @param result Output: parsed command structure
   * @return true if parsing successful, false on error
   */
  bool parseCommand(const char *cmd, ParsedCommand &result);

  /**
   * @brief Convert ParsedCommand to command_data_extended_t
   * 
   * Populates the inter-core communication structure with data from
   * a parsed command. Maps generic parameters to command-specific
   * fields based on command type.
   * 
   * This method knows the parameter order for each command type:
   * - MOVE_MULTI_DOF: target_angles[], speed, accel, sync_strategy
   * - AUTO_MAP: min_angles[], max_angles[], steps[], torque, settle_time
   * - etc.
   * 
   * @param result Parsed command from parseCommand()
   * @param cmd_data Output: populated command data for core1
   */
  void populateCommandData(const ParsedCommand &result, command_data_extended_t &cmd_data);
};

#endif // COMMAND_PARSER_H
