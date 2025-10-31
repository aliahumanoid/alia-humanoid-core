/**
 * @file JointConfig.h
 * @brief Joint configuration structures and default parameters
 * 
 * This file defines the complete configuration structure for multi-DOF joints.
 * Each joint can have:
 * - Multiple degrees of freedom (DOF) - up to MAX_DOFS
 * - Multiple motors - up to MAX_MOTORS (2 motors per DOF in agonist-antagonist pairs)
 * 
 * STRUCTURE HIERARCHY:
 * JointConfig (top level)
 *   ├─ DofConfig[MAX_DOFS]
 *   │   ├─ MotionParams (speed, acceleration, path planning)
 *   │   ├─ AngleLimits (min/max joint angles)
 *   │   └─ ZeroMappingParams (calibration parameters)
 *   └─ MotorConfig[MAX_MOTORS]
 *       └─ MotorPIDParams (motor control gains with filtering)
 * 
 * DEFAULT VALUES:
 * - Inner loop PID: Used for motor torque control
 * - Outer loop PID: Used for position control cascade
 * - All defaults are conservative and should be tuned per joint type
 */

#ifndef JOINT_CONFIG_H
#define JOINT_CONFIG_H

#include <Arduino.h>

// ============================================================================
// DEFAULT PID PARAMETERS
// ============================================================================

// Default inner loop PID values (motor torque control)
// These control the motor's torque output directly
constexpr float PID_DEFAULT_INNER_KP = 10.0f;
constexpr float PID_DEFAULT_INNER_KI = 1.0f;
constexpr float PID_DEFAULT_INNER_KD = 0.25f;

// Default outer loop PID values (position control)
// These control the joint position using the cascade architecture
constexpr float PID_DEFAULT_OUTER_KP = 1.0f;
constexpr float PID_DEFAULT_OUTER_KI = 0.1f;
constexpr float PID_DEFAULT_OUTER_KD = 0.05f;

// Default tendon control parameters
constexpr float PID_DEFAULT_STIFFNESS_DEG = 1.0f;  // Tendon pretension reference (degrees)
constexpr float PID_DEFAULT_CASCADE       = 0.25f; // Cascade influence factor (0.0-1.0)

// ============================================================================
// SYSTEM LIMITS
// ============================================================================

// Note: MAX_MOTORS and MAX_DOFS are also defined in global.h
// These local definitions ensure this header is self-contained
#ifndef MAX_MOTORS
#define MAX_MOTORS 6       // Maximum motors per joint (hip has 6)
#endif

#ifndef MAX_DOFS
#define MAX_DOFS 3         // Maximum DOFs per joint (hip has 3)
#endif

#define DEFAULT_JOINT_ID 0 // Default joint ID when not specified

// ============================================================================
// CONFIGURATION STRUCTURES
// ============================================================================

/**
 * @brief Motor PID parameters with filtering (4-term)
 * 
 * Extended PID parameters for motor control including low-pass filtering.
 * This extends the basic PIDGains structure (from common_types.h) with
 * an additional tau parameter for derivative filtering.
 * 
 * Used in: Motor inner control loop for torque output
 */
struct MotorPIDParams {
  float kp;  // Proportional gain
  float ki;  // Integral gain
  float kd;  // Derivative gain
  float tau; // Low-pass filter time constant for derivative term (seconds)
};

/**
 * @brief Motor configuration for a single motor
 * 
 * Each DOF has 2 motors in agonist-antagonist configuration.
 * The agonist motor pulls in the positive direction of the DOF,
 * while the antagonist provides counter-tension.
 */
struct MotorConfig {
  uint8_t id;           // Motor CAN ID for communication
  uint8_t dof_index;    // Which DOF this motor contributes to (0, 1, or 2)
  char name[16];        // Human-readable name (e.g., "extensor", "flexor")
  bool invert;          // Direction inversion flag
  bool is_agonist;      // true = agonist (primary), false = antagonist
  float max_torque;     // Maximum allowed torque (mNm)
  float reduction_gear; // Gear reduction ratio (e.g., 10.0 means 10:1)
  MotorPIDParams pid;   // Inner loop PID gains for this motor
};

/**
 * @brief DOF motion parameters for trajectory planning
 * 
 * Controls how the joint moves between positions.
 */
struct MotionParams {
  int path_steps;                         // Number of steps for path discretization
  float max_speed;                        // Maximum angular velocity (rad/s)
  float accel_time;                       // Acceleration ramp time (seconds)
  int sampling_period;                    // Control loop sampling period (microseconds)
  float holding_position_error_threshold; // Error threshold (degrees) - motors stop if exceeded
};

/**
 * @brief DOF angle limits for safety
 * 
 * Defines the safe operating range for the joint.
 */
struct AngleLimits {
  float min_angle; // Minimum allowed angle (degrees)
  float max_angle; // Maximum allowed angle (degrees)
};

/**
 * @brief Zero-finding and calibration parameters
 * 
 * Parameters for the automatic calibration procedure that maps
 * motor angles to joint angles. This process establishes the
 * relationship between tendon lengths and joint positions.
 */
struct ZeroMappingParams {
  // Offset recalculation parameters
  int recalc_offset_torque;             // Torque for recalculating zero offsets (mNm)
  int recalc_offset_duration;           // Duration for offset recalculation (milliseconds)
  float zero_angle_offset;              // Angle offset to apply after finding zero (degrees)
  
  // Pretensioning parameters
  float pretension_torque;              // Initial pretension torque (mNm)
  int pretension_timeout;               // Timeout for pretensioning phase (milliseconds)
  
  // Auto-mapping parameters
  float tensioning_torque;              // Constant tension during mapping (mNm)
  float auto_mapping_step;              // Angular step size for mapping (degrees)
  int auto_mapping_settle_time;         // Settling time between steps (milliseconds)
  float auto_mapping_speed;             // Motor speed during mapping (degrees/s)
  float auto_mapping_resistance_torque; // Resistance torque threshold for mapping (mNm)
  float position_threshold;             // Position convergence threshold (degrees)
  
  // Safe mapping range (more conservative than absolute limits)
  float auto_mapping_min_angle;         // Minimum angle for auto-mapping (degrees)
  float auto_mapping_max_angle;         // Maximum angle for auto-mapping (degrees)
};

/**
 * @brief Complete configuration for a single degree of freedom
 * 
 * Combines all parameters needed to control one DOF:
 * motion planning, safety limits, and calibration settings.
 */
struct DofConfig {
  char name[32];                  // Human-readable DOF name (e.g., "flexion_extension")
  uint8_t encoder_channel;        // Encoder channel index for this DOF
  bool encoder_invert;            // true = invert encoder readings
  MotionParams motion;            // Motion and trajectory parameters
  AngleLimits limits;             // Safety angle limits
  ZeroMappingParams zero_mapping; // Calibration and zero-finding parameters
};

/**
 * @brief Complete configuration for an entire joint
 * 
 * Top-level structure containing all DOFs and motors for a joint.
 * This is the structure used in config_presets.h to define each joint type.
 * 
 * IMPORTANT: Arrays are sized to MAX_DOFS and MAX_MOTORS, but only
 * the first dof_count and motor_count elements are valid.
 */
struct JointConfig {
  char name[32];                  // Joint name (e.g., "knee_left", "hip_right")
  uint8_t joint_id;               // Unique joint identifier (see config_presets.h)
  uint8_t dof_count;              // Number of active DOFs (1-3)
  uint8_t motor_count;            // Number of active motors (2-6)
  DofConfig dofs[MAX_DOFS];       // Array of DOF configurations
  MotorConfig motors[MAX_MOTORS]; // Array of motor configurations
};

#endif // JOINT_CONFIG_H
