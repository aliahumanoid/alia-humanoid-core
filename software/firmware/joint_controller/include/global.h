/**
 * @file global.h
 * @brief Global definitions and data structures for the joint controller
 * 
 * This file contains:
 * - Hardware pin definitions (SPI, CAN)
 * - Device configuration constants
 * - Size limits for buffers and arrays
 * - Runtime data structures for inter-core communication
 * - Flash storage structures for persistent configuration
 * 
 * FLASH STORAGE VERSIONS:
 * - Version 2: Legacy DeviceData (deprecated, TX only)
 * - Version 4: PIDOnlyDeviceData (motor PID + outer loop parameters)
 * - Version 5: LinearEquationsDeviceData (calibration equations + limits)
 * 
 * MULTI-DOF SUPPORT:
 * - MAX_DOFS = 3 (knee=1, ankle=2, hip=3)
 * - MAX_MOTORS = 6 (3 DOF × 2 motors per DOF)
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <commands.h> // Command definitions
#include <common_types.h>
#include <array>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// SPI0 PICO MASTER -> PICO SLAVE
// Wiring:
// Master RX  GP16 <-> GP19  Slave TX
// Master CS  GP17 <-> GP17  Slave CS
// Master CK  GP18 <-> GP18  Slave CK
// Master TX  GP19 <-> GP16  Slave RX
const int PIN_RX  = 16;
const int PIN_CS  = 17;
const int PIN_SCK = 18;
const int PIN_TX  = 19;

// CAN interface pins - Motor CAN (J4 CAN_Servo)
#define CAN_CS_PIN 9
#define CAN_INT_PIN 13

// CAN interface pins - Host CAN (J5 CAN_Controller)
#define CAN_HOST_CS_PIN 8
#define CAN_HOST_INT_PIN 14

// ============================================================================
// DEVICE CONFIGURATION
// ============================================================================

// Joint encoder configuration
#define JOINTENCODERINDEX 0 // Joint encoder index
#define JOINTFLIP false     // Flip joint direction

// Flash storage identification
#define MAGIC_NUMBER 0xABCD1234
#define FLASH_STRUCT_VERSION 2 // Legacy version (use version 4 or 5 for new saves)

// ============================================================================
// SIZE LIMITS & CONSTANTS
// ============================================================================

// Note: All SERIAL_* and CMD_* command definitions live in commands.h

// Motion control limits
#define MAX_POINTS 1000 // Maximum points received from host for motion tracking
#define MAX_STEPS 2000  // Maximum steps for a single movement

// Buffer sizes
#define MAX_MESSAGE_DATA_SIZE 500 // Maximum size of message buffer

// Multi-DOF system limits
#ifndef MAX_DOFS
#define MAX_DOFS 3 // Maximum supported DOFs (hip has 3)
#endif

#ifndef MAX_MOTORS
#define MAX_MOTORS 6 // Maximum supported motors (3 DOF × 2 motors per DOF)
#endif

// Calibration data limits
#ifndef MAX_MAPPING_DATA_SIZE
#define MAX_MAPPING_DATA_SIZE 100 // WARNING: values >100 exceed available flash space — may crash
#endif

#ifndef MAX_MOVEMENT_DATA_SIZE
#define MAX_MOVEMENT_DATA_SIZE 1500 // Maximum size of arrays saved during movement
#endif

// Control thresholds
#define MAX_ANGLE_DIFFERENCE 15.0f // Maximum allowed angle difference
#define EPSILON 0.2f               // Tolerance for floating-point comparisons

// ============================================================================
// RUNTIME DATA STRUCTURES
// ============================================================================

/**
 * @brief Shared data structure for inter-core communication
 * 
 * Used to exchange encoder data and messages between cores.
 */
struct shared_data_t {
  volatile int flag;               // Synchronization flag
  MultiAngleData encoder_data_ext; // Extensor encoder data
  MultiAngleData encoder_data_flex;// Flexor encoder data
  char message[MAX_MESSAGE_DATA_SIZE]; // Text message buffer
};

/**
 * @brief Command data for movement instructions
 */
struct command_data_t {
  float angle;        // Target angle in degrees
  float speed;        // Movement speed
  float acceleration; // Acceleration rate
  int path_type;      // Path type (linear, cubic, etc.)
};

/**
 * @brief Measuring data for sensor readouts
 */
struct measuring_data_t {
  volatile int flag;           // Synchronization flag
  float extensor_data_output;  // Extensor measurement
  float flexor_data_output;    // Flexor measurement
};

/**
 * @brief Movement data arrays for trajectory recording
 * 
 * Stores time-series data during movement execution for analysis and debugging.
 */
struct movement_data_t {
  std::array<float, MAX_MOVEMENT_DATA_SIZE> alpha_next;   // Next alpha angles
  std::array<float, MAX_MOVEMENT_DATA_SIZE> gamma_curr;   // Current gamma angles
  std::array<float, MAX_MOVEMENT_DATA_SIZE> theta_curr;   // Current theta angles
  std::array<float, MAX_MOVEMENT_DATA_SIZE> gamma_next;   // Next gamma angles
  std::array<float, MAX_MOVEMENT_DATA_SIZE> theta_next;   // Next theta angles
  std::array<float, MAX_MOVEMENT_DATA_SIZE> cycle_time;   // Cycle timestamps
  std::array<float, MAX_MOVEMENT_DATA_SIZE> gamma_torque; // Gamma motor torques
  std::array<float, MAX_MOVEMENT_DATA_SIZE> theta_torque; // Theta motor torques
  int size; // Number of valid data points
};


// ============================================================================
// FLASH STORAGE STRUCTURES
// ============================================================================

/**
 * @brief Flash structure for PID parameters (Version 4)
 * 
 * Stores motor PID gains and outer loop control parameters.
 * This is the current version used for saving/loading PID configuration.
 */
struct PIDOnlyDeviceData {
  uint32_t magic_number;               // Magic number (0xABCD1234) for validation
  uint16_t version;                    // Structure version (4)
  uint16_t checksum;                   // Data integrity checksum
  uint8_t joint_type;                  // Joint type: JOINT_KNEE, JOINT_ANKLE, JOINT_HIP
  uint8_t dof_count;                   // Number of degrees of freedom
  uint8_t motor_count;                  // Number of motors
  uint8_t reserved;                     // Padding for alignment
  struct PIDGains pid_data[MAX_MOTORS]; // Inner loop PID for each motor
  float outer_loop_kp[MAX_DOFS];       // Outer loop proportional gain per DOF
  float outer_loop_ki[MAX_DOFS];       // Outer loop integral gain per DOF
  float outer_loop_kd[MAX_DOFS];       // Outer loop derivative gain per DOF
  float stiffness_ref_deg[MAX_DOFS];   // Tendon pretension reference in degrees per DOF
  float cascade_influence[MAX_DOFS];   // Cascade influence factor (0.0–1.0) per DOF
  uint32_t timestamp;                  // Unix timestamp of last save
};

/**
 * @brief Legacy mapping data structure (deprecated)
 * 
 * Used only for transmission (TX) of old calibration data.
 * Not used for new saves.
 */
struct MultiDofMappingData_t {
  uint8_t dof_count;   // Number of calibrated DOFs
  uint8_t reserved[3]; // Alignment padding
  struct {
    float agonist_data[MAX_MAPPING_DATA_SIZE];    // Agonist motor angle data
    float antagonist_data[MAX_MAPPING_DATA_SIZE]; // Antagonist motor angle data
    float joint_data[MAX_MAPPING_DATA_SIZE];      // Joint angle data
    int size;            // Number of valid data points
    uint8_t flag;        // Calibration status flag
    uint8_t reserved[3]; // Padding for alignment
  } dof_data[MAX_DOFS];  // Data for up to 3 DOFs
};

/**
 * @brief Legacy device data structure (Version 2) - DEPRECATED
 * 
 * This structure is kept only for backward compatibility with old firmware.
 * DO NOT USE for new saves - use PIDOnlyDeviceData (v4) or 
 * LinearEquationsDeviceData (v5) instead.
 */
struct DeviceData {
  uint32_t magic_number;               // Magic number (0xABCD1234) for validation
  uint16_t version;                    // Structure version (2)
  uint16_t checksum;                   // Data integrity checksum
  uint8_t joint_type;                  // Joint type: JOINT_KNEE, JOINT_ANKLE, JOINT_HIP
  uint8_t dof_count;                    // Number of degrees of freedom
  uint8_t reserved[2];                  // Padding for alignment
  struct PIDGains pid_data[MAX_MOTORS]; // PID parameters for each motor
  struct MultiDofMappingData_t mapping_data; // Legacy mapping data
};

/**
 * @brief Linear equation coefficients for motor-joint mapping
 * 
 * Represents the linear relationship: motor_angle = slope × joint_angle + intercept
 */
struct LinearEquationData {
  float slope;         // Slope (angular coefficient m)
  float intercept;     // Y-intercept (b)
  float r_squared;     // Coefficient of determination (R²) - fit quality
  float mse;           // Mean squared error
  uint8_t valid;       // Validity flag: 0 = invalid, 1 = valid
  uint8_t reserved[3]; // Padding for alignment
};

/**
 * @brief Flash structure for calibration equations (Version 5)
 * 
 * Stores linear equations mapping joint angles to motor angles,
 * along with safe operating limits for each DOF.
 * This is the current version used for saving/loading calibration data.
 */
struct LinearEquationsDeviceData {
  uint32_t magic_number; // Magic number (0xABCD1234) for validation
  uint16_t version;      // Structure version (5)
  uint16_t checksum;     // Data integrity checksum
  uint8_t joint_type;    // Joint type: JOINT_KNEE, JOINT_ANKLE, JOINT_HIP
  uint8_t dof_count;     // Number of degrees of freedom
  uint8_t motor_count;   // Number of motors (for verification)
  uint8_t reserved;      // Padding for alignment
  struct {
    struct LinearEquationData agonist;    // Equation for agonist motor
    struct LinearEquationData antagonist; // Equation for antagonist motor
    float joint_safe_min;                 // Safe minimum joint angle (degrees)
    float joint_safe_max;                 // Safe maximum joint angle (degrees)
    float agonist_safe_min;               // Safe minimum agonist angle (degrees)
    float agonist_safe_max;               // Safe maximum agonist angle (degrees)
    float antagonist_safe_min;            // Safe minimum antagonist angle (degrees)
    float antagonist_safe_max;            // Safe maximum antagonist angle (degrees)
    uint8_t calculated;                   // Calculation status: 0 = not calculated, 1 = calculated
    uint8_t reserved_bytes[3];            // Padding for alignment
  } dof_equations[MAX_DOFS];              // Equations for each DOF
  uint32_t timestamp;                     // Unix timestamp of last save
};

#endif // GLOBAL_H
