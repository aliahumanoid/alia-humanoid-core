/**
 * @file main.cpp
 * @brief Main entry point for joint controller firmware - System initialization only
 *
 * This file contains:
 * - Global variable definitions (shared across cores)
 * - setup() function for system initialization
 *
 * Core execution loops have been separated for better modularity:
 * - Core0 loop (serial communication) is implemented in core0.cpp
 * - Core1 loop (hardware operations) is implemented in core1.cpp
 *
 * @see main_common.h for shared declarations
 * @see core0.cpp for serial communication and command dispatch
 * @see core1.cpp for movement execution and hardware access
 */

#include "main_common.h"

//----------------------------------------------------------------------------
// ACTIVE JOINT CONFIGURATION
//----------------------------------------------------------------------------
// Set the joint type for this PICO board
// Possible values: JOINT_KNEE_LEFT, JOINT_KNEE_RIGHT, JOINT_ANKLE_LEFT,
// JOINT_ANKLE_RIGHT, JOINT_HIP_LEFT, JOINT_HIP_RIGHT
#define ACTIVE_JOINT JOINT_ANKLE_RIGHT

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
const JointConfig &ACTIVE_JOINT_CONFIG = getConfigById(ACTIVE_JOINT);
//----------------------------------------------------------------------------

#pragma region Variables

// init program flag
bool init_prg = true;
// command array
char command[100];
// SERVO CANBUS
MCP_CAN CAN(&SPI1, CAN_CS_PIN);
// JOINT ENCODER board with 3 encoders
Encoders encoder1(PIN_RX, PIN_CS, PIN_SCK, PIN_TX);

// Time offset for synchronization
float time_offset = 0;

// Encoder test flags
bool encoder_test_active             = false;
uint8_t encoder_test_joint_id        = 0;
uint8_t encoder_test_dof_index       = 0;
bool encoder_test_all_dofs           = false; // NEW: test all DOFs
unsigned long last_encoder_test_time = 0;

// Global variable for auto‑mapping state
AutoMappingState_t auto_mapping_state = {0};

// Control flag for sending mapping data
bool auto_mapping_data_ready_to_send = false; // true = auto‑mapping data ready to send to host

// NOTE: movement safety is handled by JointController::isSystemReadyForMovement()
// which checks linear equations availability and calibrated offsets

#pragma endregion

#pragma region SharedData
// Data structures for multi‑joint support
shared_data_extended_t shared_data_ext       = {0};
command_data_extended_t command_data_ext     = {0};
measuring_data_extended_t measuring_data_ext = {0};

// Command parser
CommandParser command_parser;

// Active joint controller
JointController *active_joint_controller = nullptr;

using MovementSample = movement_sample_t;

queue_t movement_sample_queue;
volatile bool movement_sample_stream_active = false;
volatile bool movement_sample_stream_done   = false;
volatile uint8_t movement_sample_joint_id   = 0;
volatile bool movement_sample_overflow      = false;
uint16_t movement_sample_counters[MAX_DOFS] = {0};

// === NEW INTER‑CORE COMMUNICATION SYSTEM ===
// Double‑buffered command passing
command_data_extended_t command_buffer[2] = {0};
volatile int active_buffer                = 0;
volatile bool buffer_ready[2]             = {false, false};
volatile uint8_t pending_command_type     = 0;

// Separate emergency flag for extra safety
volatile bool emergency_stop_requested = false;

// Smooth transition flag
volatile bool smooth_transition_active = false;

// Array of active controllers accessible from core1 (indices 1..6 for joints)
JointController *active_controllers[7] = {nullptr}; // Index 0 not used

// Active controller state for current cycle
uint8_t current_joint_id  = 0;
uint8_t current_dof_index = 0;


#pragma endregion

// ============================================================================
// CORE LOOP IMPLEMENTATIONS
// ============================================================================
// Core0 loop (serial communication) is implemented in core0.cpp
// Core1 loop (hardware operations) is implemented in core1.cpp
// Both are declared in main_common.h and linked automatically by PlatformIO

#pragma region Setup

void setup() {

#pragma region Init Serial_Interface/Motors
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // wait for serial monitor to open:
  delay(5000);

  // Version handshake events
  Serial.println("EVT:FW:VERSION " FW_VERSION);
  Serial.println("EVT:PROTO " PROTO_VERSION);
  Serial.println("EVT:BUILD " BUILD_GIT_SHA " " BUILD_DATE);
  Serial.println("EVT:READY");

  // Configure SPI1 - CANBUS
  SPI1.setRX(12);
  SPI1.setTX(11);
  SPI1.setSCK(10);
  SPI1.begin();
  spi1_lock_init();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize CAN module
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    LOG_INFO("CAN module initialized successfully on SPI1.");
  } else {
    LOG_ERROR("Failed to initialize CAN module on SPI1!");
    LOG_INFO("Continuing without CAN (normal for serial-only testing)");
    // NOTE: Removed while(1) to allow serial-only testing without CAN hardware
  }
  // set the CAN mode to normal
  if (CAN.setMode(MCP_NORMAL) != CAN_OK) {
    LOG_ERROR("Failed to set normal mode.");
  } else {
    LOG_INFO("Normal mode set successfully.");
  }

  // Initialize waypoint buffers for CAN-based control
  waypoint_buffers_init(ACTIVE_JOINT_CONFIG.dof_count);
  LOG_INFO("Waypoint buffers initialized for " + String(ACTIVE_JOINT_CONFIG.dof_count) + " DOFs");

  // Verify CS pin is HIGH (inactive) after initialization
  LOG_INFO("CS pin state: Motor CAN (GP" + String(CAN_CS_PIN) + ")=" + String(digitalRead(CAN_CS_PIN)));

  LOG_INFO("Joint firmware starting!");

  // blink the LED 10 times to signal the start of the program
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  LOG_INFO("Joint firmware started!");

  // Initialize encoders
  encoder1.begin();

#pragma endregion

#pragma region JointControllerInit
  // Print active joint info
  LOG_DEBUG("------------------------------------");
  LOG_DEBUG("Active joint configuration:");
  LOG_DEBUG("ID: " + String(ACTIVE_JOINT));
  LOG_DEBUG("Name: " + String(ACTIVE_JOINT_CONFIG.name));
  LOG_DEBUG("DOF count: " + String(ACTIVE_JOINT_CONFIG.dof_count));
  LOG_DEBUG("Motor count: " + String(ACTIVE_JOINT_CONFIG.motor_count));

  // Print DOF info
  for (int i = 0; i < ACTIVE_JOINT_CONFIG.dof_count; i++) {
    LOG_DEBUG("DOF " + String(i) + ": " + String(ACTIVE_JOINT_CONFIG.dofs[i].name));
    LOG_DEBUG("  Encoder channel: " + String(ACTIVE_JOINT_CONFIG.dofs[i].encoder_channel));
    LOG_DEBUG("  Angle limits: " + String(ACTIVE_JOINT_CONFIG.dofs[i].limits.min_angle) + " / " + String(ACTIVE_JOINT_CONFIG.dofs[i].limits.max_angle));
  }

  // Print motor info
  for (int i = 0; i < ACTIVE_JOINT_CONFIG.motor_count; i++) {
    LOG_DEBUG("Motor " + String(i) + ": ");
    LOG_DEBUG("  ID: " + String(ACTIVE_JOINT_CONFIG.motors[i].id));
    LOG_DEBUG("  DOF: " + String(ACTIVE_JOINT_CONFIG.motors[i].dof_index));
    LOG_DEBUG("  Name: " + String(ACTIVE_JOINT_CONFIG.motors[i].name));
  }
  LOG_DEBUG("------------------------------------");

  // Initialize active joint controller
  active_joint_controller = new JointController(ACTIVE_JOINT_CONFIG, &CAN, &encoder1);
  if (!active_joint_controller->init()) {
    LOG_ERROR("Failed to initialize controller for " + String(ACTIVE_JOINT_CONFIG.name) + "!");
    // Blink LED quickly to signal an error
    for (int i = 0; i < 20; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
  } else {
    DBG_PRINTLN("Controller for " + String(ACTIVE_JOINT_CONFIG.name) +
                " initialized successfully.");
    // Blink LED slowly to signal success
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  // Register controller in global array for core1 access
  active_controllers[ACTIVE_JOINT] = active_joint_controller;

  // Attempt to load PID parameters from flash (safe system)
  LOG_INFO("------------------------------------");
  LOG_INFO("Attempting to load PID parameters from flash...");
  if (active_joint_controller->loadPIDDataFromFlash()) {
    LOG_INFO("PID parameters successfully loaded from flash!");

    // Blink LED to signal successful PID load
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  } else {
    LOG_INFO("No PID parameters found in flash - applying default PID values (kp=" + String(PID_DEFAULT_INNER_KP, 2) + ", ki=" + String(PID_DEFAULT_INNER_KI, 2) + ", kd=" + String(PID_DEFAULT_INNER_KD, 2) + ")");

    // Blink LED (medium) to show default config is used
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
    }
  }

  // Attempt to load linear equations from flash
  LOG_INFO("------------------------------------");
  LOG_INFO("Attempting to load linear equations from flash...");
  if (active_joint_controller->loadLinearEquationsFromFlash()) {
    LOG_INFO("✓ Linear equations successfully loaded from flash!");
    LOG_INFO("✓ System ready for autonomous control without Pi5");
    LOG_INFO("✓ Ultra-compact equations enable precise motion");

    // Blink LED (special pattern) to signal equations loaded
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  } else {
    LOG_INFO("No linear equations found in flash — auto-mapping required");
    LOG_INFO("Equations will be computed and saved automatically after the first auto-mapping");

    // Blink LED to signal missing equations
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // SAFETY: Movement is controlled by isSystemReadyForMovement()
  LOG_INFO("SAFETY: System initialized — movement controlled by linear equations + calibrated offsets");
  LOG_INFO("Mapping data will be sent to the client for visualization/diagnostics only");

  // Blink LED to signal test mode (different pattern)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
  LOG_DEBUG("------------------------------------");

  // Allocate memory for shared data (use maximum number of motors)
  shared_data_ext.motor_data       = new MultiAngleData[ACTIVE_JOINT_CONFIG.motor_count];
  measuring_data_ext.motor_outputs = new float[ACTIVE_JOINT_CONFIG.motor_count];

  queue_init(&movement_sample_queue, sizeof(MovementSample), 512);
  clearMovementSampleQueue();
#pragma endregion
}
#pragma endregion

// ============================================================================
// #pragma region MAIN LOOP
// ============================================================================

/**
 * @brief Arduino framework main loop - Core0 execution
 *
 * This is the entry point for the main loop required by the Arduino framework.
 * It delegates to core0_main_loop() which handles serial communication and
 * command dispatch.
 *
 * NOTE: Arduino framework requires loop() to be in the same file as setup()
 *
 * @see core0.cpp for the actual Core0 loop implementation (core0_main_loop)
 * @see core1.cpp for the Core1 hardware operations loop (core1_loop)
 */
void loop() {
  core0_main_loop();
}

#pragma endregion
