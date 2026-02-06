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
// #define ACTIVE_JOINT JOINT_KNEE_RIGHT // Configured in main_common.h

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
// Flash operation synchronization flag
volatile bool flash_operation_in_progress = false;
// Movement in progress flag (Core1 sets during movement, Core0 pauses streaming)
volatile bool movement_in_progress = false;
// Host CAN polling suspend (Core0 sets during startup sequence to avoid SPI conflicts)
volatile bool suspend_host_can_polling = false;
// System settings (loaded from flash at boot)
SystemSettingsData system_settings = {};
bool system_settings_loaded = false;
// Control loop timing (configurable via CAN)
volatile uint16_t inner_loop_period_us = 2000;  // 2000µs = 500Hz (default)
volatile uint8_t outer_loop_divisor = 1;        // 500Hz/1 = 500Hz (default, same as inner)
volatile uint16_t torque_ramp_time_ms = 100;    // Time for 0→max torque (default: 100ms, 0=disabled)
volatile uint16_t encoder_error_threshold_ms = 100;  // Encoder error threshold (default: 100ms)
// CAN error detection: time-window based (more robust to EMI glitches)
volatile uint16_t can_error_window_ms = 50;      // 50ms window (default)
volatile uint8_t can_error_threshold = 5;         // 5 errors in window = emergency stop
// Compliance control (deflection/stall, anti-slack, soft hold)
volatile float expected_velocity_deadband_deg_s = 1.0f;
volatile float hold_error_threshold_deg = 6.0f;
volatile uint16_t hold_time_threshold_ms = 120;
volatile float hold_release_threshold_deg = 2.0f;
volatile float hold_release_velocity_deg_s = 1.0f;
volatile float hold_release_max_velocity_deg_s = 30.0f;  // Max velocity for error-based release
volatile uint16_t hold_release_time_ms = 150;
volatile float move_error_threshold_deg = 8.0f;
volatile float move_velocity_ratio = 0.2f;
volatile uint16_t move_time_threshold_ms = 300;
volatile uint8_t velocity_filter_samples = 5;
volatile float anti_slack_margin_deg = 5.0f;
volatile bool anti_slack_enabled = true;
volatile float soft_hold_torque_ratio = 0.3f;   // 30% torque during compliance
volatile float min_tension_torque = 20.0f;
volatile uint16_t soft_hold_ramp_down_ms = 600;   // Torque release ramp (ms)
volatile uint16_t soft_hold_ramp_up_ms = 800;     // Torque recovery ramp (ms)
volatile bool soft_hold_enabled = true;
volatile ComplianceRecoveryPolicy recovery_policy = RECOVERY_STAY_AT_CURRENT;
volatile uint16_t recovery_ramp_back_ms = 1000;
ComplianceState compliance_state[MAX_DOFS] = {};
// command array
char command[100];
// SERVO CANBUS (J4 CAN_Servo - Motor communication)
MCP_CAN CAN(&SPI1, CAN_CS_PIN);
// HOST CANBUS (J5 CAN_Controller - Host/Jetson communication)
MCP_CAN CAN_HOST(&SPI1, CAN_HOST_CS_PIN);

// NOTE: Direct encoder reading (DirectEncoders) is configured and initialized
// in setup() after determining the number of DOFs for the active joint.
// The global instance 'directEncoders' is defined in DirectEncoders.cpp

// Time offset for synchronization
float time_offset = 0;

// Encoder test flags (Serial - Core0)
bool encoder_test_active             = false;
uint8_t encoder_test_joint_id        = 0;
uint8_t encoder_test_dof_index       = 0;
bool encoder_test_all_dofs           = false; // NEW: test all DOFs
unsigned long last_encoder_test_time = 0;

// Encoder streaming via CAN (high-frequency - Core1)
volatile bool encoder_stream_can_active = false;
volatile uint32_t encoder_stream_last_send_us = 0;

// Joint identification broadcast (triggered via CAN, emitted on Serial)
volatile bool identify_broadcast_active = false;
volatile uint32_t identify_broadcast_start_ms = 0;
const uint32_t IDENTIFY_BROADCAST_DURATION_MS = 3000;  // Broadcast for 3 seconds
const uint32_t IDENTIFY_BROADCAST_INTERVAL_MS = 500;   // Emit every 500ms
volatile uint32_t identify_broadcast_last_emit_ms = 0;

// PID diagnostics for tuning (updated by Core1 waypoint loop)
volatile PIDDiagnostics pid_diagnostics = {0};
volatile bool pid_diag_stream_active = false;

// Cached motor angles (for safety checks without redundant CAN reads)
// This eliminates the ~2ms delay per motor during periodic safety checks
volatile CachedMotorAngles cached_motor_angles = {
    .agonist = {0, 0, 0},
    .antagonist = {0, 0, 0},
    .valid = {false, false, false},
    .last_update_ms = 0
};

// Movement metrics for PID tuning evaluation
MetricsTracker metrics_tracker[3] = {};
MovementMetrics last_movement_metrics[3] = {};
volatile bool metrics_ready[3] = {false, false, false};
volatile bool metrics_tracking_enabled = true;  // Enable by default

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

// === SHARED DOF ANGLES (Updated by Core0, read by Core1) ===
SharedDofAngles shared_dof_angles = {
  .seq = 0,
  .angles = {0},
  .velocities = {0},
  .timestamp_us = 0,
  .valid = {false, false, false},
  .updated = false,
  .dof_count = 0
};

// === NEW INTER‑CORE COMMUNICATION SYSTEM ===
// Double‑buffered command passing
command_data_extended_t command_buffer[2] = {0};
volatile int active_buffer                = 0;
volatile bool buffer_ready[2]             = {false, false};
volatile uint8_t pending_command_type     = 0;

// Separate emergency flag for extra safety (atomic for cross-core access)
std::atomic<bool> emergency_stop_requested{false};

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
  // Wait for USB CDC connection (up to 500ms, exit early if connected)
  // Previously delay(5000) — reduced to avoid blocking hardware init
  {
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 500)) {
      // Yield to USB stack
    }
  }

  // Version handshake events
  SERIAL_COM_LN("EVT:FW:VERSION " FW_VERSION);
  SERIAL_COM_LN("EVT:PROTO " PROTO_VERSION);
  SERIAL_COM_LN("EVT:BUILD " BUILD_GIT_SHA " " BUILD_DATE);
  // Joint identification event (for auto-port selection)
  SERIAL_COM("EVT:JOINT ");
  SERIAL_COM(ACTIVE_JOINT);
  SERIAL_COM(" ");
  SERIAL_COM_LN(ACTIVE_JOINT_CONFIG.name);
  SERIAL_COM_LN("EVT:READY");

  // Start identification broadcast (reuses existing Core0 mechanism)
  // Emits EVT:JOINT every 500ms for 3 seconds, so the host can discover
  // this joint even if it connects after boot
  identify_broadcast_active = true;
  identify_broadcast_start_ms = millis();
  identify_broadcast_last_emit_ms = 0;  // Force immediate first emit

  // Configure SPI1 - CANBUS
  SPI1.setRX(12);
  SPI1.setTX(11);
  SPI1.setSCK(10);
  SPI1.begin();
  // Note: SPI speed is set in mcp_can library (see lib/mcp_can/mcp_can.cpp)

  // Initialize hardware safety system (Rev B: watchdog + power gate)
  // On Rev A: safe no-ops, motor power assumed always on
  safety_init();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Motor CAN module (J4 CAN_Servo)
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    LOG_INFO("Motor CAN (J4) initialized successfully on SPI1, CS=GP" + String(CAN_CS_PIN));
  } else {
    LOG_ERROR("Failed to initialize Motor CAN on SPI1!");
    LOG_INFO("Continuing without Motor CAN (normal for serial-only testing)");
  }
  
  // === STARTUP LOOPBACK TEST ===
  LOG_INFO("=== MCP2515 LOOPBACK TEST AT STARTUP ===");
  if (CAN.setMode(MCP_LOOPBACK) == CAN_OK) {
    LOG_INFO("Loopback mode: ENABLED");
    
    // Send test message
    unsigned char testData[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};
    unsigned long testId = 0x7FF;
    
    byte sendResult = CAN.sendMsgBuf(testId, 0, 8, testData);
    if (sendResult == CAN_OK) {
      LOG_INFO("Loopback TX: OK");
      
      delay(10);
      
      if (CAN.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId;
        unsigned char rxLen;
        unsigned char rxBuf[8];
        
        if (CAN.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
          // Log what we received
          LOG_INFO("RX ID=0x" + String(rxId, HEX) + " Len=" + String(rxLen));
          String rxDataStr = "RX Data: ";
          for (int i = 0; i < rxLen; i++) {
            rxDataStr += String(rxBuf[i], HEX) + " ";
          }
          LOG_INFO(rxDataStr);
          
          bool dataMatch = (rxId == testId) && (rxLen == 8);
          for (int i = 0; i < 8 && dataMatch; i++) {
            if (rxBuf[i] != testData[i]) dataMatch = false;
          }
          
          if (dataMatch) {
            LOG_INFO("✓ LOOPBACK TEST PASSED - SPI + MCP2515 OK!");
          } else {
            LOG_ERROR("✗ LOOPBACK DATA MISMATCH!");
            LOG_ERROR("Expected ID=0x7FF, Data=DE AD BE EF 12 34 56 78");
            // Check if this is a stale message - try reading again
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
              LOG_WARN("More messages in buffer - trying second read...");
              if (CAN.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
                LOG_INFO("2nd RX ID=0x" + String(rxId, HEX) + " Len=" + String(rxLen));
                String rx2Str = "2nd RX Data: ";
                for (int i = 0; i < rxLen; i++) {
                  rx2Str += String(rxBuf[i], HEX) + " ";
                }
                LOG_INFO(rx2Str);
                
                // Check if second message matches
                dataMatch = (rxId == testId) && (rxLen == 8);
                for (int i = 0; i < 8 && dataMatch; i++) {
                  if (rxBuf[i] != testData[i]) dataMatch = false;
                }
                if (dataMatch) {
                  LOG_INFO("✓ 2nd message matched - buffer had stale data");
                }
              }
            }
          }
        } else {
          LOG_ERROR("✗ LOOPBACK RX read failed");
        }
      } else {
        LOG_ERROR("✗ LOOPBACK RX: No message received");
      }
    } else {
      LOG_ERROR("✗ LOOPBACK TX failed (code " + String(sendResult) + ")");
    }
  } else {
    LOG_ERROR("✗ Failed to enter loopback mode - SPI PROBLEM!");
  }
  LOG_INFO("=== END LOOPBACK TEST ===");
  
  // Return to normal mode
  if (CAN.setMode(MCP_NORMAL) != CAN_OK) {
    LOG_ERROR("Failed to set Motor CAN to normal mode.");
  } else {
    LOG_INFO("Motor CAN normal mode set successfully.");
  }

  // Initialize Host CAN module (J5 CAN_Controller)
  if (CAN_HOST.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    LOG_INFO("Host CAN (J5) initialized successfully on SPI1, CS=GP" + String(CAN_HOST_CS_PIN));
    
    // SAFETY: Flush any stale messages from MCP2515 RX buffers
    // This prevents old waypoints from executing after a reset
    int flushed = 0;
    unsigned long flush_start = millis();
    while (CAN_HOST.checkReceive() == CAN_MSGAVAIL && (millis() - flush_start) < 100) {
      unsigned long rx_id;
      unsigned char len;
      unsigned char buf[8];
      CAN_HOST.readMsgBuf(&rx_id, &len, buf);
      flushed++;
    }
    if (flushed > 0) {
      LOG_WARN("Host CAN: Flushed " + String(flushed) + " stale messages from RX buffer");
    }
  } else {
    LOG_ERROR("Failed to initialize Host CAN on SPI1!");
    LOG_INFO("Continuing without Host CAN (waypoints via serial only)");
  }
  
  // === HOST CAN LOOPBACK TEST ===
  LOG_INFO("=== HOST CAN (J5) LOOPBACK TEST ===");
  if (CAN_HOST.setMode(MCP_LOOPBACK) == CAN_OK) {
    LOG_INFO("Host CAN Loopback mode: ENABLED");
    
    unsigned char testData[8] = {0xCA, 0xFE, 0xBA, 0xBE, 0x11, 0x22, 0x33, 0x44};
    unsigned long testId = 0x123;
    
    byte sendResult = CAN_HOST.sendMsgBuf(testId, 0, 8, testData);
    if (sendResult == CAN_OK) {
      LOG_INFO("Host CAN Loopback TX: OK");
      
      delay(10);
      
      if (CAN_HOST.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId;
        unsigned char rxLen;
        unsigned char rxBuf[8];
        
        if (CAN_HOST.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
          LOG_INFO("Host CAN RX ID=0x" + String(rxId, HEX) + " Len=" + String(rxLen));
          String rxDataStr = "Host CAN RX Data: ";
          for (int i = 0; i < rxLen; i++) {
            rxDataStr += String(rxBuf[i], HEX) + " ";
          }
          LOG_INFO(rxDataStr);
          
          bool dataMatch = (rxId == testId) && (rxLen == 8);
          for (int i = 0; i < 8 && dataMatch; i++) {
            if (rxBuf[i] != testData[i]) dataMatch = false;
          }
          
          if (dataMatch) {
            LOG_INFO("✓ HOST CAN LOOPBACK PASSED!");
          } else {
            LOG_ERROR("✗ HOST CAN LOOPBACK DATA MISMATCH!");
            LOG_ERROR("Expected ID=0x123, Data=CA FE BA BE 11 22 33 44");
          }
        } else {
          LOG_ERROR("✗ Host CAN Loopback RX read failed");
        }
      } else {
        LOG_ERROR("✗ Host CAN Loopback RX: No message received");
      }
    } else {
      LOG_ERROR("✗ Host CAN Loopback TX failed (code " + String(sendResult) + ")");
    }
  } else {
    LOG_ERROR("✗ Host CAN failed to enter loopback mode");
  }
  LOG_INFO("=== END HOST CAN LOOPBACK TEST ===");
  
  if (CAN_HOST.setMode(MCP_NORMAL) != CAN_OK) {
    LOG_ERROR("Failed to set Host CAN to normal mode.");
  } else {
    LOG_INFO("Host CAN normal mode set successfully.");
  }

  // Initialize waypoint buffers for CAN-based control
  waypoint_buffers_init(ACTIVE_JOINT_CONFIG.dof_count);
  LOG_INFO("Waypoint buffers initialized for " + String(ACTIVE_JOINT_CONFIG.dof_count) + " DOFs");

  // Verify CS pins are HIGH (inactive) after initialization
  LOG_INFO("CS pin state: Motor CAN (GP" + String(CAN_CS_PIN) + ")=" + String(digitalRead(CAN_CS_PIN)) +
           ", Host CAN (GP" + String(CAN_HOST_CS_PIN) + ")=" + String(digitalRead(CAN_HOST_CS_PIN)));

  LOG_INFO("Joint firmware starting!");

  // blink the LED 10 times to signal the start of the program
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  LOG_INFO("Joint firmware started!");

  // Initialize direct encoder reading (MT6835 sensors via SPI0)
  // Configure which encoders are connected based on active joint DOF count
  LOG_INFO("=== DIRECT ENCODER CONFIGURATION ===");
  LOG_INFO("Active joint: " + String(ACTIVE_JOINT_CONFIG.name));
  LOG_INFO("DOF count: " + String(ACTIVE_JOINT_CONFIG.dof_count));
  
  for (int i = 0; i < ACTIVE_JOINT_CONFIG.dof_count; i++) {
    uint8_t encoder_channel = ACTIVE_JOINT_CONFIG.dofs[i].encoder_channel;
    if (encoder_channel < DIRECT_ENCODER_COUNT) {
      directEncoders.setEncoderConnected(encoder_channel, true);
      LOG_INFO("Encoder " + String(encoder_channel) + " enabled for DOF " + String(i) + 
               " (" + String(ACTIVE_JOINT_CONFIG.dofs[i].name) + ")");
    }
  }
  
  // Initialize direct encoders (SPI0 to MT6835 sensors)
  directEncoders.begin();
  LOG_INFO("=================================");

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

  // Initialize active joint controller (uses direct encoder reading)
  active_joint_controller = new JointController(ACTIVE_JOINT_CONFIG, &CAN, &directEncoders);
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

  // Load system settings from flash
  LOG_INFO("------------------------------------");
  LOG_INFO("Loading system settings from flash...");
  if (load_system_settings_data(&system_settings)) {
    system_settings_loaded = true;
    LOG_INFO("System settings loaded: auto_start=" + String(system_settings.auto_start_enabled ? "ENABLED" : "DISABLED"));
  } else {
    // Initialize with defaults and save to flash (first boot / new Pico)
    LOG_INFO("No system settings found — initializing defaults and saving to flash...");
    system_settings.auto_start_enabled = false;
    system_settings.auto_start_pretension = 0;
    system_settings.auto_start_duration = 0;
    system_settings.joint_type = ACTIVE_JOINT;
    
    // Save defaults to flash so next boot will find valid settings
    save_system_settings_data(system_settings);
    system_settings_loaded = true;
    LOG_INFO("Default system settings saved: auto_start=DISABLED");
  }

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

  // Enable motor power now that all systems are initialized
  // Rev B: GP22 HIGH — motors can now receive commands
  // Rev A: no-op (motors always powered)
  safety_motor_power_enable();
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
