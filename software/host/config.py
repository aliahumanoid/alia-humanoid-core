"""
Configuration module for the Joint Controller Host Application.

This module defines all configuration parameters including:
- Serial communication settings
- Flask server configuration
- Joint definitions and angle limits (loaded from joint_config.json)
- Serial command mappings (COMMANDS for current protocol)

IMPORTANT: Joint definitions (JOINTS, MIN_ANGLES, MAX_ANGLES) are now
automatically loaded from joint_config.json, which is generated from the
firmware's config_presets.h. This ensures firmware and host stay in sync.

To update joint configurations:
    1. Modify software/firmware/joint_controller/include/config_presets.h
    2. Run: python software/scripts/extract_joint_config.py
    3. Restart the host application
"""
import os
import json
from pathlib import Path

# ==================== Serial Communication ====================
BAUD_RATE = int(os.environ.get('BAUD_RATE', 115200))

# ==================== Flask Server Configuration ====================
FLASK_HOST = os.environ.get('FLASK_HOST', '0.0.0.0')
FLASK_PORT = int(os.environ.get('FLASK_PORT', 5001))
FLASK_DEBUG = os.environ.get('FLASK_DEBUG', 'False').lower() == 'true'

# ==================== Sampling Configuration ====================
tsample = 0.01  # Sampling interval in seconds (matches microcontroller update frequency)

# ==================== Joint Configuration (Loaded from JSON) ====================
def _load_joint_config():
    """
    Load joint configuration from JSON file generated from firmware config.
    
    The JSON file is the single source of truth for joint definitions,
    generated from software/firmware/joint_controller/include/config_presets.h
    
    Returns:
        tuple: (JOINTS dict, MIN_ANGLES dict, MAX_ANGLES dict)
    """
    config_dir = Path(__file__).parent.parent
    json_path = config_dir / 'joint_config.json'
    
    if not json_path.exists():
        raise FileNotFoundError(
            f"Joint configuration file not found: {json_path}\n"
            f"Please run: python software/scripts/extract_joint_config.py"
        )
    
    with open(json_path, 'r') as f:
        config = json.load(f)
    
    joints = {}
    min_angles = {}
    max_angles = {}
    
    # Convert from firmware naming (snake_case) to host naming (UPPER_CASE)
    # e.g., "knee_left" -> "KNEE_LEFT"
    for joint_key, joint_data in config['joints'].items():
        host_key = joint_key.upper()
        
        # Format joint name nicely (e.g., "knee_left" -> "Left Knee")
        parts = joint_key.split('_')
        if len(parts) == 2:
            nice_name = f"{parts[1].title()} {parts[0].title()}"  # "left knee" -> "Left Knee"
        else:
            nice_name = joint_key.replace('_', ' ').title()
        
        # Build JOINTS entry
        joints[host_key] = {
            'id': joint_data['id'],
            'name': nice_name,
            'dofs': [
                {
                    'index': dof['index'],
                    'name': dof['name'].replace('_', '-'),  # Convert to hyphenated form
                    'zero_angle_offset': dof.get('zero_angle_offset', 0.0)  # Reference angle for "Set Zero"
                }
                for dof in joint_data['dofs']
            ]
        }
        
        # Build MIN_ANGLES and MAX_ANGLES
        if joint_data['dof_count'] == 1:
            # Single DOF: store as scalar
            min_angles[host_key] = joint_data['dofs'][0]['min_angle']
            max_angles[host_key] = joint_data['dofs'][0]['max_angle']
        else:
            # Multi-DOF: store as dict indexed by DOF index
            min_angles[host_key] = {
                dof['index']: dof['min_angle'] for dof in joint_data['dofs']
            }
            max_angles[host_key] = {
                dof['index']: dof['max_angle'] for dof in joint_data['dofs']
            }
    
    return joints, min_angles, max_angles


# Load joint configuration from JSON (firmware-generated single source of truth)
JOINTS, MIN_ANGLES, MAX_ANGLES = _load_joint_config()

# ==================== Serial Protocol Commands ====================
# Serial commands in JOINT:DOF:COMMAND:PARAMS format
# This is the current protocol used for communication with firmware
COMMANDS = {
    'STOP': 'STOP',              # Stop motors
    'PRETENSION': 'PRETENSION',  # Pretension motors
    'PRETENSION_ALL': 'PRETENSION_ALL',  # Pretension all motors simultaneously
    'RELEASE': 'RELEASE',        # Release motors
    'RELEASE_ALL': 'RELEASE_ALL',  # Release all motors simultaneously
    'SET_ZERO': 'SET_ZERO_CURRENT_POS',      # Set current position as zero
    'START_AUTO_MAPPING': 'START_AUTO_MAPPING',  # Start advanced automatic mapping
    'STOP_AUTO_MAPPING': 'STOP_AUTO_MAPPING',    # Stop advanced automatic mapping
    'RECALC_OFFSET': 'RECALC_OFFSET', # Recalculate motor offsets
    'START_TEST_ENCODER': 'START_TEST_ENCODER',  # Start encoder test
    'STOP_TEST_ENCODER': 'STOP_TEST_ENCODER',    # Stop encoder test
    'SYNC': 'SYNC',              # Synchronize times
    'SET_PID': 'SET_PID',        # Set PID parameters
    'GET_PID': 'GET_PID',        # Get PID parameters
    'SET_PID_OUTER': 'SET_PID_OUTER',  # Set outer loop parameters
    'GET_PID_OUTER': 'GET_PID_OUTER',  # Get outer loop parameters
    'SAVE_PID': 'SAVE_PID',      # Save PID parameters to flash
    'LOAD_PID': 'LOAD_PID',      # Load PID parameters from flash
    'RECALC_SAFE_LIMITS': 'RECALC_SAFE_LIMITS',  # Recalculate safe limits from equations
    'CAN_DIAG': 'CAN_DIAG',       # CAN bus diagnostic test
    'STARTUP_SEQUENCE': 'STARTUP_SEQUENCE',  # Manual startup sequence (recalc all DOFs + enter HOLDING)
    'SET_AUTO_START': 'SET_AUTO_START',      # Enable/disable auto-start on boot
    'GET_AUTO_START': 'GET_AUTO_START',       # Query auto-start setting
    'CHECK_OFFSETS': 'CHECK_OFFSETS',          # Validate saved motor offsets (smart recalc detection)
    'CASCADE_SPEED_SCALING': 'CASCADE_SPEED_SCALING',  # Velocity-dependent stiffness scaling
    'GET_ENCODER_OFFSETS': 'GET_ENCODER_OFFSETS',      # Query encoder offsets for cross-validation
}

# ==================== CAN ID Constants ====================
# System Control (Host → Controller)
CAN_ID_EMERGENCY_STOP = 0x000
CAN_ID_TIME_SYNC = 0x002
CAN_ID_ENCODER_STREAM_CTRL = 0x003
CAN_ID_PID_DIAG_CTRL = 0x004
CAN_ID_INTERPOLATION_MODE = 0x005
CAN_ID_LOOP_FREQUENCY = 0x006
CAN_ID_PID_DIAG_FREQ = 0x007
CAN_ID_IDENTIFY_REQUEST = 0x008
CAN_ID_STARTUP_SEQUENCE = 0x009

# Trajectory (Host → Controller)
CAN_ID_MULTI_DOF_WAYPOINT_BASE = 0x380

# Status Feedback (Controller → Host)
CAN_ID_STATUS_BASE = 0x400
CAN_ID_ENCODER_STREAM_DATA = 0x410
CAN_ID_PID_DIAG_DATA = 0x420
CAN_ID_PID_TORQUE_DATA = 0x430
CAN_ID_MOVEMENT_METRICS = 0x440
CAN_ID_SMOOTHNESS_METRICS = 0x460
CAN_ID_PID_INNER_TERMS = 0x470
CAN_ID_PID_OUTER_TERMS = 0x480
CAN_ID_STARTUP_STATUS = 0x490
CAN_ID_JOINT_ANNOUNCE = 0x4A0

# Startup event types
STARTUP_EVT_BEGIN = 0
STARTUP_EVT_DOF_READY = 1
STARTUP_EVT_DOF_FAILED = 2
STARTUP_EVT_COMPLETE = 3
STARTUP_EVT_FAILED = 4

# Startup reason codes
STARTUP_REASON_OK = 0
STARTUP_REASON_NO_CONTROLLER = 1
STARTUP_REASON_NO_EQUATIONS = 2
STARTUP_REASON_ENCODER_TIMEOUT = 3
STARTUP_REASON_POSITION_RANGE = 4
STARTUP_REASON_RECALC_ERROR = 5
STARTUP_REASON_GLOBAL_TIMEOUT = 6
