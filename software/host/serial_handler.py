"""
Serial communication handler for joint controller firmware.

This module provides the SerialHandler class which manages:
- Serial port communication with joint controller boards
- Protocol parsing (EVT:, RSP:, ERROR:, WARN: messages)
- Command sending and response handling
- Real-time data streaming via WebSocket
- Mapping data collection and storage
- Movement data recording
- Session logging
"""
import re
import threading
import time
import serial
from typing import Any, Dict, List, Optional
from config import BAUD_RATE, tsample, COMMANDS, JOINTS
import logging
import numpy as np
from flask_socketio import SocketIO
from datetime import datetime
import json
import os
import math
from serial_logger import SerialLogger

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SerialHandler:
    """
    Handles serial communication with a single joint controller board.
    
    Manages bidirectional communication, protocol parsing, data collection,
    and WebSocket broadcasting for real-time monitoring.
    
    Attributes:
        socketio: Flask-SocketIO instance for WebSocket broadcasting
        serial_port: Serial port path (e.g., '/dev/ttyUSB0')
        listening: Threading event controlling message listener
        serial_lock: Thread lock for serial write operations
    """
    def __init__(self, socketio: SocketIO, port: str):
        self.socketio = socketio
        self.serial_port = port
        self.listening = threading.Event()
        self.listening.set()
        self.serial_lock = threading.Lock()
        self.extensor: float = 0.0
        self.flexor: float = 0.0
        self.start_measure_motor_output_time: Optional[float] = None
        self.movement_web_data: Dict = {}
        self.joint_status: Dict[str, Any] = {"position": 0, "message": "Ready"}
        self.status_message: List[str] = []
        self.time_offsets: Dict[str, float] = (
            {}
        )  # Stores time offsets for each PICO

        # Time synchronization helpers
        self.time_sync_reference_clock: Optional[float] = None
        self.time_sync_baseline_offset: Optional[float] = None

        # Structure to store PID values for each joint/DOF/motor
        self.pid_values = {}
        self.pid_outer_values = {}

        # Variables to handle duplicate messages
        self.last_error_message = ""
        self.error_count = 0
        self.error_threshold = 5  # Show only 5 consecutive identical messages

        # Add variable to track the last joint for which we requested PID
        self.last_pid_request_joint = None

        # Variable to store automatic mapping data
        self.automatic_mapping_data: Dict[str, Any] = {}

        # Structure to store encoder data for /get_encoder_data endpoint
        self.current_encoder_data: Dict[str, Dict[str, Any]] = {}

        # Variable to track currently active joint (used to save mapping data)
        self.current_active_joint: Optional[str] = None

        # Stores the last joint for which encoder test was started
        self.last_encoder_test_joint: Optional[str] = None

        # Protocol state for contextual filtering
        self.is_receiving_raw_mapping_data: bool = False
        self.expected_raw_data_points: int = 0
        self.received_raw_data_points: int = 0

        # Initialize logger for serial communication
        self.serial_logger = SerialLogger("logs/serial_communication.log")
        self.serial_logger.log_info(f"Initializing SerialHandler for port: {port}")

        self.synchronize_time()

    def pause_listening(self):
        self.listening.clear()  # Suspend listening thread
        time.sleep(0.001)  # Reduced from 10ms to 1ms

    def resume_listening(self):
        self.listening.set()  # Resume listening thread
        time.sleep(0.001)  # Reduced from 10ms to 1ms

    def send_serial_command(self, command):
        """
        Legacy method for compatibility - now automatically uses CMD: prefix
        """
        try:
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1) as ser:
                self.send_command_with_prefix(command, ser)
        except serial.SerialException as e:
            error_msg = f"Serial communication error: {e}"
            logger.error(error_msg)
            self.serial_logger.log_error(error_msg)
            self.status_message.append(f"Serial error: {e}")
        except Exception as e:
            error_msg = f"Error in send_serial_command: {e}"
            logger.error(error_msg)
            self.serial_logger.log_error(error_msg)

    def listen_for_messages(self) -> None:
        while True:
            try:
                with serial.Serial(self.serial_port, BAUD_RATE, timeout=0.05) as ser:
                    while True:
                        if not self.listening.is_set():
                            self.listening.wait()
                        # with self.serial_lock:
                        line = ser.readline().decode().strip()

                        self.handle_serial_message(line, ser)

            except serial.SerialException as e:
                error_msg = f"Serial communication error: {e}"
                logger.error(error_msg)
                self.serial_logger.log_error(error_msg)
                continue
            except Exception as e:
                error_msg = f"Error in listen_for_messages: {e}"
                logger.error(error_msg)
                self.serial_logger.log_error(error_msg)
                continue

    def handle_serial_message(self, line: str, ser: serial.Serial) -> None:
        # Log received message (if not empty)
        if line.strip():
            self.serial_logger.log_received_message(line)

        # New protocol: process only messages with EVT: prefix
        if line.startswith("EVT:"):
            # Remove EVT: prefix and process message
            actual_message = self.remove_evt_prefix(line)

            # Handle duplicate error messages
            if (
                "Error" in actual_message
                or "error" in actual_message
                or "READ_ML_ANGLE" in actual_message
            ):
                # If it's the same error message as the last one
                if actual_message == self.last_error_message:
                    self.error_count += 1
                # If we exceeded threshold, don't queue the message
                if self.error_count > self.error_threshold:
                    # Every 50 messages, update count just to keep track
                    if self.error_count % 50 == 0:
                        self.status_message.append(
                            f"Repeated error {self.error_count} times: {actual_message}"
                        )
                    return
            else:
                # New error message, reset counter
                self.last_error_message = actual_message
                self.error_count = 1

            # Contextual filter for lines with mapping tokens
            if re.match(r"^[abc]\d+_\d+_[-+]?\d*\.?\d+\|", actual_message):
                if self.is_receiving_raw_mapping_data:
                    # We're in raw data reception mode - PROCESS the line
                    logger.debug(
                        f"Processing raw mapping data line: {actual_message[:50]}..."
                    )
                    # Line will be processed normally by underlying flow
                else:
                    # We're in normal mode - IGNORE the line (it's debug output)
                    logger.debug(
                        f"Ignored mapping debug line: {actual_message[:50]}..."
                    )
                    return

            # Proceed with normal message handling (using actual_message without prefix)
            if actual_message.startswith("SYNC_RESPONSE"):
                self.status_message.append(actual_message)
            elif actual_message.startswith("ACK"):
                self.status_message.append(actual_message)
            elif actual_message.startswith("MAPPING_DATA"):
                self.status_message.append(actual_message)
                self.handle_mapping_data(actual_message, ser)
            elif actual_message.startswith("MAPPING_REQUEST"):
                self.status_message.append(actual_message)
                self.handle_mapping_request(actual_message, ser)
            elif actual_message.startswith("MOVEMENT_SAMPLE_HEADER"):
                self.status_message.append(actual_message)
                self.handle_movement_sample_header(actual_message, ser)
            elif actual_message.startswith("MOVEMENT_DATA"):
                self.status_message.append(actual_message)
                self.handle_movement_data(actual_message, ser)
            elif actual_message.startswith("PID_OUTER"):
                self.status_message.append(actual_message)
                self.handle_pid_outer_data(actual_message)
            elif actual_message.startswith("PID"):
                self.status_message.append(actual_message)
                self.handle_pid_data(actual_message)
            elif actual_message.startswith("MEASURE"):
                self.status_message.append(actual_message)
                self.handle_measure_data(actual_message)
            elif actual_message.startswith("ENCODER_DATA"):
                self.status_message.append(actual_message)
                self.handle_encoder_data(actual_message)
            elif actual_message.startswith(("KNEE", "ANKLE", "HIP")):
                self.status_message.append(actual_message)
                self.handle_joint_message(actual_message)
            elif actual_message:
                self.status_message.append(actual_message)
                logger.info(f"Received EVT message: {actual_message}")
        elif line.strip():  # Messages without EVT: prefix are only logged
            logger.info(f"Received non-EVT message (logged only): {line}")
            # Add to status messages for UI anyway, but with distinctive prefix
            self.status_message.append(f"{line}")
        # Empty lines are silently ignored

    def handle_mapping_data(self, line: str, ser: serial.Serial) -> None:
        # MAPPING_DATA(total_points,dof_count) protocol
        match = re.match(r"MAPPING_DATA\((\d+),(\d+)\)", line)
        if match:
            total_points = int(match.group(1))
            dof_count = int(match.group(2))

            logger.info(
                f"Received MAPPING_DATA command: {total_points} points, {dof_count} DOF"
            )

            # Activate raw data reception mode
            self.is_receiving_raw_mapping_data = True
            self.expected_raw_data_points = total_points
            self.received_raw_data_points = 0
            logger.debug(
                f"🔄 Raw data reception mode ACTIVATED: expecting {total_points} points"
            )

            # Handle automatic mapping protocol
            self.handle_automatic_mapping_data(total_points, dof_count, ser)
        else:
            logger.error(f"Invalid MAPPING_DATA command format: {line}")

    def handle_movement_data(self, line: str, ser: serial.Serial) -> None:
        """
        Handles movement data in new multi-DOF format:
        MOVEMENT_DATA(joint_name,dof,length,array_type)

        Args:
            line: Line received from PICO
            ser: Serial connection to read data
        """
        # New multi-DOF format: MOVEMENT_DATA(joint_name,dof,length,array_type)
        match = re.match(r"MOVEMENT_DATA\(([^,]+),(\d+|ALL),(\d+),(\d+)\)", line)
        if match:
            joint_name = match.group(1)
            dof = match.group(2)  # Can be a number or "ALL"
            length = int(match.group(3))
            array_type = int(match.group(4))

            logger.info(
                f"Received movement data from {joint_name} DOF {dof}: {length} samples, type {array_type}"
            )

            # Set active joint for this movement
            self.current_active_joint = joint_name

            # Handle different array types
            if array_type in (7, 8):
                self.handle_movement_data_multi_dof(
                    joint_name, dof, length, array_type, ser
                )
            elif array_type == 5:
                self.handle_movement_data_multi_dof(
                    joint_name, dof, length, array_type, ser
                )
            else:
                logger.warning(
                    f"Array type {array_type} not supported for {joint_name} DOF {dof}. Supported types: 5, 7"
                )
        else:
            logger.error(f"Invalid multi-DOF MOVEMENT_DATA command format: {line}")
            logger.error(
                "Expected format: MOVEMENT_DATA(joint_name,dof,length,array_type)"
            )

    def handle_movement_data_multi_dof(
        self,
        joint_name: str,
        dof: str,
        length: int,
        array_type: int,
        ser: serial.Serial,
    ) -> None:
        """
        Handles multi-DOF movement data with hierarchical structure

        Args:
            joint_name: Joint name (e.g. "KNEE_LEFT", "ANKLE_LEFT")
            dof: Specific DOF (e.g. "0", "1") or "ALL"
            length: Number of samples to read
            array_type: Array type (5 or 7)
            ser: Serial connection to read data
        """
        try:
            logger.info(
                f"📊 Processing {length} samples for {joint_name} DOF {dof} (type {array_type})"
            )

            # Initialize hierarchical data structure for this movement
            movement_data = self.initialize_movement_data_structure(
                joint_name, dof, array_type, length
            )

            # Read and parse each data line
            for sample_idx in range(length):
                line = ser.readline().decode().strip()

                if not line:
                    logger.warning(
                        f"Empty line received at sample {sample_idx} for {joint_name} DOF {dof}"
                    )
                    continue

                # Handle EVT: prefixes if present
                clean_line = self.remove_evt_prefix(line)

                # Parse data line with joint/DOF context
                parsed_data = self.parse_movement_data_line_multi_dof(
                    clean_line, joint_name, dof, sample_idx, array_type
                )

                if parsed_data:
                    # Store data in hierarchical structure
                    self.store_movement_sample_data(
                        movement_data, parsed_data, sample_idx
                    )
                else:
                    logger.warning(
                        f"Error parsing sample {sample_idx} for {joint_name} DOF {dof}: {clean_line[:50]}..."
                    )

            # Process and send collected data
            self.process_movement_data_multi_dof(movement_data)
            logger.info(
                f"✅ Movement data processed for {joint_name} DOF {dof}: {length} samples"
            )

        except Exception as e:
            logger.error(
                f"Error handling multi-DOF movement data for {joint_name} DOF {dof}: {e}"
            )
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def initialize_movement_data_structure(
        self, joint_name: str, dof: str, array_type: int, length: int
    ) -> Dict:
        """
        Initializes hierarchical data structure for movement data

        Returns:
            Dict: Initialized hierarchical data structure
        """
        return {
            "metadata": {
                "timestamp": time.time(),
                "total_samples": length,
                "array_type": array_type,
                "source_joint": joint_name,
                "source_dof": dof,
            },
            "joints": {
                joint_name: {
                    f"dof_{dof}": {
                        "joint_angles": [],  # alpha target - joint angles over time
                        "joint_angles_actual": [],  # alpha effettivo
                        "motor_angles": {
                            "agonist_current": [],  # gamma_curr - current agonist motor
                            "antagonist_current": [],  # theta_curr - current antagonist motor
                            "agonist_next": [],  # gamma_next - next agonist motor
                            "antagonist_next": [],  # theta_next - next antagonist motor
                        },
                        "motor_torques": (
                            {
                                "agonist": [],  # gamma_torque (solo se array_type=7)
                                "antagonist": [],  # theta_torque (solo se array_type=7)
                            }
                            if array_type == 7
                            else {}
                        ),
                        "metadata": {
                            "motor_names": self.get_motor_names_for_joint_dof(
                                joint_name, dof
                            ),
                            "array_type": array_type,
                            "samples_count": length,
                        },
                    }
                }
            },
        }

    def get_motor_names_for_joint_dof(
        self, joint_name: str, dof: str
    ) -> Dict[str, str]:
        """
        Returns descriptive motor names for a specific joint/DOF

        Args:
            joint_name: Joint name
            dof: Joint DOF

        Returns:
            Dict with agonist/antagonist names for this joint/DOF
        """
        joint_type = joint_name.split("_")[0]  # Estrae KNEE, ANKLE, HIP

        try:
            dof_index = int(dof) if dof != "ALL" else 0
        except ValueError:
            dof_index = 0

        if joint_type == "KNEE":
            return {"agonist": "extensor", "antagonist": "flexor"}
        elif joint_type == "ANKLE":
            if dof_index == 0:  # plantar-dorsal    
                return {"agonist": "plantar", "antagonist": "dorsal"}
            elif dof_index == 1:  # inversion-eversion
                return {"agonist": "inversion", "antagonist": "eversion"}
        elif joint_type == "HIP":
            if dof_index == 0:  # flexion-extension
                return {"agonist": "flexor", "antagonist": "extensor"}
            elif dof_index == 1:  # abduction-adduction 
                return {"agonist": "abductor", "antagonist": "adductor"}

        # Fallback for unrecognized types
        return {"agonist": "motor_1", "antagonist": "motor_2"}

    def parse_movement_data_line_multi_dof(
        self, line: str, joint_name: str, dof: str, sample_idx: int, array_type: int
    ) -> Optional[Dict]:
        """
        Parses multi-DOF movement data line

        Format array_type=5: a{index}_{alpha}|b{index}_{gamma_curr}|c{index}_{theta_curr}|d{index}_{gamma_next}|e{index}_{theta_next}
        Format array_type=7: adds |f{index}_{gamma_torque}|g{index}_{theta_torque}

        Args:
            line: Line to parse
            joint_name: Joint name for context
            dof: DOF for context
            sample_idx: Expected sample index
            array_type: Array type (5 or 7)

        Returns:
            Dict with parsed data or None if error
        """
        try:
            if array_type == 8:
                match = re.match(
                    r"a(\d+)_([-+]?\d*\.?\d+)\|h\1_([-+]?\d*\.?\d+)\|b\1_([-+]?\d*\.?\d+)\|c\1_([-+]?\d*\.?\d+)\|d\1_([-+]?\d*\.?\d+)\|e\1_([-+]?\d*\.?\d+)\|f\1_([-+]?\d*\.?\d+)\|g\1_([-+]?\d*\.?\d+)",
                    line,
                )
                if match:
                    index = int(match.group(1))
                    joint_target = float(match.group(2))
                    joint_actual = float(match.group(3))
                    gamma_curr = float(match.group(4))
                    theta_curr = float(match.group(5))
                    gamma_next = float(match.group(6))
                    theta_next = float(match.group(7))
                    gamma_torque = float(match.group(8))
                    theta_torque = float(match.group(9))

                    return {
                        "index": index,
                        "joint_target": joint_target,
                        "joint_actual": joint_actual,
                        "motor_angles": {
                            "agonist_current": gamma_curr,
                            "antagonist_current": theta_curr,
                            "agonist_next": gamma_next,
                            "antagonist_next": theta_next,
                        },
                        "motor_torques": {
                            "agonist": gamma_torque,
                            "antagonist": theta_torque,
                        },
                    }

            if array_type == 5:
                match = re.match(
                    r"a(\d+)_([-+]?\d*\.?\d+)\|b\1_([-+]?\d*\.?\d+)\|c\1_([-+]?\d*\.?\d+)\|d\1_([-+]?\d*\.?\d+)\|e\1_([-+]?\d*\.?\d+)",
                    line,
                )
                if match:
                    index = int(match.group(1))
                    alpha_next = float(match.group(2))
                    gamma_curr = float(match.group(3))
                    theta_curr = float(match.group(4))
                    gamma_next = float(match.group(5))
                    theta_next = float(match.group(6))

                    return {
                        "index": index,
                        "joint_target": alpha_next,
                        "motor_angles": {
                            "agonist_current": gamma_curr,
                            "antagonist_current": theta_curr,
                            "agonist_next": gamma_next,
                            "antagonist_next": theta_next,
                        },
                    }

            elif array_type == 7:
                match = re.match(
                    r"a(\d+)_([-+]?\d*\.?\d+)\|b\1_([-+]?\d*\.?\d+)\|c\1_([-+]?\d*\.?\d+)\|d\1_([-+]?\d*\.?\d+)\|e\1_([-+]?\d*\.?\d+)\|f\1_([-+]?\d*\.?\d+)\|g\1_([-+]?\d*\.?\d+)",
                    line,
                )
                if match:
                    index = int(match.group(1))
                    alpha_next = float(match.group(2))
                    gamma_curr = float(match.group(3))
                    theta_curr = float(match.group(4))
                    gamma_next = float(match.group(5))
                    theta_next = float(match.group(6))
                    gamma_torque = float(match.group(7))
                    theta_torque = float(match.group(8))

                    return {
                        "index": index,
                        "joint_target": alpha_next,
                        "motor_angles": {
                            "agonist_current": gamma_curr,
                            "antagonist_current": theta_curr,
                            "agonist_next": gamma_next,
                            "antagonist_next": theta_next,
                        },
                        "motor_torques": {
                            "agonist": gamma_torque,
                            "antagonist": theta_torque,
                        },
                        "joint_actual": alpha_next,
                    }

            logger.warning(
                f"Invalid movement line format for {joint_name} DOF {dof} type {array_type}: {line[:50]}..."
            )
            return None

        except Exception as e:
            logger.error(
                f"Error parsing movement line {joint_name} DOF {dof}: {e}"
            )
            return None

    def store_movement_sample_data(
        self, movement_data: Dict, parsed_data: Dict, sample_idx: int
    ) -> None:
        """
        Stores sample data in movement structure

        Args:
            movement_data: Movement data structure
            parsed_data: Parsed sample data
            sample_idx: Sample index
        """
        try:
            joint_name = movement_data["metadata"]["source_joint"]
            dof = movement_data["metadata"]["source_dof"]
            dof_key = f"dof_{dof}"

            if (
                joint_name in movement_data["joints"]
                and dof_key in movement_data["joints"][joint_name]
            ):
                dof_data = movement_data["joints"][joint_name][dof_key]

                joint_target = parsed_data.get("joint_target")
                if joint_target is None:
                    joint_target = parsed_data.get("joint_angle", 0.0)

                joint_actual = parsed_data.get("joint_actual", joint_target)

                dof_data["joint_angles"].append(joint_target)
                if "joint_angles_actual" in dof_data:
                    dof_data["joint_angles_actual"].append(joint_actual)

                # Store motor angles
                motor_angles = parsed_data["motor_angles"]
                dof_data["motor_angles"]["agonist_current"].append(
                    motor_angles["agonist_current"]
                )
                dof_data["motor_angles"]["antagonist_current"].append(
                    motor_angles["antagonist_current"]
                )
                dof_data["motor_angles"]["agonist_next"].append(
                    motor_angles["agonist_next"]
                )
                dof_data["motor_angles"]["antagonist_next"].append(
                    motor_angles["antagonist_next"]
                )

                # Store motor torques if present
                if "motor_torques" in parsed_data and "motor_torques" in dof_data:
                    motor_torques = parsed_data["motor_torques"]
                    dof_data["motor_torques"]["agonist"].append(
                        motor_torques["agonist"]
                    )
                    dof_data["motor_torques"]["antagonist"].append(
                        motor_torques["antagonist"]
                    )

        except Exception as e:
            logger.error(f"Error storing sample {sample_idx}: {e}")

    def process_movement_data_multi_dof(self, movement_data: Dict) -> None:
        """
        Processes multi-DOF movement data and sends to UI

        Args:
            movement_data: Hierarchical data structure with movement data
        """
        try:
            # Update internal structure with multi-DOF data
            self.movement_web_data = movement_data

            # Emit SocketIO event with multi-DOF structure
            self.socketio.emit(
                "movement_data_multi_dof",
                {
                    "data": movement_data,
                    "timestamp": movement_data["metadata"]["timestamp"],
                },
                namespace="/movement",
            )

            # Summary log
            joint_name = movement_data["metadata"]["source_joint"]
            dof = movement_data["metadata"]["source_dof"]
            samples_count = movement_data["metadata"]["total_samples"]
            array_type = movement_data["metadata"]["array_type"]

            logger.info(
                f"📈 Multi-DOF movement data sent to UI: {joint_name} DOF {dof}, {samples_count} samples, type {array_type}"
            )

        except Exception as e:
            logger.error(f"Error processing multi-DOF movement data: {e}")

    def handle_pid_data(self, line: str) -> None:
        # New format: PID:<DOF>:<MOTOR_TYPE>:<KP>:<KI>:<KD>:<TAU>
        match = re.match(
            r"PID:(\d+):(\d+):([\d\.\-]+):([\d\.\-]+):([\d\.\-]+):([\d\.\-]+)", line
        )
        if match:
            dof, motor_type, kp, ki, kd, tau = match.groups()
            dof = int(dof)
            motor_type = int(motor_type)  # 1=agonista, 2=antagonista
            kp, ki, kd, tau = map(float, [kp, ki, kd, tau])

            # Use last joint that requested PIDs, if available
            active_joint = self.last_pid_request_joint

            # If we don't have last joint, try to find active joint
            if not active_joint:
                for joint_id in list(JOINTS.keys()):
                    if (
                        joint_id in self.joint_status
                        and "active" in self.joint_status[joint_id]
                        and self.joint_status[joint_id]["active"]
                    ):
                        active_joint = joint_id
                        break

            if not active_joint:
                active_joint = "UNKNOWN_JOINT"

            # Save PID values in structure
            if active_joint not in self.pid_values:
                self.pid_values[active_joint] = {}
            if dof not in self.pid_values[active_joint]:
                self.pid_values[active_joint][dof] = {}

            motor_name = "agonist" if motor_type == 1 else "antagonist"
            self.pid_values[active_joint][dof][motor_name] = {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "tau": tau,
            }

            logger.info(
                f"PID for {active_joint} DOF {dof} motor {motor_name}: kp={kp}, ki={ki}, kd={kd}, tau={tau}"
            )

            # Emit event to update UI
            self.socketio.emit(
                "pid_data",
                {
                    "joint": active_joint,
                    "dof": dof,
                    "motor_type": motor_type,
                    "values": {"kp": kp, "ki": ki, "kd": kd, "tau": tau},
                },
                namespace="/movement",
            )
        else:
            # Support for old format (if needed)
            old_match = re.match(r"PID\(([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)\)", line)
            if old_match:
                kp, ki, kd = map(float, old_match.groups())
                logger.info(f"PID (old format): kp={kp}, ki={ki}, kd={kd}")
                self.socketio.emit(
                    "pid_data", {"kp": kp, "ki": ki, "kd": kd}, namespace="/movement"
                )

    def handle_movement_sample_header(self, line: str, ser: serial.Serial) -> None:
        """
        Handles new movement sample protocol:
        EVT:MOVEMENT_SAMPLE_HEADER(joint_id,dof_count)
        EVT:DOF0_SAMPLE_COUNT(count)
        EVT:DOF0_SAMPLE(idx,joint_target,joint_actual,motor_agonist_curr,motor_antagonist_curr,motor_agonist_ref,motor_antagonist_ref,torque_agonist,torque_antagonist)
        ...
        EVT:MOVEMENT_SAMPLES_END
        """
        try:
            # Parse header: MOVEMENT_SAMPLE_HEADER(joint_id,dof_count)
            match = re.match(r"MOVEMENT_SAMPLE_HEADER\((\d+),(\d+)\)", line)
            if not match:
                logger.error(f"Invalid MOVEMENT_SAMPLE_HEADER format: {line}")
                return

            joint_id = int(match.group(1))
            dof_count = int(match.group(2))
            
            # Map joint_id to joint_name
            joint_name_map = {1: "KNEE_LEFT", 2: "KNEE_RIGHT", 3: "ANKLE_LEFT", 
                            4: "ANKLE_RIGHT", 5: "HIP_LEFT", 6: "HIP_RIGHT"}
            joint_name = joint_name_map.get(joint_id, f"JOINT_{joint_id}")
            
            logger.info(f"📊 Receiving movement samples for {joint_name} ({dof_count} DOFs)")

            # Initialize data structure for each DOF
            dof_data = {}
            for dof in range(dof_count):
                dof_data[dof] = {
                    "joint_angles": [],
                    "joint_targets": [],
                    "motor_agonist_angles": [],
                    "motor_antagonist_angles": [],
                    "motor_agonist_refs": [],
                    "motor_antagonist_refs": [],
                    "motor_agonist_torques": [],
                    "motor_antagonist_torques": []
                }

            # Read lines until MOVEMENT_SAMPLES_END
            sample_lines_read = 0
            max_lines_to_read = 1000  # Safety limit
            
            while sample_lines_read < max_lines_to_read:
                line = ser.readline().decode().strip()
                sample_lines_read += 1
                
                # Log received message to serial_logger
                if line.strip():
                    self.serial_logger.log_received_message(line)
                
                if not line:
                    logger.debug(f"Empty line #{sample_lines_read}")
                    continue
                
                # Log what we're reading (first 100 chars)
                logger.debug(f"Sample line #{sample_lines_read}: {line[:100]}")
                
                # Remove EVT: prefix if present
                clean_line = self.remove_evt_prefix(line)
                
                if clean_line == "MOVEMENT_SAMPLES_END":
                    logger.info(f"✅ Movement samples reception complete for {joint_name} (read {sample_lines_read} lines)")
                    break
                
                # Skip non-sample lines (INFO, DBG, CMD, RSP, etc.)
                if any(clean_line.startswith(prefix) for prefix in ["INFO:", "DBG:", "CMD:", "RSP:", "WARN:", "ERROR:"]):
                    logger.debug(f"Skipping non-sample line: {clean_line[:50]}")
                    continue
                
                # Parse DOF{X}_SAMPLE_COUNT(count)
                count_match = re.match(r"DOF(\d+)_SAMPLE_COUNT\((\d+)\)", clean_line)
                if count_match:
                    dof_idx = int(count_match.group(1))
                    sample_count = int(count_match.group(2))
                    logger.info(f"  DOF {dof_idx}: expecting {sample_count} samples")
                    continue
                
                # Parse DOF{X}_SAMPLE(data...)
                sample_match = re.match(
                    r"DOF(\d+)_SAMPLE\((\d+),([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+)\)",
                    clean_line
                )
                if sample_match:
                    dof_idx = int(sample_match.group(1))
                    idx = int(sample_match.group(2))
                    joint_target = float(sample_match.group(3))
                    joint_actual = float(sample_match.group(4))
                    motor_agonist_curr = float(sample_match.group(5))
                    motor_antagonist_curr = float(sample_match.group(6))
                    motor_agonist_ref = float(sample_match.group(7))
                    motor_antagonist_ref = float(sample_match.group(8))
                    torque_agonist = float(sample_match.group(9))
                    torque_antagonist = float(sample_match.group(10))
                    
                    # Store sample data
                    if dof_idx in dof_data:
                        dof_data[dof_idx]["joint_targets"].append(joint_target)
                        dof_data[dof_idx]["joint_angles"].append(joint_actual)
                        dof_data[dof_idx]["motor_agonist_angles"].append(motor_agonist_curr)
                        dof_data[dof_idx]["motor_antagonist_angles"].append(motor_antagonist_curr)
                        dof_data[dof_idx]["motor_agonist_refs"].append(motor_agonist_ref)
                        dof_data[dof_idx]["motor_antagonist_refs"].append(motor_antagonist_ref)
                        dof_data[dof_idx]["motor_agonist_torques"].append(torque_agonist)
                        dof_data[dof_idx]["motor_antagonist_torques"].append(torque_antagonist)
            
            # Warn if we hit the max lines limit without finding END
            if sample_lines_read >= max_lines_to_read:
                logger.warning(f"⚠️ Reached max lines ({max_lines_to_read}) without finding MOVEMENT_SAMPLES_END")

            # Debug: Log what we collected
            logger.info(f"🔍 Collected data summary:")
            for dof_idx, data in dof_data.items():
                logger.info(f"   DOF {dof_idx}: {len(data['joint_angles'])} samples")
            
            # Send ALL DOFs in a single message (correct format for frontend)
            total_samples = sum(len(data['joint_angles']) for data in dof_data.values())
            
            # Transform data structure to match frontend expectations
            transformed_dof_data = {}
            for dof_idx, data in dof_data.items():
                transformed_dof_data[dof_idx] = {
                    "joint_angles": data["joint_angles"],
                    "joint_targets": data["joint_targets"],
                    "motor_angles": {
                        "agonist_current": data["motor_agonist_angles"],
                        "antagonist_current": data["motor_antagonist_angles"],
                        "agonist_next": data["motor_agonist_refs"],
                        "antagonist_next": data["motor_antagonist_refs"]
                    },
                    "motor_torques": {
                        "agonist": data["motor_agonist_torques"],
                        "antagonist": data["motor_antagonist_torques"]
                    }
                }
            
            # Build payload with correct structure: joints -> joint_name -> dof_idx -> data
            payload = {
                "metadata": {
                    "source_joint": joint_name,
                    "source_dof": "ALL",
                    "total_samples": total_samples,
                    "array_type": 7  # Full cascade data
                },
                "joints": {
                    joint_name: transformed_dof_data  # All DOFs for this joint
                }
            }
            
            logger.info(f"📤 Sending {total_samples} samples for {joint_name} ({len(transformed_dof_data)} DOFs) to frontend")
            self.socketio.emit("movement_data_multi_dof", payload, namespace="/movement")

        except Exception as e:
            logger.error(f"❌ EXCEPTION in handle_movement_sample_header: {e}")
            logger.error(f"❌ Exception type: {type(e).__name__}")
            import traceback
            logger.error(f"❌ Traceback:")
            traceback.print_exc()
            logger.error(f"❌ Full traceback: {traceback.format_exc()}")

    def handle_pid_outer_data(self, line: str) -> None:
        match = re.match(
            r"PID_OUTER:(\d+):([\d\.\-]+):([\d\.\-]+):([\d\.\-]+):([\d\.\-]+):([\d\.\-]+)",
            line,
        )
        if not match:
            logger.error(f"Invalid PID_OUTER format: {line}")
            return

        dof, kp, ki, kd, stiffness, cascade = match.groups()
        dof = int(dof)
        kp, ki, kd, stiffness, cascade = map(float, [kp, ki, kd, stiffness, cascade])

        active_joint = self.last_pid_request_joint
        if not active_joint:
            active_joint = "UNKNOWN_JOINT"

        if active_joint not in self.pid_outer_values:
            self.pid_outer_values[active_joint] = {}

        self.pid_outer_values[active_joint][dof] = {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "stiffness": stiffness,
            "cascade": cascade,
        }

        logger.info(
            f"PID esterno per {active_joint} DOF {dof}: kp={kp}, ki={ki}, kd={kd}, stiffness={stiffness}, cascade={cascade}"
        )

        self.socketio.emit(
            "pid_outer_data",
            {
                "joint": active_joint,
                "dof": dof,
                "values": {
                    "kp": kp,
                    "ki": ki,
                    "kd": kd,
                    "stiffness": stiffness,
                    "cascade": cascade,
                },
            },
            namespace="/movement",
        )

    def handle_measure_data(self, line: str) -> None:
        """
        Handles measurement data in new format: JOINT:DOF:MEASURE:value1|value2|...
        """
        try:
            parts = line.split(":")
            if len(parts) < 4 or parts[2] != "MEASURE":
                logger.warning(f"Invalid MEASURE format: {line}")
                return

            joint = parts[0]
            dof = parts[1]
            values_str = parts[3]

            # Parse pipe-separated values
            values = [float(val) for val in values_str.split("|") if val.strip()]

            if not values:
                logger.warning(f"No valid values found in: {values_str}")
                return

            # Update joint state
            self._update_joint_measure_status(joint, dof, values)
            logger.info(
                f"Motor Output ({joint} DOF {dof}): {len(values)} values received"
            )

        except (ValueError, IndexError) as e:
            logger.error(f"Error parsing MEASURE data: {e}")

    def _update_joint_measure_status(
        self, joint: str, dof: str, values: List[float]
    ) -> None:
        """
        Updates joint state with measurement data

        Args:
            joint (str): Nome del giunto
            dof (str): Joint DOF
            values (List[float]): Measurement values
        """
        # Initialize structure if necessary
        if joint not in self.joint_status:
            self.joint_status[joint] = {}
        if dof not in self.joint_status[joint]:
            self.joint_status[joint][dof] = {}
        if "measure" not in self.joint_status[joint][dof]:
            self.joint_status[joint][dof]["measure"] = {}

        # Map values based on joint type and DOF
        measure_data = self._map_measure_values_by_joint_type(joint, dof, values)

        # Update state
        self.joint_status[joint][dof]["measure"] = measure_data

        # Update legacy variables for compatibility with existing code that uses them
        if joint.startswith("KNEE") and dof == "0" and len(values) >= 2:
            self.extensor = values[0]
            self.flexor = values[1]

            # Emit event for real-time visualization
            self.socketio.emit(
                "joint_measure",
                {
                    "joint": joint,
                    "dof": dof,
                    "data": measure_data,
                    "timestamp": time.time(),
                },
                namespace="/movement",
            )

    def _map_measure_values_by_joint_type(
        self, joint: str, dof: str, values: List[float]
    ) -> Dict[str, float]:
        """
        Maps measurement values based on joint type and DOF

        Args:
            joint (str): Joint name
            dof (str): Joint DOF
            values (List[float]): Measurement values

        Returns:
            Dict[str, float]: Mapped values with descriptive names
        """
        joint_type = joint.split("_")[0]  # Extracts KNEE, ANKLE, HIP
        measure_data = {}

        try:
            dof_index = int(dof) if dof != "ALL" else 0
        except ValueError:
            dof_index = 0

        if joint_type == "KNEE":
            # For KNEE: always extensor/flexor
            if len(values) >= 1:
                measure_data["extensor"] = values[0]
            if len(values) >= 2:
                measure_data["flexor"] = values[1]

        elif joint_type == "ANKLE":
            if dof_index == 0:  # plantar-dorsal
                if len(values) >= 1:
                    measure_data["plantar"] = values[0]
                if len(values) >= 2:
                    measure_data["dorsal"] = values[1]
            elif dof_index == 1:  # inversione-eversione
                if len(values) >= 1:
                    measure_data["inversion"] = values[0]
                if len(values) >= 2:
                    measure_data["eversion"] = values[1]
            else:
                # Unrecognized DOF, use generic names
                for i, val in enumerate(values):
                    measure_data[f"motor_{i+1}"] = val

        elif joint_type == "HIP":
            if dof_index == 0:  # flexion-extension
                for i, val in enumerate(values):
                    measure_data[f"flex_ext_{i+1}"] = val
            elif dof_index == 1:  # abduction-adduction 
                for i, val in enumerate(values):
                    measure_data[f"abd_add_{i+1}"] = val
            else:
                # Unrecognized DOF, use generic names
                for i, val in enumerate(values):
                    measure_data[f"motor_{i+1}"] = val
        else:
            # Unrecognized joint type, use generic names
            for i, val in enumerate(values):
                measure_data[f"motor_{i+1}"] = val

        return measure_data

    def handle_encoder_data(self, line: str) -> None:
        """
        Handles messages in format ENCODER_DATA:DOF=0:ANGLE=-50.00:COUNT=0

        Args:
            line (str): Message to handle
        """
        try:
            # Split string to extract values
            parts = line.split(":")
            if len(parts) < 4:
                logger.warning(f"Invalid encoder message format: {line}")
                return

            # Extract values from parameters
            dof_part = parts[1].split("=")
            angle_part = parts[2].split("=")
            count_part = parts[3].split("=")

            if len(dof_part) == 2 and len(angle_part) == 2 and len(count_part) == 2:
                dof = int(dof_part[1])
                angle = float(angle_part[1])
                count = int(count_part[1])

                # Print encoder values (debug level to avoid flooding logs during streaming)
                logger.debug(f"Encoder Data: DOF={dof}, Angle={angle}, Count={count}")

                # Determine active joint giving priority to ongoing encoder test
                active_joint = self.last_encoder_test_joint
                if not active_joint:
                    active_joint = self.current_active_joint
                if not active_joint:
                    active_joint = self.determine_active_joint_from_context()
                if not active_joint:
                    # Fallback: use last joint that made a PID request
                    active_joint = self.last_pid_request_joint or "KNEE_LEFT"

                # Initialize structure for joint if it doesn't exist
                if active_joint not in self.current_encoder_data:
                    self.current_encoder_data[active_joint] = {
                        "timestamp": 0,
                        "dof_positions": {},
                        "raw_values": {},
                        "is_active": False,
                    }

                # Update encoder data for this joint
                current_time = time.time()
                encoder_data = self.current_encoder_data[active_joint]
                encoder_data["timestamp"] = current_time
                encoder_data["dof_positions"][str(dof)] = angle
                encoder_data["raw_values"][str(dof)] = count
                encoder_data["is_active"] = True

                # Emit event for UI (kept for compatibility)
                self.socketio.emit(
                    "encoder_data",
                    {
                        "joint": active_joint,
                        "dof": dof,
                        "angle": angle,
                        "count": count,
                        "timestamp": current_time,
                    },
                    namespace="/movement",
                )

                # Update legacy state (kept for compatibility)
                joint_key = f"ENCODER_DOF_{dof}"
                if joint_key not in self.joint_status:
                    self.joint_status[joint_key] = {}

                self.joint_status[joint_key]["angle"] = angle
                self.joint_status[joint_key]["count"] = count

                logger.debug(
                    f"Encoder data stored for {active_joint} DOF {dof}: angle={angle}, count={count}"
                )

            else:
                logger.warning(f"Invalid encoder parameters format: {line}")

        except Exception as e:
            logger.error(f"Error handling encoder message: {e}")

    def handle_automatic_mapping_data(
        self, total_points: int, dof_count: int, ser: serial.Serial
    ) -> None:
        """
        Handles new automatic mapping protocol MAPPING_DATA(total_points,dof_count)

        Args:
            total_points (int): Total number of mapping points
            dof_count (int): Number of degrees of freedom for the joint
            ser (serial.Serial): Serial connection to read data
        """
        try:
            # Initialize data structures to store mapping data
            mapping_data = self.initialize_mapping_data_structure(
                total_points, dof_count
            )

            logger.info(
                f"Starting mapping data reception: {total_points} points, {dof_count} DOF"
            )

            # Read all mapping points
            for point_idx in range(total_points):
                line = ser.readline().decode().strip()
                if not line:
                    logger.warning(f"Riga vuota ricevuta al punto {point_idx}")
                    continue

                # Validation: process data only if they have the EVT: prefix
                if not line.startswith("EVT:"):
                    logger.debug(
                        f"Ignorata riga senza prefisso EVT: al punto {point_idx}: {line[:50]}..."
                    )
                    continue

                # Remove EVT: prefix for processing
                clean_line = line[4:]  # Remove 'EVT:'
                logger.debug(
                    f"Processing EVT: line point {point_idx}: {clean_line[:50]}..."
                )

                # Update raw data received counter (using cleaned line)
                if re.match(r"^[abc]\d+_\d+_[-+]?\d*\.?\d+\|", clean_line):
                    self.received_raw_data_points += 1
                    logger.debug(
                        f"📊 Raw data received: {self.received_raw_data_points}/{self.expected_raw_data_points}"
                    )

                # Parse mapping point line (using cleaned line)
                point_data = self.parse_mapping_point_line(
                    clean_line, point_idx, dof_count
                )
                if point_data:
                    # Store data in structure
                    self.store_mapping_point_data(mapping_data, point_data, point_idx)
                else:
                    logger.error(
                        f"Error parsing point {point_idx}: {clean_line}"
                    )

            # Deactivate raw data reception mode
            self.is_receiving_raw_mapping_data = False
            logger.debug(
                f"🔄 Raw data reception mode DEACTIVATED: received {self.received_raw_data_points}/{self.expected_raw_data_points} points"
            )

            # Validate and process received data
            if self.validate_mapping_data(mapping_data, total_points, dof_count):
                self.process_automatic_mapping_data(
                    mapping_data, total_points, dof_count, ser
                )
                logger.info(
                    "Automatic mapping data received and processed successfully"
                )
            else:
                logger.error("Mapping data validation failed")

        except Exception as e:
            logger.error(f"Error handling automatic mapping data: {e}")
            # Make sure to deactivate mode even in case of error
            self.is_receiving_raw_mapping_data = False

    def initialize_mapping_data_structure(
        self, total_points: int, dof_count: int
    ) -> Dict:
        """
        Initializes data structure to store mapping data

        Args:
            total_points (int): Total number of points
            dof_count (int): Number of DOFs

        Returns:
            Dict: Initialized data structure
        """
        mapping_data = {
            "total_points": total_points,
            "dof_count": dof_count,
            "points": [],
        }

        # Initialize arrays for each DOF
        for dof in range(dof_count):
            mapping_data[f"dof_{dof}"] = {
                "joint_angles": [None] * total_points,
                "agonist_angles": [None] * total_points,
                "antagonist_angles": [None] * total_points,
            }

        return mapping_data

    def parse_mapping_point_line(
        self, line: str, point_idx: int, dof_count: int
    ) -> Optional[Dict]:
        """
        Parses mapping data line in new format
        Format: a{dof}_{point}_{angle}|b{dof}_{point}_{angle}|c{dof}_{point}_{angle}|...

        Args:
            line (str): Line to parse (already cleaned, without EVT: prefix)
            point_idx (int): Expected point index
            dof_count (int): Expected number of DOFs

        Returns:
            Optional[Dict]: Parsed data or None if error
        """
        try:
            # Remove whitespace and line ending characters
            line = line.strip()

            # Ignora righe vuote
            if not line:
                return None

            # Divide the line by pipe separator and remove empty tokens
            tokens = [token.strip() for token in line.split("|") if token.strip()]

            # Verify that we have at least 3 tokens (a complete DOF)
            if len(tokens) < 3:
                logger.warning(
                    f"Insufficient number of tokens: received {len(tokens)}, minimum 3"
                )
                return None

            # Log for debug: show the actually present tokens
            logger.debug(f"Point {point_idx}: {len(tokens)} tokens received")

            # Determine which DOFs are actually present in data
            present_dofs = set()
            for token in tokens:
                parsed_token = self.parse_mapping_token(token, point_idx)
                if parsed_token:
                    present_dofs.add(parsed_token["dof"])

            # Verify that the tokens are multiples of 3 for the present DOFs
            expected_tokens_for_present_dofs = len(present_dofs) * 3
            if len(tokens) != expected_tokens_for_present_dofs:
                logger.warning(
                    f"Token non coerenti: {len(tokens)} token per {len(present_dofs)} DOF (attesi {expected_tokens_for_present_dofs})"
                )
                # Continue parsing, it might be a valid case

            point_data = {}

            # Parse each token
            for token in tokens:
                parsed_token = self.parse_mapping_token(token, point_idx)
                if parsed_token:
                    dof = parsed_token["dof"]
                    value_type = parsed_token["type"]  # 'a', 'b', 'c'
                    angle = parsed_token["angle"]

                    if dof not in point_data:
                        point_data[dof] = {}

                    # Map types to descriptive names according to unified standard:
                    # a = Joint angle for the specified DOF
                    # b = Agonist motor angle for the specified DOF
                    # c = Antagonist motor angle for the specified DOF
                    if value_type == "a":
                        point_data[dof]["joint_angle"] = angle
                    elif value_type == "b":
                        point_data[dof]["agonist_angle"] = angle
                    elif value_type == "c":
                        point_data[dof]["antagonist_angle"] = angle
                else:
                    logger.error(f"Error parsing token: {token}")
                    return None

            # Verify that each present DOF has all 3 values (a, b, c)
            for dof, dof_data in point_data.items():
                required_keys = ["joint_angle", "agonist_angle", "antagonist_angle"]
                missing_keys = [key for key in required_keys if key not in dof_data]
                if missing_keys:
                    logger.warning(
                        f"DOF {dof} punto {point_idx}: valori mancanti: {missing_keys}"
                    )

            return point_data

        except Exception as e:
            logger.error(f"Error parsing mapping line: {e}")
            return None

    def parse_mapping_token(self, token: str, expected_point: int) -> Optional[Dict]:
        """
        Parses single token in format: {type}{dof}_{point}_{angle}
        Note: token should already be cleaned (without EVT: prefix)

        Args:
            token (str): Token to parse (already cleaned)
            expected_point (int): Expected point index

        Returns:
            Optional[Dict]: Parsed token data or None if error
        """
        try:
            # Pattern regex for the new format: {type}{dof}_{point}_{angle}
            match = re.match(r"([abc])(\d+)_(\d+)_([-+]?\d*\.?\d+)", token)
            if not match:
                logger.error(f"Invalid token format: {token}")
                return None

            value_type = match.group(1)  # 'a', 'b', 'c'
            dof = int(match.group(2))
            point = int(match.group(3))
            angle = float(match.group(4))

            # Verify that the point corresponds to the expected one
            if point != expected_point:
                logger.warning(
                    f"Point index mismatch: expected {expected_point}, received {point}"
                )

            return {"type": value_type, "dof": dof, "point": point, "angle": angle}

        except Exception as e:
            logger.error(f"Error parsing token {token}: {e}")
            return None

    def store_mapping_point_data(
        self, mapping_data: Dict, point_data: Dict, point_idx: int
    ) -> None:
        """
        Stores a point's data in mapping structure

        Args:
            mapping_data (Dict): Mapping data structure
            point_data (Dict): Point data to store
            point_idx (int): Point index
        """
        try:
            for dof, dof_data in point_data.items():
                dof_key = f"dof_{dof}"
                if dof_key in mapping_data:
                    if "joint_angle" in dof_data:
                        mapping_data[dof_key]["joint_angles"][point_idx] = dof_data[
                            "joint_angle"
                        ]
                    if "agonist_angle" in dof_data:
                        mapping_data[dof_key]["agonist_angles"][point_idx] = dof_data[
                            "agonist_angle"
                        ]
                    if "antagonist_angle" in dof_data:
                        mapping_data[dof_key]["antagonist_angles"][point_idx] = (
                            dof_data["antagonist_angle"]
                        )

        except Exception as e:
            logger.error(
                f"Error storing point data {point_idx}: {e}"
            )

    def validate_mapping_data(
        self, mapping_data: Dict, total_points: int, dof_count: int
    ) -> bool:
        """
        Validates received mapping data

        Args:
            mapping_data (Dict): Mapping data to validate
            total_points (int): Expected number of points
            dof_count (int): Expected number of DOFs

        Returns:
            bool: True if the data is valid
        """
        try:
            # Determine which DOFs are actually present in data
            present_dofs = []
            for dof in range(dof_count):
                dof_key = f"dof_{dof}"
                if dof_key in mapping_data:
                    present_dofs.append(dof)

            if not present_dofs:
                logger.error("No DOF found in the data")
                return False

            # Log the actually present DOFs
            logger.info(
                f"DOFs present in the data: {present_dofs} (declared {dof_count})"
            )

            # Verify only the DOFs that are actually present
            for dof in present_dofs:
                dof_key = f"dof_{dof}"
                dof_data = mapping_data[dof_key]

                # Verify that all arrays have the correct length
                for angle_type in [
                    "joint_angles",
                    "agonist_angles",
                    "antagonist_angles",
                ]:
                    if angle_type not in dof_data:
                        logger.error(f"Angle type {angle_type} missing for DOF {dof}")
                        return False

                    angles = dof_data[angle_type]
                    if len(angles) != total_points:
                        logger.error(
                            f"Incorrect array length for DOF {dof}, {angle_type}: expected {total_points}, received {len(angles)}"
                        )
                        return False

                    # Count valid values (not None)
                    valid_values = [x for x in angles if x is not None]
                    none_count = total_points - len(valid_values)

                    if none_count > 0:
                        logger.warning(
                            f"DOF {dof}, {angle_type}: {none_count} missing values on {total_points}"
                        )
                        # We do not consider this a fatal error, only a warning

            # Update actual dof_count in data structure
            mapping_data["actual_dof_count"] = len(present_dofs)
            mapping_data["present_dofs"] = present_dofs

            logger.info(
                f"Mapping data validation completed: {len(present_dofs)} valid DOF out of {dof_count} declared"
            )
            return True

        except Exception as e:
            logger.error(f"Error validating mapping data: {e}")
            return False

    def process_automatic_mapping_data(
        self, mapping_data: Dict, total_points: int, dof_count: int, ser: serial.Serial
    ) -> None:
        """
        Processes received automatic mapping data and automatically sends enriched data to PICO

        Args:
            mapping_data (Dict): Validated mapping data
            total_points (int): Number of points
            dof_count (int): Number of declared DOFs
            ser (serial.Serial): Serial connection to send enriched data
        """
        try:
            # Use the actually present DOFs
            actual_dof_count = mapping_data.get("actual_dof_count", dof_count)
            present_dofs = mapping_data.get("present_dofs", list(range(dof_count)))

            logger.info(f"Processing {actual_dof_count} actual DOF: {present_dofs}")

            # Update internal data structure for compatibility with existing code
            if present_dofs and 0 in present_dofs:
                # For first DOF, maintain compatibility with existing format
                dof_0_data = mapping_data["dof_0"]

                # Verify that the data of the first DOF is complete
                if (
                    dof_0_data["joint_angles"]
                    and dof_0_data["agonist_angles"]
                    and dof_0_data["antagonist_angles"]
                ):

                    # Remove None values for pandas compatibility
                    valid_indices = [
                        i
                        for i, val in enumerate(dof_0_data["joint_angles"])
                        if val is not None
                    ]

                    if valid_indices:
                        # Import pandas only if necessary and create DataFrame
                        try:
                            import pandas as pd

                            self.df_mapping = pd.DataFrame(
                                {
                                    "knee_angle": [
                                        dof_0_data["joint_angles"][i]
                                        for i in valid_indices
                                    ],
                                    "extensor_angle": [
                                        dof_0_data["agonist_angles"][i]
                                        for i in valid_indices
                                    ],
                                    "flexor_angle": [
                                        dof_0_data["antagonist_angles"][i]
                                        for i in valid_indices
                                    ],
                                }
                            )
                            logger.info(
                                f"DataFrame created with {len(valid_indices)} valid points"
                            )
                        except ImportError:
                            logger.warning(
                                "Pandas not available, skipping creation of legacy DataFrame"
                            )
                    else:
                        logger.warning("No valid points found for DOF 0")
                else:
                    logger.warning(
                        "Incomplete data for DOF 0, skipping creation of legacy DataFrame"
                    )
            else:
                logger.info("DOF 0 not present, skipping legacy compatibility")

            # Store all mapping data for future use
            self.automatic_mapping_data = mapping_data

            # Determine active joint and save data to file
            # First try to use current_active_joint if it was set from a mapping request
            active_joint = (
                self.current_active_joint or self.determine_active_joint_from_context()
            )

            # Add the joint name to the mapping data
            if active_joint:
                mapping_data["joint_name"] = active_joint
            if active_joint:
                logger.info(
                    f"Saving mapping data for active joint: {active_joint}"
                )

                # Notify UI to update active joint combo boxes
                self.socketio.emit(
                    "update_active_joint",
                    {
                        "joint": active_joint,
                        "dof_count": actual_dof_count,
                        "set_dof_to_all": True,  # Set DOF to "ALL" by default
                        "source": "automatic_mapping",
                    },
                    namespace="/movement",
                )

                success = self.save_mapping_data_to_file(active_joint, mapping_data)
                if success:
                    logger.info(
                        f"Mapping data saved successfully for {active_joint}"
                    )
                else:
                    logger.error(
                        f"Error saving mapping data for {active_joint}"
                    )

                # Enriched data is automatically saved by the frontend when the user visualizes the graphs
            else:
                logger.warning(
                    "Unable to determine the active joint, saving skipped"
                )

            # Enrich data with interpolation and extrapolation
            logger.info(
                "Enrichment of mapping data with interpolation and extrapolation..."
            )
            enriched_data = self.enrich_mapping_data_with_interpolation(mapping_data)

            # NOTE: Mapping data is NOT sent back to firmware
            # Firmware uses linear equations computed directly from raw auto-mapping data
            # Enriched data is only used for visualization in the host UI
            logger.info("ℹ️  Mapping data enriched for UI visualization (not sent to firmware)")

            # Emit event for user interface with actual data
            self.socketio.emit(
                "automatic_mapping_data",
                {
                    "total_points": total_points,
                    "dof_count": actual_dof_count,  # Use the actual count
                    "declared_dof_count": dof_count,  # Keep the declared count
                    "present_dofs": present_dofs,
                    "data": mapping_data,
                    "joint_name": active_joint,  # Add the joint name to the data
                    "timestamp": time.time(),
                },
                namespace="/movement",
            )

            # Log received data for debug
            logger.info("Automatic mapping data processed:")
            for dof in present_dofs:
                dof_data = mapping_data[f"dof_{dof}"]

                # Count only valid values (not None)
                valid_joint_angles = [
                    x for x in dof_data["joint_angles"] if x is not None
                ]
                valid_agonist_angles = [
                    x for x in dof_data["agonist_angles"] if x is not None
                ]
                valid_antagonist_angles = [
                    x for x in dof_data["antagonist_angles"] if x is not None
                ]

                logger.info(f"  DOF {dof}: {len(valid_joint_angles)} punti validi")
                if valid_joint_angles:
                    logger.info(
                        f"    Range angoli giunto: {min(valid_joint_angles):.2f} - {max(valid_joint_angles):.2f}"
                    )
                if valid_agonist_angles:
                    logger.info(
                        f"    Range angoli agonista: {min(valid_agonist_angles):.2f} - {max(valid_agonist_angles):.2f}"
                    )
                if valid_antagonist_angles:
                    logger.info(
                        f"    Range angoli antagonista: {min(valid_antagonist_angles):.2f} - {max(valid_antagonist_angles):.2f}"
                    )

        except Exception as e:
            logger.error(
                f"Error processing automatic mapping data: {e}"
            )
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def handle_mapping_request(self, line: str, ser: serial.Serial) -> None:
        """
        Handles mapping data requests from PICO in format MAPPING_REQUEST:joint_id:dof_count

        Args:
            line (str): Received line in format MAPPING_REQUEST:joint_id:dof_count
            ser (serial.Serial): Serial connection to send response
        """
        try:
            # Parse MAPPING_REQUEST:joint_id:dof_count command
            match = re.match(r"MAPPING_REQUEST:(\d+):(\d+)", line)
            if not match:
                logger.error(f"Invalid MAPPING_REQUEST command format: {line}")
                return

            joint_id = int(match.group(1))
            dof_count = int(match.group(2))

            logger.info(
                f"Mapping request received for joint ID {joint_id} with {dof_count} DOF"
            )

            # Convert joint ID to joint name
            joint_name = self.get_joint_name_from_id(joint_id)
            if not joint_name:
                logger.error(f"Unrecognized joint ID: {joint_id}")
                self.send_mapping_not_found_response(ser, joint_id, dof_count)
                return

            # Set active joint for this request
            self.current_active_joint = joint_name
            logger.info(f"Set active joint from request: {joint_name}")

            # Notify the UI to update the active joint combo boxes
            self.socketio.emit(
                "update_active_joint",
                {
                    "joint": joint_name,
                    "dof_count": dof_count,
                    "set_dof_to_all": True,  # Set DOF to "ALL" by default
                    "source": "mapping_request",
                },
                namespace="/movement",
            )

            # Load mapping data from file
            mapping_data = self.load_mapping_data_from_file(joint_name)
            if not mapping_data:
                logger.warning(
                    f"Mapping data not found for joint {joint_name}"
                )
                self.send_mapping_not_found_response(ser, joint_id, dof_count)
                return

            # Verify that the DOFs correspond
            stored_dof_count = mapping_data.get("dof_count", 0)
            if stored_dof_count != dof_count:
                logger.warning(
                    f"DOFs do not correspond: requested {dof_count}, stored {stored_dof_count}"
                )
                # Continue anyway, but adapt to the requested number

            # Enrich data with interpolation and extrapolation before sending
            logger.info(
                f"Enrichment of mapping data for request {joint_name}..."
            )
            enriched_data = self.enrich_mapping_data_with_interpolation(mapping_data)

            # NOTE: Mapping data is NOT sent back to firmware
            # Firmware uses linear equations computed directly from raw auto-mapping data
            # Enriched data is only for visualization in the host UI
            logger.info(f"ℹ️  Mapping data for {joint_name} ready for UI visualization (not sent to firmware)")

        except Exception as e:
            logger.error(f"Error handling mapping request: {e}")
            try:
                joint_id = int(line.split(":")[1]) if ":" in line else 0
                dof_count = int(line.split(":")[2]) if line.count(":") >= 2 else 1
                self.send_mapping_not_found_response(ser, joint_id, dof_count)
            except Exception:
                logger.error("Unable to send error response")

    def get_joint_name_from_id(self, joint_id: int) -> Optional[str]:
        """
        Convert the joint ID to joint name

        Args:
            joint_id (int): Numeric joint ID from firmware

        Returns:
            Optional[str]: Joint name or None if not found
        """
        # Mapping of the joint IDs to the joint names (based on the firmware configuration)
        # #define JOINT_NONE                  0
        # #define JOINT_KNEE_LEFT             1  // Left
        # #define JOINT_KNEE_RIGHT            2  // Right
        # #define JOINT_ANKLE_LEFT            3  // Left
        # #define JOINT_ANKLE_RIGHT           4  // Right
        # #define JOINT_HIP_LEFT              5  // Left
        # #define JOINT_HIP_RIGHT             6  // Right
        joint_id_mapping = {
            0: None,  # JOINT_NONE
            1: "KNEE_LEFT",
            2: "KNEE_RIGHT",
            3: "ANKLE_LEFT",
            4: "ANKLE_RIGHT",
            5: "HIP_LEFT",
            6: "HIP_RIGHT",
        }

        return joint_id_mapping.get(joint_id)

    def save_mapping_data_to_file(self, joint_name: str, mapping_data: Dict) -> bool:
        """
        Save the mapping data to file for a specific joint

        Args:
            joint_name (str): Joint name
            mapping_data (Dict): Mapping data to save

        Returns:
            bool: True if save was successful
        """
        try:
            # Create directory if it doesn't exist
            os.makedirs("mapping_data", exist_ok=True)

            # Name of the file based on the joint
            filename = f"mapping_data/{joint_name.lower()}_mapping.json"

            # Prepare data for saving
            save_data = {
                "joint_name": joint_name,
                "timestamp": datetime.now().isoformat(),
                "total_points": mapping_data.get("total_points", 0),
                "dof_count": mapping_data.get("dof_count", 0),
                "actual_dof_count": mapping_data.get("actual_dof_count", 0),
                "present_dofs": mapping_data.get("present_dofs", []),
                "mapping_data": {},
            }

            # Copy the mapping data for each present DOF
            present_dofs = mapping_data.get("present_dofs", [])
            for dof in present_dofs:
                dof_key = f"dof_{dof}"
                if dof_key in mapping_data:
                    save_data["mapping_data"][dof_key] = mapping_data[dof_key]

            # Save to JSON file
            with open(filename, "w") as f:
                json.dump(save_data, f, indent=2, ensure_ascii=False)

            logger.info(f"Mapping data saved for {joint_name} in {filename}")
            return True

        except Exception as e:
            logger.error(
                f"Error saving mapping data for {joint_name}: {e}"
            )
            return False

    def load_mapping_data_from_file(self, joint_name: str) -> Optional[Dict]:
        """
        Load the mapping data from file for a specific joint

        Args:
            joint_name (str): Joint name

        Returns:
            Optional[Dict]: Loaded mapping data or None if not found
        """
        try:
            filename = f"mapping_data/{joint_name.lower()}_mapping.json"

            if not os.path.exists(filename):
                logger.info(f"Mapping file not found: {filename}")
                return None

            # Load the data from the JSON file
            with open(filename, "r") as f:
                save_data = json.load(f)

            # Reconstruct the original data structure
            mapping_data = {
                "total_points": save_data.get("total_points", 0),
                "dof_count": save_data.get("dof_count", 0),
                "actual_dof_count": save_data.get("actual_dof_count", 0),
                "present_dofs": save_data.get("present_dofs", []),
                "joint_name": joint_name,
                "loaded_from_file": True,
                "file_timestamp": save_data.get("timestamp"),
            }

            # Copy the mapping data
            stored_mapping_data = save_data.get("mapping_data", {})
            for dof_key, dof_data in stored_mapping_data.items():
                mapping_data[dof_key] = dof_data

            logger.info(f"Mapping data loaded for {joint_name} from {filename}")
            logger.info(
                f"  Punti: {mapping_data['total_points']}, DOF: {mapping_data['actual_dof_count']}"
            )
            return mapping_data

        except Exception as e:
            logger.error(
                f"Error loading mapping data for {joint_name}: {e}"
            )
            return None


    def send_mapping_not_found_response(
        self, ser: serial.Serial, joint_id: int, dof_count: int
    ) -> None:
        """
        Sends data-not-found response to PICO

        Args:
            ser (serial.Serial): Serial connection
            joint_id (int): Joint ID
            dof_count (int): Number of requested DOFs
        """
        try:
            # Send error response that doesn't confuse protocol
            # We don't use MAPPING_DATA because it's reserved for PICO→Pi5
            response = f"MAPPING_NOT_FOUND({joint_id},{dof_count})"
            self.send_line_to_pico(response, ser)
            logger.info(f"Sent 'data not found' response for joint {joint_id}")
        except Exception as e:
            logger.error(f"Error sending data-not-found response: {e}")

    def send_line_to_pico(self, line: str, ser: serial.Serial) -> None:
        """
        Sends line to PICO with error handling and automatic CMD: prefix

        Args:
            line (str): Line to send (without prefix)
            ser (serial.Serial): Serial connection
        """
        try:
            self.send_command_with_prefix(line, ser)
        except Exception as e:
            logger.error(f"Error sending line to PICO: {e}")

    def determine_active_joint_from_context(self) -> Optional[str]:
        """
        Determines active joint from context (used when receiving mapping data)

        Returns:
            Optional[str]: Active joint name or None if not determinable
        """
        # Try to determine from last sent command or user interface
        # For now we use simple logic, can be extended

        # Search for the active joint in the states
        for joint_name in [
            "KNEE_LEFT",
            "KNEE_RIGHT",
            "ANKLE_LEFT",
            "ANKLE_RIGHT",
            "HIP_LEFT",
            "HIP_RIGHT",
        ]:
            if joint_name in self.joint_status:
                joint_data = self.joint_status[joint_name]
                # If the joint has active status or recent messages, consider it active
                if "message" in joint_data or any(
                    dof_data
                    for dof_data in joint_data.values()
                    if isinstance(dof_data, dict)
                ):
                    self.current_active_joint = joint_name
                    return joint_name

        # Default fallback - use the active joint stored or None
        return self.current_active_joint

    def calculate_linear_regression(
        self, x_data: List[float], y_data: List[float]
    ) -> Optional[Dict]:
        """
        Calculates linear regression using least squares method

        Args:
            x_data: Array of X values
            y_data: Array of Y values

        Returns:
            Dict with slope, intercept, r2, equation or None if error
        """
        try:
            if len(x_data) != len(y_data) or len(x_data) < 2:
                return None

            n = len(x_data)
            sum_x = sum(x_data)
            sum_y = sum(y_data)
            sum_xy = sum(x * y for x, y in zip(x_data, y_data))
            sum_xx = sum(x * x for x in x_data)
            # sum_yy = sum(y * y for y in y_data)  # Not used in the current calculation

            # Calculate slope and intercept
            slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x)
            intercept = (sum_y - slope * sum_x) / n

            # Calculate R²
            y_mean = sum_y / n
            ss_total = sum((y - y_mean) ** 2 for y in y_data)
            ss_residual = sum(
                (y - (slope * x + intercept)) ** 2 for x, y in zip(x_data, y_data)
            )
            r2 = 1 - (ss_residual / ss_total)

            return {
                "slope": slope,
                "intercept": intercept,
                "r2": r2,
                "equation": f"y = {slope:.4f}x + {intercept:.4f}",
                "r2_text": f"R² = {r2:.4f}",
            }
        except Exception as e:
            logger.error(f"Error calculating linear regression: {e}")
            return None

    def generate_interpolated_and_extrapolated_points(
        self,
        regression: Dict,
        min_x: float,
        max_x: float,
        step_size: float = None,
        extrapolation_percent: float = 10.0,
    ) -> Dict:
        """
        Generate interpolated and extrapolated points using the regression equation

        Args:
            regression: Linear regression result
            min_x: Minimum X-axis value from measured data
            max_x: Maximum X-axis value from measured data
            step_size: Interval between points (None = auto-calculate from data)
            extrapolation_percent: Percentage of extrapolation beyond limits (default 10%)

        Returns:
            Dict con interpolated, extrapolated, full points e extended_range
        """
        try:
            if not regression:
                return {
                    "interpolated": [],
                    "extrapolated": [],
                    "full": [],
                    "extended_range": {"min": min_x, "max": max_x},
                }

            # Auto-calculate step size if not specified
            if step_size is None:
                range_span = max_x - min_x
                # Use a step size that generates approximately 20-30 points in the original range
                # but not less than 1 degree and not more than 5 degrees
                calculated_step = range_span / 25  # Target: 25 points in the original range
                step_size = max(1.0, min(5.0, round(calculated_step)))
                logger.debug(
                    f"Step size auto-calcolato: {step_size}° (range: {range_span:.1f}°)"
                )

            # Calculate extrapolation range
            range_val = max_x - min_x
            extension = range_val * (extrapolation_percent / 100)

            extended_min_x = min_x - extension
            extended_max_x = max_x + extension

            # Round to nearest integer degrees for clean values
            rounded_extended_min_x = math.floor(extended_min_x)
            rounded_extended_max_x = math.ceil(extended_max_x)

            # Generate interpolated points (original range)
            interpolated_points = []
            x = min_x
            while x <= max_x:
                y = regression["slope"] * x + regression["intercept"]
                interpolated_points.append({"x": x, "y": y, "type": "interpolated"})
                x += step_size

            # Generate extrapolated points
            extrapolated_points = []

            # Downward extrapolation (minimum)
            x = rounded_extended_min_x
            while x < min_x:
                y = regression["slope"] * x + regression["intercept"]
                extrapolated_points.append({"x": x, "y": y, "type": "extrapolated_low"})
                x += step_size

            # Upward extrapolation (maximum)
            x = max_x + step_size
            while x <= rounded_extended_max_x:
                y = regression["slope"] * x + regression["intercept"]
                extrapolated_points.append(
                    {"x": x, "y": y, "type": "extrapolated_high"}
                )
                x += step_size

            # Combine all points and sort by x
            all_points = interpolated_points + extrapolated_points
            all_points.sort(key=lambda p: p["x"])

            return {
                "interpolated": interpolated_points,
                "extrapolated": extrapolated_points,
                "full": all_points,
                "extended_range": {
                    "min": rounded_extended_min_x,
                    "max": rounded_extended_max_x,
                    "original_min": min_x,
                    "original_max": max_x,
                    "extension": extension,
                },
            }
        except Exception as e:
            logger.error(f"Error generating interpolated/extrapolated points: {e}")
            return {
                "interpolated": [],
                "extrapolated": [],
                "full": [],
                "extended_range": {"min": min_x, "max": max_x},
            }

    def enrich_mapping_data_with_interpolation(self, mapping_data: Dict) -> Dict:
        """
        Enrich the mapping data with interpolated and extrapolated points

        Args:
            mapping_data: Original mapping data

        Returns:
            Enriched data with interpolation and extrapolation
        """
        try:
            if not mapping_data or not mapping_data.get("present_dofs"):
                return mapping_data

            # Create deep copy of original data
            enriched_data = json.loads(json.dumps(mapping_data))

            # Process each DOF
            present_dofs = mapping_data.get("present_dofs", [])

            for dof in present_dofs:
                dof_key = f"dof_{dof}"
                dof_data = mapping_data.get(dof_key)

                if not dof_data or not all(
                    key in dof_data
                    for key in ["joint_angles", "agonist_angles", "antagonist_angles"]
                ):
                    continue

                # Filter valid points
                valid_points = []
                for i in range(len(dof_data["joint_angles"])):
                    if (
                        i < len(dof_data["joint_angles"])
                        and i < len(dof_data["agonist_angles"])
                        and i < len(dof_data["antagonist_angles"])
                        and dof_data["joint_angles"][i] is not None
                        and dof_data["agonist_angles"][i] is not None
                        and dof_data["antagonist_angles"][i] is not None
                    ):

                        valid_points.append(
                            {
                                "joint": dof_data["joint_angles"][i],
                                "agonist": dof_data["agonist_angles"][i],
                                "antagonist": dof_data["antagonist_angles"][i],
                            }
                        )

                if len(valid_points) < 2:
                    logger.warning(
                        f"DOF {dof}: insufficient points for interpolation ({len(valid_points)})"
                    )
                    continue

                # Sort points by joint angle
                valid_points.sort(key=lambda p: p["joint"])

                joint_angles = [p["joint"] for p in valid_points]
                agonist_angles = [p["agonist"] for p in valid_points]
                antagonist_angles = [p["antagonist"] for p in valid_points]

                min_joint_angle = min(joint_angles)
                max_joint_angle = max(joint_angles)

                # Calculate linear regressions
                agonist_regression = self.calculate_linear_regression(
                    joint_angles, agonist_angles
                )
                antagonist_regression = self.calculate_linear_regression(
                    joint_angles, antagonist_angles
                )

                if agonist_regression and antagonist_regression:
                    # Calculate smart step size based on original data
                    original_steps = []
                    for i in range(len(joint_angles) - 1):
                        step = abs(joint_angles[i + 1] - joint_angles[i])
                        if step > 0:  # Evita step zero
                            original_steps.append(step)

                    if original_steps:
                        # Use median of original steps, rounded to nearest degree
                        median_step = sorted(original_steps)[len(original_steps) // 2]
                        smart_step_size = max(1.0, round(median_step))
                        logger.info(
                            f"DOF {dof}: Smart step size = {smart_step_size}° (original median: {median_step:.2f}°)"
                        )
                    else:
                        smart_step_size = None  # Use automatic calculation

                    # Generate interpolated and extrapolated points
                    agonist_extended = (
                        self.generate_interpolated_and_extrapolated_points(
                            agonist_regression,
                            min_joint_angle,
                            max_joint_angle,
                            smart_step_size,
                            10.0,
                        )
                    )
                    antagonist_extended = (
                        self.generate_interpolated_and_extrapolated_points(
                            antagonist_regression,
                            min_joint_angle,
                            max_joint_angle,
                            smart_step_size,
                            10.0,
                        )
                    )

                    # Add enriched data
                    enriched_data[dof_key]["interpolation"] = {
                        "agonist": {
                            "regression": agonist_regression,
                            "interpolated_points": agonist_extended["interpolated"],
                            "extrapolated_points": agonist_extended["extrapolated"],
                            "full_points": agonist_extended["full"],
                            "extended_range": agonist_extended["extended_range"],
                        },
                        "antagonist": {
                            "regression": antagonist_regression,
                            "interpolated_points": antagonist_extended["interpolated"],
                            "extrapolated_points": antagonist_extended["extrapolated"],
                            "full_points": antagonist_extended["full"],
                            "extended_range": antagonist_extended["extended_range"],
                        },
                        "metadata": {
                            "original_data_points": len(valid_points),
                            "interpolated_points_count": len(
                                agonist_extended["interpolated"]
                            ),
                            "extrapolated_points_count": len(
                                agonist_extended["extrapolated"]
                            ),
                            "total_points_count": len(agonist_extended["full"]),
                            "extrapolation_percent": 10.0,
                            "step_size": smart_step_size or "auto",
                            "original_median_step": (
                                median_step if original_steps else "N/A"
                            ),
                        },
                    }

                    logger.info(
                        f"DOF {dof}: Interpolation completed - {len(valid_points)} original, "
                        f"{len(agonist_extended['interpolated'])} interpolated, "
                        f"{len(agonist_extended['extrapolated'])} extrapolated "
                        f"(agonist R²: {agonist_regression['r2']:.4f}, "
                        f"antagonist R²: {antagonist_regression['r2']:.4f})"
                    )
                else:
                    logger.warning(
                        f"DOF {dof}: Unable to calculate linear regression"
                    )

            return enriched_data

        except Exception as e:
            logger.error(f"Error enriching mapping data: {e}")
            return mapping_data

    def get_output_data(self) -> Dict[str, float]:
        elapsed_time = (
            time.time() - self.start_measure_motor_output_time
            if self.start_measure_motor_output_time
            else 0
        )
        return {
            "time": elapsed_time,
            "extensor_output": self.extensor,
            "flexor_output": self.flexor,
        }

    def synchronize_time(self) -> Dict[str, float]:
        """
        Perform NTP-like time synchronization with firmware.
        
        Returns:
            Dict containing:
                - offset: calculated time offset (seconds)
                - rtt: round-trip time (seconds)
                - T1: host send time
                - T2: firmware receive time
                - T3: firmware send time (same as T2)
                - T4: host receive time
                - success: True if sync successful
        """
        result = {
            "success": False,
            "offset": 0.0,
            "offset_ms": 0.0,
            "offset_drift_ms": None,
            "rtt": 0.0,
            "rtt_ms": 0.0,
            "T1": 0.0,
            "T2": 0.0,
            "T3": 0.0,
            "T4": 0.0,
            "host_epoch_send": 0.0,
            "host_epoch_receive": 0.0,
            "host_midpoint_epoch": 0.0,
            "host_midpoint_seconds": 0.0,
            "host_midpoint_iso": None,
            "firmware_boot_epoch": 0.0,
            "firmware_boot_iso": None,
            "firmware_uptime_seconds": 0.0,
            "is_baseline": False,
            "error": None
        }
        
        try:
            # Pause listening thread and wait for it to release the serial port
            self.pause_listening()
            time.sleep(0.2)  # Give listener time to exit the 'with' block
            
            if self.time_sync_reference_clock is None:
                self.time_sync_reference_clock = time.perf_counter()
            
            # Open a temporary dedicated connection for sync
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1.0) as ser:
                # Clear any pending data
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                
                # T1: Send SYNC command with host timestamp
                host_epoch_send = time.time()
                host_clock_send = time.perf_counter()
                T1 = host_clock_send - self.time_sync_reference_clock
                result["T1"] = T1
                result["host_epoch_send"] = host_epoch_send
                command = f"CMD:SYNC({T1:.6f})\n"
                ser.write(command.encode())
                ser.flush()  # Ensure data is sent immediately
                logger.info(f"Sent SYNC command at T1={T1:.6f}s (relative)")
                
                # Wait for response
                response_received = False
                timeout_start = time.perf_counter()
                
                while time.perf_counter() - timeout_start < 1.0:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        logger.debug(f"Sync received: {line}")
                        
                        # T4: Record receive time immediately
                        host_clock_receive = time.perf_counter()
                        T4 = host_clock_receive - self.time_sync_reference_clock
                        
                        # Parse EVT:SYNC_RESPONSE(T1_echo,T2)
                        if line.startswith("EVT:SYNC_RESPONSE("):
                            result["T4"] = T4
                            try:
                                # Extract T1_echo and T2 from response
                                values_str = line[18:-1]  # Remove "EVT:SYNC_RESPONSE(" and ")"
                                T1_echo, T2 = map(float, values_str.split(','))
                                result["T2"] = T2
                                result["T3"] = T2  # T3 = T2 for immediate response
                                
                                # Calculate offset using NTP algorithm
                                # offset = ((T2 - T1) + (T3 - T4)) / 2
                                # Since T3 = T2, this simplifies to T2 - (T1 + T4) / 2
                                host_round_trip = T4 - T1
                                host_midpoint = T1 + host_round_trip / 2.0
                                offset_seconds = T2 - host_midpoint

                                result["offset"] = offset_seconds
                                result["offset_ms"] = offset_seconds * 1000.0
                                result["rtt"] = host_round_trip
                                result["rtt_ms"] = host_round_trip * 1000.0
                                result["firmware_uptime_seconds"] = T2
                                result["host_midpoint_seconds"] = host_midpoint

                                host_epoch_receive = host_epoch_send + (host_round_trip)
                                result["host_epoch_receive"] = host_epoch_receive
                                host_midpoint_epoch = host_epoch_send + (host_round_trip / 2.0)
                                result["host_midpoint_epoch"] = host_midpoint_epoch

                                firmware_boot_epoch = host_midpoint_epoch - T2
                                result["firmware_boot_epoch"] = firmware_boot_epoch
                                try:
                                    result["firmware_boot_iso"] = datetime.fromtimestamp(firmware_boot_epoch).isoformat()
                                    result["host_midpoint_iso"] = datetime.fromtimestamp(host_midpoint_epoch).isoformat()
                                except (OverflowError, OSError, ValueError):
                                    result["firmware_boot_iso"] = None
                                    result["host_midpoint_iso"] = None

                                if self.time_sync_baseline_offset is None:
                                    self.time_sync_baseline_offset = offset_seconds
                                    result["offset_drift_ms"] = 0.0
                                    result["is_baseline"] = True
                                else:
                                    drift_ms = (offset_seconds - self.time_sync_baseline_offset) * 1000.0
                                    result["offset_drift_ms"] = drift_ms
                                    result["is_baseline"] = False

                                result["success"] = True
                                response_received = True
                                
                                logger.info(
                                    f"Time sync successful: offset={offset_seconds:.6f}s (drift={result['offset_drift_ms'] or 0:.3f}ms), "
                                    f"RTT={host_round_trip:.6f}s"
                                )
                                break
                                
                            except (ValueError, IndexError) as e:
                                result["error"] = f"Failed to parse response: {e}"
                                logger.error(result["error"])
                                break
                    
                    time.sleep(0.01)  # Small delay to avoid busy-waiting
                
                if not response_received:
                    result["error"] = "Timeout waiting for SYNC_RESPONSE"
                    logger.error(result["error"])

        except serial.SerialException as e:
            result["error"] = f"Serial communication error: {e}"
            logger.error(result["error"])
        except Exception as e:
            result["error"] = f"Unexpected error: {e}"
            logger.error(result["error"])
        finally:
            # Resume listening thread
            self.resume_listening()
            
        return result

    def generate_command(self, joint, dof, command, params=None):
        """
        Generates command in new format JOINT:DOF:COMMAND:PARAMS

        Args:
            joint (str): Joint name (KNEE, ANKLE, HIP)
            dof (str/int): DOF index (0, 1) or 'ALL' for all
            command (str): Command name
            params (list, optional): Optional list of parameters

        Returns:
            str: Formatted command
        """
        cmd = f"{joint}:{dof}:{command}"

        try:
            if params:
                # Filter invalid parameters
                valid_params = []
                for p in params:
                    if p is None or p == "":
                        continue
                    valid_params.append(str(p))

                if valid_params:
                    param_str = ":".join(valid_params)
                    cmd += f":{param_str}"
        except Exception as e:
            logger.error(f"Error formatting parameters: {e}")

        return cmd

    def send_new_command(
        self, joint, dof, command, params=None, retries=3, timeout=1000
    ):
        """
        Sends command in new format with timeout and retry support

        Args:
            joint (str): Joint name (KNEE, ANKLE, HIP)
            dof (str/int): DOF index (0, 1) or 'ALL' for all
            command (str): Command name
            params (list, optional): Optional list of parameters
            retries (int): Number of retries in case of failure
            timeout (int): Timeout in milliseconds

        Returns:
            bool: True if command was sent successfully
        """
        cmd = self.generate_command(joint, dof, command, params)
        try:
            # Track active joint when sending mapping commands
            if command in ["START_AUTO_MAPPING", "MAPPING"]:
                self.current_active_joint = joint
                logger.info(f"Set active joint: {joint}")

                # Notify UI that active joint has changed
                self.socketio.emit(
                    "update_active_joint",
                    {
                        "joint": joint,
                        "set_dof_to_all": dof
                        == "ALL",  # Keep current DOF if not "ALL"
                        "source": "user_command",
                    },
                    namespace="/movement",
                )
            elif command == COMMANDS.get("START_TEST_ENCODER"):
                # Track joint associated with encoder test to correctly assign data
                self.last_encoder_test_joint = joint
                self.current_active_joint = joint
                logger.info(f"Test encoder started: set active joint {joint}")
            elif command == COMMANDS.get("STOP_TEST_ENCODER"):
                logger.info(f"Test encoder stopped for joint {joint}")
                if self.last_encoder_test_joint == joint:
                    # Keep information until new test starts, so UI can read last data
                    logger.debug("Last encoder test joint unchanged")

            self.pause_listening()
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1) as ser:
                self.send_command_with_prefix(cmd, ser)

                # If we're stopping a measurement or mapping command, also send STOP command
                if command in ["STOP_MEASURE", "STOP_TEST_ENCODER", "STOP_AUTO_MAPPING"]:
                    # Send STOP command to interrupt ongoing operations
                    stop_cmd = f"{joint}:{dof}:STOP"
                    self.send_command_with_prefix(stop_cmd, ser)
                    logger.info(f"STOP command sent to interrupt operation: CMD:{stop_cmd}")

                    # Also reset error counters
                    self.last_error_message = ""
                    self.error_count = 0

        except Exception as e:
            logger.error(f"Error sending command {cmd}: {e}")
            return False
        finally:
            self.resume_listening()

        return True

    def handle_joint_message(self, line: str) -> None:
        """
        Handles messages in format JOINT:DOF:MESSAGE

        Args:
            line (str): Message to handle
        """
        try:
            parts = line.split(":")
            if len(parts) < 3:
                logger.warning(f"Invalid message format: {line}")
                return

            joint = parts[0]  # KNEE, ANKLE, HIP
            dof = parts[1]  # 0, 1, ALL
            msg_type = parts[2]

            # Update joint state
            if joint not in self.joint_status:
                self.joint_status[joint] = {}

            if dof != "ALL":
                if dof not in self.joint_status[joint]:
                    self.joint_status[joint][dof] = {}

                if msg_type == "POSITION":
                    # Format: JOINT:DOF:POSITION:value
                    if len(parts) >= 4:
                        position = float(parts[3])
                        self.joint_status[joint][dof]["position"] = position
                elif msg_type == "MEASURE":
                    # Format: JOINT:DOF:MEASURE:value1|value2|...
                    if len(parts) >= 4:
                        # Splitting values by pipe for multiple motors
                        values = parts[3].split("|")
                        value_dict = {}

                        # Modify from KNEE to KNEE_LEFT/KNEE_RIGHT
                        joint_type = joint.split("_")[0]  # Extract KNEE, ANKLE, HIP

                        if joint_type == "KNEE":
                            if len(values) >= 1:
                                value_dict["extensor"] = float(values[0])
                            if len(values) >= 2:
                                value_dict["flexor"] = float(values[1])
                        elif joint_type == "ANKLE":
                            try:
                                dof_index = int(dof) if dof != "ALL" else 0
                                if dof_index == 0:  # plantar-dorsal
                                    if len(values) >= 1:
                                        value_dict["plantar"] = float(values[0])
                                    if len(values) >= 2:
                                        value_dict["dorsal"] = float(values[1])
                                elif dof_index == 1:  # inversion-eversion
                                    if len(values) >= 1:
                                        value_dict["inversion"] = float(values[0])
                                    if len(values) >= 2:
                                        value_dict["eversion"] = float(values[1])
                            except ValueError:
                                # Handles case where dof is not convertible to int (e.g. 'ALL')
                                logger.warning(f"Non-numeric DOF for ANKLE: {dof}")
                        elif joint_type == "HIP":
                            try:
                                dof_index = int(dof) if dof != "ALL" else 0
                                if dof_index == 0:  # flexion-extension
                                    for i, val in enumerate(values):
                                        value_dict[f"flex_ext_{i+1}"] = float(val)
                                elif dof_index == 1:  # abduction-adduction
                                    for i, val in enumerate(values):
                                        value_dict[f"abd_add_{i+1}"] = float(val)
                            except ValueError:
                                # Handles case where dof is not convertible to int (e.g. 'ALL')
                                logger.warning(f"Non-numeric DOF for HIP: {dof}")

                        self.joint_status[joint][dof]["measure"] = value_dict

                        # Emit data for real-time chart
                        self.socketio.emit(
                            "joint_measure",
                            {
                                "joint": joint,
                                "dof": dof,
                                "data": value_dict,
                                "timestamp": time.time(),
                            },
                            namespace="/movement",
                        )

                elif msg_type == "ERROR":
                    # Format: JOINT:DOF:ERROR:message
                    if len(parts) >= 4:
                        error_msg = parts[3]
                        self.joint_status[joint][dof]["error"] = error_msg
                        logger.error(
                            f"Error from joint {joint} DOF {dof}: {error_msg}"
                        )
            else:
                # General messages for entire joint
                if msg_type == "STATUS":
                    # Format: JOINT:ALL:STATUS:message
                    if len(parts) >= 4:
                        status_msg = parts[3]
                        self.joint_status[joint]["message"] = status_msg
        except Exception as e:
            logger.error(f"Error handling joint message: {e}")

    def get_pid_for_joint_dof(self, joint, dof, motor_type):
        """
        Requests PID parameters for a specific DOF of a joint and motor type

        Args:
            joint (str): Joint name
            dof (int/str): DOF index or 'ALL'
            motor_type (int): Motor type (1=agonist, 2=antagonist)
        """
        # Store joint for which we're requesting PIDs
        self.last_pid_request_joint = joint

        # Check if DOF is 'ALL' and in that case use 0 for request
        dof_to_use = 0 if dof == "ALL" else dof

        cmd = self.generate_command(
            joint, dof_to_use, COMMANDS["GET_PID"], [motor_type]
        )
        try:
            self.pause_listening()
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1) as ser:
                self.send_command_with_prefix(cmd, ser)
                logger.info(f"PID request sent: CMD:{cmd}")
        except Exception as e:
            logger.error(f"Error requesting PID {cmd}: {e}")
        finally:
            self.resume_listening()

    def get_outer_pid_for_joint_dof(self, joint, dof):
        """Requests outer loop PID parameters for a specific DOF."""
        self.last_pid_request_joint = joint

        dof_to_use = 0 if dof == "ALL" else int(dof)

        cmd = self.generate_command(
            joint, dof_to_use, COMMANDS["GET_PID_OUTER"], []
        )

        try:
            self.pause_listening()
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1) as ser:
                self.send_command_with_prefix(cmd, ser)
                logger.info(f"Outer PID request sent: CMD:{cmd}")
        except Exception as e:
            logger.error(f"Error requesting outer PID {cmd}: {e}")
        finally:
            self.resume_listening()

    def set_pid_for_joint_dof(self, joint, dof, motor_type, kp, ki, kd, tau):
        """
        Sets PID parameters for a specific DOF of a joint and motor type

        Args:
            joint (str): Joint name
            dof (int/str): DOF index or 'ALL'
            motor_type (int): Motor type (1=agonist, 2=antagonist)
            kp (float): Proportional parameter
            ki (float): Integral parameter
            kd (float): Derivative parameter
            tau (float): Time constant of the filter
        """
        # Check if DOF is 'ALL' and in that case use 0 for setting
        dof_to_use = 0 if dof == "ALL" else dof

        cmd = self.generate_command(
            joint, dof_to_use, COMMANDS["SET_PID"], [motor_type, kp, ki, kd, tau]
        )
        try:
            self.pause_listening()
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1) as ser:
                self.send_command_with_prefix(cmd, ser)
                logger.info(f"PID setting sent: CMD:{cmd}")
        except Exception as e:
            logger.error(f"Error setting PID {cmd}: {e}")
        finally:
            self.resume_listening()

    def set_outer_pid_for_joint_dof(self, joint, dof, kp, ki, kd, stiffness_deg, cascade_influence):
        """Sets outer loop parameters for a specific DOF."""
        dof_to_use = 0 if dof == "ALL" else int(dof)

        cmd = self.generate_command(
            joint,
            dof_to_use,
            COMMANDS["SET_PID_OUTER"],
            [kp, ki, kd, stiffness_deg, cascade_influence],
        )

        try:
            self.pause_listening()
            with serial.Serial(self.serial_port, BAUD_RATE, timeout=1) as ser:
                self.send_command_with_prefix(cmd, ser)
                logger.info(f"Outer PID setting sent: CMD:{cmd}")
        except Exception as e:
            logger.error(f"Error setting outer PID {cmd}: {e}")
        finally:
            self.resume_listening()

    def add_cmd_prefix(self, command: str) -> str:
        """
        Adds CMD: prefix to a command if not already present

        Args:
            command (str): Command to prefix

        Returns:
            str: Command with prefix CMD:
        """
        if not command.startswith("CMD:"):
            return f"CMD:{command}"
        return command

    def remove_evt_prefix(self, message: str) -> str:
        """
        Removes EVT: prefix from a message if present

        Args:
            message (str): Message to remove the prefix from

        Returns:
            str: Message without prefix EVT:
        """
        if message.startswith("EVT:"):
            return message[4:]  # Remove "EVT:"
        return message

    def send_command_with_prefix(self, command: str, ser: serial.Serial) -> None:
        """
        Sends command to PICO with automatic CMD: prefix

        Args:
            command (str): Command to send (without prefix)
            ser (serial.Serial): Serial connection
        """
        try:
            prefixed_command = self.add_cmd_prefix(command)

            # Verify command length
            command_length = len(prefixed_command.encode())
            MAX_COMMAND_LENGTH = (
                128  # Maximum buffer size of the PICO (increased from 64)
            )
            if command_length > MAX_COMMAND_LENGTH:
                error_msg = f"Command length {command_length} exceeds buffer size {MAX_COMMAND_LENGTH}."
                logger.error(error_msg)
                self.serial_logger.log_error(error_msg)
                raise ValueError(error_msg)

            ser.write(prefixed_command.encode() + b"\n")
            ser.flush()
            # No delay needed after flush - flush() already waits for transmission

            # Log sent message
            self.serial_logger.log_sent_message(prefixed_command)
            logger.debug(f"Sent command: {prefixed_command}")
        except Exception as e:
            error_msg = f"Error sending command {command}: {e}"
            logger.error(error_msg)
            self.serial_logger.log_error(error_msg)
