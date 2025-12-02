"""
Flask API routes for the Joint Controller host application.

This module defines all HTTP endpoints for:
- Joint control commands (movement, PID tuning, calibration)
- Mapping data management (load, save, visualize)
- Serial communication (port assignment, status)
- System configuration (joint limits, available commands)
"""
from flask import jsonify, request, current_app, render_template
from typing import Dict, Any, List, Optional
from serial_manager import SerialManager
from config import JOINTS, MIN_ANGLES, MAX_ANGLES, COMMANDS
import json
import os
import time
import utils
from pathlib import Path
import logging

# CAN interface detection
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

logger = logging.getLogger(__name__)

def register_routes(app, serial_manager: SerialManager, can_manager=None):
    """
    Register all Flask routes for the joint controller application.
    
    Args:
        app: Flask application instance
        serial_manager: SerialManager instance for hardware communication
    """

    def handler_or_error(joint: str):
        handler = serial_manager.get_handler_for_joint(joint)
        if handler is None:
            return None, (
                jsonify({
                    "status": "error",
                    "message": f"No serial port associated with joint {joint}.",
                    "joint": joint,
                }),
                400,
            )
        return handler, None, None

    def can_unavailable_response():
        if not CAN_AVAILABLE or can_manager is None:
            return jsonify({
                "status": "error",
                "message": "CAN features not available on this host (python-can missing or disabled)."
            }), 503
        return None

    def load_mapping_from_file(joint_name: str):
        filename = f"mapping_data/{joint_name.lower()}_mapping.json"
        if not os.path.exists(filename):
            return None

        with open(filename, "r") as file_obj:
            saved = json.load(file_obj)

        mapping_data = {
            "total_points": saved.get("total_points", 0),
            "dof_count": saved.get("dof_count", 0),
            "actual_dof_count": saved.get("actual_dof_count", 0),
            "present_dofs": saved.get("present_dofs", []),
            "joint_name": joint_name,
            "loaded_from_file": True,
            "file_timestamp": saved.get("timestamp"),
        }

        for dof_key, dof_data in saved.get("mapping_data", {}).items():
            mapping_data[dof_key] = dof_data

        return mapping_data

    @app.route('/serial_ports', methods=['GET'])
    def list_serial_ports():
        return jsonify({
            "status": "success",
            "ports": serial_manager.list_available_ports(),
            "mappings": serial_manager.get_joint_to_port_mapping(),
        })

    @app.route('/serial_mapping', methods=['POST'])
    def assign_serial_mapping():
        data = request.json or {}
        joint = data.get('joint')
        port = data.get('port')

        if not joint:
            return jsonify({
                "status": "error",
                "message": "Specify joint to associate.",
            }), 400

        normalized_port = port or None

        try:
            result = serial_manager.assign_port_to_joint(joint, normalized_port)
        except Exception as exc:
            current_app.logger.exception("Error during serial assignment")
            return jsonify({
                "status": "error",
                "message": f"Cannot assign port {port}: {exc}",
            }), 400

        return jsonify({
            "status": "success",
            "result": result,
            "mappings": serial_manager.get_joint_to_port_mapping(),
        })

    @app.route('/can_interfaces', methods=['GET'])
    def list_can_interfaces():
        """
        List available CAN interfaces detected on the system.
        
        Returns:
            JSON with available CAN interfaces or error if python-can not available
        """
        if not CAN_AVAILABLE:
            return jsonify({
                "status": "error",
                "message": "python-can library not installed",
                "interfaces": []
            }), 503
        
        try:
            # Detect available CAN configurations
            configs = can.detect_available_configs()
            
            # Format interfaces for UI
            interfaces = []
            for config in configs:
                interface_type = config.get('interface', 'unknown')
                channel = config.get('channel', 'N/A')
                
                # Create a display name
                if interface_type == 'slcan':
                    # Extract port name from channel for SLCAN
                    display_name = f"SLCAN ({channel.split('/')[-1] if channel != 'N/A' else 'N/A'})"
                else:
                    display_name = f"{interface_type.upper()} ({channel})"
                
                interfaces.append({
                    "interface": interface_type,
                    "channel": channel,
                    "display_name": display_name,
                    "value": json.dumps(config)  # Serialize config for later use
                })
            
            return jsonify({
                "status": "success",
                "interfaces": interfaces,
                "count": len(interfaces)
            })
        
        except Exception as e:
            logger.exception("Error detecting CAN interfaces")
            return jsonify({
                "status": "error",
                "message": f"Error detecting CAN interfaces: {str(e)}",
                "interfaces": []
            }), 500

    @app.route('/can_test_init', methods=['POST'])
    def test_can_init():
        """
        Test CAN bus initialization with selected interface.
        
        Expected JSON:
            {
                "config": "{\"interface\":\"slcan\",\"channel\":\"/dev/cu.usbmodem...\"}"
            }
        
        Returns:
            JSON with initialization result and bus information
        """
        if not CAN_AVAILABLE:
            return jsonify({
                "status": "error",
                "message": "python-can library not installed"
            }), 503
        
        try:
            data = request.get_json()
            config_str = data.get('config')
            
            if not config_str:
                return jsonify({
                    "status": "error",
                    "message": "No CAN interface config provided"
                }), 400
            
            # Parse config
            config = json.loads(config_str)
            interface = config.get('interface')
            channel = config.get('channel')
            
            if not interface or not channel:
                return jsonify({
                    "status": "error",
                    "message": "Invalid config: missing interface or channel"
                }), 400
            
            # Try to initialize CAN bus
            logger.info(f"Initializing CAN bus: {interface} on {channel} @ 1 Mbps")
            
            bus = can.Bus(
                interface=interface,
                channel=channel,
                bitrate=1000000  # 1 Mbps
            )
            
            # Get bus info
            bus_info = {
                "interface": interface,
                "channel": channel,
                "bitrate": "1 Mbps",
                "channel_info": str(bus.channel_info) if hasattr(bus, 'channel_info') else "N/A"
            }
            
            # Try to receive (non-blocking, 0.5s timeout)
            logger.info("Testing message reception (0.5s timeout)...")
            msg = bus.recv(timeout=0.5)
            
            if msg:
                bus_info["test_message"] = {
                    "arbitration_id": f"0x{msg.arbitration_id:03X}",
                    "data": msg.data.hex(),
                    "timestamp": msg.timestamp
                }
                logger.info(f"Received CAN message: {msg}")
            else:
                bus_info["test_message"] = None
                logger.info("No CAN messages received (normal if no devices transmitting)")
            
            # Shutdown bus
            bus.shutdown()
            logger.info("CAN bus closed cleanly")
            
            return jsonify({
                "status": "success",
                "message": "CAN bus initialized successfully",
                "bus_info": bus_info
            })
        
        except json.JSONDecodeError as e:
            logger.error(f"Invalid config JSON: {e}")
            return jsonify({
                "status": "error",
                "message": f"Invalid config format: {str(e)}"
            }), 400
        
        except can.CanError as e:
            logger.exception("CAN initialization error")
            return jsonify({
                "status": "error",
                "message": f"CAN error: {str(e)}",
                "error_type": "can_error"
            }), 500
        
        except Exception as e:
            logger.exception("Unexpected error during CAN test")
            return jsonify({
                "status": "error",
                "message": f"Unexpected error: {str(e)}",
                "error_type": "unknown"
            }), 500

    @app.route('/can/connect', methods=['POST'])
    def connect_can_interface():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        config_payload = data.get('config')
        bitrate_override = data.get('bitrate')

        if isinstance(config_payload, str):
            try:
                config = json.loads(config_payload)
            except json.JSONDecodeError as exc:
                return jsonify({
                    "status": "error",
                    "message": f"Invalid config JSON: {exc}"
                }), 400
        elif isinstance(config_payload, dict):
            config = dict(config_payload)
        else:
            config = {}

        if bitrate_override:
            try:
                config['bitrate'] = int(bitrate_override)
            except ValueError:
                return jsonify({
                    "status": "error",
                    "message": "bitrate must be an integer value"
                }), 400

        if 'bitrate' not in config:
            config['bitrate'] = getattr(can_manager, "DEFAULT_BITRATE", 1_000_000)

        try:
            info = can_manager.connect(config)
            return jsonify({
                "status": "success",
                "message": "CAN interface connected",
                "info": info
            })
        except ValueError as exc:
            return jsonify({
                "status": "error",
                "message": str(exc)
            }), 400
        except Exception as exc:
            logger.exception("Failed to connect CAN interface")
            return jsonify({
                "status": "error",
                "message": f"Unable to connect: {exc}"
            }), 500

    @app.route('/can/disconnect', methods=['POST'])
    def disconnect_can_interface():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            can_manager.disconnect()
            return jsonify({
                "status": "success",
                "message": "CAN interface disconnected"
            })
        except Exception as exc:
            logger.exception("Error while disconnecting CAN interface")
            return jsonify({
                "status": "error",
                "message": f"Unable to disconnect: {exc}"
            }), 500

    @app.route('/can/status', methods=['GET'])
    def get_can_status():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            state = can_manager.get_connection_state()
            return jsonify({
                "status": "success",
                "state": state
            })
        except Exception as exc:
            logger.exception("Unable to fetch CAN status")
            return jsonify({
                "status": "error",
                "message": f"Unable to fetch status: {exc}"
            }), 500

    @app.route('/can/time_sync', methods=['POST'])
    def send_can_time_sync():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        timestamp_ms = data.get('timestamp_ms')
        if timestamp_ms is None:
            timestamp_ms = int(time.time() * 1000)
        else:
            try:
                timestamp_ms = int(timestamp_ms)
            except ValueError:
                return jsonify({
                    "status": "error",
                    "message": "timestamp_ms must be an integer"
                }), 400

        try:
            result = can_manager.send_time_sync(timestamp_ms)
            return jsonify({
                "status": "success",
                "message": f"Time sync broadcast at {result['timestamp_ms']} ms",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to send CAN time sync")
            return jsonify({
                "status": "error",
                "message": f"Unable to send time sync: {exc}"
            }), 500

    @app.route('/can/waypoint', methods=['POST'])
    def send_can_waypoint():
        """
        Send waypoint command using Multi-DOF format.
        
        Sends all DOFs of a joint in a single CAN frame (8 bytes).
        
        Request JSON:
            {
                "joint": "ANKLE_RIGHT",
                "angles_deg": [45.0, 10.0, -5.0],  // DOF0, DOF1, DOF2 (use null for unused)
                "t_offset_ms": 1000                // Offset from current time
            }
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        joint = data.get('joint')
        angles_deg = data.get('angles_deg')
        t_offset_ms = data.get('t_offset_ms')

        if not joint:
            return jsonify({
                "status": "error",
                "message": "Joint is required"
            }), 400
        if angles_deg is None or not isinstance(angles_deg, list):
            return jsonify({
                "status": "error",
                "message": "angles_deg must be a list of up to 3 angles (use null for unused DOFs)"
            }), 400
        if t_offset_ms is None:
            return jsonify({
                "status": "error",
                "message": "t_offset_ms is required (offset from last time sync)"
            }), 400

        try:
            # Convert angles, keeping None for unused DOFs
            processed_angles = []
            for i, angle in enumerate(angles_deg[:3]):  # Max 3 DOFs
                if angle is not None:
                    processed_angles.append(float(angle))
                else:
                    processed_angles.append(None)
            
            t_offset_ms = int(t_offset_ms)
            if t_offset_ms < 0 or t_offset_ms > 65535:
                return jsonify({
                    "status": "error",
                    "message": "t_offset_ms must be 0-65535"
                }), 400
                
        except ValueError as exc:
            return jsonify({
                "status": "error",
                "message": f"Invalid parameter: {exc}"
            }), 400

        try:
            details = can_manager.send_multi_dof_waypoint(joint, processed_angles, t_offset_ms)
            return jsonify({
                "status": "success",
                "message": f"Multi-DOF waypoint queued for {joint}",
                "details": details
            })
        except ValueError as exc:
            return jsonify({
                "status": "error",
                "message": str(exc)
            }), 400
        except Exception as exc:
            logger.exception("Failed to send multi-DOF waypoint")
            return jsonify({
                "status": "error",
                "message": f"Unable to send multi-DOF waypoint: {exc}"
            }), 500

    @app.route('/can/emergency_stop', methods=['POST'])
    def send_can_emergency_stop():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        reason_code = data.get('reason_code', 0)

        try:
            reason_code = int(reason_code) & 0xFF
        except ValueError:
            return jsonify({
                "status": "error",
                "message": "reason_code must be integer"
            }), 400

        try:
            result = can_manager.send_emergency_stop(reason_code)
            return jsonify({
                "status": "success",
                "message": "Emergency stop broadcast",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to send emergency stop")
            return jsonify({
                "status": "error",
                "message": f"Unable to send emergency stop: {exc}"
            }), 500

    @app.route('/status_message', methods=['GET'])
    def get_status_message():
        popped = serial_manager.pop_status_message()
        if popped:
            port, message = popped
            return jsonify({
                "status": "success",
                "message": message,
                "port": port,
                "joint_status": serial_manager.get_combined_joint_status(),
            })
        return jsonify({
            "status": "error", 
            "message": "No update available."
        })

    @app.route('/get_output_data', methods=['GET'])
    def get_output_data():
        joint = request.args.get('joint')
        if not joint:
            return jsonify({
                "status": "error",
                "message": "Specify joint to get output data.",
            }), 400

        handler, error_response, status_code = handler_or_error(joint)
        if handler is None:
            return error_response, status_code

        return jsonify({
            "status": "success",
            "joint": joint,
            "port": serial_manager.get_port_for_joint(joint),
            "data": handler.get_output_data(),
        })

    @app.route('/joint_limits', methods=['GET'])
    def get_joint_limits():
        def normalize_limits(source, target_key):
            """Populates target[target_key] with min/max values for each joint/DOF"""
            for joint, value in source.items():
                if joint not in limits:
                    limits[joint] = {"min": {}, "max": {}, "dof_count": 0}

                if isinstance(value, dict):
                    for dof_index, angle in value.items():
                        limits[joint][target_key][str(dof_index)] = angle
                else:
                    limits[joint][target_key]["0"] = value

        limits = {}
        normalize_limits(MIN_ANGLES, "min")
        normalize_limits(MAX_ANGLES, "max")
        
        # Add DOF count and full DOF info for each joint from JOINTS config
        for joint_key, joint_info in JOINTS.items():
            if joint_key in limits and 'dofs' in joint_info:
                limits[joint_key]["dof_count"] = len(joint_info['dofs'])
                limits[joint_key]["dofs"] = joint_info['dofs']  # Include full DOF data (with zero_angle_offset)

        return jsonify({"limits": limits})
    
    @app.route('/joint_config', methods=['GET'])
    def get_joint_config():
        """
        Endpoint to get joint configuration from joint_config.json
        Includes DOF limits, motor count, encoder channels, etc.
        """
        try:
            config_path = Path(__file__).parent.parent / "joint_config.json"
            
            if not config_path.exists():
                return jsonify({
                    "status": "error",
                    "message": "joint_config.json not found"
                }), 404
            
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            
            return jsonify({
                "status": "success",
                "config": config_data
            })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error loading joint config: {str(e)}"
            }), 500

    @app.route('/get_encoder_data', methods=['GET'])
    def get_encoder_data():
        """
        Returns current encoder data for the selected joint
        """
        try:
            joint = request.args.get('joint', 'KNEE_LEFT')
            handler, error_response, status_code = handler_or_error(joint)
            if handler is None:
                return error_response, status_code

            # Check if encoder data is available
            if hasattr(handler, 'current_encoder_data') and handler.current_encoder_data:
                encoder_data = handler.current_encoder_data.get(joint, {})
                
                if encoder_data:
                    return jsonify({
                        "status": "success",
                        "data": {
                            "joint": joint,
                            "timestamp": encoder_data.get('timestamp', 0),
                            "dof_positions": encoder_data.get('dof_positions', {}),
                            "raw_encoder_values": encoder_data.get('raw_values', {}),
                            "is_active": encoder_data.get('is_active', False)
                        }
                    })
            
            # No data available
            return jsonify({
                "status": "success",
                "data": {
                    "joint": joint,
                    "timestamp": 0,
                    "dof_positions": {},
                    "raw_encoder_values": {},
                    "is_active": False
                },
                "message": "No encoder data available from Pico"
            })
                
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error retrieving encoder data: {str(e)}",
                "data": None
            })



    @app.route('/get_movement_data_multi_dof', methods=['GET'])
    def get_movement_data_multi_dof():
        """
        Endpoint for multi-DOF movement data with hierarchical structure
        """
        try:
            joint_filter = request.args.get('joint', None)
            dof = request.args.get('dof', None)

            handlers = []
            if joint_filter:
                handler, error_response, status_code = handler_or_error(joint_filter)
                if handler is None:
                    return error_response, status_code
                handlers = [handler]
            else:
                handlers = list(serial_manager.get_handler_snapshot().values())

            for handler in handlers:
                movement_data = getattr(handler, 'movement_web_data', None)
                if not (
                    isinstance(movement_data, dict)
                    and 'metadata' in movement_data
                    and 'joints' in movement_data
                ):
                    continue

                response_data = movement_data
                if joint_filter and dof:
                    filtered = {
                        "metadata": movement_data["metadata"],
                        "joints": {},
                    }
                    if joint_filter in movement_data["joints"]:
                        dof_key = f"dof_{dof}"
                        joint_block = movement_data["joints"][joint_filter]
                        if dof_key in joint_block:
                            filtered["joints"][joint_filter] = {
                                dof_key: joint_block[dof_key]
                            }
                    response_data = filtered

                elif joint_filter:
                    filtered = {
                        "metadata": movement_data["metadata"],
                        "joints": {},
                    }
                    if joint_filter in movement_data["joints"]:
                        filtered["joints"][joint_filter] = movement_data["joints"][joint_filter]
                    response_data = filtered

                return jsonify({
                    "status": "success",
                    "data": response_data,
                    "filter": (
                        {"joint": joint_filter, "dof": dof}
                        if joint_filter and dof
                        else ({"joint": joint_filter} if joint_filter else None)
                    ),
                    "has_data": bool(response_data.get("joints")),
                })

            return jsonify({
                "status": "success",
                "data": None,
                "message": "No multi-DOF movement data available",
                "has_data": False,
            })
                
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error retrieving movement data: {str(e)}",
                "data": None,
                "has_data": False
            })

    @app.route('/get_mapping_data', methods=['GET'])
    def get_mapping_data():
        """
        Returns mapping data received from MAPPING_DATA protocol
        """
        joint = request.args.get('joint')
        if not joint:
            return jsonify({
                "status": "error",
                "message": "Specify joint to get mapping data.",
            }), 400

        handler, error_response, status_code = handler_or_error(joint)
        if handler is None:
            return error_response, status_code

        if hasattr(handler, 'automatic_mapping_data') and handler.automatic_mapping_data:
            serializable_data = utils.convert_to_serializable(handler.automatic_mapping_data)
            return jsonify({
                "status": "success",
                "data": serializable_data,
                "has_data": True,
                "joint": joint,
            })

        return jsonify({
            "status": "success",
            "message": "No mapping data available",
            "has_data": False,
            "joint": joint,
        })

    @app.route('/command', methods=['POST'])
    def receive_command():
        data: Dict[str, Any] = request.json
        cmd: str = data.get('cmd', '')
        
        # Estrai i parametri per il nuovo formato di comando
        joint = data.get('joint', 'KNEE_LEFT')  # Default a KNEE_LEFT
        dof = data.get('dof', 'ALL')            # Default a ALL
        
        status = "success"
        message = f"Received Command {cmd} for {joint}:{dof}."

        handler, error_response, status_code = handler_or_error(joint)
        if handler is None:
            return error_response, status_code

        try:
            if cmd == "pretension":
                # Send command appropriate based on DOF
                if dof == 'ALL':
                    handler.send_new_command(joint, dof, COMMANDS['PRETENSION_ALL'])
                else:
                    handler.send_new_command(joint, dof, COMMANDS['PRETENSION'])
            elif cmd == "release":
                # Send command without parameters
                if dof == 'ALL':
                    handler.send_new_command(joint, dof, COMMANDS['RELEASE_ALL'])
                else:
                    handler.send_new_command(joint, dof, COMMANDS['RELEASE'])
            elif cmd == "start-auto-mapping":
                # Start advanced automatic mapping
                handler.send_new_command(joint, dof, COMMANDS['START_AUTO_MAPPING'])
                message = f"Started advanced automatic mapping for joint {joint} DOF {dof}"
            elif cmd == "stop-auto-mapping":
                # Stop advanced automatic mapping
                handler.send_new_command(joint, dof, COMMANDS['STOP_AUTO_MAPPING'])
                message = f"Stopped advanced automatic mapping for joint {joint} DOF {dof}"
            elif cmd == "recalc-offset":
                # Send command without parameters
                handler.send_new_command(joint, dof, COMMANDS['RECALC_OFFSET'])
            elif cmd == "set-zero":
                # Send command to set current position as zero
                handler.send_new_command(joint, dof, COMMANDS['SET_ZERO'])
                message = f"Current position set as zero for {joint} DOF {dof}"
            elif cmd == "start-test-encoder":
                handler.send_new_command(joint, dof, COMMANDS['START_TEST_ENCODER'])
            elif cmd == "stop-test-encoder":
                handler.send_new_command(joint, dof, COMMANDS['STOP_TEST_ENCODER'])
            elif cmd == "can-diag":
                handler.send_new_command(joint, dof, COMMANDS['CAN_DIAG'])
                message = f"CAN diagnostic started for {joint}"
            elif cmd == "reset-errors":
                # Reset error counter and clear message queue
                handler.status_message = []
                handler.last_error_message = ""
                handler.error_count = 0
                message = "Error counters reset and message queue cleared."
            elif cmd == "stop-move":
                handler.send_new_command(joint, dof, COMMANDS['STOP'])
            elif cmd == "move-multi-dof":
                # Handle simultaneous Multi-DOF command
                angle0 = float(data.get('angle0', 0))
                angle1 = float(data.get('angle1', 0))
                angle2 = float(data.get('angle2', 0))
                mask = int(data.get('mask', 3))
                sync = int(data.get('sync', 1))
                speed = float(data.get('speed', 0.5))
                accel = float(data.get('accel', 2.0))
                path = int(data.get('path', 1))
                
                # Verify angular limits for active DOFs
                valid_angles = True
                error_msgs = []
                
                # Check DOF 0 if included in mask
                if mask & 1:  # Bit 0 active
                    if isinstance(MIN_ANGLES[joint], dict):
                        if not (MIN_ANGLES[joint][0] <= angle0 <= MAX_ANGLES[joint][0]):
                            valid_angles = False
                            error_msgs.append(f"DOF 0: angle must be between {MIN_ANGLES[joint][0]} and {MAX_ANGLES[joint][0]} degrees")
                    else:
                        if not (MIN_ANGLES[joint] <= angle0 <= MAX_ANGLES[joint]):
                            valid_angles = False
                            error_msgs.append(f"DOF 0: angle must be between {MIN_ANGLES[joint]} and {MAX_ANGLES[joint]} degrees")
                
                # Check DOF 1 if included in mask
                if mask & 2:  # Bit 1 active
                    if isinstance(MIN_ANGLES[joint], dict) and len(MIN_ANGLES[joint]) > 1:
                        if not (MIN_ANGLES[joint][1] <= angle1 <= MAX_ANGLES[joint][1]):
                            valid_angles = False
                            error_msgs.append(f"DOF 1: angle must be between {MIN_ANGLES[joint][1]} and {MAX_ANGLES[joint][1]} degrees")
                
                # Check DOF 2 if included in mask (not yet implemented in UI but prepared)
                if mask & 4:  # Bit 2 active
                    if isinstance(MIN_ANGLES[joint], dict) and len(MIN_ANGLES[joint]) > 2:
                        if not (MIN_ANGLES[joint][2] <= angle2 <= MAX_ANGLES[joint][2]):
                            valid_angles = False
                            error_msgs.append(f"DOF 2: angle must be between {MIN_ANGLES[joint][2]} and {MAX_ANGLES[joint][2]} degrees")
                
                if valid_angles:
                    # Build parameters for Multi-DOF command
                    params = [angle0, angle1, angle2, mask, sync, speed, accel, path]
                    # Use acknowledgment-based sending to wait for movement completion
                    success = handler.send_movement_command_with_ack(joint, 'ALL', COMMANDS['MOVE_MULTI_DOF'], params, timeout=30.0)
                    if success:
                        message = f"✅ Multi-DOF movement completed for {joint}: DOF0={angle0}°, DOF1={angle1}°, mask={mask}, sync={sync}"
                    else:
                        status = "error"
                        message = f"❌ Multi-DOF movement timeout or error for {joint}"
                else:
                    status = "error"
                    message = "Angle validation errors: " + "; ".join(error_msgs)
            elif cmd == "get-pid":
                # New format: Request PID for specific DOF and motor type
                dof_index = data.get('dof', 0)
                # Check if dof_index is 'ALL' string
                if dof_index == 'ALL':
                    dof_index = 0  # Use 0 as default value
                else:
                    dof_index = int(dof_index)  # Convert to int only if not 'ALL'
                motor_type = int(data.get('motor_type', 1))  # Default: agonista (1)
                handler.get_pid_for_joint_dof(joint, dof_index, motor_type)
                message = f"PID request sent for {joint} DOF {dof_index} motor {motor_type}"
            elif cmd == "get-pid-outer":
                dof_index = data.get('dof', 0)
                if dof_index == 'ALL':
                    dof_index = 0
                else:
                    dof_index = int(dof_index)
                handler.get_outer_pid_for_joint_dof(joint, dof_index)
                message = f"Outer PID request sent for {joint} DOF {dof_index}"
            elif cmd == "set-pid":
                # New format: Set PID for specific DOF and motor type
                dof_index = data.get('dof', 0)
                # Check if dof_index is 'ALL' string
                if dof_index == 'ALL':
                    dof_index = 0  # Use 0 as default value
                else:
                    dof_index = int(dof_index)  # Convert to int only if not 'ALL'
                motor_type = int(data.get('motor_type', 1))  # Default: agonist (1)
                kp = float(data.get('kp', 0))
                ki = float(data.get('ki', 0))
                kd = float(data.get('kd', 0))
                tau = float(data.get('tau', 0.02))  # Default value for tau
                handler.set_pid_for_joint_dof(joint, dof_index, motor_type, kp, ki, kd, tau)
                message = f"PID values set for {joint} DOF {dof_index} motor {motor_type}"
            elif cmd == "set-pid-outer":
                dof_index = data.get('dof', 0)
                if dof_index == 'ALL':
                    dof_index = 0
                else:
                    dof_index = int(dof_index)
                kp = float(data.get('kp', 0))
                ki = float(data.get('ki', 0))
                kd = float(data.get('kd', 0))
                stiffness = float(data.get('stiffness', 1.0))
                cascade = float(data.get('cascade', 0.25))
                handler.set_outer_pid_for_joint_dof(joint, dof_index, kp, ki, kd, stiffness, cascade)
                message = f"Outer loop PID values set for {joint} DOF {dof_index}"
            elif cmd == "load-pid-all":
                # First, send LOAD_PID command to firmware to reload from flash
                handler.send_new_command(joint, 'ALL', COMMANDS['LOAD_PID'])
                
                # Then request all PID values for current joint and DOFs
                # (give firmware time to load, requests will be queued)
                for dof_index in range(3):  # Supports up to 3 DOFs
                    # Verify if this DOF is valid for the joint
                    is_valid_dof = False
                    
                    if joint in JOINTS:
                        joint_info = JOINTS[joint]
                        if 'dofs' in joint_info and dof_index < len(joint_info['dofs']):
                            is_valid_dof = True
                            
                    if is_valid_dof:
                        # Request PID for both motor types
                        handler.get_pid_for_joint_dof(joint, dof_index, 1)
                        handler.get_pid_for_joint_dof(joint, dof_index, 2)
                        handler.get_outer_pid_for_joint_dof(joint, dof_index)

                message = f"PID load from flash requested and values being read for {joint}"
            elif cmd == "save-pid":
                handler.send_new_command(joint, 'ALL', COMMANDS['SAVE_PID'])
                message = "PID save request sent"
            elif cmd == "select-joint":
                # When selecting a new joint, set as active and load PIDs
                joint_id = data.get('joint', 'KNEE_LEFT')
                
                # Set joint as active in status
                for j in list(JOINTS.keys()):
                    if j in handler.joint_status:
                        if j == joint_id:
                            if 'active' not in handler.joint_status[j]:
                                handler.joint_status[j] = {}
                            handler.joint_status[j]['active'] = True
                        else:
                            if j in handler.joint_status:
                                if 'active' not in handler.joint_status[j]:
                                    handler.joint_status[j] = {}
                                handler.joint_status[j]['active'] = False
                
                # Load PIDs for new joint
                for dof_index in range(3):  # Supports up to 3 DOFs
                    # Verify if this DOF is valid for the joint
                    is_valid_dof = False
                    
                    if joint_id in JOINTS:
                        joint_info = JOINTS[joint_id]
                        if 'dofs' in joint_info and dof_index < len(joint_info['dofs']):
                            is_valid_dof = True
                            
                    if is_valid_dof:
                        # Request PID for both motor types
                        handler.get_pid_for_joint_dof(joint_id, dof_index, 1)
                        handler.get_pid_for_joint_dof(joint_id, dof_index, 2)
                        handler.get_outer_pid_for_joint_dof(joint_id, dof_index)

                message = f"Joint {joint_id} selected and PIDs requested"
            elif cmd == "start-measure-output":
                handler.send_new_command(joint, dof, COMMANDS['START_MEASURE'])
            elif cmd == "stop-measure-output":
                handler.send_new_command(joint, dof, COMMANDS['STOP_MEASURE'])
            elif cmd == "GET_MOVEMENT_DATA":
                # Request movement data from firmware (on-demand)
                handler.send_new_command(joint, dof, COMMANDS['GET_MOVEMENT_DATA'])
                message = f"Movement data request sent to {joint}"
            else:
                status = "error"
                message = f"Command {cmd} not recognized."
        except Exception as e:
            status = "error"
            message = f"Error: {str(e)}"

        return jsonify({
            "status": status, 
            "message": message, 
            "joint_status": serial_manager.get_combined_joint_status()
        })

    @app.route('/get_saved_mapping_data/<joint_name>', methods=['GET'])
    def get_saved_mapping_data(joint_name):
        """
        Returns saved mapping data for a specific joint
        """
        try:
            joint_key = joint_name.upper()
            mapping_data = load_mapping_from_file(joint_key)
            if mapping_data:
                serializable_data = utils.convert_to_serializable(mapping_data)
                return jsonify({
                    "status": "success",
                    "data": serializable_data,
                    "has_data": True,
                    "joint_name": joint_key,
                    "file_timestamp": mapping_data.get('file_timestamp')
                })
            else:
                return jsonify({
                    "status": "success", 
                    "message": f"No saved mapping data for {joint_key}",
                    "has_data": False,
                    "joint_name": joint_key
                })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error loading data for {joint_key}: {str(e)}",
                "has_data": False
            })

    @app.route('/list_saved_mapping_files', methods=['GET'])
    def list_saved_mapping_files():
        """
        Lists all saved mapping files
        """
        try:
            import os
            import glob
            
            mapping_files = []
            mapping_dir = "mapping_data"
            
            if os.path.exists(mapping_dir):
                pattern = os.path.join(mapping_dir, "*_mapping.json")
                files = glob.glob(pattern)
                
                for file_path in files:
                    try:
                        # Extract joint name from filename
                        filename = os.path.basename(file_path)
                        joint_name = filename.replace('_mapping.json', '').upper()
                        
                        # Get file information
                        stat = os.stat(file_path)
                        
                        # Load metadata from file
                        with open(file_path, 'r') as f:
                            import json
                            data = json.load(f)
                            
                        mapping_files.append({
                            "joint_name": joint_name,
                            "filename": filename,
                            "file_size": stat.st_size,
                            "modified_time": stat.st_mtime,
                            "total_points": data.get('total_points', 0),
                            "dof_count": data.get('dof_count', 0),
                            "actual_dof_count": data.get('actual_dof_count', 0),
                            "timestamp": data.get('timestamp'),
                            "present_dofs": data.get('present_dofs', [])
                        })
                    except Exception:
                        # If there's an error reading a specific file, continue with others
                        continue
            
            return jsonify({
                "status": "success",
                "files": mapping_files,
                "count": len(mapping_files)
            })
            
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error listing files: {str(e)}",
                "files": [],
                "count": 0
            })

    @app.route('/delete_saved_mapping_data/<joint_name>', methods=['DELETE'])
    def delete_saved_mapping_data(joint_name):
        """
        Deletes saved mapping data for a specific joint
        """
        try:
            import os
            filename = f"mapping_data/{joint_name.lower()}_mapping.json"
            
            if os.path.exists(filename):
                os.remove(filename)
                return jsonify({
                    "status": "success",
                    "message": f"Mapping data deleted for {joint_name.upper()}",
                    "joint_name": joint_name.upper()
                })
            else:
                return jsonify({
                    "status": "error",
                    "message": f"Mapping file not found for {joint_name.upper()}",
                    "joint_name": joint_name.upper()
                })
                
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error deleting data for {joint_name.upper()}: {str(e)}",
                "joint_name": joint_name.upper()
            })

    @app.route('/save_enriched_mapping_data', methods=['POST'])
    def save_enriched_mapping_data():
        """
        Saves enriched mapping data with interpolation and extrapolation
        """
        try:
            import os
            import json
            from datetime import datetime
            
            data = request.json
            joint_name = data.get('joint_name', 'UNKNOWN').upper()
            enriched_data = data.get('enriched_data')
            
            if not enriched_data:
                return jsonify({
                    "status": "error",
                    "message": "No enriched data provided"
                })
            
            # Ensure directory exists
            mapping_dir = "mapping_data"
            if not os.path.exists(mapping_dir):
                os.makedirs(mapping_dir)
            
            # Create filename for enriched data
            filename = f"{mapping_dir}/{joint_name.lower()}_enriched_mapping.json"
            
            # Add save timestamp
            enriched_data['saved_timestamp'] = datetime.now().isoformat()
            enriched_data['file_version'] = '3.1.1_enriched'
            enriched_data['joint_name'] = joint_name
            
            # Save enriched data
            with open(filename, 'w') as f:
                json.dump(enriched_data, f, indent=2)
            
            return jsonify({
                "status": "success",
                "message": f"Enriched mapping data saved for {joint_name}",
                "joint_name": joint_name,
                "filename": filename,
                "total_dofs": len(enriched_data.get('present_dofs', [])),
                "enrichment_included": True
            })
            
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error saving enriched data: {str(e)}"
            })

    @app.route('/sequence/start', methods=['POST'])
    def start_sequence():
        """
        Starts sequence data collection
        """
        try:
            data = request.get_json()
            joint = data.get('joint')
            
            if not joint:
                return jsonify({
                    "status": "error",
                    "message": "Joint name is required"
                }), 400
            
            handler, error, code = handler_or_error(joint)
            if error:
                return error, code
            
            handler.start_sequence_data_collection()
            return jsonify({
                "status": "success",
                "message": "Sequence data collection started"
            })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error starting sequence: {str(e)}"
            }), 500

    @app.route('/sequence/stop', methods=['POST'])
    def stop_sequence():
        """
        Stops sequence data collection
        """
        try:
            data = request.get_json()
            joint = data.get('joint')
            
            if not joint:
                return jsonify({
                    "status": "error",
                    "message": "Joint name is required"
                }), 400
            
            handler, error, code = handler_or_error(joint)
            if error:
                return error, code
            
            handler.stop_sequence_data_collection()
            return jsonify({
                "status": "success",
                "message": "Sequence data collection stopped",
                "steps_collected": len(handler.get_sequence_movement_data())
            })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error stopping sequence: {str(e)}"
            }), 500

    @app.route('/sequence/data', methods=['GET'])
    def get_sequence_data():
        """
        Returns accumulated sequence movement data
        """
        try:
            joint = request.args.get('joint')
            
            if not joint:
                return jsonify({
                    "status": "error",
                    "message": "Joint name is required"
                }), 400
            
            handler, error, code = handler_or_error(joint)
            if error:
                return error, code
            
            data = handler.get_sequence_movement_data()
            return jsonify({
                "status": "success",
                "steps": len(data),
                "data": data
            })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"Error getting sequence data: {str(e)}"
            }), 500

    @app.route('/')
    def index():
        return render_template('index.html')

    return app
