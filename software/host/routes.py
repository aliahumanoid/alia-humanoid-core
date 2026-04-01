"""
Flask API routes for the Joint Controller host application.

This module defines all HTTP endpoints for:
- Joint control commands (movement, PID tuning, calibration)
- Mapping data management (load, save, visualize)
- Serial communication (port assignment, status)
- System configuration (joint limits, available commands)
"""
from flask import jsonify, request, current_app, render_template, send_from_directory
from typing import Dict, Any, List, Optional
from serial_manager import SerialManager
from config import JOINTS, MIN_ANGLES, MAX_ANGLES, COMMANDS
from motor_can_bench import DEFAULT_MOTOR_ID, DEFAULT_OUTPUT_RATIO, analyze_csv_log
from motor_rs485_private import (
    MotorRs485PrivateReader,
    DEFAULT_RS485_PRIVATE_BAUD,
    DEFAULT_RS485_PRIVATE_MOTOR_ID,
)
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


def register_routes(
    app,
    serial_manager: SerialManager,
    can_manager=None,
    motor_private_reader: Optional[MotorRs485PrivateReader] = None,
):
    """
    Register all Flask routes for the joint controller application.
    
    Args:
        app: Flask application instance
        serial_manager: SerialManager instance for hardware communication
    """

    if motor_private_reader is None:
        motor_private_reader = MotorRs485PrivateReader(serial_manager)

    def handler_or_error(joint: str):
        handler = serial_manager.get_handler_for_joint(joint)
        if handler is None:
            return (
                None,
                jsonify({
                    "status": "error",
                    "message": f"No serial port associated with joint {joint}.",
                    "joint": joint,
                }),
                400,
            )
        return handler, None, None

    def provisioning_handler_or_error(joint: str, port: Optional[str]):
        if port:
            handler = serial_manager.get_handler_for_port(port, create_if_missing=True)
            if handler is None:
                return (
                    None,
                    jsonify({
                        "status": "error",
                        "message": f"No serial handler available for port {port}.",
                        "joint": joint,
                        "port": port,
                    }),
                    400,
                )
            return handler, None, None
        return handler_or_error(joint)

    def can_unavailable_response():
        if not CAN_AVAILABLE or can_manager is None:
            return jsonify({
                "status": "error",
                "message": "CAN features not available on this host (python-can missing or disabled)."
            }), 503
        return None

    def get_joint_dof_meta(joint_name: str, dof_index: int):
        joint_info = JOINTS.get(joint_name)
        if not joint_info:
            return None
        for dof in joint_info.get("dofs", []):
            if dof.get("index") == int(dof_index):
                return dof
        return None

    def get_pid_motor_types_for_dof(joint_name: str, dof_index: int) -> list[int]:
        dof_meta = get_joint_dof_meta(joint_name, dof_index)
        if dof_meta and dof_meta.get("drive_type") == "direct_drive":
            return [3]
        return [1, 2]

    def get_zero_operation_label(joint_name: str, dof_index: int) -> str:
        dof_meta = get_joint_dof_meta(joint_name, dof_index)
        if dof_meta and dof_meta.get("drive_type") == "direct_drive":
            return "Reference set"
        return "Zero set"

    def load_mapping_from_file(joint_name: str):
        # Try enriched data first (has interpolation/extrapolation), then raw
        enriched = f"mapping_data/{joint_name.lower()}_enriched_mapping.json"
        raw = f"mapping_data/{joint_name.lower()}_mapping.json"
        filename = enriched if os.path.exists(enriched) else raw
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

        # Raw format stores DOF data under "mapping_data" dict.
        # Enriched format stores DOF data at top level (dof_0, dof_1, ...).
        # Check both locations.
        for dof_key, dof_data in saved.get("mapping_data", {}).items():
            mapping_data[dof_key] = dof_data

        # Also check top-level dof_* keys (enriched format)
        for key, value in saved.items():
            if key.startswith("dof_") and isinstance(value, dict) and key not in mapping_data:
                mapping_data[key] = value

        return mapping_data

    @app.route('/analysis')
    def analysis_page():
        """
        Serves the movement analysis page for CSV visualization.
        """
        return render_template('analysis.html')

    @app.route('/motor_test')
    def motor_test_page():
        """
        Serves the dedicated single-motor CAN bench page.
        """
        return render_template(
            'motor_test.html',
            motor_id=DEFAULT_MOTOR_ID,
            output_ratio=DEFAULT_OUTPUT_RATIO,
        )

    @app.route('/motor_tuning')
    def motor_tuning_page():
        """
        Serves the read-only single-motor CAN tuning page.
        """
        return render_template(
            'motor_tuning.html',
            motor_id=DEFAULT_MOTOR_ID,
            output_ratio=DEFAULT_OUTPUT_RATIO,
            rs485_motor_id=DEFAULT_RS485_PRIVATE_MOTOR_ID,
            rs485_baud=DEFAULT_RS485_PRIVATE_BAUD,
        )

    @app.route('/api/motor_test/status', methods=['GET'])
    def motor_test_status():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        output_ratio = request.args.get("output_ratio", DEFAULT_OUTPUT_RATIO, type=float)
        try:
            result = can_manager.get_motor_test_status(
                motor_id=DEFAULT_MOTOR_ID,
                output_ratio=output_ratio,
            )
        except Exception as exc:
            current_app.logger.exception("Motor test status failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_tuning/status', methods=['GET'])
    def motor_tuning_status():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        output_ratio = request.args.get("output_ratio", DEFAULT_OUTPUT_RATIO, type=float)
        try:
            result = can_manager.get_motor_tuning_status(
                motor_id=DEFAULT_MOTOR_ID,
                output_ratio=output_ratio,
            )
        except Exception as exc:
            current_app.logger.exception("Motor tuning status failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_tuning/read', methods=['POST'])
    def motor_tuning_read():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.json or {}
        action = (data.get("action") or "").strip().lower()
        timeout_s = float(data.get("timeout_s", 0.2))
        output_ratio = float(data.get("output_ratio", DEFAULT_OUTPUT_RATIO))

        if not action:
            return jsonify({"status": "error", "message": "action is required"}), 400

        try:
            result = can_manager.motor_tuning_read(
                action,
                motor_id=DEFAULT_MOTOR_ID,
                timeout_s=timeout_s,
                output_ratio=output_ratio,
            )
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except TimeoutError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 504
        except Exception as exc:
            current_app.logger.exception("Motor tuning read failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_tuning/private_status', methods=['GET'])
    def motor_tuning_private_status():
        try:
            result = motor_private_reader.get_status()
        except Exception as exc:
            current_app.logger.exception("Motor tuning private status failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_tuning/private_read', methods=['POST'])
    def motor_tuning_private_read():
        data = request.json or {}
        action = (data.get("action") or "").strip().lower()
        port = (data.get("port") or "").strip()
        timeout_s = float(data.get("timeout_s", 0.5))
        baud = int(data.get("baud", DEFAULT_RS485_PRIVATE_BAUD))

        if not action:
            return jsonify({"status": "error", "message": "action is required"}), 400
        if not port:
            return jsonify({"status": "error", "message": "port is required"}), 400

        try:
            result = motor_private_reader.read(
                action,
                port=port,
                motor_id=DEFAULT_RS485_PRIVATE_MOTOR_ID,
                baud=baud,
                timeout_s=timeout_s,
            )
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except TimeoutError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 504
        except Exception as exc:
            current_app.logger.exception("Motor tuning private read failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_tuning/private_write_ram', methods=['POST'])
    def motor_tuning_private_write_ram():
        data = request.json or {}
        action = (data.get("action") or "").strip().lower()
        port = (data.get("port") or "").strip()
        timeout_s = float(data.get("timeout_s", 0.5))
        baud = int(data.get("baud", DEFAULT_RS485_PRIVATE_BAUD))
        confirm_token = (data.get("confirm_token") or "").strip()
        current_pid = data.get("current_pid") or {}
        base_raw_hex = data.get("base_raw_hex") or ""

        if action != "current_pid_ram":
            return jsonify({"status": "error", "message": "unsupported private write action"}), 400
        if not port:
            return jsonify({"status": "error", "message": "port is required"}), 400
        if confirm_token != "WRITE_CURRENT_PID_RAM":
            return jsonify({"status": "error", "message": "confirmation token missing or invalid"}), 400
        if not isinstance(current_pid, dict):
            return jsonify({"status": "error", "message": "current_pid must be an object"}), 400
        if not base_raw_hex:
            return jsonify({"status": "error", "message": "base_raw_hex is required"}), 400

        try:
            result = motor_private_reader.write_current_pid_ram(
                port=port,
                motor_id=DEFAULT_RS485_PRIVATE_MOTOR_ID,
                baud=baud,
                timeout_s=timeout_s,
                kp=int(current_pid.get("kp", 0)),
                ki=int(current_pid.get("ki", 0)),
                kd=int(current_pid.get("kd", 0)),
                base_raw_hex=base_raw_hex,
            )
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except TimeoutError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 504
        except Exception as exc:
            current_app.logger.exception("Motor tuning private RAM write failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_test/action', methods=['POST'])
    def motor_test_action():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.json or {}
        action = (data.get("action") or "").strip().lower()
        value = data.get("value")
        timeout_s = float(data.get("timeout_s", 0.2))
        output_ratio = float(data.get("output_ratio", DEFAULT_OUTPUT_RATIO))

        if not action:
            return jsonify({"status": "error", "message": "action is required"}), 400

        try:
            result = can_manager.motor_test_command(
                action,
                motor_id=DEFAULT_MOTOR_ID,
                value=value,
                timeout_s=timeout_s,
                output_ratio=output_ratio,
            )
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except TimeoutError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 504
        except Exception as exc:
            current_app.logger.exception("Motor test action failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_test/sweep', methods=['POST'])
    def motor_test_sweep():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.json or {}
        mode = (data.get("mode") or "torque").strip().lower()
        profile = (data.get("profile") or "step").strip().lower()
        extra_commands = data.get("extra_commands", [])

        try:
            result = can_manager.motor_test_run_sweep(
                motor_id=DEFAULT_MOTOR_ID,
                mode=mode,
                profile=profile,
                duration_s=float(data.get("duration_s", 3.0)),
                rate_hz=float(data.get("rate_hz", 50.0)),
                timeout_s=float(data.get("timeout_s", 0.1)),
                output_ratio=float(data.get("output_ratio", DEFAULT_OUTPUT_RATIO)),
                bias=float(data.get("bias", 0.0)),
                amplitude=float(data.get("amplitude", 0.0)),
                preload_s=float(data.get("preload_s", 0.0)),
                frequency_hz=float(data.get("frequency_hz", 1.0)),
                f0_hz=float(data.get("f0_hz", 0.2)),
                f1_hz=float(data.get("f1_hz", 8.0)),
                extra_commands=extra_commands,
                label=data.get("label"),
                stop_at_end=bool(data.get("stop_at_end", True)),
                motor_on_before=bool(data.get("motor_on_before", True)),
                power_off_at_end=bool(data.get("power_off_at_end", False)),
            )
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except TimeoutError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 504
        except Exception as exc:
            current_app.logger.exception("Motor test sweep failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/motor_test/logs', methods=['GET'])
    def motor_test_logs():
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        limit = request.args.get("limit", 20, type=int)
        try:
            logs = can_manager.list_motor_test_logs(limit=limit)
        except Exception as exc:
            current_app.logger.exception("Motor test log listing failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", "logs": logs})

    @app.route('/api/motor_test/logs/<path:filename>', methods=['GET'])
    def motor_test_download_log(filename: str):
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        log_dir = can_manager.motor_test_log_dir()
        return send_from_directory(str(log_dir), filename, as_attachment=True)

    @app.route('/api/motor_test/logs/<path:filename>/analysis', methods=['GET'])
    def motor_test_log_analysis(filename: str):
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        log_path = can_manager.motor_test_log_dir() / filename
        if not log_path.is_file():
            return jsonify({"status": "error", "message": "Log file not found"}), 404

        try:
            analysis = analyze_csv_log(log_path)
        except Exception as exc:
            current_app.logger.exception("Motor test log analysis failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", "filename": filename, **analysis})

    @app.route('/api/motor_test/logs/<path:filename>', methods=['DELETE'])
    def motor_test_delete_log(filename: str):
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        log_path = can_manager.motor_test_log_dir() / filename
        if not log_path.is_file():
            return jsonify({"status": "error", "message": "Log file not found"}), 404

        try:
            log_path.unlink()
        except Exception as exc:
            current_app.logger.exception("Motor test log deletion failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", "deleted": filename})

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

    @app.route('/discover_joints', methods=['POST'])
    def discover_joints():
        """
        Trigger automatic joint discovery.

        1. Sends CAN identify request (if CAN connected)
        2. Scans all serial ports for EVT:JOINT messages
        3. Retries once if first scan finds nothing (CAN identify triggers 3s broadcast)
        """
        discovered = {}
        can_sent = False

        # Try to send CAN identify request if connected
        if can_manager and can_manager.is_connected():
            try:
                can_manager.send_identify_request()
                can_sent = True
                # Wait a bit for controllers to start broadcasting
                time.sleep(0.2)
            except Exception as e:
                current_app.logger.warning(f"Could not send CAN identify: {e}")

        # Scan serial ports for joint identification
        try:
            discovered = serial_manager.discover_joints(timeout_seconds=4.0)
        except Exception as e:
            current_app.logger.exception("Error during joint discovery")
            return jsonify({
                "status": "error",
                "message": f"Discovery failed: {e}",
            }), 500

        # Retry once if nothing found and CAN identify was sent
        # (firmware re-broadcasts EVT:JOINT for 3s after receiving 0x008)
        if not discovered and can_sent:
            current_app.logger.info("Discovery: no joints found, retrying (CAN identify was sent)...")
            time.sleep(1.0)
            try:
                discovered = serial_manager.discover_joints(timeout_seconds=3.0)
            except Exception as e:
                current_app.logger.warning(f"Discovery retry failed: {e}")

        # Also include joints discovered via CAN announce (0x4A0)
        can_discovered = {}
        if can_manager and can_manager.is_connected():
            try:
                can_discovered = can_manager.get_discovered_joints_can()
            except Exception:
                pass

        return jsonify({
            "status": "success",
            "can_request_sent": can_sent,
            "discovered": discovered,
            "can_discovered": {str(k): v for k, v in can_discovered.items()},
            "mappings": serial_manager.get_joint_to_port_mapping(),
        })

    @app.route('/api/provisioning/identity', methods=['POST'])
    def provisioning_identity():
        data = request.get_json(silent=True) or {}
        joint = data.get('joint')
        port = data.get('port')
        if not joint:
            return jsonify({
                "status": "error",
                "message": "Missing 'joint' parameter",
            }), 400

        handler, error_response, status_code = provisioning_handler_or_error(joint, port)
        if handler is None:
            return error_response, status_code

        try:
            identity = handler.get_board_identity()
        except Exception as exc:
            current_app.logger.exception("Provisioning identity query failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        if not identity:
            return jsonify({
                "status": "error",
                "message": f"No identity response from board on joint {joint}",
                "joint": joint,
                "port": port,
            }), 504

        return jsonify({
            "status": "success",
            "joint": joint,
            "port": port or serial_manager.get_joint_to_port_mapping().get(joint),
            "identity": identity,
        })

    @app.route('/api/provisioning/joint_profile', methods=['POST'])
    def provisioning_joint_profile():
        data = request.get_json(silent=True) or {}
        joint = data.get('joint')
        profile = data.get('profile') or joint
        port = data.get('port')

        if not joint:
            return jsonify({
                "status": "error",
                "message": "Missing 'joint' parameter",
            }), 400
        if not profile:
            return jsonify({
                "status": "error",
                "message": "Missing 'profile' parameter",
            }), 400
        if profile not in JOINTS:
            return jsonify({
                "status": "error",
                "message": f"Unknown joint profile: {profile}",
            }), 400

        handler, error_response, status_code = provisioning_handler_or_error(joint, port)
        if handler is None:
            return error_response, status_code

        try:
            result = handler.set_board_joint_profile(profile)
        except Exception as exc:
            current_app.logger.exception("Provisioning joint profile failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        if not result:
            return jsonify({
                "status": "error",
                "message": f"No confirmation from board while saving profile {profile}",
                "joint": joint,
                "profile": profile,
                "port": port,
            }), 504

        return jsonify({
            "status": "success",
            "joint": joint,
            "port": port or serial_manager.get_joint_to_port_mapping().get(joint),
            "profile": profile,
            "result": result,
            "message": f"Profile {profile} saved to flash. Reboot required.",
        })

    @app.route('/api/can/startup_sequence', methods=['POST'])
    def can_startup_sequence():
        """
        Trigger startup sequence via CAN (0x009).

        Sends startup command to a specific joint controller.
        The controller will recalc offsets for all DOFs and enter HOLDING.
        Progress events are received on CAN 0x490.

        Request body:
            joint: Joint name (e.g. "KNEE_RIGHT")
            torque: Optional custom pretension torque (default: 0 = use config)
            duration: Optional custom duration ms (default: 0 = use config)
        """
        if not can_manager or not can_manager.is_connected():
            return jsonify({"status": "error", "message": "CAN not connected"}), 503

        data = request.get_json(silent=True) or {}
        joint = data.get('joint', '')
        torque = int(data.get('torque', 0))
        duration = int(data.get('duration', 0))

        if not joint:
            return jsonify({"status": "error", "message": "Missing 'joint' parameter"}), 400

        try:
            result = can_manager.send_startup_sequence(joint, torque=torque, duration=duration)
            return jsonify({"status": "ok", **result})
        except Exception as e:
            current_app.logger.exception("CAN startup sequence failed")
            return jsonify({"status": "error", "message": str(e)}), 500

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
            
            # Include current connection info if connected
            connected_config = None
            if can_manager and can_manager.is_connected():
                current = can_manager._current_config
                if current:
                    # Create value matching the format used in interfaces list
                    connected_config = json.dumps({
                        "interface": current.get("interface"),
                        "channel": current.get("channel")
                    })
            
            return jsonify({
                "status": "success",
                "interfaces": interfaces,
                "count": len(interfaces),
                "connected": can_manager.is_connected() if can_manager else False,
                "connected_config": connected_config
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
                "config": "{\"interface\":\"slcan\",\"channel\":\"/dev/ttyUSB0\"}"
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

    # Movement via SET_IMPEDANCE CAN frames (see /api/impedance/target)

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

    # ===============================================================
    # CAN ENCODER STREAMING ROUTES
    # ===============================================================

    @app.route('/can/encoder_stream/start', methods=['POST'])
    def start_encoder_stream():
        """
        Start encoder streaming via CAN at 50Hz.
        
        The controller will send angle data on 0x410 until stopped.
        Use /can/encoder_stream/data to retrieve buffered data.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            # Pass joint_name for encoder offset cross-validation (optional)
            joint_name = request.json.get("joint") if request.is_json else None
            result = can_manager.start_encoder_stream(joint_name=joint_name)
            if result.get("blocked"):
                return jsonify({
                    "status": "error",
                    "message": "Encoder streaming blocked: encoder offset mismatch detected. Re-zero required.",
                    "result": result
                }), 409
            return jsonify({
                "status": "success",
                "message": "Encoder streaming started @ 50Hz",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to start encoder streaming")
            return jsonify({
                "status": "error",
                "message": f"Unable to start encoder streaming: {exc}"
            }), 500

    @app.route('/can/encoder_stream/stop', methods=['POST'])
    def stop_encoder_stream():
        """
        Stop encoder streaming via CAN.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            result = can_manager.stop_encoder_stream()
            return jsonify({
                "status": "success",
                "message": "Encoder streaming stopped",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to stop encoder streaming")
            return jsonify({
                "status": "error",
                "message": f"Unable to stop encoder streaming: {exc}"
            }), 500

    @app.route('/can/encoder_stream/status', methods=['GET'])
    def encoder_stream_status():
        """
        Check encoder streaming status and get buffered data.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        streaming = can_manager.is_encoder_streaming()
        data = can_manager.get_encoder_stream_data()
        
        return jsonify({
            "status": "success",
            "streaming": streaming,
            "buffer_count": len(data),
            "data": data  # List of {timestamp, t_ms, angles_deg}
        })

    @app.route('/can/encoder_angles', methods=['GET'])
    def get_last_encoder_angles():
        """
        Get the last known encoder angles (persists even when streaming is stopped).
        
        This is useful for on-demand position queries before sending impedance
        commands, without requiring continuous encoder streaming.
        
        Query params:
            joint: Optional joint name (e.g., KNEE_RIGHT). If omitted, returns all joints.
        
        Returns:
            {
                "status": "success",
                "joint": "KNEE_RIGHT",
                "angles_deg": [45.2, null, null],  // DOF0, DOF1, DOF2
                "age_ms": 150.5,  // How old the reading is
                "valid": true
            }
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable
        
        joint = request.args.get('joint')
        data = can_manager.get_last_encoder_angles(joint)
        
        return jsonify({
            "status": "success",
            **data
        })

    @app.route('/can/encoder_offsets/validate', methods=['POST'])
    def validate_encoder_offsets():
        """Validate firmware encoder offsets against saved host copy via CAN."""
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        joint_name = data.get("joint")
        if not joint_name:
            return jsonify({"status": "error", "message": "Missing 'joint' parameter"}), 400

        try:
            result = can_manager.validate_encoder_offsets(joint_name)
            status_code = 200 if result["valid"] else 409
            return jsonify({
                "status": "success" if result["valid"] else "mismatch",
                "message": "Encoder offsets valid" if result["valid"] else "Encoder offset mismatch detected",
                "result": result
            }), status_code
        except Exception as exc:
            logger.exception("Failed to validate encoder offsets")
            return jsonify({"status": "error", "message": str(exc)}), 500

    @app.route('/can/set_zero', methods=['POST'])
    def can_set_zero():
        """Send set-zero command via CAN and wait for completion."""
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        joint_name = data.get("joint")
        dof_index = data.get("dof", 0)
        if not joint_name:
            return jsonify({"status": "error", "message": "Missing 'joint' parameter"}), 400

        try:
            result = can_manager.set_zero_via_can(joint_name, dof_index)
            if result.get("success"):
                op_label = get_zero_operation_label(joint_name, dof_index)
                return jsonify({
                    "status": "success",
                    "message": f"{op_label} for {joint_name} DOF {dof_index}",
                    "offsets": result.get("offsets", {})
                })
            else:
                return jsonify({
                    "status": "error",
                    "message": result.get("error", "unknown error")
                }), 504
        except Exception as exc:
            logger.exception("Failed to set zero via CAN")
            return jsonify({"status": "error", "message": str(exc)}), 500

    # ===============================================================
    # CAN PID DIAGNOSTICS STREAMING ROUTES
    # ===============================================================

    @app.route('/can/pid_diag/start', methods=['POST'])
    def start_pid_diag_stream():
        """
        Start PID diagnostics streaming via CAN at 20Hz.

        The controller will send target/error on 0x420 and torque on 0x430.
        When terms_enabled=true, also sends inner PID terms on 0x470
        and outer PID terms on 0x480.
        Data is emitted via SocketIO events: 'pid_diag', 'pid_torque',
        'pid_inner_terms', 'pid_outer_terms'.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            data = request.get_json(silent=True) or {}
            terms_enabled = data.get("terms_enabled", False)
            result = can_manager.start_pid_diag_stream(terms_enabled=terms_enabled)
            terms_str = " + P/I/D terms" if terms_enabled else ""
            return jsonify({
                "status": "success",
                "message": f"PID diagnostics streaming started @ 20Hz{terms_str}",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to start PID diagnostics streaming")
            return jsonify({
                "status": "error",
                "message": f"Unable to start PID diagnostics streaming: {exc}"
            }), 500

    @app.route('/can/pid_diag/stop', methods=['POST'])
    def stop_pid_diag_stream():
        """
        Stop PID diagnostics streaming via CAN.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            result = can_manager.stop_pid_diag_stream()
            return jsonify({
                "status": "success",
                "message": "PID diagnostics streaming stopped",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to stop PID diagnostics streaming")
            return jsonify({
                "status": "error",
                "message": f"Unable to stop PID diagnostics streaming: {exc}"
            }), 500

    # ===============================================================
    # CAN INTERPOLATION MODE ROUTE
    # ===============================================================

    # ===============================================================
    # CAN LOOP FREQUENCIES ROUTE
    # ===============================================================

    @app.route('/can/loop_frequencies', methods=['POST'])
    def set_loop_frequencies():
        """
        Set control loop frequencies.
        
        POST body: { 
            "inner_period_us": 2000,  // Inner loop period in µs (500-10000)
            "outer_divisor": 5        // Outer loop divider (1-20)
        }
        
        Default values: inner=2000µs (500Hz), outer_divisor=5 (100Hz)
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable
        
        try:
            data = request.get_json() or {}
            inner_period = int(data.get("inner_period_us", 2000))
            outer_divisor = int(data.get("outer_divisor", 5))
            
            # Validate ranges
            if inner_period < 500 or inner_period > 10000:
                return jsonify({
                    "status": "error",
                    "message": f"inner_period_us must be between 500 and 10000 (got {inner_period})"
                }), 400
            
            if outer_divisor < 1 or outer_divisor > 20:
                return jsonify({
                    "status": "error",
                    "message": f"outer_divisor must be between 1 and 20 (got {outer_divisor})"
                }), 400
            
            result = can_manager.set_loop_frequencies(inner_period, outer_divisor)
            return jsonify({
                "status": "success",
                "message": f"Loop frequencies updated: inner={result['inner_freq_hz']:.1f}Hz, outer={result['outer_freq_hz']:.1f}Hz",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to set loop frequencies")
            return jsonify({
                "status": "error",
                "message": f"Unable to set loop frequencies: {exc}"
            }), 500

    # ===============================================================
    # CAN PID DIAGNOSTICS FREQUENCY ROUTE
    # ===============================================================

    @app.route('/can/pid_diag_frequency', methods=['POST'])
    def set_pid_diag_frequency():
        """
        Set PID diagnostics streaming frequency.
        
        POST body: { 
            "freq_hz": 50  // Frequency in Hz (10-200)
        }
        
        Default: 20Hz for monitoring, 50-100Hz for NN training data.
        Higher frequency = more data points in charts and CSV exports.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable
        
        try:
            data = request.get_json() or {}
            freq_hz = int(data.get("freq_hz", 20))
            
            # Validate range
            if freq_hz < 10 or freq_hz > 200:
                return jsonify({
                    "status": "error",
                    "message": f"freq_hz must be between 10 and 200 (got {freq_hz})"
                }), 400
            
            result = can_manager.set_pid_diag_frequency(freq_hz)
            return jsonify({
                "status": "success",
                "message": f"PID diagnostics frequency set to: {freq_hz}Hz ({result['interval_ms']:.1f}ms)",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to set PID diagnostics frequency")
            return jsonify({
                "status": "error",
                "message": f"Unable to set PID diagnostics frequency: {exc}"
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
                # Pretension is operational → CAN first, serial fallback
                if can_manager and can_manager.is_connected():
                    if dof == 'ALL':
                        can_manager.pretension_all_via_can(joint)
                    else:
                        can_manager.pretension_via_can(joint, int(dof))
                    message = f"Pretension via CAN for {joint} DOF {dof}"
                else:
                    if dof == 'ALL':
                        handler.send_new_command(joint, dof, COMMANDS['PRETENSION_ALL'])
                    else:
                        handler.send_new_command(joint, dof, COMMANDS['PRETENSION'])
                    message = f"Pretension via serial for {joint} DOF {dof}"
            elif cmd == "release":
                # Release is operational → CAN first, serial fallback
                if can_manager and can_manager.is_connected():
                    if dof == 'ALL':
                        can_manager.release_all_via_can(joint)
                    else:
                        can_manager.release_via_can(joint, int(dof))
                    message = f"Release via CAN for {joint} DOF {dof}"
                else:
                    if dof == 'ALL':
                        handler.send_new_command(joint, dof, COMMANDS['RELEASE_ALL'])
                    else:
                        handler.send_new_command(joint, dof, COMMANDS['RELEASE'])
                    message = f"Release via serial for {joint} DOF {dof}"
            elif cmd == "start-auto-mapping":
                # Auto-mapping is operational → CAN first, serial fallback
                dof_int = int(dof) if dof != 'ALL' else 0
                if can_manager and can_manager.is_connected():
                    can_manager.start_auto_mapping_via_can(joint, dof_int)
                    message = f"Auto-mapping started via CAN for {joint} DOF {dof_int}"
                else:
                    handler.send_new_command(joint, dof, COMMANDS['START_AUTO_MAPPING'])
                    message = f"Auto-mapping started via serial for {joint} DOF {dof}"
            elif cmd == "stop-auto-mapping":
                # Stop auto-mapping is operational → CAN first, serial fallback
                if can_manager and can_manager.is_connected():
                    can_manager.stop_auto_mapping_via_can(joint)
                    message = f"Auto-mapping stopped via CAN for {joint}"
                else:
                    handler.send_new_command(joint, dof, COMMANDS['STOP_AUTO_MAPPING'])
                    message = f"Auto-mapping stopped via serial for {joint}"
            elif cmd == "recalc-offset":
                # Recalc-offset is operational → CAN first, serial fallback
                dof_int = int(dof) if dof != 'ALL' else 0
                if can_manager and can_manager.is_connected():
                    can_manager.recalc_offset_via_can(joint, dof_int)
                    message = f"Recalc offset via CAN for {joint} DOF {dof_int}"
                else:
                    handler.send_new_command(joint, dof, COMMANDS['RECALC_OFFSET'])
                    message = f"Recalc offset via serial for {joint} DOF {dof}"
            elif cmd == "set-zero":
                # Set-zero is operational → CAN first, serial fallback
                dof_int = int(dof) if dof != 'ALL' else 0
                op_label = get_zero_operation_label(joint, dof_int)
                if can_manager and can_manager.is_connected():
                    result = can_manager.set_zero_via_can(joint, dof_int)
                    if result.get("success"):
                        message = f"{op_label} via CAN for {joint} DOF {dof_int}"
                    else:
                        message = f"{op_label} via CAN failed: {result.get('error', 'unknown')}"
                else:
                    handler.send_new_command(joint, dof, COMMANDS['SET_ZERO'])
                    message = f"{op_label} via serial for {joint} DOF {dof}"
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
                # CAN-only (serial fallback removed — Fase 6)
                dof_index = data.get('dof', 0)
                if dof_index == 'ALL':
                    dof_index = 0
                else:
                    dof_index = int(dof_index)
                motor_type = int(data.get('motor_type', 1))
                kp = float(data.get('kp', 0))
                ki = float(data.get('ki', 0))
                kd = float(data.get('kd', 0))
                tau = float(data.get('tau', 0.02))
                if can_manager and can_manager.is_connected():
                    can_manager.set_pid_via_can(joint, dof_index, motor_type, kp, ki, kd, tau)
                    message = f"PID set via CAN for {joint} DOF {dof_index} motor {motor_type}"
                else:
                    message = "CAN not connected — cannot set PID"
                    return jsonify({"status": "error", "message": message}), 503
            elif cmd == "set-pid-outer":
                # CAN-only (serial fallback removed — Fase 6)
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
                if can_manager and can_manager.is_connected():
                    can_manager.set_pid_outer_via_can(joint, dof_index, kp, ki, kd, stiffness, cascade)
                    message = f"Outer PID set via CAN for {joint} DOF {dof_index}"
                else:
                    message = "CAN not connected — cannot set outer PID"
                    return jsonify({"status": "error", "message": message}), 503
            elif cmd == "load-pid-all":
                # Load PID from flash and refresh UI
                if can_manager and can_manager.is_connected():
                    can_manager.load_pid_via_can(joint)
                    message = f"PID load from flash via CAN for {joint}"
                else:
                    handler.send_new_command(joint, 'ALL', COMMANDS['LOAD_PID'])
                    message = f"PID load from flash via serial for {joint}"

                # Then request all PID values for current joint and DOFs
                # (give firmware time to load, requests will be queued)
                for dof_index in range(3):
                    is_valid_dof = False
                    if joint in JOINTS:
                        joint_info = JOINTS[joint]
                        if 'dofs' in joint_info and dof_index < len(joint_info['dofs']):
                            is_valid_dof = True
                    if is_valid_dof:
                        for motor_type in get_pid_motor_types_for_dof(joint, dof_index):
                            handler.get_pid_for_joint_dof(joint, dof_index, motor_type)
                        handler.get_outer_pid_for_joint_dof(joint, dof_index)
            elif cmd == "refresh-pid-all":
                # Read current PID values from firmware (no flash load, no overwrite)
                for dof_index in range(3):
                    is_valid_dof = False
                    if joint in JOINTS:
                        joint_info = JOINTS[joint]
                        if 'dofs' in joint_info and dof_index < len(joint_info['dofs']):
                            is_valid_dof = True
                    if is_valid_dof:
                        for motor_type in get_pid_motor_types_for_dof(joint, dof_index):
                            handler.get_pid_for_joint_dof(joint, dof_index, motor_type)
                        handler.get_outer_pid_for_joint_dof(joint, dof_index)
                message = f"PID values refreshed for {joint}"
            elif cmd == "save-pid":
                # Save PID is operational → CAN first, serial fallback
                if can_manager and can_manager.is_connected():
                    can_manager.save_pid_via_can(joint)
                    message = f"PID save to flash via CAN for {joint}"
                else:
                    handler.send_new_command(joint, 'ALL', COMMANDS['SAVE_PID'])
                    message = "PID save to flash via serial"
            elif cmd == "recalc-safe-limits":
                handler.send_new_command(joint, 'ALL', COMMANDS['RECALC_SAFE_LIMITS'])
                message = "Safe limits recalculation requested"
            elif cmd == "save-linear-eq":
                # Save linear equations to flash — CAN only (no serial command exists)
                if can_manager and can_manager.is_connected():
                    can_manager.save_linear_eq_via_can(joint)
                    message = f"Linear equations save to flash via CAN for {joint}"
                else:
                    status = "error"
                    message = "Save linear equations requires CAN connection (no serial fallback)"
            elif cmd == "load-linear-eq":
                # Load linear equations from flash — CAN only (no serial command exists)
                if can_manager and can_manager.is_connected():
                    can_manager.load_linear_eq_via_can(joint)
                    message = f"Linear equations load from flash via CAN for {joint}"
                else:
                    status = "error"
                    message = "Load linear equations requires CAN connection (no serial fallback)"
            elif cmd == "startup-sequence":
                # Startup sequence is operational → CAN first, serial fallback
                if can_manager and can_manager.is_connected():
                    can_manager.send_startup_sequence(joint)
                    message = f"Startup sequence via CAN for {joint}"
                else:
                    handler.send_new_command(joint, 'ALL', COMMANDS['STARTUP_SEQUENCE'])
                    message = "Startup sequence via serial (recalc + hold)"
            elif cmd == "check-offsets":
                # Check if saved motor offsets are still valid (smart recalc detection)
                handler.send_new_command(joint, 'ALL', COMMANDS['CHECK_OFFSETS'])
                message = "Offset validation check initiated"
            elif cmd == "set-auto-start":
                # Set auto-start is operational → CAN first, serial fallback
                enabled = int(data.get('enabled', 0))
                if can_manager and can_manager.is_connected():
                    can_manager.set_auto_start_via_can(joint, enabled)
                    message = f"Auto-start {'enabled' if enabled else 'disabled'} via CAN"
                else:
                    handler.send_new_command(joint, 'ALL', f"{COMMANDS['SET_AUTO_START']}:ENABLED={enabled}")
                    message = f"Auto-start {'enabled' if enabled else 'disabled'} via serial"
            elif cmd == "get-auto-start":
                # Query current auto-start setting
                handler.send_new_command(joint, 'ALL', COMMANDS['GET_AUTO_START'])
                message = "Auto-start status requested"
            elif cmd == "friction-feedforward":
                # Friction feedforward compensation → CAN first, serial fallback
                fric_enabled = int(data.get('fric_enabled', 1))
                fric_torque = float(data.get('fric_torque', 30.0))
                fric_speed = float(data.get('fric_speed', 3.0))
                if can_manager and can_manager.is_connected():
                    fric_params = {
                        'fric_en': float(fric_enabled),
                        'fric_torque': fric_torque,
                        'fric_speed': fric_speed,
                    }
                    can_manager.friction_ff_via_can(joint, fric_params)
                    message = f"Friction feedforward set via CAN for {joint}"
                else:
                    params = (f"{COMMANDS['FRICTION_FF']}:FRIC_EN={fric_enabled}"
                              f":FRIC_TORQUE={fric_torque}:FRIC_SPEED={fric_speed}")
                    handler.send_new_command(joint, 'ALL', params)
                    message = f"Friction feedforward set via serial for {joint}"
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
                        for motor_type in get_pid_motor_types_for_dof(joint_id, dof_index):
                            handler.get_pid_for_joint_dof(joint_id, dof_index, motor_type)
                        handler.get_outer_pid_for_joint_dof(joint_id, dof_index)

                message = f"Joint {joint_id} selected and PIDs requested"
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

    # ── Impedance API ─────────────────────────────────────────────────

    @app.route('/api/impedance/target', methods=['POST'])
    def impedance_target():
        """Send SET_IMPEDANCE command to firmware.

        Required: joint_name, dof_index, q_target, dq_target, stiffness
        Optional: kp, ki, kd, tau_ff, kp_inner, ki_inner, kd_inner
        Omitted optional params → firmware keeps previous values.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.json or {}
        required = ["joint_name", "dof_index", "q_target", "dq_target", "stiffness"]
        missing = [k for k in required if k not in data]
        if missing:
            return jsonify({"status": "error",
                            "message": f"Missing fields: {', '.join(missing)}"}), 400

        try:
            kwargs = dict(
                joint_name=data["joint_name"],
                dof_index=int(data["dof_index"]),
                q_target=float(data["q_target"]),
                dq_target=abs(float(data["dq_target"])),
                stiffness=float(data["stiffness"]),
            )
            if "kp" in data:
                kwargs["kp"] = float(data["kp"])
            if "ki" in data:
                kwargs["ki"] = float(data["ki"])
            if "kd" in data:
                kwargs["kd"] = float(data["kd"])
            if "tau_ff" in data:
                kwargs["tau_ff"] = int(data["tau_ff"])
            if "kp_inner" in data:
                kwargs["kp_inner"] = float(data["kp_inner"])
            if "ki_inner" in data:
                kwargs["ki_inner"] = float(data["ki_inner"])
            if "kd_inner" in data:
                kwargs["kd_inner"] = float(data["kd_inner"])

            result = can_manager.send_impedance_target(**kwargs)
        except KeyError as exc:
            return jsonify({"status": "error",
                            "message": f"Unknown joint: {exc}"}), 400
        except Exception as exc:
            current_app.logger.exception("Impedance target failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/impedance/ctrl', methods=['POST'])
    def impedance_ctrl():
        """Send IMPEDANCE_CTRL command to firmware."""
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.json or {}
        if "joint_name" not in data or "sub_cmd" not in data:
            return jsonify({"status": "error",
                            "message": "Missing joint_name or sub_cmd"}), 400

        try:
            result = can_manager.send_impedance_ctrl(
                joint_name=data["joint_name"],
                sub_cmd=int(data["sub_cmd"]),
                param=int(data.get("param", 0)),
            )
        except KeyError as exc:
            return jsonify({"status": "error",
                            "message": f"Unknown joint: {exc}"}), 400
        except Exception as exc:
            current_app.logger.exception("Impedance ctrl failed")
            return jsonify({"status": "error", "message": str(exc)}), 500

        return jsonify({"status": "success", **result})

    @app.route('/api/impedance/state', methods=['GET'])
    def impedance_state():
        """Return latest JOINT_STATE data collected from CAN broadcasts."""
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable
        return jsonify({"status": "success",
                        "joint_state": can_manager.get_joint_state()})

    @app.route('/api/impedance/sync_outer', methods=['POST'])
    def impedance_sync_outer():
        """Send SET_PID_OUTER via CAN only (no serial fallback).

        Used by the Impedance Move panel to sync cascade/gains before
        SET_IMPEDANCE, bypassing the /command route which requires a
        serial port assignment.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable
        data = request.json or {}
        required = ["joint_name", "dof_index", "kp", "ki", "kd",
                     "stiffness", "cascade"]
        missing = [k for k in required if k not in data]
        if missing:
            return jsonify({"status": "error",
                            "message": f"Missing fields: {', '.join(missing)}"}), 400
        try:
            result = can_manager.set_pid_outer_via_can(
                joint_name=data["joint_name"],
                dof_index=int(data["dof_index"]),
                kp=float(data["kp"]),
                ki=float(data["ki"]),
                kd=float(data["kd"]),
                stiffness=float(data["stiffness"]),
                influence=float(data["cascade"]),
            )
        except KeyError as exc:
            return jsonify({"status": "error",
                            "message": f"Unknown joint: {exc}"}), 400
        except Exception as exc:
            current_app.logger.exception("Impedance sync_outer failed")
            return jsonify({"status": "error", "message": str(exc)}), 500
        return jsonify({"status": "success", **result})

    # ── Joint Configuration ───────────────────────────────────────────

    @app.route('/api/joints', methods=['GET'])
    def api_joints():
        """Return available joints and their config (for dynamic UI population)."""
        joints = {}
        for joint_name, joint_info in JOINTS.items():
            enriched = dict(joint_info)
            dofs = []
            for dof in joint_info.get("dofs", []):
                dof_index = int(dof["index"])
                min_raw = MIN_ANGLES.get(joint_name)
                max_raw = MAX_ANGLES.get(joint_name)
                if isinstance(min_raw, dict):
                    min_angle = min_raw.get(dof_index)
                else:
                    min_angle = min_raw
                if isinstance(max_raw, dict):
                    max_angle = max_raw.get(dof_index)
                else:
                    max_angle = max_raw

                dofs.append({
                    **dof,
                    "min_angle": min_angle,
                    "max_angle": max_angle,
                })
            enriched["dofs"] = dofs
            joints[joint_name] = enriched

        return jsonify({"status": "success", "joints": joints})

    @app.route('/')
    def index():
        return render_template('index.html')

    return app
