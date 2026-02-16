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
from waypoint_types import (
    WaypointBatch, ValidationResult,
    build_batch, validate_batch, deduplicate_batch, batch_to_dicts,
    get_dof_limits, MAX_BATCH_SIZE,
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

def register_routes(app, serial_manager: SerialManager, can_manager=None, stream_test_service=None):
    """
    Register all Flask routes for the joint controller application.
    
    Args:
        app: Flask application instance
        serial_manager: SerialManager instance for hardware communication
    """

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

    def can_unavailable_response():
        if not CAN_AVAILABLE or can_manager is None:
            return jsonify({
                "status": "error",
                "message": "CAN features not available on this host (python-can missing or disabled)."
            }), 503
        return None

    # Maximum time sync age before waypoints are rejected (ms)
    MAX_SYNC_AGE_MS = 2000.0

    def _waypoint_preflight(joint_name: str, waypoints_raw: list):
        """
        Validate preconditions for sending a waypoint batch.

        Returns:
            (WaypointBatch, None)               on success
            (None, (jsonify_response, status))   on failure
        """
        # 1. CAN connected
        if not can_manager.is_connected():
            return None, (jsonify({
                "status": "error",
                "message": "CAN bus not connected"
            }), 503)

        # 2. Time sync freshness
        sync_age = can_manager.last_time_sync_age_ms()
        if sync_age is None:
            return None, (jsonify({
                "status": "error",
                "message": "No time sync sent yet — send time sync first"
            }), 400)
        if sync_age > MAX_SYNC_AGE_MS:
            return None, (jsonify({
                "status": "error",
                "message": f"Time sync stale ({sync_age:.0f}ms ago, limit {MAX_SYNC_AGE_MS:.0f}ms)"
            }), 400)

        # 3. Build batch dataclass from raw dicts
        try:
            batch = build_batch(joint_name, waypoints_raw)
        except Exception as exc:
            return None, (jsonify({
                "status": "error",
                "message": f"Invalid waypoint data: {exc}"
            }), 400)

        # 4. Validate batch (angle limits, monotonicity, velocity)
        result: ValidationResult = validate_batch(batch)
        if not result.ok:
            error_details = [
                {"index": e.index, "field": e.field, "message": e.message}
                for e in result.errors
            ]
            return None, (jsonify({
                "status": "error",
                "message": f"Waypoint validation failed ({len(result.errors)} error(s))",
                "validation_errors": error_details,
            }), 400)

        # Log warnings (non-fatal)
        for w in result.warnings:
            logger.warning(f"[Waypoint] {w}")

        # 5. Deduplicate (remove zero-step quantization duplicates)
        original_count = len(batch.entries)
        batch = deduplicate_batch(batch)
        if len(batch.entries) < original_count:
            logger.info(
                f"[Waypoint] Deduplication removed "
                f"{original_count - len(batch.entries)} zero-step waypoints "
                f"({original_count} → {len(batch.entries)})"
            )

        return batch, None

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

    @app.route('/analysis')
    def analysis_page():
        """
        Serves the movement analysis page for CSV visualization.
        """
        return render_template('analysis.html')

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
        4. Sends time sync automatically after successful discovery
        """
        discovered = {}
        can_sent = False
        time_synced = False

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

        # Auto-send time sync after successful discovery
        if discovered and can_manager and can_manager.is_connected():
            try:
                can_manager.send_time_sync()
                time_synced = True
                current_app.logger.info("Discovery: auto time sync sent")
            except Exception as e:
                current_app.logger.warning(f"Auto time sync after discovery failed: {e}")

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
            "time_synced": time_synced,
            "discovered": discovered,
            "can_discovered": {str(k): v for k, v in can_discovered.items()},
            "mappings": serial_manager.get_joint_to_port_mapping(),
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

            # Validate angles against physical limits
            joint_key = joint.upper()
            if joint_key in JOINTS:
                dof_limits = get_dof_limits(joint_key)
                for dof_idx, angle in enumerate(processed_angles):
                    if angle is not None and dof_idx in dof_limits:
                        lo, hi = dof_limits[dof_idx]
                        if angle < lo or angle > hi:
                            return jsonify({
                                "status": "error",
                                "message": f"DOF{dof_idx} angle {angle:.2f}° out of range [{lo:.1f}, {hi:.1f}]"
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

    @app.route('/can/waypoint_batch', methods=['POST'])
    def send_can_waypoint_batch():
        """
        Send a batch of waypoints in deterministic order.

        Request body:
        {
            "joint": "ANKLE_RIGHT",
            "waypoints": [
                {"angles_deg": [10.0, 5.0, null], "t_offset_ms": 500},
                {"angles_deg": [15.0, 7.0, null], "t_offset_ms": 600},
                ...
            ]
        }

        Preflight checks: CAN connected, time sync fresh, angle/velocity/
        monotonicity validation.  Per-joint concurrency lock prevents
        interleaved batches.
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        data = request.get_json() or {}
        joint = data.get('joint')
        waypoints_raw = data.get('waypoints', [])

        if not joint:
            return jsonify({
                "status": "error",
                "message": "Missing 'joint' parameter"
            }), 400

        if not waypoints_raw or not isinstance(waypoints_raw, list):
            return jsonify({
                "status": "error",
                "message": "Missing or invalid 'waypoints' array"
            }), 400

        # --- Preflight: CAN, time sync, build & validate batch ---
        batch, preflight_error = _waypoint_preflight(joint, waypoints_raw)
        if preflight_error:
            return preflight_error

        try:
            # NOTE: Interpolation mode is NOT forced here anymore.
            # The current mode (set by oscillation test or other commands) is used.
            # - LINEAR: For dense trajectories (many waypoints, small delta-t)
            # - COSINE: For sparse trajectories (few waypoints, large delta-t)
            # User should set mode via oscillation test or a dedicated control.

            # Use deduplicated batch (from preflight) instead of raw
            result = can_manager.send_waypoint_batch(
                joint, batch_to_dicts(batch),
                batch_id=batch.batch_id,
            )

            sent = result["sent"]
            total = result["total"]

            # Determine status: success / partial / error
            # HTTP codes: 200 = all sent, 207 = partial, 502 = none sent
            if sent == total:
                status = "success"
                http_code = 200
            elif sent > 0:
                status = "partial"
                http_code = 207  # Multi-Status — some waypoints sent
            else:
                status = "error"
                http_code = 502  # Upstream failure — CAN layer sent nothing

            return jsonify({
                "status": status,
                "message": f"Batch {batch.batch_id}: {sent}/{total} waypoints sent to {joint}",
                "result": result,
            }), http_code

        except ValueError as exc:
            msg = str(exc)
            # Per-joint concurrency conflict → 409
            if "already in progress" in msg:
                return jsonify({
                    "status": "error",
                    "message": msg
                }), 409
            return jsonify({
                "status": "error",
                "message": msg
            }), 400
        except Exception as exc:
            logger.exception("Failed to send waypoint batch")
            return jsonify({
                "status": "error",
                "message": f"Unable to send waypoint batch: {exc}"
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

    # ===============================================================
    # CAN ENCODER STREAMING ROUTES
    # ===============================================================

    @app.route('/can/encoder_stream/start', methods=['POST'])
    def start_encoder_stream():
        """
        Start encoder streaming via CAN at 200Hz.
        
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
        
        This is useful for on-demand position queries before sending waypoints,
        without requiring continuous encoder streaming.
        
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
                return jsonify({
                    "status": "success",
                    "message": f"Zero set for {joint_name} DOF {dof_index}",
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

    @app.route('/can/interpolation_mode', methods=['POST'])
    def set_interpolation_mode():
        """
        Set waypoint interpolation mode.
        
        POST body: { "mode": "linear" | "cosine" }
        
        - "linear": Step response for PID tuning (sharp transitions)
        - "cosine": Smooth S-curve for operational movements (reduced vibrations)
        """
        unavailable = can_unavailable_response()
        if unavailable:
            return unavailable

        try:
            data = request.get_json() or {}
            mode = data.get("mode", "linear")
            
            if mode not in ["linear", "cosine"]:
                return jsonify({
                    "status": "error",
                    "message": f"Invalid mode '{mode}'. Use 'linear' or 'cosine'."
                }), 400
            
            result = can_manager.set_interpolation_mode(mode)
            return jsonify({
                "status": "success",
                "message": f"Interpolation mode set to: {mode}",
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to set interpolation mode")
            return jsonify({
                "status": "error",
                "message": f"Unable to set interpolation mode: {exc}"
            }), 500

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

    @app.route('/can/reanchor_interval', methods=['POST'])
    def set_reanchor_interval():
        """Set waypoint re-anchor interval via CAN."""
        try:
            data = request.get_json(silent=True)
            if not data or 'interval' not in data:
                return jsonify({
                    "status": "error",
                    "message": "Missing 'interval' field"
                }), 400
            requested_interval = int(data['interval'])
            result = can_manager.set_reanchor_interval(requested_interval)
            applied_interval = int(result.get("interval", requested_interval))
            clamped = bool(result.get("clamped", False))
            message = f"WP re-anchor interval set to: {applied_interval}" + \
                      (" (disabled)" if applied_interval == 0 else " WPs")
            if clamped:
                message += f" [clamped from {requested_interval}]"
            return jsonify({
                "status": "success",
                "message": message,
                "result": result
            })
        except Exception as exc:
            logger.exception("Failed to set re-anchor interval")
            return jsonify({
                "status": "error",
                "message": f"Unable to set re-anchor interval: {exc}"
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
                if can_manager and can_manager.is_connected():
                    result = can_manager.set_zero_via_can(joint, dof_int)
                    if result.get("success"):
                        message = f"Zero set via CAN for {joint} DOF {dof_int}"
                    else:
                        message = f"Set-zero CAN failed: {result.get('error', 'unknown')}"
                else:
                    handler.send_new_command(joint, dof, COMMANDS['SET_ZERO'])
                    message = f"Current position set as zero for {joint} DOF {dof} (serial)"
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
                # Set PID is operational → CAN first, serial fallback
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
                    handler.set_pid_for_joint_dof(joint, dof_index, motor_type, kp, ki, kd, tau)
                    message = f"PID set via serial for {joint} DOF {dof_index} motor {motor_type}"
            elif cmd == "set-pid-outer":
                # Set PID outer is operational → CAN first, serial fallback
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
                    handler.set_outer_pid_for_joint_dof(joint, dof_index, kp, ki, kd, stiffness, cascade)
                    message = f"Outer PID set via serial for {joint} DOF {dof_index}"
            elif cmd == "load-pid-all":
                # Load PID is operational → CAN first, serial fallback
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
                        handler.get_pid_for_joint_dof(joint, dof_index, 1)
                        handler.get_pid_for_joint_dof(joint, dof_index, 2)
                        handler.get_outer_pid_for_joint_dof(joint, dof_index)
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
            elif cmd == "cascade-speed-scaling":
                # Cascade speed scaling is operational → CAN first, serial fallback
                enabled = int(data.get('enabled', 1))
                min_factor = float(data.get('min_factor', 0.3))
                speed_low = float(data.get('speed_low', 3.0))
                speed_high = float(data.get('speed_high', 15.0))
                ema_enabled = int(data.get('ema_enabled', 1))
                ema_alpha = float(data.get('ema_alpha', 0.5))
                tau_enabled = int(data.get('tau_enabled', 0))
                tau_high = float(data.get('tau_high', 0.03))
                tau_speed = float(data.get('tau_speed', 10.0))
                jema_enabled = int(data.get('jema_enabled', 0))
                jema_alpha = float(data.get('jema_alpha', 0.5))
                jema_speed = float(data.get('jema_speed', 15.0))
                fric_enabled = int(data.get('fric_enabled', 0))
                fric_torque = float(data.get('fric_torque', 30.0))
                fric_speed = float(data.get('fric_speed', 3.0))
                if can_manager and can_manager.is_connected():
                    css_params = {
                        'enabled': float(enabled),
                        'min': min_factor,
                        'low': speed_low,
                        'high': speed_high,
                        'ema_en': float(ema_enabled),
                        'ema_alpha': ema_alpha,
                        'tau_en': float(tau_enabled),
                        'tau_high': tau_high,
                        'tau_speed': tau_speed,
                        'jema_en': float(jema_enabled),
                        'jema_alpha': jema_alpha,
                        'jema_speed': jema_speed,
                        'fric_en': float(fric_enabled),
                        'fric_torque': fric_torque,
                        'fric_speed': fric_speed,
                    }
                    can_manager.cascade_speed_scaling_via_can(joint, css_params)
                    message = f"Cascade speed scaling set via CAN for {joint}"
                else:
                    params = (f"{COMMANDS['CASCADE_SPEED_SCALING']}:ENABLED={enabled}:MIN={min_factor}"
                              f":LOW={speed_low}:HIGH={speed_high}:EMA_EN={ema_enabled}:EMA_ALPHA={ema_alpha}"
                              f":TAU_EN={tau_enabled}:TAU_HIGH={tau_high}:TAU_SPEED={tau_speed}"
                              f":JEMA_EN={jema_enabled}:JEMA_ALPHA={jema_alpha}:JEMA_SPEED={jema_speed}"
                              f":FRIC_EN={fric_enabled}:FRIC_TORQUE={fric_torque}:FRIC_SPEED={fric_speed}")
                    handler.send_new_command(joint, 'ALL', params)
                    message = (f"Cascade speed scaling set via serial for {joint}")
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

    # ------------------------------------------------------------------
    # Stream Test endpoints
    # ------------------------------------------------------------------

    @app.route('/stream_test/start', methods=['POST'])
    def stream_test_start():
        if stream_test_service is None:
            return jsonify({"status": "error", "message": "Stream test service not available"}), 503
        try:
            data = request.get_json() or {}
            result = stream_test_service.start(data)
            return jsonify({"status": "success", **result})
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 409
        except Exception as exc:
            logger.exception("Stream test start error")
            return jsonify({"status": "error", "message": str(exc)}), 500

    @app.route('/stream_test/stop', methods=['POST'])
    def stream_test_stop():
        if stream_test_service is None:
            return jsonify({"status": "error", "message": "Stream test service not available"}), 503
        try:
            data = request.get_json() or {}
            session_id = data.get("session_id", "")
            reason = data.get("reason", "operator_stop")
            result = stream_test_service.stop(session_id, reason=reason)
            return jsonify(result)
        except ValueError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 400
        except Exception as exc:
            logger.exception("Stream test stop error")
            return jsonify({"status": "error", "message": str(exc)}), 500

    @app.route('/stream_test/status', methods=['GET'])
    def stream_test_status():
        if stream_test_service is None:
            return jsonify({"status": "error", "message": "Stream test service not available"}), 503
        session_id = request.args.get("session_id")
        result = stream_test_service.get_status(session_id)
        return jsonify({"status": "success", **result})

    @app.route('/stream_test/metrics', methods=['GET'])
    def stream_test_metrics():
        if stream_test_service is None:
            return jsonify({"status": "error", "message": "Stream test service not available"}), 503
        metrics = stream_test_service.get_metrics()
        return jsonify({"status": "success", "metrics": metrics})

    @app.route('/stream_test/events', methods=['GET'])
    def stream_test_events():
        if stream_test_service is None:
            return jsonify({"status": "error", "message": "Stream test service not available"}), 503
        session_id = request.args.get("session_id")
        after_seq = request.args.get("after_seq", 0, type=int)
        events = stream_test_service.get_events(session_id, after_seq)
        return jsonify({"status": "success", "events": events})

    @app.route('/')
    def index():
        return render_template('index.html')

    return app
