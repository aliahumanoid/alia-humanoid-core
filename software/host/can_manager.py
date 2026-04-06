"""
CAN bus manager for host ↔ joint-controller communication.

Provides a high-level API for:
- Connecting/disconnecting to a python-can interface
- Sending protocol-defined frames (impedance targets, commands, emergency stop)
- Receiving status telemetry and exposing it to the Flask app/UI

CAN ID allocation is documented in docs/CAN_SYSTEM_ARCHITECTURE.md.
Key ranges:
- 0x000-0x01D: System control + operational commands (Host → Controller)
- 0x140-0x280: Motor commands (Controller → Motors, internal)
- 0x400-0x550: Status feedback + diagnostics (Controller → Host)
"""
from __future__ import annotations

import json
import logging
import struct
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, Optional

from config import (
    CAN_ID_EVENT_NOTICE,
    CAN_ID_FAULT_STATUS,
    CAN_ID_FAULT_SNAPSHOT_CTRL,
    CAN_ID_FAULT_SNAPSHOT_DATA,
    CAN_ID_FAULT_SNAPSHOT_META,
    CAN_ID_HEALTH_STATUS,
    CAN_ID_RETENSION_PROBE_RESULT,
    JOINTS,
)
from diagnostic_history import DiagnosticHistoryWriter
from jetson_controller.protocol import (
    DIAG_EVENT_NAMES,
    DIAG_FAULT_NAMES,
    DIAG_PHASE_NAMES,
    DIAG_REBOOT_REASON_NAMES,
    DIAG_SEVERITY_NAMES,
    DIAG_SOURCE_NAMES,
    FAULT_SNAPSHOT_PENDING_TIMEOUT_S,
    STARTUP_REASON_NAMES,
    FaultSnapshotMeta,
    decode_fault_mask,
    decode_fault_snapshot_blob,
    decode_fault_snapshot_chunk,
    decode_fault_snapshot_meta,
    decode_source_id,
    encode_fault_snapshot_ctrl,
)
from motor_can_bench import (
    CONTROL_COMMANDS,
    DEFAULT_MOTOR_ID,
    DEFAULT_OUTPUT_RATIO,
    READ_COMMANDS,
    build_default_label,
    build_control_frame,
    build_read_frame,
    build_zero_frame,
    bytes_hex,
    csv_fieldnames,
    decode_feedback_frame,
    decode_raw_frame,
    decode_multi_frame,
    decode_pid_frame,
    decode_single_frame,
    decode_acceleration_frame,
    default_csv_path,
    generate_target,
    parse_extra_commands,
    write_csv_rows,
)
from probe_history import ProbeHistoryWriter
from serial_logger import SerialLogger

try:
    import can  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    can = None  # type: ignore


def _decode_state_bits(state_bits: int) -> Dict[str, bool]:
    return {
        "config_valid": bool(state_bits & 0x01),
        "controller_ready": bool(state_bits & 0x02),
        "motion_ready": bool(state_bits & 0x04),
        "motor_power_enabled": bool(state_bits & 0x08),
        "impedance_enabled": bool(state_bits & 0x10),
        "watchdog_armed": bool(state_bits & 0x20),
        "watchdog_warning": bool(state_bits & 0x40),
        "snapshot_available": bool(state_bits & 0x80),
    }

class CanManager:
    """High-level helper that manages python-can Bus lifecycle and protocol helpers."""

    DEFAULT_BITRATE = 1_000_000  # 1 Mbps (maximum speed test)
    MOTOR_ID_BASE = 0x140
    MOTOR_TEST_LOG_DIR = Path(__file__).resolve().parent / "logs" / "motor_can_bench"
    PROBE_HISTORY_DIR = Path(__file__).resolve().parent / "logs" / "probe_history"
    DIAGNOSTIC_HISTORY_DIR = Path(__file__).resolve().parent / "logs" / "diagnostic_history"

    def __init__(self, socketio=None, comm_logger: Optional[SerialLogger] = None) -> None:
        if can is None:
            raise RuntimeError("python-can is not installed. Install python-can to use CAN features.")

        self.socketio = socketio
        self.logger = logging.getLogger(__name__)
        self.comm_logger = comm_logger
        self._bus: Optional[can.BusABC] = None
        self._listener_thread: Optional[threading.Thread] = None
        self._listener_stop = threading.Event()
        self._lock = threading.Lock()
        self._current_config: Optional[Dict[str, Any]] = None
        self._last_status_by_joint: Dict[str, Dict[str, Any]] = {}
        self._status_messages: deque = deque(maxlen=200)
        self._last_rx_timestamp: Optional[float] = None
        self._stats = {
            "tx_frames": 0,
            "rx_frames": 0,
            "errors": 0,
            "last_error": None,
        }
        self._joint_id_lookup = {data["id"]: name for name, data in JOINTS.items()}
        
        # Encoder streaming state
        self._encoder_stream_active = False
        self._encoder_stream_callback = None
        self._encoder_stream_data: deque = deque(maxlen=500)  # Buffer for streaming data
        
        # Last known encoder angles per joint (persists even when streaming stops)
        # Format: {joint_id: {"angles_deg": [...], "timestamp_ms": int, "timestamp": float}}
        self._last_encoder_angles: Dict[int, Dict[str, Any]] = {}
        
        # Encoder offset cross-validation via CAN
        self._encoder_offset_buffer: Dict[int, float] = {}  # enc_index → offset_rad
        self._encoder_offset_event = threading.Event()
        self._encoder_offset_expected: int = 0  # how many responses expected

        # CAN set-zero completion event
        self._zero_complete_event = threading.Event()

        # Impedance gain cache per (joint_name, dof_index)
        # Avoids sending hardcoded defaults when gains are omitted (None)
        self._impedance_gain_cache: Dict[tuple, Dict[str, float]] = {}

        # Impedance TX lock — serializes multi-frame SET_IMPEDANCE sends
        self._impedance_tx_lock = threading.Lock()

        # Single-motor direct CAN bench state
        self._motor_test_lock = threading.Lock()
        self._motor_test_abort_requested = threading.Event()
        self._motor_response_condition = threading.Condition()
        self._motor_recent_frames: Dict[tuple[int, int], Dict[str, Any]] = {}
        self._motor_frame_history: deque = deque(maxlen=256)
        self._motor_frame_seq = 0

        # Impedance control — joint state feedback from firmware
        # Format: {joint_name: {dof: {"q_deg": float, "dq_deg_s": float,
        #          "tau_a": int, "tau_b": int, "status": int, "timestamp": float}}}
        self._joint_state: Dict[str, Dict[int, Dict[str, Any]]] = {}
        self._health_status: Dict[str, Dict[str, Any]] = {}
        self._fault_status: Dict[str, Dict[str, Any]] = {}
        self._fault_snapshot_meta: Dict[str, Dict[str, Any]] = {}
        self._fault_snapshots: Dict[str, Dict[str, Any]] = {}
        self._diagnostic_events: deque[Dict[str, Any]] = deque(maxlen=500)
        self._health_status_pending: Dict[tuple[int, int], Dict[str, Any]] = {}
        self._fault_snapshot_pending: Dict[int, Dict[str, Any]] = {}
        self._fault_snapshot_request_seq = 0
        self._retension_probe_state: Dict[str, Dict[int, Dict[str, Any]]] = {}
        self._retension_probe_pending: Dict[tuple[int, int], Dict[str, Any]] = {}
        self._probe_history = ProbeHistoryWriter(self.PROBE_HISTORY_DIR, source="flask_can_manager")
        self._diagnostic_history = DiagnosticHistoryWriter(
            self.DIAGNOSTIC_HISTORY_DIR, source="flask_can_manager"
        )

        # Configuration persistence
        self._config_file = "can_config.json"

    # ------------------------------------------------------------------
    # Configuration persistence
    # ------------------------------------------------------------------
    def save_config(self) -> bool:
        """
        Save the current CAN configuration to a JSON file.
        
        Returns:
            bool: True if saved successfully, False otherwise
        """
        if not self._current_config:
            self.logger.warning("No CAN config to save")
            return False
        
        try:
            with open(self._config_file, 'w') as f:
                json.dump(self._current_config, f, indent=2)
            self.logger.info(f"CAN config saved to {self._config_file}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to save CAN config: {e}")
            return False

    def load_config(self) -> Optional[Dict[str, Any]]:
        """
        Load CAN configuration from file.
        
        Returns:
            Optional[Dict[str, Any]]: Configuration dict or None if not found
        """
        try:
            with open(self._config_file, 'r') as f:
                config = json.load(f)
            self.logger.info(f"Loaded CAN config from {self._config_file}: {config}")
            return config
        except FileNotFoundError:
            self.logger.info("No saved CAN config found")
            return None
        except Exception as e:
            self.logger.error(f"Failed to load CAN config: {e}")
            return None

    def has_saved_config(self) -> bool:
        """Check if a saved CAN configuration exists."""
        import os
        return os.path.exists(self._config_file)

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Initialize python-can Bus using provided configuration dictionary.

        Args:
            config: Dict with at least 'interface' and 'channel'. Optional 'bitrate'.

        Returns:
            Dict with connection metadata for UI/telemetry.
        """
        if can is None:
            raise RuntimeError("python-can is not available in this environment.")

        interface = config.get("interface")
        channel = config.get("channel")
        bitrate = int(config.get("bitrate") or self.DEFAULT_BITRATE)

        if not interface or not channel:
            raise ValueError("interface and channel are required to connect to CAN bus.")

        self._log_can_info(f"Connecting CAN interface={interface} channel={channel} bitrate={bitrate}")
        with self._lock:
            self.disconnect()

            # For SLCAN (serial interface), explicitly configure bitrate before opening
            if interface == "serial":
                import serial
                self.logger.info("Configuring SLCAN bitrate for MKS CANable...")
                try:
                    ser = serial.Serial(channel, baudrate=115200, timeout=0.5)
                    # Close channel first
                    ser.write(b'C\r')
                    time.sleep(0.1)
                    # Set bitrate: S6 = 500 kbps, S8 = 1 Mbps
                    if bitrate == 1_000_000:
                        bitrate_cmd = b'S8\r'
                    elif bitrate == 500_000:
                        bitrate_cmd = b'S6\r'
                    else:
                        # Default to 500 kbps for unsupported bitrates
                        bitrate_cmd = b'S6\r'
                    ser.write(bitrate_cmd)
                    time.sleep(0.1)
                    # Open channel
                    ser.write(b'O\r')
                    time.sleep(0.1)
                    ser.close()
                    self._log_can_info(f"SLCAN configured: bitrate={bitrate}")
                except Exception as exc:
                    self.logger.warning("Failed to configure SLCAN: %s", exc)

            self.logger.info("Opening CAN bus: interface=%s channel=%s bitrate=%s", interface, channel, bitrate)
            bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
            self._bus = bus
            self._current_config = {
                "interface": interface,
                "channel": channel,
                "bitrate": bitrate,
                "channel_info": getattr(bus, "channel_info", None),
            }
            self._listener_stop.clear()
            self._listener_thread = threading.Thread(
                target=self._listen_loop,
                name="CanRxListener",
                daemon=True,
            )
            self._listener_thread.start()

        self.logger.info("CAN bus ready: %s", self._current_config)
        self._log_can_info(f"CAN bus ready: {self._current_config}")
        
        # Save configuration for auto-reconnect on next startup
        self.save_config()
        
        return self._current_config

    def disconnect(self) -> None:
        """Shutdown listener thread and close CAN interface."""
        self._listener_stop.set()
        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=1.0)
        self._listener_thread = None

        if self._bus is not None:
            try:
                self._bus.shutdown()
                self.logger.info("CAN bus closed")
                self._log_can_info("CAN bus closed")
            except Exception as exc:  # pragma: no cover - cleanup best effort
                self.logger.warning("Error while closing CAN bus: %s", exc)
        self._bus = None
        self._current_config = None
        with self._motor_response_condition:
            self._motor_recent_frames.clear()
            self._motor_response_condition.notify_all()

    def is_connected(self) -> bool:
        """Return True if CAN bus is initialized."""
        return self._bus is not None

    # ------------------------------------------------------------------
    # Protocol helpers
    # ------------------------------------------------------------------

    def send_emergency_stop(self, reason_code: int = 0) -> Dict[str, Any]:
        """Broadcast emergency stop frame."""
        self._ensure_connection()
        payload = bytes([reason_code & 0xFF]) + bytes(7)
        self._send_frame(0x000, payload, context=f"E-Stop reason={reason_code}")
        return {"reason": reason_code}

    def send_identify_request(self) -> Dict[str, Any]:
        """
        Broadcast joint identification request to all controllers.
        
        Controllers will emit EVT:JOINT messages on their serial ports
        periodically for 3 seconds after receiving this command.
        """
        self._ensure_connection()
        payload = bytes(8)  # Empty payload, just the command ID matters
        self._send_frame(0x008, payload, context="Joint identify request")
        self._log_can_info("Joint identification broadcast requested")
        return {"requested": True}

    def _next_fault_snapshot_seq(self) -> int:
        self._fault_snapshot_request_seq = (self._fault_snapshot_request_seq + 1) & 0xFF
        return self._fault_snapshot_request_seq

    def request_fault_snapshot_meta(self, joint_name: str) -> Dict[str, Any]:
        """Query whether a controller-local fault snapshot is available."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        seq = self._next_fault_snapshot_seq()
        arb_id, payload = encode_fault_snapshot_ctrl(joint_id, sub_cmd=0x00, seq=seq)
        self._send_frame(arb_id, payload, context=f"FAULT_SNAPSHOT QUERY_META joint={joint_name}")
        return {"joint_name": joint_name, "joint_id": joint_id, "sub_cmd": "QUERY_META", "seq": seq}

    def begin_fault_snapshot_dump(self, joint_name: str, *, first_chunk: int = 0) -> Dict[str, Any]:
        """Request a full fault snapshot dump starting from the given chunk."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        seq = self._next_fault_snapshot_seq()
        arb_id, payload = encode_fault_snapshot_ctrl(
            joint_id,
            sub_cmd=0x01,
            arg0=first_chunk,
            seq=seq,
        )
        self._send_frame(
            arb_id,
            payload,
            context=f"FAULT_SNAPSHOT BEGIN_DUMP joint={joint_name} chunk={first_chunk}",
        )
        return {
            "joint_name": joint_name,
            "joint_id": joint_id,
            "sub_cmd": "BEGIN_DUMP",
            "first_chunk": first_chunk,
            "seq": seq,
        }

    def request_fault_snapshot_chunk(
        self,
        joint_name: str,
        *,
        chunk_index: int,
        snapshot_id: int = 0xFF,
    ) -> Dict[str, Any]:
        """Request one fault snapshot chunk again."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        seq = self._next_fault_snapshot_seq()
        arb_id, payload = encode_fault_snapshot_ctrl(
            joint_id,
            sub_cmd=0x02,
            snapshot_id=snapshot_id,
            arg0=chunk_index,
            seq=seq,
        )
        self._send_frame(
            arb_id,
            payload,
            context=f"FAULT_SNAPSHOT REQUEST_CHUNK joint={joint_name} chunk={chunk_index}",
        )
        return {
            "joint_name": joint_name,
            "joint_id": joint_id,
            "sub_cmd": "REQUEST_CHUNK",
            "snapshot_id": snapshot_id,
            "chunk_index": chunk_index,
            "seq": seq,
        }

    def clear_fault_snapshot(self, joint_name: str, *, snapshot_id: int = 0xFF) -> Dict[str, Any]:
        """Clear the stored controller-local fault snapshot."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        seq = self._next_fault_snapshot_seq()
        arb_id, payload = encode_fault_snapshot_ctrl(
            joint_id,
            sub_cmd=0x03,
            snapshot_id=snapshot_id,
            seq=seq,
        )
        self._send_frame(arb_id, payload, context=f"FAULT_SNAPSHOT CLEAR joint={joint_name}")
        return {
            "joint_name": joint_name,
            "joint_id": joint_id,
            "sub_cmd": "CLEAR_SNAPSHOT",
            "snapshot_id": snapshot_id,
            "seq": seq,
        }

    def send_startup_sequence(self, joint_name: str, torque: int = 0,
                               duration: int = 0) -> Dict[str, Any]:
        """
        Send startup sequence command via CAN (0x009).

        Triggers recalc_offset for all DOFs on the target joint controller,
        then enters HOLDING state. Replaces serial CMD_STARTUP_SEQUENCE.

        Args:
            joint_name: Joint name (e.g. "KNEE_RIGHT")
            torque: Custom pretension torque (0 = use config default)
            duration: Custom pretension duration ms (0 = use config default)

        Returns:
            Dict with request status
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = struct.pack("<BBhh", joint_id, 0, torque, duration) + bytes(2)
        self._send_frame(0x009, payload,
                         context=f"Startup seq joint={joint_name} torque={torque} dur={duration}")
        self._log_can_info(f"Startup sequence requested for {joint_name}")
        return {"joint": joint_name, "requested": True}

    def start_encoder_stream(self, joint_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Start encoder streaming via CAN at 50Hz.

        Sends control command (0x003) to enable high-frequency angle data.
        Data arrives on 0x410 and is decoded in _handle_encoder_stream().

        If joint_name is provided, validates encoder offsets against saved
        host copy before starting. Blocks streaming on mismatch.
        """
        self._ensure_connection()

        # Cross-validate encoder offsets before streaming
        validation = None
        if joint_name:
            validation = self.validate_encoder_offsets(joint_name)
            if not validation["valid"]:
                self._log_can_info(
                    f"Encoder stream BLOCKED — offset mismatch for {joint_name}"
                )
                self.logger.warning(
                    f"Encoder offset validation failed: {validation['details']}"
                )
                return {
                    "streaming": False,
                    "blocked": True,
                    "reason": "encoder_offset_mismatch",
                    "validation": validation,
                }

        payload = bytes([0x01]) + bytes(7)  # 0x01 = start streaming
        self._send_frame(0x003, payload, context="Encoder stream START")
        self._encoder_stream_active = True
        self._encoder_stream_data.clear()
        self._log_can_info("Encoder streaming started @ 50Hz")
        result: Dict[str, Any] = {"streaming": True}
        if validation:
            result["validation"] = validation
        return result

    def stop_encoder_stream(self) -> Dict[str, Any]:
        """
        Stop encoder streaming via CAN.
        """
        self._ensure_connection()
        payload = bytes([0x00]) + bytes(7)  # 0x00 = stop streaming
        self._send_frame(0x003, payload, context="Encoder stream STOP")
        self._encoder_stream_active = False
        self._log_can_info("Encoder streaming stopped")
        return {"streaming": False}

    def query_encoder_offsets(self, joint_name: str, expected_count: int = 3,
                              timeout: float = 1.0) -> Dict[int, float]:
        """Query encoder offsets from firmware via CAN (0x00A → 0x4B0 response).

        Args:
            joint_name: Joint name (e.g. "KNEE_RIGHT")
            expected_count: Number of encoder responses to wait for
            timeout: Seconds to wait for all responses

        Returns:
            Dict mapping encoder_index → offset_rad
        """
        self._ensure_connection()
        if expected_count <= 0:
            return {}
        joint_id = JOINTS[joint_name]["id"]

        # Prepare buffer
        with self._lock:
            self._encoder_offset_buffer.clear()
            self._encoder_offset_expected = expected_count
            self._encoder_offset_event.clear()

        # Send request
        payload = bytes([joint_id]) + bytes(7)
        self._send_frame(0x00A, payload, context=f"Get encoder offsets joint={joint_name}")

        # Wait for responses
        self._encoder_offset_event.wait(timeout=timeout)

        with self._lock:
            result = dict(self._encoder_offset_buffer)

        if len(result) < expected_count:
            self._log_can_info(
                f"Encoder offset query: got {len(result)}/{expected_count} responses"
            )

        return result

    def _get_external_encoder_count(self, joint_name: str) -> int:
        joint_info = JOINTS.get(joint_name, {})
        return sum(
            1
            for dof in joint_info.get("dofs", [])
            if dof.get("drive_type", "antagonistic_tendon") != "direct_drive"
        )

    def validate_encoder_offsets(self, joint_name: str) -> Dict[str, Any]:
        """Query firmware encoder offsets and validate against saved host copy.

        Returns:
            Dict with 'valid' (bool), 'details' (list of per-encoder results),
            and 'firmware_offsets' (dict of received offsets).
        """
        import os
        import json as _json

        expected_count = self._get_external_encoder_count(joint_name)
        if expected_count == 0:
            return {
                "valid": True,
                "details": ["Joint has no external encoders — skipping offset validation"],
                "firmware_offsets": {},
            }

        firmware_offsets = self.query_encoder_offsets(joint_name, expected_count=expected_count)
        if not firmware_offsets:
            return {"valid": False, "details": ["No response from firmware"], "firmware_offsets": {}}

        # Load saved offsets
        filename = f"calibration_data/{joint_name.lower()}_encoder_offsets.json"
        saved_data = None
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    saved_data = _json.load(f)
            except Exception as e:
                self.logger.error(f"Error loading saved offsets: {e}")

        if saved_data is None:
            # No saved data — save current as baseline, consider valid (first run)
            self._save_encoder_offsets(joint_name, firmware_offsets)
            return {
                "valid": True,
                "details": ["No saved offsets — saved current as baseline"],
                "firmware_offsets": firmware_offsets,
            }

        # Compare
        THRESHOLD = 0.001  # ~0.06° tolerance for float precision
        details = []
        all_valid = True
        for enc_idx, fw_offset in firmware_offsets.items():
            saved_val = saved_data.get("offsets", {}).get(str(enc_idx))
            if saved_val is None:
                details.append(f"Encoder {enc_idx}: no saved reference")
                continue
            delta = abs(fw_offset - saved_val)
            if delta > THRESHOLD:
                details.append(
                    f"Encoder {enc_idx}: MISMATCH fw={fw_offset:.6f} saved={saved_val:.6f} "
                    f"delta={delta:.6f} rad"
                )
                all_valid = False
            else:
                details.append(f"Encoder {enc_idx}: OK (delta={delta:.8f} rad)")

        if all_valid:
            # Update saved offsets (in case of tiny float drift)
            self._save_encoder_offsets(joint_name, firmware_offsets)

        # Emit mismatch warning to UI
        if not all_valid and self.socketio:
            self.socketio.emit(
                "encoder_offset_mismatch",
                {
                    "joint_name": joint_name,
                    "details": details,
                    "firmware_offsets": {str(k): v for k, v in firmware_offsets.items()},
                },
                namespace="/movement",
            )

        return {"valid": all_valid, "details": details, "firmware_offsets": firmware_offsets}

    def set_zero_via_can(self, joint_name: str, dof_index: int,
                         timeout: float = 5.0) -> Dict[str, Any]:
        """Send set-zero command via CAN and wait for completion.

        Args:
            joint_name: Joint name (e.g. "KNEE_RIGHT")
            dof_index: DOF index within the joint
            timeout: Seconds to wait for zero-complete response

        Returns:
            Dict with 'success' (bool) and either 'offsets' or 'error'.
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]

        # Prepare state for receiving offset data + completion summary
        with self._lock:
            self._encoder_offset_buffer.clear()
            self._zero_complete_event.clear()

        # Send set-zero command: [joint_id, dof_index, 0, 0, 0, 0, 0, 0]
        payload = bytes([joint_id, dof_index, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x00B, payload,
                         context=f"Set zero {joint_name} DOF {dof_index}")

        # Wait for zero-complete summary frame (0x4C0)
        if self._zero_complete_event.wait(timeout=timeout):
            with self._lock:
                offsets = dict(self._encoder_offset_buffer)
            # Save new offsets after successful zero
            self._save_encoder_offsets(joint_name, offsets)
            self._log_can_info(
                f"Set-zero complete: {joint_name} DOF {dof_index}, "
                f"{len(offsets)} encoder offsets saved"
            )
            return {"success": True, "offsets": {str(k): v for k, v in offsets.items()}}
        else:
            self.logger.warning(
                f"Set-zero timeout: {joint_name} DOF {dof_index} "
                f"(no response within {timeout}s)"
            )
            return {"success": False, "error": "timeout"}

    def pretension_via_can(self, joint_name: str, dof_index: int,
                           torque: int = 0) -> Dict[str, Any]:
        """Send pretension command via CAN for a single DOF."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        torque_lo = torque & 0xFF
        torque_hi = (torque >> 8) & 0xFF
        payload = bytes([joint_id, dof_index, torque_lo, torque_hi, 0, 0, 0, 0])
        self._send_frame(0x00C, payload,
                         context=f"Pretension {joint_name} DOF {dof_index} torque={torque}")
        return {"success": True}

    def pretension_all_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send pretension-all command via CAN."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x00D, payload,
                         context=f"Pretension ALL {joint_name}")
        return {"success": True}

    def release_via_can(self, joint_name: str, dof_index: int,
                        torque: int = 0) -> Dict[str, Any]:
        """Send release command via CAN for a single DOF."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        torque_lo = torque & 0xFF
        torque_hi = (torque >> 8) & 0xFF
        payload = bytes([joint_id, dof_index, torque_lo, torque_hi, 0, 0, 0, 0])
        self._send_frame(0x00E, payload,
                         context=f"Release {joint_name} DOF {dof_index}")
        return {"success": True}

    def release_all_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send release-all command via CAN."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x00F, payload,
                         context=f"Release ALL {joint_name}")
        return {"success": True}

    def recalc_offset_via_can(self, joint_name: str, dof_index: int,
                              torque: int = 0, duration: int = 0) -> Dict[str, Any]:
        """Send recalc-offset command via CAN."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = struct.pack("<BBhh", joint_id, dof_index, torque, duration) + bytes(4)
        self._send_frame(0x010, payload,
                         context=f"Recalc offset {joint_name} DOF {dof_index}")
        return {"success": True}

    def save_pid_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send save-PID-to-flash command via CAN (0x011)."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x011, payload, context=f"Save PID {joint_name}")
        return {"success": True}

    def load_pid_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send load-PID-from-flash command via CAN (0x012)."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x012, payload, context=f"Load PID {joint_name}")
        return {"success": True}

    # MCP2515 has only 2 receive buffers.  Back-to-back frames at 250 kbps
    # can overflow the RX FIFO before the firmware polls.  A short pause
    # between frames gives the 500 Hz control loop time to drain the buffer.
    _MULTI_FRAME_DELAY_S = 0.003  # 3 ms — ~1.5 control-loop iterations
    _IMPEDANCE_DEFAULTS = {
        "kp": 8.0,
        "ki": 8.0,
        "kd": 0.08,
        "tau_ff": 0,
        "kp_inner": 10.0,
        "ki_inner": 1.0,
        "kd_inner": 0.25,
    }

    def set_pid_via_can(self, joint_name: str, dof_index: int, motor_type: int,
                        kp: float, ki: float, kd: float, tau: float) -> Dict[str, Any]:
        """Send SET_PID via CAN (0x013) — 4 sequential frames.

        Args:
            joint_name: Joint name (e.g. "KNEE_RIGHT")
            dof_index: DOF index
            motor_type: 1=agonist, 2=antagonist
            kp, ki, kd, tau: PID parameters
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        for seq, val in enumerate([kp, ki, kd, tau]):
            if seq > 0:
                time.sleep(self._MULTI_FRAME_DELAY_S)
            payload = struct.pack("<BBBBf", joint_id, dof_index, motor_type, seq, val)
            self._send_frame(0x013, payload,
                             context=f"SET_PID seq={seq} val={val:.4f}")
        return {"success": True}

    def set_pid_outer_via_can(self, joint_name: str, dof_index: int,
                               kp: float, ki: float, kd: float,
                               stiffness: float, influence: float) -> Dict[str, Any]:
        """Send SET_PID_OUTER via CAN (0x014) — 5 sequential frames.

        Args:
            joint_name: Joint name (e.g. "KNEE_RIGHT")
            dof_index: DOF index
            kp, ki, kd: Outer PID gains
            stiffness: Stiffness in degrees
            influence: Cascade influence factor
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        for seq, val in enumerate([kp, ki, kd, stiffness, influence]):
            if seq > 0:
                time.sleep(self._MULTI_FRAME_DELAY_S)
            payload = struct.pack("<BBBBf", joint_id, dof_index, seq, 0, val)
            self._send_frame(0x014, payload,
                             context=f"SET_PID_OUTER seq={seq} val={val:.4f}")
        return {"success": True}

    def friction_ff_via_can(self, joint_name: str,
                            params: Dict[str, float]) -> Dict[str, Any]:
        """Send FRICTION_FF parameters via CAN (0x015).

        Each parameter is sent as a separate frame: [joint_id, param_id, float32(4), 0, 0].
        A final frame with param_id=0xFF signals completion.

        Args:
            joint_name: Joint name
            params: Dict of parameter name → value. Keys: fric_en, fric_torque, fric_speed
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]

        param_map = {"fric_en": 0, "fric_torque": 1, "fric_speed": 2}

        first = True
        for name, value in params.items():
            pid = param_map.get(name)
            if pid is not None:
                if not first:
                    time.sleep(self._MULTI_FRAME_DELAY_S)
                first = False
                payload = struct.pack("<BBf", joint_id, pid, float(value)) + bytes(2)
                self._send_frame(0x015, payload,
                                 context=f"FRICTION_FF param={name} val={value}")

        # Send apply marker
        time.sleep(self._MULTI_FRAME_DELAY_S)
        payload = struct.pack("<BBf", joint_id, 0xFF, 0.0) + bytes(2)
        self._send_frame(0x015, payload, context="FRICTION_FF apply")
        return {"success": True}

    # ================================================================
    # IMPEDANCE CONTROL (Scenario B — SET_IMPEDANCE / IMPEDANCE_CTRL)
    # ================================================================

    def send_impedance_target(self, joint_name: str, dof_index: int,
                               q_target: float, dq_target: float,
                               stiffness: float,
                               kp: float = None, ki: float = None, kd: float = None,
                               tau_ff: int = None,
                               kp_inner: float = None, ki_inner: float = None,
                               kd_inner: float = None) -> Dict[str, Any]:
        """Send SET_IMPEDANCE via CAN (0x01D) — 1 to 4 frames.

        Variable-frame protocol with has_more flag (bit 7 of byte 1):
          Byte 1 = [has_more:1][seq:3][dof:4]

        Frame 0 (seq=0): [joint_id, flags, q×100, dq×10, stiff×10]                   — always sent
        Frame 1 (seq=1): [joint_id, flags, Kp×100, Ki×100, Kd×100]                   — optional
        Frame 2 (seq=2): [joint_id, flags, kp_inner×100, ki_inner×100, kd_inner×100] — optional
        Frame 3 (seq=3): [joint_id, flags, tau_ff, 0, 0]                             — optional

        The last frame has has_more=0, which triggers application on firmware.
        Parameters not sent retain their previous values on the firmware side.

        Semantics:
        - q_target = joint goal
        - dq_target = cruise speed magnitude for the controller-side rolling segment
        - the RP2350 interpolates q_ref/dq_ref locally at control-loop rate

        Fast path (50-200 Hz host updates): call with only q/dq/stiffness → 1 frame, ~0.1 ms.
        Full update: call with all params → 4 frames, ~9 ms.

        Args:
            joint_name: Joint name (e.g. "KNEE_RIGHT")
            dof_index: DOF index (0-2)
            q_target: Target joint goal (degrees)
            dq_target: Cruise speed magnitude for the local rolling segment (deg/s)
            stiffness: Co-contraction stiffness (degrees)
            kp: Outer position gain (None = keep firmware's current value)
            ki: Outer integral gain (None = keep current)
            kd: Outer velocity gain (None = keep firmware's current value)
            tau_ff: Feedforward torque, raw motor units (None = keep current)
            kp_inner: Inner PID Kp (None = keep current)
            ki_inner: Inner PID Ki (None = keep current)
            kd_inner: Inner PID Kd (None = keep current)
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        dq_target = abs(dq_target)

        # Serialize entire impedance transaction (cache + frames + ctrl).
        # Prevents: cache contamination between concurrent requests,
        # and a disable slipping between seq=0 and later optional frames.
        with self._impedance_tx_lock:
            # Read cache snapshot — resolve None gains from last-sent values.
            # Cache is NOT mutated until all frames are sent successfully.
            cache_key = (joint_name, dof_index)
            cached = self._impedance_gain_cache.get(cache_key, {})

            # Determine which frames are needed
            need_frame1 = (kp is not None or ki is not None or kd is not None)
            need_frame2 = (kp_inner is not None or ki_inner is not None or kd_inner is not None)
            need_frame3 = (tau_ff is not None)

            # Resolve effective gain values for frame packing (cached or default)
            defaults = self._IMPEDANCE_DEFAULTS
            kp_val = kp if kp is not None else cached.get("kp", defaults["kp"])
            ki_val = ki if ki is not None else cached.get("ki", defaults["ki"])
            kd_val = kd if kd is not None else cached.get("kd", defaults["kd"])
            tau_ff_val = tau_ff if tau_ff is not None else cached.get("tau_ff", defaults["tau_ff"])
            kpi_val = kp_inner if kp_inner is not None else cached.get("kp_inner", defaults["kp_inner"])
            kii_val = ki_inner if ki_inner is not None else cached.get("ki_inner", defaults["ki_inner"])
            kdi_val = kd_inner if kd_inner is not None else cached.get("kd_inner", defaults["kd_inner"])

            # --- Frame 0: q (×100), dq (×10), stiffness (×10) — always sent ---
            q_int = int(round(q_target * 100))      # 0.01° resolution, ±327°
            dq_int = int(round(dq_target * 10))      # 0.1°/s resolution, ±3276°/s
            stiff_int = int(round(stiffness * 10))    # 0.1° resolution, ±3276°
            has_more_0 = need_frame1 or need_frame2 or need_frame3
            flags0 = (int(has_more_0) << 7) | (0 << 4) | (dof_index & 0x0F)
            payload0 = struct.pack("<BBhhh", joint_id, flags0,
                                    q_int, dq_int, stiff_int)
            payload0 = payload0.ljust(8, b'\x00')

            self._send_frame(0x01D, payload0, context="SET_IMPEDANCE seq=0")

            if need_frame1:
                time.sleep(self._MULTI_FRAME_DELAY_S)
                # --- Frame 1: Kp, Ki, Kd ---
                kp_int = int(round(kp_val * 100))
                ki_int = int(round(ki_val * 100))
                kd_int = int(round(kd_val * 100))
                has_more_1 = need_frame2 or need_frame3
                flags1 = (int(has_more_1) << 7) | (1 << 4) | (dof_index & 0x0F)
                payload1 = struct.pack("<BBhhh", joint_id, flags1,
                                        kp_int, ki_int, kd_int)
                payload1 = payload1.ljust(8, b'\x00')
                self._send_frame(0x01D, payload1, context="SET_IMPEDANCE seq=1")

            if need_frame2:
                time.sleep(self._MULTI_FRAME_DELAY_S)
                # --- Frame 2: kp_inner, ki_inner, kd_inner ---
                kpi_int = int(round(kpi_val * 100))
                kii_int = int(round(kii_val * 100))
                kdi_int = int(round(kdi_val * 100))
                flags2 = (int(need_frame3) << 7) | (2 << 4) | (dof_index & 0x0F)
                payload2 = struct.pack("<BBhhh", joint_id, flags2,
                                        kpi_int, kii_int, kdi_int)
                payload2 = payload2.ljust(8, b'\x00')
                self._send_frame(0x01D, payload2, context="SET_IMPEDANCE seq=2")

            if need_frame3:
                time.sleep(self._MULTI_FRAME_DELAY_S)
                tau_ff_int = int(round(tau_ff_val))
                flags3 = (0 << 7) | (3 << 4) | (dof_index & 0x0F)
                payload3 = struct.pack("<BBh", joint_id, flags3, tau_ff_int)
                payload3 = payload3.ljust(8, b'\x00')
                self._send_frame(0x01D, payload3, context="SET_IMPEDANCE seq=3")

            # Commit to cache ONLY after all frames sent successfully.
            # If _send_frame raised an exception above, cache stays unchanged.
            if kp is not None:
                cached["kp"] = kp
            if ki is not None:
                cached["ki"] = ki
            if kd is not None:
                cached["kd"] = kd
            if tau_ff is not None:
                cached["tau_ff"] = tau_ff
            if kp_inner is not None:
                cached["kp_inner"] = kp_inner
            if ki_inner is not None:
                cached["ki_inner"] = ki_inner
            if kd_inner is not None:
                cached["kd_inner"] = kd_inner
            self._impedance_gain_cache[cache_key] = cached

        return {"success": True}

    def send_impedance_ctrl(self, joint_name: str, sub_cmd: int,
                             param: int = 0) -> Dict[str, Any]:
        """Send IMPEDANCE_CTRL via CAN (0x01E).

        Args:
            joint_name: Joint name
            sub_cmd: 0x00=disable (all DOFs → HOLDING),
                     0x01=enable (informational),
                     0x02=set_watchdog_ms
            param: Parameter for sub_cmd (e.g. watchdog timeout in ms)
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        # Share impedance TX lock — prevents disable from slipping between
        # seq=0 and later optional frames of a concurrent SET_IMPEDANCE transaction.
        with self._impedance_tx_lock:
            payload = struct.pack("<BBH", joint_id, sub_cmd, param)
            payload = payload.ljust(8, b'\x00')
            self._send_frame(0x01E, payload,
                             context=f"IMPEDANCE_CTRL cmd=0x{sub_cmd:02X} param={param}")
        return {"success": True}

    def start_auto_mapping_via_can(self, joint_name: str,
                                    dof_index: int = 0) -> Dict[str, Any]:
        """Send start-auto-mapping command via CAN (0x016).

        Uses firmware config defaults for torque, step size, settle time.
        Note: Firmware always maps ALL DOFs simultaneously (Cartesian product)
        regardless of dof_index. Byte 1 is reserved for potential future use.
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, dof_index, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x016, payload,
                         context=f"Start auto-mapping {joint_name} DOF {dof_index}")
        return {"success": True}

    def stop_auto_mapping_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send stop-auto-mapping command via CAN (0x017)."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x017, payload,
                         context=f"Stop auto-mapping {joint_name}")
        return {"success": True}

    def save_linear_eq_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send save-linear-equations-to-flash command via CAN (0x018)."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x018, payload,
                         context=f"Save linear eq {joint_name}")
        return {"success": True}

    def load_linear_eq_via_can(self, joint_name: str) -> Dict[str, Any]:
        """Send load-linear-equations-from-flash command via CAN (0x019)."""
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = bytes([joint_id, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(0x019, payload,
                         context=f"Load linear eq {joint_name}")
        return {"success": True}

    def set_auto_start_via_can(self, joint_name: str, enabled: int,
                                torque: int = 0,
                                duration: int = 0) -> Dict[str, Any]:
        """Send set-auto-start command via CAN (0x01A).

        Args:
            joint_name: Joint name
            enabled: 1=enable, 0=disable
            torque: Pretension torque override (0=use default)
            duration: Startup duration override ms (0=use default)
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]
        payload = struct.pack("<BBhH", joint_id, enabled,
                              int(torque), int(duration)) + bytes(2)
        self._send_frame(0x01A, payload,
                         context=f"Set auto-start {joint_name} en={enabled}")
        return {"success": True}

    def _save_encoder_offsets(self, joint_name: str, offsets: Dict[int, float]) -> None:
        """Save encoder offsets to calibration_data/{joint}_encoder_offsets.json"""
        import os
        import json as _json
        from datetime import datetime

        try:
            os.makedirs("calibration_data", exist_ok=True)
            filename = f"calibration_data/{joint_name.lower()}_encoder_offsets.json"
            data = {
                "joint_name": joint_name,
                "timestamp": datetime.now().isoformat(),
                "offsets": {str(k): v for k, v in offsets.items()},
            }
            with open(filename, "w") as f:
                _json.dump(data, f, indent=2)
            self._log_can_info(f"Encoder offsets saved for {joint_name}")
        except Exception as e:
            self.logger.error(f"Error saving encoder offsets for {joint_name}: {e}")

    def start_pid_diag_stream(self, terms_enabled: bool = False) -> Dict[str, Any]:
        """
        Start PID diagnostics streaming via CAN at 20Hz.

        Sends control command (0x004) to enable PID diagnostic data.
        Data arrives on 0x420 (target/error) and 0x430 (torque).
        When terms_enabled=True, also sends 0x470 (inner P/I/D) and 0x480 (outer P/I/D).

        Args:
            terms_enabled: Enable per-term P/I/D breakdown streaming (0x470/0x480)
        """
        self._ensure_connection()
        terms_byte = 0x01 if terms_enabled else 0x00
        payload = bytes([0x01, terms_byte]) + bytes(6)  # byte0=start, byte1=terms
        self._send_frame(0x004, payload, context="PID diag stream START")
        terms_str = " + P/I/D terms" if terms_enabled else ""
        self._log_can_info(f"PID diagnostics streaming started @ 20Hz{terms_str}")
        return {"streaming": True, "terms_enabled": terms_enabled}

    def stop_pid_diag_stream(self) -> Dict[str, Any]:
        """
        Stop PID diagnostics streaming via CAN.
        """
        self._ensure_connection()
        payload = bytes([0x00]) + bytes(7)  # 0x00 = stop streaming
        self._send_frame(0x004, payload, context="PID diag stream STOP")
        self._log_can_info("PID diagnostics streaming stopped")
        return {"streaming": False}

    def set_loop_frequencies(self, inner_period_us: int, outer_divisor: int) -> Dict[str, Any]:
        """
        Set control loop frequencies.
        
        Args:
            inner_period_us: Inner loop period in microseconds (500-10000µs)
                            Default 2000µs = 500Hz
            outer_divisor: Outer loop runs every N inner cycles (1-20)
                          Default 5 = 100Hz when inner is 500Hz
        
        Sends control command (0x006) to set loop frequencies.
        """
        self._ensure_connection()
        
        # Validate ranges
        inner_period_us = max(500, min(10000, inner_period_us))
        outer_divisor = max(1, min(20, outer_divisor))
        
        # Pack: inner_period_us (2 bytes little-endian) + outer_divisor (1 byte)
        payload = struct.pack('<HB', inner_period_us, outer_divisor) + bytes(5)
        
        inner_freq = 1000000.0 / inner_period_us
        outer_freq = inner_freq / outer_divisor
        
        self._send_frame(0x006, payload, context=f"Loop freq: inner={inner_freq:.1f}Hz, outer={outer_freq:.1f}Hz")
        self._log_can_info(f"Loop frequencies set: inner={inner_freq:.1f}Hz ({inner_period_us}µs), outer={outer_freq:.1f}Hz (÷{outer_divisor})")
        
        return {
            "inner_period_us": inner_period_us,
            "outer_divisor": outer_divisor,
            "inner_freq_hz": inner_freq,
            "outer_freq_hz": outer_freq
        }

    def set_pid_diag_frequency(self, freq_hz: int) -> Dict[str, Any]:
        """
        Set PID diagnostics streaming frequency.
        
        Args:
            freq_hz: Frequency in Hz (10-200)
                    Default 20Hz for monitoring, 50-100Hz for NN training data
        
        Sends control command (0x007) to set PID diagnostics frequency.
        """
        self._ensure_connection()
        
        # Validate range
        freq_hz = max(10, min(200, freq_hz))
        
        payload = bytes([freq_hz]) + bytes(7)
        
        interval_ms = 1000 / freq_hz
        
        self._send_frame(0x007, payload, context=f"PID diag freq: {freq_hz}Hz")
        self._log_can_info(f"PID diagnostics frequency set to: {freq_hz}Hz ({interval_ms:.1f}ms)")
        
        return {
            "freq_hz": freq_hz,
            "interval_ms": interval_ms
        }

    def is_encoder_streaming(self) -> bool:
        """Check if encoder streaming is currently active."""
        return self._encoder_stream_active

    def get_encoder_stream_data(self) -> list:
        """Get buffered encoder stream data (oldest first)."""
        with self._lock:
            data = list(self._encoder_stream_data)
            self._encoder_stream_data.clear()
        return data

    def set_encoder_stream_callback(self, callback) -> None:
        """
        Set a callback for real-time encoder data.
        
        Callback signature: callback(angles_deg: list, timestamp_ms: int)
        """
        self._encoder_stream_callback = callback

    def get_last_encoder_angles(self, joint_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Get the last known encoder angles (persists even after streaming stops).
        
        This is useful for on-demand position queries without requiring active streaming.
        
        Args:
            joint_name: Optional joint name filter. If None, returns all joints.
        
        Returns:
            If joint_name specified: {angles_deg: [...], timestamp_ms: int, age_ms: float}
            If joint_name is None: {joint_name: {angles_deg: [...], ...}, ...}
        """
        current_time = time.time()
        
        with self._lock:
            if joint_name:
                # Find joint ID from name
                joint_id = None
                for jid, jname in self._joint_id_lookup.items():
                    if jname == joint_name:
                        joint_id = jid
                        break
                
                if joint_id is None or joint_id not in self._last_encoder_angles:
                    return {"valid": False, "message": f"No encoder data for {joint_name}"}
                
                data = self._last_encoder_angles[joint_id].copy()
                data["age_ms"] = (current_time - data["timestamp"]) * 1000
                data["valid"] = True
                return data
            else:
                # Return all joints
                result = {}
                for joint_id, data in self._last_encoder_angles.items():
                    joint_name = data.get("joint_name", f"JOINT_{joint_id}")
                    result[joint_name] = data.copy()
                    result[joint_name]["age_ms"] = (current_time - data["timestamp"]) * 1000
                    result[joint_name]["valid"] = True
                return result

    # ------------------------------------------------------------------
    # Telemetry accessors
    # ------------------------------------------------------------------
    def get_connection_state(self) -> Dict[str, Any]:
        """Return connection metadata plus last telemetry snapshot."""
        with self._lock:
            config_copy = dict(self._current_config) if self._current_config else None
            status_copy = {k: dict(v) for k, v in self._last_status_by_joint.items()}
            stats_copy = dict(self._stats)
            last_rx = self._last_rx_timestamp
            health_copy = {k: dict(v) for k, v in self._health_status.items()}
            fault_copy = {k: dict(v) for k, v in self._fault_status.items()}
            snapshot_meta_copy = {k: dict(v) for k, v in self._fault_snapshot_meta.items()}
            snapshot_dump_copy = {k: dict(v) for k, v in self._fault_snapshots.items()}
            events_copy = [dict(event) for event in self._diagnostic_events]

        return {
            "connected": self.is_connected(),
            "config": config_copy,
            "last_status": status_copy,
            "last_rx_timestamp": last_rx,
            "stats": stats_copy,
            "status_messages": list(self._status_messages),
            "diagnostics": {
                "health_status": health_copy,
                "fault_status": fault_copy,
                "fault_snapshot_meta": snapshot_meta_copy,
                "fault_snapshots": snapshot_dump_copy,
                "events": events_copy,
            },
        }

    # ------------------------------------------------------------------
    # Single motor direct CAN bench helpers
    # ------------------------------------------------------------------
    def motor_test_log_dir(self) -> Path:
        return self.MOTOR_TEST_LOG_DIR

    def list_motor_test_logs(self, limit: int = 20) -> list[Dict[str, Any]]:
        log_dir = self.motor_test_log_dir()
        if not log_dir.exists():
            return []

        files = sorted(
            (path for path in log_dir.glob("*.csv") if path.is_file()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        results = []
        for path in files[: max(0, limit)]:
            stat = path.stat()
            results.append({
                "name": path.name,
                "size_bytes": stat.st_size,
                "modified_ts": stat.st_mtime,
            })
        return results

    def get_motor_test_status(
        self,
        *,
        motor_id: int = DEFAULT_MOTOR_ID,
        output_ratio: float = DEFAULT_OUTPUT_RATIO,
    ) -> Dict[str, Any]:
        state = self.get_connection_state()
        recent: Dict[str, Any] = {}
        with self._motor_response_condition:
            recent_raw = self._motor_recent_frames.get((motor_id, READ_COMMANDS["raw"]))
            recent_state2 = self._motor_recent_frames.get((motor_id, READ_COMMANDS["state2"]))
            recent_single = self._motor_recent_frames.get((motor_id, READ_COMMANDS["single"]))
            recent_multi = self._motor_recent_frames.get((motor_id, READ_COMMANDS["multi"]))
            recent_torque = self._motor_recent_frames.get((motor_id, CONTROL_COMMANDS["torque"]))
            recent_speed = self._motor_recent_frames.get((motor_id, CONTROL_COMMANDS["speed"]))
            recent_on = self._motor_recent_frames.get((motor_id, CONTROL_COMMANDS["on"]))
            recent_stop = self._motor_recent_frames.get((motor_id, CONTROL_COMMANDS["stop"]))
            recent_off = self._motor_recent_frames.get((motor_id, CONTROL_COMMANDS["off"]))

        recent.update({
            "raw": self._decode_motor_response(recent_raw["data"], output_ratio=output_ratio) if recent_raw else None,
            "state2": self._decode_motor_response(recent_state2["data"], output_ratio=output_ratio) if recent_state2 else None,
            "single": self._decode_motor_response(recent_single["data"], output_ratio=output_ratio) if recent_single else None,
            "multi": self._decode_motor_response(recent_multi["data"], output_ratio=output_ratio) if recent_multi else None,
            "torque": self._decode_motor_response(recent_torque["data"], output_ratio=output_ratio) if recent_torque else None,
            "speed": self._decode_motor_response(recent_speed["data"], output_ratio=output_ratio) if recent_speed else None,
            "echo": {
                "on": self._serialize_motor_frame(recent_on) if recent_on else None,
                "stop": self._serialize_motor_frame(recent_stop) if recent_stop else None,
                "off": self._serialize_motor_frame(recent_off) if recent_off else None,
            },
        })
        return {
            **state,
            "motor_id": motor_id,
            "output_ratio": output_ratio,
            "motor_recent": recent,
            "log_dir": str(self.motor_test_log_dir()),
            "recent_logs": self.list_motor_test_logs(limit=10),
        }

    def get_motor_tuning_status(
        self,
        *,
        motor_id: int = DEFAULT_MOTOR_ID,
        output_ratio: float = DEFAULT_OUTPUT_RATIO,
    ) -> Dict[str, Any]:
        state = self.get_connection_state()
        with self._motor_response_condition:
            recent_pid = self._motor_recent_frames.get((motor_id, READ_COMMANDS["pid"]))
            recent_accel = self._motor_recent_frames.get((motor_id, READ_COMMANDS["acceleration"]))
            recent_raw = self._motor_recent_frames.get((motor_id, READ_COMMANDS["raw"]))
            recent_state2 = self._motor_recent_frames.get((motor_id, READ_COMMANDS["state2"]))
            recent_single = self._motor_recent_frames.get((motor_id, READ_COMMANDS["single"]))
            recent_multi = self._motor_recent_frames.get((motor_id, READ_COMMANDS["multi"]))
        return {
            **state,
            "motor_id": motor_id,
            "output_ratio": output_ratio,
            "recent": {
                "pid": self._decode_motor_response(recent_pid["data"], output_ratio=output_ratio) if recent_pid else None,
                "acceleration": self._decode_motor_response(recent_accel["data"], output_ratio=output_ratio) if recent_accel else None,
                "raw": self._decode_motor_response(recent_raw["data"], output_ratio=output_ratio) if recent_raw else None,
                "state2": self._decode_motor_response(recent_state2["data"], output_ratio=output_ratio) if recent_state2 else None,
                "single": self._decode_motor_response(recent_single["data"], output_ratio=output_ratio) if recent_single else None,
                "multi": self._decode_motor_response(recent_multi["data"], output_ratio=output_ratio) if recent_multi else None,
            },
        }

    def motor_tuning_read(
        self,
        action: str,
        *,
        motor_id: int = DEFAULT_MOTOR_ID,
        timeout_s: float = 0.2,
        output_ratio: float = DEFAULT_OUTPUT_RATIO,
    ) -> Dict[str, Any]:
        allowed = {"pid", "acceleration", "raw", "state2", "single", "multi"}
        if action not in allowed:
            raise ValueError(f"Unsupported tuning read action: {action}")
        return self.motor_test_command(
            action,
            motor_id=motor_id,
            value=None,
            timeout_s=timeout_s,
            output_ratio=output_ratio,
        )

    def motor_tuning_probe(
        self,
        action: str,
        *,
        motor_id: int = DEFAULT_MOTOR_ID,
        timeout_s: float = 0.3,
        output_ratio: float = DEFAULT_OUTPUT_RATIO,
    ) -> Dict[str, Any]:
        allowed = {"pid", "acceleration"}
        if action not in allowed:
            raise ValueError(f"Unsupported tuning probe action: {action}")
        self._ensure_connection()
        arbitration_id = self.MOTOR_ID_BASE + int(motor_id)
        payload = build_read_frame(action)
        expected_cmd = READ_COMMANDS[action]

        with self._motor_test_lock:
            with self._motor_response_condition:
                min_seq = self._motor_frame_seq
            self._send_frame(
                arbitration_id,
                payload,
                context=f"MotorTuning probe {action} motor={motor_id}",
            )
            deadline = time.monotonic() + timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                with self._motor_response_condition:
                    self._motor_response_condition.wait(timeout=min(remaining, 0.02))

            with self._motor_response_condition:
                frames = [
                    dict(frame)
                    for frame in self._motor_frame_history
                    if frame["motor_id"] == motor_id and frame.get("seq", -1) > min_seq
                ]

        matched = next((frame for frame in frames if frame["command"] == expected_cmd), None)
        return {
            "action": action,
            "motor_id": motor_id,
            "arbitration_id": f"0x{arbitration_id:03X}",
            "tx_hex": bytes_hex(payload),
            "probe_timeout_s": timeout_s,
            "frames_seen": len(frames),
            "matched_expected": matched is not None,
            "frames": [self._serialize_motor_frame(frame) for frame in frames],
            "decoded_expected": (
                self._decode_motor_response(matched["data"], output_ratio=output_ratio) if matched else None
            ),
        }

    def motor_test_command(
        self,
        action: str,
        *,
        motor_id: int = DEFAULT_MOTOR_ID,
        value: Optional[float] = None,
        timeout_s: float = 0.2,
        output_ratio: float = DEFAULT_OUTPUT_RATIO,
    ) -> Dict[str, Any]:
        self._ensure_connection()
        arbitration_id = self.MOTOR_ID_BASE + int(motor_id)
        if action in ("stop", "off"):
            self._motor_test_abort_requested.set()
            acquired = self._motor_test_lock.acquire(timeout=0.1)
            if not acquired:
                blind_stop_steps = self._motor_force_stop_blind(
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    timeout_s=timeout_s,
                    power_off=(action == "off"),
                )
                return {
                    "action": action,
                    "motor_id": motor_id,
                    "arbitration_id": f"0x{arbitration_id:03X}",
                    "warning": "Emergency blind stop sent while sweep lock was busy",
                    "blind_stop_steps": blind_stop_steps,
                    "abort_requested": True,
                }
        else:
            acquired = False

        if not acquired:
            self._motor_test_lock.acquire()

        try:
            if action in READ_COMMANDS:
                payload = build_read_frame(action)
                response = self._motor_exchange_locked(
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    payload=payload,
                    expected_cmd=READ_COMMANDS[action],
                    timeout_s=timeout_s,
                    context=f"MotorTest read {action} motor={motor_id}",
                )
                return self._motor_exchange_result(
                    action=action,
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    payload=payload,
                    response=response,
                    output_ratio=output_ratio,
                )

            if action in ("torque", "speed"):
                if value is None:
                    raise ValueError(f"value is required for action '{action}'")
                payload = build_control_frame(action, float(value))
                response = self._motor_exchange_locked(
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    payload=payload,
                    expected_cmd=CONTROL_COMMANDS[action],
                    timeout_s=timeout_s,
                    context=f"MotorTest {action}={value} motor={motor_id}",
                )
                return self._motor_exchange_result(
                    action=action,
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    payload=payload,
                    response=response,
                    output_ratio=output_ratio,
                    target=value,
                )

            if action in ("on", "off", "stop"):
                if action == "on":
                    payload = build_zero_frame(CONTROL_COMMANDS[action])
                    warning = None
                    try:
                        response = self._motor_exchange_locked(
                            motor_id=motor_id,
                            arbitration_id=arbitration_id,
                            payload=payload,
                            expected_cmd=CONTROL_COMMANDS[action],
                            timeout_s=timeout_s,
                            context=f"MotorTest {action} motor={motor_id}",
                        )
                    except TimeoutError:
                        response = None
                        warning = f"{action} echo timed out"
                    result = {
                        "action": action,
                        "motor_id": motor_id,
                        "arbitration_id": f"0x{arbitration_id:03X}",
                        "tx_hex": bytes_hex(payload),
                        "response": self._serialize_motor_frame(response) if response else None,
                        "warning": warning,
                    }
                    return result

                safe_stop = self._motor_safe_stop_locked(
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    timeout_s=timeout_s,
                    power_off=(action in ("off", "stop")),
                )
                return {
                    "action": action,
                    "motor_id": motor_id,
                    "arbitration_id": f"0x{arbitration_id:03X}",
                    **safe_stop,
                    "abort_requested": True,
                }
        finally:
            self._motor_test_lock.release()
            if action in ("stop", "off"):
                self._motor_test_abort_requested.clear()

        raise ValueError(f"Unsupported motor action: {action}")

    def motor_test_run_sweep(
        self,
        *,
        motor_id: int = DEFAULT_MOTOR_ID,
        mode: str,
        profile: str,
        duration_s: float,
        rate_hz: float,
        timeout_s: float,
        output_ratio: float = DEFAULT_OUTPUT_RATIO,
        bias: float = 0.0,
        amplitude: float = 0.0,
        preload_s: float = 0.0,
        frequency_hz: float = 1.0,
        f0_hz: float = 0.2,
        f1_hz: float = 8.0,
        extra_commands: Optional[list[str]] = None,
        label: Optional[str] = None,
        stop_at_end: bool = True,
        motor_on_before: bool = True,
        power_off_at_end: bool = False,
    ) -> Dict[str, Any]:
        self._ensure_connection()
        if duration_s <= 0:
            raise ValueError("duration_s must be > 0")
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        extras = parse_extra_commands(extra_commands)
        fieldnames = csv_fieldnames(extras)
        arbitration_id = self.MOTOR_ID_BASE + int(motor_id)
        control_cmd = CONTROL_COMMANDS[mode]
        rows: list[Dict[str, Any]] = []
        total_samples = max(1, int(round(duration_s * rate_hz)))
        start = time.perf_counter()
        resolved_label = label or build_default_label(
            motor_id=motor_id,
            mode=mode,
            profile=profile,
            bias=bias,
            amplitude=amplitude,
            duration_s=duration_s,
            rate_hz=rate_hz,
            preload_s=preload_s,
            frequency_hz=frequency_hz,
            f0_hz=f0_hz,
            f1_hz=f1_hz,
        )

        with self._motor_test_lock:
            self._motor_test_abort_requested.clear()
            motor_on_warning = None
            neutral_warning = None
            stop_warning = None
            aborted_reason = None
            safe_stop = None
            cleanup_error = None
            if motor_on_before:
                on_payload = build_zero_frame(CONTROL_COMMANDS["on"])
                on_result = self._motor_send_best_effort_locked(
                    motor_id=motor_id,
                    arbitration_id=arbitration_id,
                    payload=on_payload,
                    expected_cmd=CONTROL_COMMANDS["on"],
                    timeout_s=timeout_s,
                    context=f"MotorSweep on motor={motor_id}",
                )
                if on_result["timed_out"]:
                    motor_on_warning = "motor on echo timed out"
                time.sleep(min(0.05, timeout_s))

            try:
                for index in range(total_samples):
                    if self._motor_test_abort_requested.is_set():
                        aborted_reason = "abort requested by manual stop/off"
                        break
                    loop_start = time.perf_counter()
                    t_s = loop_start - start
                    target = generate_target(
                        profile=profile,
                        t_s=t_s,
                        duration_s=duration_s,
                        bias=bias,
                        amplitude=amplitude,
                        preload_s=preload_s,
                        frequency_hz=frequency_hz,
                        f0_hz=f0_hz,
                        f1_hz=f1_hz,
                    )
                    payload = build_control_frame(mode, target)
                    try:
                        response = self._motor_exchange_locked(
                            motor_id=motor_id,
                            arbitration_id=arbitration_id,
                            payload=payload,
                            expected_cmd=control_cmd,
                            timeout_s=timeout_s,
                            context=f"MotorSweep {mode}={target:.3f}",
                        )
                    except TimeoutError:
                        try:
                            response = self._motor_exchange_locked(
                                motor_id=motor_id,
                                arbitration_id=arbitration_id,
                                payload=payload,
                                expected_cmd=control_cmd,
                                timeout_s=timeout_s,
                                context=f"MotorSweep retry {mode}={target:.3f}",
                            )
                        except TimeoutError as exc:
                            aborted_reason = str(exc)
                            break

                    decoded = self._decode_motor_response(response["data"], output_ratio=output_ratio)
                    row: Dict[str, Any] = {
                        "motor_id": motor_id,
                        "output_ratio": output_ratio,
                        "duration_s": duration_s,
                        "rate_hz": rate_hz,
                        "timeout_s": timeout_s,
                        "bias": bias,
                        "amplitude": amplitude,
                        "preload_s": preload_s,
                        "frequency_hz": frequency_hz,
                        "f0_hz": f0_hz,
                        "f1_hz": f1_hz,
                        "label": resolved_label,
                        "t_s": t_s,
                        "mode": mode,
                        "profile": profile,
                        "target": target,
                        **decoded,
                    }

                    extra_failed = False
                    for extra_name in extras:
                        extra_payload = build_read_frame(extra_name)
                        try:
                            extra_response = self._motor_exchange_locked(
                                motor_id=motor_id,
                                arbitration_id=arbitration_id,
                                payload=extra_payload,
                                expected_cmd=READ_COMMANDS[extra_name],
                                timeout_s=timeout_s,
                                context=f"MotorSweep extra {extra_name}",
                            )
                        except TimeoutError as exc:
                            aborted_reason = str(exc)
                            extra_failed = True
                            break
                        row.update(self._decode_motor_response(extra_response["data"], output_ratio=output_ratio))

                    rows.append(row)
                    if extra_failed:
                        break

                    next_deadline = start + ((index + 1) / rate_hz)
                    remaining = next_deadline - time.perf_counter()
                    if remaining > 0:
                        time.sleep(remaining)
            finally:
                if stop_at_end:
                    try:
                        neutral_payload = build_control_frame(mode, 0.0)
                        neutral_result = self._motor_send_best_effort_locked(
                            motor_id=motor_id,
                            arbitration_id=arbitration_id,
                            payload=neutral_payload,
                            expected_cmd=control_cmd,
                            timeout_s=timeout_s,
                            context=f"MotorSweep neutral {mode}=0 motor={motor_id}",
                        )
                        if neutral_result["timed_out"]:
                            neutral_warning = f"{mode}=0 echo timed out"

                        time.sleep(min(0.05, timeout_s))

                        safe_stop = self._motor_safe_stop_locked(
                            motor_id=motor_id,
                            arbitration_id=arbitration_id,
                            timeout_s=timeout_s,
                            power_off=power_off_at_end,
                        )
                        stop_warning = safe_stop["warning"]
                    except Exception as exc:
                        cleanup_error = str(exc)
                        safe_stop = {
                            "safe_stop_steps": [],
                            "blind_stop_steps": self._motor_force_stop_blind(
                                motor_id=motor_id,
                                arbitration_id=arbitration_id,
                                timeout_s=timeout_s,
                                power_off=power_off_at_end,
                            ),
                            "warning": "Cleanup fallback used after exception",
                        }
                        if not stop_warning:
                            stop_warning = safe_stop["warning"]
                self._motor_test_abort_requested.clear()

        csv_path = default_csv_path(
            self.motor_test_log_dir(),
            mode=mode,
            profile=profile,
            label=resolved_label,
        )
        write_csv_rows(csv_path, rows, fieldnames)
        return {
            "status": "error" if aborted_reason else "success",
            "message": aborted_reason or cleanup_error or stop_warning or neutral_warning,
            "motor_id": motor_id,
            "mode": mode,
            "profile": profile,
            "samples": len(rows),
            "rate_hz": rate_hz,
            "duration_s": duration_s,
            "csv_path": str(csv_path),
            "csv_name": csv_path.name,
            "label": resolved_label,
            "motor_on_warning": motor_on_warning,
            "neutral_warning": neutral_warning,
            "stop_warning": stop_warning,
            "safe_stop": safe_stop,
            "aborted_reason": aborted_reason,
            "cleanup_error": cleanup_error,
        }

    def clear_status_messages(self) -> None:
        """Utility for tests to clear buffered status messages."""
        with self._lock:
            self._status_messages.clear()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _ensure_connection(self) -> None:
        if self._bus is None:
            raise RuntimeError("CAN bus is not connected. Select and connect an interface first.")

    def _send_frame(self, arbitration_id: int, data: bytes, context: Optional[str] = None) -> None:
        if self._bus is None:
            raise RuntimeError("Cannot send frame: CAN bus not initialized.")
        
        # For SLCAN, write directly to the bus's underlying serial port
        # This avoids opening a new connection that interferes with RX
        if self._current_config.get("interface") == "serial":
            try:
                # Access python-can's internal serial connection
                serial_port = getattr(self._bus, 'serialPortOrig', None) or getattr(self._bus, '_ser', None)
                if serial_port is None:
                    # Fallback: try the standard send
                    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
                    self._bus.send(message)
                else:
                    # Format: tiiildd...dd\r where iii=ID (3 hex), l=length (1 hex), dd=data (2 hex each)
                    can_id_hex = f"{arbitration_id:03X}"
                    length_hex = f"{len(data):01X}"
                    data_hex = data.hex().upper()
                    slcan_cmd = f"t{can_id_hex}{length_hex}{data_hex}\r"
                    serial_port.write(slcan_cmd.encode('ascii'))
                
                self._stats["tx_frames"] += 1
                self._log_can_sent(arbitration_id, data, context)
                return
            except Exception as exc:
                err_msg = f"TX error id=0x{arbitration_id:03X}: {exc}"
                self.logger.warning(f"[CAN] {err_msg}")
                self._stats["errors"] += 1
                self._stats["last_error"] = str(exc)
                raise RuntimeError(err_msg) from exc
        
        # For other interfaces, use python-can directly
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self._bus.send(message)
            self._stats["tx_frames"] += 1
            self._log_can_sent(arbitration_id, data, context)
        except can.CanError as exc:
            self._stats["errors"] += 1
            self._stats["last_error"] = str(exc)
            self.logger.exception("Failed to send CAN frame: id=0x%03X", arbitration_id)
            self._log_can_error(f"TX error id=0x{arbitration_id:03X}: {exc}")
            raise

    def _listen_loop(self) -> None:
        """Continuously read CAN messages and decode known frames."""
        self._log_can_info("Listener thread started")
        
        # For SLCAN, use custom parsing since python-can has issues on macOS
        if self._current_config and self._current_config.get("interface") == "serial":
            self._slcan_listen_loop()
            return
        
        # Standard python-can listener for other interfaces
        loop_count = 0
        while not self._listener_stop.is_set():
            bus = self._bus
            if bus is None:
                time.sleep(0.1)
                continue
            
            loop_count += 1
            
            try:
                message = bus.recv(timeout=0.2)
            except can.CanError as exc:
                self.logger.warning("CAN recv error: %s", exc)
                self._stats["errors"] += 1
                self._stats["last_error"] = str(exc)
                self._log_can_error(f"RX error: {exc}")
                time.sleep(0.5)
                continue

            if message is None:
                if loop_count % 25 == 0:
                    self._log_can_info(f"Listener active, waiting for CAN frames... (loops={loop_count})")
                continue

            self._log_can_info(f"RX frame id=0x{message.arbitration_id:03X} len={len(message.data)}")
            self._last_rx_timestamp = time.time()
            self._handle_message(message)

    def _slcan_listen_loop(self) -> None:
        """Custom SLCAN listener that reads directly from serial port."""
        import serial
        
        channel = self._current_config.get("channel")
        self._log_can_info(f"Starting SLCAN listener on {channel}")
        
        try:
            # Open separate serial connection for reading
            ser = serial.Serial(channel, baudrate=115200, timeout=0.1)
            buffer = ""
            frame_count = 0
            
            while not self._listener_stop.is_set():
                try:
                    # Read available data
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
                        buffer += data
                        
                        # Process complete frames (terminated by \r or \n)
                        while '\r' in buffer or '\n' in buffer:
                            # Find end of frame
                            end_idx = -1
                            for i, c in enumerate(buffer):
                                if c in '\r\n':
                                    end_idx = i
                                    break
                            
                            if end_idx == -1:
                                break
                            
                            frame = buffer[:end_idx].strip()
                            buffer = buffer[end_idx+1:]
                            
                            if frame and frame.startswith('t'):
                                # Standard CAN frame: tiiildd...dd
                                msg = self._decode_slcan_frame(frame)
                                if msg:
                                    frame_count += 1
                                    self._last_rx_timestamp = time.time()
                                    self._handle_message(msg)
                                    
                                    # Log every 50 frames
                                    if frame_count % 50 == 0:
                                        self._log_can_info(f"SLCAN RX: {frame_count} frames received")
                    else:
                        time.sleep(0.005)  # 5ms sleep when no data
                        
                except serial.SerialException as exc:
                    self._log_can_error(f"SLCAN read error: {exc}")
                    time.sleep(0.5)
                    
        except Exception as exc:
            self._log_can_error(f"SLCAN listener failed: {exc}")
        finally:
            try:
                ser.close()
            except:
                pass
            self._log_can_info("SLCAN listener stopped")

    def _decode_slcan_frame(self, frame: str):
        """Decode SLCAN ASCII frame to can.Message."""
        try:
            # Format: tiiildd...dd where iii=ID (3 hex), l=length (1 hex), dd=data
            if len(frame) < 5:
                return None
            
            can_id = int(frame[1:4], 16)
            length = int(frame[4], 16)
            
            if len(frame) < 5 + length * 2:
                return None
            
            data_hex = frame[5:5 + length * 2]
            data = bytes.fromhex(data_hex)
            
            return can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False,
                timestamp=time.time()
            )
        except (ValueError, IndexError):
            return None

    def _handle_message(self, message: "can.Message") -> None:
        arb_id = message.arbitration_id
        data = bytes(message.data)
        self._expire_fault_snapshot_pending(message.timestamp)

        # Direct motor responses (0x140 + motor_id) for single-motor CAN bench.
        if self.MOTOR_ID_BASE < arb_id < 0x200 and len(data) >= 1:
            self._handle_motor_test_frame(arb_id, data, message.timestamp)
            return

        # Encoder stream data (0x410-0x41F) - high-frequency, minimal logging
        # Each joint uses 0x410 + joint_id to allow filtering when multiple controllers on bus
        if 0x410 <= arb_id <= 0x41F and len(data) >= 8:
            joint_id = arb_id - 0x410
            self._handle_encoder_stream(data, message.timestamp, joint_id)
            return  # Don't log every frame
        
        # PID diagnostics data (0x420-0x42F) - target and error
        if 0x420 <= arb_id <= 0x42F and len(data) >= 8:
            joint_id = arb_id - 0x420
            self._handle_pid_diag_data(data, message.timestamp, joint_id)
            return  # Don't log every frame
        
        # PID torque data (0x430-0x43F) - torque commands
        if 0x430 <= arb_id <= 0x43F and len(data) >= 8:
            joint_id = arb_id - 0x430
            self._handle_pid_torque_data(data, message.timestamp, joint_id)
            return  # Don't log every frame
        
        # Movement metrics data (0x440-0x44F) - timing/accuracy metrics
        if 0x440 <= arb_id <= 0x44F and len(data) >= 8:
            # Decode joint and DOF from ID: 0x440 + joint*3 + dof
            offset = arb_id - 0x440
            joint_id = offset // 3
            dof = offset % 3
            self._handle_movement_metrics_frame1(data, message.timestamp, joint_id, dof)
            return
        
        # Movement metrics data (0x450-0x45F) - torque/duration metrics
        if 0x450 <= arb_id <= 0x45F and len(data) >= 8:
            offset = arb_id - 0x450
            joint_id = offset // 3
            dof = offset % 3
            self._handle_movement_metrics_frame2(data, message.timestamp, joint_id, dof)
            return
        
        # Smoothness metrics data (0x460-0x46F) - oscillation/vibration metrics
        if 0x460 <= arb_id <= 0x46F and len(data) >= 8:
            offset = arb_id - 0x460
            joint_id = offset // 3
            dof = offset % 3
            self._handle_smoothness_metrics_frame(data, message.timestamp, joint_id, dof)
            return

        # Inner PID terms data (0x470-0x47F) - P/I/D/FF breakdown
        if 0x470 <= arb_id <= 0x47F and len(data) >= 8:
            joint_id = arb_id - 0x470
            self._handle_pid_inner_terms(data, message.timestamp, joint_id)
            return  # Don't log every frame

        # Outer PID terms data (0x480-0x48F) - P/I/D/output breakdown
        if 0x480 <= arb_id <= 0x48F and len(data) >= 8:
            joint_id = arb_id - 0x480
            self._handle_pid_outer_terms(data, message.timestamp, joint_id)
            return  # Don't log every frame

        # Startup status events (0x490-0x49F) - startup sequence progress
        if 0x490 <= arb_id <= 0x49F and len(data) >= 5:
            joint_id = arb_id - 0x490
            self._handle_startup_status(data, message.timestamp, joint_id)
            return

        # Joint announce / discovery (0x4A0-0x4AF) - joint identification via CAN
        if 0x4A0 <= arb_id <= 0x4AF and len(data) >= 8:
            joint_id = arb_id - 0x4A0
            self._handle_joint_announce(data, message.timestamp, joint_id)
            return

        # Encoder offset response (0x4B0-0x4BF) - cross-validation data
        if 0x4B0 <= arb_id <= 0x4BF and len(data) >= 5:
            enc_index = data[0]
            offset_rad = struct.unpack_from("<f", data, 4)[0]
            with self._lock:
                self._encoder_offset_buffer[enc_index] = offset_rad
                if len(self._encoder_offset_buffer) >= self._encoder_offset_expected:
                    self._encoder_offset_event.set()
            return

        # Zero-complete summary (0x4C0-0x4CF) - sent after set-zero via CAN
        if 0x4C0 <= arb_id <= 0x4CF and len(data) >= 4:
            joint_id = arb_id - 0x4C0
            enc_count = data[3]
            self._log_can_info(
                f"Zero complete: joint={joint_id} encoders={enc_count}"
            )
            self._zero_complete_event.set()
            return

        # Safe limits per DOF (0x4E0-0x4EF) - emitted on encoder stream start
        if 0x4E0 <= arb_id <= 0x4EF and len(data) >= 6:
            joint_id = arb_id - 0x4E0
            dof = data[0]
            min_i16 = struct.unpack_from("<h", data, 2)[0]
            max_i16 = struct.unpack_from("<h", data, 4)[0]
            safe_min = min_i16 / 100.0
            safe_max = max_i16 / 100.0
            joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")
            self._log_can_info(
                f"Safe limits [{joint_name}] DOF {dof}: [{safe_min:.1f}, {safe_max:.1f}]"
            )
            if self.socketio:
                self.socketio.emit(
                    "safe_limits",
                    {"joint_id": joint_id, "dof": dof, "min": safe_min, "max": safe_max},
                    namespace="/movement",
                )
            return

        # Holding diagnostics (0x4D0-0x4DF) — Phase 1 slack/bias data
        if 0x4D0 <= arb_id <= 0x4DF and len(data) >= 8:
            joint_id = arb_id - 0x4D0
            self._handle_diag_hold(data, message.timestamp, joint_id)
            return

        # Joint state broadcast (0x4F0-0x4FF) — impedance mode feedback
        if 0x4F0 <= arb_id <= 0x4FF and len(data) >= 8:
            joint_id = arb_id - 0x4F0
            self._handle_joint_state(data, message.timestamp, joint_id)
            return

        # Retension probe summary (0x500-0x50F) — host-side probe policy input
        if CAN_ID_RETENSION_PROBE_RESULT <= arb_id <= CAN_ID_RETENSION_PROBE_RESULT + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_RETENSION_PROBE_RESULT
            self._handle_retension_probe_result(data, message.timestamp, joint_id)
            return

        # Diagnostic health summary/counters (0x510-0x51F)
        if CAN_ID_HEALTH_STATUS <= arb_id <= CAN_ID_HEALTH_STATUS + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_HEALTH_STATUS
            self._handle_health_status(data, message.timestamp, joint_id)
            return

        # Diagnostic fault state (0x520-0x52F)
        if CAN_ID_FAULT_STATUS <= arb_id <= CAN_ID_FAULT_STATUS + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_FAULT_STATUS
            self._handle_fault_status(data, message.timestamp, joint_id)
            return

        # Diagnostic event timeline (0x530-0x53F)
        if CAN_ID_EVENT_NOTICE <= arb_id <= CAN_ID_EVENT_NOTICE + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_EVENT_NOTICE
            self._handle_event_notice(data, message.timestamp, joint_id)
            return

        # Fault snapshot metadata (0x540-0x54F)
        if CAN_ID_FAULT_SNAPSHOT_META <= arb_id <= CAN_ID_FAULT_SNAPSHOT_META + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_FAULT_SNAPSHOT_META
            self._handle_fault_snapshot_meta(data, message.timestamp, joint_id)
            return

        # Fault snapshot payload chunks (0x550-0x55F)
        if CAN_ID_FAULT_SNAPSHOT_DATA <= arb_id <= CAN_ID_FAULT_SNAPSHOT_DATA + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_FAULT_SNAPSHOT_DATA
            self._handle_fault_snapshot_chunk(data, message.timestamp, joint_id)
            return

        # Debug: log any received CAN frame (throttled)
        if arb_id >= 0x400:
            self._log_can_received(arb_id, data, context=f"Status frame 0x{arb_id:03X}")

        # NEW: Status messages now use 0x400 base (Priority Level 4)
        if 0x400 <= arb_id < 0x500 and len(data) >= 8:
            self._decode_status_frame(arb_id, data, message.timestamp)
        else:
            snippet = {
                "id": f"0x{arb_id:03X}",
                "data": data.hex(),
                "timestamp": message.timestamp,
            }
            with self._lock:
                self._status_messages.appendleft({"type": "unknown", "frame": snippet})
            self._log_can_received(arb_id, data, context="Unknown frame")

    def _handle_motor_test_frame(self, arbitration_id: int, data: bytes, timestamp: float) -> None:
        motor_id = arbitration_id - self.MOTOR_ID_BASE
        command = data[0]
        frame = {
            "motor_id": motor_id,
            "command": command,
            "arbitration_id": f"0x{arbitration_id:03X}",
            "timestamp": timestamp,
            "data": data,
            "data_hex": bytes_hex(data),
        }
        with self._motor_response_condition:
            self._motor_frame_seq += 1
            frame["seq"] = self._motor_frame_seq
            self._motor_recent_frames[(motor_id, command)] = frame
            self._motor_frame_history.append(frame)
            self._motor_response_condition.notify_all()
        self._log_can_received(
            arbitration_id,
            data,
            context=f"Motor frame motor={motor_id} cmd=0x{command:02X}",
        )

    def _motor_wait_for_response(self, motor_id: int, command: int, min_seq: int, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        with self._motor_response_condition:
            while True:
                frame = self._motor_recent_frames.get((motor_id, command))
                if frame and frame.get("seq", -1) > min_seq:
                    return dict(frame)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Timeout waiting for motor_id={motor_id} cmd=0x{command:02X}"
                    )
                self._motor_response_condition.wait(timeout=remaining)

    def _motor_exchange_locked(
        self,
        *,
        motor_id: int,
        arbitration_id: int,
        payload: bytes,
        expected_cmd: int,
        timeout_s: float,
        context: str,
    ) -> Dict[str, Any]:
        with self._motor_response_condition:
            min_seq = self._motor_frame_seq
        self._send_frame(arbitration_id, payload, context=context)
        return self._motor_wait_for_response(motor_id, expected_cmd, min_seq, timeout_s)

    def _motor_send_best_effort_locked(
        self,
        *,
        motor_id: int,
        arbitration_id: int,
        payload: bytes,
        expected_cmd: Optional[int],
        timeout_s: float,
        context: str,
    ) -> Dict[str, Any]:
        with self._motor_response_condition:
            min_seq = self._motor_frame_seq
        self._send_frame(arbitration_id, payload, context=context)
        if expected_cmd is None:
            return {"timed_out": False, "response": None}
        try:
            response = self._motor_wait_for_response(motor_id, expected_cmd, min_seq, timeout_s)
            return {"timed_out": False, "response": response}
        except TimeoutError:
            return {"timed_out": True, "response": None}

    def _motor_send_blind_locked(
        self,
        *,
        arbitration_id: int,
        payload: bytes,
        context: str,
    ) -> Dict[str, Any]:
        self._send_frame(arbitration_id, payload, context=context)
        return {"tx_hex": bytes_hex(payload)}

    def _motor_force_stop_blind(
        self,
        *,
        motor_id: int,
        arbitration_id: int,
        timeout_s: float,
        power_off: bool,
        repeats: int = 3,
    ) -> list[Dict[str, Any]]:
        details: list[Dict[str, Any]] = []
        steps = [
            ("torque_zero", build_control_frame("torque", 0.0)),
            ("speed_zero", build_control_frame("speed", 0.0)),
            ("stop", build_zero_frame(CONTROL_COMMANDS["stop"])),
        ]
        if power_off:
            steps.append(("off", build_zero_frame(CONTROL_COMMANDS["off"])))

        for repeat_index in range(repeats):
            for label, payload in steps:
                self._send_frame(
                    arbitration_id=arbitration_id,
                    data=payload,
                    context=f"Motor blind stop {label} motor={motor_id} repeat={repeat_index + 1}",
                )
                details.append(
                    {
                        "repeat": repeat_index + 1,
                        "step": label,
                        "tx_hex": bytes_hex(payload),
                    }
                )
                time.sleep(min(0.03, timeout_s))
        return details

    def _motor_force_stop_blind_locked(
        self,
        *,
        motor_id: int,
        arbitration_id: int,
        timeout_s: float,
        power_off: bool,
        repeats: int = 3,
    ) -> list[Dict[str, Any]]:
        return self._motor_force_stop_blind(
            motor_id=motor_id,
            arbitration_id=arbitration_id,
            timeout_s=timeout_s,
            power_off=power_off,
            repeats=repeats,
        )

    def _motor_safe_stop_locked(
        self,
        *,
        motor_id: int,
        arbitration_id: int,
        timeout_s: float,
        power_off: bool,
    ) -> Dict[str, Any]:
        steps = [
            ("speed_zero", build_control_frame("speed", 0.0), CONTROL_COMMANDS["speed"]),
            ("torque_zero", build_control_frame("torque", 0.0), CONTROL_COMMANDS["torque"]),
            ("stop", build_zero_frame(CONTROL_COMMANDS["stop"]), CONTROL_COMMANDS["stop"]),
        ]
        if power_off:
            steps.append(("off", build_zero_frame(CONTROL_COMMANDS["off"]), CONTROL_COMMANDS["off"]))

        warnings = []
        details = []
        for label, payload, expected_cmd in steps:
            result = self._motor_send_best_effort_locked(
                motor_id=motor_id,
                arbitration_id=arbitration_id,
                payload=payload,
                expected_cmd=expected_cmd,
                timeout_s=timeout_s,
                context=f"Motor safe stop {label} motor={motor_id}",
            )
            details.append({
                "step": label,
                "tx_hex": bytes_hex(payload),
                "response": self._serialize_motor_frame(result["response"]) if result["response"] else None,
                "timed_out": result["timed_out"],
            })
            if result["timed_out"]:
                warnings.append(f"{label} echo timed out")
            time.sleep(min(0.05, timeout_s))

        blind_stop_steps = self._motor_force_stop_blind_locked(
            motor_id=motor_id,
            arbitration_id=arbitration_id,
            timeout_s=timeout_s,
            power_off=power_off,
        )

        return {
            "safe_stop_steps": details,
            "blind_stop_steps": blind_stop_steps,
            "warning": "; ".join(warnings) if warnings else None,
        }

    def _decode_motor_response(self, data: bytes, *, output_ratio: float) -> Dict[str, Any]:
        command = data[0]
        if command in (CONTROL_COMMANDS["torque"], CONTROL_COMMANDS["speed"], READ_COMMANDS["state2"]):
            return decode_feedback_frame(data, output_ratio=output_ratio)
        if command == READ_COMMANDS["pid"]:
            return decode_pid_frame(data)
        if command == READ_COMMANDS["acceleration"]:
            return decode_acceleration_frame(data)
        if command == READ_COMMANDS["raw"]:
            return decode_raw_frame(data, output_ratio=output_ratio)
        if command == READ_COMMANDS["single"]:
            return decode_single_frame(data, output_ratio=output_ratio)
        if command == READ_COMMANDS["multi"]:
            return decode_multi_frame(data, output_ratio=output_ratio)
        return {
            "motor_cmd_echo": command,
            "raw_hex": bytes_hex(data),
        }

    def _motor_exchange_result(
        self,
        *,
        action: str,
        motor_id: int,
        arbitration_id: int,
        payload: bytes,
        response: Dict[str, Any],
        output_ratio: float,
        target: Optional[float] = None,
    ) -> Dict[str, Any]:
        result = {
            "action": action,
            "motor_id": motor_id,
            "arbitration_id": f"0x{arbitration_id:03X}",
            "tx_hex": bytes_hex(payload),
            "response": self._serialize_motor_frame(response),
            "decoded": self._decode_motor_response(response["data"], output_ratio=output_ratio),
        }
        if target is not None:
            result["target"] = target
        return result

    def _serialize_motor_frame(self, response: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "motor_id": response.get("motor_id"),
            "command": response.get("command"),
            "arbitration_id": response.get("arbitration_id"),
            "timestamp": response.get("timestamp"),
            "data_hex": response.get("data_hex"),
            "seq": response.get("seq"),
        }

    def _decode_status_frame(self, arbitration_id: int, data: bytes, timestamp: float) -> None:
        # NEW: Status base changed from 0x200 to 0x400
        joint_id = arbitration_id - 0x400
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")

        dof_index = data[0]
        current_angle = struct.unpack_from("<h", data, 1)[0] / 100.0
        target_angle = struct.unpack_from("<h", data, 3)[0] / 100.0
        progress = data[5]
        flags = data[6]
        temperature = data[7]

        status_payload = {
            "joint": joint_name,
            "joint_id": joint_id,
            "timestamp": timestamp,
            "dof_index": dof_index,
            "current_angle_deg": current_angle,
            "target_angle_deg": target_angle,
            "progress_pct": progress,
            "flags": flags,
            "temperature_c": temperature,
            "arbitration_id": f"0x{arbitration_id:03X}",
        }

        with self._lock:
            self._last_status_by_joint[joint_name] = status_payload
            self._status_messages.appendleft({"type": "status", "data": status_payload})

        context = (
            f"Status joint={joint_name} dof={dof_index} current={current_angle:.2f}° "
            f"target={target_angle:.2f}° progress={progress}% flags=0x{flags:02X} temp={temperature}C"
        )
        self._log_can_received(arbitration_id, data, context=context)

        if self.socketio:
            try:
                self.socketio.emit("can_status", status_payload, namespace="/movement")
            except Exception:  # pragma: no cover - optional socket broadcast
                self.logger.debug("SocketIO emit failed for CAN status", exc_info=True)

    def _handle_encoder_stream(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode encoder stream data from CAN (0x410-0x41F).
        
        Each joint sends on its own CAN ID: 0x410 + joint_id
        This allows filtering when multiple controllers are on the bus.
        
        Frame format (8 bytes):
        - Bytes 0-1: int16_t dof0_angle (0.01° resolution, 0x7FFF = invalid)
        - Bytes 2-3: int16_t dof1_angle (0.01° resolution, 0x7FFF = invalid)
        - Bytes 4-5: int16_t dof2_angle (0.01° resolution, 0x7FFF = invalid)
        - Bytes 6-7: uint16_t t_offset_ms (ms since last time sync)
        """
        UNUSED_DOF = 0x7FFF
        
        # Unpack: 3x int16 angles + 1x uint16 timestamp
        dof0_raw, dof1_raw, dof2_raw, t_ms = struct.unpack("<hhhH", data)
        
        # Convert to degrees (0.01° resolution)
        angles_deg = []
        for raw in [dof0_raw, dof1_raw, dof2_raw]:
            if raw == UNUSED_DOF:
                angles_deg.append(None)
            else:
                angles_deg.append(raw / 100.0)
        
        # Look up joint name from ID
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id}")
        
        # Build data point with joint info
        data_point = {
            "timestamp": timestamp,
            "t_ms": t_ms,
            "angles_deg": angles_deg,
            "joint_id": joint_id,
            "joint_name": joint_name,
        }
        
        # Buffer data and store last known angles
        with self._lock:
            self._encoder_stream_data.append(data_point)
            buffer_size = len(self._encoder_stream_data)
            # Store last known angles for on-demand access (persists after streaming stops)
            self._last_encoder_angles[joint_id] = {
                "angles_deg": angles_deg,
                "timestamp_ms": t_ms,
                "timestamp": timestamp,
                "joint_name": joint_name,
            }
        
        # Debug log (throttled to 1Hz per joint)
        self._stats["rx_frames"] += 1
        # Track frames per joint for separate throttling
        if not hasattr(self, '_rx_frames_per_joint'):
            self._rx_frames_per_joint = {}
        self._rx_frames_per_joint[joint_id] = self._rx_frames_per_joint.get(joint_id, 0) + 1
        
        if self._rx_frames_per_joint[joint_id] % 50 == 0:  # Log every 50 frames per joint
            dof0_str = f"{angles_deg[0]:.2f}°" if angles_deg[0] is not None else "N/A"
            self._log_can_info(f"Encoder stream RX [{joint_name}] (0x{0x410+joint_id:03X}): {buffer_size} buffered, DOF0={dof0_str}")
        
        # Invoke callback if set (for real-time UI updates)
        if self._encoder_stream_callback:
            try:
                self._encoder_stream_callback(angles_deg, t_ms, joint_id)
            except Exception:
                pass  # Don't let callback errors break streaming
        
        # Emit via SocketIO for real-time UI updates
        if self.socketio:
            try:
                self.socketio.emit("encoder_stream", data_point, namespace="/movement")
            except Exception:
                pass  # Don't let socket errors break streaming

    def _handle_pid_diag_data(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode PID diagnostics data from CAN (0x420-0x42F).
        
        Frame format (8 bytes):
        - Bytes 0-1: int16_t target0 (0.01° resolution)
        - Bytes 2-3: int16_t target1 (0.01° resolution)
        - Bytes 4-5: int16_t error0 (0.01° resolution)
        - Bytes 6-7: int16_t error1 (0.01° resolution)
        """
        target0_raw, target1_raw, error0_raw, error1_raw = struct.unpack("<hhhh", data[:8])
        
        data_point = {
            "type": "pid_diag",
            "joint_id": joint_id,
            "joint_name": self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}"),
            "target_deg": [target0_raw / 100.0, target1_raw / 100.0],
            "error_deg": [error0_raw / 100.0, error1_raw / 100.0],
            "timestamp": timestamp,
        }
        
        # Emit via SocketIO for real-time UI updates
        if self.socketio:
            try:
                self.socketio.emit("pid_diag", data_point, namespace="/movement")
            except Exception:
                pass

    def _handle_pid_torque_data(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode PID torque command data from CAN (0x430-0x43F).
        
        Frame format (8 bytes):
        - Bytes 0-1: int16_t torque_A0 (agonist DOF0)
        - Bytes 2-3: int16_t torque_B0 (antagonist DOF0)
        - Bytes 4-5: int16_t torque_A1 (agonist DOF1)
        - Bytes 6-7: int16_t torque_B1 (antagonist DOF1)
        """
        torque_A0, torque_B0, torque_A1, torque_B1 = struct.unpack("<hhhh", data[:8])
        
        data_point = {
            "type": "pid_torque",
            "joint_id": joint_id,
            "joint_name": self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}"),
            "torque_A": [torque_A0, torque_A1],
            "torque_B": [torque_B0, torque_B1],
            "timestamp": timestamp,
        }
        
        # Emit via SocketIO for real-time UI updates
        if self.socketio:
            try:
                self.socketio.emit("pid_torque", data_point, namespace="/movement")
            except Exception:
                pass

    def _handle_joint_state(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode JOINT_STATE broadcast from CAN (0x4F0 + joint_id).

        Frame format (8 bytes):
        - Byte 0: uint8 dof index
        - Bytes 1-2: int16 q_actual (x100, 0.01° resolution)
        - Bytes 3-4: int16 dq_actual (x10, 0.1°/s resolution)
        - Byte 5: int8 tau_A (torque A / 4)
        - Byte 6: int8 tau_B (torque B / 4)
        - Byte 7: uint8 status bits (bit0=valid, bit1=holding, bit2=watchdog_warning)
        """
        dof = data[0]
        q_raw, dq_raw = struct.unpack_from("<hh", data, 1)
        tau_a_div4 = struct.unpack_from("<b", data, 5)[0]
        tau_b_div4 = struct.unpack_from("<b", data, 6)[0]
        status = data[7]

        q_deg = q_raw / 100.0
        dq_deg_s = dq_raw / 10.0
        tau_a = tau_a_div4 * 4
        tau_b = tau_b_div4 * 4

        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")

        state_entry = {
            "q_deg": q_deg,
            "dq_deg_s": dq_deg_s,
            "tau_a": tau_a,
            "tau_b": tau_b,
            "status": status,
            "valid": bool(status & 0x01),
            "holding": bool(status & 0x02),
            "watchdog_warning": bool(status & 0x04),
            "timestamp": timestamp,
        }

        with self._lock:
            if joint_name not in self._joint_state:
                self._joint_state[joint_name] = {}
            self._joint_state[joint_name][dof] = state_entry

        # Emit via SocketIO for real-time UI updates
        data_point = {
            "type": "joint_state",
            "joint_id": joint_id,
            "joint_name": joint_name,
            "dof": dof,
            **state_entry,
        }
        if self.socketio:
            try:
                self.socketio.emit("joint_state", data_point, namespace="/movement")
            except Exception:
                pass  # Don't let socket errors break streaming

    def _handle_diag_hold(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode DIAG_HOLD frames from CAN (0x4D0 + joint_id).
        Two frames per DOF, distinguished by bit7 of byte 0:
          Frame 1 (dof & 0x7F): ema, resA, resB, flags
          Frame 2 (dof | 0x80): iqA, iqB, stiffness, trim
        Accumulates both frames, emits socket event when frame 2 arrives.
        """
        dof_byte = data[0]
        is_frame2 = bool(dof_byte & 0x80)
        dof = dof_byte & 0x7F

        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")

        if not hasattr(self, "_diag_hold_pending"):
            self._diag_hold_pending = {}

        key = (joint_id, dof)

        if not is_frame2:
            # Frame 1: dof(u8), ema(i16), resA(i16), resB(i16), flags(u8)
            ema_x100, res_a_x100, res_b_x100 = struct.unpack_from("<hhh", data, 1)
            flags = data[7]
            self._diag_hold_pending[key] = {
                "type": "diag_hold",
                "joint_id": joint_id,
                "joint_name": joint_name,
                "dof": dof,
                "ema": ema_x100 / 100.0,
                "residual_A": res_a_x100 / 100.0,
                "residual_B": res_b_x100 / 100.0,
                "iq_valid": bool(flags & 0x01),
                "ema_settled": bool(flags & 0x02),
                "timestamp": timestamp,
            }
        else:
            # Frame 2: dof|0x80(u8), iqA(i16), iqB(i16), stiff_x10(i16), trim_x10(i8 signed)
            iq_a, iq_b, stiff_x10 = struct.unpack_from("<hhh", data, 1)
            trim_x10 = struct.unpack_from("<b", data, 7)[0]  # signed int8

            if key in self._diag_hold_pending:
                entry = self._diag_hold_pending.pop(key)
                entry["iq_A"] = iq_a
                entry["iq_B"] = iq_b
                iq_abs_max = max(abs(iq_a), abs(iq_b))
                entry["iq_ratio"] = (
                    min(abs(iq_a), abs(iq_b)) / iq_abs_max
                    if entry["iq_valid"] and iq_abs_max > 0
                    else -1.0
                )
                entry["stiffness"] = stiff_x10 / 10.0
                entry["tension_trim"] = trim_x10 / 10.0  # Phase 2

                if self.socketio:
                    try:
                        self.socketio.emit("diag_hold", entry, namespace="/movement")
                    except Exception:
                        pass

    def _handle_retension_probe_result(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode retension-probe result frames from CAN (0x500 + joint_id).

        Three frames per DOF:
          Frame 1: dof, q, base stiffness, pre_ratio, flags
          Frame 2: dof|0x40, dur_ratio, delta_ratio, recruit_norm, class
          Frame 3: dof|0x80, effort_pre, boost, pulse_ms, min_samples
        """
        marker = data[0]
        frame_kind = marker & 0xC0
        dof = marker & 0x3F
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")
        key = (joint_id, dof)
        class_names = {
            0: "UNKNOWN",
            1: "LOW_EFFORT",
            2: "NO_CORRECTION",
            3: "NO_EFFECT",
            4: "SLACK_LIKELY",
        }

        if frame_kind == 0x00:
            q_x100, base_stiff_x10, pre_ratio_x1000 = struct.unpack_from("<hhh", data, 1)
            flags = data[7]
            self._retension_probe_pending[key] = {
                "type": "retension_probe",
                "joint_id": joint_id,
                "joint_name": joint_name,
                "dof": dof,
                "q_deg": q_x100 / 100.0,
                "baseline_stiffness_deg": base_stiff_x10 / 10.0,
                "pre_ratio": pre_ratio_x1000 / 1000.0,
                "weak_side": "B" if (flags & 0x01) else "A",
                "timestamp": timestamp,
            }
            return

        entry = self._retension_probe_pending.get(key)
        if entry is None:
            return

        if frame_kind == 0x40:
            dur_ratio_x1000, delta_ratio_x1000, recruit_norm_x1000 = struct.unpack_from("<hhh", data, 1)
            class_code = data[7]
            entry["dur_ratio"] = dur_ratio_x1000 / 1000.0
            entry["delta_ratio"] = delta_ratio_x1000 / 1000.0
            entry["recruit_norm"] = recruit_norm_x1000 / 1000.0
            entry["class_code"] = class_code
            entry["classification"] = class_names.get(class_code, f"CODE_{class_code}")
            return

        if frame_kind == 0x80:
            effort_pre, boost_x10, pulse_ms = struct.unpack_from("<HhH", data, 1)
            min_samples = data[7]
            entry["effort_pre"] = int(effort_pre)
            entry["probe_boost_deg"] = boost_x10 / 10.0
            entry["probe_pulse_ms"] = int(pulse_ms)
            entry["min_samples"] = int(min_samples)
            with self._lock:
                self._retension_probe_state.setdefault(joint_name, {})[dof] = dict(entry)
            self._probe_history.append(entry)
            self._retension_probe_pending.pop(key, None)
            if self.socketio:
                try:
                    self.socketio.emit("retension_probe", entry, namespace="/movement")
                except Exception:
                    pass

    def _publish_diagnostic(self, payload: Dict[str, Any], socket_event: str) -> None:
        try:
            self._diagnostic_history.append(payload)
        except Exception:
            self.logger.warning("Failed to persist diagnostic history", exc_info=True)

        with self._lock:
            self._status_messages.appendleft({"type": payload["type"], "data": payload})

        if self.socketio:
            try:
                self.socketio.emit(socket_event, payload, namespace="/movement")
            except Exception:
                pass

    def _handle_health_status(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        frame_kind = data[0] & 0xC0
        seq = data[7]
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")
        key = (joint_id, seq)

        with self._lock:
            for stale_key in [item for item in self._health_status_pending if item[0] == joint_id and item != key]:
                self._health_status_pending.pop(stale_key, None)
            pending = self._health_status_pending.setdefault(
                key,
                {
                    "joint": joint_name,
                    "joint_name": joint_name,
                    "joint_id": joint_id,
                    "timestamp": timestamp,
                    "seq": seq,
                },
            )
            pending["timestamp"] = timestamp
            if frame_kind == 0x00:
                pending["summary"] = {
                    "state_bits": data[1],
                    "phase_code": data[2],
                    "reboot_reason_code": data[3],
                    "uptime_s": struct.unpack_from("<H", data, 4)[0],
                    "fault_epoch": data[6],
                }
            elif frame_kind == 0x40:
                pending["counters"] = {
                    "host_can_tx_error_count": data[1],
                    "host_can_rx_error_count": data[2],
                    "motor_can_tx_error_count": data[3],
                    "loop_overrun_count": data[4],
                    "watchdog_trip_count": data[5],
                    "can_recovery_count": data[6],
                }
            elif frame_kind == 0x80:
                avg_us, max_us, budget_us = struct.unpack_from("<HHH", data, 2)
                diag = self._diagnostics.setdefault(joint_name, {})
                diag["loop_avg_us"] = avg_us
                diag["loop_max_us"] = max_us
                diag["loop_budget_us"] = budget_us
                return
            else:
                return

            summary = pending.get("summary")
            counters = pending.get("counters")
            if not summary or not counters:
                return

            payload = {
                "type": "health_status",
                "joint": joint_name,
                "joint_name": joint_name,
                "joint_id": joint_id,
                "timestamp": pending["timestamp"],
                "seq": seq,
                "state_bits": summary["state_bits"],
                "state": _decode_state_bits(summary["state_bits"]),
                "phase_code": summary["phase_code"],
                "phase": DIAG_PHASE_NAMES.get(summary["phase_code"], f"CODE_{summary['phase_code']}"),
                "reboot_reason_code": summary["reboot_reason_code"],
                "reboot_reason": DIAG_REBOOT_REASON_NAMES.get(
                    summary["reboot_reason_code"], f"CODE_{summary['reboot_reason_code']}"
                ),
                "uptime_s": summary["uptime_s"],
                "fault_epoch": summary["fault_epoch"],
                **counters,
            }
            self._health_status[joint_name] = dict(payload)
            self._health_status_pending.pop(key, None)

        self._publish_diagnostic(payload, "health_status")

    def _handle_fault_status(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        seq, active_bits, latched_bits, primary_fault_code, source_id, fault_epoch = struct.unpack(
            "<BHHBBB", data[:8]
        )
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")
        source_info = decode_source_id(source_id)

        payload = {
            "type": "fault_status",
            "joint": joint_name,
            "joint_name": joint_name,
            "joint_id": joint_id,
            "timestamp": timestamp,
            "seq": seq,
            "active_fault_bits": active_bits,
            "latched_fault_bits": latched_bits,
            "active_faults": decode_fault_mask(active_bits),
            "latched_faults": decode_fault_mask(latched_bits),
            "primary_fault_code": None if primary_fault_code == 0xFF else primary_fault_code,
            "primary_fault": None
            if primary_fault_code == 0xFF
            else DIAG_FAULT_NAMES.get(primary_fault_code, f"CODE_{primary_fault_code}"),
            "source_id": source_id,
            "fault_epoch": fault_epoch,
            **source_info,
        }

        with self._lock:
            self._fault_status[joint_name] = dict(payload)

        self._publish_diagnostic(payload, "fault_status")

    def _handle_event_notice(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        event_code, flags, source_kind, source_index, detail0, detail1, event_seq = struct.unpack(
            "<BBBBBBH", data[:8]
        )
        severity_code = flags & 0x03
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")

        payload: Dict[str, Any] = {
            "type": "event_notice",
            "joint": joint_name,
            "joint_name": joint_name,
            "joint_id": joint_id,
            "timestamp": timestamp,
            "event_seq": event_seq,
            "event_code": event_code,
            "event": DIAG_EVENT_NAMES.get(event_code, f"CODE_{event_code}"),
            "flags": flags,
            "severity_code": severity_code,
            "severity": DIAG_SEVERITY_NAMES.get(severity_code, f"CODE_{severity_code}"),
            "is_assert": bool(flags & 0x04),
            "is_clear": bool(flags & 0x08),
            "latched_changed": bool(flags & 0x10),
            "snapshot_frozen": bool(flags & 0x20),
            "host_attention": bool(flags & 0x40),
            "source_kind_code": source_kind,
            "source_kind": DIAG_SOURCE_NAMES.get(source_kind, f"CODE_{source_kind}"),
            "source_index": None if source_index == 0xFF else source_index,
            "detail0": detail0,
            "detail1": detail1,
        }

        if event_code in {0x01}:
            payload["reboot_reason_code"] = detail0
            payload["reboot_reason"] = DIAG_REBOOT_REASON_NAMES.get(detail0, f"CODE_{detail0}")
            payload["phase_code"] = detail1
            payload["phase"] = DIAG_PHASE_NAMES.get(detail1, f"CODE_{detail1}")
        elif event_code in {0x02, 0x03}:
            payload["previous_phase_code"] = detail0
            payload["previous_phase"] = DIAG_PHASE_NAMES.get(detail0, f"CODE_{detail0}")
            payload["phase_code"] = detail1
            payload["phase"] = DIAG_PHASE_NAMES.get(detail1, f"CODE_{detail1}")
        elif event_code in {0x04, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11}:
            payload["phase_code"] = detail1
            payload["phase"] = DIAG_PHASE_NAMES.get(detail1, f"CODE_{detail1}")

        if event_code in {0x0B, 0x0C, 0x0D}:
            payload["fault_code"] = detail0
            payload["fault_name"] = DIAG_FAULT_NAMES.get(detail0, f"CODE_{detail0}")
        elif event_code in {0x05, 0x06}:
            payload["startup_reason_code"] = detail0
            payload["startup_reason"] = STARTUP_REASON_NAMES.get(detail0, f"CODE_{detail0}")
            payload["elapsed_ms_approx"] = detail1
        elif event_code == 0x08:
            payload["elapsed_10ms"] = detail0
        elif event_code == 0x0F:
            payload["loop_overrun_count"] = detail0
        elif event_code == 0x10:
            payload["fault_code"] = detail0
            payload["fault_name"] = DIAG_FAULT_NAMES.get(detail0, f"CODE_{detail0}")

        with self._lock:
            self._diagnostic_events.appendleft(dict(payload))

        self._publish_diagnostic(payload, "event_notice")

    def _handle_fault_snapshot_meta(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        meta = decode_fault_snapshot_meta(data, joint_id)
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}")
        payload = {
            "type": "fault_snapshot_meta",
            "joint": joint_name,
            "joint_name": joint_name,
            "joint_id": joint_id,
            "timestamp": timestamp,
            "snapshot_id": meta.snapshot_id,
            "freeze_event_code": meta.freeze_event_code,
            "freeze_event": meta.freeze_event_name,
            "primary_fault_code": meta.primary_fault_code,
            "primary_fault": meta.primary_fault_name,
            "flags": meta.flags,
            "state": {
                "snapshot_present": meta.snapshot_present,
                "frozen_on_critical_fault": meta.frozen_on_critical_fault,
                "truncated": meta.truncated,
                "dumped_once": meta.dumped_once,
                "checksum_available": meta.checksum_available,
                "fixed_layout_v1": meta.fixed_layout_v1,
            },
            "total_chunks": meta.total_chunks,
            "payload_bytes": meta.payload_bytes,
            "seq": meta.seq,
        }

        with self._lock:
            self._fault_snapshot_meta[joint_name] = dict(payload)

        if meta.snapshot_present and meta.total_chunks > 0 and meta.payload_bytes > 0:
            pending = self._fault_snapshot_pending.get(joint_id)
            if pending is None or pending.get("snapshot_id") != meta.snapshot_id:
                pending = {"chunks": {}}
                self._fault_snapshot_pending[joint_id] = pending
            pending.update(
                {
                    "snapshot_id": meta.snapshot_id,
                    "total_chunks": meta.total_chunks,
                    "payload_bytes": meta.payload_bytes,
                    "meta_payload": payload,
                    "last_timestamp": timestamp,
                }
            )
        else:
            self._fault_snapshot_pending.pop(joint_id, None)

        self._publish_diagnostic(payload, "fault_snapshot_meta")

    def _handle_fault_snapshot_chunk(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        chunk = decode_fault_snapshot_chunk(data, joint_id)
        pending = self._fault_snapshot_pending.setdefault(joint_id, {"chunks": {}})
        if pending.get("snapshot_id") not in (None, chunk.snapshot_id):
            pending.clear()
            pending["chunks"] = {}
        pending["snapshot_id"] = chunk.snapshot_id
        pending.setdefault("chunks", {})[chunk.chunk_index] = chunk.payload
        pending["last_timestamp"] = timestamp

        total_chunks = pending.get("total_chunks")
        payload_bytes = pending.get("payload_bytes")
        meta_payload = pending.get("meta_payload")
        if not isinstance(total_chunks, int) or not isinstance(payload_bytes, int) or not isinstance(meta_payload, dict):
            return

        chunks: Dict[int, bytes] = pending["chunks"]  # type: ignore[assignment]
        if len(chunks) < total_chunks:
            return
        if any(index not in chunks for index in range(total_chunks)):
            return

        raw = b"".join(chunks[index] for index in range(total_chunks))[:payload_bytes]
        decoded_snapshot = decode_fault_snapshot_blob(raw)
        payload = {
            "type": "fault_snapshot_dump",
            "joint": meta_payload["joint_name"],
            "joint_name": meta_payload["joint_name"],
            "joint_id": joint_id,
            "timestamp": timestamp,
            "snapshot_id": meta_payload["snapshot_id"],
            "freeze_event_code": meta_payload["freeze_event_code"],
            "freeze_event": meta_payload["freeze_event"],
            "primary_fault_code": meta_payload["primary_fault_code"],
            "primary_fault": meta_payload["primary_fault"],
            "payload_bytes": len(raw),
            "total_chunks": total_chunks,
            "snapshot": decoded_snapshot,
            "raw_hex": raw.hex(),
        }

        with self._lock:
            self._fault_snapshots[meta_payload["joint_name"]] = dict(payload)
        self._fault_snapshot_pending.pop(joint_id, None)
        self._publish_diagnostic(payload, "fault_snapshot_dump")

    def _expire_fault_snapshot_pending(self, now_timestamp: float) -> None:
        stale_joint_ids = [
            joint_id
            for joint_id, pending in self._fault_snapshot_pending.items()
            if isinstance(pending.get("last_timestamp"), (int, float))
            and (now_timestamp - float(pending["last_timestamp"])) > FAULT_SNAPSHOT_PENDING_TIMEOUT_S
        ]
        for joint_id in stale_joint_ids:
            self._fault_snapshot_pending.pop(joint_id, None)

    def get_joint_state(self) -> Dict[str, Any]:
        """Return current joint state data for all joints/DOFs in impedance mode."""
        with self._lock:
            # Deep copy to avoid race conditions
            return {
                joint_name: {
                    str(dof): {
                        **dict(state),
                        **(
                            {"retension_probe": dict(self._retension_probe_state.get(joint_name, {}).get(dof, {}))}
                            if self._retension_probe_state.get(joint_name, {}).get(dof)
                            else {}
                        ),
                    }
                    for dof, state in dofs.items()
                }
                for joint_name, dofs in self._joint_state.items()
            }

    def _handle_pid_inner_terms(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode inner PID terms breakdown from CAN (0x470-0x47F).

        Frame format (8 bytes):
        - Bytes 0-1: int16_t p_term (proportional increment)
        - Bytes 2-3: int16_t i_term (integral term)
        - Bytes 4-5: int16_t d_term (filtered derivative term)
        - Bytes 6-7: int16_t ff_term (feedforward term)
        """
        p_raw, i_raw, d_raw, ff_raw = struct.unpack("<hhhh", data[:8])

        # Firmware sends values scaled ×100 (incremental PID terms are small floats)
        data_point = {
            "type": "pid_inner_terms",
            "joint_id": joint_id,
            "joint_name": self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}"),
            "p_term": p_raw / 100.0,
            "i_term": i_raw / 100.0,
            "d_term": d_raw / 100.0,
            "ff_term": ff_raw / 100.0,
            "timestamp": timestamp,
        }

        if self.socketio:
            try:
                self.socketio.emit("pid_inner_terms", data_point, namespace="/movement")
            except Exception:
                pass

    def _handle_pid_outer_terms(self, data: bytes, timestamp: float, joint_id: int = 0) -> None:
        """
        Decode outer PID terms breakdown from CAN (0x480-0x48F).

        Frame format (8 bytes):
        - Bytes 0-1: int16_t p_term (proportional increment)
        - Bytes 2-3: int16_t i_term (integral term)
        - Bytes 4-5: int16_t d_term (filtered derivative term)
        - Bytes 6-7: int16_t output_x100 (delta_theta × 100)
        """
        p_raw, i_raw, d_raw, output_x100 = struct.unpack("<hhhh", data[:8])

        # Firmware sends P/I/D values scaled ×100 (incremental terms are small floats)
        # output_x100 is delta_theta × 100 (already scaled in firmware)
        data_point = {
            "type": "pid_outer_terms",
            "joint_id": joint_id,
            "joint_name": self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}"),
            "p_term": p_raw / 100.0,
            "i_term": i_raw / 100.0,
            "d_term": d_raw / 100.0,
            "output_x100": output_x100,
            "timestamp": timestamp,
        }

        if self.socketio:
            try:
                self.socketio.emit("pid_outer_terms", data_point, namespace="/movement")
            except Exception:
                pass

    def _handle_movement_metrics_frame1(self, data: bytes, timestamp: float, joint_id: int, dof: int) -> None:
        """
        Decode movement metrics frame 1 from CAN (0x440-0x44F).
        
        Frame format (8 bytes):
        - Bytes 0-1: uint16_t rise_time_ms
        - Bytes 2-3: uint16_t settling_time_ms
        - Bytes 4-5: int16_t overshoot (% × 100)
        - Bytes 6-7: int16_t sse (° × 100)
        """
        rise_time, settling_time, overshoot_x100, sse_x100 = struct.unpack("<HHhh", data[:8])
        
        # Store partial metrics (wait for frame 2 to complete)
        key = (joint_id, dof)
        if not hasattr(self, '_pending_metrics'):
            self._pending_metrics = {}
        
        self._pending_metrics[key] = {
            "type": "movement_metrics",
            "joint_id": joint_id,
            "joint_name": self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}"),
            "dof": dof,
            "rise_time_ms": rise_time,
            "settling_time_ms": settling_time,
            "overshoot_pct": overshoot_x100 / 100.0,
            "sse_deg": sse_x100 / 100.0,
            "timestamp": timestamp,
        }
        
        self.logger.debug(f"Metrics frame 1: DOF {dof} rise={rise_time}ms settle={settling_time}ms")

    def _handle_movement_metrics_frame2(self, data: bytes, timestamp: float, joint_id: int, dof: int) -> None:
        """
        Decode movement metrics frame 2 from CAN (0x450-0x45F).
        
        Frame format (8 bytes):
        - Bytes 0-1: int16_t max_torque_A
        - Bytes 2-3: int16_t max_torque_B
        - Bytes 4-5: uint16_t duration_ms
        - Byte 6: uint8_t dof_index
        - Byte 7: uint8_t flags
        """
        max_torque_A, max_torque_B, duration_ms, dof_index, flags = struct.unpack("<hhHBB", data[:8])
        
        # Complete metrics with frame 1 data
        key = (joint_id, dof)
        if not hasattr(self, '_pending_metrics'):
            self._pending_metrics = {}
        
        if key in self._pending_metrics:
            metrics = self._pending_metrics.pop(key)
            metrics["max_torque_A"] = max_torque_A
            metrics["max_torque_B"] = max_torque_B
            metrics["duration_ms"] = duration_ms
            metrics["flags"] = flags
            
            # Calculate a score (0-100) based on metrics
            score = self._calculate_movement_score(metrics)
            metrics["score"] = score
            
            # Log the complete metrics
            self.logger.info(
                f"Movement metrics DOF {dof}: rise={metrics['rise_time_ms']}ms, "
                f"settle={metrics['settling_time_ms']}ms, overshoot={metrics['overshoot_pct']:.1f}%, "
                f"sse={metrics['sse_deg']:.2f}°, score={score}/100"
            )
            
            # Emit via SocketIO for UI display
            if self.socketio:
                try:
                    self.socketio.emit("movement_metrics", metrics, namespace="/movement")
                except Exception:
                    pass
        else:
            self.logger.warning(f"Metrics frame 2 received without frame 1 for DOF {dof}")

    def _handle_smoothness_metrics_frame(self, data: bytes, timestamp: float, joint_id: int, dof: int) -> None:
        """
        Decode smoothness metrics frame from CAN (0x460-0x46F).
        
        Frame format (8 bytes):
        - Bytes 0-1: int16_t rms_error (° × 100)
        - Bytes 2-3: int16_t jitter (° × 100)
        - Byte 4: uint8_t oscillation_count
        - Byte 5: uint8_t score_rms (0-100)
        - Byte 6: uint8_t score_jitter (0-100)
        - Byte 7: uint8_t score_smoothness (0-100)
        """
        rms_x100, jitter_x100, osc_count, score_rms, score_jitter, score_smooth = struct.unpack("<hhBBBB", data[:8])
        
        smoothness_metrics = {
            "type": "smoothness_metrics",
            "joint_id": joint_id,
            "joint_name": self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id:02d}"),
            "dof": dof,
            "rms_error_deg": rms_x100 / 100.0,
            "jitter_deg": jitter_x100 / 100.0,
            "oscillation_count": osc_count,
            "score_rms": score_rms,
            "score_jitter": score_jitter,
            "score_smoothness": score_smooth,
            "timestamp": timestamp,
        }
        
        # Log smoothness metrics
        self.logger.info(
            f"Smoothness metrics DOF {dof}: rms={rms_x100/100.0:.2f}°({score_rms}), "
            f"osc={osc_count}, jitter={jitter_x100/100.0:.3f}°({score_jitter}), "
            f"smooth={score_smooth}/100"
        )
        
        # Emit via SocketIO for UI display
        if self.socketio:
            try:
                self.socketio.emit("smoothness_metrics", smoothness_metrics, namespace="/movement")
            except Exception:
                pass

    def _calculate_movement_score(self, metrics: dict) -> int:
        """
        Calculate a score 0-100 based on movement performance.
        
        Scoring breakdown:
        - Rise time: 25 points (optimal: < 200ms)
        - Settling time: 25 points (optimal: < 500ms)
        - Overshoot: 25 points (optimal: < 5%)
        - SSE: 25 points (optimal: < 0.1°)
        """
        score = 0
        
        # Rise time score (25 pts, linear decay 0-500ms)
        rise_ms = metrics.get("rise_time_ms", 0)
        if rise_ms <= 200:
            score += 25
        elif rise_ms < 500:
            score += int(25 * (500 - rise_ms) / 300)
        
        # Settling time score (25 pts, linear decay 0-1000ms)
        settle_ms = metrics.get("settling_time_ms", 0)
        if settle_ms <= 500:
            score += 25
        elif settle_ms < 1000:
            score += int(25 * (1000 - settle_ms) / 500)
        
        # Overshoot score (25 pts, linear decay 0-10%)
        overshoot = abs(metrics.get("overshoot_pct", 0))
        if overshoot <= 5:
            score += 25
        elif overshoot < 10:
            score += int(25 * (10 - overshoot) / 5)
        
        # SSE score (25 pts, linear decay 0-0.5°)
        sse = abs(metrics.get("sse_deg", 0))
        if sse <= 0.1:
            score += 25
        elif sse < 0.5:
            score += int(25 * (0.5 - sse) / 0.4)
        
        return min(100, max(0, score))

    # ------------------------------------------------------------------
    # Startup status handler (CAN 0x490 + joint_id)
    # ------------------------------------------------------------------
    def _handle_startup_status(self, data: bytes, timestamp: float, joint_id: int) -> None:
        """
        Decode startup status events from CAN (0x490-0x49F).

        Frame format:
            Byte 0: event_type (0=BEGIN, 1=DOF_READY, 2=DOF_FAILED, 3=COMPLETE, 4=FAILED)
            Byte 1: dof_index
            Byte 2: reason_code
            Byte 3-4: elapsed_ms (uint16_t LE)
        """
        event_type = data[0]
        dof_index = data[1]
        reason_code = data[2]
        elapsed_ms = struct.unpack_from("<H", data, 3)[0]

        event_names = {0: "BEGIN", 1: "DOF_READY", 2: "DOF_FAILED", 3: "COMPLETE", 4: "FAILED"}
        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id}")
        evt_name = event_names.get(event_type, f"UNKNOWN_{event_type}")
        reason_name = STARTUP_REASON_NAMES.get(reason_code, f"CODE_{reason_code}")

        payload = {
            "joint": joint_name,
            "joint_id": joint_id,
            "event": evt_name,
            "dof_index": dof_index,
            "reason": reason_name,
            "elapsed_ms": elapsed_ms,
            "timestamp": timestamp,
        }

        self.logger.info(
            f"Startup status [{joint_name}]: {evt_name} DOF={dof_index} "
            f"reason={reason_name} elapsed={elapsed_ms}ms"
        )

        with self._lock:
            self._status_messages.appendleft({"type": "startup_status", "data": payload})

        if self.socketio:
            try:
                self.socketio.emit("startup_status", payload, namespace="/movement")
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Joint announce handler (CAN 0x4A0 + joint_id)
    # ------------------------------------------------------------------
    def _handle_joint_announce(self, data: bytes, timestamp: float, joint_id: int) -> None:
        """
        Decode joint announce frames from CAN (0x4A0-0x4AF).

        Frame format:
            Byte 0: joint_id
            Byte 1: dof_count
            Byte 2: motor_count
            Byte 3: ready_flag (0x01 = ready for movement)
            Byte 4: fw_version_major
            Byte 5: fw_version_minor
            Byte 6: fw_version_patch
            Byte 7: clock_synced (0x01 = synced)
        """
        dof_count = data[1]
        motor_count = data[2]
        ready = bool(data[3])
        fw_version = f"{data[4]}.{data[5]}.{data[6]}"
        clock_synced = bool(data[7])

        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id}")

        payload = {
            "joint": joint_name,
            "joint_id": joint_id,
            "dof_count": dof_count,
            "motor_count": motor_count,
            "ready": ready,
            "fw_version": fw_version,
            "clock_synced": clock_synced,
            "timestamp": timestamp,
            "source": "can",
        }

        self.logger.info(
            f"Joint announce [{joint_name}] via CAN: {dof_count} DOFs, "
            f"{motor_count} motors, ready={ready}, fw={fw_version}, synced={clock_synced}"
        )

        with self._lock:
            # Store discovered joint info (can be queried via get_discovered_joints_can())
            if not hasattr(self, '_discovered_joints_can'):
                self._discovered_joints_can = {}
            self._discovered_joints_can[joint_id] = payload
            self._status_messages.appendleft({"type": "joint_announce", "data": payload})

        if self.socketio:
            try:
                self.socketio.emit("joint_announce", payload, namespace="/")
            except Exception:
                pass

    def get_discovered_joints_can(self) -> Dict[int, Dict[str, Any]]:
        """Return joints discovered via CAN announce frames."""
        with self._lock:
            if not hasattr(self, '_discovered_joints_can'):
                return {}
            return dict(self._discovered_joints_can)

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------
    def _log_can_sent(self, arbitration_id: int, data: bytes, context: Optional[str] = None) -> None:
        if not self.comm_logger:
            return
        hex_data = data.hex()
        suffix = f" | {context}" if context else ""
        self.comm_logger.log_sent_message(f"[CAN] TX id=0x{arbitration_id:03X} data={hex_data}{suffix}")

    def _log_can_received(self, arbitration_id: int, data: bytes, context: Optional[str] = None) -> None:
        if not self.comm_logger:
            return
        hex_data = data.hex()
        suffix = f" | {context}" if context else ""
        self.comm_logger.log_received_message(f"[CAN] RX id=0x{arbitration_id:03X} data={hex_data}{suffix}")

    def _log_can_info(self, message: str) -> None:
        if self.comm_logger:
            self.comm_logger.log_info(f"[CAN] {message}")

    def _log_can_error(self, message: str) -> None:
        if self.comm_logger:
            self.comm_logger.log_error(f"[CAN] {message}")
