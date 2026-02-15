"""
CAN bus manager for host ↔ joint-controller communication.

Provides a high-level API for:
- Connecting/disconnecting to a python-can interface
- Sending protocol-defined frames (time sync, waypoint, emergency stop)
- Receiving status telemetry and exposing it to the Flask app/UI

CAN ID Allocation (Priority-Optimized):
Priority Level 0 (Emergency):
- 0x000: Emergency Stop (broadcast)
Priority Level 1 (System Control + Operational Commands):
- 0x002: Time Sync (broadcast)
- 0x003: Encoder Stream Control (start/stop)
- 0x004: PID Diagnostics Stream Control
- 0x005: Interpolation Mode (linear/smooth)
- 0x006: Loop Frequency Config (inner/outer)
- 0x007: PID Diagnostics Stream Frequency
- 0x008: Joint Identification Request (broadcast)
- 0x009: Startup Sequence (recalc + HOLDING)
- 0x00A: Get Encoder Offsets (query → response on 0x4B0+joint)
- 0x00B: Set Zero (→ notification on 0x4C0+joint)
- 0x00C: Pretension single DOF
- 0x00D: Pretension all DOFs
- 0x00E: Release single DOF
- 0x00F: Release all DOFs
- 0x010: Recalculate Motor Offsets
- 0x011: Save PID to Flash
- 0x012: Load PID from Flash
- 0x013: Set Inner PID Parameters (multi-frame, 4 seq)
- 0x014: Set Outer PID Parameters (multi-frame, 5 seq)
- 0x015: Cascade Speed Scaling (per-param + 0xFF apply)
- 0x016: Start Auto-Mapping (all DOFs)
- 0x017: Stop Auto-Mapping
- 0x018: Save Linear Equations to Flash
- 0x019: Load Linear Equations from Flash
- 0x01A: Set Auto-Start on Boot
Priority Level 2 (Motor Control - CRITICAL @ 500 Hz):
- 0x140-0x280: Motor Commands (Ctrl → Motors via LKM_Motor)
Priority Level 3 (Trajectory - 50-100 Hz):
- 0x380-0x39F: Multi-DOF Waypoint Commands (Host → Ctrl, per joint)
Priority Level 4 (Status Feedback - Lowest):
- 0x400+joint: Status/Feedback
- 0x410+joint: Encoder Stream Data (@ 50 Hz)
- 0x420+joint: PID Diagnostics - Target + Error
- 0x430+joint: PID Diagnostics - Torque A + B
- 0x440+joint*3+dof: Movement Metrics
- 0x460+joint*3+dof: Smoothness Metrics
- 0x470+joint: PID Inner Terms (P/I/D/FF, optional)
- 0x480+joint: PID Outer Terms (P/I/D/output, optional)
- 0x490+joint: Startup Status Events
- 0x4A0+joint: Joint Announce/Discovery
- 0x4B0+joint: Encoder Offsets Response
- 0x4C0+joint: Zero Complete Notification

Note: Motor commands have higher priority than waypoints to ensure PID loop stability.
"""
from __future__ import annotations

import json
import logging
import struct
import threading
import time
from collections import deque
from typing import Any, Dict, Optional

from config import JOINTS
from serial_logger import SerialLogger

try:
    import can  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    can = None  # type: ignore


class CanManager:
    """High-level helper that manages python-can Bus lifecycle and protocol helpers."""

    DEFAULT_BITRATE = 1_000_000  # 1 Mbps (maximum speed test)

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

        # Time sync tracking (for preflight gating)
        self._last_time_sync_ts: Optional[float] = None

        # Per-joint batch locks (prevents concurrent batch sends to same joint)
        self._batch_locks: Dict[str, threading.Lock] = {}

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

    def is_connected(self) -> bool:
        """Return True if CAN bus is initialized."""
        return self._bus is not None

    # ------------------------------------------------------------------
    # Protocol helpers
    # ------------------------------------------------------------------
    def send_time_sync(self, timestamp_ms: Optional[int] = None) -> Dict[str, Any]:
        """Broadcast absolute time reference to all controllers."""
        self._ensure_connection()
        if timestamp_ms is None:
            timestamp_ms = int(time.time() * 1000)
        # Ensure timestamp fits in uint32_t (0 to 4,294,967,295)
        timestamp_ms = timestamp_ms & 0xFFFFFFFF
        payload = struct.pack("<II", timestamp_ms, 0)
        self._send_frame(0x002, payload, context=f"TimeSync ts={timestamp_ms}")
        self._last_time_sync_ts = time.monotonic()
        return {"timestamp_ms": timestamp_ms}

    def last_time_sync_age_ms(self) -> Optional[float]:
        """Return ms since last time sync, or None if never synced."""
        if self._last_time_sync_ts is None:
            return None
        return (time.monotonic() - self._last_time_sync_ts) * 1000.0

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
        Start encoder streaming via CAN at 200Hz.

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

    def validate_encoder_offsets(self, joint_name: str) -> Dict[str, Any]:
        """Query firmware encoder offsets and validate against saved host copy.

        Returns:
            Dict with 'valid' (bool), 'details' (list of per-encoder results),
            and 'firmware_offsets' (dict of received offsets).
        """
        import os
        import json as _json

        firmware_offsets = self.query_encoder_offsets(joint_name)
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
            payload = struct.pack("<BBBBf", joint_id, dof_index, seq, 0, val)
            self._send_frame(0x014, payload,
                             context=f"SET_PID_OUTER seq={seq} val={val:.4f}")
        return {"success": True}

    def cascade_speed_scaling_via_can(self, joint_name: str,
                                       params: Dict[str, float]) -> Dict[str, Any]:
        """Send CASCADE_SPEED_SCALING parameters via CAN (0x015).

        Each parameter is sent as a separate frame: [joint_id, param_id, float32(4), 0, 0].
        A final frame with param_id=0xFF signals completion.

        Args:
            joint_name: Joint name
            params: Dict of parameter name → value. Known keys:
                    enabled, min, low, high, ema_en, ema_alpha,
                    tau_en, tau_high, tau_speed, jema_en, jema_alpha,
                    jema_speed, fric_en, fric_torque, fric_speed
        """
        self._ensure_connection()
        joint_id = JOINTS[joint_name]["id"]

        param_map = {
            "enabled": 0, "min": 1, "low": 2, "high": 3,
            "ema_en": 4, "ema_alpha": 5,
            "tau_en": 6, "tau_high": 7, "tau_speed": 8,
            "jema_en": 9, "jema_alpha": 10, "jema_speed": 11,
            "fric_en": 12, "fric_torque": 13, "fric_speed": 14,
        }

        for name, value in params.items():
            pid = param_map.get(name)
            if pid is not None:
                payload = struct.pack("<BBf", joint_id, pid, float(value)) + bytes(2)
                self._send_frame(0x015, payload,
                                 context=f"CASCADE param={name} val={value}")

        # Send apply marker
        payload = struct.pack("<BBf", joint_id, 0xFF, 0.0) + bytes(2)
        self._send_frame(0x015, payload, context="CASCADE apply")
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

    def set_interpolation_mode(self, mode: str) -> Dict[str, Any]:
        """
        Set waypoint interpolation mode.
        
        Args:
            mode: "linear" for step response (PID tuning) or "cosine" for smooth motion
        
        Sends control command (0x005) to set interpolation mode.
        """
        self._ensure_connection()
        mode_value = 0 if mode == "linear" else 1
        payload = bytes([mode_value]) + bytes(7)
        self._send_frame(0x005, payload, context=f"Interpolation mode={mode}")
        self._log_can_info(f"Interpolation mode set to: {mode}")
        return {"mode": mode}

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

    def set_reanchor_interval(self, interval: int) -> Dict[str, Any]:
        """
        Set waypoint re-anchor interval.

        Args:
            interval: Number of consumed WPs between re-anchors (0 = disabled)

        Sends control command (0x01B) to set re-anchor interval.
        """
        self._ensure_connection()
        interval = max(0, min(65535, interval))
        payload = struct.pack('<H', interval) + bytes(6)
        self._send_frame(0x01B, payload, context=f"WP re-anchor interval={interval}")
        self._log_can_info(f"WP re-anchor interval set to: {interval}" +
                           (" (disabled)" if interval == 0 else " WPs"))
        return {"interval": interval}

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

    def send_multi_dof_waypoint(
        self,
        joint_name: str,
        angles_deg: list,
        t_offset_ms: int,
    ) -> Dict[str, Any]:
        """
        Send Multi-DOF waypoint command to a joint (optimized format).
        
        This is the recommended format for production use, as it sends all DOFs
        of a joint in a single CAN frame, reducing bus traffic by 66%.
        
        Format (8 bytes):
            Byte 0-1: int16_t dof0_angle (0.01° resolution, 0x7FFF = unused)
            Byte 2-3: int16_t dof1_angle (0.01° resolution, 0x7FFF = unused)
            Byte 4-5: int16_t dof2_angle (0.01° resolution, 0x7FFF = unused)
            Byte 6-7: uint16_t t_offset_ms (offset from batch start, firmware-anchored)

        Args:
            joint_name: Host joint key (e.g., 'KNEE_LEFT')
            angles_deg: List of target angles [dof0, dof1, dof2] in degrees.
                        Use None for unused DOFs.
            t_offset_ms: Time offset from batch start in milliseconds (0-65535).
                        Firmware anchors to the local clock at the first WP
                        arrival and computes arrival = anchor + t_offset_ms.
        
        Returns:
            Dict with frame details for debugging.
        
        Example:
            # 3-DOF joint (ankle): all DOFs used, arrive in 1 second
            send_multi_dof_waypoint('ANKLE_RIGHT', [45.0, 10.0, -5.0], 1000)
            
            # 1-DOF joint (knee): only DOF0 used, arrive in 500ms
            send_multi_dof_waypoint('KNEE_RIGHT', [90.0, None, None], 500)
        """
        self._ensure_connection()

        joint_key = joint_name.upper()
        if joint_key not in JOINTS:
            raise ValueError(f"Unknown joint '{joint_name}'.")

        joint_info = JOINTS[joint_key]
        joint_id = joint_info["id"]
        
        # Multi-DOF waypoint uses 0x380 base
        arbitration_id = 0x380 + joint_id
        
        # Sentinel value for unused DOF
        UNUSED_DOF = 0x7FFF
        
        # Convert angles to 0.01° resolution, use sentinel for None/unused
        angle_counts = []
        for i in range(3):
            if i < len(angles_deg) and angles_deg[i] is not None:
                counts = int(round(angles_deg[i] * 100))
                counts = max(min(counts, 32767), -32768)
                angle_counts.append(counts)
            else:
                angle_counts.append(UNUSED_DOF)
        
        # Clamp t_offset to uint16 range
        t_offset_ms = max(0, min(t_offset_ms, 65535))
        
        # Pack payload: 3x int16 angles + 1x uint16 offset
        payload = struct.pack("<hhhH", 
                              angle_counts[0], 
                              angle_counts[1], 
                              angle_counts[2], 
                              t_offset_ms)
        
        # Build context string for logging
        angles_str = ", ".join(
            f"DOF{i}={angles_deg[i]:.2f}°" if i < len(angles_deg) and angles_deg[i] is not None else f"DOF{i}=unused"
            for i in range(3)
        )
        context = f"MultiDOF joint={joint_key} id={joint_id} {angles_str} t_offset={t_offset_ms}ms"
        
        self._send_frame(arbitration_id, payload, context=context)

        return {
            "joint": joint_key,
            "joint_id": joint_id,
            "angles_deg": angles_deg,
            "angle_counts": angle_counts,
            "t_offset_ms": t_offset_ms,
            "arbitration_id": f"0x{arbitration_id:03X}",
            "format": "multi_dof",
        }

    def send_waypoint_batch(
        self,
        joint_name: str,
        waypoints: list,
        inter_waypoint_delay_ms: float = 2.0,
        batch_id: str = "",
        min_lead_ms: float = 15.0,
    ) -> Dict[str, Any]:
        """
        Send a batch of waypoints sequentially (deterministic order).

        This ensures all waypoints arrive in order and none are lost.
        A small delay between waypoints prevents CAN buffer overflow.
        A per-joint lock prevents concurrent batches to the same joint.

        TIMING MODEL (batch-anchor):
        The t_offset_ms from JS represents "desired arrival time from batch
        start".  These offsets are sent to the firmware **unchanged**.
        The firmware anchors them: when the first waypoint of a batch
        arrives, the controller saves ``anchor_local_ms = millis()`` and
        computes every arrival time as ``anchor_local_ms + t_offset_ms``.
        This eliminates per-frame jitter caused by sampling millis() on
        each CAN receive — all inter-waypoint deltas are exact.

        LATE WAYPOINT POLICY:
        A waypoint is considered "late" when the time remaining before
        its intended arrival (``t_offset - elapsed``) drops below
        *min_lead_ms*.  Late waypoints are skipped (the firmware
        interpolates from the previous to the next waypoint).  If
        MAX_CONSECUTIVE_LATE consecutive waypoints are late, the batch
        is aborted — the system cannot keep up.

        Args:
            joint_name: Joint name (e.g., 'ANKLE_RIGHT')
            waypoints: List of dicts with 'angles_deg' and 't_offset_ms'
            inter_waypoint_delay_ms: Delay between waypoints (default 2ms)
            batch_id: Optional batch identifier for traceability
            min_lead_ms: Minimum remaining lead to consider on-time (default 15)

        Returns:
            Dict with batch statistics

        Raises:
            ValueError: If a batch is already in progress for this joint
        """
        import time
        from waypoint_types import MAX_CONSECUTIVE_LATE

        self._ensure_connection()

        # Per-joint concurrency lock — prevents interleaved batches
        joint_key = joint_name.upper()
        lock = self._batch_locks.setdefault(joint_key, threading.Lock())
        if not lock.acquire(timeout=0):
            raise ValueError(f"Waypoint batch already in progress for {joint_key}")

        try:
            success_count = 0
            failed_indices = []
            skipped_indices = []
            consecutive_late = 0
            total_late = 0
            aborted = False
            delay_sec = inter_waypoint_delay_ms / 1000.0
            max_timing_drift_ms = 0.0
            tag = f"[{batch_id}]" if batch_id else ""

            # Socket.IO progress throttle
            PROGRESS_INTERVAL = 50

            # Track elapsed time for late-waypoint detection
            batch_start_time = time.perf_counter()

            for i, wp in enumerate(waypoints):
                try:
                    angles = wp.get('angles_deg', [None, None, None])
                    original_t_offset = wp.get('t_offset_ms', 0)

                    # Elapsed time since batch start (ms)
                    elapsed_ms = (time.perf_counter() - batch_start_time) * 1000.0

                    # Remaining lead: how far in the future is this waypoint?
                    # The firmware anchors all offsets to the first-WP arrival
                    # time, so the effective lead shrinks as transmission
                    # progresses.
                    remaining_lead_ms = original_t_offset - elapsed_ms

                    # Track timing drift (informational)
                    offset_loss = max(0.0, -remaining_lead_ms)
                    if offset_loss > max_timing_drift_ms:
                        max_timing_drift_ms = offset_loss

                    # --- Late waypoint policy ---
                    # First waypoint (i==0) is always sent regardless of lead
                    if remaining_lead_ms < min_lead_ms and i > 0:
                        consecutive_late += 1
                        total_late += 1
                        if consecutive_late >= MAX_CONSECUTIVE_LATE:
                            self.logger.error(
                                f"Batch {tag} aborted: {consecutive_late} consecutive "
                                f"late waypoints, system cannot maintain timing"
                            )
                            aborted = True
                            break
                        self.logger.warning(
                            f"Waypoint {i} {tag} late: remaining_lead="
                            f"{remaining_lead_ms:.0f}ms < min_lead={min_lead_ms}ms, skipping"
                        )
                        skipped_indices.append(i)
                        continue  # Skip — firmware interpolates from prev to next

                    consecutive_late = 0  # Reset on successful send

                    # Send original t_offset — firmware uses batch-anchor
                    self.send_multi_dof_waypoint(
                        joint_name, angles, original_t_offset
                    )
                    success_count += 1

                    if batch_id:
                        self.logger.debug(
                            f"{tag}:{i} sent t_off={original_t_offset}ms "
                            f"(remaining_lead={remaining_lead_ms:.0f}ms)"
                        )

                    # Small delay to prevent CAN buffer overflow
                    if i < len(waypoints) - 1:
                        time.sleep(delay_sec)

                except Exception as exc:
                    self.logger.warning(f"Waypoint {i} {tag} failed: {exc}")
                    failed_indices.append(i)

                # --- Socket.IO progress (throttled) ---
                if (self.socketio and batch_id
                        and (i + 1) % PROGRESS_INTERVAL == 0):
                    try:
                        self.socketio.emit("batch_progress", {
                            "batch_id": batch_id,
                            "joint": joint_key,
                            "sent": success_count,
                            "total": len(waypoints),
                            "elapsed_ms": round(elapsed_ms, 1),
                        }, namespace="/movement")
                    except Exception:
                        pass  # Non-critical

            total_elapsed_ms = (time.perf_counter() - batch_start_time) * 1000.0
            status_str = "aborted" if aborted else "complete"
            self.logger.info(
                f"Waypoint batch {tag} {status_str}: {success_count}/{len(waypoints)} "
                f"sent in {total_elapsed_ms:.1f}ms "
                f"(late={total_late}, skipped={len(skipped_indices)})"
            )

            # --- Socket.IO batch complete event ---
            if self.socketio and batch_id:
                try:
                    self.socketio.emit("batch_complete", {
                        "batch_id": batch_id,
                        "joint": joint_key,
                        "sent": success_count,
                        "total": len(waypoints),
                        "elapsed_ms": round(total_elapsed_ms, 1),
                        "status": "success" if not aborted else "aborted",
                    }, namespace="/movement")
                except Exception:
                    pass

            return {
                "total": len(waypoints),
                "sent": success_count,
                "failed_indices": failed_indices,
                "skipped_indices": skipped_indices,
                "elapsed_ms": round(total_elapsed_ms, 1),
                "timing_drift_ms": round(max_timing_drift_ms, 1),
                "late_count": total_late,
                "aborted": aborted,
                "joint": joint_key,
                "batch_id": batch_id,
            }
        finally:
            lock.release()

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

        return {
            "connected": self.is_connected(),
            "config": config_copy,
            "last_status": status_copy,
            "last_rx_timestamp": last_rx,
            "stats": stats_copy,
            "status_messages": list(self._status_messages),
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
        reason_names = {0: "OK", 1: "NO_CONTROLLER", 2: "NO_EQUATIONS", 3: "ENCODER_TIMEOUT",
                        4: "POSITION_RANGE", 5: "RECALC_ERROR", 6: "GLOBAL_TIMEOUT"}

        joint_name = self._joint_id_lookup.get(joint_id, f"JOINT_{joint_id}")
        evt_name = event_names.get(event_type, f"UNKNOWN_{event_type}")
        reason_name = reason_names.get(reason_code, f"CODE_{reason_code}")

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

