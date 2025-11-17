"""
CAN bus manager for host ↔ joint-controller communication.

Provides a high-level API for:
- Connecting/disconnecting to a python-can interface
- Sending protocol-defined frames (time sync, waypoint, emergency stop)
- Receiving status telemetry and exposing it to the Flask app/UI

CAN ID Allocation (Priority-Optimized):
- 0x000: Emergency Stop (Priority Level 0 - Highest)
- 0x002: Time Sync (Priority Level 1 - System)
- 0x140-0x280: Motor Commands (Priority Level 2 - CRITICAL @ 500 Hz)
- 0x300-0x31F: Waypoint Commands (Priority Level 3 - Trajectory @ 50-100 Hz)
- 0x400-0x4FF: Status Feedback (Priority Level 4 - Lowest @ 10-50 Hz)

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
        return {"timestamp_ms": timestamp_ms}

    def send_emergency_stop(self, reason_code: int = 0) -> Dict[str, Any]:
        """Broadcast emergency stop frame."""
        self._ensure_connection()
        payload = bytes([reason_code & 0xFF]) + bytes(7)
        self._send_frame(0x000, payload, context=f"E-Stop reason={reason_code}")
        return {"reason": reason_code}

    def send_waypoint(
        self,
        joint_name: str,
        dof_index: int,
        angle_deg: float,
        arrival_time_ms: int,
        mode: int = 0x01,
    ) -> Dict[str, Any]:
        """
        Send waypoint command to a specific joint/DOF.

        Args:
            joint_name: Host joint key (e.g., 'KNEE_LEFT')
            dof_index: Target DOF index within joint (0..N)
            angle_deg: Target angle in degrees
            arrival_time_ms: Absolute timeline when target should be reached
            mode: Trajectory mode (see CAN_CONTROL_PROTOCOL)
        """
        self._ensure_connection()

        joint_key = joint_name.upper()
        if joint_key not in JOINTS:
            raise ValueError(f"Unknown joint '{joint_name}'.")

        joint_info = JOINTS[joint_key]
        if dof_index < 0 or dof_index >= len(joint_info["dofs"]):
            raise ValueError(f"Invalid DOF index {dof_index} for joint {joint_key}.")

        joint_id = joint_info["id"]
        # NEW: 0x300 base for waypoints (Priority Level 3)
        # Motor commands (0x140-0x280) have higher priority for PID stability
        arbitration_id = 0x300 + joint_id

        # Convert to 0.01° resolution within signed int16
        angle_counts = int(round(angle_deg * 100))
        angle_counts = max(min(angle_counts, 32767), -32768)

        payload = struct.pack("<BhIB", dof_index & 0xFF, angle_counts, arrival_time_ms & 0xFFFFFFFF, mode & 0xFF)
        context = (
            f"Waypoint joint={joint_key} id={joint_id} dof={dof_index} angle={angle_deg:.2f}° "
            f"(counts={angle_counts}) arrival={arrival_time_ms} mode={mode}"
        )
        self._send_frame(arbitration_id, payload, context=context)

        return {
            "joint": joint_key,
            "dof_index": dof_index,
            "angle_deg": angle_deg,
            "angle_counts": angle_counts,
            "t_arrival_ms": arrival_time_ms,
            "mode": mode,
            "arbitration_id": f"0x{arbitration_id:03X}",
        }

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
        
        # If using SLCAN (serial interface), send ASCII command directly
        if self._current_config.get("interface") == "serial":
            try:
                import serial
                # Format: tiiildd...dd\r where iii=ID (3 hex), l=length (1 hex), dd=data (2 hex each)
                can_id_hex = f"{arbitration_id:03X}"
                length_hex = f"{len(data):01X}"
                data_hex = data.hex().upper()
                slcan_cmd = f"t{can_id_hex}{length_hex}{data_hex}\r"
                
                # Send directly to serial port
                ser = serial.Serial(self._current_config["channel"], baudrate=115200, timeout=0.1)
                ser.write(slcan_cmd.encode('ascii'))
                ser.close()
                
                self._stats["tx_frames"] += 1
                self._log_can_sent(arbitration_id, data, context)
                return
            except Exception as exc:
                err_msg = f"TX error id=0x{arbitration_id:03X}: {exc}"
                self.logger.log_error(f"[CAN] {err_msg}")
                raise RuntimeError(err_msg) from exc
        
        # Otherwise use python-can
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
        while not self._listener_stop.is_set():
            bus = self._bus
            if bus is None:
                time.sleep(0.1)
                continue
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
                continue

            self._stats["rx_frames"] += 1
            self._last_rx_timestamp = time.time()
            self._handle_message(message)

    def _handle_message(self, message: "can.Message") -> None:
        arb_id = message.arbitration_id
        data = bytes(message.data)

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

