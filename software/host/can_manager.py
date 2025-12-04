"""
CAN bus manager for host ↔ joint-controller communication.

Provides a high-level API for:
- Connecting/disconnecting to a python-can interface
- Sending protocol-defined frames (time sync, waypoint, emergency stop)
- Receiving status telemetry and exposing it to the Flask app/UI

CAN ID Allocation (Priority-Optimized):
- 0x000: Emergency Stop (Priority Level 0 - Highest)
- 0x002: Time Sync (Priority Level 1 - System)
- 0x003: Encoder Stream Control (start/stop)
- 0x140-0x280: Motor Commands (Priority Level 2 - CRITICAL @ 500 Hz)
- 0x380-0x39F: Multi-DOF Waypoint Commands (Priority Level 3 - @ 50-100 Hz)
- 0x400-0x4FF: Status Feedback (Priority Level 4 - Lowest @ 10-50 Hz)
- 0x410: Encoder Stream Data (Controller → Host @ 200 Hz)

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

    def start_encoder_stream(self) -> Dict[str, Any]:
        """
        Start encoder streaming via CAN at 200Hz.
        
        Sends control command (0x003) to enable high-frequency angle data.
        Data arrives on 0x410 and is decoded in _handle_encoder_stream().
        """
        self._ensure_connection()
        payload = bytes([0x01]) + bytes(7)  # 0x01 = start streaming
        self._send_frame(0x003, payload, context="Encoder stream START")
        self._encoder_stream_active = True
        self._encoder_stream_data.clear()
        self._log_can_info("Encoder streaming started @ 50Hz")
        return {"streaming": True}

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
            Byte 6-7: uint16_t t_offset_ms (offset from CURRENT time, like single-DOF)
        
        Args:
            joint_name: Host joint key (e.g., 'KNEE_LEFT')
            angles_deg: List of target angles [dof0, dof1, dof2] in degrees.
                        Use None for unused DOFs.
            t_offset_ms: Time offset from current time in milliseconds (0-65535).
                        Same semantics as single-DOF arrival_offset_ms.
        
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

        # Encoder stream data (0x410) - high-frequency, minimal logging
        if arb_id == 0x410 and len(data) >= 8:
            self._handle_encoder_stream(data, message.timestamp)
            return  # Don't log every frame
        
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

    def _handle_encoder_stream(self, data: bytes, timestamp: float) -> None:
        """
        Decode encoder stream data from CAN (0x410).
        
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
        
        # Build data point
        data_point = {
            "timestamp": timestamp,
            "t_ms": t_ms,
            "angles_deg": angles_deg,
        }
        
        # Buffer data
        with self._lock:
            self._encoder_stream_data.append(data_point)
            buffer_size = len(self._encoder_stream_data)
        
        # Debug log (throttled to 1Hz)
        self._stats["rx_frames"] += 1
        if self._stats["rx_frames"] % 50 == 0:  # Log every 50 frames (~1/sec at 50Hz)
            self._log_can_info(f"Encoder stream RX: {buffer_size} buffered, DOF0={angles_deg[0]:.2f}°" if angles_deg[0] else f"Encoder stream RX: {buffer_size} buffered")
        
        # Invoke callback if set (for real-time UI updates)
        if self._encoder_stream_callback:
            try:
                self._encoder_stream_callback(angles_deg, t_ms)
            except Exception:
                pass  # Don't let callback errors break streaming
        
        # Emit via SocketIO for real-time UI updates
        if self.socketio:
            try:
                self.socketio.emit("encoder_stream", data_point, namespace="/movement")
            except Exception:
                pass  # Don't let socket errors break streaming

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

