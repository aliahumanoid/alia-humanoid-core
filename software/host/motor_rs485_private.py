"""
Read-only helper for the vendor-private LK/LingKong RS485 setting pathway.

This module mirrors the proven Windows-side probe logic so the host UI can
inspect factory defaults directly from a local serial port when available.
"""
from __future__ import annotations

import copy
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List

import serial
from serial.tools import list_ports


HEAD = 0x3E
DEFAULT_RS485_PRIVATE_BAUD = 115200
DEFAULT_RS485_PRIVATE_TIMEOUT = 0.5
DEFAULT_RS485_PRIVATE_MOTOR_ID = 1
CMD_READ_SETTING = 0x14
CMD_WRITE_SETTING = 0x15
CMD_READ_CALIB = 0x16


@dataclass(frozen=True)
class FieldSpec:
    name: str
    fmt: str
    size: int
    align: int
    scale: float | None = None
    note: str | None = None


@dataclass(frozen=True)
class PrivateFrame:
    command: int
    motor_id: int
    data_len: int
    data: bytes
    raw: bytes
    cmd_checksum_ok: bool
    data_checksum_ok: bool


SETTING_FIELDS: list[FieldSpec] = [
    FieldSpec("driverId", "<B", 1, 1),
    FieldSpec("busType", "<B", 1, 1),
    FieldSpec("rs485BaudRate", "<B", 1, 1),
    FieldSpec("canBaudRate", "<B", 1, 1),
    FieldSpec("broadcastMode", "<B", 1, 1),
    FieldSpec("spinDirection", "<B", 1, 1),
    FieldSpec("protectMotorTempEnable", "<B", 1, 1),
    FieldSpec("protectDriverTempEnable", "<B", 1, 1),
    FieldSpec("protectUnderVoltageEnable", "<B", 1, 1),
    FieldSpec("protectOverVoltageEnable", "<B", 1, 1),
    FieldSpec("protectOverCurrentEnable", "<B", 1, 1),
    FieldSpec("protectShortCircuitEnable", "<B", 1, 1),
    FieldSpec("protectStallEnable", "<B", 1, 1),
    FieldSpec("protectLostInputEnable", "<B", 1, 1),
    FieldSpec("protectMotorTemp", "<B", 1, 1, note="likely degC"),
    FieldSpec("protectDriverTemp", "<B", 1, 1, note="likely degC"),
    FieldSpec("protectUnderVoltage", "<H", 2, 2, scale=100.0, note="likely volts"),
    FieldSpec("protectOverVoltage", "<H", 2, 2, scale=100.0, note="likely volts"),
    FieldSpec("protectOverCurrent", "<H", 2, 2, scale=100.0, note="likely amps"),
    FieldSpec("protectOverCurrentTime", "<H", 2, 2, note="likely ms"),
    FieldSpec("protectStallTime", "<H", 2, 2, note="likely ms"),
    FieldSpec("protectLostInputTime", "<H", 2, 2, note="likely ms"),
    FieldSpec("brakeResEnable", "<B", 1, 1),
    FieldSpec("brakeResOnVoltage", "<H", 2, 2, scale=100.0, note="likely volts"),
    FieldSpec("inputType", "<B", 1, 1),
    FieldSpec("pwmInputControlMode", "<B", 1, 1),
    FieldSpec("pwmInputMinValue", "<H", 2, 2),
    FieldSpec("pwmInputMaxValue", "<H", 2, 2),
    FieldSpec("pwmInputCenterValue", "<H", 2, 2),
    FieldSpec("pwmInputDeadband", "<H", 2, 2),
    FieldSpec("pwmToTorqueRatio", "<H", 2, 2),
    FieldSpec("pwmToSpeedRatio", "<H", 2, 2),
    FieldSpec("pwmToAngleRatio", "<H", 2, 2),
    FieldSpec("pulsesPerCircle", "<H", 2, 2),
    FieldSpec("anglePidKp", "<H", 2, 2),
    FieldSpec("anglePidKi", "<H", 2, 2),
    FieldSpec("anglePidKd", "<H", 2, 2),
    FieldSpec("speedPidKp", "<H", 2, 2),
    FieldSpec("speedPidKi", "<H", 2, 2),
    FieldSpec("speedPidKd", "<H", 2, 2),
    FieldSpec("currentPidKp", "<H", 2, 2),
    FieldSpec("currentPidKi", "<H", 2, 2),
    FieldSpec("currentPidKd", "<H", 2, 2),
    FieldSpec("maxTorque", "<h", 2, 2, note="iq counts"),
    FieldSpec("maxSpeed", "<i", 4, 4, scale=100.0, note="likely dps"),
    FieldSpec("maxAngle", "<q", 8, 8, scale=100.0, note="likely deg"),
    FieldSpec("currentRamp", "<h", 2, 2),
    FieldSpec("speedRamp", "<i", 4, 4),
    FieldSpec("uniqueId", "<I", 4, 4),
    FieldSpec("savedFlag", "<I", 4, 4),
]


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def bytes_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def hex_bytes(text: str) -> bytes:
    cleaned = (text or "").replace("\n", " ").replace("\r", " ").strip()
    if not cleaned:
        return b""
    return bytes(int(part, 16) for part in cleaned.split())


def align_offset(offset: int, align: int) -> int:
    if offset % align:
        offset += align - (offset % align)
    return offset


def compute_struct_layout(fields: list[FieldSpec], pack: int = 8) -> tuple[list[tuple[FieldSpec, int]], int]:
    layout: list[tuple[FieldSpec, int]] = []
    offset = 0
    max_align = 1
    for field in fields:
        effective_align = min(field.align, pack)
        max_align = max(max_align, effective_align)
        offset = align_offset(offset, effective_align)
        layout.append((field, offset))
        offset += field.size
    offset = align_offset(offset, max_align)
    return layout, offset


SETTING_LAYOUT, SETTING_SIZE = compute_struct_layout(SETTING_FIELDS)
SETTING_LAYOUT_BY_NAME = {field.name: (field, offset) for field, offset in SETTING_LAYOUT}


def build_frame(command: int, motor_id: int, payload: bytes = b"") -> bytes:
    if not 1 <= motor_id <= 32:
        raise ValueError(f"Motor ID must be between 1 and 32, got {motor_id}")
    header = bytes((HEAD, command, motor_id, len(payload)))
    packet = bytearray(header)
    packet.append(checksum(header))
    packet.extend(payload)
    if payload:
        packet.append(checksum(payload))
    return bytes(packet)


def open_serial(port: str, baud: int, timeout_s: float) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=timeout_s,
        write_timeout=timeout_s,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )


def read_matching_command_any_len(
    ser: serial.Serial,
    expected_command: int,
    expected_id: int,
    timeout_s: float,
) -> PrivateFrame:
    deadline = time.monotonic() + timeout_s
    buffer = bytearray()

    while time.monotonic() < deadline:
        if not buffer:
            chunk = ser.read(1)
            if chunk:
                buffer.extend(chunk)
            else:
                continue

        while True:
            head_index = buffer.find(bytes((HEAD,)))
            if head_index < 0:
                buffer.clear()
                break
            if head_index > 0:
                del buffer[:head_index]

            if len(buffer) < 5:
                more = ser.read(5 - len(buffer))
                if more:
                    buffer.extend(more)
                break

            command = buffer[1]
            motor_id = buffer[2]
            data_len = buffer[3]
            total_len = 5 + data_len + (1 if data_len else 0)

            if len(buffer) < total_len:
                more = ser.read(total_len - len(buffer))
                if more:
                    buffer.extend(more)
                if len(buffer) < total_len:
                    break

            raw = bytes(buffer[:total_len])
            del buffer[:total_len]

            cmd_checksum_ok = checksum(raw[:4]) == raw[4]
            data = raw[5:-1] if data_len else b""
            data_checksum_ok = checksum(data) == raw[-1] if data_len else True

            if command != expected_command or motor_id != expected_id:
                continue

            return PrivateFrame(
                command=command,
                motor_id=motor_id,
                data_len=data_len,
                data=data,
                raw=raw,
                cmd_checksum_ok=cmd_checksum_ok,
                data_checksum_ok=data_checksum_ok,
            )

        chunk = ser.read(1)
        if chunk:
            buffer.extend(chunk)

    raise TimeoutError(f"Timeout waiting for cmd=0x{expected_command:02X}, id={expected_id}")


def send_private_read(
    ser: serial.Serial,
    command: int,
    motor_id: int,
    timeout_s: float,
) -> tuple[bytes, PrivateFrame]:
    request = build_frame(command, motor_id)
    ser.reset_input_buffer()
    ser.write(request)
    ser.flush()
    frame = read_matching_command_any_len(ser, command, motor_id, timeout_s)
    return request, frame


def decode_setting_blob(data: bytes) -> Dict[str, Any]:
    decoded: Dict[str, Any] = {
        "_data_len": len(data),
        "_expected_len": SETTING_SIZE,
        "_raw_hex": bytes_hex(data),
    }
    for field, offset in SETTING_LAYOUT:
        if offset + field.size > len(data):
            decoded[f"{field.name}__missing"] = True
            continue
        value = struct.unpack_from(field.fmt, data, offset)[0]
        decoded[field.name] = value
        decoded[f"{field.name}__offset"] = offset
        if field.scale:
            decoded[f"{field.name}__scaled"] = value / field.scale
        if field.note:
            decoded[f"{field.name}__note"] = field.note

    decoded["_summary"] = {
        "angle_pid": {
            "kp": decoded.get("anglePidKp"),
            "ki": decoded.get("anglePidKi"),
            "kd": decoded.get("anglePidKd"),
        },
        "speed_pid": {
            "kp": decoded.get("speedPidKp"),
            "ki": decoded.get("speedPidKi"),
            "kd": decoded.get("speedPidKd"),
        },
        "current_pid": {
            "kp": decoded.get("currentPidKp"),
            "ki": decoded.get("currentPidKi"),
            "kd": decoded.get("currentPidKd"),
        },
        "max_torque_current_counts": decoded.get("maxTorque"),
        "current_ramp": decoded.get("currentRamp"),
        "speed_ramp": decoded.get("speedRamp"),
        "max_speed_dps": decoded.get("maxSpeed__scaled"),
        "max_angle_deg": decoded.get("maxAngle__scaled"),
    }
    return decoded


class MotorRs485PrivateReader:
    """Read/write helper for the vendor-private RS485 motor setting path."""

    def __init__(self, serial_manager=None) -> None:
        self.serial_manager = serial_manager
        self._lock = threading.Lock()
        self._recent: Dict[str, Dict[str, Any] | None] = {
            "setting": None,
            "calib": None,
            "last_write": None,
        }

    def list_available_ports(self) -> List[str]:
        if self.serial_manager is not None:
            try:
                ports = self.serial_manager.list_available_ports()
                if isinstance(ports, list):
                    return ports
            except Exception:
                pass
        return [port.device for port in list_ports.comports()]

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            recent = copy.deepcopy(self._recent)
        return {
            "available_ports": self.list_available_ports(),
            "motor_id": DEFAULT_RS485_PRIVATE_MOTOR_ID,
            "baud": DEFAULT_RS485_PRIVATE_BAUD,
            "recent": recent,
        }

    def read(
        self,
        action: str,
        *,
        port: str,
        motor_id: int = DEFAULT_RS485_PRIVATE_MOTOR_ID,
        baud: int = DEFAULT_RS485_PRIVATE_BAUD,
        timeout_s: float = DEFAULT_RS485_PRIVATE_TIMEOUT,
    ) -> Dict[str, Any]:
        if action == "setting":
            result = self._read_setting(port=port, motor_id=motor_id, baud=baud, timeout_s=timeout_s)
        elif action == "calib":
            result = self._read_calib(port=port, motor_id=motor_id, baud=baud, timeout_s=timeout_s)
        else:
            raise ValueError(f"Unsupported private RS485 read action: {action}")
        with self._lock:
            self._recent[action] = copy.deepcopy(result)
        return result

    def write_current_pid_ram(
        self,
        *,
        port: str,
        kp: int,
        ki: int,
        kd: int,
        base_raw_hex: str,
        motor_id: int = DEFAULT_RS485_PRIVATE_MOTOR_ID,
        baud: int = DEFAULT_RS485_PRIVATE_BAUD,
        timeout_s: float = DEFAULT_RS485_PRIVATE_TIMEOUT,
    ) -> Dict[str, Any]:
        base_payload = hex_bytes(base_raw_hex)
        if len(base_payload) != SETTING_SIZE:
            raise ValueError(
                f"base_raw_hex must decode to {SETTING_SIZE} bytes, got {len(base_payload)}"
            )

        for label, value in (("kp", kp), ("ki", ki), ("kd", kd)):
            if not 0 <= int(value) <= 0xFFFF:
                raise ValueError(f"Current PID {label} must be between 0 and 65535")

        patched = bytearray(base_payload)
        struct.pack_into("<H", patched, SETTING_LAYOUT_BY_NAME["currentPidKp"][1], int(kp))
        struct.pack_into("<H", patched, SETTING_LAYOUT_BY_NAME["currentPidKi"][1], int(ki))
        struct.pack_into("<H", patched, SETTING_LAYOUT_BY_NAME["currentPidKd"][1], int(kd))

        request = build_frame(CMD_WRITE_SETTING, motor_id, bytes(patched))
        with open_serial(port, baud, timeout_s) as ser:
            ser.reset_input_buffer()
            ser.write(request)
            ser.flush()
            time.sleep(min(0.10, max(0.02, timeout_s / 4.0)))

            ack_capture = bytearray()
            ack_deadline = time.monotonic() + min(0.15, timeout_s)
            while time.monotonic() < ack_deadline:
                waiting = ser.in_waiting
                if waiting:
                    ack_capture.extend(ser.read(waiting))
                else:
                    time.sleep(0.01)

            ser.reset_input_buffer()
            verify_request, verify_frame = send_private_read(ser, CMD_READ_SETTING, motor_id, timeout_s)

        verified = decode_setting_blob(verify_frame.data)
        applied = (
            verified.get("currentPidKp") == int(kp)
            and verified.get("currentPidKi") == int(ki)
            and verified.get("currentPidKd") == int(kd)
        )

        result = {
            "action": "write_setting_private_current_pid_ram",
            "command": f"0x{CMD_WRITE_SETTING:02X}",
            "motor_id": motor_id,
            "port": port,
            "baud": baud,
            "timeout_s": timeout_s,
            "tx_hex": bytes_hex(request),
            "ack_rx_hex": bytes_hex(bytes(ack_capture)) if ack_capture else None,
            "base_current_pid": {
                "kp": struct.unpack_from("<H", base_payload, SETTING_LAYOUT_BY_NAME["currentPidKp"][1])[0],
                "ki": struct.unpack_from("<H", base_payload, SETTING_LAYOUT_BY_NAME["currentPidKi"][1])[0],
                "kd": struct.unpack_from("<H", base_payload, SETTING_LAYOUT_BY_NAME["currentPidKd"][1])[0],
            },
            "requested_current_pid": {
                "kp": int(kp),
                "ki": int(ki),
                "kd": int(kd),
            },
            "verification": {
                "read_command": f"0x{CMD_READ_SETTING:02X}",
                "tx_hex": bytes_hex(verify_request),
                "rx_hex": bytes_hex(verify_frame.raw),
                "cmd_checksum_ok": verify_frame.cmd_checksum_ok,
                "data_checksum_ok": verify_frame.data_checksum_ok,
                "decoded": verified,
            },
            "applied": applied,
            "note": (
                "Only current PID was patched. On this firmware/unit, 0x15 write effects have been "
                "observed to persist across power-cycle, so this path must be treated as persistence-"
                "capable rather than volatile RAM-only."
            ),
            "timestamp": time.time(),
        }

        with self._lock:
            self._recent["setting"] = {
                "action": "read_setting_private",
                "command": f"0x{CMD_READ_SETTING:02X}",
                "motor_id": motor_id,
                "port": port,
                "baud": baud,
                "timeout_s": timeout_s,
                "tx_hex": bytes_hex(verify_request),
                "rx_hex": bytes_hex(verify_frame.raw),
                "cmd_checksum_ok": verify_frame.cmd_checksum_ok,
                "data_checksum_ok": verify_frame.data_checksum_ok,
                "decoded": copy.deepcopy(verified),
                "timestamp": time.time(),
            }
            self._recent["last_write"] = copy.deepcopy(result)

        return result

    def _read_setting(self, *, port: str, motor_id: int, baud: int, timeout_s: float) -> Dict[str, Any]:
        with open_serial(port, baud, timeout_s) as ser:
            request, frame = send_private_read(ser, CMD_READ_SETTING, motor_id, timeout_s)
        return {
            "action": "read_setting_private",
            "command": f"0x{CMD_READ_SETTING:02X}",
            "motor_id": motor_id,
            "port": port,
            "baud": baud,
            "timeout_s": timeout_s,
            "tx_hex": bytes_hex(request),
            "rx_hex": bytes_hex(frame.raw),
            "cmd_checksum_ok": frame.cmd_checksum_ok,
            "data_checksum_ok": frame.data_checksum_ok,
            "decoded": decode_setting_blob(frame.data),
            "timestamp": time.time(),
        }

    def _read_calib(self, *, port: str, motor_id: int, baud: int, timeout_s: float) -> Dict[str, Any]:
        with open_serial(port, baud, timeout_s) as ser:
            request, frame = send_private_read(ser, CMD_READ_CALIB, motor_id, timeout_s)
        return {
            "action": "read_calib_private",
            "command": f"0x{CMD_READ_CALIB:02X}",
            "motor_id": motor_id,
            "port": port,
            "baud": baud,
            "timeout_s": timeout_s,
            "tx_hex": bytes_hex(request),
            "rx_hex": bytes_hex(frame.raw),
            "cmd_checksum_ok": frame.cmd_checksum_ok,
            "data_checksum_ok": frame.data_checksum_ok,
            "data_len": frame.data_len,
            "data_hex": bytes_hex(frame.data),
            "note": "Raw calibration blob only. The private calibration layout is not yet fully decoded.",
            "timestamp": time.time(),
        }
