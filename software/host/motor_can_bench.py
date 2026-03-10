"""
Utility helpers for single-motor CAN bench tests.

The goal is to keep CAN motor bench CSVs as close as possible to the
RS485 bench schema so the same analysis flow can be reused.
"""

from __future__ import annotations

import csv
import math
import re
import struct
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

DEFAULT_MOTOR_ID = 1
DEFAULT_OUTPUT_RATIO = 10.0
READ_COMMANDS = {
    "pid": 0x30,
    "acceleration": 0x33,
    "state2": 0x9C,
    "single": 0x94,
    "multi": 0x92,
}
CONTROL_COMMANDS = {
    "on": 0x88,
    "off": 0x80,
    "stop": 0x81,
    "torque": 0xA1,
    "speed": 0xA2,
}


def bytes_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def build_zero_frame(command: int) -> bytes:
    return bytes([command, 0, 0, 0, 0, 0, 0, 0])


def build_torque_frame(value: int) -> bytes:
    if not -2048 <= value <= 2048:
        raise ValueError("Torque value must be in [-2048, 2048]")
    frame = bytearray(build_zero_frame(CONTROL_COMMANDS["torque"]))
    frame[4:6] = struct.pack("<h", value)
    return bytes(frame)


def build_speed_frame(speed_dps: float) -> bytes:
    frame = bytearray(build_zero_frame(CONTROL_COMMANDS["speed"]))
    frame[4:8] = struct.pack("<i", int(round(speed_dps * 100.0)))
    return bytes(frame)


def build_read_frame(name: str) -> bytes:
    if name not in READ_COMMANDS:
        raise ValueError(f"Unsupported read command: {name}")
    return build_zero_frame(READ_COMMANDS[name])


def parse_i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def parse_u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def parse_u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def sign_extend_56(value: int) -> int:
    if value & (1 << 55):
        return value - (1 << 56)
    return value


def decode_feedback_frame(data: bytes, *, output_ratio: float = DEFAULT_OUTPUT_RATIO) -> dict[str, float | int]:
    if len(data) != 8:
        raise ValueError(f"Expected 8-byte CAN feedback frame, got {len(data)} bytes")
    iq_counts = parse_i16(data, 2)
    encoder_raw = parse_u16(data, 6)
    actuator_abs_deg = encoder_raw * 360.0 / 65536.0
    return {
        "temperature_c": struct.unpack_from("<b", data, 1)[0],
        "iq_counts": iq_counts,
        "iq_current_a_est": iq_counts * 33.0 / 2048.0,
        "speed_dps": parse_i16(data, 4),
        "encoder_raw": encoder_raw,
        "actuator_abs_deg": actuator_abs_deg,
        "motor_cmd_echo": data[0],
        "actuator_abs_from_feedback_single_deg": actuator_abs_deg,
        "motor_single_from_feedback_deg": actuator_abs_deg * output_ratio,
    }


def decode_single_frame(data: bytes, *, output_ratio: float = DEFAULT_OUTPUT_RATIO) -> dict[str, float | int]:
    if len(data) != 8 or data[0] != READ_COMMANDS["single"]:
        raise ValueError("Invalid single-angle response frame")
    angle_centideg = parse_u32(data, 4)
    return {
        "single_angle_centideg": angle_centideg,
        "single_angle_deg": angle_centideg / 100.0,
        "actuator_abs_from_single_deg": angle_centideg / (100.0 * output_ratio),
    }


def decode_multi_frame(data: bytes, *, output_ratio: float = DEFAULT_OUTPUT_RATIO) -> dict[str, float | int]:
    if len(data) != 8 or data[0] != READ_COMMANDS["multi"]:
        raise ValueError("Invalid multi-angle response frame")
    raw_56 = int.from_bytes(data[1:8], byteorder="little", signed=False)
    angle_centideg = sign_extend_56(raw_56)
    return {
        "multi_angle_centideg": angle_centideg,
        "multi_angle_deg": angle_centideg / 100.0,
        "actuator_cont_from_multi_deg": angle_centideg / (100.0 * output_ratio),
    }


def decode_pid_frame(data: bytes) -> dict[str, int | dict[str, int]]:
    if len(data) != 8 or data[0] != READ_COMMANDS["pid"]:
        raise ValueError("Invalid PID response frame")
    return {
        "position_pid_kp": data[2],
        "position_pid_ki": data[3],
        "speed_pid_kp": data[4],
        "speed_pid_ki": data[5],
        "torque_pid_kp": data[6],
        "torque_pid_ki": data[7],
        "position_pid": {"kp": data[2], "ki": data[3]},
        "speed_pid": {"kp": data[4], "ki": data[5]},
        "torque_pid": {"kp": data[6], "ki": data[7]},
    }


def decode_acceleration_frame(data: bytes) -> dict[str, int]:
    if len(data) != 8 or data[0] != READ_COMMANDS["acceleration"]:
        raise ValueError("Invalid acceleration response frame")
    return {
        "acceleration_dps2": struct.unpack_from("<i", data, 4)[0],
    }


def parse_extra_commands(raw: str | Iterable[str] | None) -> list[str]:
    if raw is None:
        return []
    if isinstance(raw, str):
        normalized = raw.strip().lower()
        if normalized in ("", "none"):
            return []
        items = [item.strip() for item in normalized.split(",") if item.strip()]
    else:
        items = [str(item).strip().lower() for item in raw if str(item).strip()]
    for item in items:
        if item not in ("single", "multi"):
            raise ValueError("Extra commands valid values: none, single, multi")
    return items


def generate_target(
    *,
    profile: str,
    t_s: float,
    duration_s: float,
    bias: float,
    amplitude: float,
    preload_s: float,
    frequency_hz: float,
    f0_hz: float,
    f1_hz: float,
) -> float:
    if profile == "hold":
        return bias + amplitude
    if profile == "step":
        return bias if t_s < preload_s else bias + amplitude
    if profile == "square":
        sign = 1.0 if math.sin(2.0 * math.pi * frequency_hz * t_s) >= 0.0 else -1.0
        return bias + amplitude * sign
    if profile == "chirp":
        k = (f1_hz - f0_hz) / max(duration_s, 1e-9)
        phase = 2.0 * math.pi * (f0_hz * t_s + 0.5 * k * t_s * t_s)
        return bias + amplitude * math.sin(phase)
    raise ValueError(f"Unsupported profile {profile}")


def build_control_frame(mode: str, target: float) -> bytes:
    if mode == "torque":
        return build_torque_frame(int(round(target)))
    if mode == "speed":
        return build_speed_frame(target)
    raise ValueError(f"Unsupported mode {mode}")


def csv_fieldnames(extra_commands: Iterable[str]) -> list[str]:
    fieldnames = [
        "motor_id",
        "output_ratio",
        "duration_s",
        "rate_hz",
        "timeout_s",
        "bias",
        "amplitude",
        "preload_s",
        "frequency_hz",
        "f0_hz",
        "f1_hz",
        "label",
        "t_s",
        "mode",
        "profile",
        "target",
        "temperature_c",
        "iq_counts",
        "iq_current_a_est",
        "speed_dps",
        "encoder_raw",
        "actuator_abs_deg",
        "motor_cmd_echo",
        "actuator_abs_from_feedback_single_deg",
        "motor_single_from_feedback_deg",
    ]
    extras = list(extra_commands)
    if "single" in extras:
        fieldnames.extend(["single_angle_centideg", "single_angle_deg", "actuator_abs_from_single_deg"])
    if "multi" in extras:
        fieldnames.extend(["multi_angle_centideg", "multi_angle_deg", "actuator_cont_from_multi_deg"])
    return fieldnames


def safe_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    return slug.strip("._") or "motor_can_test"


def build_default_label(
    *,
    motor_id: int,
    mode: str,
    profile: str,
    bias: float,
    amplitude: float,
    duration_s: float,
    rate_hz: float,
    preload_s: float,
    frequency_hz: float,
    f0_hz: float,
    f1_hz: float,
) -> str:
    return (
        f"motor{motor_id}_{mode}_{profile}"
        f"_bias{bias:g}_amp{amplitude:g}"
        f"_dur{duration_s:g}_rate{rate_hz:g}"
        f"_pre{preload_s:g}_sq{frequency_hz:g}"
        f"_f0{f0_hz:g}_f1{f1_hz:g}"
    )


def default_csv_path(log_dir: Path, *, mode: str, profile: str, label: str | None = None) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = safe_slug(label or f"{mode}_{profile}")
    return log_dir / f"{timestamp}_{stem}.csv"


def write_csv_rows(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _parse_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value not in ("", None) else 0.0


def analyze_csv_log(path: Path) -> dict[str, Any]:
    rows = load_csv_rows(path)
    if not rows:
        return {
            "row_count": 0,
            "summary": {},
            "metadata": {},
            "series": {"t_s": [], "target": [], "speed_dps": [], "iq_counts": [], "actuator_abs_deg": []},
        }

    first = rows[0]
    times = [_parse_float(row, "t_s") for row in rows]
    targets = [_parse_float(row, "target") for row in rows]
    speeds = [_parse_float(row, "speed_dps") for row in rows]
    iq_counts = [_parse_float(row, "iq_counts") for row in rows]
    actuator_abs_deg = [_parse_float(row, "actuator_abs_deg") for row in rows]
    dts = [curr - prev for prev, curr in zip(times, times[1:])]
    sample_rate_hz = (1.0 / (sum(dts) / len(dts))) if dts else 0.0

    metadata_keys = [
        "motor_id",
        "output_ratio",
        "duration_s",
        "rate_hz",
        "timeout_s",
        "bias",
        "amplitude",
        "preload_s",
        "frequency_hz",
        "f0_hz",
        "f1_hz",
        "label",
        "mode",
        "profile",
    ]
    metadata = {key: first.get(key) for key in metadata_keys if key in first}

    summary: dict[str, Any] = {
        "duration_s": times[-1] - times[0] if len(times) >= 2 else 0.0,
        "sample_rate_hz": sample_rate_hz,
        "target_min": min(targets),
        "target_max": max(targets),
        "speed_min": min(speeds),
        "speed_max": max(speeds),
        "speed_mean": sum(speeds) / len(speeds),
        "iq_min": min(iq_counts),
        "iq_max": max(iq_counts),
        "iq_mean": sum(iq_counts) / len(iq_counts),
    }

    profile = first.get("profile", "")
    if profile == "step":
        pre_indices = [idx for idx, t_s in enumerate(times) if t_s < 1.0]
        post_indices = [idx for idx, t_s in enumerate(times) if t_s >= 2.0]
        if pre_indices and post_indices:
            input_initial = sum(targets[idx] for idx in pre_indices) / len(pre_indices)
            input_final = sum(targets[idx] for idx in post_indices) / len(post_indices)
            output_initial = sum(speeds[idx] for idx in pre_indices) / len(pre_indices)
            output_final = sum(speeds[idx] for idx in post_indices) / len(post_indices)
            input_step = input_final - input_initial
            output_change = output_final - output_initial
            gain = (output_change / input_step) if input_step else 0.0
            summary.update({
                "input_initial": input_initial,
                "input_final": input_final,
                "input_step": input_step,
                "output_initial": output_initial,
                "output_final": output_final,
                "output_change": output_change,
                "gain_final_over_step": gain,
            })

    return {
        "row_count": len(rows),
        "metadata": metadata,
        "summary": summary,
        "series": {
            "t_s": times,
            "target": targets,
            "speed_dps": speeds,
            "iq_counts": iq_counts,
            "actuator_abs_deg": actuator_abs_deg,
        },
    }
