"""
CAN frame encoding/decoding for the Alia joint controller protocol.

Pure functions — no I/O, no state. All encoding matches can_manager.py
byte-for-byte for protocol compatibility.

CAN ID allocation:
  0x000  Emergency Stop (broadcast)
  0x002  Time Sync (broadcast)
  0x003  Encoder Stream Control
  0x008  Identify Request (broadcast)
  0x009  Startup Sequence
  0x01D  SET_IMPEDANCE (1-4 frames)
  0x01E  IMPEDANCE_CTRL

Feedback (controller → host):
  0x410+joint  Encoder Stream Data (50 Hz)
  0x490+joint  Startup Status
  0x4A0+joint  Joint Announce
  0x4F0+joint  Joint State (impedance feedback, 50 Hz)
  0x500+joint  Retension Probe Result
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

# ---------------------------------------------------------------------------
# CAN ID constants (mirrors host/config.py)
# ---------------------------------------------------------------------------
CAN_ID_EMERGENCY_STOP = 0x000
CAN_ID_TIME_SYNC = 0x002
CAN_ID_ENCODER_STREAM_CTRL = 0x003
CAN_ID_IDENTIFY_REQUEST = 0x008
CAN_ID_STARTUP_SEQUENCE = 0x009
CAN_ID_PRETENSION = 0x00C
CAN_ID_PRETENSION_ALL = 0x00D
CAN_ID_SET_IMPEDANCE = 0x01D
CAN_ID_IMPEDANCE_CTRL = 0x01E

CAN_ID_ENCODER_STREAM_DATA = 0x410
CAN_ID_STARTUP_STATUS = 0x490
CAN_ID_JOINT_ANNOUNCE = 0x4A0
CAN_ID_JOINT_STATE = 0x4F0
CAN_ID_RETENSION_PROBE_RESULT = 0x500

# Sentinel for unused DOF slots
UNUSED_DOF = 0x7FFF

# Inter-frame delay for multi-frame sequences (3 ms = ~1.5 control loops @ 500 Hz)
MULTI_FRAME_DELAY_S = 0.003


# ---------------------------------------------------------------------------
# Decoded message dataclasses
# ---------------------------------------------------------------------------
@dataclass
class JointAnnounce:
    joint_id: int
    dof_count: int
    motor_count: int
    ready: bool
    fw_version: str
    clock_synced: bool


@dataclass
class StartupStatus:
    joint_id: int
    event_type: int       # 0=BEGIN, 1=DOF_READY, 2=DOF_FAILED, 3=COMPLETE, 4=FAILED
    dof_index: int
    reason_code: int      # 0=OK, 1=NO_CONTROLLER, ..., 6=GLOBAL_TIMEOUT
    elapsed_ms: int

    EVENT_NAMES = {0: "BEGIN", 1: "DOF_READY", 2: "DOF_FAILED", 3: "COMPLETE", 4: "FAILED"}
    REASON_NAMES = {
        0: "OK", 1: "NO_CONTROLLER", 2: "NO_EQUATIONS", 3: "ENCODER_TIMEOUT",
        4: "POSITION_RANGE", 5: "RECALC_ERROR", 6: "GLOBAL_TIMEOUT",
        7: "PARTIAL_HOLD",
        8: "REFERENCE_REQUIRED",
    }

    @property
    def event_name(self) -> str:
        return self.EVENT_NAMES.get(self.event_type, f"UNKNOWN_{self.event_type}")

    @property
    def reason_name(self) -> str:
        return self.REASON_NAMES.get(self.reason_code, f"CODE_{self.reason_code}")

    @property
    def is_complete(self) -> bool:
        return self.event_type == 3

    @property
    def is_failed(self) -> bool:
        return self.event_type in (2, 4)

    @property
    def is_partial(self) -> bool:
        """COMPLETE but some DOFs stayed IDLE (encoder invalid at startup)."""
        return self.event_type == 3 and self.reason_code == 7


@dataclass
class JointState:
    """Impedance feedback broadcast (0x4F0+joint)."""
    joint_id: int
    dof_index: int
    q_actual_deg: float      # actual joint angle
    dq_actual_deg_s: float   # actual joint velocity
    tau_agonist: int          # agonist torque command (raw, ×4)
    tau_antagonist: int       # antagonist torque command (raw, ×4)
    valid: bool               # encoder/joint state valid
    holding: bool             # no active rolling segment
    watchdog_warning: bool    # >80% watchdog timeout elapsed


@dataclass
class RetensionProbeResult:
    joint_id: int
    dof_index: int
    q_deg: float
    baseline_stiffness_deg: float
    pre_ratio: float
    dur_ratio: float
    delta_ratio: float
    recruit_norm: float
    effort_pre: int
    probe_boost_deg: float
    probe_pulse_ms: int
    weak_side: str
    class_code: int
    classification: str
    min_samples: int


@dataclass
class EncoderData:
    joint_id: int
    angles_deg: list[Optional[float]]   # up to 3 DOFs, None = unused
    t_offset_ms: int


# ---------------------------------------------------------------------------
# Encode functions (host → firmware)
# ---------------------------------------------------------------------------

def encode_time_sync(timestamp_ms: Optional[int] = None) -> tuple[int, bytes]:
    """Encode time sync broadcast (0x002)."""
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000)
    timestamp_ms = timestamp_ms & 0xFFFFFFFF
    payload = struct.pack("<II", timestamp_ms, 0)
    return CAN_ID_TIME_SYNC, payload


def encode_emergency_stop(reason: int = 0) -> tuple[int, bytes]:
    """Encode emergency stop broadcast (0x000)."""
    payload = bytes([reason & 0xFF]) + bytes(7)
    return CAN_ID_EMERGENCY_STOP, payload


def encode_identify_request() -> tuple[int, bytes]:
    """Encode joint identification broadcast (0x008)."""
    return CAN_ID_IDENTIFY_REQUEST, bytes(8)


def encode_startup_sequence(joint_id: int, torque: int = 0,
                            duration: int = 0) -> tuple[int, bytes]:
    """Encode startup sequence command (0x009)."""
    payload = struct.pack("<BBhh", joint_id, 0, torque, duration) + bytes(2)
    return CAN_ID_STARTUP_SEQUENCE, payload


def encode_pretension_all(joint_id: int) -> tuple[int, bytes]:
    """Encode pretension-all command (0x00D).

    Re-enables motor power after an emergency stop and pretensions all DOFs.
    """
    payload = bytes([joint_id]) + bytes(7)
    return CAN_ID_PRETENSION_ALL, payload


def encode_encoder_stream_ctrl(start: bool) -> tuple[int, bytes]:
    """Encode encoder stream start/stop (0x003)."""
    payload = bytes([0x01 if start else 0x00]) + bytes(7)
    return CAN_ID_ENCODER_STREAM_CTRL, payload


def encode_impedance_ctrl(joint_id: int, sub_cmd: int,
                          param: int = 0) -> tuple[int, bytes]:
    """Encode IMPEDANCE_CTRL (0x01E).

    sub_cmd: 0x00=disable, 0x01=enable, 0x02=set_watchdog_ms
    """
    payload = struct.pack("<BBH", joint_id, sub_cmd, param)
    payload = payload.ljust(8, b'\x00')
    return CAN_ID_IMPEDANCE_CTRL, payload


# ---------------------------------------------------------------------------
# SET_IMPEDANCE frame builders (0x01D)
# ---------------------------------------------------------------------------

def _impedance_flags(seq: int, dof: int, has_more: bool) -> int:
    """Build byte 1: [has_more:1][seq:3][dof:4]."""
    return (int(has_more) << 7) | ((seq & 0x07) << 4) | (dof & 0x0F)


def encode_set_impedance_frame0(
    joint_id: int, dof: int,
    q_deg: float, dq_deg_s: float, stiffness_deg: float,
    has_more: bool = False,
) -> tuple[int, bytes]:
    """Frame 0 (always sent): position, velocity, stiffness."""
    q_int = int(round(q_deg * 100))
    dq_int = int(round(abs(dq_deg_s) * 10))
    stiff_int = int(round(stiffness_deg * 10))
    flags = _impedance_flags(0, dof, has_more)
    payload = struct.pack("<BBhhh", joint_id, flags, q_int, dq_int, stiff_int)
    payload = payload.ljust(8, b'\x00')
    return CAN_ID_SET_IMPEDANCE, payload


def encode_set_impedance_frame1(
    joint_id: int, dof: int,
    kp: float, ki: float, kd: float,
    has_more: bool = False,
) -> tuple[int, bytes]:
    """Frame 1 (optional): outer PID gains."""
    flags = _impedance_flags(1, dof, has_more)
    payload = struct.pack("<BBhhh", joint_id, flags,
                          int(round(kp * 100)),
                          int(round(ki * 100)),
                          int(round(kd * 100)))
    payload = payload.ljust(8, b'\x00')
    return CAN_ID_SET_IMPEDANCE, payload


def encode_set_impedance_frame2(
    joint_id: int, dof: int,
    kp_inner: float, ki_inner: float, kd_inner: float,
    has_more: bool = False,
) -> tuple[int, bytes]:
    """Frame 2 (optional): inner PID gains."""
    flags = _impedance_flags(2, dof, has_more)
    payload = struct.pack("<BBhhh", joint_id, flags,
                          int(round(kp_inner * 100)),
                          int(round(ki_inner * 100)),
                          int(round(kd_inner * 100)))
    payload = payload.ljust(8, b'\x00')
    return CAN_ID_SET_IMPEDANCE, payload


def encode_set_impedance_frame3(
    joint_id: int, dof: int,
    tau_ff: int,
) -> tuple[int, bytes]:
    """Frame 3 (optional): feedforward torque. Always last (has_more=0)."""
    flags = _impedance_flags(3, dof, False)
    payload = struct.pack("<BBh", joint_id, flags, int(round(tau_ff)))
    payload = payload.ljust(8, b'\x00')
    return CAN_ID_SET_IMPEDANCE, payload


# ---------------------------------------------------------------------------
# Decode functions (firmware → host)
# ---------------------------------------------------------------------------

def decode_joint_announce(data: bytes, joint_id: int) -> JointAnnounce:
    """Decode joint announce frame (0x4A0+joint)."""
    return JointAnnounce(
        joint_id=joint_id,
        dof_count=data[1],
        motor_count=data[2],
        ready=bool(data[3]),
        fw_version=f"{data[4]}.{data[5]}.{data[6]}",
        clock_synced=bool(data[7]),
    )


def decode_startup_status(data: bytes, joint_id: int) -> StartupStatus:
    """Decode startup status frame (0x490+joint)."""
    return StartupStatus(
        joint_id=joint_id,
        event_type=data[0],
        dof_index=data[1],
        reason_code=data[2],
        elapsed_ms=struct.unpack_from("<H", data, 3)[0],
    )


def decode_encoder_stream(data: bytes, joint_id: int) -> EncoderData:
    """Decode encoder stream frame (0x410+joint).

    Format: [dof0_int16, dof1_int16, dof2_int16, t_offset_uint16]
    Angles scaled ×100 (0.01° resolution). 0x7FFF = unused.
    """
    dof0_raw, dof1_raw, dof2_raw, t_ms = struct.unpack("<hhhH", data)
    angles = []
    for raw in (dof0_raw, dof1_raw, dof2_raw):
        angles.append(None if raw == UNUSED_DOF else raw / 100.0)
    return EncoderData(joint_id=joint_id, angles_deg=angles, t_offset_ms=t_ms)


def decode_joint_state(data: bytes, joint_id: int) -> JointState:
    """Decode joint state broadcast (0x4F0+joint).

    Format (8 bytes):
      byte 0:   uint8   dof_index
      byte 1-2: int16   q_actual × 100
      byte 3-4: int16   dq_actual × 10
      byte 5:   int8    tau_agonist ÷ 4
      byte 6:   int8    tau_antagonist ÷ 4
      byte 7:   uint8   status bits (bit0=valid, bit1=holding, bit2=watchdog)
    """
    dof = data[0]
    q_raw, dq_raw = struct.unpack_from("<hh", data, 1)
    tau_a = struct.unpack_from("<b", data, 5)[0]
    tau_b = struct.unpack_from("<b", data, 6)[0]
    status = data[7]
    return JointState(
        joint_id=joint_id,
        dof_index=dof,
        q_actual_deg=q_raw / 100.0,
        dq_actual_deg_s=dq_raw / 10.0,
        tau_agonist=tau_a * 4,
        tau_antagonist=tau_b * 4,
        valid=bool(status & 0x01),
        holding=bool(status & 0x02),
        watchdog_warning=bool(status & 0x04),
    )
