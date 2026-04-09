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
  0x01F  FAULT_SNAPSHOT_CTRL

Feedback (controller → host):
  0x410+joint  Encoder Stream Data (50 Hz)
  0x490+joint  Startup Status
  0x4A0+joint  Joint Announce
  0x4F0+joint  Joint State (impedance feedback, 50 Hz)
  0x500+joint  Retension Probe Result
  0x510+joint  Health Status
  0x520+joint  Fault Status
  0x530+joint  Event Notice
  0x540+joint  Fault Snapshot Meta
  0x550+joint  Fault Snapshot Data
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
CAN_ID_LOOP_FREQUENCY = 0x006
CAN_ID_IDENTIFY_REQUEST = 0x008
CAN_ID_STARTUP_SEQUENCE = 0x009
CAN_ID_PRETENSION = 0x00C
CAN_ID_PRETENSION_ALL = 0x00D
CAN_ID_SET_IMPEDANCE = 0x01D
CAN_ID_IMPEDANCE_CTRL = 0x01E
CAN_ID_FAULT_SNAPSHOT_CTRL = 0x01F
CAN_ID_FW_UPDATE_CTRL = 0x020
CAN_ID_FW_UPDATE_META_A = 0x021
CAN_ID_FW_UPDATE_META_B = 0x022
CAN_ID_FW_UPDATE_DATA = 0x023

CAN_ID_ENCODER_STREAM_DATA = 0x410
CAN_ID_STARTUP_STATUS = 0x490
CAN_ID_JOINT_ANNOUNCE = 0x4A0
CAN_ID_JOINT_STATE = 0x4F0
CAN_ID_RETENSION_PROBE_RESULT = 0x500
CAN_ID_HEALTH_STATUS = 0x510
CAN_ID_FAULT_STATUS = 0x520
CAN_ID_EVENT_NOTICE = 0x530
CAN_ID_FAULT_SNAPSHOT_META = 0x540
CAN_ID_FAULT_SNAPSHOT_DATA = 0x550
CAN_ID_FW_UPDATE_STATUS = 0x560
CAN_ID_FW_UPDATE_UID = 0x570
CAN_ID_FW_UPDATE_INFO = 0x580
CAN_ID_FW_UPDATE_PROGRESS = 0x590

# Sentinel for unused DOF slots
UNUSED_DOF = 0x7FFF
FAULT_SNAPSHOT_UNUSED_I16 = 0x7FFF

# Inter-frame delay for multi-frame sequences (3 ms = ~1.5 control loops @ 500 Hz)
MULTI_FRAME_DELAY_S = 0.003
FAULT_SNAPSHOT_PENDING_TIMEOUT_S = 5.0

FAULT_SNAPSHOT_CTRL_SUBCMDS = {
    0x00: "QUERY_META",
    0x01: "BEGIN_DUMP",
    0x02: "REQUEST_CHUNK",
    0x03: "CLEAR_SNAPSHOT",
}

FAULT_SNAPSHOT_LAYOUT_NAMES = {
    1: "FIXED_V1",
}

FAULT_SNAPSHOT_DOF_STATE_NAMES = {
    0: "IDLE",
    1: "MOVING",
    2: "HOLDING",
}

DIAG_FAULT_NAMES = {
    0: "HOST_CAN_WARN",
    1: "MOTOR_CAN_WARN",
    2: "LOOP_OVERRUN",
    3: "HOST_WATCHDOG_TIMEOUT",
    4: "ENCODER_INVALID",
    5: "ENCODER_STALE",
    6: "MOTOR_TIMEOUT",
    7: "SAFETY_LIMIT",
    8: "MAPPING_LIMIT",
    9: "MOTOR_RANGE",
    10: "STARTUP_FAILED",
    11: "CONFIG_INVALID",
    12: "FLASH_ERROR",
    13: "BAD_COMMAND",
    14: "ESTOP_LATCHED",
    15: "INTERNAL_ERROR",
}

DIAG_EVENT_NAMES = {
    0x01: "BOOT_COMPLETE",
    0x02: "READY_ASSERTED",
    0x03: "READY_CLEARED",
    0x04: "STARTUP_BEGIN",
    0x05: "STARTUP_COMPLETE",
    0x06: "STARTUP_FAILED",
    0x07: "WATCHDOG_WARNING",
    0x08: "WATCHDOG_TIMEOUT",
    0x09: "ESTOP_ASSERTED",
    0x0A: "ESTOP_CLEARED",
    0x0B: "FAULT_SET",
    0x0C: "FAULT_CLEARED",
    0x0D: "ENCODER_INVALID",
    0x0E: "MOTOR_TIMEOUT",
    0x0F: "LOOP_OVERRUN_BURST",
    0x10: "SNAPSHOT_FROZEN",
    0x11: "SNAPSHOT_AVAILABLE",
}

DIAG_SOURCE_NAMES = {
    0: "GLOBAL",
    1: "DOF",
    2: "MOTOR",
    3: "HOST_CAN",
    4: "MOTOR_CAN",
    5: "STARTUP",
    6: "CONFIG",
    7: "SAFETY",
}

DIAG_PHASE_NAMES = {
    0: "BOOT",
    1: "SAFE_MODE_UNPROVISIONED",
    2: "IDLE_NOT_READY",
    3: "STARTUP_RUNNING",
    4: "READY",
    5: "LOCAL_HOLD",
    6: "FAULT_LOCKOUT",
    7: "SERVICE_ONLY",
}

DIAG_REBOOT_REASON_NAMES = {
    0: "POWER_ON",
    1: "SOFT_RESET_CMD",
    2: "WATCHDOG_RESET",
    3: "BROWNOUT_OR_POWER_DIP",
    4: "CAN_FAULT_RECOVERY",
    5: "FLASH_RECOVERY",
    6: "UNKNOWN",
    7: "RESERVED",
}

DIAG_HEALTH_EXT_LOOP_TIMING = 0x00
DIAG_HEALTH_EXT_CAN_DETAILS = 0x01

MCP2515_EFLG_NAMES = {
    0x80: "RX1OVR",
    0x40: "RX0OVR",
    0x20: "TXBO",
    0x10: "TXEP",
    0x08: "RXEP",
    0x04: "TXWAR",
    0x02: "RXWAR",
    0x01: "EWARN",
}

DIAG_SEVERITY_NAMES = {
    0: "INFO",
    1: "WARN",
    2: "ERROR",
    3: "CRITICAL",
}

STARTUP_REASON_NAMES = {
    0: "OK",
    1: "NO_CONTROLLER",
    2: "NO_EQUATIONS",
    3: "ENCODER_TIMEOUT",
    4: "POSITION_RANGE",
    5: "RECALC_ERROR",
    6: "GLOBAL_TIMEOUT",
    7: "PARTIAL_HOLD",
    8: "REFERENCE_REQUIRED",
}

FW_UPDATE_EVENT_NAMES = {
    0x01: "INFO_READY",
    0x02: "MAINTENANCE_ENTERED",
    0x03: "MAINTENANCE_EXITED",
    0x04: "BEGIN_ACCEPTED",
    0x05: "PAGE_COMMITTED",
    0x06: "VERIFY_OK",
    0x07: "ACTIVATE_OK",
    0x08: "CANDIDATE_BOOT_OK",
    0x09: "CONFIRM_OK",
    0x0A: "ROLLBACK_OCCURRED",
    0x40: "ERROR",
}

FW_UPDATE_ERROR_NAMES = {
    0x00: "NONE",
    0x01: "BUSY",
    0x02: "INVALID_STATE",
    0x03: "UID_MISMATCH",
    0x04: "INVALID_SLOT",
    0x05: "UPDATE_NOT_STARTED",
    0x06: "PAGE_SEQ_MISMATCH",
    0x07: "FRAG_INDEX_MISMATCH",
    0x08: "PAGE_CRC_MISMATCH",
    0x09: "SLOT_BOUNDS_ERROR",
    0x0A: "IMAGE_CRC_MISMATCH",
    0x0B: "VERIFY_FAILED",
    0x0C: "CONFIRMATION_TIMEOUT",
    0x0D: "CANDIDATE_BOOT_FAILED",
    0x0E: "CORE_TIMEOUT",
}

FW_UPDATE_BOOT_STATE_NAMES = {
    0: "STABLE",
    1: "MAINTENANCE",
    2: "RECEIVING",
    3: "VERIFIED",
    4: "PENDING_TEST",
    5: "CANDIDATE_RUNNING",
    6: "ROLLBACK_REQUIRED",
    7: "ROLLED_BACK",
}


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
    REASON_NAMES = STARTUP_REASON_NAMES

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
class FirmwareUpdateStatus:
    joint_id: int
    event_code: int
    boot_state: int
    active_slot: int
    pending_slot: int
    error_code: int
    flags: int
    value: int

    @property
    def event_name(self) -> str:
        return FW_UPDATE_EVENT_NAMES.get(self.event_code, f"EVENT_{self.event_code}")

    @property
    def boot_state_name(self) -> str:
        return FW_UPDATE_BOOT_STATE_NAMES.get(self.boot_state, f"BOOT_{self.boot_state}")

    @property
    def error_name(self) -> str:
        return FW_UPDATE_ERROR_NAMES.get(self.error_code, f"ERR_{self.error_code}")


@dataclass
class FirmwareUpdateInfo:
    joint_id: int
    active_slot: int
    pending_slot: int
    boot_state: int
    attempts_remaining: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    flags: int

    @property
    def fw_version(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_patch}"

    @property
    def maintenance_active(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def update_in_progress(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def candidate_awaiting_confirmation(self) -> bool:
        return bool(self.flags & 0x04)


@dataclass
class FirmwareUpdateProgress:
    joint_id: int
    next_page_index: int
    last_page_seq: int
    last_frag_index: int
    boot_state: int


@dataclass
class FirmwareUpdateUid:
    joint_id: int
    uid: bytes


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
class HealthStatusSummary:
    joint_id: int
    seq: int
    state_bits: int
    phase_code: int
    reboot_reason_code: int
    uptime_s: int
    fault_epoch: int

    @property
    def phase_name(self) -> str:
        return DIAG_PHASE_NAMES.get(self.phase_code, f"CODE_{self.phase_code}")

    @property
    def reboot_reason_name(self) -> str:
        return DIAG_REBOOT_REASON_NAMES.get(
            self.reboot_reason_code, f"CODE_{self.reboot_reason_code}"
        )

    @property
    def config_valid(self) -> bool:
        return bool(self.state_bits & 0x01)

    @property
    def controller_ready(self) -> bool:
        return bool(self.state_bits & 0x02)

    @property
    def motion_ready(self) -> bool:
        return bool(self.state_bits & 0x04)

    @property
    def motor_power_enabled(self) -> bool:
        return bool(self.state_bits & 0x08)

    @property
    def impedance_enabled(self) -> bool:
        return bool(self.state_bits & 0x10)

    @property
    def watchdog_armed(self) -> bool:
        return bool(self.state_bits & 0x20)

    @property
    def watchdog_warning(self) -> bool:
        return bool(self.state_bits & 0x40)

    @property
    def snapshot_available(self) -> bool:
        return bool(self.state_bits & 0x80)


@dataclass
class HealthStatusCounters:
    joint_id: int
    seq: int
    host_can_tx_error_count: int
    host_can_rx_error_count: int
    motor_can_tx_error_count: int
    loop_overrun_count: int
    watchdog_trip_count: int
    can_recovery_count: int


@dataclass
class HealthStatusCanDetails:
    joint_id: int
    seq: int
    host_can_tec: int
    host_can_rec: int
    host_can_eflg: int
    motor_can_tec: int
    motor_can_rec: int
    motor_can_eflg: int

    @property
    def host_can_eflg_names(self) -> list[str]:
        return decode_can_eflg(self.host_can_eflg)

    @property
    def motor_can_eflg_names(self) -> list[str]:
        return decode_can_eflg(self.motor_can_eflg)


def decode_can_eflg(value: int) -> list[str]:
    return [name for bit, name in MCP2515_EFLG_NAMES.items() if value & bit]


@dataclass
class FaultStatus:
    joint_id: int
    seq: int
    active_fault_bits: int
    latched_fault_bits: int
    primary_fault_code: Optional[int]
    source_id: int
    fault_epoch: int

    @property
    def active_fault_names(self) -> list[str]:
        return decode_fault_mask(self.active_fault_bits)

    @property
    def latched_fault_names(self) -> list[str]:
        return decode_fault_mask(self.latched_fault_bits)

    @property
    def primary_fault_name(self) -> Optional[str]:
        if self.primary_fault_code is None:
            return None
        return DIAG_FAULT_NAMES.get(self.primary_fault_code, f"CODE_{self.primary_fault_code}")

    @property
    def source_kind(self) -> str:
        return decode_source_id(self.source_id)["source_kind"]

    @property
    def source_name(self) -> str:
        return decode_source_id(self.source_id)["source"]

    @property
    def source_index(self) -> Optional[int]:
        return decode_source_id(self.source_id)["source_index"]


@dataclass
class EventNotice:
    joint_id: int
    event_seq: int
    event_code: int
    flags: int
    source_kind_code: int
    source_index: int
    detail0: int
    detail1: int

    @property
    def event_name(self) -> str:
        return DIAG_EVENT_NAMES.get(self.event_code, f"CODE_{self.event_code}")

    @property
    def severity_code(self) -> int:
        return self.flags & 0x03

    @property
    def severity_name(self) -> str:
        return DIAG_SEVERITY_NAMES.get(self.severity_code, f"CODE_{self.severity_code}")

    @property
    def is_assert(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def is_clear(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def latched_changed(self) -> bool:
        return bool(self.flags & 0x10)

    @property
    def snapshot_frozen(self) -> bool:
        return bool(self.flags & 0x20)

    @property
    def host_attention(self) -> bool:
        return bool(self.flags & 0x40)

    @property
    def source_kind(self) -> str:
        return DIAG_SOURCE_NAMES.get(self.source_kind_code, f"CODE_{self.source_kind_code}")

    @property
    def source_index_value(self) -> Optional[int]:
        return None if self.source_index == 0xFF else self.source_index


@dataclass
class FaultSnapshotMeta:
    joint_id: int
    snapshot_id: int
    freeze_event_code: int
    primary_fault_code: Optional[int]
    flags: int
    total_chunks: int
    payload_bytes: int
    seq: int

    @property
    def freeze_event_name(self) -> str:
        return DIAG_EVENT_NAMES.get(self.freeze_event_code, f"CODE_{self.freeze_event_code}")

    @property
    def primary_fault_name(self) -> Optional[str]:
        if self.primary_fault_code is None:
            return None
        return DIAG_FAULT_NAMES.get(self.primary_fault_code, f"CODE_{self.primary_fault_code}")

    @property
    def snapshot_present(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def frozen_on_critical_fault(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def truncated(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def dumped_once(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def checksum_available(self) -> bool:
        return bool(self.flags & 0x10)

    @property
    def fixed_layout_v1(self) -> bool:
        return bool(self.flags & 0x20)


@dataclass
class FaultSnapshotChunk:
    joint_id: int
    snapshot_id: int
    chunk_index: int
    payload: bytes


@dataclass
class EncoderData:
    joint_id: int
    angles_deg: list[Optional[float]]   # up to 3 DOFs, None = unused
    t_offset_ms: int


def decode_fault_mask(mask: int) -> list[str]:
    return [name for bit, name in DIAG_FAULT_NAMES.items() if mask & (1 << bit)]


def decode_source_id(source_id: int) -> dict[str, object]:
    if source_id <= 0x0F:
        return {"source": f"DOF_{source_id}", "source_kind": "DOF", "source_index": source_id}
    if 0x80 <= source_id <= 0x8F:
        return {
            "source": f"MOTOR_{source_id - 0x80}",
            "source_kind": "MOTOR",
            "source_index": source_id - 0x80,
        }
    if source_id == 0xE0:
        return {"source": "HOST_CAN", "source_kind": "HOST_CAN", "source_index": None}
    if source_id == 0xE1:
        return {"source": "MOTOR_CAN", "source_kind": "MOTOR_CAN", "source_index": None}
    return {"source": "GLOBAL", "source_kind": "GLOBAL", "source_index": None}


def _decode_snapshot_controller_flags(flags: int) -> dict[str, bool]:
    return {
        "motor_power_enabled": bool(flags & 0x01),
        "impedance_enabled": bool(flags & 0x02),
        "estop_latched": bool(flags & 0x04),
        "config_valid": bool(flags & 0x08),
        "controller_ready": bool(flags & 0x10),
        "frozen_on_critical_fault": bool(flags & 0x20),
    }


def _decode_snapshot_dof_flags(flags: int) -> dict[str, bool]:
    return {
        "q_valid": bool(flags & 0x01),
        "impedance_valid": bool(flags & 0x02),
        "watchdog_timed_out": bool(flags & 0x04),
        "holding": bool(flags & 0x08),
        "motor_angles_valid": bool(flags & 0x10),
        "direct_drive": bool(flags & 0x20),
    }


def _decode_optional_i16(value: int, *, scale: float = 1.0) -> Optional[float]:
    if value == FAULT_SNAPSHOT_UNUSED_I16:
        return None
    return value / scale


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


def encode_fault_snapshot_ctrl(
    joint_id: int,
    *,
    sub_cmd: int,
    snapshot_id: int = 0xFF,
    arg0: int = 0,
    arg1: int = 0,
    arg2: int = 0,
    arg3: int = 0,
    seq: int = 0,
) -> tuple[int, bytes]:
    """Encode FAULT_SNAPSHOT_CTRL (0x01F)."""
    payload = bytes(
        [
            sub_cmd & 0xFF,
            joint_id & 0xFF,
            snapshot_id & 0xFF,
            arg0 & 0xFF,
            arg1 & 0xFF,
            arg2 & 0xFF,
            arg3 & 0xFF,
            seq & 0xFF,
        ]
    )
    return CAN_ID_FAULT_SNAPSHOT_CTRL, payload


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


def encode_loop_frequency(inner_period_us: int, outer_divisor: int) -> tuple[int, bytes]:
    """Encode loop frequency update (0x006).

    byte 0-1: inner loop period in microseconds (uint16 LE)
    byte 2: outer loop divisor (uint8)
    """
    payload = struct.pack("<HB", inner_period_us, outer_divisor).ljust(8, b"\x00")
    return CAN_ID_LOOP_FREQUENCY, payload


def encode_impedance_ctrl(joint_id: int, sub_cmd: int,
                          param: int = 0) -> tuple[int, bytes]:
    """Encode IMPEDANCE_CTRL (0x01E).

    sub_cmd: 0x00=disable, 0x01=enable, 0x02=set_watchdog_ms
    """
    payload = struct.pack("<BBH", joint_id, sub_cmd, param)
    payload = payload.ljust(8, b'\x00')
    return CAN_ID_IMPEDANCE_CTRL, payload


# ---------------------------------------------------------------------------
# Firmware update frame builders (0x020-0x023)
# ---------------------------------------------------------------------------

FW_UPDATE_OP_GET_INFO = 0x01
FW_UPDATE_OP_ENTER_MAINTENANCE = 0x02
FW_UPDATE_OP_EXIT_MAINTENANCE = 0x03
FW_UPDATE_OP_BEGIN_UPDATE = 0x04
FW_UPDATE_OP_END_UPDATE = 0x05
FW_UPDATE_OP_VERIFY_UPDATE = 0x06
FW_UPDATE_OP_ACTIVATE_SLOT = 0x07
FW_UPDATE_OP_CONFIRM_UPDATE = 0x08
FW_UPDATE_OP_ABORT_UPDATE = 0x09
FW_UPDATE_OP_REBOOT = 0x0A

FW_UPDATE_EVT_INFO_READY = 0x01
FW_UPDATE_EVT_MAINTENANCE_ENTERED = 0x02
FW_UPDATE_EVT_MAINTENANCE_EXITED = 0x03
FW_UPDATE_EVT_BEGIN_ACCEPTED = 0x04
FW_UPDATE_EVT_PAGE_COMMITTED = 0x05
FW_UPDATE_EVT_VERIFY_OK = 0x06
FW_UPDATE_EVT_ACTIVATE_OK = 0x07
FW_UPDATE_EVT_CANDIDATE_BOOT_OK = 0x08
FW_UPDATE_EVT_CONFIRM_OK = 0x09
FW_UPDATE_EVT_ROLLBACK_OCCURRED = 0x0A
FW_UPDATE_EVT_ERROR = 0x40

FW_UPDATE_ERR_NONE = 0x00
FW_UPDATE_ERR_BUSY = 0x01
FW_UPDATE_ERR_INVALID_STATE = 0x02
FW_UPDATE_ERR_UID_MISMATCH = 0x03
FW_UPDATE_ERR_INVALID_SLOT = 0x04
FW_UPDATE_ERR_UPDATE_NOT_STARTED = 0x05
FW_UPDATE_ERR_PAGE_SEQ_MISMATCH = 0x06
FW_UPDATE_ERR_FRAG_INDEX_MISMATCH = 0x07
FW_UPDATE_ERR_PAGE_CRC_MISMATCH = 0x08
FW_UPDATE_ERR_SLOT_BOUNDS_ERROR = 0x09
FW_UPDATE_ERR_IMAGE_CRC_MISMATCH = 0x0A
FW_UPDATE_ERR_VERIFY_FAILED = 0x0B
FW_UPDATE_ERR_CONFIRMATION_TIMEOUT = 0x0C
FW_UPDATE_ERR_CANDIDATE_BOOT_FAILED = 0x0D
FW_UPDATE_ERR_CORE_TIMEOUT = 0x0E


def fw_update_crc16_ccitt(data: bytes) -> int:
    """Match firmware CRC16-CCITT implementation for per-page validation."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_fw_update_ctrl(opcode: int, *args: int) -> tuple[int, bytes]:
    payload = bytes([opcode & 0xFF, *(arg & 0xFF for arg in args[:7])]).ljust(8, b"\x00")
    return CAN_ID_FW_UPDATE_CTRL, payload


def encode_fw_update_get_info() -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_GET_INFO)


def encode_fw_update_enter_maintenance(*, require_idle: bool = True,
                                       reject_if_startup_active: bool = True,
                                       stay_after_reboot: bool = False) -> tuple[int, bytes]:
    flags = 0
    if require_idle:
        flags |= 0x01
    if reject_if_startup_active:
        flags |= 0x02
    if stay_after_reboot:
        flags |= 0x04
    return encode_fw_update_ctrl(FW_UPDATE_OP_ENTER_MAINTENANCE, flags)


def encode_fw_update_exit_maintenance() -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_EXIT_MAINTENANCE)


def encode_fw_update_begin(target_slot: int, *,
                           fw_major: int = 0,
                           fw_minor: int = 0,
                           fw_patch: int = 0,
                           image_format: int = 1,
                           allow_same_version: bool = True,
                           candidate_boots_in_maintenance: bool = True) -> tuple[int, bytes]:
    options = 0
    if allow_same_version:
        options |= 0x01
    if candidate_boots_in_maintenance:
        options |= 0x02
    return encode_fw_update_ctrl(
        FW_UPDATE_OP_BEGIN_UPDATE,
        target_slot,
        image_format,
        fw_major,
        fw_minor,
        fw_patch,
        options,
        0,
    )


def encode_fw_update_end() -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_END_UPDATE)


def encode_fw_update_verify(target_slot: int) -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_VERIFY_UPDATE, target_slot)


def encode_fw_update_activate(target_slot: int, attempts_remaining: int = 1) -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_ACTIVATE_SLOT, target_slot, attempts_remaining)


def encode_fw_update_confirm(target_slot: int) -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_CONFIRM_UPDATE, target_slot)


def encode_fw_update_abort() -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_ABORT_UPDATE)


def encode_fw_update_reboot(mode: int = 0) -> tuple[int, bytes]:
    return encode_fw_update_ctrl(FW_UPDATE_OP_REBOOT, mode)


def encode_fw_update_meta_a(image_size_bytes: int, image_crc32: int) -> tuple[int, bytes]:
    payload = struct.pack("<II", image_size_bytes & 0xFFFFFFFF, image_crc32 & 0xFFFFFFFF)
    return CAN_ID_FW_UPDATE_META_A, payload


def encode_fw_update_meta_b(board_uid_crc32: int,
                            protocol_major: int = 1,
                            protocol_minor: int = 0) -> tuple[int, bytes]:
    payload = struct.pack(
        "<IBBBB",
        board_uid_crc32 & 0xFFFFFFFF,
        protocol_major & 0xFF,
        protocol_minor & 0xFF,
        0,
        0,
    )
    return CAN_ID_FW_UPDATE_META_B, payload


def encode_fw_update_page_begin(page_index: int,
                                page_seq: int,
                                page_payload: bytes,
                                *,
                                is_final_page: bool) -> tuple[int, bytes]:
    page_len = len(page_payload)
    if page_len <= 0 or page_len > 256:
        raise ValueError("page_payload must contain between 1 and 256 bytes")
    page_crc16 = fw_update_crc16_ccitt(page_payload)
    length_mod = page_len % 256
    flags = 0x01 if is_final_page else 0x00
    payload = bytes(
        [
            page_index & 0xFF,
            (page_index >> 8) & 0xFF,
            (page_index >> 16) & 0xFF,
            page_seq & 0xFF,
            length_mod & 0xFF,
            flags,
        ]
    ) + struct.pack("<H", page_crc16)
    return CAN_ID_FW_UPDATE_META_B, payload


def encode_fw_update_page_frag(page_seq: int, frag_index: int, data: bytes) -> tuple[int, bytes]:
    if len(data) > 6:
        raise ValueError("page fragment payload cannot exceed 6 bytes")
    payload = bytes([page_seq & 0xFF, frag_index & 0xFF]) + data.ljust(6, b"\x00")
    return CAN_ID_FW_UPDATE_DATA, payload


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

def decode_fw_update_status(data: bytes, joint_id: int) -> FirmwareUpdateStatus:
    event_code, boot_state, active_slot, pending_slot, error_code, flags, value = struct.unpack(
        "<BBBBBBH", data
    )
    return FirmwareUpdateStatus(
        joint_id=joint_id,
        event_code=event_code,
        boot_state=boot_state,
        active_slot=active_slot,
        pending_slot=pending_slot,
        error_code=error_code,
        flags=flags,
        value=value,
    )


def decode_fw_update_uid(data: bytes, joint_id: int) -> FirmwareUpdateUid:
    return FirmwareUpdateUid(joint_id=joint_id, uid=bytes(data[:8]))


def decode_fw_update_info(data: bytes, joint_id: int) -> FirmwareUpdateInfo:
    return FirmwareUpdateInfo(
        joint_id=joint_id,
        active_slot=data[0],
        pending_slot=data[1],
        boot_state=data[2],
        attempts_remaining=data[3],
        fw_major=data[4],
        fw_minor=data[5],
        fw_patch=data[6],
        flags=data[7],
    )


def decode_fw_update_progress(data: bytes, joint_id: int) -> FirmwareUpdateProgress:
    next_page_index = data[0] | (data[1] << 8) | (data[2] << 16)
    return FirmwareUpdateProgress(
        joint_id=joint_id,
        next_page_index=next_page_index,
        last_page_seq=data[3],
        last_frag_index=data[4],
        boot_state=data[5],
    )


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


def decode_health_status_summary(data: bytes, joint_id: int) -> HealthStatusSummary:
    """Decode summary half of HEALTH_STATUS (0x510+joint, frame kind 0x00)."""
    return HealthStatusSummary(
        joint_id=joint_id,
        seq=data[7],
        state_bits=data[1],
        phase_code=data[2],
        reboot_reason_code=data[3],
        uptime_s=struct.unpack_from("<H", data, 4)[0],
        fault_epoch=data[6],
    )


def decode_health_status_counters(data: bytes, joint_id: int) -> HealthStatusCounters:
    """Decode counter half of HEALTH_STATUS (0x510+joint, frame kind 0x40)."""
    return HealthStatusCounters(
        joint_id=joint_id,
        seq=data[7],
        host_can_tx_error_count=data[1],
        host_can_rx_error_count=data[2],
        motor_can_tx_error_count=data[3],
        loop_overrun_count=data[4],
        watchdog_trip_count=data[5],
        can_recovery_count=data[6],
    )


def decode_health_status_can_details(data: bytes, joint_id: int) -> HealthStatusCanDetails:
    """Decode CAN-detail extension of HEALTH_STATUS (0x510+joint, frame kind 0x81)."""
    return HealthStatusCanDetails(
        joint_id=joint_id,
        seq=data[1],
        host_can_tec=data[2],
        host_can_rec=data[3],
        host_can_eflg=data[4],
        motor_can_tec=data[5],
        motor_can_rec=data[6],
        motor_can_eflg=data[7],
    )


def decode_fault_status(data: bytes, joint_id: int) -> FaultStatus:
    """Decode FAULT_STATUS (0x520+joint)."""
    seq, active_bits, latched_bits, primary_fault_code, source_id, fault_epoch = struct.unpack(
        "<BHHBBB", data[:8]
    )
    return FaultStatus(
        joint_id=joint_id,
        seq=seq,
        active_fault_bits=active_bits,
        latched_fault_bits=latched_bits,
        primary_fault_code=None if primary_fault_code == 0xFF else primary_fault_code,
        source_id=source_id,
        fault_epoch=fault_epoch,
    )


def decode_event_notice(data: bytes, joint_id: int) -> EventNotice:
    """Decode EVENT_NOTICE (0x530+joint)."""
    event_code, flags, source_kind, source_index, detail0, detail1, event_seq = struct.unpack(
        "<BBBBBBH", data[:8]
    )
    return EventNotice(
        joint_id=joint_id,
        event_seq=event_seq,
        event_code=event_code,
        flags=flags,
        source_kind_code=source_kind,
        source_index=source_index,
        detail0=detail0,
        detail1=detail1,
    )


def decode_fault_snapshot_meta(data: bytes, joint_id: int) -> FaultSnapshotMeta:
    """Decode FAULT_SNAPSHOT_META (0x540+joint)."""
    snapshot_id, freeze_event_code, primary_fault_code, flags, total_chunks, size_lo, size_hi, seq = struct.unpack(
        "<BBBBBBBB", data[:8]
    )
    payload_bytes = size_lo | (size_hi << 8)
    return FaultSnapshotMeta(
        joint_id=joint_id,
        snapshot_id=snapshot_id,
        freeze_event_code=freeze_event_code,
        primary_fault_code=None if primary_fault_code == 0xFF else primary_fault_code,
        flags=flags,
        total_chunks=total_chunks,
        payload_bytes=payload_bytes,
        seq=seq,
    )


def decode_fault_snapshot_chunk(data: bytes, joint_id: int) -> FaultSnapshotChunk:
    """Decode FAULT_SNAPSHOT_DATA (0x550+joint)."""
    return FaultSnapshotChunk(
        joint_id=joint_id,
        snapshot_id=data[0],
        chunk_index=data[1],
        payload=bytes(data[2:8]),
    )


def decode_fault_snapshot_blob(data: bytes) -> dict[str, object]:
    """Decode the fixed-layout v0.1 fault snapshot payload."""
    if len(data) < 20:
        raise ValueError(f"Fault snapshot too short: {len(data)} bytes")

    (
        layout_version,
        freeze_event_code,
        phase_code,
        primary_fault_code_raw,
        dof_count,
        motor_count,
        fault_epoch,
        snapshot_flags,
        active_fault_bits,
        latched_fault_bits,
        freeze_uptime_s,
        host_can_tx_error_count,
        host_can_rx_error_count,
        motor_can_tx_error_count,
        loop_overrun_count,
        watchdog_trip_count,
        can_recovery_count,
    ) = struct.unpack_from("<BBBBBBBBHHHBBBBBB", data, 0)

    offset = 20
    dof_records: list[dict[str, object]] = []
    for dof_index in range(dof_count):
        if len(data) < offset + 18:
            raise ValueError(
                f"Fault snapshot truncated in DOF record {dof_index}: {len(data)} bytes"
            )
        (
            q_x100,
            dq_x10,
            q_target_x100,
            hold_q_x100,
            stiffness_x10,
            motor_a_x100,
            motor_b_x100,
            tau_ff,
            state_code,
            flags,
        ) = struct.unpack_from("<hhhhhhhhBB", data, offset)
        offset += 18
        dof_records.append(
            {
                "dof_index": dof_index,
                "q_deg": _decode_optional_i16(q_x100, scale=100.0),
                "dq_deg_s": _decode_optional_i16(dq_x10, scale=10.0),
                "q_target_deg": _decode_optional_i16(q_target_x100, scale=100.0),
                "hold_q_deg": _decode_optional_i16(hold_q_x100, scale=100.0),
                "stiffness_deg": _decode_optional_i16(stiffness_x10, scale=10.0),
                "motor_agonist_angle_deg": _decode_optional_i16(motor_a_x100, scale=100.0),
                "motor_antagonist_angle_deg": _decode_optional_i16(motor_b_x100, scale=100.0),
                "tau_ff": None if tau_ff == FAULT_SNAPSHOT_UNUSED_I16 else int(tau_ff),
                "state_code": state_code,
                "state": FAULT_SNAPSHOT_DOF_STATE_NAMES.get(state_code, f"CODE_{state_code}"),
                "flags": _decode_snapshot_dof_flags(flags),
                "raw_flags": flags,
            }
        )

    primary_fault_code = None if primary_fault_code_raw == 0xFF else primary_fault_code_raw
    return {
        "layout_version": layout_version,
        "layout": FAULT_SNAPSHOT_LAYOUT_NAMES.get(layout_version, f"CODE_{layout_version}"),
        "freeze_event_code": freeze_event_code,
        "freeze_event": DIAG_EVENT_NAMES.get(freeze_event_code, f"CODE_{freeze_event_code}"),
        "phase_code": phase_code,
        "phase": DIAG_PHASE_NAMES.get(phase_code, f"CODE_{phase_code}"),
        "primary_fault_code": primary_fault_code,
        "primary_fault": None
        if primary_fault_code is None
        else DIAG_FAULT_NAMES.get(primary_fault_code, f"CODE_{primary_fault_code}"),
        "dof_count": dof_count,
        "motor_count": motor_count,
        "fault_epoch": fault_epoch,
        "snapshot_flags": _decode_snapshot_controller_flags(snapshot_flags),
        "snapshot_flags_raw": snapshot_flags,
        "active_fault_bits": active_fault_bits,
        "active_faults": decode_fault_mask(active_fault_bits),
        "latched_fault_bits": latched_fault_bits,
        "latched_faults": decode_fault_mask(latched_fault_bits),
        "freeze_uptime_s": freeze_uptime_s,
        "counters": {
            "host_can_tx_error_count": host_can_tx_error_count,
            "host_can_rx_error_count": host_can_rx_error_count,
            "motor_can_tx_error_count": motor_can_tx_error_count,
            "loop_overrun_count": loop_overrun_count,
            "watchdog_trip_count": watchdog_trip_count,
            "can_recovery_count": can_recovery_count,
        },
        "dofs": dof_records,
    }
