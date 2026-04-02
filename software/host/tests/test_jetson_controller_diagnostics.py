import struct
from pathlib import Path

from diagnostic_history import DiagnosticHistoryWriter
from jetson_controller.can_bus import _decode_rx
from jetson_controller.config import ControllerConfig, GainSet, JointControlConfig
from jetson_controller.protocol import (
    CAN_ID_EVENT_NOTICE,
    CAN_ID_FAULT_STATUS,
    CAN_ID_FAULT_SNAPSHOT_DATA,
    CAN_ID_FAULT_SNAPSHOT_META,
    CAN_ID_HEALTH_STATUS,
)
from jetson_controller.telemetry import TelemetryManager


def _build_config() -> ControllerConfig:
    joint = JointControlConfig(
        name="knee_left",
        joint_id=1,
        dof_count=1,
        dof_names=["flexion_extension"],
        dof_drive_types={0: "antagonistic_tendon"},
        dof_motor_counts={0: 2},
        dof_capabilities={0: {}},
        min_angles={0: 0.0},
        max_angles={0: 110.0},
        gains_outer=GainSet(kp=8.0, ki=1.0, kd=0.1),
        gains_inner=GainSet(kp=10.0, ki=1.0, kd=0.25),
        stiffness_deg=20.0,
        dof_gains_outer={},
        dof_gains_inner={},
        dof_stiffness_deg={},
        home_position_deg={0: 0.0},
    )
    return ControllerConfig(
        can_interface="slcan",
        can_channel="/dev/null",
        can_bitrate=1_000_000,
        startup_pretension_all=False,
        startup_recovery_settle_s=0.0,
        send_rate_hz=50,
        watchdog_ms=100,
        homing_speed_deg_s=10.0,
        homing_tolerance_deg=1.0,
        nudge_speed_deg_s=5.0,
        joints={"KNEE_LEFT": joint},
    )


def _snapshot_frames():
    raw = struct.pack(
        "<BBBBBBBBHHHBBBBBB",
        1,
        0x08,
        4,
        3,
        1,
        2,
        9,
        0x3B,
        0x0009,
        0x0008,
        123,
        1, 2, 3, 4, 5, 6,
    ) + struct.pack(
        "<hhhhhhhhBB",
        1234,
        56,
        1200,
        1180,
        200,
        1010,
        1020,
        15,
        2,
        0x1F,
    )
    total_chunks = (len(raw) + 5) // 6
    meta = bytes([5, 0x08, 3, 0x23, total_chunks, len(raw) & 0xFF, (len(raw) >> 8) & 0xFF, 9])
    chunks = []
    for chunk_index in range(total_chunks):
        start = chunk_index * 6
        payload = raw[start:start + 6].ljust(6, b"\x00")
        chunks.append(bytes([5, chunk_index]) + payload)
    return meta, chunks, raw


def test_jetson_telemetry_combines_health_status_and_persists(tmp_path: Path):
    telemetry = TelemetryManager(_build_config())
    telemetry._diagnostic_history = DiagnosticHistoryWriter(tmp_path, source="test_suite")

    summary = struct.pack("<BBBBHBB", 0x00, 0x6F, 4, 2, 123, 9, 7)
    counters = struct.pack("<BBBBBBBB", 0x40, 1, 2, 3, 4, 5, 6, 7)

    telemetry._dispatch(CAN_ID_HEALTH_STATUS + 1, summary, 1000.0)
    assert telemetry.states["KNEE_LEFT"].health_status is None

    telemetry._dispatch(CAN_ID_HEALTH_STATUS + 1, counters, 1000.1)
    payload = telemetry.states["KNEE_LEFT"].health_status

    assert payload is not None
    assert payload["seq"] == 7
    assert payload["phase"] == "READY"
    assert payload["reboot_reason"] == "WATCHDOG_RESET"
    assert payload["state"]["config_valid"] is True
    assert payload["state"]["watchdog_warning"] is True
    assert payload["watchdog_trip_count"] == 5
    assert (tmp_path / "knee_left.jsonl").exists()


def test_jetson_telemetry_decodes_faults_and_events(tmp_path: Path):
    telemetry = TelemetryManager(_build_config())
    telemetry._diagnostic_history = DiagnosticHistoryWriter(tmp_path, source="test_suite")

    fault_frame = struct.pack("<BHHBBB", 3, 0x0009, 0x0008, 3, 0, 5)
    event_frame = struct.pack("<BBBBBBH", 0x08, 0x46, 1, 0, 12, 4, 33)

    telemetry._dispatch(CAN_ID_FAULT_STATUS + 1, fault_frame, 2000.0)
    telemetry._dispatch(CAN_ID_EVENT_NOTICE + 1, event_frame, 2000.1)

    fault = telemetry.states["KNEE_LEFT"].fault_status
    event = telemetry.states["KNEE_LEFT"].diagnostic_events[0]

    assert fault is not None
    assert fault.active_fault_names == ["HOST_CAN_WARN", "HOST_WATCHDOG_TIMEOUT"]
    assert fault.latched_fault_names == ["HOST_WATCHDOG_TIMEOUT"]
    assert fault.primary_fault_name == "HOST_WATCHDOG_TIMEOUT"
    assert fault.source_name == "DOF_0"

    assert event.event_name == "WATCHDOG_TIMEOUT"
    assert event.severity_name == "ERROR"
    assert event.source_kind == "DOF"
    assert event.source_index_value == 0


def test_can_bus_logger_decodes_new_diagnostic_frames():
    summary = struct.pack("<BBBBHBB", 0x00, 0x6F, 4, 2, 123, 9, 7)
    fault_frame = struct.pack("<BHHBBB", 3, 0x0009, 0x0008, 3, 0, 5)
    event_frame = struct.pack("<BBBBBBH", 0x08, 0x46, 1, 0, 12, 4, 33)

    decoded, high_freq = _decode_rx(CAN_ID_HEALTH_STATUS + 1, summary)
    assert decoded.startswith("HEALTH_SUMMARY j=1")
    assert high_freq is False

    decoded, high_freq = _decode_rx(CAN_ID_FAULT_STATUS + 1, fault_frame)
    assert decoded.startswith("FAULT_STATUS j=1")
    assert "HOST_WATCHDOG_TIMEOUT" in decoded
    assert high_freq is False

    decoded, high_freq = _decode_rx(CAN_ID_EVENT_NOTICE + 1, event_frame)
    assert decoded.startswith("EVENT_NOTICE j=1")
    assert "WATCHDOG_TIMEOUT" in decoded
    assert high_freq is False


def test_jetson_telemetry_assembles_fault_snapshot_dump(tmp_path: Path):
    telemetry = TelemetryManager(_build_config())
    telemetry._diagnostic_history = DiagnosticHistoryWriter(tmp_path, source="test_suite")
    meta_frame, chunk_frames, raw = _snapshot_frames()

    telemetry._dispatch(CAN_ID_FAULT_SNAPSHOT_META + 1, meta_frame, 3000.0)
    for index, frame in enumerate(chunk_frames):
        telemetry._dispatch(CAN_ID_FAULT_SNAPSHOT_DATA + 1, frame, 3000.1 + index * 0.01)

    meta_payload = telemetry.states["KNEE_LEFT"].fault_snapshot_meta
    dump_payload = telemetry.states["KNEE_LEFT"].fault_snapshot_dump

    assert meta_payload is not None
    assert dump_payload is not None
    assert meta_payload["snapshot_id"] == 5
    assert meta_payload["state"]["snapshot_present"] is True
    assert dump_payload["payload_bytes"] == len(raw)
    assert dump_payload["snapshot"]["primary_fault"] == "HOST_WATCHDOG_TIMEOUT"
    assert dump_payload["snapshot"]["dofs"][0]["state"] == "HOLDING"


def test_can_bus_logger_decodes_fault_snapshot_frames():
    meta_frame, chunk_frames, _ = _snapshot_frames()

    decoded, high_freq = _decode_rx(CAN_ID_FAULT_SNAPSHOT_META + 1, meta_frame)
    assert decoded.startswith("FAULT_SNAPSHOT_META j=1")
    assert "present=True" in decoded
    assert high_freq is False

    decoded, high_freq = _decode_rx(CAN_ID_FAULT_SNAPSHOT_DATA + 1, chunk_frames[0])
    assert decoded.startswith("FAULT_SNAPSHOT_DATA j=1")
    assert "chunk=0" in decoded
    assert high_freq is False


def test_jetson_telemetry_expires_stale_snapshot_pending_state():
    telemetry = TelemetryManager(_build_config())
    meta_frame, _, _ = _snapshot_frames()

    telemetry._dispatch(CAN_ID_FAULT_SNAPSHOT_META + 1, meta_frame, 10.0)
    assert 1 in telemetry._fault_snapshot_pending

    telemetry._expire_fault_snapshot_pending(16.0)

    assert 1 not in telemetry._fault_snapshot_pending
