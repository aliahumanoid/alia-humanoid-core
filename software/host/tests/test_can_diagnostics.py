import struct
from types import SimpleNamespace

import can_manager as can_manager_module
from can_manager import CanManager
from diagnostic_history import DiagnosticHistoryWriter
from jetson_controller.protocol import CAN_ID_FAULT_SNAPSHOT_CTRL


def _build_manager(tmp_path, monkeypatch):
    if can_manager_module.can is None:
        monkeypatch.setattr(can_manager_module, "can", SimpleNamespace(BusABC=object))
    manager = CanManager()
    manager._diagnostic_history = DiagnosticHistoryWriter(tmp_path, source="test_suite")
    return manager


def _snapshot_frames():
    raw = struct.pack(
        "<BBBBBBBBHHHBBBBBB",
        1,      # layout version
        0x08,   # freeze event = WATCHDOG_TIMEOUT
        4,      # phase = READY
        3,      # primary fault = HOST_WATCHDOG_TIMEOUT
        1,      # dof_count
        2,      # motor_count
        9,      # fault_epoch
        0x3B,   # snapshot flags
        0x0009, # active fault bits
        0x0008, # latched fault bits
        123,    # freeze uptime
        1, 2, 3, 4, 5, 6,
    ) + struct.pack(
        "<hhhhhhhhBB",
        1234,   # q_deg = 12.34
        56,     # dq = 5.6
        1200,   # q_target = 12.0
        1180,   # hold_q = 11.8
        200,    # stiffness = 20.0
        1010,   # motor A = 10.1
        1020,   # motor B = 10.2
        15,     # tau_ff
        2,      # HOLDING
        0x1F,   # all flags except direct_drive
    )
    total_chunks = (len(raw) + 5) // 6
    meta = bytes([5, 0x08, 3, 0x23, total_chunks, len(raw) & 0xFF, (len(raw) >> 8) & 0xFF, 9])
    chunks = []
    for chunk_index in range(total_chunks):
        start = chunk_index * 6
        payload = raw[start:start + 6].ljust(6, b"\x00")
        chunks.append(bytes([5, chunk_index]) + payload)
    return meta, chunks, raw


def test_health_status_frames_are_combined_and_persisted(tmp_path, monkeypatch):
    manager = _build_manager(tmp_path, monkeypatch)

    summary = struct.pack("<BBBBHBB", 0x00, 0x6F, 4, 2, 123, 9, 7)
    counters = struct.pack("<BBBBBBBB", 0x40, 1, 2, 3, 4, 5, 6, 7)

    manager._handle_health_status(summary, 1000.0, joint_id=1)
    manager._handle_health_status(counters, 1000.1, joint_id=1)

    connection_state = manager.get_connection_state()
    payload = connection_state["diagnostics"]["health_status"]["KNEE_LEFT"]

    assert payload["seq"] == 7
    assert payload["phase"] == "READY"
    assert payload["reboot_reason"] == "WATCHDOG_RESET"
    assert payload["state"]["config_valid"] is True
    assert payload["state"]["watchdog_warning"] is True
    assert payload["host_can_tx_error_count"] == 1
    assert payload["watchdog_trip_count"] == 5
    assert connection_state["status_messages"][0]["type"] == "health_status"
    assert (tmp_path / "knee_left.jsonl").exists()


def test_fault_and_event_frames_are_decoded(tmp_path, monkeypatch):
    manager = _build_manager(tmp_path, monkeypatch)

    fault_frame = struct.pack("<BHHBBB", 3, 0x0009, 0x0008, 3, 0, 5)
    event_frame = struct.pack("<BBBBBBH", 0x08, 0x46, 1, 0, 12, 4, 33)

    manager._handle_fault_status(fault_frame, 2000.0, joint_id=1)
    manager._handle_event_notice(event_frame, 2000.1, joint_id=1)

    connection_state = manager.get_connection_state()
    fault_payload = connection_state["diagnostics"]["fault_status"]["KNEE_LEFT"]
    event_payload = connection_state["diagnostics"]["events"][0]

    assert fault_payload["active_faults"] == ["HOST_CAN_WARN", "HOST_WATCHDOG_TIMEOUT"]
    assert fault_payload["latched_faults"] == ["HOST_WATCHDOG_TIMEOUT"]
    assert fault_payload["primary_fault"] == "HOST_WATCHDOG_TIMEOUT"
    assert fault_payload["source"] == "DOF_0"

    assert event_payload["event"] == "WATCHDOG_TIMEOUT"
    assert event_payload["severity"] == "ERROR"
    assert event_payload["source_kind"] == "DOF"
    assert event_payload["source_index"] == 0
    assert event_payload["elapsed_10ms"] == 12
    assert event_payload["phase"] == "READY"


def test_fault_snapshot_frames_are_assembled_and_persisted(tmp_path, monkeypatch):
    manager = _build_manager(tmp_path, monkeypatch)
    meta_frame, chunk_frames, raw = _snapshot_frames()

    manager._handle_fault_snapshot_meta(meta_frame, 3000.0, joint_id=1)
    for index, frame in enumerate(chunk_frames):
        manager._handle_fault_snapshot_chunk(frame, 3000.1 + index * 0.01, joint_id=1)

    connection_state = manager.get_connection_state()
    meta_payload = connection_state["diagnostics"]["fault_snapshot_meta"]["KNEE_LEFT"]
    dump_payload = connection_state["diagnostics"]["fault_snapshots"]["KNEE_LEFT"]

    assert meta_payload["snapshot_id"] == 5
    assert meta_payload["freeze_event"] == "WATCHDOG_TIMEOUT"
    assert meta_payload["state"]["snapshot_present"] is True

    snapshot = dump_payload["snapshot"]
    assert dump_payload["payload_bytes"] == len(raw)
    assert snapshot["primary_fault"] == "HOST_WATCHDOG_TIMEOUT"
    assert snapshot["phase"] == "READY"
    assert snapshot["dofs"][0]["q_deg"] == 12.34
    assert snapshot["dofs"][0]["state"] == "HOLDING"
    assert snapshot["dofs"][0]["flags"]["motor_angles_valid"] is True
    assert (tmp_path / "knee_left.jsonl").exists()


def test_fault_snapshot_request_helpers_send_expected_frames(tmp_path, monkeypatch):
    manager = _build_manager(tmp_path, monkeypatch)
    sent = []

    monkeypatch.setattr(manager, "_ensure_connection", lambda: None)
    monkeypatch.setattr(manager, "_send_frame", lambda arb_id, data, context=None: sent.append((arb_id, data, context)))

    result = manager.begin_fault_snapshot_dump("KNEE_LEFT", first_chunk=2)

    assert result["sub_cmd"] == "BEGIN_DUMP"
    assert sent[0][0] == CAN_ID_FAULT_SNAPSHOT_CTRL
    assert sent[0][1] == bytes([0x01, 1, 0xFF, 2, 0, 0, 0, result["seq"]])


def test_fault_snapshot_pending_state_expires_after_timeout(tmp_path, monkeypatch):
    manager = _build_manager(tmp_path, monkeypatch)
    meta_frame, _, _ = _snapshot_frames()

    manager._handle_fault_snapshot_meta(meta_frame, 10.0, joint_id=1)
    assert 1 in manager._fault_snapshot_pending

    manager._expire_fault_snapshot_pending(16.0)

    assert 1 not in manager._fault_snapshot_pending


def test_fault_snapshot_chunks_accept_out_of_order_and_duplicate_delivery(tmp_path, monkeypatch):
    manager = _build_manager(tmp_path, monkeypatch)
    meta_frame, chunk_frames, raw = _snapshot_frames()

    manager._handle_fault_snapshot_meta(meta_frame, 3000.0, joint_id=1)
    manager._handle_fault_snapshot_chunk(chunk_frames[1], 3000.1, joint_id=1)
    manager._handle_fault_snapshot_chunk(chunk_frames[0], 3000.2, joint_id=1)
    manager._handle_fault_snapshot_chunk(chunk_frames[1], 3000.3, joint_id=1)

    connection_state = manager.get_connection_state()
    assert "KNEE_LEFT" not in connection_state["diagnostics"]["fault_snapshots"]

    for index, frame in enumerate(chunk_frames[2:], start=2):
        manager._handle_fault_snapshot_chunk(frame, 3000.3 + index * 0.01, joint_id=1)

    dump_payload = manager.get_connection_state()["diagnostics"]["fault_snapshots"]["KNEE_LEFT"]
    assert dump_payload["payload_bytes"] == len(raw)
    assert dump_payload["snapshot"]["primary_fault"] == "HOST_WATCHDOG_TIMEOUT"
