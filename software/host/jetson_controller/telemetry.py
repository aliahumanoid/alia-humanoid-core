"""
Telemetry manager: processes incoming CAN frames and tracks joint state.

Runs as an asyncio task, consuming messages from CanBus.recv() and
updating JointState objects that other modules can query.
"""
from __future__ import annotations

import asyncio
from collections import deque
import logging
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from diagnostic_history import DiagnosticHistoryWriter
from probe_history import ProbeHistoryWriter

from .can_bus import CanBus
from .config import ControllerConfig
from .protocol import (
    CAN_ID_ENCODER_STREAM_DATA,
    CAN_ID_EVENT_NOTICE,
    CAN_ID_FAULT_STATUS,
    CAN_ID_FAULT_SNAPSHOT_DATA,
    CAN_ID_FAULT_SNAPSHOT_META,
    CAN_ID_HEALTH_STATUS,
    CAN_ID_JOINT_ANNOUNCE,
    CAN_ID_JOINT_STATE,
    CAN_ID_RETENSION_PROBE_RESULT,
    CAN_ID_STARTUP_STATUS,
    DIAG_HEALTH_EXT_CAN_DETAILS,
    DIAG_HEALTH_EXT_LOOP_TIMING,
    DIAG_FAULT_NAMES,
    FAULT_SNAPSHOT_PENDING_TIMEOUT_S,
    DIAG_PHASE_NAMES,
    DIAG_REBOOT_REASON_NAMES,
    STARTUP_REASON_NAMES,
    EventNotice,
    FaultStatus,
    FaultSnapshotMeta,
    HealthStatusCanDetails,
    HealthStatusCounters,
    HealthStatusSummary,
    JointAnnounce,
    RetensionProbeResult,
    StartupStatus,
    decode_fault_snapshot_blob,
    decode_fault_snapshot_chunk,
    decode_fault_snapshot_meta,
    decode_event_notice,
    decode_encoder_stream,
    decode_fault_status,
    decode_health_status_can_details,
    decode_health_status_counters,
    decode_health_status_summary,
    decode_joint_announce,
    decode_joint_state,
    decode_startup_status,
)

logger = logging.getLogger(__name__)


@dataclass
class JointState:
    """Live state for a single joint, updated from CAN feedback."""
    angles_deg: dict[int, float] = field(default_factory=dict)
    velocities_deg_s: dict[int, float] = field(default_factory=dict)
    torques_agonist: dict[int, int] = field(default_factory=dict)
    torques_antagonist: dict[int, int] = field(default_factory=dict)
    holding: dict[int, bool] = field(default_factory=dict)
    probe_results: dict[int, RetensionProbeResult] = field(default_factory=dict)
    health_status: Optional[dict[str, object]] = None
    fault_status: Optional[FaultStatus] = None
    fault_snapshot_meta: Optional[dict[str, object]] = None
    fault_snapshot_dump: Optional[dict[str, object]] = None
    diagnostic_events: deque[EventNotice] = field(
        default_factory=lambda: deque(maxlen=32)
    )
    last_update: float = 0.0           # time.monotonic()
    is_online: bool = False
    announce: Optional[JointAnnounce] = None
    rx_count: int = 0


class TelemetryManager:
    """Processes CAN feedback and maintains per-joint state."""

    def __init__(self, config: ControllerConfig):
        self._config = config
        self.states: dict[str, JointState] = {}

        # Initialize state for each controlled joint
        for key, jcfg in config.joints.items():
            self.states[key] = JointState()

        # Events for FSM synchronization
        self.announce_events: dict[int, asyncio.Event] = {}  # joint_id → event
        self.startup_events: dict[int, asyncio.Event] = {}   # joint_id → event
        self.startup_results: dict[int, StartupStatus] = {}  # last status per joint
        self.unexpected_announces: dict[int, JointAnnounce] = {}

        # Set up events for expected joints
        for key, jcfg in config.joints.items():
            self.announce_events[jcfg.joint_id] = asyncio.Event()
            self.startup_events[jcfg.joint_id] = asyncio.Event()

        self._running = False
        self._retension_probe_pending: dict[tuple[int, int], dict[str, object]] = {}
        self._health_status_pending: dict[tuple[int, int], dict[str, object]] = {}
        self.health_status: dict[int, dict[str, object]] = {}
        self.fault_status: dict[int, FaultStatus] = {}
        self.fault_snapshot_meta: dict[int, FaultSnapshotMeta] = {}
        self.fault_snapshot_dumps: dict[int, dict[str, object]] = {}
        self.diagnostic_events: deque[EventNotice] = deque(maxlen=256)
        self._fault_snapshot_pending: dict[int, dict[str, object]] = {}
        self._probe_history = ProbeHistoryWriter(
            Path(__file__).resolve().parent.parent / "logs" / "probe_history",
            source="jetson_telemetry",
        )
        self._diagnostic_history = DiagnosticHistoryWriter(
            Path(__file__).resolve().parent.parent / "logs" / "diagnostic_history",
            source="jetson_telemetry",
        )

    async def listen(self, can_bus: CanBus) -> None:
        """Main telemetry loop — consume CAN messages and dispatch.

        Waits for CAN bus to be connected before attempting to receive.
        """
        self._running = True
        logger.info("Telemetry listener started")

        # Wait until CAN bus is connected
        while self._running and not can_bus.connected:
            await asyncio.sleep(0.1)

        if not self._running:
            return

        logger.info("Telemetry listener receiving")

        while self._running:
            msg = await can_bus.recv(timeout=0.5)
            if msg is None:
                continue
            self._dispatch(msg.arbitration_id, bytes(msg.data), msg.timestamp)

    def stop(self) -> None:
        self._running = False

    def _dispatch(self, arb_id: int, data: bytes, timestamp: float) -> None:
        """Route incoming CAN frame to appropriate handler."""
        self._expire_fault_snapshot_pending(timestamp)
        # Encoder stream (0x410-0x41F)
        if 0x410 <= arb_id <= 0x41F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_ENCODER_STREAM_DATA
            self._handle_encoder(data, joint_id)
            return

        # Startup status (0x490-0x49F)
        if 0x490 <= arb_id <= 0x49F and len(data) >= 5:
            joint_id = arb_id - CAN_ID_STARTUP_STATUS
            self._handle_startup(data, joint_id)
            return

        # Joint announce (0x4A0-0x4AF)
        if 0x4A0 <= arb_id <= 0x4AF and len(data) >= 8:
            joint_id = arb_id - CAN_ID_JOINT_ANNOUNCE
            self._handle_announce(data, joint_id)
            return

        # Joint state / impedance feedback (0x4F0-0x4FF)
        if 0x4F0 <= arb_id <= 0x4FF and len(data) >= 8:
            joint_id = arb_id - CAN_ID_JOINT_STATE
            self._handle_joint_state(data, joint_id)
            return

        # Retension probe result (0x500-0x50F)
        if CAN_ID_RETENSION_PROBE_RESULT <= arb_id <= CAN_ID_RETENSION_PROBE_RESULT + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_RETENSION_PROBE_RESULT
            self._handle_retension_probe(data, joint_id, timestamp)
            return

        # Health status summary/counters (0x510-0x51F)
        if CAN_ID_HEALTH_STATUS <= arb_id <= CAN_ID_HEALTH_STATUS + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_HEALTH_STATUS
            self._handle_health_status(data, joint_id, timestamp)
            return

        # Fault status (0x520-0x52F)
        if CAN_ID_FAULT_STATUS <= arb_id <= CAN_ID_FAULT_STATUS + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_FAULT_STATUS
            self._handle_fault_status(data, joint_id, timestamp)
            return

        # Event notices (0x530-0x53F)
        if CAN_ID_EVENT_NOTICE <= arb_id <= CAN_ID_EVENT_NOTICE + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_EVENT_NOTICE
            self._handle_event_notice(data, joint_id, timestamp)
            return

        # Fault snapshot metadata (0x540-0x54F)
        if CAN_ID_FAULT_SNAPSHOT_META <= arb_id <= CAN_ID_FAULT_SNAPSHOT_META + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_FAULT_SNAPSHOT_META
            self._handle_fault_snapshot_meta(data, joint_id, timestamp)
            return

        # Fault snapshot chunk data (0x550-0x55F)
        if CAN_ID_FAULT_SNAPSHOT_DATA <= arb_id <= CAN_ID_FAULT_SNAPSHOT_DATA + 0x0F and len(data) >= 8:
            joint_id = arb_id - CAN_ID_FAULT_SNAPSHOT_DATA
            self._handle_fault_snapshot_chunk(data, joint_id, timestamp)
            return

    def _handle_joint_state(self, data: bytes, joint_id: int) -> None:
        js = decode_joint_state(data, joint_id)
        key = self._config._id_to_key.get(joint_id)
        if key is None:
            return

        state = self.states.get(key)
        if state is None:
            return

        dof = js.dof_index
        state.angles_deg[dof] = js.q_actual_deg
        state.velocities_deg_s[dof] = js.dq_actual_deg_s
        state.torques_agonist[dof] = js.tau_agonist
        state.torques_antagonist[dof] = js.tau_antagonist
        state.holding[dof] = js.holding
        state.last_update = time.monotonic()
        state.is_online = True
        state.rx_count += 1

    def _handle_retension_probe(self, data: bytes, joint_id: int, timestamp: float) -> None:
        marker = data[0]
        frame_kind = marker & 0xC0
        dof = marker & 0x3F
        key = self._config._id_to_key.get(joint_id)
        if key is None:
            return

        pending_key = (joint_id, dof)
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
            self._retension_probe_pending[pending_key] = {
                "q_deg": q_x100 / 100.0,
                "baseline_stiffness_deg": base_stiff_x10 / 10.0,
                "pre_ratio": pre_ratio_x1000 / 1000.0,
                "weak_side": "B" if (flags & 0x01) else "A",
                "timestamp": timestamp,
            }
            return

        pending = self._retension_probe_pending.get(pending_key)
        if pending is None:
            return

        if frame_kind == 0x40:
            dur_ratio_x1000, delta_ratio_x1000, recruit_norm_x1000 = struct.unpack_from("<hhh", data, 1)
            class_code = data[7]
            pending["dur_ratio"] = dur_ratio_x1000 / 1000.0
            pending["delta_ratio"] = delta_ratio_x1000 / 1000.0
            pending["recruit_norm"] = recruit_norm_x1000 / 1000.0
            pending["class_code"] = class_code
            pending["classification"] = class_names.get(class_code, f"CODE_{class_code}")
            return

        if frame_kind != 0x80:
            return

        effort_pre, boost_x10, pulse_ms = struct.unpack_from("<HhH", data, 1)
        min_samples = data[7]
        result = RetensionProbeResult(
            joint_id=joint_id,
            dof_index=dof,
            q_deg=float(pending["q_deg"]),
            baseline_stiffness_deg=float(pending["baseline_stiffness_deg"]),
            pre_ratio=float(pending["pre_ratio"]),
            dur_ratio=float(pending.get("dur_ratio", -1.0)),
            delta_ratio=float(pending.get("delta_ratio", 0.0)),
            recruit_norm=float(pending.get("recruit_norm", 0.0)),
            effort_pre=int(effort_pre),
            probe_boost_deg=boost_x10 / 10.0,
            probe_pulse_ms=int(pulse_ms),
            weak_side=str(pending["weak_side"]),
            class_code=int(pending.get("class_code", 0)),
            classification=str(pending.get("classification", "UNKNOWN")),
            min_samples=int(min_samples),
        )

        state = self.states.get(key)
        if state is not None:
            state.probe_results[dof] = result

        self._probe_history.append(
            {
                "joint_id": joint_id,
                "joint_name": key,
                "dof": dof,
                "q_deg": result.q_deg,
                "baseline_stiffness_deg": result.baseline_stiffness_deg,
                "pre_ratio": result.pre_ratio,
                "dur_ratio": result.dur_ratio,
                "delta_ratio": result.delta_ratio,
                "recruit_norm": result.recruit_norm,
                "effort_pre": result.effort_pre,
                "probe_boost_deg": result.probe_boost_deg,
                "probe_pulse_ms": result.probe_pulse_ms,
                "weak_side": result.weak_side,
                "class_code": result.class_code,
                "classification": result.classification,
                "min_samples": result.min_samples,
            }
        )
        logger.info(
            f"RPROBE [{key}] DOF={dof} cls={result.classification} "
            f"q={result.q_deg:.1f} preR={result.pre_ratio:.3f} "
            f"dR={result.delta_ratio:.3f} dMinN={result.recruit_norm:.3f}"
        )
        self._retension_probe_pending.pop(pending_key, None)

    def _joint_state_for_id(self, joint_id: int) -> tuple[Optional[str], Optional[JointState]]:
        key = self._config._id_to_key.get(joint_id)
        if key is None:
            return None, None
        return key, self.states.get(key)

    def _record_diagnostic(self, payload: dict[str, object]) -> None:
        try:
            self._diagnostic_history.append(payload)
        except Exception:
            logger.warning("Failed to persist diagnostic history", exc_info=True)

    @staticmethod
    def _health_status_payload(
        joint_id: int,
        joint_name: str,
        summary: HealthStatusSummary,
        counters: HealthStatusCounters,
        timestamp: float,
        can_details: HealthStatusCanDetails | None = None,
        loop_timing: tuple[int, int, int] | None = None,
    ) -> dict[str, object]:
        payload = {
            "type": "health_status",
            "joint_id": joint_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "seq": summary.seq,
            "state_bits": summary.state_bits,
            "state": {
                "config_valid": summary.config_valid,
                "controller_ready": summary.controller_ready,
                "motion_ready": summary.motion_ready,
                "motor_power_enabled": summary.motor_power_enabled,
                "impedance_enabled": summary.impedance_enabled,
                "watchdog_armed": summary.watchdog_armed,
                "watchdog_warning": summary.watchdog_warning,
                "snapshot_available": summary.snapshot_available,
            },
            "phase_code": summary.phase_code,
            "phase": summary.phase_name,
            "reboot_reason_code": summary.reboot_reason_code,
            "reboot_reason": summary.reboot_reason_name,
            "uptime_s": summary.uptime_s,
            "fault_epoch": summary.fault_epoch,
            "host_can_tx_error_count": counters.host_can_tx_error_count,
            "host_can_rx_error_count": counters.host_can_rx_error_count,
            "motor_can_tx_error_count": counters.motor_can_tx_error_count,
            "loop_overrun_count": counters.loop_overrun_count,
            "watchdog_trip_count": counters.watchdog_trip_count,
            "can_recovery_count": counters.can_recovery_count,
        }
        if can_details is not None:
            payload["host_can_tec"] = can_details.host_can_tec
            payload["host_can_rec"] = can_details.host_can_rec
            payload["host_can_eflg"] = can_details.host_can_eflg
            payload["host_can_eflg_names"] = can_details.host_can_eflg_names
            payload["motor_can_tec"] = can_details.motor_can_tec
            payload["motor_can_rec"] = can_details.motor_can_rec
            payload["motor_can_eflg"] = can_details.motor_can_eflg
            payload["motor_can_eflg_names"] = can_details.motor_can_eflg_names
        if loop_timing is not None:
            payload["loop_avg_us"] = loop_timing[0]
            payload["loop_max_us"] = loop_timing[1]
            payload["loop_budget_us"] = loop_timing[2]
        return payload

    @staticmethod
    def _health_loop_timing_payload(
        joint_id: int,
        joint_name: str,
        seq: int,
        avg_us: int,
        max_us: int,
        budget_us: int,
        timestamp: float,
    ) -> dict[str, object]:
        return {
            "type": "health_loop_timing",
            "joint_id": joint_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "seq": seq,
            "loop_avg_us": avg_us,
            "loop_max_us": max_us,
            "loop_budget_us": budget_us,
        }

    @staticmethod
    def _health_can_detail_payload(
        joint_id: int,
        joint_name: str,
        details: HealthStatusCanDetails,
        timestamp: float,
    ) -> dict[str, object]:
        return {
            "type": "health_can_details",
            "joint_id": joint_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "seq": details.seq,
            "host_can_tec": details.host_can_tec,
            "host_can_rec": details.host_can_rec,
            "host_can_eflg": details.host_can_eflg,
            "host_can_eflg_names": details.host_can_eflg_names,
            "motor_can_tec": details.motor_can_tec,
            "motor_can_rec": details.motor_can_rec,
            "motor_can_eflg": details.motor_can_eflg,
            "motor_can_eflg_names": details.motor_can_eflg_names,
        }

    @staticmethod
    def _fault_status_payload(
        joint_name: str,
        fault: FaultStatus,
        timestamp: float,
    ) -> dict[str, object]:
        return {
            "type": "fault_status",
            "joint_id": fault.joint_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "seq": fault.seq,
            "active_fault_bits": fault.active_fault_bits,
            "latched_fault_bits": fault.latched_fault_bits,
            "active_faults": fault.active_fault_names,
            "latched_faults": fault.latched_fault_names,
            "primary_fault_code": fault.primary_fault_code,
            "primary_fault": fault.primary_fault_name,
            "source_id": fault.source_id,
            "source": fault.source_name,
            "source_kind": fault.source_kind,
            "source_index": fault.source_index,
            "fault_epoch": fault.fault_epoch,
        }

    @staticmethod
    def _event_notice_payload(
        joint_name: str,
        event: EventNotice,
        timestamp: float,
    ) -> dict[str, object]:
        payload: dict[str, object] = {
            "type": "event_notice",
            "joint_id": event.joint_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "event_seq": event.event_seq,
            "event_code": event.event_code,
            "event": event.event_name,
            "flags": event.flags,
            "severity_code": event.severity_code,
            "severity": event.severity_name,
            "is_assert": event.is_assert,
            "is_clear": event.is_clear,
            "latched_changed": event.latched_changed,
            "snapshot_frozen": event.snapshot_frozen,
            "host_attention": event.host_attention,
            "source_kind_code": event.source_kind_code,
            "source_kind": event.source_kind,
            "source_index": event.source_index_value,
            "detail0": event.detail0,
            "detail1": event.detail1,
        }

        if event.event_code == 0x01:
            payload["reboot_reason_code"] = event.detail0
            payload["reboot_reason"] = DIAG_REBOOT_REASON_NAMES.get(
                event.detail0, f"CODE_{event.detail0}"
            )
            payload["phase_code"] = event.detail1
            payload["phase"] = DIAG_PHASE_NAMES.get(event.detail1, f"CODE_{event.detail1}")
        elif event.event_code in {0x02, 0x03}:
            payload["previous_phase_code"] = event.detail0
            payload["previous_phase"] = DIAG_PHASE_NAMES.get(
                event.detail0, f"CODE_{event.detail0}"
            )
            payload["phase_code"] = event.detail1
            payload["phase"] = DIAG_PHASE_NAMES.get(event.detail1, f"CODE_{event.detail1}")
        elif event.event_code in {0x05, 0x06}:
            payload["startup_reason_code"] = event.detail0
            payload["startup_reason"] = STARTUP_REASON_NAMES.get(
                event.detail0, f"CODE_{event.detail0}"
            )
            payload["elapsed_ms_approx"] = event.detail1
        elif event.event_code in {0x04, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11}:
            payload["phase_code"] = event.detail1
            payload["phase"] = DIAG_PHASE_NAMES.get(event.detail1, f"CODE_{event.detail1}")

        if event.event_code in {0x0B, 0x0C, 0x0D}:
            payload["fault_code"] = event.detail0
            payload["fault_name"] = DIAG_FAULT_NAMES.get(event.detail0, f"CODE_{event.detail0}")
        elif event.event_code == 0x08:
            payload["elapsed_10ms"] = event.detail0
        elif event.event_code == 0x0F:
            payload["loop_overrun_count"] = event.detail0
        elif event.event_code == 0x10:
            payload["fault_code"] = event.detail0
            payload["fault_name"] = DIAG_FAULT_NAMES.get(event.detail0, f"CODE_{event.detail0}")

        return payload

    @staticmethod
    def _fault_snapshot_meta_payload(
        joint_name: str,
        meta: FaultSnapshotMeta,
        timestamp: float,
    ) -> dict[str, object]:
        return {
            "type": "fault_snapshot_meta",
            "joint_id": meta.joint_id,
            "joint_name": joint_name,
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

    @staticmethod
    def _fault_snapshot_dump_payload(
        joint_name: str,
        meta_payload: dict[str, object],
        decoded_snapshot: dict[str, object],
        *,
        timestamp: float,
        raw_bytes: bytes,
    ) -> dict[str, object]:
        return {
            "type": "fault_snapshot_dump",
            "joint_name": joint_name,
            "joint_id": meta_payload["joint_id"],
            "timestamp": timestamp,
            "snapshot_id": meta_payload["snapshot_id"],
            "freeze_event_code": meta_payload["freeze_event_code"],
            "freeze_event": meta_payload["freeze_event"],
            "primary_fault_code": meta_payload["primary_fault_code"],
            "primary_fault": meta_payload["primary_fault"],
            "payload_bytes": len(raw_bytes),
            "total_chunks": meta_payload["total_chunks"],
            "snapshot": decoded_snapshot,
            "raw_hex": raw_bytes.hex(),
        }

    def _handle_health_status(self, data: bytes, joint_id: int, timestamp: float) -> None:
        key, state = self._joint_state_for_id(joint_id)
        if key is None or state is None:
            return

        frame_kind = data[0] & 0xC0
        ext_kind = data[0] & 0x3F if frame_kind == 0x80 else None
        seq = data[7] if frame_kind != 0x80 else data[1]
        for pending_key in [key for key in self._health_status_pending if key[0] == joint_id and key[1] != seq]:
            self._health_status_pending.pop(pending_key, None)
        pending = self._health_status_pending.setdefault(
            (joint_id, seq),
            {"timestamp": timestamp},
        )
        pending["timestamp"] = timestamp
        if frame_kind == 0x00:
            pending["summary"] = decode_health_status_summary(data, joint_id)
        elif frame_kind == 0x40:
            pending["counters"] = decode_health_status_counters(data, joint_id)
        elif frame_kind == 0x80 and len(data) >= 8:
            if ext_kind == DIAG_HEALTH_EXT_LOOP_TIMING:
                avg_us, max_us, budget_us = struct.unpack_from("<HHH", data, 2)
                pending["loop_timing"] = (avg_us, max_us, budget_us)
                if isinstance(state.health_status, dict):
                    state.health_status["loop_avg_us"] = avg_us
                    state.health_status["loop_max_us"] = max_us
                    state.health_status["loop_budget_us"] = budget_us
                self._record_diagnostic(
                    self._health_loop_timing_payload(
                        joint_id,
                        key,
                        seq,
                        avg_us,
                        max_us,
                        budget_us,
                        timestamp,
                    )
                )
            elif ext_kind == DIAG_HEALTH_EXT_CAN_DETAILS:
                details = decode_health_status_can_details(data, joint_id)
                pending["can_details"] = details
                if isinstance(state.health_status, dict):
                    state.health_status["host_can_tec"] = details.host_can_tec
                    state.health_status["host_can_rec"] = details.host_can_rec
                    state.health_status["host_can_eflg"] = details.host_can_eflg
                    state.health_status["host_can_eflg_names"] = details.host_can_eflg_names
                    state.health_status["motor_can_tec"] = details.motor_can_tec
                    state.health_status["motor_can_rec"] = details.motor_can_rec
                    state.health_status["motor_can_eflg"] = details.motor_can_eflg
                    state.health_status["motor_can_eflg_names"] = details.motor_can_eflg_names
                self._record_diagnostic(
                    self._health_can_detail_payload(
                        joint_id,
                        key,
                        details,
                        timestamp,
                    )
                )
            else:
                return
        else:
            return

        summary = pending.get("summary")
        counters = pending.get("counters")
        if summary is None or counters is None:
            return

        payload = self._health_status_payload(
            joint_id,
            key,
            summary,
            counters,
            float(pending["timestamp"]),
            pending.get("can_details"),
            pending.get("loop_timing"),
        )
        state.health_status = payload
        self.health_status[joint_id] = payload
        state.last_update = time.monotonic()
        state.is_online = True
        self._health_status_pending.pop((joint_id, seq), None)
        self._record_diagnostic(payload)
        logger.debug(
            f"HEALTH [{key}] phase={summary.phase_name} reboot={summary.reboot_reason_name} "
            f"can=({counters.host_can_tx_error_count},{counters.host_can_rx_error_count},"
            f"{counters.motor_can_tx_error_count}) overrun={counters.loop_overrun_count} "
            f"eflg=({payload.get('host_can_eflg', 0)},{payload.get('motor_can_eflg', 0)})"
        )

    def _handle_fault_status(self, data: bytes, joint_id: int, timestamp: float) -> None:
        key, state = self._joint_state_for_id(joint_id)
        if key is None or state is None:
            return

        fault = decode_fault_status(data, joint_id)
        payload = self._fault_status_payload(key, fault, timestamp)
        state.fault_status = fault
        self.fault_status[joint_id] = fault
        state.last_update = time.monotonic()
        state.is_online = True
        self._record_diagnostic(payload)

        if fault.active_fault_names:
            logger.warning(
                f"FAULT [{key}] active={fault.active_fault_names} "
                f"latched={fault.latched_fault_names} source={fault.source_name}"
            )
        else:
            logger.info(f"FAULT [{key}] cleared latched={fault.latched_fault_names}")

    def _handle_event_notice(self, data: bytes, joint_id: int, timestamp: float) -> None:
        key, state = self._joint_state_for_id(joint_id)
        if key is None or state is None:
            return

        event = decode_event_notice(data, joint_id)
        payload = self._event_notice_payload(key, event, timestamp)
        state.diagnostic_events.appendleft(event)
        self.diagnostic_events.appendleft(event)
        state.last_update = time.monotonic()
        state.is_online = True
        self._record_diagnostic(payload)

        log_message = (
            f"EVENT [{key}] {event.event_name} src={event.source_kind}"
            + (
                f":{event.source_index_value}"
                if event.source_index_value is not None
                else ""
            )
        )
        if event.severity_code >= 3:
            logger.critical(log_message)
        elif event.severity_code >= 2:
            logger.warning(log_message)
        else:
            logger.info(log_message)

    def _handle_fault_snapshot_meta(self, data: bytes, joint_id: int, timestamp: float) -> None:
        key, state = self._joint_state_for_id(joint_id)
        if key is None or state is None:
            return

        meta = decode_fault_snapshot_meta(data, joint_id)
        payload = self._fault_snapshot_meta_payload(key, meta, timestamp)
        state.fault_snapshot_meta = payload
        self.fault_snapshot_meta[joint_id] = meta
        state.last_update = time.monotonic()
        state.is_online = True
        self._record_diagnostic(payload)

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

        logger.info(
            f"SNAPSHOT_META [{key}] present={meta.snapshot_present} snapshot={meta.snapshot_id} "
            f"chunks={meta.total_chunks} bytes={meta.payload_bytes}"
        )

    def _handle_fault_snapshot_chunk(self, data: bytes, joint_id: int, timestamp: float) -> None:
        key, state = self._joint_state_for_id(joint_id)
        if key is None or state is None:
            return

        chunk = decode_fault_snapshot_chunk(data, joint_id)
        pending = self._fault_snapshot_pending.setdefault(joint_id, {"chunks": {}})
        if pending.get("snapshot_id") not in (None, chunk.snapshot_id):
            pending.clear()
            pending["chunks"] = {}
        pending["snapshot_id"] = chunk.snapshot_id
        pending.setdefault("chunks", {})[chunk.chunk_index] = chunk.payload
        pending["last_timestamp"] = timestamp
        state.last_update = time.monotonic()
        state.is_online = True

        total_chunks = pending.get("total_chunks")
        payload_bytes = pending.get("payload_bytes")
        meta_payload = pending.get("meta_payload")
        if not isinstance(total_chunks, int) or not isinstance(payload_bytes, int) or not isinstance(meta_payload, dict):
            return

        chunks: dict[int, bytes] = pending["chunks"]  # type: ignore[assignment]
        if len(chunks) < total_chunks:
            return
        if any(index not in chunks for index in range(total_chunks)):
            return

        raw = b"".join(chunks[index] for index in range(total_chunks))[:payload_bytes]
        decoded_snapshot = decode_fault_snapshot_blob(raw)
        dump_payload = self._fault_snapshot_dump_payload(
            key,
            meta_payload,
            decoded_snapshot,
            timestamp=timestamp,
            raw_bytes=raw,
        )
        state.fault_snapshot_dump = dump_payload
        self.fault_snapshot_dumps[joint_id] = dump_payload
        self._record_diagnostic(dump_payload)
        self._fault_snapshot_pending.pop(joint_id, None)
        logger.warning(
            f"SNAPSHOT_DUMP [{key}] snapshot={chunk.snapshot_id} bytes={payload_bytes} "
            f"fault={decoded_snapshot.get('primary_fault')}"
        )

    def _expire_fault_snapshot_pending(self, now_timestamp: float) -> None:
        stale_joint_ids = [
            joint_id
            for joint_id, pending in self._fault_snapshot_pending.items()
            if isinstance(pending.get("last_timestamp"), (int, float))
            and (now_timestamp - float(pending["last_timestamp"])) > FAULT_SNAPSHOT_PENDING_TIMEOUT_S
        ]
        for joint_id in stale_joint_ids:
            self._fault_snapshot_pending.pop(joint_id, None)

    def _handle_encoder(self, data: bytes, joint_id: int) -> None:
        enc = decode_encoder_stream(data, joint_id)
        jcfg = self._config.joint_by_id(joint_id)
        if jcfg is None:
            return  # Not a controlled joint

        key = self._config._id_to_key.get(joint_id)
        if key is None:
            return

        state = self.states.get(key)
        if state is None:
            return

        # Update angles (only valid DOFs)
        for i, angle in enumerate(enc.angles_deg):
            if angle is not None:
                state.angles_deg[i] = angle

        state.last_update = time.monotonic()
        state.is_online = True
        state.rx_count += 1

    def _handle_announce(self, data: bytes, joint_id: int) -> None:
        announce = decode_joint_announce(data, joint_id)
        jcfg = self._config.joint_by_id(joint_id)
        if jcfg is None:
            self.unexpected_announces[joint_id] = announce
            logger.info(
                f"Announce from unexpected joint_id={joint_id} "
                f"(configured ids={sorted(self._config._id_to_key.keys())}), ignoring"
            )
            return

        key = self._config._id_to_key[joint_id]
        state = self.states[key]
        state.announce = announce
        state.is_online = True

        logger.info(
            f"Joint announce [{jcfg.name}]: {announce.dof_count} DOFs, "
            f"{announce.motor_count} motors, fw={announce.fw_version}, "
            f"ready={announce.ready}"
        )

        evt = self.announce_events.get(joint_id)
        if evt:
            evt.set()

    def _handle_startup(self, data: bytes, joint_id: int) -> None:
        status = decode_startup_status(data, joint_id)
        self.startup_results[joint_id] = status

        jcfg = self._config.joint_by_id(joint_id)
        name = jcfg.name if jcfg else f"joint_{joint_id}"

        logger.info(
            f"Startup [{name}]: {status.event_name} DOF={status.dof_index} "
            f"reason={status.reason_name} elapsed={status.elapsed_ms}ms"
        )

        # Signal FSM on terminal events
        if status.is_complete or status.is_failed:
            evt = self.startup_events.get(joint_id)
            if evt:
                evt.set()

    def all_at_target(self, targets: dict[str, dict[int, float]],
                      tolerance_deg: float = 1.0) -> bool:
        """Check if all joints/DOFs are holding and within tolerance of target."""
        for key, dof_targets in targets.items():
            state = self.states.get(key)
            if state is None or not state.is_online:
                return False
            for dof, target in dof_targets.items():
                current = state.angles_deg.get(dof)
                if current is None:
                    return False
                if not state.holding.get(dof, False):
                    return False
                if abs(current - target) > tolerance_deg:
                    return False
        return True

    def dofs_at_target(self, targets: dict[str, dict[int, float]],
                       tolerance_deg: float = 1.0) -> dict[str, dict[int, bool]]:
        """Per-DOF arrival status using the same holding+error criterion as all_at_target()."""
        result: dict[str, dict[int, bool]] = {}
        for key, dof_targets in targets.items():
            state = self.states.get(key)
            result[key] = {}
            for dof, target in dof_targets.items():
                current = state.angles_deg.get(dof) if state is not None else None
                holding = state.holding.get(dof, False) if state is not None else False
                result[key][dof] = (
                    state is not None
                    and state.is_online
                    and current is not None
                    and holding
                    and abs(current - target) <= tolerance_deg
                )
        return result

    def any_stale(self, max_age_s: float = 2.0) -> list[str]:
        """Return list of joints with stale telemetry."""
        now = time.monotonic()
        stale = []
        for key, state in self.states.items():
            if state.is_online and (now - state.last_update) > max_age_s:
                stale.append(key)
        return stale
