"""
Telemetry manager: processes incoming CAN frames and tracks joint state.

Runs as an asyncio task, consuming messages from CanBus.recv() and
updating JointState objects that other modules can query.
"""
from __future__ import annotations

import asyncio
import logging
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from probe_history import ProbeHistoryWriter

from .can_bus import CanBus
from .config import ControllerConfig
from .protocol import (
    CAN_ID_ENCODER_STREAM_DATA,
    CAN_ID_JOINT_ANNOUNCE,
    CAN_ID_JOINT_STATE,
    CAN_ID_RETENSION_PROBE_RESULT,
    CAN_ID_STARTUP_STATUS,
    JointAnnounce,
    RetensionProbeResult,
    StartupStatus,
    decode_encoder_stream,
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

        # Set up events for expected joints
        for key, jcfg in config.joints.items():
            self.announce_events[jcfg.joint_id] = asyncio.Event()
            self.startup_events[jcfg.joint_id] = asyncio.Event()

        self._running = False
        self._retension_probe_pending: dict[tuple[int, int], dict[str, object]] = {}
        self._probe_history = ProbeHistoryWriter(
            Path(__file__).resolve().parent.parent / "logs" / "probe_history",
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
            logger.info(f"Announce from unknown joint_id={joint_id}, ignoring")
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
        """Check if all joints/DOFs are within tolerance of their targets."""
        for key, dof_targets in targets.items():
            state = self.states.get(key)
            if state is None or not state.is_online:
                return False
            for dof, target in dof_targets.items():
                current = state.angles_deg.get(dof)
                if current is None:
                    return False
                if abs(current - target) > tolerance_deg:
                    return False
        return True

    def any_stale(self, max_age_s: float = 2.0) -> list[str]:
        """Return list of joints with stale telemetry."""
        now = time.monotonic()
        stale = []
        for key, state in self.states.items():
            if state.is_online and (now - state.last_update) > max_age_s:
                stale.append(key)
        return stale
