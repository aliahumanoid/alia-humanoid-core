"""
Safety manager: emergency stop and watchdog configuration.

E-stop sends CAN 0x000 immediately. Always available regardless of FSM state.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Iterable

from .can_bus import CanBus
from .protocol import encode_emergency_stop, encode_impedance_ctrl
from .telemetry import TelemetryManager

logger = logging.getLogger(__name__)


class SafetyManager:
    _PRESTART_BLOCKING_ACTIVE_FAULTS = frozenset({
        "CONFIG_INVALID",
        "FLASH_ERROR",
        "INTERNAL_ERROR",
    })
    _PRESTART_BLOCKING_LATCHED_FAULTS = frozenset({
        "CONFIG_INVALID",
        "FLASH_ERROR",
        "INTERNAL_ERROR",
    })
    _MOVEMENT_BLOCKING_ACTIVE_FAULTS = frozenset({
        "HOST_WATCHDOG_TIMEOUT",
        "ENCODER_INVALID",
        "MOTOR_FEEDBACK_INVALID",
        "MOTOR_TIMEOUT",
        "SAFETY_LIMIT",
        "MAPPING_LIMIT",
        "MOTOR_RANGE",
        "STARTUP_FAILED",
        "CONFIG_INVALID",
        "FLASH_ERROR",
        "ESTOP_LATCHED",
        "INTERNAL_ERROR",
    })
    _MOVEMENT_BLOCKING_LATCHED_FAULTS = frozenset({
        "CONFIG_INVALID",
        "FLASH_ERROR",
        "INTERNAL_ERROR",
    })
    _FAULT_RECOVERY_HINTS = {
        "ESTOP_LATCHED": "Recover motor power with PRETENSION_ALL or power-cycle the joint controller.",
        "HOST_WATCHDOG_TIMEOUT": "Re-seed impedance targets from live telemetry and confirm host keepalive timing.",
        "ENCODER_INVALID": "Check encoder wiring, power, and angle validity before moving.",
        "MOTOR_FEEDBACK_INVALID": "Check 0xA1 replies, 0x92 re-anchor health, and motor-CAN counters before moving.",
        "MOTOR_TIMEOUT": "Check motor CAN bus, power stage, and motor responsiveness.",
        "SAFETY_LIMIT": "Inspect joint pose and clear the safety condition before moving again.",
        "MAPPING_LIMIT": "Verify mapping limits and recalibrate the joint before moving.",
        "MOTOR_RANGE": "Inspect tendon routing, pretension, and motor range before moving.",
        "STARTUP_FAILED": "Inspect startup telemetry, fix the failed DOF, then re-run startup.",
        "CONFIG_INVALID": "Provision the correct joint profile on the controller before startup.",
        "FLASH_ERROR": "Reload or reflash controller configuration before startup.",
        "INTERNAL_ERROR": "Power-cycle the controller and inspect the session log before retrying.",
    }

    def __init__(self):
        self._estop_sent = False

    @property
    def estop_latched(self) -> bool:
        return self._estop_sent

    def clear_estop_latch(self) -> None:
        self._estop_sent = False

    async def send_estop(self, can_bus: CanBus, reason: int = 0) -> None:
        """Send emergency stop broadcast. Safe to call multiple times.

        REPEAT-SEND x3 spaced >=12ms (2026-07-05 finding): the board's host-CAN MCP2515
        buffers exactly 2 frames and core1 drains it once per control cycle (4ms nominal,
        up to ~6.3ms under load) — a SINGLE 0x000 frame landing in a full inter-poll window
        is silently hard-dropped (the same live-proven mechanism that dropped LOOP_FREQUENCY
        2/2 on the loaded ankle), and a lost e-stop has NO automatic backstop (the impedance
        watchdog freezes into an ACTIVELY DRIVEN hold, it does not de-energize). Three sends
        spaced beyond the worst inter-poll window hit statistically independent windows
        (residual drop ~1e-3); the e-stop handler is idempotent, repeats are harmless.
        """
        if not can_bus.connected:
            logger.warning("Cannot send E-stop: CAN not connected")
            return
        arb_id, data = encode_emergency_stop(reason)
        for attempt in range(3):
            await can_bus.send(arb_id, data)
            if attempt < 2:
                await asyncio.sleep(0.012)
        self._estop_sent = True
        logger.critical(f"EMERGENCY STOP sent x3 (reason={reason})")

    async def configure_watchdog(self, can_bus: CanBus, joint_id: int,
                                 timeout_ms: int) -> None:
        """Set impedance watchdog timeout for a joint."""
        arb_id, data = encode_impedance_ctrl(joint_id, sub_cmd=0x02, param=timeout_ms)
        await can_bus.send(arb_id, data)
        logger.info(f"Impedance watchdog set: joint_id={joint_id} timeout={timeout_ms}ms")

    def check_telemetry(self, telemetry: TelemetryManager,
                        max_age_s: float = 2.0) -> list[str]:
        """Return joint names with stale telemetry (potential connection loss)."""
        return telemetry.any_stale(max_age_s)

    @classmethod
    def _fault_hint(cls, fault_name: str) -> str | None:
        return cls._FAULT_RECOVERY_HINTS.get(fault_name)

    # Faults that are expected when resuming a session where the previous host
    # disconnected without sending E-stop.  The controller kept running, the
    # watchdog timed out, and the control loop may have accumulated overruns
    # while idling without host commands.  These clear automatically once the
    # new host starts streaming SET_IMPEDANCE frames.
    _SESSION_RESUME_IGNORABLE_ACTIVE = frozenset({
        "HOST_WATCHDOG_TIMEOUT",
        "LOOP_OVERRUN",
    })

    def diagnostic_blockers(
        self,
        telemetry: TelemetryManager,
        joint_keys: Iterable[str] | None = None,
        *,
        mode: str = "movement",
        allow_estop_recovery: bool = False,
        allow_session_resume: bool = False,
    ) -> dict[str, dict[str, object]]:
        """Return blocking diagnostic faults by joint for the requested mode.

        Modes:
          - ``prestart``: block only on non-recoverable controller faults
          - ``movement``: block on active motion-safety faults and persistent hard faults

        Flags:
          - ``allow_estop_recovery``: exclude ESTOP_LATCHED from blockers
          - ``allow_session_resume``: exclude HOST_WATCHDOG_TIMEOUT and
            LOOP_OVERRUN from active blockers (expected when resuming after a
            previous host disconnected without E-stop)
        """
        if mode == "prestart":
            blocking_active = self._PRESTART_BLOCKING_ACTIVE_FAULTS
            blocking_latched = self._PRESTART_BLOCKING_LATCHED_FAULTS
        else:
            blocking_active = self._MOVEMENT_BLOCKING_ACTIVE_FAULTS
            blocking_latched = self._MOVEMENT_BLOCKING_LATCHED_FAULTS

        blockers: dict[str, dict[str, object]] = {}
        keys = list(joint_keys or telemetry.states.keys())
        for key in keys:
            state = telemetry.states.get(key)
            if state is None:
                continue
            fault = getattr(state, "fault_status", None)
            if fault is None:
                continue

            active_faults = [
                name for name in getattr(fault, "active_fault_names", [])
                if name in blocking_active
            ]
            latched_faults = [
                name for name in getattr(fault, "latched_fault_names", [])
                if name in blocking_latched
            ]

            if allow_estop_recovery:
                active_faults = [name for name in active_faults if name != "ESTOP_LATCHED"]
                latched_faults = [name for name in latched_faults if name != "ESTOP_LATCHED"]

            if allow_session_resume:
                active_faults = [
                    name for name in active_faults
                    if name not in self._SESSION_RESUME_IGNORABLE_ACTIVE
                ]

            if not active_faults and not latched_faults:
                continue

            latest_event = next(iter(getattr(state, "diagnostic_events", []) or []), None)
            health = getattr(state, "health_status", None) or {}
            hint_fault = active_faults[0] if active_faults else latched_faults[0]
            blockers[key] = {
                "active": active_faults,
                "latched": latched_faults,
                "phase": health.get("phase"),
                "latest_event": getattr(latest_event, "event_name", None),
                "latest_severity": getattr(latest_event, "severity_name", None),
                "hint": self._fault_hint(hint_fault),
            }

        return blockers

    def format_diagnostic_blockers(self, blockers: dict[str, dict[str, object]]) -> str:
        """Format blocking diagnostic faults into a concise operator-facing string."""
        segments: list[str] = []
        for key, info in blockers.items():
            parts = [key]
            active = list(info.get("active", []))
            latched = list(info.get("latched", []))
            if active:
                parts.append("active=" + ",".join(active))
            if latched:
                parts.append("latched=" + ",".join(latched))
            phase = info.get("phase")
            if phase:
                parts.append(f"phase={phase}")
            latest_event = info.get("latest_event")
            latest_severity = info.get("latest_severity")
            if latest_event:
                if latest_severity:
                    parts.append(f"last={latest_event}/{latest_severity}")
                else:
                    parts.append(f"last={latest_event}")
            hint = info.get("hint")
            segment = " ".join(parts)
            if hint:
                segment += f". {hint}"
            segments.append(segment)
        return "; ".join(segments)
