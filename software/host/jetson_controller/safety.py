"""
Safety manager: emergency stop and watchdog configuration.

E-stop sends CAN 0x000 immediately. Always available regardless of FSM state.
"""
from __future__ import annotations

import logging

from .can_bus import CanBus
from .protocol import encode_emergency_stop, encode_impedance_ctrl
from .telemetry import TelemetryManager

logger = logging.getLogger(__name__)


class SafetyManager:

    def __init__(self):
        self._estop_sent = False

    @property
    def estop_latched(self) -> bool:
        return self._estop_sent

    def clear_estop_latch(self) -> None:
        self._estop_sent = False

    async def send_estop(self, can_bus: CanBus, reason: int = 0) -> None:
        """Send emergency stop broadcast. Safe to call multiple times."""
        if not can_bus.connected:
            logger.warning("Cannot send E-stop: CAN not connected")
            return
        arb_id, data = encode_emergency_stop(reason)
        await can_bus.send(arb_id, data)
        self._estop_sent = True
        logger.critical(f"EMERGENCY STOP sent (reason={reason})")

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
