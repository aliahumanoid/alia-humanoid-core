"""
50 Hz impedance control loop.

Sends SET_IMPEDANCE frame 0 to all controlled DOFs every cycle.
Gains are only re-sent when explicitly changed (dirty flag).

Lifecycle is managed via start()/stop() which own the asyncio.Task.
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Optional

from .can_bus import CanBus
from .config import ControllerConfig, GainSet
from .protocol import (
    encode_set_impedance_frame0,
    encode_set_impedance_frame1,
    encode_set_impedance_frame2,
    encode_set_impedance_frame3,
)

logger = logging.getLogger(__name__)


@dataclass
class ImpedanceTarget:
    """Per-DOF impedance target."""
    q_deg: float = 0.0
    dq_deg_s: float = 5.0
    stiffness_deg: float = 25.0
    gains_outer: GainSet = field(default_factory=lambda: GainSet(8.0, 1.0, 0.08))
    gains_inner: GainSet = field(default_factory=lambda: GainSet(10.0, 1.0, 0.25))


class ImpedanceLoop:
    """Async 50 Hz control loop for SET_IMPEDANCE streaming."""

    def __init__(self, config: ControllerConfig):
        self._config = config
        self._period = 1.0 / config.send_rate_hz
        self._task: Optional[asyncio.Task] = None

        # Per-joint, per-DOF targets
        self.targets: dict[str, dict[int, ImpedanceTarget]] = {}
        self._gains_dirty: dict[str, dict[int, bool]] = {}

        for key, jcfg in config.joints.items():
            self.targets[key] = {}
            self._gains_dirty[key] = {}
            for dof in range(jcfg.dof_count):
                self.targets[key][dof] = ImpedanceTarget(
                    q_deg=jcfg.home_position_deg.get(dof, 0.0),
                    stiffness_deg=jcfg.stiffness_deg,
                    gains_outer=GainSet(
                        jcfg.gains_outer.kp,
                        jcfg.gains_outer.ki,
                        jcfg.gains_outer.kd,
                    ),
                    gains_inner=GainSet(
                        jcfg.gains_inner.kp,
                        jcfg.gains_inner.ki,
                        jcfg.gains_inner.kd,
                    ),
                )
                self._gains_dirty[key][dof] = False

        # Stats
        self.cycle_count = 0
        self.last_cycle_time_ms = 0.0    # Send time only (how long _send_cycle took)
        self.avg_cycle_time_ms = 0.0     # EMA of send time
        self.avg_period_ms = 0.0         # EMA of full cycle period (send + sleep)

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    def start(self, can_bus: CanBus) -> None:
        """Start the impedance loop as a managed task. No-op if already running."""
        if self.running:
            logger.warning("Impedance loop already running, ignoring duplicate start")
            return
        self._task = asyncio.create_task(self._run(can_bus), name="impedance-loop")
        self._task.add_done_callback(self._on_task_done)

    async def stop(self) -> None:
        """Stop the impedance loop and wait for it to finish."""
        if self._task is None or self._task.done():
            return
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        self._task = None
        logger.info("Impedance loop stopped")

    @staticmethod
    def _on_task_done(task: asyncio.Task) -> None:
        if task.cancelled():
            return
        exc = task.exception()
        if exc:
            logger.error(f"Impedance loop crashed: {exc}")

    async def _run(self, can_bus: CanBus) -> None:
        """Internal loop: send SET_IMPEDANCE at configured rate."""
        logger.info(f"Impedance loop started @ {self._config.send_rate_hz} Hz")
        last_cycle_start = time.monotonic()
        try:
            while True:
                t0 = time.monotonic()

                # Track full period (time since last cycle start)
                if self.cycle_count > 0:
                    period_ms = (t0 - last_cycle_start) * 1000.0
                    alpha_p = 0.05
                    self.avg_period_ms = (
                        alpha_p * period_ms +
                        (1 - alpha_p) * self.avg_period_ms
                    )
                else:
                    self.avg_period_ms = self._period * 1000.0
                last_cycle_start = t0

                await self._send_cycle(can_bus)

                self.cycle_count += 1
                elapsed = time.monotonic() - t0
                self.last_cycle_time_ms = elapsed * 1000.0

                # Exponential moving average of send time
                alpha = 0.05
                self.avg_cycle_time_ms = (
                    alpha * self.last_cycle_time_ms +
                    (1 - alpha) * self.avg_cycle_time_ms
                )

                # Sleep remainder of period
                sleep_time = self._period - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
        except asyncio.CancelledError:
            raise  # Propagate so task.cancelled() returns True

    async def _send_cycle(self, can_bus: CanBus) -> None:
        """Send one impedance update for all DOFs."""
        for key, jcfg in self._config.joints.items():
            for dof in range(jcfg.dof_count):
                target = self.targets[key][dof]

                if self._gains_dirty[key][dof]:
                    await self._send_with_gains(can_bus, jcfg.joint_id, dof, target)
                    self._gains_dirty[key][dof] = False
                else:
                    await self._send_position_only(can_bus, jcfg.joint_id, dof, target)

    async def _send_position_only(self, can_bus: CanBus, joint_id: int,
                                  dof: int, target: ImpedanceTarget) -> None:
        """Send frame 0 only (regime operation)."""
        arb_id, data = encode_set_impedance_frame0(
            joint_id, dof,
            q_deg=target.q_deg,
            dq_deg_s=target.dq_deg_s,
            stiffness_deg=target.stiffness_deg,
            has_more=False,
        )
        await can_bus.send(arb_id, data)

    async def _send_with_gains(self, can_bus: CanBus, joint_id: int,
                               dof: int, target: ImpedanceTarget) -> None:
        """Send frames 0+1+2+3 (gain change + tau_ff reset)."""
        tau_ff = getattr(target, 'tau_ff', 0)
        frames = [
            encode_set_impedance_frame0(
                joint_id, dof,
                q_deg=target.q_deg,
                dq_deg_s=target.dq_deg_s,
                stiffness_deg=target.stiffness_deg,
                has_more=True,
            ),
            encode_set_impedance_frame1(
                joint_id, dof,
                kp=target.gains_outer.kp,
                ki=target.gains_outer.ki,
                kd=target.gains_outer.kd,
                has_more=True,
            ),
            encode_set_impedance_frame2(
                joint_id, dof,
                kp_inner=target.gains_inner.kp,
                ki_inner=target.gains_inner.ki,
                kd_inner=target.gains_inner.kd,
                has_more=True,
            ),
            encode_set_impedance_frame3(
                joint_id, dof,
                tau_ff=tau_ff,
            ),
        ]
        await can_bus.send_impedance_sequence(frames)

    # ------------------------------------------------------------------
    # Public API for updating targets
    # ------------------------------------------------------------------

    def set_target(self, joint: str, dof: int, q_deg: float,
                   dq_deg_s: float = None) -> None:
        """Update position target for a DOF."""
        t = self.targets[joint][dof]
        t.q_deg = q_deg
        if dq_deg_s is not None:
            t.dq_deg_s = dq_deg_s

    def set_gains(self, joint: str, dof: int,
                  outer: GainSet = None, inner: GainSet = None) -> None:
        """Update gains for a DOF. Marks dirty for next cycle."""
        t = self.targets[joint][dof]
        if outer:
            t.gains_outer = outer
        if inner:
            t.gains_inner = inner
        self._gains_dirty[joint][dof] = True

    def set_stiffness(self, joint: str, dof: int, stiffness_deg: float) -> None:
        """Update co-contraction stiffness."""
        self.targets[joint][dof].stiffness_deg = stiffness_deg

    def set_all_targets(self, q_deg: float, dq_deg_s: float = 5.0) -> None:
        """Set the same target for all DOFs (useful for synchronized homing)."""
        for key in self.targets:
            for dof in self.targets[key]:
                self.targets[key][dof].q_deg = q_deg
                self.targets[key][dof].dq_deg_s = dq_deg_s
