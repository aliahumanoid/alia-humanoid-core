"""
50 Hz impedance control loop.

Sends SET_IMPEDANCE frame 0 to all controlled DOFs every cycle.
Gains are only re-sent when explicitly changed (dirty flag).

Lifecycle is managed via start()/stop() which own the asyncio.Task.
"""
from __future__ import annotations

import asyncio
import logging
import math
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

# Hard host-side clamp on the learned feedforward torque (RAW motor-iq units, int16 on the
# wire, SET_IMPEDANCE frame3). 30 raw units = inside the bench-exercised GMS friction-FF
# territory (Fs=30). The firmware does NOT clamp tau_ff at commit, so this host clamp is the
# tau_ff-specific safety limit; the per-motor PID output saturation + rate limit downstream
# in the firmware remain authoritative.
TAU_FF_ABS_MAX_RAW = 30


@dataclass
class ImpedanceTarget:
    """Per-DOF impedance target.

    q_deg / dq_deg_s / stiffness_deg are the SET_IMPEDANCE frame0 fields actually streamed each
    cycle. When interpolation is enabled (set_rate with interpolate=True), q_deg is the *operator*
    setpoint and q_cmd is the host-interpolated value the cycle actually sends — slewed toward
    q_deg at dq_deg_s (deg/s) so a higher stream rate yields a genuinely smoother position
    reference toward the 250 Hz inner loop. With interpolation off, q_cmd tracks q_deg exactly
    (None => use q_deg), so existing 50 Hz behaviour is byte-for-byte unchanged.
    """
    q_deg: float = 0.0
    dq_deg_s: float = 5.0
    stiffness_deg: float = 25.0
    gains_outer: GainSet = field(default_factory=lambda: GainSet(8.0, 1.0, 0.08))
    gains_inner: GainSet = field(default_factory=lambda: GainSet(10.0, 1.0, 0.25))
    q_cmd: Optional[float] = None    # host-interpolated commanded position (None => follow q_deg)
    tau_ff: int = 0                  # feedforward torque, RAW motor-iq units (frame3; 0 = none)


class ImpedanceLoop:
    """Async 50 Hz control loop for SET_IMPEDANCE streaming."""

    # Host-side stream-rate ceiling. A SET_IMPEDANCE frame is up to 4 sub-frames * (DOFs) per cycle;
    # at 500 kbit/s a single-DOF frame0 (~120 bits w/ overhead) is ~0.25 ms, so even 150 Hz is well
    # under a few % bus load for one DOF. Cap conservatively to avoid bus overrun on multi-DOF joints.
    # NOTE (armed tau_ff 2-frame path budget): with the tau_ff channel ARMED the steady stream is
    # frame0+frame3 per DOF with the 3 ms MULTI_FRAME_DELAY_S between them, so each cycle costs
    # >= ~3.5 ms send time alone — a commanded 200 Hz (5 ms period) silently degrades to ~150 Hz
    # effective. Recommend <= 120 Hz for armed sessions. The stream loop logs a one-time warning
    # when the realized avg period exceeds the commanded period by > 20%.
    MAX_STREAM_HZ = 200.0

    def __init__(self, config: ControllerConfig):
        self._config = config
        self._base_period = 1.0 / config.send_rate_hz
        self._period = self._base_period
        self._interpolate = False        # host-interpolate q toward the operator setpoint per cycle
        self._task: Optional[asyncio.Task] = None

        # Per-joint, per-DOF targets
        self.targets: dict[str, dict[int, ImpedanceTarget]] = {}
        self._gains_dirty: dict[str, dict[int, bool]] = {}
        # Velocity-gated co-contraction modulation, per (joint, dof). Empty => OFF (baseline
        # stiffness streamed unchanged). When enabled, stiffness is REDUCED smoothly during motion
        # and restored at rest, never below `floor` (eyelet-break safety). See set_cocontraction_vel.
        self._cocon_vel: dict[tuple[str, int], dict] = {}
        # Per-DOF low-passed stiffness command (the actually-streamed, rate-limited stiffness).
        self._stiff_cmd: dict[tuple[str, int], float] = {}
        # Per-DOF previous-cycle commanded position, for the commanded-velocity estimate that gates
        # the co-contraction modulation (deg/s = |q_cmd - prev_q_cmd| / period).
        self._prev_q_cmd: dict[tuple[str, int], float] = {}
        # (joint, dof) pairs with the tau_ff feedforward channel ARMED. Empty = OFF: the
        # steady-regime stream stays frame0-only, byte-identical to the legacy behavior.
        self._tau_ff_armed: set[tuple[str, int]] = set()

        for key, jcfg in config.joints.items():
            self.targets[key] = {}
            self._gains_dirty[key] = {}
            for dof in range(jcfg.dof_count):
                self.targets[key][dof] = ImpedanceTarget(
                    q_deg=jcfg.home_position_deg.get(dof, 0.0),
                    stiffness_deg=jcfg.stiffness_for(dof),
                    gains_outer=GainSet(
                        jcfg.outer_gains_for(dof).kp,
                        jcfg.outer_gains_for(dof).ki,
                        jcfg.outer_gains_for(dof).kd,
                    ),
                    gains_inner=GainSet(
                        jcfg.inner_gains_for(dof).kp,
                        jcfg.inner_gains_for(dof).ki,
                        jcfg.inner_gains_for(dof).kd,
                    ),
                )
                self._gains_dirty[key][dof] = False

        # Stats
        self.cycle_count = 0
        self.last_cycle_time_ms = 0.0    # Send time only (how long _send_cycle took)
        self.avg_cycle_time_ms = 0.0     # EMA of send time
        self.avg_period_ms = 0.0         # EMA of full cycle period (send + sleep)
        self._rate_warned = False        # one-time stream-degradation warning latch

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def stream_hz(self) -> float:
        """Current SET_IMPEDANCE stream rate (Hz)."""
        return 1.0 / self._period if self._period > 0 else 0.0

    def set_rate(self, hz: Optional[float], interpolate: bool = False) -> float:
        """Set the SET_IMPEDANCE stream rate host-side (takes effect next cycle). Returns the rate
        actually applied (clamped to [config.send_rate_hz .. MAX_STREAM_HZ] to avoid bus overrun and
        never *slower* than the configured baseline). hz=None restores the configured baseline rate
        and disables interpolation. interpolate=True slews q_cmd toward the operator q_deg at
        dq_deg_s each cycle, so a faster stream produces a genuinely smoother position reference."""
        if hz is None:
            self._period = self._base_period
            self._interpolate = False
            return self.stream_hz
        hz = max(1.0 / self._base_period, min(float(hz), self.MAX_STREAM_HZ))
        self._period = 1.0 / hz
        was_interp = self._interpolate
        self._interpolate = bool(interpolate)
        if not self._interpolate:
            # Reset any latched interpolation state so q_cmd snaps back to following q_deg.
            for key in self.targets:
                for dof in self.targets[key]:
                    self.targets[key][dof].q_cmd = None
        elif not was_interp:
            # Enabling interpolation: seed q_cmd from the CURRENT commanded position so a later
            # set_target jump slews FROM here (not snapping straight to the new setpoint).
            for key in self.targets:
                for dof in self.targets[key]:
                    t = self.targets[key][dof]
                    if t.q_cmd is None:
                        t.q_cmd = t.q_deg
        return hz

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
        logger.info(f"Impedance loop started @ {self.stream_hz:.0f} Hz"
                    f"{' (interp)' if self._interpolate else ''}")
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

                # One-time degradation warning: the armed tau_ff 2-frame path (or a
                # loaded host) can make the realized period exceed the commanded one
                # (see the MAX_STREAM_HZ note — 200 Hz armed degrades to ~150 Hz).
                if (not self._rate_warned and self.cycle_count >= 100
                        and self.avg_period_ms > 1.2 * self._period * 1000.0):
                    self._rate_warned = True
                    logger.warning(
                        f"SET_IMPEDANCE stream degraded: realized avg period "
                        f"{self.avg_period_ms:.1f} ms vs commanded "
                        f"{self._period * 1000.0:.1f} ms (> 20% over). The armed "
                        f"tau_ff 2-frame path costs ~3.5 ms/DOF per cycle — "
                        f"recommend <= 120 Hz for armed sessions.")

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

    def set_cocontraction_vel(self, joint: str, dof: int, enabled: bool, *,
                              floor: float, gain: float, tau_s: float = 0.15) -> None:
        """Enable/disable velocity-gated co-contraction for a DOF (operator's reciprocal-inhibition
        idea). When enabled, the streamed stiffness each cycle is

            stiff_target = max(floor, baseline - gain * |commanded velocity|)

        then LOW-PASSED (first-order, time-constant tau_s) so the command never steps — only smooth,
        rate-limited changes reach the bus. The HARD FLOOR is enforced AFTER the low-pass too, so the
        releasing tendon can NEVER be commanded slack regardless of transient. Symmetric: the firmware
        splits a single stiffness value +/- across both tendons (see report); asymmetric/per-tendon
        lowering is NOT host-feasible with the current SET_IMPEDANCE frame.

        floor is REQUIRED (eyelet-break safety). enabled=False removes the modulation (baseline
        stiffness streamed unchanged)."""
        key = (joint, dof)
        if not enabled:
            self._cocon_vel.pop(key, None)
            self._stiff_cmd.pop(key, None)
            return
        if floor is None:
            raise ValueError("set_cocontraction_vel: stiff_floor is REQUIRED (eyelet-break safety)")
        baseline = self.targets[joint][dof].stiffness_deg
        floor = max(0.0, float(floor))
        if floor > baseline:
            # Never raise stiffness above baseline via the floor; clamp so the modulation is a
            # pure reduction. (A floor above baseline would be a no-op reduction anyway.)
            floor = baseline
        self._cocon_vel[key] = {
            "baseline": baseline,
            "floor": floor,
            "gain": max(0.0, float(gain)),
            "tau_s": max(1e-3, float(tau_s)),
        }
        self._stiff_cmd[key] = baseline    # start at rest (full co-contraction)

    def _cmd_stiffness(self, joint: str, dof: int, target: ImpedanceTarget, q_cmd: float) -> float:
        """The stiffness this cycle actually streams. Baseline unless velocity-gated co-contraction
        is enabled for this DOF, in which case it's the floor-clamped, low-passed value.

        |commanded velocity| is estimated from the change in the position actually sent this cycle
        (q_cmd) vs the previous cycle, divided by the loop period. This captures motion whether the
        target steps directly (run_*_pattern set_target) OR is host-interpolated, and is exactly 0
        at rest -> full co-contraction restored."""
        key = (joint, dof)
        prev = self._prev_q_cmd.get(key, q_cmd)
        self._prev_q_cmd[key] = q_cmd
        cfg = self._cocon_vel.get(key)
        if cfg is None:
            return target.stiffness_deg
        cmd_vel = abs(q_cmd - prev) / self._period if self._period > 0 else 0.0
        stiff_target = max(cfg["floor"], cfg["baseline"] - cfg["gain"] * cmd_vel)
        # first-order low-pass toward stiff_target (no step changes on the bus)
        cur = self._stiff_cmd.get(key, cfg["baseline"])
        alpha = min(1.0, self._period / cfg["tau_s"])
        cur += alpha * (stiff_target - cur)
        cur = max(cfg["floor"], cur)    # hard floor AFTER the low-pass — never slack
        self._stiff_cmd[key] = cur
        return cur

    def _cmd_q(self, target: ImpedanceTarget) -> float:
        """The position this cycle actually sends. With interpolation off, it's the operator q_deg
        exactly (existing behaviour). With interpolation on, q_cmd is slewed toward q_deg by at most
        dq_deg_s*period each cycle (a host-side reference filter toward the 250 Hz inner loop)."""
        if not self._interpolate:
            return target.q_deg
        cur = target.q_cmd if target.q_cmd is not None else target.q_deg
        max_step = max(0.0, abs(target.dq_deg_s)) * self._period
        err = target.q_deg - cur
        if abs(err) <= max_step or max_step <= 0.0:
            cur = target.q_deg
        else:
            cur += max_step if err > 0 else -max_step
        target.q_cmd = cur
        return cur

    async def _send_cycle(self, can_bus: CanBus) -> None:
        """Send one impedance update for all DOFs."""
        for key, jcfg in self._config.joints.items():
            for dof in range(jcfg.dof_count):
                target = self.targets[key][dof]
                q_cmd = self._cmd_q(target)
                stiff_cmd = self._cmd_stiffness(key, dof, target, q_cmd)

                if self._gains_dirty[key][dof]:
                    await self._send_with_gains(can_bus, jcfg.joint_id, dof, target, q_cmd, stiff_cmd)
                    self._gains_dirty[key][dof] = False
                elif (key, dof) in self._tau_ff_armed:
                    await self._send_with_tau_ff(can_bus, jcfg.joint_id, dof, target, q_cmd, stiff_cmd)
                else:
                    await self._send_position_only(can_bus, jcfg.joint_id, dof, target, q_cmd, stiff_cmd)

    async def _send_position_only(self, can_bus: CanBus, joint_id: int,
                                  dof: int, target: ImpedanceTarget, q_cmd: float,
                                  stiff_cmd: float) -> None:
        """Send frame 0 only (regime operation)."""
        arb_id, data = encode_set_impedance_frame0(
            joint_id, dof,
            q_deg=q_cmd,
            dq_deg_s=target.dq_deg_s,
            stiffness_deg=stiff_cmd,
            has_more=False,
        )
        await can_bus.send(arb_id, data)

    async def _send_with_tau_ff(self, can_bus: CanBus, joint_id: int,
                                dof: int, target: ImpedanceTarget, q_cmd: float,
                                stiff_cmd: float) -> None:
        """Send frames 0+3 (armed tau_ff feedforward). frame0 carries has_more=1 so it OPENS
        the firmware staging transaction; frame3 (has_more=0 by construction in the encoder)
        CLOSES it, committing {q, dq, stiffness, tau_ff} atomically and refreshing the
        impedance watchdog. Sent via send_impedance_sequence for the proven 3 ms inter-frame
        pacing (fits the MCP2515 2-deep RX buffer)."""
        frames = [
            encode_set_impedance_frame0(
                joint_id, dof,
                q_deg=q_cmd,
                dq_deg_s=target.dq_deg_s,
                stiffness_deg=stiff_cmd,
                has_more=True,
            ),
            encode_set_impedance_frame3(
                joint_id, dof,
                tau_ff=target.tau_ff,
            ),
        ]
        await can_bus.send_impedance_sequence(frames)

    async def _send_with_gains(self, can_bus: CanBus, joint_id: int,
                               dof: int, target: ImpedanceTarget, q_cmd: float,
                               stiff_cmd: float) -> None:
        """Send frames 0+1+2+3 (gain change + tau_ff reset)."""
        tau_ff = getattr(target, 'tau_ff', 0)
        frames = [
            encode_set_impedance_frame0(
                joint_id, dof,
                q_deg=q_cmd,
                dq_deg_s=target.dq_deg_s,
                stiffness_deg=stiff_cmd,
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

    def set_tau_ff(self, joint: str, dof: int, raw_units: Optional[float]) -> int:
        """Arm/update/disarm the per-cycle feedforward-torque channel for a DOF.

        raw_units is in RAW motor-iq units (int16 on the wire, SET_IMPEDANCE frame3). The
        firmware applies it antisymmetrically (uff_A += tau_ff, uff_B -= tau_ff => zero net
        co-contraction change) upstream of torque saturation + rate limit. Host-clamped to
        +-TAU_FF_ABS_MAX_RAW HERE, before any encode — the firmware does NOT clamp tau_ff
        at commit.

        None DISARMS: tau_ff is zeroed and the DOF is marked gains-dirty so exactly ONE
        4-frame transaction (the existing verified gains path, which already terminates
        with frame3) commits the zero; the stream then reverts to the byte-identical
        frame0-only path. Returns the value actually armed (0 on disarm).
        """
        t = self.targets[joint][dof]
        if raw_units is None:
            t.tau_ff = 0
            self._tau_ff_armed.discard((joint, dof))
            self._gains_dirty[joint][dof] = True
            return 0
        f = float(raw_units)
        if not math.isfinite(f):
            # Fail LOUD at the right layer: int(round(nan/inf)) below would raise a
            # cryptic ValueError/OverflowError from inside the rounding, and a
            # non-finite feedforward must never be silently clamped onto the wire.
            raise ValueError(f"tau_ff must be finite (got {raw_units!r})")
        v = int(round(f))
        v = max(-TAU_FF_ABS_MAX_RAW, min(TAU_FF_ABS_MAX_RAW, v))
        t.tau_ff = v
        self._tau_ff_armed.add((joint, dof))
        return v

    def set_all_targets(self, q_deg: float, dq_deg_s: float = 5.0) -> None:
        """Set the same target for all DOFs (useful for synchronized homing)."""
        for key in self.targets:
            for dof in self.targets[key]:
                self.targets[key][dof].q_deg = q_deg
                self.targets[key][dof].dq_deg_s = dq_deg_s
