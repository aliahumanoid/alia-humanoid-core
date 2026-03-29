"""
Startup finite state machine.

Sequence: INIT → CAN_CONNECT → DISCOVER → STARTUP → STREAM → INIT_GAINS → HOME → READY

Each state transitions on success or raises on timeout/failure.
The TUI is notified of state changes for display.
"""
from __future__ import annotations

import asyncio
import enum
import logging
import time

from .can_bus import CanBus
from .config import ControllerConfig
from .protocol import (
    encode_encoder_stream_ctrl,
    encode_identify_request,
    encode_pretension_all,
    encode_set_impedance_frame0,
    encode_set_impedance_frame1,
    encode_set_impedance_frame2,
    encode_set_impedance_frame3,
    encode_startup_sequence,
    encode_time_sync,
)
from .safety import SafetyManager
from .telemetry import TelemetryManager

logger = logging.getLogger(__name__)


class FSMState(enum.Enum):
    INIT = "INIT"
    CAN_CONNECT = "CAN_CONNECT"
    DISCOVER = "DISCOVER"
    DISCOVERED = "DISCOVERED"
    STARTUP = "STARTUP"
    STREAM = "STREAM"
    INIT_GAINS = "INIT_GAINS"
    HOME = "HOME"
    READY = "READY"
    ERROR = "ERROR"


class StartupError(Exception):
    """Non-recoverable startup failure."""
    pass


class StartupFSM:
    """Run through the startup sequence to bring joints to READY state."""

    def __init__(self):
        self.state = FSMState.INIT
        self._state_callback = None  # optional TUI callback

    def on_state_change(self, callback) -> None:
        """Register callback(state: FSMState, message: str)."""
        self._state_callback = callback

    def _set_state(self, state: FSMState, msg: str = "") -> None:
        self.state = state
        logger.info(f"FSM → {state.value}: {msg}")
        if self._state_callback:
            self._state_callback(state, msg)

    async def run(self, can_bus: CanBus, config: ControllerConfig,
                  telemetry: TelemetryManager, safety: SafetyManager) -> bool:
        """Execute full startup sequence. Returns True if READY reached."""
        if not await self.run_discover(can_bus, config, telemetry):
            return False
        return await self.run_startup(can_bus, config, telemetry, safety)

    async def run_discover(self, can_bus: CanBus, config: ControllerConfig,
                           telemetry: TelemetryManager) -> bool:
        """Connect and discover joints. Returns True on success.

        After success the FSM is in DISCOVERED state, waiting for the
        user to press [S] before motors are enabled.
        """
        try:
            await self._connect(can_bus, config)
            await self._discover(can_bus, config, telemetry)
            self._set_state(FSMState.DISCOVERED,
                            "Press [S] to start motors")
            return True
        except StartupError as e:
            self._set_state(FSMState.ERROR, str(e))
            logger.error(f"Discover failed: {e}")
            return False

    async def run_startup(self, can_bus: CanBus, config: ControllerConfig,
                          telemetry: TelemetryManager,
                          safety: SafetyManager) -> bool:
        """Run startup sequence (pretension → startup → stream → gains → home).

        Expects joints to be already discovered.
        """
        try:
            await self._startup(can_bus, config, telemetry)
            await self._stream(can_bus, config, telemetry)
            await self._init_gains(can_bus, config, telemetry, safety)
            await self._home(can_bus, config, telemetry)

            self._set_state(FSMState.READY, "All joints at home position")
            return True

        except StartupError as e:
            self._set_state(FSMState.ERROR, str(e))
            logger.error(f"Startup failed: {e}")
            return False

    # ------------------------------------------------------------------
    # State implementations
    # ------------------------------------------------------------------

    async def _connect(self, can_bus: CanBus, config: ControllerConfig) -> None:
        if can_bus.connected:
            self._set_state(FSMState.CAN_CONNECT, "CAN already connected, skipping")
            return
        self._set_state(FSMState.CAN_CONNECT, "Opening CAN interface")
        try:
            await can_bus.connect(
                interface=config.can_interface,
                channel=config.can_channel,
                bitrate=config.can_bitrate,
            )
        except Exception as e:
            raise StartupError(f"CAN connection failed: {e}") from e

    async def _discover(self, can_bus: CanBus, config: ControllerConfig,
                        telemetry: TelemetryManager) -> None:
        self._set_state(FSMState.DISCOVER, "Discovering joints")
        telemetry.unexpected_announces.clear()

        # Send time sync first
        arb_id, data = encode_time_sync()
        await can_bus.send(arb_id, data)
        await asyncio.sleep(0.05)

        # Broadcast identify request
        arb_id, data = encode_identify_request()
        await can_bus.send(arb_id, data)

        # Wait for announces from all expected joints
        expected = {jcfg.joint_id: key for key, jcfg in config.joints.items()}
        timeout = 5.0
        discovered = []

        for joint_id, joint_key in expected.items():
            evt = telemetry.announce_events.get(joint_id)
            if evt is None:
                continue
            try:
                await asyncio.wait_for(evt.wait(), timeout=timeout)
                discovered.append(joint_key)
                logger.info(f"Discovered {joint_key} (id={joint_id})")
            except asyncio.TimeoutError:
                unexpected_ids = sorted(telemetry.unexpected_announces.keys())
                mismatch_hint = ""
                if unexpected_ids:
                    mismatch_hint = (
                        f"; saw unexpected joint ids {unexpected_ids} instead. "
                        "Check controller.yaml / --joint selection against board provisioning."
                    )
                raise StartupError(
                    f"Joint {joint_key} (id={joint_id}) did not respond "
                    f"within {timeout}s{mismatch_hint}"
                )

        self._set_state(FSMState.DISCOVER,
                        f"Discovered {len(discovered)}/{len(expected)} joints")

    async def _startup(self, can_bus: CanBus, config: ControllerConfig,
                       telemetry: TelemetryManager) -> None:
        self._set_state(FSMState.STARTUP, "Running startup sequence")

        for key, jcfg in config.joints.items():
            logger.info(f"Starting up {key} (id={jcfg.joint_id})")

            # Re-enable motor power (clears post-e-stop lockout)
            arb_id, data = encode_pretension_all(jcfg.joint_id)
            await can_bus.send(arb_id, data)
            await asyncio.sleep(0.1)  # Let firmware process pretension

            # Reset event for this joint
            evt = telemetry.startup_events.get(jcfg.joint_id)
            if evt:
                evt.clear()

            arb_id, data = encode_startup_sequence(jcfg.joint_id)
            await can_bus.send(arb_id, data)

            # Wait for COMPLETE or FAILED
            timeout = 15.0  # Generous — recalc can take time with multiple DOFs
            try:
                await asyncio.wait_for(evt.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                raise StartupError(
                    f"Startup timeout for {key} after {timeout}s"
                )

            result = telemetry.startup_results.get(jcfg.joint_id)
            if result and result.is_failed:
                raise StartupError(
                    f"Startup failed for {key}: {result.event_name} "
                    f"DOF={result.dof_index} reason={result.reason_name}"
                )
            if result and result.is_partial:
                raise StartupError(
                    f"Startup partial for {key}: some DOFs have no hold position "
                    f"(encoder invalid) — joint not safe for movement"
                )

            self._set_state(FSMState.STARTUP, f"{key} ready")

    async def _stream(self, can_bus: CanBus, config: ControllerConfig,
                      telemetry: TelemetryManager) -> None:
        self._set_state(FSMState.STREAM, "Starting encoder stream")

        arb_id, data = encode_encoder_stream_ctrl(start=True)
        await can_bus.send(arb_id, data)

        # Wait for encoder data to arrive from all joints
        timeout = 3.0
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            all_online = all(
                s.rx_count > 0 for s in telemetry.states.values()
            )
            if all_online:
                self._set_state(FSMState.STREAM, "Encoder stream active")
                return
            await asyncio.sleep(0.1)

        # Check which joints are missing
        missing = [k for k, s in telemetry.states.items() if s.rx_count == 0]
        raise StartupError(f"No encoder data from: {missing}")

    async def _init_gains(self, can_bus: CanBus, config: ControllerConfig,
                          telemetry: TelemetryManager,
                          safety: SafetyManager) -> None:
        self._set_state(FSMState.INIT_GAINS, "Sending initial gains")

        for key, jcfg in config.joints.items():
            # Configure impedance watchdog
            await safety.configure_watchdog(
                can_bus, jcfg.joint_id, config.watchdog_ms
            )
            await asyncio.sleep(0.01)

            for dof in range(jcfg.dof_count):
                # Get current position from encoder (hold in place)
                state = telemetry.states.get(key)
                current_angle = 0.0
                if state and dof in state.angles_deg:
                    current_angle = state.angles_deg[dof]

                # Build 4-frame sequence: position + outer gains + inner gains + tau_ff
                # Frame 3 explicitly sets tau_ff=0 to clear any stale feedforward
                # from a previous session (webapp or earlier Jetson run).
                frames = [
                    encode_set_impedance_frame0(
                        jcfg.joint_id, dof,
                        q_deg=current_angle, dq_deg_s=0.0,
                        stiffness_deg=jcfg.stiffness_deg,
                        has_more=True,
                    ),
                    encode_set_impedance_frame1(
                        jcfg.joint_id, dof,
                        kp=jcfg.gains_outer.kp,
                        ki=jcfg.gains_outer.ki,
                        kd=jcfg.gains_outer.kd,
                        has_more=True,
                    ),
                    encode_set_impedance_frame2(
                        jcfg.joint_id, dof,
                        kp_inner=jcfg.gains_inner.kp,
                        ki_inner=jcfg.gains_inner.ki,
                        kd_inner=jcfg.gains_inner.kd,
                        has_more=True,
                    ),
                    encode_set_impedance_frame3(
                        jcfg.joint_id, dof,
                        tau_ff=0,  # Last frame — triggers apply
                    ),
                ]
                await can_bus.send_impedance_sequence(frames)

                logger.info(
                    f"Init gains [{key}] DOF{dof}: hold@{current_angle:.1f}deg, "
                    f"outer=({jcfg.gains_outer.kp}/{jcfg.gains_outer.ki}/{jcfg.gains_outer.kd}), "
                    f"inner=({jcfg.gains_inner.kp}/{jcfg.gains_inner.ki}/{jcfg.gains_inner.kd})"
                )
                await asyncio.sleep(0.02)  # Small gap between DOFs

        self._set_state(FSMState.INIT_GAINS, "Gains initialized, impedance active")

    async def _home(self, can_bus: CanBus, config: ControllerConfig,
                    telemetry: TelemetryManager) -> None:
        self._set_state(FSMState.HOME, "Moving to home position")

        # Build target map for convergence check
        targets: dict[str, dict[int, float]] = {}

        for key, jcfg in config.joints.items():
            targets[key] = {}
            for dof, target_deg in jcfg.home_position_deg.items():
                targets[key][dof] = target_deg

                # Send frame 0 only (gains already set)
                arb_id, data = encode_set_impedance_frame0(
                    jcfg.joint_id, dof,
                    q_deg=target_deg,
                    dq_deg_s=config.homing_speed_deg_s,
                    stiffness_deg=jcfg.stiffness_deg,
                    has_more=False,
                )
                await can_bus.send(arb_id, data)
                await asyncio.sleep(0.005)  # Brief gap between DOFs

        # Wait for convergence
        timeout = 30.0  # Generous for slow homing
        deadline = time.monotonic() + timeout
        period = 1.0 / config.send_rate_hz
        arrived_keepalive_period = min(0.2, max(period, config.watchdog_ms / 1000.0 / 4.0))
        last_arrived_keepalive: dict[tuple[str, int], float] = {}

        while time.monotonic() < deadline:
            if telemetry.all_at_target(targets, config.homing_tolerance_deg):
                self._set_state(FSMState.HOME, "Home position reached")
                return

            arrived_map = telemetry.dofs_at_target(targets, config.homing_tolerance_deg)
            now = time.monotonic()

            # Keep active DOFs at full rate, but refresh already-arrived DOFs more slowly.
            # This matches the webapp behavior and avoids unnecessary CAN traffic while
            # another DOF/controller is still converging.
            for key, jcfg in config.joints.items():
                for dof, target_deg in jcfg.home_position_deg.items():
                    if arrived_map.get(key, {}).get(dof, False):
                        last_sent = last_arrived_keepalive.get((key, dof), 0.0)
                        if (now - last_sent) < arrived_keepalive_period:
                            continue

                    arb_id, data = encode_set_impedance_frame0(
                        jcfg.joint_id, dof,
                        q_deg=target_deg,
                        dq_deg_s=config.homing_speed_deg_s,
                        stiffness_deg=jcfg.stiffness_deg,
                        has_more=False,
                    )
                    await can_bus.send(arb_id, data)
                    last_arrived_keepalive[(key, dof)] = now

            await asyncio.sleep(period)

        raise StartupError("Homing timeout — joints did not reach target")
