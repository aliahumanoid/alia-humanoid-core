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
    RECOVERY_SETTLE = "RECOVERY_SETTLE"
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
            all_ready = self._all_joints_ready(config, telemetry)
            if all_ready:
                self._set_state(FSMState.DISCOVERED,
                                "Joints already active — press [S] to resume control")
            else:
                self._set_state(FSMState.DISCOVERED,
                                "Press [S] to start motors")
            return True
        except StartupError as e:
            self._set_state(FSMState.ERROR, str(e))
            logger.error(f"Discover failed: {e}")
            return False

    async def run_startup(self, can_bus: CanBus, config: ControllerConfig,
                          telemetry: TelemetryManager,
                          safety: SafetyManager,
                          startup_torque: int = 0,
                          startup_duration: int = 0) -> bool:
        """Run startup sequence (pretension → startup → stream → gains → home).

        Expects joints to be already discovered.
        """
        try:
            all_ready_on_entry = self._all_joints_ready(config, telemetry)
            telemetry_estop_latched = (
                all_ready_on_entry
                and self._has_active_estop_latch(config, telemetry)
            )
            recovering_after_estop = all_ready_on_entry and (
                safety.estop_latched or telemetry_estop_latched
            )
            self._assert_no_blocking_diagnostics(
                config,
                telemetry,
                safety,
                mode="prestart",
                allow_estop_recovery=recovering_after_estop,
                context="Startup precheck",
            )
            await self._startup(
                can_bus,
                config,
                telemetry,
                startup_torque=startup_torque,
                startup_duration=startup_duration,
            )

            # After a host-side E-stop the firmware keeps announcing ready=True,
            # but motor-moving commands stay rejected until PRETENSION(_ALL)
            # re-enables motor power. Recover only in that specific path.
            if recovering_after_estop:
                await self._recover_ready_joints_after_estop(can_bus, config)

            await self._stream(can_bus, config, telemetry)
            await self._init_gains(can_bus, config, telemetry, safety)
            await asyncio.sleep(0.15)
            if all_ready_on_entry and not recovering_after_estop:
                if await self._wait_for_active_estop_latch(config, telemetry):
                    await self._recover_ready_joints_after_estop(can_bus, config)
                    recovering_after_estop = True
            self._assert_no_blocking_diagnostics(
                config,
                telemetry,
                safety,
                mode="movement",
                allow_estop_recovery=False,
                allow_session_resume=all_ready_on_entry,
                context="Movement enable",
            )

            if recovering_after_estop and config.startup_recovery_settle_s > 0.0:
                await self._settle_current_pose(
                    can_bus,
                    config,
                    telemetry,
                    duration_s=config.startup_recovery_settle_s,
                )

            # Post-E-stop / reconnect case: all joints are already in HOLD and
            # announce ready=True. Re-homing here just drives them back to the
            # configured home pose, which is not what the user expects from a
            # "resume" startup. Keep the current pose and let the host re-seed
            # the impedance targets from live telemetry before restarting the loop.
            if all_ready_on_entry:
                self._set_state(FSMState.READY, "All joints already ready; resuming current pose")
                safety.clear_estop_latch()
                return True

            await self._home(can_bus, config, telemetry)
            await asyncio.sleep(0.15)
            self._assert_no_blocking_diagnostics(
                config,
                telemetry,
                safety,
                mode="movement",
                allow_estop_recovery=False,
                context="Homing completion",
            )

            self._set_state(FSMState.READY, "All joints at home position")
            safety.clear_estop_latch()
            return True

        except StartupError as e:
            self._set_state(FSMState.ERROR, str(e))
            logger.error(f"Startup failed: {e}")
            return False

    @staticmethod
    def _all_joints_ready(config: ControllerConfig, telemetry: TelemetryManager) -> bool:
        for key in config.joints:
            state = telemetry.states.get(key)
            if state is None or state.announce is None or not state.announce.ready:
                return False
        return True

    @staticmethod
    def _has_active_estop_latch(config: ControllerConfig,
                                telemetry: TelemetryManager) -> bool:
        for key in config.joints:
            state = telemetry.states.get(key)
            if state is None:
                continue
            fault = getattr(state, "fault_status", None)
            if fault is None:
                continue
            if "ESTOP_LATCHED" in getattr(fault, "active_fault_names", []):
                return True
        return False

    @staticmethod
    def _active_fault_names(config: ControllerConfig,
                            telemetry: TelemetryManager) -> set[str]:
        active: set[str] = set()
        for key in config.joints:
            state = telemetry.states.get(key)
            if state is None:
                continue
            fault = getattr(state, "fault_status", None)
            if fault is None:
                continue
            active.update(getattr(fault, "active_fault_names", []))
        return active

    async def _wait_for_active_estop_latch(self, config: ControllerConfig,
                                           telemetry: TelemetryManager,
                                           timeout_s: float = 1.2) -> bool:
        """Allow one diagnostic cycle to reveal a latent ESTOP_LATCHED fault.

        When reconnecting to a controller that still announces ``ready=True``,
        fault telemetry can lag behind JOINT_ANNOUNCE by up to about one second.
        Give the controller a short window to publish ESTOP_LATCHED before
        assuming the resume path is safe.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._has_active_estop_latch(config, telemetry):
                return True
            await asyncio.sleep(0.05)
        return self._has_active_estop_latch(config, telemetry)

    async def _wait_for_faults_clear(self, config: ControllerConfig,
                                     telemetry: TelemetryManager,
                                     fault_names: set[str],
                                     timeout_s: float) -> bool:
        """Wait for a set of active fault names to clear from telemetry."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            active = self._active_fault_names(config, telemetry)
            if not (active & fault_names):
                return True
            await asyncio.sleep(0.05)
        return not (self._active_fault_names(config, telemetry) & fault_names)

    @staticmethod
    def _assert_no_blocking_diagnostics(
        config: ControllerConfig,
        telemetry: TelemetryManager,
        safety: SafetyManager,
        *,
        mode: str,
        allow_estop_recovery: bool,
        allow_session_resume: bool = False,
        context: str,
    ) -> None:
        blockers = safety.diagnostic_blockers(
            telemetry,
            joint_keys=config.joints.keys(),
            mode=mode,
            allow_estop_recovery=allow_estop_recovery,
            allow_session_resume=allow_session_resume,
        )
        if blockers:
            raise StartupError(
                f"{context} blocked by diagnostics: {safety.format_diagnostic_blockers(blockers)}"
            )

    async def _recover_ready_joints_after_estop(self, can_bus: CanBus,
                                                config: ControllerConfig) -> None:
        self._set_state(FSMState.STARTUP, "Recovering motor power after E-stop")
        for key, jcfg in config.joints.items():
            arb_id, data = encode_pretension_all(jcfg.joint_id)
            await can_bus.send(arb_id, data)
            logger.info(f"Recovery PRETENSION_ALL sent for {key}")
            await asyncio.sleep(0.10)
        await asyncio.sleep(0.20)

    async def _settle_current_pose(self, can_bus: CanBus, config: ControllerConfig,
                                   telemetry: TelemetryManager,
                                   duration_s: float) -> None:
        self._set_state(
            FSMState.RECOVERY_SETTLE,
            f"Holding current pose for {duration_s:.0f}s after E-stop recovery",
        )

        hold_targets: dict[str, dict[int, float]] = {}
        for key, jcfg in config.joints.items():
            hold_targets[key] = {}
            state = telemetry.states.get(key)
            for dof in range(jcfg.dof_count):
                q_hold = 0.0
                if state is not None and dof in state.angles_deg:
                    q_hold = state.angles_deg[dof]
                hold_targets[key][dof] = q_hold

        deadline = time.monotonic() + duration_s
        period = 1.0 / config.send_rate_hz
        while time.monotonic() < deadline:
            for key, jcfg in config.joints.items():
                for dof, q_hold in hold_targets[key].items():
                    arb_id, data = encode_set_impedance_frame0(
                        jcfg.joint_id,
                        dof,
                        q_deg=q_hold,
                        dq_deg_s=0.0,
                        stiffness_deg=jcfg.stiffness_for(dof),
                        has_more=False,
                    )
                    await can_bus.send(arb_id, data)
            await asyncio.sleep(period)

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
                       telemetry: TelemetryManager,
                       startup_torque: int = 0,
                       startup_duration: int = 0) -> None:
        self._set_state(FSMState.STARTUP, "Running startup sequence")

        for key, jcfg in config.joints.items():
            state = telemetry.states.get(key)
            if state and state.announce and state.announce.ready:
                logger.info(f"Skipping startup for {key}: joint already ready")
                self._set_state(FSMState.STARTUP, f"{key} already ready")
                continue

            logger.info(f"Starting up {key} (id={jcfg.joint_id})")

            # Optional recovery step: re-enable motor power after a previous E-stop.
            # Keep disabled by default so the startup path matches the webapp's
            # CAN startup sequence more closely while debugging controller issues.
            if config.startup_pretension_all:
                arb_id, data = encode_pretension_all(jcfg.joint_id)
                await can_bus.send(arb_id, data)
                # Give the controller one diagnostic cycle to publish any
                # post-pretension transient faults before deciding startup is safe.
                await asyncio.sleep(0.15)
                transient_faults = {"ESTOP_LATCHED", "MOTOR_TIMEOUT"}
                if not await self._wait_for_faults_clear(
                    config,
                    telemetry,
                    transient_faults,
                    timeout_s=1.0,
                ):
                    still_active = sorted(
                        self._active_fault_names(config, telemetry) & transient_faults
                    )
                    fault_text = ",".join(still_active) if still_active else "unknown"
                    raise StartupError(
                        f"{key} did not recover after PRETENSION_ALL: active={fault_text}. "
                        "Check motor CAN bus, power stage, and motor responsiveness."
                    )
                await asyncio.sleep(0.05)
            else:
                logger.info(f"Skipping PRETENSION_ALL before startup for {key}")

            # Reset event for this joint
            evt = telemetry.startup_events.get(jcfg.joint_id)
            if evt:
                evt.clear()

            if startup_torque > 0 or startup_duration > 0:
                logger.info(
                    "Startup override for %s: torque=%d duration=%dms",
                    key,
                    startup_torque,
                    startup_duration,
                )
            arb_id, data = encode_startup_sequence(
                jcfg.joint_id,
                torque=startup_torque,
                duration=startup_duration,
            )
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
                if result.reason_name == "REFERENCE_REQUIRED":
                    raise StartupError(
                        f"Startup failed for {key}: DOF{result.dof_index} requires a saved "
                        f"direct-drive reference. Set Reference for that DOF, then re-run startup."
                    )
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
                outer_gains = jcfg.outer_gains_for(dof)
                inner_gains = jcfg.inner_gains_for(dof)
                stiffness_deg = jcfg.stiffness_for(dof)
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
                        stiffness_deg=stiffness_deg,
                        has_more=True,
                    ),
                    encode_set_impedance_frame1(
                        jcfg.joint_id, dof,
                        kp=outer_gains.kp,
                        ki=outer_gains.ki,
                        kd=outer_gains.kd,
                        has_more=True,
                    ),
                    encode_set_impedance_frame2(
                        jcfg.joint_id, dof,
                        kp_inner=inner_gains.kp,
                        ki_inner=inner_gains.ki,
                        kd_inner=inner_gains.kd,
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
                    f"outer=({outer_gains.kp}/{outer_gains.ki}/{outer_gains.kd}), "
                    f"inner=({inner_gains.kp}/{inner_gains.ki}/{inner_gains.kd}), "
                    f"stiffness={stiffness_deg}"
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
                    stiffness_deg=jcfg.stiffness_for(dof),
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
                        stiffness_deg=jcfg.stiffness_for(dof),
                        has_more=False,
                    )
                    await can_bus.send(arb_id, data)
                    last_arrived_keepalive[(key, dof)] = now

            await asyncio.sleep(period)

        raise StartupError("Homing timeout — joints did not reach target")
