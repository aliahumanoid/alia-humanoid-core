"""
Entry point for the Alia Jetson controller.

Usage:
    python -m jetson_controller
    python -m jetson_controller --config path/to/controller.yaml
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from .can_bus import CanBus
from .config import load_config
from .fsm import StartupFSM
from .impedance_loop import ImpedanceLoop
from .protocol import encode_fault_snapshot_ctrl
from .safety import SafetyManager
from .serial_monitor import discover_ports, preflight_boot
from .session_log import setup_session_logging, get_log_path
from .telemetry import TelemetryManager
from .tui import TUI

logger = logging.getLogger("jetson_controller")


def setup_logging(verbose: bool = False) -> None:
    # Root logger accepts everything; handlers decide what to show
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # Console handler: INFO always (verbose flag does NOT dump DEBUG to terminal)
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    ))
    root.addHandler(console)

    # Quiet noisy libraries
    logging.getLogger("can").setLevel(logging.WARNING)

    # Unified session log file: captures DEBUG (50 Hz frames etc.)
    # Console stays at INFO — no flooding.
    setup_session_logging(verbose, clear=True)
    logger.info(f"Session log: {get_log_path()}")


async def main(config_path: str = None, verbose: bool = False,
               selected_joints: list[str] | None = None,
               preflight_auto: bool = False,
               preflight_serials: list[str] | None = None) -> int:
    setup_logging(verbose)

    # Load configuration
    try:
        config = load_config(config_path, selected_joints=selected_joints)
    except (FileNotFoundError, ValueError) as e:
        logger.error(f"Configuration error: {e}")
        return 1

    logger.info(f"Loaded config: {len(config.joints)} joints, "
                f"CAN={config.can_interface}:{config.can_channel}")
    for key, jcfg in config.joints.items():
        logger.info(f"  {key}: id={jcfg.joint_id}, dofs={jcfg.dof_count}")

    preflight_ports: list[str] = []
    if preflight_serials:
        for port in preflight_serials:
            if port not in preflight_ports:
                preflight_ports.append(port)
    if preflight_auto:
        auto_ports = discover_ports(verbose_output=False)
        if not auto_ports:
            logger.warning("Serial preflight requested, but no Pico USB serial ports were auto-discovered")
        for port in auto_ports:
            if port not in preflight_ports:
                preflight_ports.append(port)

    if preflight_ports:
        logger.info(f"Serial preflight on {len(preflight_ports)} port(s): {', '.join(preflight_ports)}")
        try:
            for port in preflight_ports:
                preflight_boot(port)
                await asyncio.sleep(0.1)
        except Exception as exc:
            logger.error(f"Serial preflight failed: {exc}")
            return 1

    # Initialize components
    can_bus = CanBus()
    telemetry = TelemetryManager(config)
    safety = SafetyManager()
    impedance = ImpedanceLoop(config)
    tui = TUI(config, telemetry, impedance, can_bus)

    # Wire up FSM state → TUI
    fsm = StartupFSM()
    fsm.on_state_change(tui.set_fsm_state)

    # ------------------------------------------------------------------
    # Key command callbacks
    # ------------------------------------------------------------------

    async def on_estop():
        await impedance.stop()
        await safety.send_estop(can_bus)

    async def on_home():
        for key, jcfg in config.joints.items():
            for dof, target in jcfg.home_position_deg.items():
                impedance.set_target(key, dof, target, config.homing_speed_deg_s)

    async def on_startup():
        """Run startup sequence (pretension → startup → stream → gains → home).

        Re-discovers joints first, then runs motor startup.
        """
        if not can_bus.connected:
            logger.warning("Cannot startup: CAN not connected")
            return
        # Stop impedance loop first — quiet the bus before re-init
        await impedance.stop()
        # Reset telemetry events for fresh detection
        for jcfg in config.joints.values():
            evt = telemetry.announce_events.get(jcfg.joint_id)
            if evt:
                evt.clear()
            evt = telemetry.startup_events.get(jcfg.joint_id)
            if evt:
                evt.clear()
        new_fsm = StartupFSM()
        new_fsm.on_state_change(tui.set_fsm_state)
        # Re-discover then startup
        if not await new_fsm.run_discover(can_bus, config, telemetry):
            return
        all_ready_on_entry = all(
            (state is not None and state.announce is not None and state.announce.ready)
            for state in (telemetry.states.get(key) for key in config.joints)
        )
        ready = await new_fsm.run_startup(can_bus, config, telemetry, safety)
        if ready:
            if all_ready_on_entry:
                for key, jcfg in config.joints.items():
                    state = telemetry.states.get(key)
                    for dof in range(jcfg.dof_count):
                        current_q = 0.0
                        if state is not None and dof in state.angles_deg:
                            current_q = state.angles_deg[dof]
                        impedance.set_target(key, dof, current_q, dq_deg_s=0.0)
                        impedance.set_stiffness(key, dof, jcfg.stiffness_for(dof))
                logger.info("Startup resume: seeded impedance targets from current pose")
            impedance.start(can_bus)

    async def on_discover():
        """Send time_sync + identify_request only (quick connectivity check)."""
        if not can_bus.connected:
            logger.warning("Cannot discover: CAN not connected")
            return
        from .protocol import encode_time_sync, encode_identify_request
        arb_id, data = encode_time_sync()
        await can_bus.send(arb_id, data)
        await asyncio.sleep(0.05)
        arb_id, data = encode_identify_request()
        await can_bus.send(arb_id, data)
        logger.info("Discover sent (time_sync + identify_request)")

    async def on_nudge(delta_deg: float, joint_key: str, dof: int):
        """Nudge one selected DOF by delta_deg from current target."""
        jcfg = config.joints[joint_key]
        target = impedance.targets[joint_key][dof]
        new_q = target.q_deg + delta_deg
        min_a = jcfg.min_angles.get(dof, -180.0)
        max_a = jcfg.max_angles.get(dof, 180.0)
        clamped_q = max(min_a, min(max_a, new_q))
        impedance.set_target(joint_key, dof, clamped_q, config.nudge_speed_deg_s)
        dof_label = jcfg.dof_names[dof] if dof < len(jcfg.dof_names) else f"dof_{dof}"
        if abs(clamped_q - new_q) > 1e-6:
            logger.info(
                f"Nudge {joint_key} DOF{dof} ({dof_label}) {delta_deg:+.1f}° "
                f"clamped to {clamped_q:+.1f}° within [{min_a:+.1f}°, {max_a:+.1f}°]"
            )
        else:
            logger.info(
                f"Nudge {joint_key} DOF{dof} ({dof_label}) {delta_deg:+.1f}° -> {clamped_q:+.1f}°"
            )

    snapshot_seq = 0

    def _next_snapshot_seq() -> int:
        nonlocal snapshot_seq
        snapshot_seq = (snapshot_seq + 1) & 0xFF
        return snapshot_seq

    async def on_snapshot(joint_key: str):
        """Query metadata and, if available, request a fault snapshot dump."""
        if not can_bus.connected:
            logger.warning("Cannot request fault snapshot: CAN not connected")
            return

        jcfg = config.joints[joint_key]
        state = telemetry.states.get(joint_key)
        previous_meta = getattr(state, "fault_snapshot_meta", None) if state is not None else None
        previous_timestamp = (
            previous_meta.get("timestamp")
            if isinstance(previous_meta, dict)
            else None
        )

        arb_id, data = encode_fault_snapshot_ctrl(
            jcfg.joint_id,
            sub_cmd=0x00,
            seq=_next_snapshot_seq(),
        )
        await can_bus.send(arb_id, data)
        logger.info(f"Fault snapshot query sent for {joint_key}")

        deadline = asyncio.get_running_loop().time() + 0.25
        meta_payload = previous_meta if isinstance(previous_meta, dict) else None
        while asyncio.get_running_loop().time() < deadline:
            state = telemetry.states.get(joint_key)
            current_meta = getattr(state, "fault_snapshot_meta", None) if state is not None else None
            if isinstance(current_meta, dict):
                meta_payload = current_meta
                if current_meta.get("timestamp") != previous_timestamp:
                    break
            await asyncio.sleep(0.05)

        if not isinstance(meta_payload, dict):
            logger.warning(f"No fault snapshot metadata available for {joint_key}")
            return

        meta_state = meta_payload.get("state", {})
        if not isinstance(meta_state, dict) or not meta_state.get("snapshot_present"):
            logger.info(f"No stored fault snapshot available for {joint_key}")
            return

        snapshot_id = int(meta_payload.get("snapshot_id", 0xFF))
        arb_id, data = encode_fault_snapshot_ctrl(
            jcfg.joint_id,
            sub_cmd=0x01,
            snapshot_id=snapshot_id,
            arg0=0,
            seq=_next_snapshot_seq(),
        )
        await can_bus.send(arb_id, data)
        logger.info(f"Fault snapshot dump requested for {joint_key} snapshot={snapshot_id}")

    async def on_toggle_loop():
        """Pause/resume the impedance loop."""
        if impedance.running:
            await impedance.stop()
            logger.info("Impedance loop PAUSED")
        else:
            impedance.start(can_bus)
            logger.info("Impedance loop RESUMED")

    tui.on_estop(on_estop)
    tui.on_home(on_home)
    tui.on_startup(on_startup)
    tui.on_discover(on_discover)
    tui.on_nudge(on_nudge)
    tui.on_snapshot(on_snapshot)
    tui.on_toggle_loop(on_toggle_loop)

    # ------------------------------------------------------------------
    # Main lifecycle
    # ------------------------------------------------------------------

    # Start background tasks
    telemetry_task = asyncio.create_task(telemetry.listen(can_bus))
    tui_task = asyncio.create_task(tui.run())

    exit_code = 0
    try:
        # Phase 1: Connect CAN + discover joints (safe, no motor movement)
        discovered = await fsm.run_discover(can_bus, config, telemetry)

        if discovered:
            # Wait for user to press [S] to start motors
            logger.info("Joints discovered — press [S] to start motors")
        else:
            logger.error("Discovery failed — press Q to exit")
            exit_code = 1

        # Wait for quit (user presses [S] for startup or [Q] to exit)
        await tui.wait_for_quit()

        # Graceful shutdown
        await impedance.stop()

    except KeyboardInterrupt:
        logger.info("Interrupted")

    finally:
        # Always stop impedance and send E-stop on exit
        await impedance.stop()
        if can_bus.connected:
            await safety.send_estop(can_bus, reason=0)

        # Stop all tasks
        telemetry.stop()
        telemetry_task.cancel()
        tui_task.cancel()

        try:
            await telemetry_task
        except asyncio.CancelledError:
            pass
        try:
            await tui_task
        except asyncio.CancelledError:
            pass

        # Disconnect CAN
        await can_bus.disconnect()

    return exit_code


def cli():
    parser = argparse.ArgumentParser(
        description="Alia Jetson real-time controller"
    )
    parser.add_argument(
        "--config", "-c",
        help="Path to controller.yaml (default: config/controller.yaml)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable debug logging"
    )
    parser.add_argument(
        "--joint", "-j",
        action="append",
        help="Joint profile(s) to control, e.g. --joint knee_right --joint ankle_right"
    )
    parser.add_argument(
        "--preflight-auto",
        action="store_true",
        help="Open all auto-discovered Pico USB serial ports once before CAN startup, then close them"
    )
    parser.add_argument(
        "--preflight-serial",
        action="append",
        help="Open a specific board USB serial port once before CAN startup (repeatable)"
    )
    args = parser.parse_args()

    exit_code = asyncio.run(
        main(
            args.config,
            args.verbose,
            args.joint,
            args.preflight_auto,
            args.preflight_serial,
        )
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    cli()
