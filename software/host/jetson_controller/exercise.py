"""
Headless Jetson exercise runner.

Runs the normal CAN startup path, starts the impedance loop, then executes a
repeatable target pattern across the selected joints/DOFs for a fixed duration.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from .can_bus import CanBus
from .config import ControllerConfig, JointControlConfig, load_config
from .fsm import StartupFSM
from .impedance_loop import ImpedanceLoop
from .safety import SafetyManager
from .serial_monitor import discover_ports, preflight_boot
from .session_log import get_log_path, setup_session_logging
from .telemetry import TelemetryManager
from .protocol import encode_loop_frequency

logger = logging.getLogger("jetson_controller.exercise")


@dataclass(frozen=True)
class ExerciseStep:
    joint_key: str
    dof: int
    target_deg: float


@dataclass(frozen=True)
class ExerciseDofPlan:
    joint_key: str
    dof: int
    center_deg: float
    targets_deg: list[float]


def _write_run_report(path: str, payload: dict[str, object]) -> None:
    report_path = Path(path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def setup_logging(verbose: bool = False) -> None:
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    ))
    root.addHandler(console)

    logging.getLogger("can").setLevel(logging.WARNING)

    setup_session_logging(verbose, clear=True)
    logger.info(f"Session log: {get_log_path()}")


def _collect_preflight_ports(preflight_auto: bool,
                             preflight_serials: list[str] | None) -> list[str]:
    ports: list[str] = []
    if preflight_serials:
        for port in preflight_serials:
            if port not in ports:
                ports.append(port)
    if preflight_auto:
        for port in discover_ports(verbose_output=False):
            if port not in ports:
                ports.append(port)
    return ports


def _safe_window(jcfg: JointControlConfig, dof: int, margin_deg: float) -> tuple[float, float]:
    safe_min = float(jcfg.min_angles[dof]) + margin_deg
    safe_max = float(jcfg.max_angles[dof]) - margin_deg
    if safe_min > safe_max:
        raise ValueError(
            f"Invalid safe window for {jcfg.name} DOF{dof}: "
            f"min={safe_min:.1f} max={safe_max:.1f}"
        )
    return safe_min, safe_max


def _build_plans(config: ControllerConfig,
                 span_cap_deg: float,
                 margin_deg: float,
                 min_excursion_deg: float) -> list[ExerciseDofPlan]:
    plans: list[ExerciseDofPlan] = []

    for key, jcfg in config.joints.items():
        for dof in range(jcfg.dof_count):
            safe_min, safe_max = _safe_window(jcfg, dof, margin_deg)
            center = (safe_min + safe_max) / 2.0
            pos_span = max(0.0, min(span_cap_deg, safe_max - center))
            neg_span = max(0.0, min(span_cap_deg, center - safe_min))

            targets: list[float] = []
            if pos_span >= min_excursion_deg:
                targets.append(center + pos_span)
            if abs(center - (targets[-1] if targets else 1e9)) >= 0.1:
                targets.append(center)
            if neg_span >= min_excursion_deg:
                targets.append(center - neg_span)
                if abs(targets[-1] - center) >= 0.1:
                    targets.append(center)

            deduped: list[float] = []
            for target in targets:
                if not deduped or abs(deduped[-1] - target) >= 0.1:
                    deduped.append(target)

            if not deduped:
                continue

            plans.append(
                ExerciseDofPlan(
                    joint_key=key,
                    dof=dof,
                    center_deg=center,
                    targets_deg=deduped,
                )
            )

    return plans


def _interleave_steps(plans: list[ExerciseDofPlan]) -> list[ExerciseStep]:
    per_dof_sequences = [
        [ExerciseStep(plan.joint_key, plan.dof, target) for target in plan.targets_deg]
        for plan in plans
    ]
    interleaved: list[ExerciseStep] = []
    idx = 0
    while True:
        added = False
        for seq in per_dof_sequences:
            if idx < len(seq):
                interleaved.append(seq[idx])
                added = True
        if not added:
            break
        idx += 1

    return interleaved


async def _wait_for_target(telemetry: TelemetryManager, joint_key: str, dof: int,
                           target_deg: float, tolerance_deg: float,
                           timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    target_map = {joint_key: {dof: target_deg}}

    while time.monotonic() < deadline:
        if telemetry.dofs_at_target(target_map, tolerance_deg)[joint_key][dof]:
            return True
        await asyncio.sleep(0.05)
    return False


def _move_timeout(current_deg: float, target_deg: float,
                  speed_deg_s: float) -> float:
    travel = abs(target_deg - current_deg)
    return max(2.5, (travel / max(speed_deg_s, 1.0)) + 2.0)


async def run_exercise(config_path: str | None,
                       verbose: bool,
                       selected_joints: list[str] | None,
                       preflight_auto: bool,
                       preflight_serials: list[str] | None,
                       pretension_before_startup: bool,
                       duration_s: float,
                       dwell_s: float,
                       span_cap_deg: float,
                       margin_deg: float,
                       min_excursion_deg: float,
                       inner_loop_us: int | None,
                       outer_loop_divisor: int | None,
                       estop_on_exit: bool,
                       report_json: str | None = None) -> int:
    setup_logging(verbose)
    config: ControllerConfig | None = None
    plans: list[ExerciseDofPlan] = []
    run_started_at_unix = time.time()
    startup_completed_at_unix: float | None = None
    movement_started_at_unix: float | None = None
    movement_finished_at_unix: float | None = None
    step_count = 0
    failure_reason: str | None = None

    try:
        config = load_config(config_path, selected_joints=selected_joints)
    except (FileNotFoundError, ValueError) as exc:
        logger.error(f"Configuration error: {exc}")
        return 1

    if pretension_before_startup:
        config.startup_pretension_all = True

    logger.info(f"Loaded config: {len(config.joints)} joints, CAN={config.can_interface}:{config.can_channel}")
    for key, jcfg in config.joints.items():
        logger.info(f"  {key}: id={jcfg.joint_id}, dofs={jcfg.dof_count}")

    preflight_ports = _collect_preflight_ports(preflight_auto, preflight_serials)
    if preflight_ports:
        logger.info(f"Serial preflight on {len(preflight_ports)} port(s): {', '.join(preflight_ports)}")
        try:
            for port in preflight_ports:
                preflight_boot(port)
                await asyncio.sleep(0.1)
        except Exception as exc:
            logger.error(f"Serial preflight failed: {exc}")
            return 1

    plans = _build_plans(config, span_cap_deg, margin_deg, min_excursion_deg)
    steps = _interleave_steps(plans)
    if not steps:
        logger.error("Exercise plan is empty: no DOF has a valid excursion")
        return 1

    logger.info("Exercise plan:")
    for plan in plans:
        logger.info(
            f"  {plan.joint_key} DOF{plan.dof}: center={plan.center_deg:+.1f}° "
            f"targets={', '.join(f'{target:+.1f}°' for target in plan.targets_deg)}"
        )

    can_bus = CanBus()
    telemetry = TelemetryManager(config)
    safety = SafetyManager()
    impedance = ImpedanceLoop(config)
    fsm = StartupFSM()

    telemetry_task = asyncio.create_task(telemetry.listen(can_bus))
    exit_code = 0

    try:
        discovered = await fsm.run_discover(can_bus, config, telemetry)
        if not discovered:
            return 1

        if inner_loop_us is not None or outer_loop_divisor is not None:
            if len(config.joints) != 1:
                raise RuntimeError(
                    "Loop frequency override is broadcast-only; refuse when exercising multiple joints"
                )
            unexpected_ids = sorted(telemetry.unexpected_announces.keys())
            if unexpected_ids:
                raise RuntimeError(
                    "Loop frequency override is broadcast-only; unexpected joints are present on the bus: "
                    f"{unexpected_ids}"
                )

        ready = await fsm.run_startup(can_bus, config, telemetry, safety)
        if not ready:
            return 1
        startup_completed_at_unix = time.time()

        if inner_loop_us is not None or outer_loop_divisor is not None:
            inner_us = inner_loop_us or 2000
            outer_div = outer_loop_divisor or 1
            arb_id, payload = encode_loop_frequency(inner_us, outer_div)
            await can_bus.send(arb_id, payload)
            logger.info(
                "Applied loop frequency override: inner=%dus (%.1fHz), outer_div=%d (%.1fHz)",
                inner_us,
                1000000.0 / inner_us,
                outer_div,
                (1000000.0 / inner_us) / outer_div,
            )
            await asyncio.sleep(0.2)

        impedance.start(can_bus)
        await asyncio.sleep(0.5)

        for plan in plans:
            state = telemetry.states.get(plan.joint_key)
            current_deg = None
            if state is not None:
                current_deg = state.angles_deg.get(plan.dof)
            if current_deg is None:
                current_deg = impedance.targets[plan.joint_key][plan.dof].q_deg

            impedance.set_target(
                plan.joint_key,
                plan.dof,
                plan.center_deg,
                config.nudge_speed_deg_s,
            )
            logger.info(
                f"Centering [{plan.joint_key}] DOF{plan.dof} "
                f"{current_deg:+.1f}° -> {plan.center_deg:+.1f}°"
            )
            centered = await _wait_for_target(
                telemetry,
                plan.joint_key,
                plan.dof,
                plan.center_deg,
                config.homing_tolerance_deg,
                _move_timeout(current_deg, plan.center_deg, config.nudge_speed_deg_s),
            )
            if not centered:
                stale = telemetry.any_stale()
                raise RuntimeError(
                    f"Centering timeout for {plan.joint_key} DOF{plan.dof} "
                    f"at {plan.center_deg:+.1f}°; stale={stale}"
                )

        await asyncio.sleep(dwell_s)

        started = time.monotonic()
        movement_started_at_unix = time.time()

        while (time.monotonic() - started) < duration_s:
            for step in steps:
                if (time.monotonic() - started) >= duration_s:
                    break

                state = telemetry.states.get(step.joint_key)
                current_deg = None
                if state is not None:
                    current_deg = state.angles_deg.get(step.dof)
                if current_deg is None:
                    current_deg = impedance.targets[step.joint_key][step.dof].q_deg

                impedance.set_target(
                    step.joint_key,
                    step.dof,
                    step.target_deg,
                    config.nudge_speed_deg_s,
                )
                logger.info(
                    f"Exercise target [{step.joint_key}] DOF{step.dof} "
                    f"{current_deg:+.1f}° -> {step.target_deg:+.1f}°"
                )

                reached = await _wait_for_target(
                    telemetry,
                    step.joint_key,
                    step.dof,
                    step.target_deg,
                    config.homing_tolerance_deg,
                    _move_timeout(current_deg, step.target_deg, config.nudge_speed_deg_s),
                )
                step_count += 1
                if not reached:
                    stale = telemetry.any_stale()
                    raise RuntimeError(
                        f"Target timeout for {step.joint_key} DOF{step.dof} "
                        f"at {step.target_deg:+.1f}°; stale={stale}"
                    )

                await asyncio.sleep(dwell_s)

        logger.info(
            f"Exercise complete: duration={time.monotonic() - started:.1f}s "
            f"steps={step_count}"
        )
        movement_finished_at_unix = time.time()

    except Exception as exc:
        logger.error(f"Exercise failed: {exc}")
        failure_reason = str(exc)
        movement_finished_at_unix = movement_finished_at_unix or time.time()
        exit_code = 1
    finally:
        await impedance.stop()
        if can_bus.connected and estop_on_exit:
            await safety.send_estop(can_bus, reason=0)
        telemetry.stop()
        telemetry_task.cancel()
        try:
            await telemetry_task
        except asyncio.CancelledError:
            pass
        await can_bus.disconnect()
        if report_json:
            report = {
                "config_path": config_path,
                "selected_joints": selected_joints or (list(config.joints.keys()) if config is not None else []),
                "preflight_auto": preflight_auto,
                "preflight_serials": preflight_serials or [],
                "pretension_before_startup": pretension_before_startup,
                "duration_s": duration_s,
                "dwell_s": dwell_s,
                "span_cap_deg": span_cap_deg,
                "margin_deg": margin_deg,
                "min_excursion_deg": min_excursion_deg,
                "inner_loop_us": inner_loop_us,
                "outer_loop_divisor": outer_loop_divisor,
                "estop_on_exit": estop_on_exit,
                "session_log_path": str(get_log_path()),
                "started_at_unix": run_started_at_unix,
                "startup_completed_at_unix": startup_completed_at_unix,
                "movement_started_at_unix": movement_started_at_unix,
                "movement_finished_at_unix": movement_finished_at_unix,
                "finished_at_unix": time.time(),
                "exit_code": exit_code,
                "failure_reason": failure_reason,
                "step_count": step_count,
                "plans": [
                    {
                        "joint_key": plan.joint_key,
                        "dof": plan.dof,
                        "center_deg": plan.center_deg,
                        "targets_deg": plan.targets_deg,
                    }
                    for plan in plans
                ],
            }
            _write_run_report(report_json, report)
            logger.info(f"Wrote exercise report: {report_json}")

    return exit_code


def cli() -> None:
    parser = argparse.ArgumentParser(description="Alia Jetson headless exercise runner")
    parser.add_argument("--config", "-c", help="Path to controller.yaml")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable debug logging")
    parser.add_argument(
        "--joint", "-j",
        action="append",
        help="Joint profile(s) to control, e.g. --joint knee_right --joint ankle_right",
    )
    parser.add_argument(
        "--preflight-auto",
        action="store_true",
        help="Open all auto-discovered Pico USB serial ports once before CAN startup, then close them",
    )
    parser.add_argument(
        "--preflight-serial",
        action="append",
        help="Open a specific board USB serial port once before CAN startup (repeatable)",
    )
    parser.add_argument(
        "--pretension-before-startup",
        action="store_true",
        help="Force PRETENSION_ALL before startup for this run only",
    )
    parser.add_argument("--duration-sec", type=float, default=300.0, help="Total exercise duration")
    parser.add_argument("--dwell-sec", type=float, default=0.2, help="Pause after each reached target")
    parser.add_argument("--span-cap-deg", type=float, default=20.0, help="Maximum excursion from home per DOF")
    parser.add_argument("--margin-deg", type=float, default=5.0, help="Keep this margin from configured limits")
    parser.add_argument("--min-excursion-deg", type=float, default=3.0, help="Minimum excursion to include in the pattern")
    parser.add_argument(
        "--inner-loop-us",
        type=int,
        help="Override inner control-loop period in microseconds for this run only (single isolated controller only)",
    )
    parser.add_argument(
        "--outer-loop-divisor",
        type=int,
        help="Override outer loop divisor for this run only (single isolated controller only)",
    )
    parser.add_argument(
        "--report-json",
        help="Write a structured JSON report for this exercise run",
    )
    parser.add_argument("--estop-on-exit", action="store_true", help="Send EMERGENCY_STOP after the exercise completes")
    args = parser.parse_args()

    if args.inner_loop_us is not None and not (500 <= args.inner_loop_us <= 10000):
        parser.error("--inner-loop-us must be between 500 and 10000")
    if args.outer_loop_divisor is not None and not (1 <= args.outer_loop_divisor <= 20):
        parser.error("--outer-loop-divisor must be between 1 and 20")

    exit_code = asyncio.run(
        run_exercise(
            args.config,
            args.verbose,
            args.joint,
            args.preflight_auto,
            args.preflight_serial,
            args.pretension_before_startup,
            args.duration_sec,
            args.dwell_sec,
            args.span_cap_deg,
            args.margin_deg,
            args.min_excursion_deg,
            args.inner_loop_us,
            args.outer_loop_divisor,
            args.estop_on_exit,
            args.report_json,
        )
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    cli()
