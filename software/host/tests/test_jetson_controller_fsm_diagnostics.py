import asyncio
from collections import deque
from types import SimpleNamespace

import jetson_controller.fsm as fsm_module
from jetson_controller.config import ControllerConfig, GainSet, JointControlConfig
from jetson_controller.fsm import FSMState, StartupFSM
from jetson_controller.protocol import DIAG_FAULT_NAMES, FaultStatus, JointAnnounce
from jetson_controller.safety import SafetyManager


_FAULT_BITS = {name: bit for bit, name in DIAG_FAULT_NAMES.items()}


def _build_config() -> ControllerConfig:
    joint = JointControlConfig(
        name="knee_left",
        joint_id=1,
        dof_count=1,
        dof_names=["flexion_extension"],
        dof_drive_types={0: "antagonistic_tendon"},
        dof_motor_counts={0: 2},
        dof_capabilities={0: {}},
        min_angles={0: 0.0},
        max_angles={0: 110.0},
        gains_outer=GainSet(kp=8.0, ki=1.0, kd=0.1),
        gains_inner=GainSet(kp=10.0, ki=1.0, kd=0.25),
        stiffness_deg=20.0,
        dof_gains_outer={},
        dof_gains_inner={},
        dof_stiffness_deg={},
        home_position_deg={0: 0.0},
    )
    return ControllerConfig(
        can_interface="slcan",
        can_channel="/dev/null",
        can_bitrate=1_000_000,
        startup_pretension_all=False,
        startup_recovery_settle_s=0.0,
        send_rate_hz=50,
        watchdog_ms=100,
        homing_speed_deg_s=10.0,
        homing_tolerance_deg=1.0,
        nudge_speed_deg_s=5.0,
        joints={"KNEE_LEFT": joint},
    )


def _fault_status(
    *,
    active: list[str] | None = None,
    latched: list[str] | None = None,
    primary: str | None = None,
    source_id: int = 0,
    fault_epoch: int = 1,
) -> FaultStatus:
    active = active or []
    latched = latched or []
    active_bits = sum(1 << _FAULT_BITS[name] for name in active)
    latched_bits = sum(1 << _FAULT_BITS[name] for name in latched)
    primary_name = primary or (active[0] if active else latched[0] if latched else None)
    primary_code = _FAULT_BITS[primary_name] if primary_name is not None else None
    return FaultStatus(
        joint_id=1,
        seq=1,
        active_fault_bits=active_bits,
        latched_fault_bits=latched_bits,
        primary_fault_code=primary_code,
        source_id=source_id,
        fault_epoch=fault_epoch,
    )


def _telemetry_state(
    *,
    ready: bool,
    phase: str,
    active_faults: list[str] | None = None,
    latched_faults: list[str] | None = None,
):
    return SimpleNamespace(
        announce=JointAnnounce(
            joint_id=1,
            dof_count=1,
            motor_count=2,
            ready=ready,
            fw_version="0.1.0",
            clock_synced=True,
        ),
        fault_status=_fault_status(active=active_faults, latched=latched_faults),
        health_status={"phase": phase},
        diagnostic_events=deque(),
        angles_deg={0: 0.0},
        holding={0: True},
        is_online=True,
        rx_count=1,
    )


async def _noop(*_args, **_kwargs) -> None:
    return None


def test_fsm_startup_blocks_on_persistent_controller_fault(monkeypatch):
    config = _build_config()
    state = _telemetry_state(
        ready=False,
        phase="SAFE_MODE_UNPROVISIONED",
        latched_faults=["CONFIG_INVALID"],
    )
    telemetry = SimpleNamespace(states={"KNEE_LEFT": state})
    safety = SafetyManager()
    fsm = StartupFSM()
    messages: list[tuple[FSMState, str]] = []
    fsm.on_state_change(lambda state_value, msg: messages.append((state_value, msg)))

    called = {"startup": False}

    async def unexpected_startup(*_args, **_kwargs) -> None:
        called["startup"] = True

    monkeypatch.setattr(fsm_module.asyncio, "sleep", _noop)
    fsm._startup = unexpected_startup  # type: ignore[method-assign]
    fsm._stream = _noop  # type: ignore[method-assign]
    fsm._init_gains = _noop  # type: ignore[method-assign]
    fsm._home = _noop  # type: ignore[method-assign]

    result = asyncio.run(fsm.run_startup(object(), config, telemetry, safety))

    assert result is False
    assert fsm.state == FSMState.ERROR
    assert called["startup"] is False
    assert "CONFIG_INVALID" in messages[-1][1]
    assert "Provision the correct joint profile" in messages[-1][1]


def test_fsm_resume_allows_estop_recovery_when_fault_clears(monkeypatch):
    config = _build_config()
    state = _telemetry_state(
        ready=True,
        phase="FAULT_LOCKOUT",
        active_faults=["ESTOP_LATCHED"],
        latched_faults=["ESTOP_LATCHED"],
    )
    telemetry = SimpleNamespace(states={"KNEE_LEFT": state})
    safety = SafetyManager()
    safety._estop_sent = True
    fsm = StartupFSM()

    async def recover(*_args, **_kwargs) -> None:
        state.fault_status = _fault_status(latched=["ESTOP_LATCHED"])
        state.health_status = {"phase": "READY"}

    async def should_not_home(*_args, **_kwargs) -> None:
        raise AssertionError("resume path should not home already-ready joints")

    monkeypatch.setattr(fsm_module.asyncio, "sleep", _noop)
    fsm._startup = _noop  # type: ignore[method-assign]
    fsm._recover_ready_joints_after_estop = recover  # type: ignore[method-assign]
    fsm._stream = _noop  # type: ignore[method-assign]
    fsm._init_gains = _noop  # type: ignore[method-assign]
    fsm._home = should_not_home  # type: ignore[method-assign]

    result = asyncio.run(fsm.run_startup(object(), config, telemetry, safety))

    assert result is True
    assert fsm.state == FSMState.READY
    assert safety.estop_latched is False


def test_fsm_resume_blocks_if_estop_fault_remains_active(monkeypatch):
    config = _build_config()
    state = _telemetry_state(
        ready=True,
        phase="FAULT_LOCKOUT",
        active_faults=["ESTOP_LATCHED"],
        latched_faults=["ESTOP_LATCHED"],
    )
    telemetry = SimpleNamespace(states={"KNEE_LEFT": state})
    safety = SafetyManager()
    safety._estop_sent = True
    fsm = StartupFSM()
    messages: list[tuple[FSMState, str]] = []
    fsm.on_state_change(lambda state_value, msg: messages.append((state_value, msg)))

    monkeypatch.setattr(fsm_module.asyncio, "sleep", _noop)
    fsm._startup = _noop  # type: ignore[method-assign]
    fsm._recover_ready_joints_after_estop = _noop  # type: ignore[method-assign]
    fsm._stream = _noop  # type: ignore[method-assign]
    fsm._init_gains = _noop  # type: ignore[method-assign]
    fsm._home = _noop  # type: ignore[method-assign]

    result = asyncio.run(fsm.run_startup(object(), config, telemetry, safety))

    assert result is False
    assert fsm.state == FSMState.ERROR
    assert "ESTOP_LATCHED" in messages[-1][1]
    assert "PRETENSION_ALL" in messages[-1][1]


def test_fsm_resume_recovers_when_controller_reports_estop_latched_without_host_latch(monkeypatch):
    config = _build_config()
    state = _telemetry_state(
        ready=True,
        phase="FAULT_LOCKOUT",
        active_faults=["ESTOP_LATCHED"],
        latched_faults=["ESTOP_LATCHED"],
    )
    telemetry = SimpleNamespace(states={"KNEE_LEFT": state})
    safety = SafetyManager()
    fsm = StartupFSM()

    async def recover(*_args, **_kwargs) -> None:
        state.fault_status = _fault_status(latched=["ESTOP_LATCHED"])
        state.health_status = {"phase": "READY"}

    async def should_not_home(*_args, **_kwargs) -> None:
        raise AssertionError("resume path should not home already-ready joints")

    monkeypatch.setattr(fsm_module.asyncio, "sleep", _noop)
    fsm._startup = _noop  # type: ignore[method-assign]
    fsm._recover_ready_joints_after_estop = recover  # type: ignore[method-assign]
    fsm._stream = _noop  # type: ignore[method-assign]
    fsm._init_gains = _noop  # type: ignore[method-assign]
    fsm._home = should_not_home  # type: ignore[method-assign]

    result = asyncio.run(fsm.run_startup(object(), config, telemetry, safety))

    assert result is True
    assert fsm.state == FSMState.READY
    assert safety.estop_latched is False


def test_fsm_wait_for_faults_clear_observes_motor_timeout_recovery(monkeypatch):
    config = _build_config()
    state = _telemetry_state(
        ready=False,
        phase="FAULT_LOCKOUT",
        active_faults=["MOTOR_TIMEOUT"],
        latched_faults=["MOTOR_TIMEOUT"],
    )
    telemetry = SimpleNamespace(states={"KNEE_LEFT": state})
    fsm = StartupFSM()
    calls = {"sleep": 0}

    async def fake_sleep(_delay: float) -> None:
        calls["sleep"] += 1
        if calls["sleep"] == 1:
            state.fault_status = _fault_status(latched=["MOTOR_TIMEOUT"])

    monkeypatch.setattr(fsm_module.asyncio, "sleep", fake_sleep)

    cleared = asyncio.run(
        fsm._wait_for_faults_clear(config, telemetry, {"MOTOR_TIMEOUT"}, timeout_s=0.2)
    )

    assert cleared is True


def test_fsm_resume_waits_for_delayed_estop_fault_and_recovers(monkeypatch):
    config = _build_config()
    state = _telemetry_state(
        ready=True,
        phase="READY",
        active_faults=[],
        latched_faults=["ESTOP_LATCHED"],
    )
    telemetry = SimpleNamespace(states={"KNEE_LEFT": state})
    safety = SafetyManager()
    fsm = StartupFSM()

    async def recover(*_args, **_kwargs) -> None:
        state.fault_status = _fault_status(latched=["ESTOP_LATCHED"])
        state.health_status = {"phase": "READY"}

    async def delayed_estop(*_args, **_kwargs) -> bool:
        state.fault_status = _fault_status(
            active=["ESTOP_LATCHED"],
            latched=["ESTOP_LATCHED"],
        )
        state.health_status = {"phase": "FAULT_LOCKOUT"}
        return True

    async def should_not_home(*_args, **_kwargs) -> None:
        raise AssertionError("resume path should not home already-ready joints")

    monkeypatch.setattr(fsm_module.asyncio, "sleep", _noop)
    fsm._startup = _noop  # type: ignore[method-assign]
    fsm._wait_for_active_estop_latch = delayed_estop  # type: ignore[method-assign]
    fsm._recover_ready_joints_after_estop = recover  # type: ignore[method-assign]
    fsm._stream = _noop  # type: ignore[method-assign]
    fsm._init_gains = _noop  # type: ignore[method-assign]
    fsm._home = should_not_home  # type: ignore[method-assign]

    result = asyncio.run(fsm.run_startup(object(), config, telemetry, safety))

    assert result is True
    assert fsm.state == FSMState.READY
    assert safety.estop_latched is False
