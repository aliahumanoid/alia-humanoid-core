import asyncio
from collections import deque
import gc
import importlib
import json
import os
import subprocess
import sys
import types
import warnings
from pathlib import Path


HOST_DIR = Path(__file__).resolve().parent.parent
if str(HOST_DIR) not in sys.path:
    sys.path.insert(0, str(HOST_DIR))


def _install_fake_rich() -> None:
    if "rich.console" in sys.modules:
        return

    rich_pkg = types.ModuleType("rich")
    console_mod = types.ModuleType("rich.console")
    live_mod = types.ModuleType("rich.live")
    panel_mod = types.ModuleType("rich.panel")
    table_mod = types.ModuleType("rich.table")
    text_mod = types.ModuleType("rich.text")

    class DummyConsole:
        def __init__(self, *args, **kwargs):
            pass

    class DummyLive:
        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def update(self, *args, **kwargs):
            pass

    class DummyPanel:
        def __init__(self, *args, **kwargs):
            pass

    class DummyTable:
        def __init__(self, *args, **kwargs):
            pass

        def add_column(self, *args, **kwargs):
            pass

        def add_row(self, *args, **kwargs):
            pass

    class DummyText:
        def __init__(self, *args, **kwargs):
            pass

        def append(self, *args, **kwargs):
            pass

    def dummy_group(*args, **kwargs):
        return None

    console_mod.Console = DummyConsole
    console_mod.Group = dummy_group
    live_mod.Live = DummyLive
    panel_mod.Panel = DummyPanel
    table_mod.Table = DummyTable
    text_mod.Text = DummyText

    sys.modules["rich"] = rich_pkg
    sys.modules["rich.console"] = console_mod
    sys.modules["rich.live"] = live_mod
    sys.modules["rich.panel"] = panel_mod
    sys.modules["rich.table"] = table_mod
    sys.modules["rich.text"] = text_mod


def _install_fake_yaml() -> None:
    if "yaml" in sys.modules:
        return

    yaml_mod = types.ModuleType("yaml")
    yaml_mod.safe_load = lambda *_args, **_kwargs: {}
    sys.modules["yaml"] = yaml_mod


class _DummyConfig:
    can_interface = "slcan"
    joints = {}


class _DummyTelemetry:
    states = {}


class _DummyImpedance:
    targets = {}
    cycle_count = 0
    avg_cycle_time_ms = 0.0
    avg_period_ms = 0.0


class _DummyBus:
    tx_count = 0
    rx_count = 0
    errors = 0


class _DiagJointCfg:
    dof_count = 1
    dof_names = ["flexion_extension"]


class _DiagConfig:
    can_interface = "slcan"
    joints = {"KNEE_LEFT": _DiagJointCfg()}


def _load_tui_class():
    _install_fake_rich()
    _install_fake_yaml()
    sys.modules.pop("jetson_controller.tui", None)
    module = importlib.import_module("jetson_controller.tui")
    return module.TUI


def test_can_bus_recv_before_connect_returns_none():
    from jetson_controller.can_bus import CanBus

    async def scenario():
        bus = CanBus()
        msg = await bus.recv(timeout=0.01)
        assert msg is None

    asyncio.run(scenario())


def test_tui_rejected_long_op_does_not_warn():
    TUI = _load_tui_class()
    tui = TUI(_DummyConfig(), _DummyTelemetry(), _DummyImpedance(), _DummyBus())

    active = 0
    peak = 0

    async def startup():
        nonlocal active, peak
        active += 1
        peak = max(peak, active)
        await asyncio.sleep(0.02)
        active -= 1

    tui.on_startup(startup)

    async def scenario():
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            await tui._handle_key("s")
            await tui._handle_key("s")
            await asyncio.sleep(0.05)
            gc.collect()
            assert peak == 1
            assert not caught

    asyncio.run(scenario())


def test_tui_diag_label_prefers_active_faults():
    TUI = _load_tui_class()
    from jetson_controller.protocol import FaultStatus

    state = types.SimpleNamespace(
        health_status={"phase": "READY"},
        fault_status=FaultStatus(
            joint_id=1,
            seq=1,
            active_fault_bits=(1 << 3),
            latched_fault_bits=(1 << 3),
            primary_fault_code=3,
            source_id=0,
            fault_epoch=2,
        ),
        diagnostic_events=deque(),
        is_online=True,
        angles_deg={0: 0.0},
        rx_count=1,
    )
    telemetry = types.SimpleNamespace(states={"KNEE_LEFT": state})
    tui = TUI(_DiagConfig(), telemetry, _DummyImpedance(), _DummyBus())

    label, style = tui._joint_diag_label(state)

    assert label == "READY ACT:WATCHDOG"
    assert style == "bold red"


def test_tui_selected_diag_summary_includes_last_event_and_counters():
    TUI = _load_tui_class()
    from jetson_controller.protocol import EventNotice, FaultStatus

    event = EventNotice(
        joint_id=1,
        event_seq=33,
        event_code=0x08,
        flags=0x46,
        source_kind_code=1,
        source_index=0,
        detail0=12,
        detail1=4,
    )
    state = types.SimpleNamespace(
        health_status={
            "phase": "READY",
            "reboot_reason": "WATCHDOG_RESET",
            "state": {"watchdog_warning": True},
            "host_can_tx_error_count": 1,
            "host_can_rx_error_count": 2,
            "motor_can_tx_error_count": 3,
            "loop_overrun_count": 4,
            "watchdog_trip_count": 5,
            "fault_epoch": 9,
        },
        fault_status=FaultStatus(
            joint_id=1,
            seq=1,
            active_fault_bits=(1 << 3),
            latched_fault_bits=(1 << 3),
            primary_fault_code=3,
            source_id=0,
            fault_epoch=9,
        ),
        diagnostic_events=deque([event]),
        is_online=True,
        angles_deg={0: 0.0},
        rx_count=1,
    )
    telemetry = types.SimpleNamespace(states={"KNEE_LEFT": state})
    tui = TUI(_DiagConfig(), telemetry, _DummyImpedance(), _DummyBus())

    summary, summary_style = tui._selected_diag_summary()
    counters, counters_style = tui._selected_health_counters()

    assert "Diag KNEE_LEFT:" in summary
    assert "phase=READY" in summary
    assert "reboot=WATCHDOG_RESET" in summary
    assert "active=WATCHDOG" in summary
    assert "last=WATCHDOG_TIMEOUT/ERROR@DOF:0" in summary
    assert summary_style == "bold red"
    assert counters == "Health ctrs: can=1/2/3 overrun=4 watchdog=5 epoch=9"
    assert counters_style == "dim"


def test_tui_snapshot_key_uses_selected_joint():
    TUI = _load_tui_class()
    telemetry = types.SimpleNamespace(states={"KNEE_LEFT": types.SimpleNamespace()})
    tui = TUI(_DiagConfig(), telemetry, _DummyImpedance(), _DummyBus())
    requested: list[str] = []

    async def snapshot(joint_key: str):
        requested.append(joint_key)

    tui.on_snapshot(snapshot)

    async def scenario():
        await tui._handle_key("r")

    asyncio.run(scenario())

    assert requested == ["KNEE_LEFT"]
    assert tui._status_line == "Requesting fault snapshot for KNEE_LEFT..."


def test_run_sh_forwards_single_mode_args_and_rejects_ambiguous_dual_mode(
    tmp_path,
):
    host_dir = tmp_path / "host"
    jetson_dir = host_dir / "jetson_controller"
    jetson_dir.mkdir(parents=True)

    script = (HOST_DIR / "jetson_controller" / "run.sh").read_text()
    run_sh = jetson_dir / "run.sh"
    run_sh.write_text(script)
    os.chmod(run_sh, 0o755)

    (host_dir / "requirements.txt").write_text("")
    python_bin = host_dir / ".venv" / "bin" / "python"
    python_bin.parent.mkdir(parents=True)
    python_bin.write_text(
        "\n".join(
            [
                "#!/usr/bin/env python3",
                "import json",
                "import sys",
                "",
                "args = sys.argv[1:]",
                "if args[:3] == ['-m', 'pip', 'install']:",
                "    raise SystemExit(0)",
                "print(json.dumps(args))",
            ]
        )
        + "\n"
    )
    os.chmod(python_bin, 0o755)

    proc = subprocess.run(
        [str(run_sh), "--no-serial", "--config", "/tmp/controller.yaml"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    assert json.loads(proc.stdout.strip()) == [
        "-m",
        "jetson_controller",
        "-v",
        "--config",
        "/tmp/controller.yaml",
    ]

    proc = subprocess.run(
        [str(run_sh), "--serial", "--port", "/dev/ttyACM0"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    assert json.loads(proc.stdout.strip()) == [
        "-m",
        "jetson_controller.serial_monitor",
        "-v",
        "--port",
        "/dev/ttyACM0",
    ]

    proc = subprocess.run(
        [str(run_sh), "--config", "/tmp/controller.yaml"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 2
    assert "--no-serial or --serial" in proc.stderr


def test_session_log_clear_preserves_existing_appenders(tmp_path, monkeypatch):
    from jetson_controller import session_log

    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    log_file = log_dir / "session.log"

    monkeypatch.setattr(session_log, "LOG_DIR", log_dir)
    monkeypatch.setattr(session_log, "LOG_FILE", log_file)

    with open(log_file, "a", encoding="utf-8") as handle:
        handle.write("before\n")
        handle.flush()
        session_log._clear_previous_log()
        handle.write("after\n")
        handle.flush()

    assert log_file.read_text(encoding="utf-8") == "after\n"


def test_serial_monitor_discover_tolerates_missing_vid_pid(monkeypatch, capsys):
    from jetson_controller import serial_monitor

    ports = [
        types.SimpleNamespace(
            device="/dev/ttyACM0",
            description="Generic USB CDC",
            vid=None,
            pid=None,
        )
    ]
    monkeypatch.setattr(
        serial_monitor.serial.tools.list_ports,
        "comports",
        lambda: ports,
    )

    assert serial_monitor.discover() == []
    assert "VID:PID=n/a" in capsys.readouterr().out
