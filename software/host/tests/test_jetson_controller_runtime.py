import asyncio
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
