import asyncio
import gc
import importlib
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


def test_run_sh_uses_project_venv_binaries():
    script = (HOST_DIR / "jetson_controller" / "run.sh").read_text()

    assert 'PYTHON_BIN="$VENV_DIR/bin/python"' in script
    assert '"$PYTHON_BIN" -m pip install -q -r "$HOST_DIR/requirements.txt"' in script
    assert 'exec "$PYTHON_BIN" -m jetson_controller -v "$@"' in script
    assert 'source "$VENV_DIR/bin/activate"' not in script
