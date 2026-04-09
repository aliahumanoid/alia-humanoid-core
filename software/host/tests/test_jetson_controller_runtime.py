import asyncio
import can
from collections import deque
import gc
import importlib
import json
import os
import queue
import subprocess
import sys
import threading
import types
import warnings
from pathlib import Path
from unittest import mock


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


def test_can_bus_send_uses_tx_worker_and_preserves_order():
    from jetson_controller.can_bus import CanBus

    class _FakeBus:
        def __init__(self):
            self.sent = []

        def send(self, msg):
            self.sent.append((msg.arbitration_id, bytes(msg.data)))

        def shutdown(self):
            return None

    async def scenario():
        bus = CanBus()
        fake = _FakeBus()
        bus._bus = fake
        bus._loop = asyncio.get_running_loop()
        bus._tx_queue = queue.Queue()
        bus._sender_stop.clear()
        bus._sender_thread = threading.Thread(
            target=bus._tx_loop, daemon=True, name="can-sender-test"
        )
        bus._sender_thread.start()
        try:
            first = asyncio.create_task(bus.send(0x123, b"\x01\x02"))
            await asyncio.sleep(0)
            second = asyncio.create_task(bus.send(0x124, b"\x03"))
            await asyncio.gather(first, second)
            assert fake.sent == [(0x123, b"\x01\x02"), (0x124, b"\x03")]
            assert bus.tx_count == 2
        finally:
            await bus.disconnect()

    asyncio.run(scenario())


def test_can_bus_send_propagates_tx_errors():
    from jetson_controller.can_bus import CanBus

    class _FailingBus:
        def send(self, msg):
            raise can.CanError("synthetic tx failure")

        def shutdown(self):
            return None

    async def scenario():
        bus = CanBus()
        bus._bus = _FailingBus()
        bus._loop = asyncio.get_running_loop()
        bus._tx_queue = queue.Queue()
        bus._sender_stop.clear()
        bus._sender_thread = threading.Thread(
            target=bus._tx_loop, daemon=True, name="can-sender-test"
        )
        bus._sender_thread.start()
        try:
            try:
                await bus.send(0x123, b"\x01")
            except can.CanError as exc:
                assert "synthetic tx failure" in str(exc)
            else:
                raise AssertionError("Expected CanError from queued sender")
            assert bus.errors == 1
        finally:
            await bus.disconnect()

    asyncio.run(scenario())


def test_can_bus_listener_drops_tx_echo_frames():
    from jetson_controller.can_bus import CanBus

    class _FakeLoop:
        def __init__(self):
            self.scheduled = []

        def call_soon_threadsafe(self, callback, *args):
            self.scheduled.append((callback, args))

    class _FakeBus:
        def __init__(self, owner):
            self._owner = owner
            self._calls = 0

        def recv(self, timeout=0.2):
            self._calls += 1
            if self._calls == 1:
                self._owner._listener_stop.set()
                return types.SimpleNamespace(
                    arbitration_id=0x123,
                    data=b"\x01\x02",
                    is_rx=False,
                )
            return None

    bus = CanBus()
    bus._loop = _FakeLoop()
    bus._rx_queue = object()
    bus._bus = _FakeBus(bus)
    bus._listener_stop.clear()

    with mock.patch.object(bus, "_log_rx") as log_rx:
        bus._listen_loop()

    assert bus.rx_count == 0
    assert bus._loop.scheduled == []
    log_rx.assert_not_called()


def test_config_autodetect_candle_channel():
    from jetson_controller import config as mod

    with mock.patch.object(
        mod.can,
        "detect_available_configs",
        return_value=[{"interface": "candle", "channel": "001A00473945501720303651:0"}],
    ):
        channel = mod._autodetect_can_channel("candle")

    assert channel == "001A00473945501720303651:0"


def test_fw_update_stream_image_supports_periodic_burst_pause():
    from jetson_controller import fw_update as mod

    artifact = mod.UpdateArtifact(
        manifest_path=Path("/tmp/test_manifest.json"),
        image_path=Path("/tmp/test_firmware.bin"),
        target_slot=1,
        image_size_bytes=mod.FLASH_PAGE_SIZE * 3,
        image_crc32=0,
        image_bytes=bytes(range(256)) * 3,
    )
    timing = mod.UpdateTimingConfig(
        ctrl_gap_s=0.0,
        page_begin_gap_s=0.0,
        frag_gap_s=0.0,
        burst_percent=50.0,
        burst_pause_s=1.0,
        page_retry_limit=0,
        page_retry_backoff_s=0.0,
    )

    committed_pages = []
    burst_sleeps = []

    async def fake_send_with_gap(_can_bus, _frame, *, delay_s=0.0):
        return None

    async def fake_wait_for_page_result(_can_bus, _joint_id, *, timeout_s, allow_page_retry):
        assert allow_page_retry is False
        page_index = len(committed_pages)
        committed_pages.append(page_index)
        return types.SimpleNamespace(
            status=types.SimpleNamespace(
                event_code=mod.FW_UPDATE_EVT_PAGE_COMMITTED,
                error_code=mod.FW_UPDATE_ERR_NONE,
                error_name="NONE",
                value=page_index,
            ),
        )

    async def fake_sleep(delay_s):
        burst_sleeps.append(delay_s)

    async def scenario():
        with mock.patch.object(mod, "send_with_gap", fake_send_with_gap), \
             mock.patch.object(mod, "wait_for_page_result", fake_wait_for_page_result), \
             mock.patch.object(mod.asyncio, "sleep", fake_sleep):
            completed, streamed = await mod.stream_image(
                object(),
                8,
                artifact,
                timing=timing,
            )
            assert completed is True
            assert streamed == artifact.image_size_bytes
            assert burst_sleeps == [1.0]

    asyncio.run(scenario())


def test_wait_for_candidate_boot_reports_stale_selector_hint_when_stuck_pending_test():
    from jetson_controller import fw_update as mod
    from jetson_controller import protocol as proto

    class _FakeCanBus:
        def __init__(self, messages):
            self._messages = deque(messages)
            self.sent = []

        async def send(self, arbitration_id, data):
            self.sent.append((arbitration_id, bytes(data)))

        async def recv(self, timeout=None):
            if self._messages:
                return self._messages.popleft()
            if timeout:
                await asyncio.sleep(timeout)
            return None

    joint_id = 8
    data = bytes([
        1,
        2,
        mod.FW_BOOT_PENDING_TEST,
        1,
        0,
        1,
        0,
        0x01,  # maintenance_active
    ])
    msg = can.Message(
        arbitration_id=proto.CAN_ID_FW_UPDATE_INFO + joint_id,
        data=data,
        is_extended_id=False,
    )

    async def scenario():
        can_bus = _FakeCanBus([msg])
        try:
            await mod.wait_for_candidate_boot(
                can_bus,
                joint_id,
                2,
                timeout_s=0.01,
            )
        except TimeoutError as exc:
            text = str(exc)
            assert "PENDING_TEST" in text
            assert "boot_update selector" in text
            assert "pico2_boot_update_debug" in text
        else:
            raise AssertionError("Expected timeout while controller remained in PENDING_TEST")

    asyncio.run(scenario())


def test_exercise_applies_loop_frequency_override_before_impedance_start():
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod
    from jetson_controller.protocol import encode_loop_frequency

    sent_frames = []
    start_snapshots = []

    class _FakeCanBus:
        connected = False

        async def send(self, arbitration_id, data):
            sent_frames.append((arbitration_id, bytes(data)))

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {}
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {
                "hip_roll_bench_right": {
                    0: types.SimpleNamespace(q_deg=0.0),
                }
            }

        def start(self, _can_bus):
            start_snapshots.append(list(sent_frames))

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return True

        async def run_startup(self, _can_bus, config, _telemetry, _safety):
            assert config.startup_pretension_all is True
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )

    plan = mod.ExerciseDofPlan(
        joint_key="hip_roll_bench_right",
        dof=0,
        center_deg=0.0,
        targets_deg=[0.0],
    )
    step = mod.ExerciseStep(
        joint_key="hip_roll_bench_right",
        dof=0,
        target_deg=0.0,
    )

    async def _wait_immediately(*_args, **_kwargs):
        return True

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM), \
             mock.patch.object(mod, "_build_plans", return_value=[plan]), \
             mock.patch.object(mod, "_interleave_steps", return_value=[step]), \
             mock.patch.object(mod, "_wait_for_target", _wait_immediately):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=True,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=2000,
                outer_loop_divisor=5,
                estop_on_exit=False,
                report_json=None,
                can_preflight=False,
            )
            assert rc == 0

    asyncio.run(scenario())

    expected = encode_loop_frequency(2000, 5)
    assert sent_frames[0] == expected
    assert start_snapshots
    assert start_snapshots[0][0] == expected


def test_exercise_writes_report_json(tmp_path):
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod

    report_path = tmp_path / "exercise_report.json"

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {
                "hip_roll_bench_right": {
                    0: types.SimpleNamespace(q_deg=0.0),
                }
            }

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return True

        async def run_startup(self, *_args, **_kwargs):
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )
    plan = mod.ExerciseDofPlan(
        joint_key="hip_roll_bench_right",
        dof=0,
        center_deg=0.0,
        targets_deg=[0.0],
    )
    step = mod.ExerciseStep(
        joint_key="hip_roll_bench_right",
        dof=0,
        target_deg=0.0,
    )

    async def _wait_immediately(*_args, **_kwargs):
        return True

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM), \
             mock.patch.object(mod, "_build_plans", return_value=[plan]), \
             mock.patch.object(mod, "_interleave_steps", return_value=[step]), \
             mock.patch.object(mod, "_wait_for_target", _wait_immediately):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=None,
                outer_loop_divisor=None,
                estop_on_exit=False,
                report_json=str(report_path),
                can_preflight=False,
            )
            assert rc == 0

    asyncio.run(scenario())

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["exit_code"] == 0
    assert report["step_count"] == 0
    assert report["movement_started_at_unix"] is not None
    assert report["plans"][0]["joint_key"] == "hip_roll_bench_right"


def test_exercise_rejects_loop_frequency_override_when_multiple_joints_selected():
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {}
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {}

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return True

        async def run_startup(self, *_args, **_kwargs):
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            ),
            "knee_right": types.SimpleNamespace(
                joint_id=2,
                name="Knee Right",
                dof_count=1,
                min_angles={0: 0.0},
                max_angles={0: 110.0},
            ),
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right", "knee_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=2000,
                outer_loop_divisor=5,
                estop_on_exit=False,
                report_json=None,
                can_preflight=False,
            )
            assert rc == 1

    asyncio.run(scenario())


def test_exercise_rejects_loop_frequency_override_when_unexpected_joint_is_present():
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {}
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {
                "hip_roll_bench_right": {
                    0: types.SimpleNamespace(q_deg=0.0),
                }
            }

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, _can_bus, _config, telemetry):
            telemetry.unexpected_announces[9] = object()
            return True

        async def run_startup(self, *_args, **_kwargs):
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=2000,
                outer_loop_divisor=5,
                estop_on_exit=False,
                report_json=None,
                can_preflight=False,
            )
            assert rc == 1

    asyncio.run(scenario())


def test_exercise_can_preflight_rejects_active_can_faults():
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod
    from jetson_controller.protocol import FaultStatus

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {
                "hip_roll_bench_right": types.SimpleNamespace(
                    health_status={
                        "phase": "READY",
                        "host_can_tx_error_count": 12,
                        "host_can_rx_error_count": 0,
                        "motor_can_tx_error_count": 0,
                        "host_can_tec": 12,
                        "host_can_rec": 0,
                        "host_can_eflg": 0,
                        "host_can_eflg_names": [],
                        "motor_can_tec": 0,
                        "motor_can_rec": 0,
                        "motor_can_eflg": 0,
                        "motor_can_eflg_names": [],
                    },
                    fault_status=FaultStatus(
                        joint_id=8,
                        seq=1,
                        active_fault_bits=(1 << 0),
                        latched_fault_bits=(1 << 0),
                        primary_fault_code=0,
                        source_id=3,
                        fault_epoch=1,
                    ),
                    angles_deg={0: 0.0},
                )
            }
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {
                "hip_roll_bench_right": {0: types.SimpleNamespace(q_deg=0.0)}
            }

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return True

        async def run_startup(self, *_args, **_kwargs):
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )
    plan = mod.ExerciseDofPlan(
        joint_key="hip_roll_bench_right",
        dof=0,
        center_deg=0.0,
        targets_deg=[0.0],
    )
    step = mod.ExerciseStep(
        joint_key="hip_roll_bench_right",
        dof=0,
        target_deg=0.0,
    )

    async def _wait_immediately(*_args, **_kwargs):
        return True

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM), \
             mock.patch.object(mod, "_build_plans", return_value=[plan]), \
             mock.patch.object(mod, "_interleave_steps", return_value=[step]), \
             mock.patch.object(mod, "_wait_for_target", _wait_immediately):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=None,
                outer_loop_divisor=None,
                estop_on_exit=False,
                report_json=None,
                can_preflight=True,
            )
            assert rc == 1

    asyncio.run(scenario())


def test_exercise_can_preflight_accepts_clean_bus():
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {
                "hip_roll_bench_right": types.SimpleNamespace(
                    health_status={
                        "phase": "READY",
                        "host_can_tx_error_count": 0,
                        "host_can_rx_error_count": 0,
                        "motor_can_tx_error_count": 0,
                        "host_can_tec": 0,
                        "host_can_rec": 0,
                        "host_can_eflg": 0,
                        "host_can_eflg_names": [],
                        "motor_can_tec": 0,
                        "motor_can_rec": 0,
                        "motor_can_eflg": 0,
                        "motor_can_eflg_names": [],
                    },
                    fault_status=None,
                    angles_deg={0: 0.0},
                )
            }
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {
                "hip_roll_bench_right": {0: types.SimpleNamespace(q_deg=0.0)}
            }

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return True

        async def run_startup(self, *_args, **_kwargs):
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )
    plan = mod.ExerciseDofPlan(
        joint_key="hip_roll_bench_right",
        dof=0,
        center_deg=0.0,
        targets_deg=[0.0],
    )
    step = mod.ExerciseStep(
        joint_key="hip_roll_bench_right",
        dof=0,
        target_deg=0.0,
    )

    async def _wait_immediately(*_args, **_kwargs):
        return True

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM), \
             mock.patch.object(mod, "_build_plans", return_value=[plan]), \
             mock.patch.object(mod, "_interleave_steps", return_value=[step]), \
             mock.patch.object(mod, "_wait_for_target", _wait_immediately):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=None,
                outer_loop_divisor=None,
                estop_on_exit=False,
                report_json=None,
                can_preflight=True,
            )
            assert rc == 0

    asyncio.run(scenario())


def test_exercise_can_preflight_ignores_nonfatal_eflg_bits():
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {
                "hip_roll_bench_right": types.SimpleNamespace(
                    health_status={
                        "phase": "READY",
                        "host_can_tx_error_count": 0,
                        "host_can_rx_error_count": 0,
                        "motor_can_tx_error_count": 0,
                        "host_can_tec": 0,
                        "host_can_rec": 0,
                        "host_can_eflg": 0x01,
                        "host_can_eflg_names": ["EWARN"],
                        "motor_can_tec": 0,
                        "motor_can_rec": 0,
                        "motor_can_eflg": 0x40,
                        "motor_can_eflg_names": ["RX0OVR"],
                    },
                    fault_status=None,
                    angles_deg={0: 0.0},
                )
            }
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {
                "hip_roll_bench_right": {0: types.SimpleNamespace(q_deg=0.0)}
            }

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return True

        async def run_startup(self, *_args, **_kwargs):
            return True

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )
    plan = mod.ExerciseDofPlan(
        joint_key="hip_roll_bench_right",
        dof=0,
        center_deg=0.0,
        targets_deg=[0.0],
    )
    step = mod.ExerciseStep(
        joint_key="hip_roll_bench_right",
        dof=0,
        target_deg=0.0,
    )

    async def _wait_immediately(*_args, **_kwargs):
        return True

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM), \
             mock.patch.object(mod, "_build_plans", return_value=[plan]), \
             mock.patch.object(mod, "_interleave_steps", return_value=[step]), \
             mock.patch.object(mod, "_wait_for_target", _wait_immediately):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=None,
                outer_loop_divisor=None,
                estop_on_exit=False,
                report_json=None,
                can_preflight=True,
            )
            assert rc == 0

    asyncio.run(scenario())


def test_exercise_report_marks_failed_startup_exit_code(tmp_path):
    _install_fake_rich()
    _install_fake_yaml()

    from jetson_controller import exercise as mod

    report_path = tmp_path / "exercise_report_failed.json"

    class _FakeCanBus:
        connected = False

        async def send(self, *_args, **_kwargs):
            return None

        async def disconnect(self):
            return None

    class _FakeTelemetry:
        def __init__(self, _config):
            self.states = {}
            self.unexpected_announces = {}

        async def listen(self, _can_bus):
            await asyncio.sleep(3600)

        def stop(self):
            return None

        def any_stale(self):
            return False

    class _FakeSafety:
        async def send_estop(self, *_args, **_kwargs):
            return None

    class _FakeImpedance:
        def __init__(self, _config):
            self.targets = {}

        def start(self, _can_bus):
            return None

        def set_target(self, *_args, **_kwargs):
            return None

        async def stop(self):
            return None

    class _FakeFSM:
        async def run_discover(self, *_args, **_kwargs):
            return False

        async def run_startup(self, *_args, **_kwargs):
            return False

    config = types.SimpleNamespace(
        can_interface="slcan",
        can_channel="dummy",
        joints={
            "hip_roll_bench_right": types.SimpleNamespace(
                joint_id=8,
                name="Hip Roll Bench Right",
                dof_count=1,
                min_angles={0: -20.0},
                max_angles={0: 20.0},
            )
        },
        nudge_speed_deg_s=5.0,
        homing_tolerance_deg=1.0,
        startup_pretension_all=False,
    )

    async def scenario():
        with mock.patch.object(mod, "load_config", return_value=config), \
             mock.patch.object(mod, "CanBus", _FakeCanBus), \
             mock.patch.object(mod, "TelemetryManager", _FakeTelemetry), \
             mock.patch.object(mod, "SafetyManager", _FakeSafety), \
             mock.patch.object(mod, "ImpedanceLoop", _FakeImpedance), \
             mock.patch.object(mod, "StartupFSM", _FakeFSM):
            rc = await mod.run_exercise(
                config_path=None,
                verbose=False,
                selected_joints=["hip_roll_bench_right"],
                preflight_auto=False,
                preflight_serials=None,
                pretension_before_startup=False,
                duration_s=0.0,
                dwell_s=0.0,
                span_cap_deg=10.0,
                margin_deg=5.0,
                min_excursion_deg=1.0,
                inner_loop_us=None,
                outer_loop_divisor=None,
                estop_on_exit=False,
                report_json=str(report_path),
                can_preflight=True,
            )
            assert rc == 1

    asyncio.run(scenario())

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["exit_code"] == 1
    assert report["failure_reason"] == "discover_failed"


def test_fw_update_stream_image_retries_recoverable_page_error():
    from jetson_controller import fw_update as mod

    artifact = mod.UpdateArtifact(
        manifest_path=Path("/tmp/test_manifest.json"),
        image_path=Path("/tmp/test_firmware.bin"),
        target_slot=1,
        image_size_bytes=mod.FLASH_PAGE_SIZE,
        image_crc32=0,
        image_bytes=bytes(range(256)),
    )
    timing = mod.UpdateTimingConfig(
        ctrl_gap_s=0.0,
        page_begin_gap_s=0.00075,
        frag_gap_s=0.0005,
        page_retry_limit=1,
        page_retry_backoff_s=0.01,
    )

    sent_frames = []
    recover_calls = []
    page_results = [
        types.SimpleNamespace(
            status=types.SimpleNamespace(
                event_code=mod.FW_UPDATE_EVT_ERROR,
                error_code=mod.FW_UPDATE_ERR_FRAG_INDEX_MISMATCH,
                error_name="FRAG_INDEX_MISMATCH",
                value=1,
            )
        ),
        types.SimpleNamespace(
            status=types.SimpleNamespace(
                event_code=mod.FW_UPDATE_EVT_PAGE_COMMITTED,
                error_code=mod.FW_UPDATE_ERR_NONE,
                error_name="NONE",
                value=0,
            )
        ),
    ]

    async def fake_send_with_gap(_can_bus, frame, *, delay_s=0.0):
        sent_frames.append((frame[0], bytes(frame[1]), delay_s))

    async def fake_wait_for_page_result(_can_bus, _joint_id, *, timeout_s, allow_page_retry):
        assert allow_page_retry is True
        return page_results.pop(0)

    async def fake_recover_page_retry_state(_can_bus, _joint_id, *, page_index, backoff_s):
        recover_calls.append((page_index, backoff_s))
        return False

    async def scenario():
        with mock.patch.object(mod, "send_with_gap", fake_send_with_gap), \
             mock.patch.object(mod, "wait_for_page_result", fake_wait_for_page_result), \
             mock.patch.object(mod, "recover_page_retry_state", fake_recover_page_retry_state):
            completed, streamed = await mod.stream_image(
                object(),
                8,
                artifact,
                timing=timing,
            )
            assert completed is True
            assert streamed == artifact.image_size_bytes
            assert recover_calls == [(0, 0.01)]
            assert len(sent_frames) == 2 * (1 + 43)

    asyncio.run(scenario())


def test_fw_update_recover_page_retry_state_drains_and_resumes_same_page():
    from jetson_controller import fw_update as mod

    class _FakeBus:
        def __init__(self):
            self._messages = deque([object(), None])
            self.sent = []

        async def recv(self, timeout=None):
            if self._messages:
                return self._messages.popleft()
            return None

        async def send(self, arb_id, data):
            self.sent.append((arb_id, bytes(data)))

    consumed = []

    def fake_consume_update_message(snapshot, joint_id, msg, *, allow_error_codes=None):
        consumed.append((joint_id, msg, allow_error_codes))
        return snapshot

    async def fake_wait_for_update_snapshot(
        _can_bus,
        _joint_id,
        *,
        timeout_s,
        expect_event=None,
        require_uid=False,
        require_info=False,
        allow_error_codes=None,
    ):
        return mod.UpdateSnapshot(
            info=types.SimpleNamespace(
                boot_state=mod.FW_BOOT_RECEIVING,
                update_in_progress=True,
                flags=0,
            ),
            progress=types.SimpleNamespace(next_page_index=12),
        )

    async def scenario():
        bus = _FakeBus()
        with mock.patch.object(mod, "consume_update_message", fake_consume_update_message), \
             mock.patch.object(mod, "wait_for_update_snapshot", fake_wait_for_update_snapshot):
            already_committed = await mod.recover_page_retry_state(
                bus,
                8,
                page_index=12,
                backoff_s=0.0,
            )
            assert already_committed is False
            assert consumed == [(8, mock.ANY, mod.RECOVERABLE_PAGE_ERROR_CODES)]
            assert len(bus.sent) == 1

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
