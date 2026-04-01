"""
Terminal UI using rich library.

Displays live joint state, FSM status, CAN stats, and key commands.
Non-blocking keyboard input via asyncio.
"""
from __future__ import annotations

import asyncio
import logging
import subprocess
import sys
import termios
import time
import tty
from typing import Callable, Coroutine, Optional

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from .can_bus import CanBus
from .config import ControllerConfig
from .fsm import FSMState
from .impedance_loop import ImpedanceLoop
from .session_log import get_log_path
from .telemetry import TelemetryManager

logger = logging.getLogger(__name__)

# Type alias for async callbacks
AsyncCallback = Callable[[], Coroutine]
NudgeCallback = Callable[[float, str, int], Coroutine]


class TUI:
    """Terminal UI for the Jetson controller."""

    STEP_DEG = 5.0  # Degrees per +/- keypress

    def __init__(self, config: ControllerConfig, telemetry: TelemetryManager,
                 impedance: ImpedanceLoop, can_bus: CanBus):
        self._config = config
        self._telemetry = telemetry
        self._impedance = impedance
        self._can_bus = can_bus
        self._console = Console()

        self._fsm_state = FSMState.INIT
        self._fsm_message = ""
        self._start_time = time.monotonic()
        self._loop_paused = False
        self._status_line = ""  # Transient status message from key commands
        self._joint_keys = list(config.joints.keys())
        self._selected_joint_idx = 0
        self._selected_dof = 0

        self._quit_event = asyncio.Event()
        self._bg_tasks: set[asyncio.Task] = set()  # tracked background tasks
        self._long_op_running = False  # serializes S, D (one at a time)

        # Callbacks (wired from __main__.py)
        self._cb_estop: Optional[AsyncCallback] = None
        self._cb_home: Optional[AsyncCallback] = None
        self._cb_startup: Optional[AsyncCallback] = None
        self._cb_discover: Optional[AsyncCallback] = None
        self._cb_nudge: Optional[NudgeCallback] = None
        self._cb_toggle_loop: Optional[AsyncCallback] = None

    def set_fsm_state(self, state: FSMState, message: str = "") -> None:
        self._fsm_state = state
        self._fsm_message = message

    # --- Callback registration ---
    def on_estop(self, cb: AsyncCallback) -> None:
        self._cb_estop = cb

    def on_home(self, cb: AsyncCallback) -> None:
        self._cb_home = cb

    def on_startup(self, cb: AsyncCallback) -> None:
        self._cb_startup = cb

    def on_discover(self, cb: AsyncCallback) -> None:
        self._cb_discover = cb

    def on_nudge(self, cb) -> None:
        """cb(delta_deg: float, joint_key: str, dof: int) -> coroutine"""
        self._cb_nudge = cb

    def _selected_joint_key(self) -> str:
        return self._joint_keys[self._selected_joint_idx]

    def _selected_joint_cfg(self):
        return self._config.joints[self._selected_joint_key()]

    @staticmethod
    def _format_dof_name(name: str | None, fallback: str) -> str:
        if not name:
            return fallback
        return name.replace("_", "-")

    def _selected_dof_label(self) -> str:
        jcfg = self._selected_joint_cfg()
        if self._selected_dof < len(jcfg.dof_names):
            return self._format_dof_name(jcfg.dof_names[self._selected_dof], f"DOF{self._selected_dof}")
        return f"DOF{self._selected_dof}"

    def _clamp_selected_dof(self) -> None:
        jcfg = self._selected_joint_cfg()
        if self._selected_dof >= jcfg.dof_count:
            self._selected_dof = 0

    def _cycle_joint(self) -> None:
        if not self._joint_keys:
            return
        self._selected_joint_idx = (self._selected_joint_idx + 1) % len(self._joint_keys)
        self._clamp_selected_dof()

    def _cycle_dof(self) -> None:
        jcfg = self._selected_joint_cfg()
        self._selected_dof = (self._selected_dof + 1) % max(jcfg.dof_count, 1)

    def on_toggle_loop(self, cb: AsyncCallback) -> None:
        self._cb_toggle_loop = cb

    async def wait_for_quit(self) -> None:
        await self._quit_event.wait()

    async def run(self) -> None:
        """Main TUI loop with live display and key handling."""
        key_task = asyncio.create_task(self._key_reader())

        # Suppress console logging while TUI is active — the TUI panel
        # already shows FSM state, CAN stats, etc.  Logs still go to file.
        console_handlers = [
            h for h in logging.getLogger().handlers
            if isinstance(h, logging.StreamHandler)
            and not isinstance(h, logging.FileHandler)
        ]
        for h in console_handlers:
            h.setLevel(logging.WARNING)

        try:
            with Live(self._render(), console=self._console,
                      refresh_per_second=10, screen=False) as live:
                while not self._quit_event.is_set():
                    live.update(self._render())
                    await asyncio.sleep(0.1)
        finally:
            # Restore console logging
            for h in console_handlers:
                h.setLevel(logging.INFO)
            key_task.cancel()
            try:
                await key_task
            except asyncio.CancelledError:
                pass

    def _render(self) -> Panel:
        """Build the TUI display."""
        elapsed = time.monotonic() - self._start_time
        minutes, seconds = divmod(int(elapsed), 60)

        # Header
        state_color = {
            FSMState.READY: "green",
            FSMState.ERROR: "red",
            FSMState.HOME: "yellow",
            FSMState.RECOVERY_SETTLE: "yellow",
            FSMState.DISCOVERED: "yellow",
        }.get(self._fsm_state, "cyan")

        header = Text()
        header.append("ALIA Jetson Controller v0.1", style="bold white")
        header.append("     State: ", style="dim")
        header.append(self._fsm_state.value, style=f"bold {state_color}")
        if self._fsm_message:
            header.append(f" ({self._fsm_message})", style="dim")
        header.append(f"  {minutes:02d}:{seconds:02d}", style="dim")
        if self._loop_paused:
            header.append("  [PAUSED]", style="bold yellow")
        if self._joint_keys:
            header.append("  Sel: ", style="dim")
            header.append(
                f"{self._selected_joint_key()} DOF{self._selected_dof} ({self._selected_dof_label()})",
                style="bold yellow",
            )

        # Joint table
        table = Table(show_header=True, header_style="bold", box=None,
                      pad_edge=False, expand=True)
        table.add_column("Joint", style="bold", width=14)
        table.add_column("Online", width=6, justify="center")
        max_dofs = max((jcfg.dof_count for jcfg in self._config.joints.values()), default=1)
        for dof in range(max_dofs):
            table.add_column(f"DOF{dof}", width=10, justify="right")
            table.add_column(f"Target{dof}", width=10, justify="right")
        table.add_column("RX", width=6, justify="right")

        for key, jcfg in self._config.joints.items():
            state = self._telemetry.states.get(key)
            if state is None:
                continue

            online = "[green]YES[/]" if state.is_online else "[red]NO[/]"

            cells = []
            for d in range(max_dofs):
                if d < jcfg.dof_count:
                    # Current angle
                    val = state.angles_deg.get(d)
                    if val is not None:
                        cells.append(f"{val:+.2f}\u00b0")
                    else:
                        cells.append("[dim]---[/]")
                    # Target angle
                    tgt = self._impedance.targets.get(key, {}).get(d)
                    if tgt is not None:
                        cells.append(f"[dim]{tgt.q_deg:+.1f}\u00b0[/]")
                    else:
                        cells.append("[dim]\u2014[/]")
                else:
                    cells.append("[dim]\u2014[/]")
                    cells.append("[dim]\u2014[/]")

            joint_label = f"> {key}" if key == self._selected_joint_key() else f"  {key}"
            rendered_cells: list[str] = []
            for d in range(max_dofs):
                current_cell = cells[d * 2]
                target_cell = cells[d * 2 + 1]
                if key == self._selected_joint_key() and d == self._selected_dof and d < jcfg.dof_count:
                    current_cell = f"[bold yellow]{current_cell}[/]"
                    target_cell = f"[bold yellow]{target_cell}[/]"
                rendered_cells.extend([current_cell, target_cell])

            table.add_row(joint_label, online, *rendered_cells, str(state.rx_count))

        # CAN stats
        can_info = Text()
        can_info.append(f"CAN: {self._config.can_interface} ", style="dim")
        can_info.append(f"TX:{self._can_bus.tx_count} ", style="green")
        can_info.append(f"RX:{self._can_bus.rx_count} ", style="cyan")
        can_info.append(f"ERR:{self._can_bus.errors}",
                        style="red" if self._can_bus.errors else "dim")

        if self._impedance.cycle_count > 0:
            can_info.append(f"  Loop: {self._impedance.avg_cycle_time_ms:.1f}ms", style="dim")
            effective_hz = 1000.0 / max(self._impedance.avg_period_ms, 0.1)
            can_info.append(f" ({effective_hz:.0f}Hz)", style="dim")

        # Status line (transient messages from key commands)
        status = Text()
        if self._status_line:
            status.append(f"\u25b6 {self._status_line}", style="italic yellow")

        # Key commands
        keys = Text()
        keys.append("[E]", style="bold red")
        keys.append("-Stop  ", style="dim")
        keys.append("[S]", style="bold cyan")
        keys.append("tartup  ", style="dim")
        keys.append("[D]", style="bold cyan")
        keys.append("iscover  ", style="dim")
        keys.append("[H]", style="bold yellow")
        keys.append("ome  ", style="dim")
        keys.append("[0]", style="bold green")
        keys.append(" Zero  ", style="dim")
        keys.append("[+]", style="bold green")
        keys.append("/", style="dim")
        keys.append("[-]", style="bold green")
        keys.append(f" {self.STEP_DEG}\u00b0 sel  ", style="dim")
        keys.append("[J]", style="bold white")
        keys.append("oint  ", style="dim")
        keys.append("[Tab]", style="bold white")
        keys.append(" DOF  ", style="dim")
        keys.append("[L]", style="bold magenta")
        keys.append("og  ", style="dim")
        keys.append("[Space]", style="bold")
        keys.append(" Pause  ", style="dim")
        keys.append("[Q]", style="bold")
        keys.append("uit", style="dim")

        group = Group(header, "", table, "", can_info, status, "", keys)
        return Panel(group, border_style="blue", expand=True)

    # ------------------------------------------------------------------
    # Keyboard handling
    # ------------------------------------------------------------------

    async def _key_reader(self) -> None:
        """Non-blocking keyboard input reader."""
        loop = asyncio.get_running_loop()
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while not self._quit_event.is_set():
                char = await loop.run_in_executor(None, self._read_char)
                if char is None:
                    continue
                await self._handle_key(char)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _read_char(self) -> Optional[str]:
        """Blocking read of a single character (runs in executor)."""
        import select
        if select.select([sys.stdin], [], [], 0.2)[0]:
            return sys.stdin.read(1)
        return None

    def _fire_long_op(self, callback: AsyncCallback) -> bool:
        """Schedule a long operation as a tracked background task.

        Returns False if a long op is already running (caller should
        show a warning). Only one long op at a time (S, D).
        E-stop and Quit can still cancel it via _cancel_bg_tasks().
        """
        if self._long_op_running:
            return False
        self._long_op_running = True
        task = asyncio.create_task(callback())
        self._bg_tasks.add(task)
        task.add_done_callback(self._bg_task_done)
        return True

    def _bg_task_done(self, task: asyncio.Task) -> None:
        """Log exceptions from background key-command tasks and untrack."""
        self._bg_tasks.discard(task)
        self._long_op_running = False
        if task.cancelled():
            return
        exc = task.exception()
        if exc:
            logger.error(f"Key command error: {exc}")

    def _cancel_bg_tasks(self) -> None:
        """Cancel all tracked background tasks (called on E-stop/Quit)."""
        for task in list(self._bg_tasks):
            task.cancel()
        self._bg_tasks.clear()
        self._long_op_running = False

    async def _handle_key(self, key: str) -> None:
        k = key.lower()

        # E-stop and Quit are always awaited inline (must be instant)
        # Both cancel any running background tasks first.
        if k == 'e':
            self._status_line = "EMERGENCY STOP!"
            self._cancel_bg_tasks()
            if self._cb_estop:
                await self._cb_estop()

        elif k == 'q':
            self._status_line = "Shutting down..."
            self._cancel_bg_tasks()
            self._quit_event.set()

        # Long operations run in background, serialized (one at a time)
        elif k == 's':
            if self._cb_startup:
                if self._fire_long_op(self._cb_startup):
                    self._status_line = "Re-running startup sequence..."
                else:
                    self._status_line = "Busy — wait for current operation"

        elif k == 'd':
            if self._cb_discover:
                if self._fire_long_op(self._cb_discover):
                    self._status_line = "Sending discover (time_sync + identify)..."
                else:
                    self._status_line = "Busy — wait for current operation"

        elif k == 'h' or k == '0':
            self._status_line = "Moving to home (0\u00b0 all DOFs)..."
            if self._cb_home:
                await self._cb_home()  # Fast: just updates targets

        elif key in ('+', '='):  # = is unshifted + on most keyboards
            self._status_line = (
                f"Nudge {self._selected_joint_key()} DOF{self._selected_dof} ({self._selected_dof_label()}) "
                f"+{self.STEP_DEG}\u00b0"
            )
            if self._cb_nudge:
                await self._cb_nudge(
                    self.STEP_DEG,
                    self._selected_joint_key(),
                    self._selected_dof,
                )

        elif key == '-':
            self._status_line = (
                f"Nudge {self._selected_joint_key()} DOF{self._selected_dof} ({self._selected_dof_label()}) "
                f"-{self.STEP_DEG}\u00b0"
            )
            if self._cb_nudge:
                await self._cb_nudge(
                    -self.STEP_DEG,
                    self._selected_joint_key(),
                    self._selected_dof,
                )

        elif k == 'j':
            self._cycle_joint()
            self._status_line = (
                f"Selected {self._selected_joint_key()} DOF{self._selected_dof} ({self._selected_dof_label()})"
            )

        elif key == '\t':
            self._cycle_dof()
            self._status_line = (
                f"Selected {self._selected_joint_key()} DOF{self._selected_dof} ({self._selected_dof_label()})"
            )

        elif k == 'l':
            log_path = get_log_path()
            if log_path.exists():
                self._status_line = f"Opening log: {log_path.name}"
                # Open in system viewer (non-blocking)
                if sys.platform == "darwin":
                    subprocess.Popen(["open", str(log_path)])
                else:
                    subprocess.Popen(["xdg-open", str(log_path)])
            else:
                self._status_line = "No log file yet"

        elif key == ' ':
            self._loop_paused = not self._loop_paused
            self._status_line = "Loop PAUSED" if self._loop_paused else "Loop RESUMED"
            if self._cb_toggle_loop:
                await self._cb_toggle_loop()
