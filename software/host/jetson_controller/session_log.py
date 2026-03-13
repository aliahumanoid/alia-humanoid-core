"""
Unified session logger for the Alia Jetson Controller.

Creates a single timestamped log file per session under logs/.
Both the CAN controller and the serial monitor write to the same file
via the standard logging module, each with a distinct source prefix.

On startup the previous log is **deleted** (only the latest session
is kept) to avoid stale data accumulation.

Log location:
    jetson_controller/logs/session.log

Usage — CAN controller side (already uses logging):
    from .session_log import setup_session_logging
    setup_session_logging(verbose=True)
    # All logging.getLogger("jetson_controller.*") messages go to file + console

Usage — serial monitor side:
    from .session_log import get_session_logger
    slog = get_session_logger()
    slog.serial("EVT:FW:VERSION 0.1.0")      # tagged [SERIAL]
    slog.serial("RSP:STATUS(2,NOT_READY)")    # tagged [SERIAL]
"""
from __future__ import annotations

import logging
import os
import time
from datetime import datetime
from pathlib import Path


# Log directory inside the package
LOG_DIR = Path(__file__).parent / "logs"
LOG_FILE = LOG_DIR / "session.log"

# Module-level state
_file_handler: logging.FileHandler | None = None
_session_start: float = 0.0


def _ensure_log_dir() -> None:
    """Create logs/ directory if it doesn't exist."""
    LOG_DIR.mkdir(exist_ok=True)
    # Add .gitignore so logs are never committed
    gitignore = LOG_DIR / ".gitignore"
    if not gitignore.exists():
        gitignore.write_text("*\n!.gitignore\n")


def _clear_previous_log() -> None:
    """Delete the previous session log."""
    if LOG_FILE.exists():
        LOG_FILE.unlink()


def setup_session_logging(verbose: bool = False, clear: bool = True) -> None:
    """Configure logging to write to both console and session log file.

    Call this once at startup.

    Args:
        verbose: If True, file handler captures DEBUG level.
        clear: If True, delete the previous log first.
               The CAN controller (main process) should clear=True.
               The serial monitor (secondary) should clear=False to append.
    """
    global _file_handler, _session_start

    if _file_handler is not None:
        return  # Already set up in this process

    _ensure_log_dir()

    if clear:
        _clear_previous_log()

    _session_start = time.monotonic()

    # Write a section header (append-safe)
    source = "CAN Controller" if clear else "Serial Monitor"
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(
            f"\n# ─── {source} started: {datetime.now().isoformat()} ───\n\n"
        )

    # Create file handler
    _file_handler = logging.FileHandler(str(LOG_FILE), mode="a", encoding="utf-8")
    _file_handler.setLevel(logging.DEBUG)  # Always capture everything to file
    _file_handler.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    ))

    # Attach to root logger so ALL modules log to file
    root = logging.getLogger()
    root.addHandler(_file_handler)


class SessionLogger:
    """Helper to write tagged serial lines to the session log."""

    def __init__(self):
        self._logger = logging.getLogger("jetson_controller.serial_rx")

    def serial(self, line: str) -> None:
        """Log a line received from firmware serial port."""
        self._logger.info(f"[SERIAL] {line}")

    def serial_sent(self, cmd: str) -> None:
        """Log a command sent to firmware serial port."""
        self._logger.debug(f"[SERIAL TX] {cmd}")


# Singleton
_session_logger: SessionLogger | None = None


def get_session_logger() -> SessionLogger:
    """Get the shared SessionLogger instance."""
    global _session_logger
    if _session_logger is None:
        _session_logger = SessionLogger()
    return _session_logger


def get_log_path() -> Path:
    """Return the path to the current session log file."""
    return LOG_FILE
