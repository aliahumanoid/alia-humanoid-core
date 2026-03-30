#!/usr/bin/env python3
"""
Serial monitor for Alia joint controller boards.

Auto-discovers Pico 2 serial ports by USB VID:PID (2E8A:000F),
filtering out CAN adapters. Streams firmware output with colour coding.

Usage:
    python -m jetson_controller.serial_monitor [-v]
    python -m jetson_controller.serial_monitor --port /dev/cu.usbmodem123

Firmware protocol:
    Boot messages:  EVT:FW:VERSION 0.1.0 / EVT:JOINT <id> <name> / EVT:READY
    Probe command:  CMD:STATUS  →  RSP:STATUS(joint,READY|NOT_READY)
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Optional

import serial
import serial.tools.list_ports

from .session_log import setup_session_logging, get_session_logger, get_log_path

logger = logging.getLogger(__name__)

# --- ANSI colours --------------------------------------------------------
_RESET = "\033[0m"
_BOLD = "\033[1m"
_DIM = "\033[2m"
_RED = "\033[91m"
_GREEN = "\033[92m"
_YELLOW = "\033[93m"
_CYAN = "\033[96m"
_MAGENTA = "\033[95m"
_GRAY = "\033[90m"

BAUD_RATE = 115200
READ_TIMEOUT_S = 0.1
BOOT_WAIT_S = 4.0  # Pico 2 resets on serial open (DTR), needs time to boot

# USB VID:PID for Raspberry Pi Pico 2 CDC serial
PICO2_VID_PID = (0x2E8A, 0x000F)

# Known CAN adapter VID:PIDs to exclude
CAN_ADAPTER_VID_PIDS = {
    (0x16D0, 0x117E),  # CANable 2
}


def _format_vid_pid(vid: Optional[int], pid: Optional[int]) -> str:
    """Format a USB VID:PID pair, tolerating missing metadata."""
    if vid is None or pid is None:
        return "n/a"
    return f"{vid:04X}:{pid:04X}"


def _colour_line(line: str) -> str:
    """Apply ANSI colour based on prefix."""
    stripped = line.strip()
    if stripped.startswith("EVT:FW:VERSION") or stripped.startswith("EVT:READY"):
        return f"{_BOLD}{_GREEN}{line}{_RESET}"
    if stripped.startswith("EVT:JOINT"):
        return f"{_GREEN}{line}{_RESET}"
    if stripped.startswith("EVT:"):
        return f"{_CYAN}{line}{_RESET}"
    if stripped.startswith("RSP:ERROR") or stripped.startswith("ERROR:"):
        return f"{_RED}{line}{_RESET}"
    if stripped.startswith("RSP:"):
        return f"{_CYAN}{line}{_RESET}"
    if stripped.startswith("WARN:"):
        return f"{_YELLOW}{line}{_RESET}"
    if stripped.startswith("DBG:"):
        return f"{_DIM}{line}{_RESET}"
    if stripped.startswith("INFO:"):
        return f"{_GRAY}{line}{_RESET}"
    return line


def discover_ports(verbose_output: bool = True) -> list[str]:
    """Auto-discover Pico 2 serial ports by USB VID:PID.

    Returns list of port paths (may be multiple boards).
    Excludes known CAN adapters.
    """
    ports = serial.tools.list_ports.comports()
    results: list[str] = []

    if verbose_output:
        print(f"{_BOLD}Serial discover:{_RESET} scanning USB devices...")

    for info in sorted(ports, key=lambda p: p.device):
        vid_pid = (info.vid, info.pid) if info.vid and info.pid else None
        desc = info.description or ""
        label = f"{info.device}  ({desc})"

        # Skip CAN adapters
        if vid_pid in CAN_ADAPTER_VID_PIDS:
            if verbose_output:
                print(f"  {_DIM}skip {label}  [CAN adapter]{_RESET}")
            continue

        # Match Pico 2
        if vid_pid == PICO2_VID_PID:
            if verbose_output:
                print(f"  {_GREEN}found{_RESET} {_CYAN}{label}{_RESET}  {_DIM}[Pico 2]{_RESET}")
            results.append(info.device)
            continue

        # Show other devices for debugging
        if "usbmodem" in info.device or "ttyACM" in info.device:
            vid_pid_text = _format_vid_pid(info.vid, info.pid)
            if verbose_output:
                print(f"  {_DIM}skip {label}  [unknown VID:PID={vid_pid_text}]{_RESET}")

    return results


def discover() -> list[str]:
    """Backward-compatible CLI discovery helper with console output."""
    return discover_ports(verbose_output=True)


def preflight_boot(port: str, boot_wait_s: float = BOOT_WAIT_S) -> None:
    """Open a Pico CDC port long enough to trigger a reboot and wait for boot.

    This intentionally replicates the side effect that today happens when the
    serial monitor or webapp serial handler opens the board USB CDC port.
    """
    logger.info(f"Preflight serial open: {port} (wait {boot_wait_s:.1f}s for boot)")
    with serial.Serial(port, BAUD_RATE, timeout=READ_TIMEOUT_S) as ser:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        time.sleep(boot_wait_s)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass


def monitor(port: str, verbose: bool = False) -> None:
    """Open serial port and stream firmware output with colour coding.

    Press Ctrl-C to exit.
    """
    print(f"\n{_BOLD}═══ Alia Serial Monitor ═══{_RESET}")
    print(f"  Port: {_CYAN}{port}{_RESET}  Baud: {BAUD_RATE}")
    print(f"  {_DIM}Ctrl-C to exit{_RESET}")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=READ_TIMEOUT_S)
    except serial.SerialException as exc:
        print(f"{_RED}Cannot open {port}: {exc}{_RESET}")
        return

    # Pico 2 resets on DTR — wait for firmware boot
    print(f"  {_DIM}Waiting {BOOT_WAIT_S:.0f}s for firmware boot (Pico reset on DTR)...{_RESET}")
    ser.reset_input_buffer()
    time.sleep(BOOT_WAIT_S)
    # Drain any partial boot output still arriving
    ser.reset_input_buffer()
    time.sleep(0.2)
    ser.reset_input_buffer()
    print()

    # Session logger (writes to shared log file)
    slog = get_session_logger()

    # Counters
    rx_lines = 0
    start_time = time.monotonic()
    fw_version = None
    joint_name = None

    try:
        # Send initial status request
        ser.write(b"CMD:STATUS\n")
        ser.flush()
        slog.serial_sent("CMD:STATUS")

        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue

            rx_lines += 1
            ts = time.monotonic() - start_time
            prefix = f"{_DIM}[{ts:8.2f}]{_RESET} "
            print(f"{prefix}{_colour_line(line)}")

            # Write to unified session log
            slog.serial(line)

            # Track firmware info
            if line.startswith("EVT:FW:VERSION"):
                fw_version = line.split()[-1] if " " in line else "?"
            if line.startswith("EVT:JOINT"):
                parts = line.split()
                if len(parts) >= 3:
                    joint_name = parts[2]

    except KeyboardInterrupt:
        elapsed = time.monotonic() - start_time
        print(f"\n{_DIM}─── Session summary ───{_RESET}")
        print(f"  {_DIM}Lines received: {rx_lines}{_RESET}")
        print(f"  {_DIM}Duration:       {elapsed:.1f}s{_RESET}")
        if fw_version:
            print(f"  {_DIM}FW version:     {fw_version}{_RESET}")
        if joint_name:
            print(f"  {_DIM}Joint:          {joint_name}{_RESET}")
    except serial.SerialException as exc:
        print(f"\n{_RED}Serial error: {exc}{_RESET}")
    finally:
        ser.close()
        print(f"{_DIM}Serial port closed.{_RESET}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Alia joint controller serial monitor"
    )
    parser.add_argument(
        "--port", "-p",
        help="Serial port (default: auto-discover Pico 2 by VID:PID)",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose logging",
    )
    args = parser.parse_args()

    # Root logger accepts everything; handlers decide what to show
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # Console: INFO only (no 50 Hz flooding)
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    ))
    root.addHandler(console)

    # Append to shared session log (doesn't clear — CAN controller clears on its start)
    setup_session_logging(args.verbose, clear=False)
    logger.info(f"Session log: {get_log_path()}")

    port = args.port
    if not port:
        found = discover()
        if not found:
            print(f"\n{_RED}No Pico 2 boards found.{_RESET}")
            print(f"{_DIM}Connect a board with joint controller firmware.{_RESET}")
            print(f"{_DIM}Expected USB VID:PID = {PICO2_VID_PID[0]:04X}:{PICO2_VID_PID[1]:04X}{_RESET}")
            sys.exit(1)
        port = found[0]
        if len(found) > 1:
            print(f"\n{_YELLOW}Multiple Pico 2 boards found, using first: {port}{_RESET}")
            print(f"{_DIM}Use --port to select a specific one.{_RESET}")

    monitor(port, verbose=args.verbose)


if __name__ == "__main__":
    main()
