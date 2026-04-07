"""
Async CAN bus wrapper using python-can.

Bridges python-can's blocking I/O to asyncio via a background listener
thread and an asyncio.Queue for received messages.
"""
from __future__ import annotations

import asyncio
import logging
import struct
import threading
import time
from typing import Callable, Optional

import can

from .protocol import (
    MULTI_FRAME_DELAY_S,
    CAN_ID_EMERGENCY_STOP, CAN_ID_TIME_SYNC, CAN_ID_ENCODER_STREAM_CTRL,
    CAN_ID_IDENTIFY_REQUEST, CAN_ID_STARTUP_SEQUENCE,
    CAN_ID_PRETENSION_ALL,
    CAN_ID_SET_IMPEDANCE, CAN_ID_IMPEDANCE_CTRL, CAN_ID_FAULT_SNAPSHOT_CTRL,
    CAN_ID_ENCODER_STREAM_DATA, CAN_ID_STARTUP_STATUS, CAN_ID_JOINT_ANNOUNCE,
    CAN_ID_JOINT_STATE, CAN_ID_HEALTH_STATUS, CAN_ID_FAULT_STATUS, CAN_ID_EVENT_NOTICE,
    CAN_ID_FAULT_SNAPSHOT_META, CAN_ID_FAULT_SNAPSHOT_DATA,
    FAULT_SNAPSHOT_CTRL_SUBCMDS,
    UNUSED_DOF,
    decode_event_notice, decode_fault_status, decode_health_status_counters,
    decode_health_status_summary, decode_fault_snapshot_meta, decode_fault_snapshot_chunk,
)

logger = logging.getLogger(__name__)

# How often to log a periodic summary for high-frequency streams (seconds)
_SUMMARY_INTERVAL_S = 5.0


# ---------------------------------------------------------------------------
# Frame decoders for human-readable logging
# ---------------------------------------------------------------------------

def _decode_tx(arb_id: int, data: bytes) -> str:
    """Decode a TX CAN frame into a human-readable string."""
    if arb_id == CAN_ID_EMERGENCY_STOP:
        reason = data[0] if data else 0
        return f"EMERGENCY_STOP reason={reason}"

    if arb_id == CAN_ID_TIME_SYNC:
        ts_ms = struct.unpack_from("<I", data, 0)[0] if len(data) >= 4 else 0
        return f"TIME_SYNC ts={ts_ms}ms"

    if arb_id == CAN_ID_IDENTIFY_REQUEST:
        return "IDENTIFY_REQUEST"

    if arb_id == CAN_ID_ENCODER_STREAM_CTRL:
        start = bool(data[0]) if data else False
        return f"ENCODER_STREAM {'START' if start else 'STOP'}"

    if arb_id == CAN_ID_STARTUP_SEQUENCE:
        joint_id = data[0] if data else 0
        return f"STARTUP_SEQUENCE joint={joint_id}"

    if arb_id == CAN_ID_PRETENSION_ALL:
        joint_id = data[0] if data else 0
        return f"PRETENSION_ALL joint={joint_id}"

    if arb_id == CAN_ID_SET_IMPEDANCE and len(data) >= 6:
        return _decode_set_impedance(data)

    if arb_id == CAN_ID_IMPEDANCE_CTRL and len(data) >= 4:
        joint_id = data[0]
        sub_cmd = data[1]
        param = struct.unpack_from("<H", data, 2)[0]
        sub_names = {0x00: "DISABLE", 0x01: "ENABLE", 0x02: "WATCHDOG"}
        sub = sub_names.get(sub_cmd, f"sub=0x{sub_cmd:02X}")
        return f"IMPEDANCE_CTRL joint={joint_id} {sub} param={param}"

    if arb_id == CAN_ID_FAULT_SNAPSHOT_CTRL and len(data) >= 8:
        sub_cmd = data[0]
        joint_id = data[1]
        snapshot_id = data[2]
        args = list(data[3:7])
        seq = data[7]
        sub_name = FAULT_SNAPSHOT_CTRL_SUBCMDS.get(sub_cmd, f"sub=0x{sub_cmd:02X}")
        return (
            f"FAULT_SNAPSHOT_CTRL joint={joint_id} {sub_name} snapshot={snapshot_id} "
            f"args={args} seq={seq}"
        )

    return f"0x{arb_id:03X} [{data.hex(' ')}]"


def _decode_set_impedance(data: bytes) -> str:
    """Decode SET_IMPEDANCE frame into readable format."""
    joint_id = data[0]
    flags = data[1]
    seq = (flags >> 4) & 0x07
    dof = flags & 0x0F
    has_more = bool(flags & 0x80)

    vals = struct.unpack_from("<hhh", data, 2)

    if seq == 0:
        q_deg = vals[0] / 100.0
        dq = vals[1] / 10.0
        stiff = vals[2] / 10.0
        more = "+" if has_more else ""
        return (f"SET_IMP{more} j={joint_id} d={dof} "
                f"q={q_deg:+.2f}° dq={dq:.1f}°/s stiff={stiff:.1f}°")
    elif seq == 1:
        kp = vals[0] / 100.0
        ki = vals[1] / 100.0
        kd = vals[2] / 100.0
        more = "+" if has_more else ""
        return (f"SET_IMP_OUTER{more} j={joint_id} d={dof} "
                f"Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}")
    elif seq == 2:
        kp = vals[0] / 100.0
        ki = vals[1] / 100.0
        kd = vals[2] / 100.0
        more = "+" if has_more else ""
        return (f"SET_IMP_INNER{more} j={joint_id} d={dof} "
                f"KpI={kp:.2f} KiI={ki:.2f} KdI={kd:.2f}")
    elif seq == 3:
        tau = vals[0]
        return f"SET_IMP_FF j={joint_id} d={dof} tau={tau}"

    return f"SET_IMPEDANCE seq={seq} [{data.hex(' ')}]"


def _decode_rx(arb_id: int, data: bytes) -> tuple[str, bool]:
    """Decode an RX CAN frame. Returns (readable_string, is_high_freq)."""
    # Encoder stream data (50 Hz — high frequency)
    if CAN_ID_ENCODER_STREAM_DATA <= arb_id < CAN_ID_ENCODER_STREAM_DATA + 16:
        joint_id = arb_id - CAN_ID_ENCODER_STREAM_DATA
        if len(data) >= 8:
            d0, d1, d2, t_ms = struct.unpack("<hhhH", data)
            angles = []
            for raw in (d0, d1, d2):
                if raw == UNUSED_DOF:
                    angles.append("--")
                else:
                    angles.append(f"{raw / 100.0:+.2f}°")
            return (f"ENCODER j={joint_id} [{', '.join(angles)}] t={t_ms}ms",
                    True)
        return f"ENCODER j={joint_id} [{data.hex(' ')}]", True

    # Startup status
    if CAN_ID_STARTUP_STATUS <= arb_id < CAN_ID_STARTUP_STATUS + 16:
        joint_id = arb_id - CAN_ID_STARTUP_STATUS
        if len(data) >= 5:
            evt = data[0]
            dof_idx = data[1]
            reason = data[2]
            elapsed = struct.unpack_from("<H", data, 3)[0]
            evt_names = {0: "BEGIN", 1: "DOF_READY", 2: "DOF_FAIL",
                         3: "COMPLETE", 4: "FAILED"}
            reason_names = {
                0: "OK",
                1: "NO_CONTROLLER",
                2: "NO_EQUATIONS",
                3: "ENCODER_TIMEOUT",
                4: "POSITION_RANGE",
                5: "RECALC_ERROR",
                6: "GLOBAL_TIMEOUT",
                7: "PARTIAL_HOLD",
                8: "REFERENCE_REQUIRED",
            }
            return (f"STARTUP_STATUS j={joint_id} "
                    f"{evt_names.get(evt, f'evt={evt}')} "
                    f"dof={dof_idx} reason={reason_names.get(reason, reason)} {elapsed}ms",
                    False)
        return f"STARTUP_STATUS j={joint_id} [{data.hex(' ')}]", False

    # Joint announce
    if CAN_ID_JOINT_ANNOUNCE <= arb_id < CAN_ID_JOINT_ANNOUNCE + 16:
        joint_id = arb_id - CAN_ID_JOINT_ANNOUNCE
        if len(data) >= 8:
            dofs = data[1]
            motors = data[2]
            ready = bool(data[3])
            fw = f"{data[4]}.{data[5]}.{data[6]}"
            return (f"JOINT_ANNOUNCE j={joint_id} dofs={dofs} motors={motors} "
                    f"ready={ready} fw={fw}",
                    False)
        return f"JOINT_ANNOUNCE j={joint_id} [{data.hex(' ')}]", False

    # Joint state (impedance feedback, 50 Hz — high frequency)
    if CAN_ID_JOINT_STATE <= arb_id < CAN_ID_JOINT_STATE + 16:
        joint_id = arb_id - CAN_ID_JOINT_STATE
        if len(data) >= 8:
            dof = data[0]
            q_raw, dq_raw = struct.unpack_from("<hh", data, 1)
            tau_a = struct.unpack_from("<b", data, 5)[0]
            tau_b = struct.unpack_from("<b", data, 6)[0]
            status = data[7]
            flags = []
            if status & 0x02:
                flags.append("HOLD")
            if status & 0x04:
                flags.append("WDG!")
            flag_str = f" [{','.join(flags)}]" if flags else ""
            return (f"JOINT_STATE j={joint_id} d={dof} "
                    f"q={q_raw / 100.0:+.2f}° dq={dq_raw / 10.0:.1f}°/s "
                    f"τA={tau_a * 4} τB={tau_b * 4}{flag_str}",
                    True)
        return f"JOINT_STATE j={joint_id} [{data.hex(' ')}]", True

    # Health status (1 Hz)
    if CAN_ID_HEALTH_STATUS <= arb_id < CAN_ID_HEALTH_STATUS + 16:
        joint_id = arb_id - CAN_ID_HEALTH_STATUS
        if len(data) >= 8:
            frame_kind = data[0] & 0xC0
            if frame_kind == 0x00:
                summary = decode_health_status_summary(data, joint_id)
                return (
                    f"HEALTH_SUMMARY j={joint_id} seq={summary.seq} phase={summary.phase_name} "
                    f"reboot={summary.reboot_reason_name} uptime={summary.uptime_s}s "
                    f"epoch={summary.fault_epoch}",
                    False,
                )
            if frame_kind == 0x40:
                counters = decode_health_status_counters(data, joint_id)
                return (
                    f"HEALTH_COUNTERS j={joint_id} seq={counters.seq} "
                    f"can=({counters.host_can_tx_error_count},{counters.host_can_rx_error_count},"
                    f"{counters.motor_can_tx_error_count}) overrun={counters.loop_overrun_count} "
                    f"wdg={counters.watchdog_trip_count}",
                    False,
                )
            if frame_kind == 0x80 and len(data) >= 8:
                avg_us, max_us, budget_us = struct.unpack_from("<HHH", data, 2)
                return (
                    f"HEALTH_LOOP j={joint_id} avg={avg_us}us max={max_us}us budget={budget_us}us",
                    False,
                )
        return f"HEALTH j={joint_id} [{data.hex(' ')}]", False

    # Fault status
    if CAN_ID_FAULT_STATUS <= arb_id < CAN_ID_FAULT_STATUS + 16:
        joint_id = arb_id - CAN_ID_FAULT_STATUS
        if len(data) >= 8:
            fault = decode_fault_status(data, joint_id)
            return (
                f"FAULT_STATUS j={joint_id} seq={fault.seq} active={fault.active_fault_names} "
                f"latched={fault.latched_fault_names} primary={fault.primary_fault_name} "
                f"src={fault.source_name}",
                False,
            )
        return f"FAULT_STATUS j={joint_id} [{data.hex(' ')}]", False

    # Event notice
    if CAN_ID_EVENT_NOTICE <= arb_id < CAN_ID_EVENT_NOTICE + 16:
        joint_id = arb_id - CAN_ID_EVENT_NOTICE
        if len(data) >= 8:
            event = decode_event_notice(data, joint_id)
            return (
                f"EVENT_NOTICE j={joint_id} seq={event.event_seq} {event.event_name} "
                f"sev={event.severity_name} src={event.source_kind}"
                + (
                    f":{event.source_index_value}"
                    if event.source_index_value is not None
                    else ""
                ),
                False,
            )
        return f"EVENT_NOTICE j={joint_id} [{data.hex(' ')}]", False

    if CAN_ID_FAULT_SNAPSHOT_META <= arb_id < CAN_ID_FAULT_SNAPSHOT_META + 16:
        joint_id = arb_id - CAN_ID_FAULT_SNAPSHOT_META
        if len(data) >= 8:
            meta = decode_fault_snapshot_meta(data, joint_id)
            return (
                f"FAULT_SNAPSHOT_META j={joint_id} snapshot={meta.snapshot_id} "
                f"present={meta.snapshot_present} chunks={meta.total_chunks} "
                f"bytes={meta.payload_bytes} freeze={meta.freeze_event_name}",
                False,
            )
        return f"FAULT_SNAPSHOT_META j={joint_id} [{data.hex(' ')}]", False

    if CAN_ID_FAULT_SNAPSHOT_DATA <= arb_id < CAN_ID_FAULT_SNAPSHOT_DATA + 16:
        joint_id = arb_id - CAN_ID_FAULT_SNAPSHOT_DATA
        if len(data) >= 8:
            chunk = decode_fault_snapshot_chunk(data, joint_id)
            return (
                f"FAULT_SNAPSHOT_DATA j={joint_id} snapshot={chunk.snapshot_id} "
                f"chunk={chunk.chunk_index}",
                False,
            )
        return f"FAULT_SNAPSHOT_DATA j={joint_id} [{data.hex(' ')}]", False

    # Unknown
    hex_data = data.hex(" ") if data else ""
    return f"RX 0x{arb_id:03X} [{hex_data}]", False


class CanBus:
    """Async-friendly CAN bus interface."""

    def __init__(self):
        self._bus: Optional[can.BusABC] = None
        self._rx_queue: Optional[asyncio.Queue] = None
        self._listener_thread: Optional[threading.Thread] = None
        self._listener_stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # Stats
        self.tx_count = 0
        self.rx_count = 0
        self.errors = 0

        # Throttled logging state
        self._last_imp_log: dict[tuple[int, int], bytes] = {}  # (joint, dof) → last data
        self._last_enc_log: dict[int, float] = {}  # joint_id → last log time
        self._imp_summary_time = 0.0
        self._imp_summary_count = 0

    @property
    def connected(self) -> bool:
        return self._bus is not None

    async def connect(self, interface: str, channel: str, bitrate: int) -> None:
        """Open the CAN interface and start the listener thread."""
        if self._bus is not None:
            raise RuntimeError("Already connected")

        self._loop = asyncio.get_running_loop()
        self._rx_queue = asyncio.Queue(maxsize=2000)

        logger.info(f"Connecting CAN: {interface} @ {channel} ({bitrate} bps)")
        self._bus = can.interface.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
        )
        logger.info("CAN bus connected")

        # Drain stale frames from previous sessions (max 100ms window)
        drained = 0
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            stale = self._bus.recv(timeout=0.01)
            if stale is None:
                break
            drained += 1
        if drained:
            logger.info(f"Drained {drained} stale CAN frame(s) from buffer")

        # Start background listener
        self._listener_stop.clear()
        self._listener_thread = threading.Thread(
            target=self._listen_loop, daemon=True, name="can-listener"
        )
        self._listener_thread.start()

    def _listen_loop(self) -> None:
        """Background thread: read CAN frames and push to asyncio queue."""
        while not self._listener_stop.is_set():
            try:
                msg = self._bus.recv(timeout=0.2)
                if msg is None:
                    continue
                self.rx_count += 1
                # Thread-safe put into asyncio queue
                if self._loop is not None and self._rx_queue is not None:
                    self._loop.call_soon_threadsafe(self._enqueue, msg)
                # Log RX frame
                try:
                    self._log_rx(msg)
                except Exception:
                    logger.warning("CAN RX logging failed", exc_info=True)
            except can.CanError as exc:
                self.errors += 1
                logger.warning(f"CAN RX error: {exc}")
                time.sleep(0.1)

    def _log_rx(self, msg: can.Message) -> None:
        """Log received CAN frame — decoded and throttled."""
        # Diagnostic-plane frames are already decoded, logged, and persisted by
        # TelemetryManager. Logging them again here duplicates file I/O on the
        # listener thread and can slow down SLCAN draining during startup.
        if CAN_ID_HEALTH_STATUS <= msg.arbitration_id < CAN_ID_FAULT_SNAPSHOT_DATA + 16:
            return

        decoded, is_high_freq = _decode_rx(msg.arbitration_id, msg.data)

        if is_high_freq:
            # High-freq streams (encoder, joint_state): sample once per interval
            now = time.monotonic()
            last = self._last_enc_log.get(msg.arbitration_id, 0.0)
            if now - last >= _SUMMARY_INTERVAL_S:
                self._last_enc_log[msg.arbitration_id] = now
                logger.debug(f"CAN RX {decoded}")
        else:
            logger.info(f"CAN RX {decoded}")

    def _enqueue(self, msg: can.Message) -> None:
        """Called from event loop thread to put message in queue."""
        try:
            self._rx_queue.put_nowait(msg)
        except asyncio.QueueFull:
            pass  # Drop oldest-style: caller can tune queue size

    async def recv(self, timeout: float = 1.0) -> Optional[can.Message]:
        """Receive next CAN message (async)."""
        if self._rx_queue is None:
            if timeout > 0:
                await asyncio.sleep(min(timeout, 0.05))
            return None
        try:
            return await asyncio.wait_for(self._rx_queue.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None

    async def send(self, arb_id: int, data: bytes) -> None:
        """Send a single CAN frame."""
        if self._bus is None:
            raise RuntimeError("Not connected")
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        # python-can send is fast but may block briefly on TX buffer
        await asyncio.get_running_loop().run_in_executor(
            None, self._send_blocking, msg
        )
        self.tx_count += 1

        # Log TX frame — decoded and throttled
        self._log_tx(arb_id, data)

    def _log_tx(self, arb_id: int, data: bytes) -> None:
        """Log TX frame with human-readable decode and smart throttling."""
        decoded = _decode_tx(arb_id, data)

        if arb_id == CAN_ID_SET_IMPEDANCE:
            # Throttle: only log when payload changes or every N seconds
            joint_id = data[0] if data else 0
            flags = data[1] if len(data) > 1 else 0
            dof = flags & 0x0F
            key = (joint_id, dof)

            prev = self._last_imp_log.get(key)
            self._imp_summary_count += 1
            now = time.monotonic()

            if prev != data:
                # Data changed — log it
                self._last_imp_log[key] = data
                logger.info(f"CAN TX {decoded}")
            elif now - self._imp_summary_time >= _SUMMARY_INTERVAL_S:
                # Periodic summary
                self._imp_summary_time = now
                logger.info(f"CAN TX [impedance stream: "
                            f"{self._imp_summary_count} frames in "
                            f"{_SUMMARY_INTERVAL_S:.0f}s]")
                self._imp_summary_count = 0
            # else: skip (same data, within interval)
        else:
            logger.info(f"CAN TX {decoded}")

    def _send_blocking(self, msg: can.Message) -> None:
        with self._tx_lock:
            self._bus.send(msg)

    async def send_impedance_sequence(self, frames: list[tuple[int, bytes]]) -> None:
        """Send a multi-frame SET_IMPEDANCE sequence with inter-frame delays.

        Args:
            frames: List of (arb_id, payload) tuples in order.
        """
        for i, (arb_id, data) in enumerate(frames):
            if i > 0:
                await asyncio.sleep(MULTI_FRAME_DELAY_S)
            await self.send(arb_id, data)

    async def disconnect(self) -> None:
        """Stop listener and close CAN bus."""
        self._listener_stop.set()
        if self._listener_thread is not None:
            self._listener_thread.join(timeout=2.0)
            self._listener_thread = None
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None
        logger.info("CAN bus disconnected")
