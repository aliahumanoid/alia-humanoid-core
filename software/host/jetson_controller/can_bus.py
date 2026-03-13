"""
Async CAN bus wrapper using python-can.

Bridges python-can's blocking I/O to asyncio via a background listener
thread and an asyncio.Queue for received messages.
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from typing import Callable, Optional

import can

from .protocol import MULTI_FRAME_DELAY_S

logger = logging.getLogger(__name__)


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
            except can.CanError as exc:
                self.errors += 1
                logger.warning(f"CAN RX error: {exc}")
                time.sleep(0.1)

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
