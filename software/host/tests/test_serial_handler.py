import threading
import time
from queue import Empty, Queue
from unittest.mock import MagicMock

import serial_handler


class _FakeSerial:
    active_count = 0
    max_active_count = 0
    counters_lock = threading.Lock()

    def __init__(self, *args, timeout=0.05, **kwargs):
        self.timeout = timeout
        self._read_queue = Queue()

    def __enter__(self):
        with self.counters_lock:
            type(self).active_count += 1
            type(self).max_active_count = max(
                type(self).max_active_count,
                type(self).active_count,
            )
        return self

    def __exit__(self, exc_type, exc, tb):
        with self.counters_lock:
            type(self).active_count -= 1

    def write(self, data):
        if b"GET_IDENTITY" in data:
            self._read_queue.put(
                b"RSP:IDENTITY(8):PROFILE=HIP_ROLL_BENCH_RIGHT:"
                b"STORED_PROFILE=HIP_ROLL_BENCH_RIGHT:SOURCE=FLASH:"
                b"UID=8242201C80B670BF:FW=0.1.0\n"
            )
        return len(data)

    def flush(self):
        return None

    def reset_input_buffer(self):
        return None

    def reset_output_buffer(self):
        return None

    def readline(self):
        try:
            return self._read_queue.get(timeout=self.timeout)
        except Empty:
            return b""


def test_get_board_identity_serializes_with_background_listener(monkeypatch):
    monkeypatch.setattr(serial_handler.serial, "Serial", _FakeSerial)
    _FakeSerial.active_count = 0
    _FakeSerial.max_active_count = 0

    fake_socketio = MagicMock()
    fake_logger = MagicMock()
    handler = serial_handler.SerialHandler(
        fake_socketio,
        "/dev/cu.fake-joint",
        serial_logger=fake_logger,
    )

    listener_thread = threading.Thread(target=handler.listen_for_messages, daemon=True)
    listener_thread.start()

    assert handler.listener_serial_active.wait(timeout=0.5)

    identity = handler.get_board_identity()

    assert identity is not None
    assert identity["profile"] == "HIP_ROLL_BENCH_RIGHT"
    assert _FakeSerial.max_active_count == 1

    handler.pause_listening()
    time.sleep(0.02)
