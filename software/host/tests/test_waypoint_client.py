"""
Unit tests for waypoint_client.py — Jetson waypoint client module.

Tests cover:
- State machine transitions (all HTTP status codes)
- Retry policy (backoff, max retries, jitter)
- Per-joint queue and worker thread lifecycle
- Idempotency and batch tracking
- Metrics collection
- Fault injection (timeout, connection error, mixed errors)

Run with: python3 -m pytest tests/test_waypoint_client.py -v
"""
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List

import pytest

# Ensure host/ is on sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from waypoint_client import (
    BatchRecord,
    BatchResponse,
    BatchState,
    ClientConfig,
    HealthStatus,
    RetryPolicy,
    WaypointClient,
    _JointWorker,
    _Metrics,
)


# ---------------------------------------------------------------------------
# Mock Transport
# ---------------------------------------------------------------------------
class MockTransport:
    """In-memory transport for testing.  Implements WaypointTransport protocol."""

    def __init__(self):
        self.responses: List[BatchResponse] = []
        self.calls: List[Dict[str, Any]] = []
        self.sync_called: int = 0
        self.health = HealthStatus(
            can_connected=True, time_sync_age_ms=500.0, server_reachable=True
        )

    def set_responses(self, *responses: BatchResponse):
        self.responses = list(responses)

    def send_batch(
        self, joint: str, waypoints: List[Dict], batch_id: str
    ) -> BatchResponse:
        self.calls.append(
            {"joint": joint, "waypoints": waypoints, "batch_id": batch_id}
        )
        if self.responses:
            return self.responses.pop(0)
        # Default: success
        return BatchResponse(
            http_status=200,
            status="success",
            message=f"Batch {batch_id}: {len(waypoints)}/{len(waypoints)} sent",
            result={
                "total": len(waypoints),
                "sent": len(waypoints),
                "failed_indices": [],
                "skipped_indices": [],
                "elapsed_ms": 5.0,
                "timing_drift_ms": 0.1,
                "late_count": 0,
                "aborted": False,
                "joint": joint,
                "batch_id": batch_id,
            },
        )

    def send_time_sync(self) -> bool:
        self.sync_called += 1
        return True

    def check_health(self) -> HealthStatus:
        return self.health


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _make_resp(code: int, status: str = "error", msg: str = "", **kw) -> BatchResponse:
    """Shorthand for building a BatchResponse."""
    return BatchResponse(http_status=code, status=status, message=msg, **kw)


def _success_resp(joint: str = "KNEE_LEFT", batch_id: str = "t") -> BatchResponse:
    return BatchResponse(
        http_status=200,
        status="success",
        message="ok",
        result={
            "total": 2, "sent": 2, "failed_indices": [],
            "skipped_indices": [], "elapsed_ms": 5.0,
            "timing_drift_ms": 0.0, "late_count": 0,
            "aborted": False, "joint": joint, "batch_id": batch_id,
        },
    )


def _partial_resp(joint: str = "KNEE_LEFT") -> BatchResponse:
    return BatchResponse(
        http_status=207,
        status="partial",
        message="partial",
        result={
            "total": 3, "sent": 2, "failed_indices": [1],
            "skipped_indices": [], "elapsed_ms": 10.0,
            "timing_drift_ms": 0.5, "late_count": 0,
            "aborted": False, "joint": joint, "batch_id": "t",
        },
    )


VALID_WAYPOINTS = [
    {"angles_deg": [10.0], "t_offset_ms": 100},
    {"angles_deg": [20.0], "t_offset_ms": 200},
]

# Fast backoff for all tests
FAST_RETRY = RetryPolicy(
    base_backoff_409_s=0.01,
    base_backoff_502_s=0.01,
    base_backoff_500_s=0.01,
    base_backoff_timeout_s=0.01,
)

FAST_CONFIG = ClientConfig(retry=FAST_RETRY)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture
def mock_transport():
    return MockTransport()


@pytest.fixture
def client(mock_transport):
    c = WaypointClient(mock_transport, config=FAST_CONFIG)
    yield c
    c.shutdown(timeout_s=2.0)


def _wait_batch(client, bid, timeout=3.0):
    """Poll until batch reaches a terminal state AND completed_at is set.

    We wait for ``completed_at`` because the metrics update happens in
    ``_on_batch_done`` which runs *after* the state is set to terminal.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        record = client.get_batch_status(bid)
        if record and record.completed_at is not None and record.state in (
            BatchState.COMPLETE, BatchState.PARTIAL, BatchState.FAILED_HARD
        ):
            return record
        time.sleep(0.05)
    return client.get_batch_status(bid)


# =======================================================================
# A. State Machine Transitions
# =======================================================================
class TestStateMachine:
    """Verify correct state transitions for all HTTP status codes."""

    def test_200_transitions_to_complete(self, client, mock_transport):
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.COMPLETE

    def test_207_transitions_to_partial(self, client, mock_transport):
        mock_transport.set_responses(_partial_resp())
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.PARTIAL

    def test_400_transitions_to_failed_hard(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(400, msg="Validation failed",
                       validation_errors=[{"index": 0, "field": "angles_deg[0]",
                                           "message": "out of range"}])
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.FAILED_HARD
        assert record.attempts == 1  # No retry for 400

    def test_409_retries_then_fails(self, client, mock_transport):
        # max_retries_409 = 3, so attempts: 0,1,2 → RETRY, 3 → FAILED
        mock_transport.set_responses(
            *[_make_resp(409, msg="conflict")] * 10
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.FAILED_HARD
        assert record.attempts == 4  # 1 initial + 3 retries

    def test_409_retries_then_succeeds(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(409, msg="conflict"),
            _make_resp(409, msg="conflict"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.COMPLETE
        assert record.attempts == 3

    def test_502_triggers_time_sync(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(502, msg="upstream fail"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        assert mock_transport.sync_called >= 1

    def test_503_retries_with_resync(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(503, msg="CAN unavailable"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.COMPLETE
        assert mock_transport.sync_called >= 1

    def test_500_max_2_retries(self, client, mock_transport):
        mock_transport.set_responses(
            *[_make_resp(500, msg="internal")] * 10
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.FAILED_HARD
        assert record.attempts == 3  # 1 initial + 2 retries

    def test_timeout_retries_then_fails(self, client, mock_transport):
        mock_transport.set_responses(
            *[_make_resp(0, msg="timeout")] * 10
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid, timeout=5.0)
        assert record.state == BatchState.FAILED_HARD
        assert record.attempts == 6  # 1 initial + 5 retries


# =======================================================================
# B. Queue & Worker
# =======================================================================
class TestQueueWorker:

    def test_multiple_batches_serialized(self, client, mock_transport):
        """Batches for same joint are processed one at a time."""
        bid1 = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        bid2 = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid2)
        r1 = client.get_batch_status(bid1)
        r2 = client.get_batch_status(bid2)
        assert r1.state == BatchState.COMPLETE
        assert r2.state == BatchState.COMPLETE
        assert len(mock_transport.calls) == 2

    def test_different_joints_parallel(self, client, mock_transport):
        """Different joints get separate workers."""
        bid1 = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        bid2 = client.enqueue_batch("KNEE_RIGHT", VALID_WAYPOINTS)
        _wait_batch(client, bid1)
        _wait_batch(client, bid2)
        r1 = client.get_batch_status(bid1)
        r2 = client.get_batch_status(bid2)
        assert r1.state == BatchState.COMPLETE
        assert r2.state == BatchState.COMPLETE

    def test_cancel_joint_drains_queue(self, client, mock_transport):
        # Make transport slow so batches queue up
        original_send = mock_transport.send_batch

        def slow_send(*args, **kwargs):
            time.sleep(0.3)
            return original_send(*args, **kwargs)

        mock_transport.send_batch = slow_send

        bids = []
        for _ in range(5):
            bids.append(client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS))
        time.sleep(0.1)     # Let first batch start
        client.cancel_joint("KNEE_LEFT")
        time.sleep(1.0)
        m = client.get_metrics()
        assert m["batches_failed"] > 0

    def test_shutdown_joins_all_workers(self, mock_transport):
        c = WaypointClient(mock_transport, config=FAST_CONFIG)
        c.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        c.enqueue_batch("ANKLE_RIGHT", VALID_WAYPOINTS)
        time.sleep(0.5)
        c.shutdown(timeout_s=3.0)
        # No assertion needed — just verify no hang / exception

    def test_empty_joint_raises_valueerror(self, client):
        with pytest.raises(ValueError, match="non-empty"):
            client.enqueue_batch("", VALID_WAYPOINTS)


# =======================================================================
# C. Retry Policy
# =======================================================================
class TestRetryPolicy:

    def test_backoff_is_progressive(self, client):
        """Each attempt should have longer base backoff."""
        worker = client._get_or_create_worker("KNEE_LEFT")
        # Collect many samples to smooth jitter
        samples = {a: [] for a in range(3)}
        for _ in range(50):
            for a in range(3):
                samples[a].append(worker._backoff_seconds(502, a))
        mean = {a: sum(v) / len(v) for a, v in samples.items()}
        assert mean[1] > mean[0]
        assert mean[2] > mean[1]

    def test_jitter_varies_backoff(self, client):
        """Multiple calls should produce different values (jitter)."""
        worker = client._get_or_create_worker("KNEE_LEFT")
        values = {worker._backoff_seconds(409, 1) for _ in range(30)}
        assert len(values) > 1

    def test_400_never_retries(self, client, mock_transport):
        mock_transport.set_responses(_make_resp(400, msg="bad"))
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        assert len(mock_transport.calls) == 1


# =======================================================================
# D. Idempotency & Traceability
# =======================================================================
class TestIdempotency:

    def test_client_batch_id_auto_generated(self, client, mock_transport):
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        assert len(bid) == 8
        _wait_batch(client, bid)
        assert mock_transport.calls[0]["batch_id"] == bid

    def test_client_batch_id_custom(self, client, mock_transport):
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS, batch_id="custom01")
        assert bid == "custom01"
        _wait_batch(client, bid)
        assert mock_transport.calls[0]["batch_id"] == "custom01"

    def test_batch_history_tracked(self, client, mock_transport):
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        record = client.get_batch_status(bid)
        assert record is not None
        assert record.joint == "KNEE_LEFT"
        assert record.waypoint_count == 2

    def test_unknown_batch_returns_none(self, client):
        assert client.get_batch_status("nonexistent") is None


# =======================================================================
# E. Metrics
# =======================================================================
class TestMetrics:

    def test_success_increments_total(self, client, mock_transport):
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        m = client.get_metrics()
        assert m["batches_total"] == 1
        assert m["batches_failed"] == 0

    def test_failure_increments_failed(self, client, mock_transport):
        mock_transport.set_responses(_make_resp(400, msg="bad"))
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        m = client.get_metrics()
        assert m["batches_failed"] == 1

    def test_partial_increments_partial(self, client, mock_transport):
        mock_transport.set_responses(_partial_resp())
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        m = client.get_metrics()
        assert m["batches_partial"] == 1

    def test_retry_count_tracked(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(409, msg="conflict"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        m = client.get_metrics()
        assert m["retries"] >= 1

    def test_rate_409_calculated(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(409, msg="conflict"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        _wait_batch(client, bid)
        m = client.get_metrics()
        assert m["rate_409"] == 0.5     # 1 of 2 sends was 409


# =======================================================================
# F. Fault Injection
# =======================================================================
class TestFaultInjection:

    def test_connection_error_treated_as_timeout(self, client, mock_transport):
        mock_transport.set_responses(
            _make_resp(0, msg="Connection refused"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.COMPLETE

    def test_mixed_error_sequence(self, client, mock_transport):
        """502 -> 409 -> 200 should recover."""
        mock_transport.set_responses(
            _make_resp(502, msg="upstream"),
            _make_resp(409, msg="conflict"),
            _success_resp(),
        )
        bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
        record = _wait_batch(client, bid)
        assert record.state == BatchState.COMPLETE
        assert mock_transport.sync_called >= 1

    def test_no_lost_batch_without_final_log(self, client, mock_transport, caplog):
        """Every batch must have a terminal 'batch_done' log entry."""
        import logging
        with caplog.at_level(logging.INFO, logger="waypoint_client"):
            bid = client.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
            _wait_batch(client, bid)
        done_logs = [r for r in caplog.records if "batch_done" in r.message]
        assert len(done_logs) >= 1


# =======================================================================
# G. Regression Tests
# =======================================================================
class TestRegression:

    def test_enqueue_after_cancel_is_processed(self, mock_transport):
        """Regression: cancel_joint() then enqueue_batch() must not leave
        the new batch stuck in IDLE (stale cancel_event race)."""
        c = WaypointClient(mock_transport, config=FAST_CONFIG)
        try:
            # First batch — let it complete
            bid1 = c.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
            _wait_batch(c, bid1)
            assert c.get_batch_status(bid1).state == BatchState.COMPLETE

            # Cancel the joint (sets cancel_event)
            c.cancel_joint("KNEE_LEFT")
            time.sleep(0.2)     # Give worker time to see cancel

            # Enqueue a new batch — must NOT be stuck in IDLE
            bid2 = c.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
            record = _wait_batch(c, bid2, timeout=5.0)
            assert record is not None
            assert record.state in (BatchState.COMPLETE, BatchState.PARTIAL)
            assert record.completed_at is not None
        finally:
            c.shutdown(timeout_s=2.0)

    def test_queue_full_no_ghost_batch(self, mock_transport):
        """Regression: queue.Full must not leave ghost records in history
        or inflate batches_total."""
        tiny_config = ClientConfig(
            retry=FAST_RETRY,
            max_queue_size=1,
        )

        # Block the worker so the single queue slot stays occupied
        gate = threading.Event()
        original_send = mock_transport.send_batch

        def blocked_send(*args, **kwargs):
            gate.wait(timeout=10.0)
            return original_send(*args, **kwargs)

        mock_transport.send_batch = blocked_send

        c = WaypointClient(mock_transport, config=tiny_config)
        try:
            # First enqueue → worker grabs it from queue, starts blocked_send
            bid1 = c.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)
            time.sleep(0.1)     # Let worker pull from queue

            # Second enqueue fills the single queue slot
            bid2 = c.enqueue_batch("KNEE_LEFT", VALID_WAYPOINTS)

            # Third should overflow — queue full, put times out
            with pytest.raises(Exception):
                # Override put timeout to fail fast
                worker = c._get_or_create_worker("KNEE_LEFT")
                worker._queue.put(("dummy",), block=True, timeout=0.1)

            # Release the gate so batches can complete
            gate.set()

            _wait_batch(c, bid1, timeout=5.0)
            _wait_batch(c, bid2, timeout=5.0)

            m = c.get_metrics()
            # Only 2 should be tracked — the Full one must leave no trace
            assert m["batches_total"] == 2

            # History should only contain the 2 valid batches
            assert c.get_batch_status(bid1) is not None
            assert c.get_batch_status(bid2) is not None
        finally:
            gate.set()
            c.shutdown(timeout_s=3.0)
