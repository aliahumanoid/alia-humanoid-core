"""
Unit tests for stream_test_service.py — continuous streaming test service.

Tests cover:
- Session lifecycle (start/stop/state machine)
- Trajectory generation (sinusoid values, chunk shape)
- Backpressure policy (defer, drop after threshold)
- Fault injection (delay, drop, burst pause)
- Metrics collection (rate, drift, counters)
- Time sync policy

Run with: python3 -m pytest tests/test_stream_test_service.py -v
"""
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock, patch

import pytest

# Ensure host/ is on sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from stream_test_service import (
    SessionState,
    StreamMetrics,
    StreamTestService,
    TrajectoryGenerator,
    _EventLog,
    _FaultInjector,
    _JointBackpressure,
    _StreamSession,
)
from waypoint_client import BatchRecord, BatchResponse, BatchState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
MINIMAL_CONFIG = {
    "joints": ["KNEE_LEFT"],
    "rate_hz": 50,
    "duration_s": 2,
    "horizon_ms": 250,
    "buffer_depth_sim": 2,
    "max_inflight_per_joint": 1,
    "trajectory": {
        "type": "sinusoid",
        "amplitude_deg": 8.0,
        "offset_deg": 0.0,
        "frequency_hz": 0.5,
    },
    "fault_profile": {"mode": "none"},
}


def _wait_state(service, target_state, timeout=5.0):
    """Poll until service reaches the target state."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = service.get_status()
        session = status.get("session")
        if session and session.get("state") == target_state:
            return True
        if status.get("state") == target_state:
            return True
        time.sleep(0.05)
    return False


# =======================================================================
# A. Session Lifecycle
# =======================================================================
class TestSessionLifecycle:

    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_start_transitions_to_running(self, mock_client_cls, mock_transport_cls):
        """Start should transition session to RUNNING."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "test_id"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        assert "session_id" in result
        assert result["state"] in ("STARTING", "RUNNING")

        # Wait for session to reach RUNNING or terminate
        time.sleep(0.5)
        status = svc.get_status()
        assert status["session"] is not None

        svc.stop(result["session_id"])
        time.sleep(0.5)

    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_stop_transitions_to_stopped(self, mock_client_cls, mock_transport_cls):
        """Stop should transition session to STOPPED."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "test_id"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        time.sleep(0.3)

        stop_result = svc.stop(result["session_id"])
        assert stop_result["state"] in ("STOPPED", "STOPPING")

        time.sleep(0.5)
        status = svc.get_status()
        assert status["session"]["state"] in ("STOPPED", "FAILED")

    def test_double_start_raises_error(self):
        """Starting a second session while one is active should raise."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")

        with patch("stream_test_service.HttpTransport"), \
             patch("stream_test_service.WaypointClient") as mock_cls:
            mock_client = MagicMock()
            mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
            mock_client.enqueue_batch.return_value = "t"
            mock_client.get_batch_status.return_value = None
            mock_cls.return_value = mock_client

            result = svc.start(MINIMAL_CONFIG)
            time.sleep(0.2)

            with pytest.raises(RuntimeError, match="already active"):
                svc.start(MINIMAL_CONFIG)

            svc.stop(result["session_id"])
            time.sleep(0.5)

    def test_stop_idempotent(self):
        """Stopping a non-existent session should return success."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.stop("nonexistent")
        assert result["state"] == "IDLE"

    def test_invalid_config_raises_valueerror(self):
        """Invalid config should raise ValueError."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        with pytest.raises(ValueError, match="joints"):
            svc.start({"rate_hz": 50})
        with pytest.raises(ValueError, match="rate_hz"):
            svc.start({"joints": ["KNEE_LEFT"], "rate_hz": 75})

    def test_status_when_idle(self):
        """Status with no session should return IDLE."""
        svc = StreamTestService()
        status = svc.get_status()
        assert status["state"] == "IDLE"
        assert status["session"] is None


# =======================================================================
# B. Trajectory Generation
# =======================================================================
class TestTrajectoryGen:

    def test_sinusoid_values(self):
        """Sinusoid trajectory should produce correct angles."""
        gen = TrajectoryGenerator(
            traj_config={
                "type": "sinusoid",
                "amplitude_deg": 10.0,
                "offset_deg": 5.0,
                "frequency_hz": 1.0,
            },
            rate_hz=100,
            horizon_ms=250,
        )
        # At t=0, sin(0)=0, angle should be offset
        chunk = gen.next_chunk(0.0)
        assert abs(chunk[0]["angles_deg"][0] - 5.0) < 0.1

        # At t=250ms (quarter period for 1Hz), sin(pi/2)=1
        chunk_250 = gen.next_chunk(250.0)
        expected = 5.0 + 10.0 * math.sin(2 * math.pi * 1.0 * 0.25)
        assert abs(chunk_250[0]["angles_deg"][0] - expected) < 0.5

    def test_chunk_shape(self):
        """Chunks should have 2-4 waypoints."""
        gen = TrajectoryGenerator(
            traj_config={"type": "sinusoid"},
            rate_hz=50,
            horizon_ms=250,
        )
        chunk = gen.next_chunk(0.0)
        assert 2 <= len(chunk) <= 4
        for wp in chunk:
            assert "angles_deg" in wp
            assert "t_offset_ms" in wp
            assert isinstance(wp["angles_deg"], list)
            assert isinstance(wp["t_offset_ms"], int)

    def test_t_offset_monotonically_increasing(self):
        """t_offset_ms values within a chunk must be strictly increasing."""
        gen = TrajectoryGenerator(
            traj_config={"type": "sinusoid"},
            rate_hz=100,
            horizon_ms=250,
        )
        chunk = gen.next_chunk(100.0)
        offsets = [wp["t_offset_ms"] for wp in chunk]
        for i in range(1, len(offsets)):
            assert offsets[i] > offsets[i - 1]

    def test_multiple_chunks_continuous(self):
        """Sequential chunks should have non-overlapping t_offset_ms."""
        gen = TrajectoryGenerator(
            traj_config={"type": "sinusoid"},
            rate_hz=50,
            horizon_ms=200,
        )
        chunk1 = gen.next_chunk(0.0)
        last_t1 = chunk1[-1]["t_offset_ms"]
        chunk2 = gen.next_chunk(last_t1 + 20)
        assert chunk2[0]["t_offset_ms"] > last_t1


# =======================================================================
# C. Backpressure
# =======================================================================
class TestBackpressure:

    def test_defer_when_full(self):
        """can_send() should return False when inflight >= max_inflight."""
        bp = _JointBackpressure(buffer_depth=2, max_inflight=1)
        bp.on_send()
        assert not bp.can_send()

    def test_allows_after_done(self):
        """can_send() should return True after on_done()."""
        bp = _JointBackpressure(buffer_depth=2, max_inflight=1)
        bp.on_send()
        assert not bp.can_send()
        bp.on_done()
        assert bp.can_send()

    def test_drop_after_consecutive_threshold(self):
        """should_drop should be True after >3 consecutive deferrals."""
        bp = _JointBackpressure(buffer_depth=2, max_inflight=1)
        bp.on_send()  # inflight = 1
        # 4 consecutive can_send() failures
        for _ in range(4):
            assert not bp.can_send()
        assert bp.should_drop

    def test_consecutive_reset_on_success(self):
        """Consecutive counter should reset when can_send() succeeds."""
        bp = _JointBackpressure(buffer_depth=2, max_inflight=1)
        bp.on_send()
        bp.can_send()  # fail, consecutive=1
        bp.can_send()  # fail, consecutive=2
        bp.on_done()
        bp.can_send()  # success, consecutive=0
        assert not bp.should_drop


# =======================================================================
# D. Fault Injection
# =======================================================================
class TestFaultInjection:

    def test_none_always_sends(self):
        fi = _FaultInjector({"mode": "none"})
        assert all(fi.pre_send(i) == "send" for i in range(100))

    def test_delay_jitter(self):
        fi = _FaultInjector({"mode": "delay_jitter"})
        assert fi.pre_send(0) == "delay"
        assert fi.delay_ms > 0

    def test_drop_rate_probabilistic(self):
        fi = _FaultInjector({"mode": "drop_rate", "drop_rate": 0.5, "seed": 42})
        results = [fi.pre_send(i) for i in range(1000)]
        drop_count = results.count("drop")
        # With 50% drop rate, should be roughly 500 ± reasonable margin
        assert 300 < drop_count < 700

    def test_burst_pause(self):
        fi = _FaultInjector({
            "mode": "burst_pause",
            "burst_period": 10,
            "burst_pause_ms": 100,
        })
        assert fi.pre_send(0) == "send"  # tick 0 is skipped
        assert fi.pre_send(10) == "pause"
        assert fi.pre_send(11) == "send"
        assert fi.pre_send(20) == "pause"

    def test_sync_stale_suppress(self):
        fi = _FaultInjector({"mode": "sync_stale"})
        assert fi.suppress_sync is True
        assert fi.pre_send(0) == "send"  # Normal send, but sync disabled

    def test_conflict_spike(self):
        fi = _FaultInjector({"mode": "conflict_spike"})
        assert fi.conflict_spike is True
        assert fi.pre_send(0) == "send"


# =======================================================================
# E. Metrics
# =======================================================================
class TestMetrics:

    def test_snapshot_schema(self):
        """Metrics snapshot should contain all required fields."""
        m = StreamMetrics(target_rate_hz=50, joints=["KNEE_LEFT"])
        snap = m.snapshot()
        required_keys = [
            "target_rate_hz", "actual_rate_hz", "scheduler_drift_ms_p95",
            "chunks_sent", "chunks_dropped", "chunks_deferred",
            "waypoints_sent", "waypoints_partial", "http_status_counts",
            "retries", "queue_fill_max", "late_ratio",
            "sync_refresh_count", "last_error",
        ]
        for key in required_keys:
            assert key in snap, f"Missing key: {key}"

    def test_drift_tracking(self):
        """Drift p95 should reflect recorded samples."""
        m = StreamMetrics(target_rate_hz=100, joints=["J"])
        for i in range(100):
            m.record_drift(float(i))
        snap = m.snapshot()
        # p95 of 0..99 should be around 95
        assert snap["scheduler_drift_ms_p95"] >= 90

    def test_counters_increment(self):
        m = StreamMetrics(target_rate_hz=50, joints=["KNEE_LEFT"])
        m.inc_sent("KNEE_LEFT", 3)
        m.inc_sent("KNEE_LEFT", 2)
        m.inc_dropped("KNEE_LEFT")
        m.inc_deferred("KNEE_LEFT")
        m.inc_sync_refresh()
        snap = m.snapshot()
        assert snap["chunks_sent"] == 2
        assert snap["waypoints_sent"] == 5
        assert snap["chunks_dropped"] == 1
        assert snap["chunks_deferred"] == 1
        assert snap["sync_refresh_count"] == 1

    def test_client_metrics_merge(self):
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.merge_client_metrics({
            "status_counts": {200: 10, 409: 2},
            "retries": 3,
        })
        snap = m.snapshot()
        assert snap["http_status_counts"] == {200: 10, 409: 2}
        assert snap["retries"] == 3

    def test_queue_fill_max(self):
        m = StreamMetrics(target_rate_hz=50, joints=["A", "B"])
        m.update_queue_fill("A", 1)
        m.update_queue_fill("A", 2)
        m.update_queue_fill("A", 1)  # max stays at 2
        m.update_queue_fill("B", 1)
        snap = m.snapshot()
        assert snap["queue_fill_max"]["A"] == 2
        assert snap["queue_fill_max"]["B"] == 1


# =======================================================================
# F. Event Log
# =======================================================================
class TestEventLog:

    def test_emit_and_since(self):
        log = _EventLog()
        log.emit("INFO", "test_event", "KNEE_LEFT", "detail")
        log.emit("WARN", "warning", "KNEE_LEFT", "detail2")
        events = log.since(0)
        assert len(events) == 2
        assert events[0]["seq"] == 1
        assert events[1]["seq"] == 2
        assert events[0]["event"] == "test_event"

    def test_since_filters(self):
        log = _EventLog()
        log.emit("INFO", "a")
        log.emit("INFO", "b")
        log.emit("INFO", "c")
        events = log.since(2)
        assert len(events) == 1
        assert events[0]["event"] == "c"

    def test_bounded_size(self):
        log = _EventLog()
        log.MAX_EVENTS = 100
        log.TRIM_TO = 50
        for i in range(150):
            log.emit("INFO", f"event_{i}")
        all_events = log.all()
        assert len(all_events) <= 100


# =======================================================================
# G. Service API
# =======================================================================
class TestServiceAPI:

    def test_get_events_when_idle(self):
        svc = StreamTestService()
        assert svc.get_events() == []

    def test_get_metrics_when_idle(self):
        svc = StreamTestService()
        assert svc.get_metrics() == {}

    def test_unknown_session_stop(self):
        """Stopping with wrong session_id should raise ValueError."""
        svc = StreamTestService()
        with patch("stream_test_service.HttpTransport"), \
             patch("stream_test_service.WaypointClient") as mock_cls:
            mock_client = MagicMock()
            mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
            mock_client.enqueue_batch.return_value = "t"
            mock_client.get_batch_status.return_value = None
            mock_cls.return_value = mock_client

            result = svc.start(MINIMAL_CONFIG)
            time.sleep(0.2)

            with pytest.raises(ValueError, match="Unknown session_id"):
                svc.stop("wrong_id")

            svc.stop(result["session_id"])
            time.sleep(0.5)
