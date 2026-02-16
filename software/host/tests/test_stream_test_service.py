"""
Unit tests for stream_test_service.py — continuous streaming test service.

Tests cover:
- Session lifecycle (start/stop/state machine)
- Config validation (Phase 1: single joint, min/max, safe_limits)
- Trajectory generation (bounded cosine, null DOFs, start_at)
- Backpressure policy (defer, drop after threshold)
- Fault injection (delay, drop, burst pause)
- Metrics collection (rate, drift, counters)
- Time sync policy
- Preposition phase (encoder check, ready criterion)

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
    _TelemetryPoller,
)
from waypoint_client import BatchRecord, BatchResponse, BatchState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
MINIMAL_CONFIG = {
    "joint": "KNEE_LEFT",
    "active_dof": 0,
    "n_dof": 1,
    "min_deg": 20.0,
    "max_deg": 80.0,
    "start_at": "min",
    "frequency_hz": 0.5,
    "rate_hz": 50,
    "duration_s": 2,
    "horizon_ms": 250,
    "buffer_depth_sim": 2,
    "max_inflight_per_joint": 1,
    "fault_profile": {"mode": "none"},
    "safe_limits": {"min": -2.0, "max": 112.0},
}


def _mock_encoder_response(angles_deg=None, age_ms=50, valid=True):
    """Build a mock requests.Response for GET /can/encoder_angles."""
    if angles_deg is None:
        angles_deg = [45.0]
    resp = MagicMock()
    resp.json.return_value = {
        "status": "success",
        "valid": valid,
        "age_ms": age_ms,
        "angles_deg": angles_deg,
    }
    resp.status_code = 200
    return resp


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
# A. Config Validation
# =======================================================================
class TestConfigValidation:

    def test_joint_string_required(self):
        """'joint' must be a non-empty string, not a list."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        with pytest.raises(ValueError, match="joint"):
            svc.start({"joints": ["KNEE_LEFT"], "rate_hz": 50,
                        "min_deg": 20, "max_deg": 80})
        with pytest.raises(ValueError, match="joint"):
            svc.start({"joint": "", "rate_hz": 50,
                        "min_deg": 20, "max_deg": 80})

    def test_invalid_rate_hz(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "rate_hz": 75}
        with pytest.raises(ValueError, match="rate_hz"):
            svc.start(cfg)

    def test_min_gte_max_raises(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "min_deg": 80.0, "max_deg": 20.0}
        with pytest.raises(ValueError, match="min_deg must be < max_deg"):
            svc.start(cfg)

    def test_min_equal_max_raises(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "min_deg": 50.0, "max_deg": 50.0}
        with pytest.raises(ValueError, match="min_deg must be < max_deg"):
            svc.start(cfg)

    def test_range_exceeds_safe_limits(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "min_deg": 10.0, "max_deg": 120.0,
               "safe_limits": {"min": 0.0, "max": 112.0}}
        with pytest.raises(ValueError, match="exceeds safe limits"):
            svc.start(cfg)

    def test_active_dof_out_of_range(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "active_dof": 2, "n_dof": 1}
        with pytest.raises(ValueError, match="active_dof"):
            svc.start(cfg)

    def test_active_dof_negative(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "active_dof": -1}
        with pytest.raises(ValueError, match="active_dof"):
            svc.start(cfg)

    def test_non_finite_min_raises(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "min_deg": float("nan")}
        with pytest.raises(ValueError, match="finite"):
            svc.start(cfg)

    def test_non_finite_max_raises(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "max_deg": float("inf")}
        with pytest.raises(ValueError, match="finite"):
            svc.start(cfg)

    def test_invalid_start_at(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "start_at": "middle"}
        with pytest.raises(ValueError, match="start_at"):
            svc.start(cfg)

    def test_zero_frequency_raises(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "frequency_hz": 0}
        with pytest.raises(ValueError, match="frequency_hz"):
            svc.start(cfg)

    def test_missing_min_max_raises(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG}
        del cfg["min_deg"]
        with pytest.raises(ValueError, match="min_deg"):
            svc.start(cfg)

    def test_safe_limits_required(self):
        """P1: safe_limits must be present — omitting it must raise."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG}
        del cfg["safe_limits"]
        with pytest.raises(ValueError, match="safe_limits"):
            svc.start(cfg)

    def test_safe_limits_null_rejected(self):
        """P1: safe_limits=None must raise."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "safe_limits": None}
        with pytest.raises(ValueError, match="safe_limits"):
            svc.start(cfg)

    def test_safe_limits_incomplete_rejected(self):
        """safe_limits without 'max' key must raise ValueError, not KeyError."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "safe_limits": {"min": 0.0}}
        with pytest.raises(ValueError, match="min.*max"):
            svc.start(cfg)

    def test_n_dof_mismatch_raises(self):
        """n_dof must match real joint DOF count from config.JOINTS."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        # KNEE_LEFT has 1 DOF — sending n_dof=3 should be rejected
        cfg = {**MINIMAL_CONFIG, "active_dof": 2, "n_dof": 3}
        with pytest.raises(ValueError, match="does not match"):
            svc.start(cfg)

    def test_string_min_max_coerced(self):
        """P2: string min_deg/max_deg should be coerced to float, not 500."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "min_deg": "20", "max_deg": "80"}
        # Should not raise TypeError — gets coerced and passes validation
        with patch("stream_test_service.HttpTransport"), \
             patch("stream_test_service.WaypointClient") as mock_cls, \
             patch("stream_test_service.http_requests") as mock_req:
            mock_client = MagicMock()
            mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
            mock_client.enqueue_batch.return_value = "t"
            mock_client.get_batch_status.return_value = None
            mock_cls.return_value = mock_client
            mock_req.get.return_value = _mock_encoder_response([45.0])

            result = svc.start(cfg)
            assert "session_id" in result
            time.sleep(0.3)
            svc.stop(result["session_id"])

    def test_non_numeric_string_min_raises(self):
        """P2: non-numeric string min_deg should raise ValueError, not TypeError."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        cfg = {**MINIMAL_CONFIG, "min_deg": "abc"}
        with pytest.raises(ValueError, match="numeric"):
            svc.start(cfg)

    def test_valid_config_accepted(self):
        """A valid config should not raise."""
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        with patch("stream_test_service.HttpTransport"), \
             patch("stream_test_service.WaypointClient") as mock_cls, \
             patch("stream_test_service.http_requests") as mock_req:
            mock_client = MagicMock()
            mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
            mock_client.enqueue_batch.return_value = "t"
            mock_client.get_batch_status.return_value = None
            mock_cls.return_value = mock_client

            mock_req.get.return_value = _mock_encoder_response([45.0])

            result = svc.start(MINIMAL_CONFIG)
            assert "session_id" in result
            time.sleep(0.3)
            svc.stop(result["session_id"])
            time.sleep(0.5)


# =======================================================================
# B. Session Lifecycle
# =======================================================================
class TestSessionLifecycle:

    @patch("stream_test_service.http_requests")
    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_start_transitions_through_prepositioning(
        self, mock_client_cls, mock_transport_cls, mock_req
    ):
        """Start should transition STARTING → PREPOSITIONING → RUNNING."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "test_id"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        mock_req.get.return_value = _mock_encoder_response([20.0])

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        assert "session_id" in result

        # Wait for session to reach RUNNING (after preposition)
        reached = _wait_state(svc, "RUNNING", timeout=8.0)
        assert reached, "Session did not reach RUNNING"

        svc.stop(result["session_id"])
        time.sleep(0.5)

    @patch("stream_test_service.http_requests")
    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_stop_transitions_to_stopped(
        self, mock_client_cls, mock_transport_cls, mock_req
    ):
        """Stop should transition session to STOPPED."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "test_id"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        mock_req.get.return_value = _mock_encoder_response([20.0])

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        _wait_state(svc, "RUNNING", timeout=8.0)

        stop_result = svc.stop(result["session_id"])
        assert stop_result["state"] in ("STOPPED", "STOPPING")

        time.sleep(0.5)
        status = svc.get_status()
        assert status["session"]["state"] in ("STOPPED", "FAILED")

    def test_double_start_raises_error(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")

        with patch("stream_test_service.HttpTransport"), \
             patch("stream_test_service.WaypointClient") as mock_cls, \
             patch("stream_test_service.http_requests") as mock_req:
            mock_client = MagicMock()
            mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
            mock_client.enqueue_batch.return_value = "t"
            mock_client.get_batch_status.return_value = None
            mock_cls.return_value = mock_client

            mock_req.get.return_value = _mock_encoder_response([20.0])

            result = svc.start(MINIMAL_CONFIG)
            time.sleep(0.2)

            with pytest.raises(RuntimeError, match="already active"):
                svc.start(MINIMAL_CONFIG)

            svc.stop(result["session_id"])
            time.sleep(0.5)

    def test_stop_idempotent(self):
        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.stop("nonexistent")
        assert result["state"] == "IDLE"

    def test_status_when_idle(self):
        svc = StreamTestService()
        status = svc.get_status()
        assert status["state"] == "IDLE"
        assert status["session"] is None


# =======================================================================
# C. Trajectory Generation (Bounded Cosine)
# =======================================================================
class TestTrajectoryGen:

    def test_bounded_cosine_stays_in_range(self):
        """All generated angles must stay within [min_deg, max_deg]."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=50, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        for tick in range(1000):
            t_base_ms = tick * 20.0  # 50Hz → 20ms period
            chunk = gen.next_chunk(t_base_ms, lead_ms=250.0)
            for wp in chunk:
                angle = wp["angles_deg"][0]
                assert 19.99 <= angle <= 80.01, (
                    f"angle={angle} out of [20, 80] at tick={tick}"
                )

    def test_start_at_min(self):
        """First angle should be approx min_deg when start_at='min'."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=50, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        chunk = gen.next_chunk(0.0, lead_ms=250.0)
        # cos(0)=1, center - amplitude*cos(0) = 50 - 30 = 20
        assert abs(chunk[0]["angles_deg"][0] - 20.0) < 0.1

    def test_start_at_max(self):
        """First angle should be approx max_deg when start_at='max'."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="max",
            frequency_hz=0.5, rate_hz=50, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        chunk = gen.next_chunk(0.0, lead_ms=250.0)
        # center + amplitude*cos(0) = 50 + 30 = 80
        assert abs(chunk[0]["angles_deg"][0] - 80.0) < 0.1

    def test_null_dofs(self):
        """Non-active DOFs should be None."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=50, horizon_ms=250,
            active_dof=1, n_dof=3,
        )
        chunk = gen.next_chunk(0.0, lead_ms=250.0)
        for wp in chunk:
            assert wp["angles_deg"][0] is None
            assert wp["angles_deg"][1] is not None  # active DOF
            assert wp["angles_deg"][2] is None

    def test_chunk_shape(self):
        """Chunks should have 2-4 waypoints with correct structure."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=50, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        chunk = gen.next_chunk(0.0)
        assert 2 <= len(chunk) <= 4
        for wp in chunk:
            assert "angles_deg" in wp
            assert "t_offset_ms" in wp
            assert isinstance(wp["angles_deg"], list)
            assert isinstance(wp["t_offset_ms"], int)

    def test_t_offset_monotonically_increasing(self):
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=100, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        chunk = gen.next_chunk(100.0, lead_ms=250.0)
        offsets = [wp["t_offset_ms"] for wp in chunk]
        for i in range(1, len(offsets)):
            assert offsets[i] > offsets[i - 1]

    def test_t_offset_never_exceeds_uint16(self):
        """P1 regression: t_offset_ms must stay within [1, 65535]."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=50, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        t_base_ms = 600_000.0  # 10 minutes
        chunk = gen.next_chunk(t_base_ms, lead_ms=250.0)
        for wp in chunk:
            assert 1 <= wp["t_offset_ms"] <= 65535

    def test_t_offset_stays_bounded_at_extreme_session(self):
        """Even at 10-minute session with 100Hz, all offsets are bounded."""
        gen = TrajectoryGenerator(
            min_deg=20.0, max_deg=80.0, start_at="min",
            frequency_hz=0.5, rate_hz=100, horizon_ms=250,
            active_dof=0, n_dof=1,
        )
        for tick in range(0, 60_000, 1000):
            t_base_ms = tick * 10.0
            chunk = gen.next_chunk(t_base_ms, lead_ms=250.0)
            for wp in chunk:
                assert wp["t_offset_ms"] <= 65535

    def test_zero_initial_velocity(self):
        """Cosine at t=0 has zero derivative → smooth start."""
        gen = TrajectoryGenerator(
            min_deg=0.0, max_deg=100.0, start_at="min",
            frequency_hz=1.0, rate_hz=1000, horizon_ms=10,
            active_dof=0, n_dof=1,
        )
        # Get first two waypoints at t=0 (1ms apart at 1000Hz)
        chunk = gen.next_chunk(0.0, lead_ms=10.0)
        a0 = chunk[0]["angles_deg"][0]
        a1 = chunk[1]["angles_deg"][0]
        # Angular velocity should be very small at start
        velocity_deg_per_ms = abs(a1 - a0)
        assert velocity_deg_per_ms < 1.0, (
            f"velocity={velocity_deg_per_ms} deg/ms too high at t=0"
        )


# =======================================================================
# D. Preposition Phase
# =======================================================================
class TestPreposition:

    @patch("stream_test_service.http_requests")
    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_encoder_stale_fails(
        self, mock_client_cls, mock_transport_cls, mock_req
    ):
        """Session should fail if encoder data is stale during preposition."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "t"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        # Stale encoder: age_ms=500 (> 300 threshold)
        mock_req.get.return_value = _mock_encoder_response(
            angles_deg=[45.0], age_ms=500, valid=True,
        )

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        reached = _wait_state(svc, "FAILED", timeout=5.0)
        assert reached, "Session did not fail on stale encoder"

    @patch("stream_test_service.http_requests")
    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_encoder_invalid_fails(
        self, mock_client_cls, mock_transport_cls, mock_req
    ):
        """Session should fail if encoder reports valid=false."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "t"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        mock_req.get.return_value = _mock_encoder_response(
            angles_deg=[45.0], age_ms=50, valid=False,
        )

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        reached = _wait_state(svc, "FAILED", timeout=5.0)
        assert reached, "Session did not fail on invalid encoder"

    @patch("stream_test_service.http_requests")
    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_encoder_null_dof_fails(
        self, mock_client_cls, mock_transport_cls, mock_req
    ):
        """Session should fail if active DOF has no encoder reading."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "t"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        # DOF 0 has None reading
        mock_req.get.return_value = _mock_encoder_response(
            angles_deg=[None], age_ms=50, valid=True,
        )

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)
        reached = _wait_state(svc, "FAILED", timeout=5.0)
        assert reached, "Session did not fail on null DOF encoder"

    @patch("stream_test_service.http_requests")
    @patch("stream_test_service.HttpTransport")
    @patch("stream_test_service.WaypointClient")
    def test_preposition_ramp_sent(
        self, mock_client_cls, mock_transport_cls, mock_req
    ):
        """Preposition should send a ramp batch via WaypointClient."""
        mock_client = MagicMock()
        mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
        mock_client.enqueue_batch.return_value = "ramp_id"
        mock_client.get_batch_status.return_value = None
        mock_client_cls.return_value = mock_client

        mock_transport = MagicMock()
        mock_transport.send_time_sync.return_value = True
        mock_transport_cls.return_value = mock_transport

        # Encoder at 45°, target is min_deg=20
        mock_req.get.return_value = _mock_encoder_response([45.0])

        svc = StreamTestService(base_url="http://127.0.0.1:5001")
        result = svc.start(MINIMAL_CONFIG)

        # Wait for preposition to send the ramp
        time.sleep(1.0)
        # First enqueue_batch call should be the ramp (20 waypoints)
        assert mock_client.enqueue_batch.called
        first_call = mock_client.enqueue_batch.call_args_list[0]
        joint_arg = first_call[0][0]
        waypoints_arg = first_call[0][1]
        assert joint_arg == "KNEE_LEFT"
        assert len(waypoints_arg) == 20
        # Ramp should end at target (min_deg=20.0)
        last_angle = waypoints_arg[-1]["angles_deg"][0]
        assert abs(last_angle - 20.0) < 0.1

        svc.stop(result["session_id"])
        time.sleep(0.5)


# =======================================================================
# E. Backpressure
# =======================================================================
class TestBackpressure:

    def test_defer_when_full(self):
        bp = _JointBackpressure(max_inflight=1)
        bp.on_send()
        assert not bp.can_send()

    def test_allows_after_done(self):
        bp = _JointBackpressure(max_inflight=1)
        bp.on_send()
        assert not bp.can_send()
        bp.on_done()
        assert bp.can_send()

    def test_drop_after_consecutive_threshold(self):
        bp = _JointBackpressure(max_inflight=1)
        bp.on_send()
        for _ in range(4):
            assert not bp.can_send()
        assert bp.should_drop

    def test_consecutive_reset_on_success(self):
        bp = _JointBackpressure(max_inflight=1)
        bp.on_send()
        bp.can_send()
        bp.can_send()
        bp.on_done()
        bp.can_send()
        assert not bp.should_drop


# =======================================================================
# F. Fault Injection
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
        assert 300 < drop_count < 700

    def test_burst_pause(self):
        fi = _FaultInjector({
            "mode": "burst_pause",
            "burst_period": 10,
            "burst_pause_ms": 100,
        })
        assert fi.pre_send(0) == "send"
        assert fi.pre_send(10) == "pause"
        assert fi.pre_send(11) == "send"
        assert fi.pre_send(20) == "pause"

    def test_sync_stale_suppress(self):
        fi = _FaultInjector({"mode": "sync_stale"})
        assert fi.suppress_sync is True
        assert fi.pre_send(0) == "send"

    def test_conflict_spike(self):
        fi = _FaultInjector({"mode": "conflict_spike"})
        assert fi.conflict_spike is True
        assert fi.pre_send(0) == "send"


# =======================================================================
# G. Metrics
# =======================================================================
class TestMetrics:

    def test_snapshot_schema(self):
        m = StreamMetrics(target_rate_hz=50, joints=["KNEE_LEFT"])
        snap = m.snapshot()
        required_keys = [
            "target_rate_hz", "actual_rate_hz", "scheduler_drift_ms_p95",
            "chunks_sent", "chunks_confirmed", "chunks_failed",
            "chunks_dropped", "chunks_deferred",
            "waypoints_sent", "waypoints_partial", "waypoints_late",
            "http_status_counts",
            "retries", "queue_fill_max", "max_inflight_per_joint",
            "partial_ratio", "late_ratio",
            "sync_refresh_count", "last_error",
            "fw_wp_accepted", "fw_wp_dropped", "fw_buffer_fill",
        ]
        for key in required_keys:
            assert key in snap, f"Missing key: {key}"

    def test_drift_tracking(self):
        m = StreamMetrics(target_rate_hz=100, joints=["J"])
        for i in range(100):
            m.record_drift(float(i))
        snap = m.snapshot()
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

    def test_partial_increments_correctly(self):
        """P3 regression: inc_partial(n) should increase waypoints_partial."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.inc_partial(3)
        m.inc_partial(1)
        snap = m.snapshot()
        assert snap["waypoints_partial"] == 4

    def test_extra_status_counts_merged(self):
        """P2 regression: conflict spike counts merge with client counts."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.merge_client_metrics({
            "status_counts": {200: 10, 409: 1},
            "retries": 0,
        })
        m.record_http_status(409)
        m.record_http_status(409)
        snap = m.snapshot()
        assert snap["http_status_counts"][409] == 3
        assert snap["http_status_counts"][200] == 10

    def test_queue_fill_max(self):
        m = StreamMetrics(target_rate_hz=50, joints=["A", "B"])
        m.update_queue_fill("A", 1)
        m.update_queue_fill("A", 2)
        m.update_queue_fill("A", 1)
        m.update_queue_fill("B", 1)
        snap = m.snapshot()
        assert snap["queue_fill_max"]["A"] == 2
        assert snap["queue_fill_max"]["B"] == 1


# =======================================================================
# H. Event Log
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
# I. Service API
# =======================================================================
class TestServiceAPI:

    def test_get_events_when_idle(self):
        svc = StreamTestService()
        assert svc.get_events() == []

    def test_get_metrics_when_idle(self):
        svc = StreamTestService()
        assert svc.get_metrics() == {}

    def test_unknown_session_stop(self):
        svc = StreamTestService()
        with patch("stream_test_service.HttpTransport"), \
             patch("stream_test_service.WaypointClient") as mock_cls, \
             patch("stream_test_service.http_requests") as mock_req:
            mock_client = MagicMock()
            mock_client.get_metrics.return_value = {"status_counts": {}, "retries": 0}
            mock_client.enqueue_batch.return_value = "t"
            mock_client.get_batch_status.return_value = None
            mock_cls.return_value = mock_client

            mock_req.get.return_value = _mock_encoder_response([45.0])

            result = svc.start(MINIMAL_CONFIG)
            time.sleep(0.2)

            with pytest.raises(ValueError, match="Unknown session_id"):
                svc.stop("wrong_id")

            svc.stop(result["session_id"])
            time.sleep(0.5)


# =======================================================================
# J. Telemetry Poller
# =======================================================================
class TestTelemetryPoller:

    def test_cached_is_none_before_start(self):
        transport = MagicMock()
        poller = _TelemetryPoller(transport, "KNEE_LEFT", interval_s=0.05)
        assert poller.get_cached() is None

    def test_polls_and_caches(self):
        """Poller fetches from transport and caches the result."""
        transport = MagicMock()
        transport.request_wp_telemetry.return_value = {"wp_accepted": 42}
        poller = _TelemetryPoller(transport, "KNEE_LEFT", interval_s=0.05)
        poller.start()
        time.sleep(0.2)
        poller.stop()
        cached = poller.get_cached()
        assert cached is not None
        assert cached["wp_accepted"] == 42
        assert transport.request_wp_telemetry.call_count >= 1

    def test_none_response_keeps_previous_cache(self):
        """If transport returns None, cached value is not overwritten."""
        transport = MagicMock()
        call_count = [0]

        def side_effect(joint):
            call_count[0] += 1
            if call_count[0] == 1:
                return {"wp_accepted": 10}
            return None

        transport.request_wp_telemetry.side_effect = side_effect
        poller = _TelemetryPoller(transport, "KNEE_LEFT", interval_s=0.05)
        poller.start()
        time.sleep(0.25)
        poller.stop()
        cached = poller.get_cached()
        assert cached is not None
        assert cached["wp_accepted"] == 10

    def test_exception_does_not_crash(self):
        """Transport exception is swallowed — poller keeps running."""
        transport = MagicMock()
        transport.request_wp_telemetry.side_effect = ConnectionError("fail")
        poller = _TelemetryPoller(transport, "KNEE_LEFT", interval_s=0.05)
        poller.start()
        time.sleep(0.15)
        poller.stop()
        assert poller.get_cached() is None
        assert transport.request_wp_telemetry.call_count >= 1

    def test_stop_is_idempotent(self):
        transport = MagicMock()
        transport.request_wp_telemetry.return_value = None
        poller = _TelemetryPoller(transport, "KNEE_LEFT", interval_s=0.05)
        poller.start()
        time.sleep(0.1)
        poller.stop()
        poller.stop()  # second stop should not raise

    def test_get_cached_is_nonblocking(self):
        """get_cached must return in << 1ms regardless of transport state."""
        transport = MagicMock()
        transport.request_wp_telemetry.return_value = {"wp_accepted": 1}
        poller = _TelemetryPoller(transport, "KNEE_LEFT", interval_s=10.0)
        poller.start()
        t0 = time.monotonic()
        poller.get_cached()
        elapsed_ms = (time.monotonic() - t0) * 1000
        poller.stop()
        assert elapsed_ms < 5.0, f"get_cached took {elapsed_ms:.1f}ms"


# =======================================================================
# K. Firmware Telemetry Wrap-Aware Accumulation
# =======================================================================
class TestFwTelemetryWrap:

    def test_first_poll_initializes_only(self):
        """First call to update_fw_telemetry should not accumulate."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.update_fw_telemetry({"wp_accepted": 100, "wp_dropped_full": 5,
                               "wp_dropped_guard": 2, "buffer_fill": 10})
        snap = m.snapshot()
        assert snap["fw_wp_accepted"] == 0
        assert snap["fw_wp_dropped"] == 0
        assert snap["fw_buffer_fill"] == 10

    def test_second_poll_accumulates_delta(self):
        """Second poll computes delta from first."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.update_fw_telemetry({"wp_accepted": 100, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 5})
        m.update_fw_telemetry({"wp_accepted": 250, "wp_dropped_full": 1,
                               "wp_dropped_guard": 2, "buffer_fill": 8})
        snap = m.snapshot()
        assert snap["fw_wp_accepted"] == 150
        assert snap["fw_wp_dropped"] == 3
        assert snap["fw_buffer_fill"] == 8

    def test_multiple_polls_accumulate(self):
        """Multiple polls accumulate correctly."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.update_fw_telemetry({"wp_accepted": 0, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 0})
        m.update_fw_telemetry({"wp_accepted": 100, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 5})
        m.update_fw_telemetry({"wp_accepted": 300, "wp_dropped_full": 0,
                               "wp_dropped_guard": 1, "buffer_fill": 3})
        snap = m.snapshot()
        assert snap["fw_wp_accepted"] == 300
        assert snap["fw_wp_dropped"] == 1

    def test_uint16_wrap_accepted(self):
        """Counter wrapping around 65535 is handled correctly."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.update_fw_telemetry({"wp_accepted": 65500, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 0})
        # 65500 + 100 = 65600 → low 16 bits = 64
        m.update_fw_telemetry({"wp_accepted": 64, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 0})
        snap = m.snapshot()
        assert snap["fw_wp_accepted"] == 100

    def test_uint16_wrap_dropped(self):
        """Dropped counters also handle wrap."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.update_fw_telemetry({"wp_accepted": 0, "wp_dropped_full": 65530,
                               "wp_dropped_guard": 65530, "buffer_fill": 0})
        m.update_fw_telemetry({"wp_accepted": 0, "wp_dropped_full": 4,
                               "wp_dropped_guard": 4, "buffer_fill": 0})
        snap = m.snapshot()
        # (4 - 65530) & 0xFFFF = 10 each
        assert snap["fw_wp_dropped"] == 20

    def test_zero_delta_no_change(self):
        """Same values between polls → no accumulation."""
        m = StreamMetrics(target_rate_hz=50, joints=["J"])
        m.update_fw_telemetry({"wp_accepted": 50, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 3})
        m.update_fw_telemetry({"wp_accepted": 50, "wp_dropped_full": 0,
                               "wp_dropped_guard": 0, "buffer_fill": 3})
        snap = m.snapshot()
        assert snap["fw_wp_accepted"] == 0
        assert snap["fw_wp_dropped"] == 0
