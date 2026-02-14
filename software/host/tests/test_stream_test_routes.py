"""
Integration tests for /stream_test/* routes using Flask test client.

These tests exercise the full request path:
    HTTP request → routes.py → StreamTestService mock

Run with: python3 -m pytest tests/test_stream_test_routes.py -v
"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# Skip if Flask/numpy not available
flask = pytest.importorskip("flask")
pytest.importorskip("numpy")


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------
VALID_CONFIG = {
    "joint": "KNEE_LEFT",
    "active_dof": 0,
    "n_dof": 1,
    "min_deg": 20.0,
    "max_deg": 80.0,
    "start_at": "min",
    "frequency_hz": 0.5,
    "rate_hz": 50,
    "duration_s": 60,
    "horizon_ms": 250,
    "buffer_depth_sim": 2,
    "max_inflight_per_joint": 1,
    "fault_profile": {"mode": "none"},
    "safe_limits": {"min": -2.0, "max": 112.0},
}


def _post_json(client, url, payload):
    return client.post(url, data=json.dumps(payload), content_type="application/json")


# =======================================================================
# A. POST /stream_test/start
# =======================================================================
class TestStreamTestStart:

    def test_start_success(self, stream_client):
        client, svc = stream_client
        resp = _post_json(client, "/stream_test/start", VALID_CONFIG)
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["status"] == "success"
        assert data["session_id"] == "st_test_001"
        assert data["state"] == "RUNNING"
        svc.start.assert_called_once_with(VALID_CONFIG)

    def test_start_empty_body(self, stream_client):
        """Empty body should be passed to service (which validates)."""
        client, svc = stream_client
        svc.start.side_effect = ValueError("'joint' must be a non-empty string")
        resp = _post_json(client, "/stream_test/start", {})
        assert resp.status_code == 400
        assert "joint" in resp.get_json()["message"].lower()

    def test_start_validation_error_returns_400(self, stream_client):
        client, svc = stream_client
        svc.start.side_effect = ValueError("rate_hz must be 50 or 100")
        resp = _post_json(client, "/stream_test/start", {"rate_hz": 75})
        assert resp.status_code == 400

    def test_start_double_session_returns_409(self, stream_client):
        client, svc = stream_client
        svc.start.side_effect = RuntimeError("Session already active")
        resp = _post_json(client, "/stream_test/start", VALID_CONFIG)
        assert resp.status_code == 409
        assert "already active" in resp.get_json()["message"].lower()

    def test_start_unexpected_error_returns_500(self, stream_client):
        client, svc = stream_client
        svc.start.side_effect = Exception("unexpected")
        resp = _post_json(client, "/stream_test/start", VALID_CONFIG)
        assert resp.status_code == 500


# =======================================================================
# B. POST /stream_test/stop
# =======================================================================
class TestStreamTestStop:

    def test_stop_success(self, stream_client):
        client, svc = stream_client
        resp = _post_json(client, "/stream_test/stop", {
            "session_id": "st_test_001",
            "reason": "operator_stop",
        })
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["state"] == "STOPPED"
        svc.stop.assert_called_once_with("st_test_001", reason="operator_stop")

    def test_stop_unknown_session_returns_400(self, stream_client):
        client, svc = stream_client
        svc.stop.side_effect = ValueError("Unknown session_id")
        resp = _post_json(client, "/stream_test/stop", {
            "session_id": "wrong_id",
        })
        assert resp.status_code == 400

    def test_stop_default_reason(self, stream_client):
        """Missing reason should default to operator_stop."""
        client, svc = stream_client
        resp = _post_json(client, "/stream_test/stop", {
            "session_id": "st_test_001",
        })
        assert resp.status_code == 200
        svc.stop.assert_called_once_with("st_test_001", reason="operator_stop")


# =======================================================================
# C. GET /stream_test/status
# =======================================================================
class TestStreamTestStatus:

    def test_status_idle(self, stream_client):
        client, svc = stream_client
        resp = client.get("/stream_test/status")
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["status"] == "success"
        assert data["state"] == "IDLE"
        assert data["session"] is None

    def test_status_with_session_id(self, stream_client):
        client, svc = stream_client
        svc.get_status.return_value = {
            "state": "RUNNING",
            "session": {
                "session_id": "st_test_001",
                "state": "RUNNING",
                "uptime_s": 10.5,
            },
        }
        resp = client.get("/stream_test/status?session_id=st_test_001")
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["session"]["state"] == "RUNNING"
        svc.get_status.assert_called_with("st_test_001")


# =======================================================================
# D. GET /stream_test/metrics
# =======================================================================
class TestStreamTestMetrics:

    def test_metrics_empty_when_idle(self, stream_client):
        client, svc = stream_client
        resp = client.get("/stream_test/metrics")
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["status"] == "success"
        assert data["metrics"] == {}

    def test_metrics_with_data(self, stream_client):
        client, svc = stream_client
        svc.get_metrics.return_value = {
            "target_rate_hz": 50,
            "actual_rate_hz": 49.8,
            "chunks_sent": 100,
        }
        resp = client.get("/stream_test/metrics")
        data = resp.get_json()
        assert data["metrics"]["target_rate_hz"] == 50
        assert data["metrics"]["chunks_sent"] == 100


# =======================================================================
# E. GET /stream_test/events
# =======================================================================
class TestStreamTestEvents:

    def test_events_empty(self, stream_client):
        client, svc = stream_client
        resp = client.get("/stream_test/events")
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["status"] == "success"
        assert data["events"] == []

    def test_events_with_after_seq(self, stream_client):
        client, svc = stream_client
        svc.get_events.return_value = [
            {"seq": 5, "level": "WARN", "event": "backpressure_defer"},
        ]
        resp = client.get("/stream_test/events?after_seq=4&session_id=st_test_001")
        assert resp.status_code == 200
        data = resp.get_json()
        assert len(data["events"]) == 1
        assert data["events"][0]["seq"] == 5
        svc.get_events.assert_called_with("st_test_001", 4)


# =======================================================================
# F. Service unavailable (stream_test_service=None)
# =======================================================================
class TestServiceUnavailable:

    def test_start_503_when_no_service(self, app_client):
        """Routes registered without stream_test_service should return 503."""
        client, _ = app_client
        resp = _post_json(client, "/stream_test/start", VALID_CONFIG)
        assert resp.status_code == 503

    def test_stop_503_when_no_service(self, app_client):
        client, _ = app_client
        resp = _post_json(client, "/stream_test/stop", {"session_id": "x"})
        assert resp.status_code == 503

    def test_status_503_when_no_service(self, app_client):
        client, _ = app_client
        resp = client.get("/stream_test/status")
        assert resp.status_code == 503

    def test_metrics_503_when_no_service(self, app_client):
        client, _ = app_client
        resp = client.get("/stream_test/metrics")
        assert resp.status_code == 503

    def test_events_503_when_no_service(self, app_client):
        client, _ = app_client
        resp = client.get("/stream_test/events")
        assert resp.status_code == 503
