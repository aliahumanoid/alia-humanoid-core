"""
Integration tests for waypoint batch routes using Flask test client.

These tests exercise the full request path:
    HTTP request → routes.py → _waypoint_preflight() → CanManager mock

NOTE: These tests require the full project venv (Flask, numpy, etc.).
Run with: python3 -m pytest tests/test_waypoint_routes.py -v
from inside the host/ directory with the venv activated.
"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# Skip entire module if Flask/numpy are not available (e.g., system Python)
flask = pytest.importorskip("flask")
pytest.importorskip("numpy")


# -----------------------------------------------------------------------
# /can/waypoint_batch route
# -----------------------------------------------------------------------
class TestWaypointBatchRoute:
    """Tests for POST /can/waypoint_batch."""

    def _post_batch(self, client, payload):
        return client.post(
            "/can/waypoint_batch",
            data=json.dumps(payload),
            content_type="application/json",
        )

    def test_missing_joint_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {
            "waypoints": [{"angles_deg": [10.0], "t_offset_ms": 100}]
        })
        assert resp.status_code == 400
        data = resp.get_json()
        assert "joint" in data["message"].lower()

    def test_missing_waypoints_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {"joint": "KNEE_LEFT"})
        assert resp.status_code == 400

    def test_empty_waypoints_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [],
        })
        assert resp.status_code == 400

    def test_can_disconnected_returns_503(self, app_client):
        client, cm = app_client
        cm.is_connected.return_value = False
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [{"angles_deg": [10.0], "t_offset_ms": 100}],
        })
        assert resp.status_code == 503

    def test_time_sync_never_done_returns_400(self, app_client):
        client, cm = app_client
        cm.last_time_sync_age_ms.return_value = None
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [{"angles_deg": [10.0], "t_offset_ms": 100}],
        })
        assert resp.status_code == 400
        assert "time sync" in resp.get_json()["message"].lower()

    def test_time_sync_stale_returns_400(self, app_client):
        client, cm = app_client
        cm.last_time_sync_age_ms.return_value = 5000.0  # 5 seconds — stale
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [{"angles_deg": [10.0], "t_offset_ms": 100}],
        })
        assert resp.status_code == 400
        assert "stale" in resp.get_json()["message"].lower()

    def test_angle_out_of_range_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [{"angles_deg": [200.0], "t_offset_ms": 100}],
        })
        assert resp.status_code == 400
        data = resp.get_json()
        assert "validation_errors" in data

    def test_velocity_violation_returns_400(self, app_client):
        client, _ = app_client
        # 100° in 10ms = 10000°/s
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [0.0], "t_offset_ms": 100},
                {"angles_deg": [100.0], "t_offset_ms": 110},
            ],
        })
        assert resp.status_code == 400
        data = resp.get_json()
        assert "validation_errors" in data
        assert any("velocity" in e["message"].lower()
                    for e in data["validation_errors"])

    def test_monotonicity_violation_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [10.0], "t_offset_ms": 500},
                {"angles_deg": [20.0], "t_offset_ms": 400},
            ],
        })
        assert resp.status_code == 400
        data = resp.get_json()
        assert "validation_errors" in data

    def test_unknown_joint_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {
            "joint": "NONEXISTENT",
            "waypoints": [{"angles_deg": [10.0], "t_offset_ms": 100}],
        })
        assert resp.status_code == 400

    def test_valid_batch_returns_success(self, app_client):
        client, cm = app_client
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [10.0], "t_offset_ms": 100},
                {"angles_deg": [20.0], "t_offset_ms": 200},
                {"angles_deg": [30.0], "t_offset_ms": 300},
            ],
        })
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["status"] == "success"
        assert "result" in data
        # CanManager.send_waypoint_batch should have been called
        cm.send_waypoint_batch.assert_called_once()

    def test_partial_batch_returns_partial(self, app_client):
        client, cm = app_client
        cm.send_waypoint_batch.return_value = {
            "total": 3, "sent": 2, "failed_indices": [1],
            "skipped_indices": [], "elapsed_ms": 10.0,
            "timing_drift_ms": 0.5, "late_count": 0,
            "aborted": False, "joint": "KNEE_LEFT", "batch_id": "t123",
        }
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [10.0], "t_offset_ms": 100},
                {"angles_deg": [20.0], "t_offset_ms": 200},
                {"angles_deg": [30.0], "t_offset_ms": 300},
            ],
        })
        assert resp.status_code == 200
        assert resp.get_json()["status"] == "partial"

    def test_concurrent_batch_returns_409(self, app_client):
        client, cm = app_client
        cm.send_waypoint_batch.side_effect = ValueError(
            "Waypoint batch already in progress for KNEE_LEFT"
        )
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [10.0], "t_offset_ms": 100},
            ],
        })
        assert resp.status_code == 409

    def test_batch_id_in_response(self, app_client):
        client, _ = app_client
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [10.0], "t_offset_ms": 100},
                {"angles_deg": [20.0], "t_offset_ms": 200},
            ],
        })
        data = resp.get_json()
        assert "batch_id" in data.get("result", {}) or "batch_id" in data.get("message", "")

    def test_deduplication_applied(self, app_client):
        """Backend should deduplicate zero-step waypoints before sending."""
        client, cm = app_client
        # Two waypoints that quantize to same int16 (4500)
        resp = self._post_batch(client, {
            "joint": "KNEE_LEFT",
            "waypoints": [
                {"angles_deg": [45.001], "t_offset_ms": 100},
                {"angles_deg": [45.002], "t_offset_ms": 200},
                {"angles_deg": [50.0], "t_offset_ms": 300},
            ],
        })
        assert resp.status_code == 200
        # The batch sent to CanManager should have 2 entries (deduped)
        call_args = cm.send_waypoint_batch.call_args
        waypoints_sent = call_args[0][1]  # second positional arg
        assert len(waypoints_sent) == 2


# -----------------------------------------------------------------------
# /can/waypoint route (single waypoint)
# -----------------------------------------------------------------------
class TestSingleWaypointRoute:
    """Tests for POST /can/waypoint."""

    def _post_wp(self, client, payload):
        return client.post(
            "/can/waypoint",
            data=json.dumps(payload),
            content_type="application/json",
        )

    def test_missing_joint_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_wp(client, {
            "angles_deg": [10.0],
            "t_offset_ms": 500,
        })
        assert resp.status_code == 400

    def test_angle_out_of_range_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_wp(client, {
            "joint": "KNEE_LEFT",
            "angles_deg": [200.0],
            "t_offset_ms": 500,
        })
        assert resp.status_code == 400
        assert "out of range" in resp.get_json()["message"]

    def test_valid_waypoint_returns_success(self, app_client):
        client, cm = app_client
        resp = self._post_wp(client, {
            "joint": "KNEE_LEFT",
            "angles_deg": [50.0],
            "t_offset_ms": 500,
        })
        assert resp.status_code == 200
        assert resp.get_json()["status"] == "success"
        cm.send_multi_dof_waypoint.assert_called_once()

    def test_t_offset_out_of_range_returns_400(self, app_client):
        client, _ = app_client
        resp = self._post_wp(client, {
            "joint": "KNEE_LEFT",
            "angles_deg": [50.0],
            "t_offset_ms": 70000,
        })
        assert resp.status_code == 400
