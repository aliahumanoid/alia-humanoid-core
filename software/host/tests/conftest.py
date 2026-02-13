"""
Shared fixtures for the waypoint pipeline test suite.
"""
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# Ensure host/ is on sys.path so project imports work
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


@pytest.fixture
def mock_can_manager():
    """A MagicMock CanManager with sensible defaults for waypoint tests."""
    cm = MagicMock()
    cm.is_connected.return_value = True
    cm.last_time_sync_age_ms.return_value = 500.0  # 500ms ago — fresh
    cm.send_multi_dof_waypoint.return_value = {
        "joint": "KNEE_LEFT",
        "joint_id": 1,
        "angles_deg": [10.0, None, None],
        "angle_counts": [1000, 32767, 32767],
        "t_offset_ms": 500,
        "arbitration_id": "0x381",
        "format": "multi_dof",
    }
    cm.send_waypoint_batch.return_value = {
        "total": 3,
        "sent": 3,
        "failed_indices": [],
        "skipped_indices": [],
        "elapsed_ms": 12.5,
        "timing_drift_ms": 0.3,
        "late_count": 0,
        "aborted": False,
        "joint": "KNEE_LEFT",
        "batch_id": "test1234",
    }
    return cm


@pytest.fixture
def mock_serial_manager():
    """A minimal MagicMock SerialManager."""
    return MagicMock()


@pytest.fixture
def app_client(mock_can_manager, mock_serial_manager):
    """Flask test client with mocked managers.

    Creates a fresh Flask app, registers routes with the mock managers,
    and returns (client, mock_can_manager) so tests can inspect calls.
    """
    from flask import Flask
    from routes import register_routes

    app = Flask(__name__)
    app.config["TESTING"] = True
    register_routes(app, mock_serial_manager, mock_can_manager)

    with app.test_client() as client:
        yield client, mock_can_manager
