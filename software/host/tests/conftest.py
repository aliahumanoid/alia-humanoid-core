"""
Shared fixtures for the webapp test suite.
"""
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# Ensure host/ is on sys.path so project imports work
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


@pytest.fixture
def mock_can_manager():
    """A MagicMock CanManager with sensible defaults."""
    cm = MagicMock()
    cm.is_connected.return_value = True
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
