"""
Main entry point for the Joint Controller Host Application.

This module initializes the Flask-SocketIO server, sets up signal handlers
for graceful shutdown, and manages the SerialManager instance for communication
with the joint controller firmware.
"""
import signal
import sys
import logging
from typing import Optional
from flask import Flask
from flask_socketio import SocketIO
from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG
from serial_manager import SerialManager
from routes import register_routes

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__, static_url_path='', static_folder='static')
socketio = SocketIO(app)

# Global serial manager instance
serial_manager_instance: Optional[SerialManager] = None


@socketio.on('connect', namespace='/movement')
def handle_connect():
    """Handle client connection to the /movement WebSocket namespace"""
    logger.info('Client connected to /movement namespace')


def init_app():
    """Initialize the application by setting up SerialManager and routes"""
    global serial_manager_instance
    serial_manager_instance = SerialManager(socketio)
    register_routes(app, serial_manager_instance)


def signal_handler(signum, frame):
    """Handles clean application shutdown on SIGINT/SIGTERM"""
    logger.info("Shutting down application...")
    if serial_manager_instance:
        serial_manager_instance.shutdown()
        logger.info("Serial session logs saved")
    sys.exit(0)


def run_server():
    """Start the Flask-SocketIO server"""
    try:
        logger.info(f"Starting server on {FLASK_HOST}:{FLASK_PORT}")
        socketio.run(
            app,
            host=FLASK_HOST,
            port=FLASK_PORT,
            debug=FLASK_DEBUG,
            use_reloader=False,  # Disabled to avoid conflicts with signal handlers
            allow_unsafe_werkzeug=True,  # Required for Flask-SocketIO compatibility
        )
    except Exception as e:
        logger.error(f"Error starting server: {e}")
        sys.exit(1)


if __name__ == '__main__':
    # Register signal handlers for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    init_app()
    run_server()
