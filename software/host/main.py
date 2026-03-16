"""
Main entry point for the Joint Controller Host Application.

This module initializes the Flask-SocketIO server, sets up signal handlers
for graceful shutdown, and manages the SerialManager instance for communication
with the joint controller firmware.

Boot sequence (after server starts):
1. Load saved CAN configuration (if exists)
2. Auto-connect to CAN bus
3. Discover joints on serial ports
"""
import signal
import sys
import logging
import threading
import time
from typing import Optional
from flask import Flask
from flask_socketio import SocketIO
from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG
from serial_manager import SerialManager
from can_manager import CanManager
from routes import register_routes

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__, static_url_path='', static_folder='static')
socketio = SocketIO(app)

# Global manager instances
serial_manager_instance: Optional[SerialManager] = None
can_manager_instance: Optional[CanManager] = None


@socketio.on('connect', namespace='/movement')
def handle_connect():
    """Handle client connection to the /movement WebSocket namespace.
    
    Sends current system state to the newly connected client so it can
    restore UI state without re-running discovery.
    """
    from flask_socketio import emit
    
    logger.info('Client connected to /movement namespace')
    
    # Send current state to the newly connected client
    state = {
        'step': 'client_connected',
        'message': 'Connected to host'
    }
    
    # Include CAN connection status
    if can_manager_instance:
        state['can_connected'] = can_manager_instance.is_connected()
        if can_manager_instance.is_connected():
            config = can_manager_instance._current_config
            if config:
                state['can_interface'] = f"{config.get('interface')} @ {config.get('channel')}"
    
    # Include serial port mappings (from previous discovery)
    if serial_manager_instance:
        mappings = serial_manager_instance.get_joint_to_port_mapping()
        if mappings:
            state['serial_mappings'] = mappings
            state['message'] = f"Connected - {len(mappings)} joint(s) mapped"
    
    emit('client_state', state)


def init_app():
    """Initialize the application by setting up SerialManager and routes"""
    global serial_manager_instance, can_manager_instance
    serial_manager_instance = SerialManager(socketio)
    try:
        shared_logger = serial_manager_instance.get_session_logger() if serial_manager_instance else None
        can_manager_instance = CanManager(socketio, comm_logger=shared_logger)
    except RuntimeError as exc:
        logger.warning("CAN features unavailable: %s", exc)
        can_manager_instance = None

    register_routes(app, serial_manager_instance, can_manager_instance)


def auto_boot_sequence():
    """
    Automatic boot sequence executed after server starts.
    
    1. Load saved CAN configuration
    2. Connect to CAN bus
    3. Discover joints on serial ports
    """
    global can_manager_instance, serial_manager_instance
    
    # Wait a bit for server to be fully ready
    time.sleep(1.0)
    
    logger.info("=== Starting automatic boot sequence ===")
    
    # Step 1: Check for saved CAN config
    if can_manager_instance is None:
        logger.warning("Boot sequence: CAN manager not available, skipping CAN auto-connect")
        return
    
    saved_config = can_manager_instance.load_config()
    if saved_config is None:
        logger.info("Boot sequence: No saved CAN config found. Manual connection required.")
        # Emit event to UI
        if socketio:
            socketio.emit('boot_status', {
                'step': 'no_config',
                'message': 'No saved CAN configuration. Please connect manually.'
            }, namespace='/movement')
        return
    
    # Step 2: Auto-connect to CAN
    try:
        logger.info(f"Boot sequence: Connecting to CAN ({saved_config})")
        can_manager_instance.connect(saved_config)
        logger.info("Boot sequence: CAN connected successfully")
        
        # Emit status to UI
        if socketio:
            socketio.emit('boot_status', {
                'step': 'can_connected',
                'message': f"CAN connected: {saved_config.get('interface')} @ {saved_config.get('channel')}"
            }, namespace='/movement')
    except Exception as e:
        logger.error(f"Boot sequence: Failed to connect CAN: {e}")
        if socketio:
            socketio.emit('boot_status', {
                'step': 'can_error',
                'message': f"CAN connection failed: {e}"
            }, namespace='/movement')
        return
    
    time.sleep(0.3)  # Brief pause after CAN connection

    # Step 4: Discover joints via serial
    try:
        time.sleep(0.2)
        logger.info("Boot sequence: Starting joint discovery...")
        
        # Send identify request via CAN
        can_manager_instance.send_identify_request()
        time.sleep(0.3)  # Let controllers start broadcasting
        
        # Scan serial ports
        if serial_manager_instance:
            discovered = serial_manager_instance.discover_joints(timeout_seconds=4.0)
            count = len(discovered)
            logger.info(f"Boot sequence: Discovered {count} joint(s)")
            
            if socketio:
                socketio.emit('boot_status', {
                    'step': 'discovery_complete',
                    'message': f"Discovered {count} joint(s)",
                    'discovered': discovered,
                    'mappings': serial_manager_instance.get_joint_to_port_mapping()
                }, namespace='/movement')
    except Exception as e:
        logger.warning(f"Boot sequence: Joint discovery failed: {e}")
    
    logger.info("=== Boot sequence complete ===")


def signal_handler(signum, frame):
    """Handles clean application shutdown on SIGINT/SIGTERM"""
    logger.info("Shutting down application...")
    if serial_manager_instance:
        serial_manager_instance.shutdown()
        logger.info("Serial session logs saved")
    if can_manager_instance:
        can_manager_instance.disconnect()
        logger.info("CAN interface closed")
    sys.exit(0)


def run_server():
    """Start the Flask-SocketIO server"""
    try:
        logger.info(f"Starting server on {FLASK_HOST}:{FLASK_PORT}")
        
        # Schedule auto-boot sequence to run shortly after server starts
        boot_thread = threading.Thread(target=auto_boot_sequence, daemon=True)
        boot_thread.start()
        
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
