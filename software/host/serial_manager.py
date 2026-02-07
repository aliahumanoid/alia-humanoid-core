"""
Serial port manager for joint controller application.

This module provides the SerialManager class which:
- Manages multiple serial port connections
- Maps joints to serial ports dynamically
- Coordinates SerialHandler instances for each port
- Provides unified status and data access across all handlers
"""
from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional, Tuple

import serial.tools.list_ports

from serial_handler import SerialHandler
from serial_logger import SerialLogger


class SerialManager:
    """
    Coordinates temporary joint-to-port assignments.
    
    Manages the lifecycle of SerialHandler instances and maintains
    mappings between joints and their associated serial ports.
    Thread-safe for concurrent access.
    """

    def __init__(self, socketio) -> None:
        self.socketio = socketio
        self._handlers_by_port: Dict[str, SerialHandler] = {}
        self._listener_threads: Dict[str, threading.Thread] = {}
        self._joint_to_port: Dict[str, str] = {}
        self._lock = threading.Lock()
        self.session_logger = SerialLogger("logs/serial_communication.log")

    def list_available_ports(self) -> List[str]:
        """
        Returns updated list of available serial ports.
        
        Returns:
            List[str]: List of serial port device paths
        """
        return [port.device for port in serial.tools.list_ports.comports()]

    def get_joint_to_port_mapping(self) -> Dict[str, str]:
        """
        Returns current joint-to-port mapping.
        
        Returns:
            Dict[str, str]: Dictionary mapping joint names to serial port paths
        """
        with self._lock:
            return dict(self._joint_to_port)

    def get_port_for_joint(self, joint: str) -> Optional[str]:
        """
        Returns the serial port assigned to a specific joint.
        
        Args:
            joint: Joint name (e.g., 'KNEE_LEFT', 'ANKLE_RIGHT')
            
        Returns:
            Optional[str]: Serial port path or None if not assigned
        """
        with self._lock:
            return self._joint_to_port.get(joint)

    def get_handler_for_joint(self, joint: str) -> Optional[SerialHandler]:
        """
        Returns the SerialHandler instance for a specific joint.
        
        Args:
            joint: Joint name (e.g., 'KNEE_LEFT', 'ANKLE_RIGHT')
            
        Returns:
            Optional[SerialHandler]: SerialHandler instance or None if not assigned
        """
        port = self.get_port_for_joint(joint)
        if not port:
            return None
        with self._lock:
            return self._handlers_by_port.get(port)

    def assign_port_to_joint(self, joint: str, port: Optional[str]) -> Dict[str, Any]:
        """
        Assigns or removes the association between a joint and a serial port.
        
        If the port is already assigned to another joint, that joint will be
        unassigned. Creates a new SerialHandler if the port is not yet managed.
        
        Args:
            joint: Joint name to assign
            port: Serial port path or None to unassign
            
        Returns:
            Dict[str, Any]: Assignment result with keys:
                - joint: Joint name
                - port: Assigned port (or None)
                - reassigned_joint: Previous joint using this port (or None)
        """
        reassigned_joint: Optional[str] = None
        with self._lock:
            if not port:
                self._joint_to_port.pop(joint, None)
                return {
                    "joint": joint,
                    "port": None,
                    "reassigned_joint": None,
                }

            existing_joint = None
            for candidate_joint, assigned_port in self._joint_to_port.items():
                if candidate_joint != joint and assigned_port == port:
                    existing_joint = candidate_joint
                    break

            handler = self._handlers_by_port.get(port)
            if handler is None:
                handler = SerialHandler(self.socketio, port=port, serial_logger=self.session_logger)
                listener = threading.Thread(
                    target=handler.listen_for_messages,
                    daemon=True,
                    name=f"SerialListener-{port}"
                )
                listener.start()
                self._handlers_by_port[port] = handler
                self._listener_threads[port] = listener

            if existing_joint:
                self._joint_to_port.pop(existing_joint, None)
                reassigned_joint = existing_joint

            self._joint_to_port[joint] = port

        return {
            "joint": joint,
            "port": port,
            "reassigned_joint": reassigned_joint,
        }

    def pop_status_message(self) -> Optional[Tuple[str, str]]:
        """
        Extracts a status message from any available handler.
        
        Returns:
            Optional[Tuple[str, str]]: Tuple of (port, message) or None if no messages
        """
        with self._lock:
            handlers_items = list(self._handlers_by_port.items())
        for port, handler in handlers_items:
            if handler.status_message:
                message = handler.status_message.pop(0)
                return port, message
        return None

    def get_combined_joint_status(self) -> Dict[str, Any]:
        """
        Merges joint status from all handlers.
        
        Returns:
            Dict[str, Any]: Combined status dictionary for all joints
        """
        combined: Dict[str, Any] = {}
        with self._lock:
            handlers = list(self._handlers_by_port.values())
        for handler in handlers:
            for joint, status in handler.joint_status.items():
                combined[joint] = status
        return combined

    def get_handler_snapshot(self) -> Dict[str, SerialHandler]:
        """
        Returns a snapshot of all active handlers.
        
        Returns:
            Dict[str, SerialHandler]: Dictionary mapping port paths to handlers
        """
        with self._lock:
            return dict(self._handlers_by_port)

    def shutdown(self) -> None:
        """
        Attempts to close open serial loggers before shutdown.
        
        Gracefully closes all SerialHandler instances and their associated
        serial loggers to ensure proper cleanup.
        """
        if self.session_logger:
            self.session_logger.close_session()

    def get_session_logger(self) -> SerialLogger:
        """
        Return the shared session logger instance.
        """
        return self.session_logger

    def discover_joints(self, timeout_seconds: float = 4.0) -> Dict[str, str]:
        """
        Scan all available serial ports and discover which joints are connected.
        
        This method opens each serial port temporarily, listens for EVT:JOINT
        messages, and builds a mapping of joint names to serial ports.
        
        Note: Call CAN identify_request before this to trigger controllers
        to emit their identification.
        
        Args:
            timeout_seconds: How long to listen for identification messages
            
        Returns:
            Dict[str, str]: Mapping of joint_name (uppercase) -> serial_port
        """
        import serial
        import time
        import logging
        
        logger = logging.getLogger(__name__)
        discovered: Dict[str, str] = {}
        ports = self.list_available_ports()
        
        logger.info(f"Starting joint discovery on {len(ports)} ports...")
        
        # First, check handlers already running — they have detected_joint_name
        # from EVT:JOINT events. This handles ports that can't be reopened.
        with self._lock:
            for port, handler in self._handlers_by_port.items():
                if handler and handler.detected_joint_name:
                    joint_upper = handler.detected_joint_name.upper()
                    discovered[joint_upper] = port
                    # Also ensure port mapping is correct
                    self._joint_to_port[joint_upper] = port
                    logger.info(f"Discovery: {joint_upper} already active on {port}")
        
        if discovered:
            logger.info(f"Found {len(discovered)} joint(s) from active handlers")
            return discovered
        
        # If no active handlers with detected joints, try opening ports directly
        connections = []
        for port in ports:
            try:
                # Skip ports that might be problematic (Bluetooth, etc.)
                if "Bluetooth" in port or "debug" in port.lower():
                    continue
                    
                conn = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=0.1,  # Short read timeout for polling
                )
                connections.append((port, conn))
                logger.debug(f"Opened {port} for discovery")
            except (serial.SerialException, OSError) as e:
                logger.debug(f"Could not open {port}: {e}")
                continue
        
        if not connections:
            logger.warning("No serial ports available for discovery")
            return discovered
        
        # Listen for EVT:JOINT messages
        start_time = time.time()
        buffers = {port: "" for port, _ in connections}
        
        while time.time() - start_time < timeout_seconds:
            for port, conn in connections:
                try:
                    if conn.in_waiting > 0:
                        data = conn.read(conn.in_waiting).decode('utf-8', errors='ignore')
                        buffers[port] += data
                        
                        # Process complete lines
                        while '\n' in buffers[port]:
                            line, buffers[port] = buffers[port].split('\n', 1)
                            line = line.strip()
                            
                            # Look for EVT:JOINT message
                            if line.startswith("EVT:JOINT "):
                                parts = line[10:].split()  # Skip "EVT:JOINT "
                                if len(parts) >= 2:
                                    joint_id = parts[0]
                                    joint_name = parts[1].upper()
                                    
                                    if joint_name not in discovered:
                                        discovered[joint_name] = port
                                        logger.info(f"Discovered {joint_name} (ID={joint_id}) on {port}")
                except Exception as e:
                    logger.debug(f"Error reading {port}: {e}")
            
            time.sleep(0.05)  # Small delay to avoid busy-waiting
        
        # Close all temporary connections
        for port, conn in connections:
            try:
                conn.close()
            except Exception:
                pass
        
        logger.info(f"Discovery complete: found {len(discovered)} joint(s)")
        
        # Update internal mapping with discovered joints
        with self._lock:
            for joint_name, port in discovered.items():
                self._joint_to_port[joint_name] = port
        
        return discovered
