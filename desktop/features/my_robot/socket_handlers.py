"""
Robot control SocketIO event handlers.

This module handles WebSocket events for robot control functionality.
"""

from flask_socketio import SocketIO
from typing import Dict, Any
import asyncio

from foxglove import FoxgloveBridge
from utils.command_converter import CommandConverter


def register_socket_handlers(socketio: SocketIO, foxglove_bridge: FoxgloveBridge) -> None:
    """
    Register SocketIO event handlers for robot control.
    
    Args:
        socketio: Flask-SocketIO instance
        foxglove_bridge: FoxgloveBridge instance
    """
    
    @socketio.on("connect_robot")
    def handle_connect_robot(data: Dict[str, Any]) -> None:
        """
        Handle robot connection request from frontend.
        
        Args:
            data: Dictionary containing connection parameters:
                - ip: Robot IP address (optional, defaults to config value)
        """
        from config import DEFAULT_ROBOT_IP
        
        ip = data.get("ip", DEFAULT_ROBOT_IP)
        socketio.emit("log", f"UI requested connection to {ip}")
        
        # Update bridge IP and trigger connection
        foxglove_bridge.pi_ip = ip
        foxglove_bridge.socketio = socketio  # Ensure socketio is set
        
        # Start connection in asyncio event loop
        loop = asyncio.get_event_loop()
        loop.create_task(foxglove_bridge.connect_to_pi_server())
    
    @socketio.on("disconnect_robot")
    def handle_disconnect_robot() -> None:
        """
        Handle robot disconnection request from frontend.
        """
        socketio.emit("log", "UI requested disconnection")
        foxglove_bridge.running = False
        foxglove_bridge.connected = False
        socketio.emit("robot_disconnected")
    
    @socketio.on("check_robot_status")
    def handle_check_robot_status() -> None:
        """
        Handle robot status check request from frontend.
        
        Responds with current connection status and robot IP.
        """
        socketio.emit("robot_status", {
            "connected": foxglove_bridge.connected,
            "ip": foxglove_bridge.pi_ip
        })
    
    @socketio.on("send_command")
    def handle_send_command(data: Dict[str, Any]) -> None:
        """
        Handle movement command from frontend.
        
        Converts text command to Twist message and sends to robot.
        
        Args:
            data: Dictionary containing command parameters:
                - command: Command string ("move up", "move down", etc.)
                - speed: Speed percentage (0-100, optional)
        """
        if not foxglove_bridge.connected:
            socketio.emit("log", "⚠️ Not connected to robot!")
            return
        
        command = data.get("command", "stop")
        speed_percentage = data.get("speed", 100)  # Default to 100%
        
        # Convert command to Twist velocities
        speed_factor = CommandConverter.speed_percentage_to_factor(speed_percentage)
        twist = CommandConverter.command_to_twist(command, speed_factor)
        
        socketio.emit("log", f"UI command: {command} (speed: {speed_percentage}%)")
        
        # Send Twist command to robot
        if foxglove_bridge.connection_loop:
            asyncio.run_coroutine_threadsafe(
                foxglove_bridge.send_twist_command(**twist),
                foxglove_bridge.connection_loop
            )
        else:
            socketio.emit("log", "⚠️ No connection loop available!")

