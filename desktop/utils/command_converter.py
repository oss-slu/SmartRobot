"""
Converts text commands to ROS2 Twist velocity parameters.
"""

from typing import Dict
from config import (
    MOVEMENT_LINEAR_X_FORWARD,
    MOVEMENT_LINEAR_X_BACKWARD,
    MOVEMENT_ANGULAR_Z_LEFT,
    MOVEMENT_ANGULAR_Z_RIGHT
)


class CommandConverter:
    """
    Converts text commands to ROS2 Twist velocity parameters.
    """
    
    @staticmethod
    def command_to_twist(command: str, speed_factor: float = 1.0) -> Dict[str, float]:
        """
        Convert a text command to Twist velocity parameters.
        
        Args:
            command: Command string ("move up", "move down", "move left", "move right", "stop")
            speed_factor: Speed multiplier (0.0 to 1.0) to scale velocities
            
        Returns:
            Dictionary with linear and angular velocity components:
            {
                "linear_x": float,
                "linear_y": float,
                "linear_z": float,
                "angular_x": float,
                "angular_y": float,
                "angular_z": float
            }
        """
        # Normalize speed factor to valid range
        speed_factor = max(0.0, min(1.0, speed_factor))
        
        # Command mapping to Twist velocities
        if command == "move up":
            return {
                "linear_x": MOVEMENT_LINEAR_X_FORWARD * speed_factor,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": 0.0
            }
        elif command == "move down":
            return {
                "linear_x": MOVEMENT_LINEAR_X_BACKWARD * speed_factor,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": 0.0
            }
        elif command == "move left":
            return {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": MOVEMENT_ANGULAR_Z_LEFT * speed_factor
            }
        elif command == "move right":
            return {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": MOVEMENT_ANGULAR_Z_RIGHT * speed_factor
            }
        else:  # "stop" or unknown command
            return {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": 0.0
            }
    
    @staticmethod
    def speed_percentage_to_factor(speed_percentage: int) -> float:
        """
        Convert speed percentage (0-100) to factor (0.0-1.0).
        
        Args:
            speed_percentage: Speed as percentage (0-100)
            
        Returns:
            Speed factor (0.0-1.0)
        """
        return max(0.0, min(100.0, float(speed_percentage))) / 100.0

