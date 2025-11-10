"""
This module holds all the setup options for the SMART Robot app 
kind of like a control panel where you can change settings without messing with the main
"""

# Flask Configuration
SECRET_KEY = 'your-secret-key'  #BRIGHT FUTURE: Move to environment variable in production
FLASK_HOST = "0.0.0.0"
FLASK_PORT = 8080
FLASK_DEBUG = False

# Robot Connection Configuration
DEFAULT_ROBOT_IP = "10.178.43.127"  #IT MIGHT CHANGE: Common "Not Connected" issue if this is wrong
ROBOT_PORT = 8765                   # Foxglove WebSocket server port

# Foxglove Bridge Configuration
COMMAND_CHANNEL_ID = 4              # Channel ID for publishing commands, just random number
TOPICS_TO_SUBSCRIBE = [             # ROS2 topics to subscribe to
    "/test_talker",
    "/diff_drive_base/odom"
]

# Movement Command Configuration
# Linear velocities (m/s)
MOVEMENT_LINEAR_X_FORWARD = 0.5
MOVEMENT_LINEAR_X_BACKWARD = -0.5

# Angular velocities (rad/s)
MOVEMENT_ANGULAR_Z_LEFT = 0.5
MOVEMENT_ANGULAR_Z_RIGHT = -0.5

# ROS2 Topic Names
TOPIC_CMD_VEL = "/diff_drive_base/cmd_vel"
FRAME_ID = "base_link"

