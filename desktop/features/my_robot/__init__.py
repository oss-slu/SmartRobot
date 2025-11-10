"""
Robot control feature module.

This module provides robot control functionality including WebSocket
communication, movement control, and status monitoring.
"""

from flask import Blueprint

# Create blueprint for robot feature
robot_bp = Blueprint(
    'robot',
    __name__,
    template_folder='templates',
    static_folder='static',
    static_url_path='/robot/static'
)

# Import routes
from . import routes

