"""
Robot control page routes.

This module contains HTTP routes for the robot control page.
"""

from flask import render_template
from . import robot_bp


@robot_bp.route("/my_robot")
def my_robot():
    """
    Robot control panel page.
    
    Returns:
        Robot control panel template
    """
    return render_template("my_robot.html")

