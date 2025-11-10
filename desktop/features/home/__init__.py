"""
Home/Landing page feature module.

This module provides the main landing page and loading screen.
"""

from flask import Blueprint

# Create blueprint for home feature
home_bp = Blueprint(
    'home',
    __name__,
    template_folder='templates',
    static_folder='static',
    static_url_path='/home/static'
)

# Import routes
from . import routes

