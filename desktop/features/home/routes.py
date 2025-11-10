"""
Home page routes.

This module contains HTTP routes for the home/landing page.
"""

from flask import render_template
from . import home_bp


@home_bp.route("/")
def index():
    """
    Landing/loading page.
    
    Returns:
        Loading page template
    """
    try:
        return render_template("loading.html")
    except Exception:
        return "Loading..."


@home_bp.route("/home")
def home():
    """
    Home page.
    
    Returns:
        Home page template
    """
    try:
        return render_template("index.html")
    except Exception:
        return "Home page not found."

