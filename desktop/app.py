"""
Main entry point for MORPH desktop application.
"""

from setup import create_app, socketio
from foxglove import FoxgloveBridge
from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG

# Create Flask application
app = create_app()

# Initialize Foxglove bridge
foxglove_bridge = FoxgloveBridge(socketio_instance=socketio)

# Register socket handlers that need the bridge
from features.my_robot.socket_handlers import register_socket_handlers
register_socket_handlers(socketio, foxglove_bridge)

# Make bridge available to features that need it
app.foxglove_bridge = foxglove_bridge


if __name__ == "__main__":
    """
    Run the Flask application.
    
    The application will be available at http://localhost:8080
    """
    socketio.run(
        app,
        host=FLASK_HOST,
        port=FLASK_PORT,
        debug=FLASK_DEBUG,
        use_reloader=False
    )
