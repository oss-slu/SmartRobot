"""
This file is basically the starting point for the Flask app 
It sets things up, loads the extensions, and hooks up all the routes and features.
"""

from flask import Flask, render_template
from flask_socketio import SocketIO
from config import SECRET_KEY

# Initialize SocketIO (will be initialized with app later)
socketio = SocketIO(cors_allowed_origins="*")


def create_app():
    """
    Create and configure Flask application.
    
    This function:
    1. Creates Flask app
    2. Initializes SocketIO
    3. Registers all features/routes
    4. Sets up error handlers
    
    Returns:
        Configured Flask application instance
    """
    # 1. Create Flask app with shared template and static folders
    import os
    base_dir = os.path.dirname(os.path.abspath(__file__))
    template_dir = os.path.join(base_dir, 'shared', 'templates')
    static_dir = os.path.join(base_dir, 'shared', 'static')
    
    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
    app.config['SECRET_KEY'] = SECRET_KEY
    
    # 2. Initialize SocketIO with the app
    socketio.init_app(app)
    
    # 3. Register all features/routes
    register_features(app)
    
    # 4. Set up error handlers
    register_error_handlers(app)
    
    return app


def register_features(app: Flask) -> None:
    """
    Register all feature blueprints with the app.
    
    Args:
        app: Flask application instance
    """
    from features.home import home_bp
    from features.my_robot import robot_bp
    
    # Register blueprints
    app.register_blueprint(home_bp)
    app.register_blueprint(robot_bp)
    
    # TODO: Register more features as they are created
    # from features.learning import learning_bp
    # from features.shop import shop_bp
    # from features.about import about_bp
    # from features.contact import contact_bp
    # app.register_blueprint(learning_bp)
    # app.register_blueprint(shop_bp)
    # app.register_blueprint(about_bp)
    # app.register_blueprint(contact_bp)


def register_error_handlers(app: Flask) -> None:
    """
    Register error handlers for the app.
    
    Args:
        app: Flask application instance
    """
    @app.errorhandler(404)
    def not_found(error):
        return render_template('errors/404.html'), 404
    
    @app.errorhandler(500)
    def internal_error(error):
        return render_template('errors/500.html'), 500


# Export socketio so other modules can use it
__all__ = ['create_app', 'socketio']

