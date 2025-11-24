from flask import Flask
from .config import Config
from .routes import bp as main_bp
from .sockets import sock, init_model
from .motor_interface import motor_interface
import sys
import os
import logging

# Filter to suppress noisy status polling logs
class StatusEndpointFilter(logging.Filter):
    def filter(self, record):
        # Suppress /api/navigation/status and static file requests
        msg = record.getMessage()
        if '/api/navigation/status' in msg:
            return False
        if '/static/' in msg:
            return False
        return True

def create_app():
    app = Flask(__name__,
                template_folder='../templates',
                static_folder='../static')

    # Suppress noisy request logging
    log = logging.getLogger('werkzeug')
    log.addFilter(StatusEndpointFilter())

    # Initialize Sock
    sock.init_app(app)
    
    # Register Blueprints
    app.register_blueprint(main_bp)
    
    # Initialize Navigation Controller
    # We need to import these from the parent directory (WebServer)
    # Since we are in WebServer/app, we can append parent to path if needed, 
    # but usually running from WebServer/web_server.py handles this.
    # However, to be safe and explicit:
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))

    nav_controller = None
    try:
        from navigation_coordinated import CoordinatedNavigationController as NavigationController
        print("Using coordinated navigation with queue system")
    except ImportError:
        print("Warning: Coordinated controller not found, using basic navigation")
        from navigation import NavigationController

    # Initialize Motor Interface
    # We create the controller instance but pass the send_command function to it
    # This is a bit circular: Controller needs send_func, Interface needs Controller (for feedback)
    # Solution: Create Controller with send_func, then start Interface with Controller.
    
    # But NavigationController __init__ takes send_motor_command_func.
    # motor_interface.send_command is that function.
    
    try:
        nav_controller = NavigationController(motor_interface.send_command)
        print(f"Navigation initialized at start position: {Config.START_POSITION}")
        
        # Start motor interface
        if motor_interface.start(nav_controller):
            print("✓ Motor control ready")
        else:
            print("⚠️  Motor control not available")
            
    except Exception as e:
        print(f"Failed to initialize navigation: {e}")

    # Initialize Vosk Model
    init_model()

    return app
