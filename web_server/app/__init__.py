import importlib
import logging
import os
from logging.handlers import RotatingFileHandler

from flask import Flask, request

from .auth import (
    announce_password_once,
    is_auth_configured,
    is_authenticated,
    is_public_request,
    setup_required_response,
    unauthorized_response,
)
from .ball_cycle_manager import ball_cycle_manager
from .config import Config
from .gpio_interface import gpio_interface
from .motor_interface import motor_interface
from .routes import bp as main_bp
from .sockets import sock


# Filter to suppress noisy status polling logs
class StatusEndpointFilter(logging.Filter):
    def filter(self, record):
        # Suppress /api/navigation/status and static file requests
        msg = record.getMessage()
        if "/api/navigation/status" in msg:
            return False
        if "/static/" in msg:
            return False
        return True


def _configure_logging(app):
    # Ensure log directory exists
    log_dir = app.config.get("LOG_DIR")
    if log_dir and not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)

    # Configure werkzeug filter
    log = logging.getLogger("werkzeug")
    if not any(isinstance(flt, StatusEndpointFilter) for flt in log.filters):
        log.addFilter(StatusEndpointFilter())

    # Configure app logging to file
    log_file = app.config.get("LOG_FILE")
    if log_file:
        file_handler = RotatingFileHandler(
            log_file, maxBytes=10 * 1024 * 1024, backupCount=5
        )
        file_handler.setFormatter(
            logging.Formatter(
                "%(asctime)s %(levelname)s: %(message)s [in %(pathname)s:%(lineno)d]"
            )
        )
        file_handler.setLevel(logging.INFO)
        app.logger.addHandler(file_handler)
        app.logger.info("Application logging to file started")


def _load_navigation_controller_class():
    for module_name in ("navigation_coordinated", "web_server.navigation_coordinated"):
        try:
            module = importlib.import_module(module_name)
            return module.CoordinatedNavigationController
        except ImportError:
            continue
    raise ImportError("Could not import CoordinatedNavigationController")


def create_app():
    app = Flask(__name__, template_folder="../templates", static_folder="../static")
    app.config.from_object(Config)

    _configure_logging(app)
    announce_password_once(app.logger)

    # Initialize Sock
    sock.init_app(app)

    # Register Blueprints
    app.register_blueprint(main_bp)

    @app.before_request
    def _require_login():
        if not is_auth_configured():
            if is_public_request(request):
                return None
            return setup_required_response()

        if is_public_request(request):
            return None
        if is_authenticated():
            return None
        return unauthorized_response()

    try:
        NavigationController = _load_navigation_controller_class()
        app.logger.info("Using coordinated navigation with queue system")

        nav_controller = NavigationController(motor_interface.send_command)
        app.logger.info(
            "Navigation initialized at start position: %s", Config.START_POSITION
        )

        if motor_interface.start(nav_controller):
            app.logger.info("Motor control ready")
        else:
            app.logger.warning("Motor control not available")
    except Exception as e:
        app.logger.exception("Failed to initialize navigation: %s", e)

    # Start GPIO interface (roller / conveyor motors + limit switches)
    if gpio_interface.start():
        app.logger.info("GPIO interface ready (roller+conveyor motors and limit switches)")
    else:
        app.logger.warning("GPIO interface not available — running in simulation mode")

    # Wire up the ball-cycle manager with all dependencies.
    # nav_controller may be None if navigation failed to init — the manager
    # handles None references gracefully.
    _nav = motor_interface.nav_controller
    ball_cycle_manager.init(_nav, gpio_interface, motor_interface)
    app.logger.info("Ball cycle manager initialised.")

    return app
