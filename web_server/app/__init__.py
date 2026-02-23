import importlib
import logging

from flask import Flask, request

from .auth import (
    announce_password_once,
    is_auth_configured,
    is_authenticated,
    is_public_request,
    setup_required_response,
    unauthorized_response,
)
from .config import Config
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


def _configure_logging():
    log = logging.getLogger("werkzeug")
    if not any(isinstance(flt, StatusEndpointFilter) for flt in log.filters):
        log.addFilter(StatusEndpointFilter())


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

    _configure_logging()
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

    return app
