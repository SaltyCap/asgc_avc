import os
import signal
import threading
import time

from flask import Blueprint, current_app, jsonify, redirect, render_template, request, url_for

from .auth import (
    check_password,
    is_auth_configured,
    is_authenticated,
    login_user,
    logout_user,
    set_password_from_plaintext,
    sanitize_next_url,
)
from .config import Config
from .motor_interface import motor_interface

# Note: nav_controller is accessed via motor_interface.nav_controller
# This avoids circular imports and ensures we use the active controller instance.

bp = Blueprint("main", __name__)


def _default_next_url():
    return url_for("main.index")


@bp.route("/login", methods=["GET", "POST"])
def login():
    """Simple password login page."""
    requested_next = request.args.get("next", "")
    next_url = sanitize_next_url(requested_next, _default_next_url())

    if not is_auth_configured():
        if request.is_json:
            return jsonify({"error": "password setup required", "setup_required": True}), 503
        return redirect(url_for("main.setup", next=next_url))

    if request.method == "GET":
        if is_authenticated():
            return redirect(next_url)
        return render_template("login.html", error=None, next_url=next_url)

    payload = request.get_json(silent=True) if request.is_json else None
    if payload:
        password = payload.get("password", "")
        next_url = sanitize_next_url(payload.get("next", next_url), _default_next_url())
    else:
        password = request.form.get("password", "")
        next_url = sanitize_next_url(request.form.get("next", next_url), _default_next_url())

    if check_password(password):
        login_user()
        current_app.logger.info("User logged in.")
        if request.is_json:
            return jsonify({"status": "ok", "redirect": next_url})
        return redirect(next_url)

    if request.is_json:
        return jsonify({"error": "invalid credentials"}), 401
    return render_template(
        "login.html",
        error="Invalid password.",
        next_url=next_url,
    ), 401


@bp.route("/logout", methods=["GET", "POST"])
def logout():
    """Clear session and return to login page."""
    logout_user()
    return redirect(url_for("main.login"))


@bp.route("/setup", methods=["GET", "POST"])
def setup():
    """Create or rotate the login password from the web UI."""
    requested_next = request.args.get("next", "")
    next_url = sanitize_next_url(requested_next, _default_next_url())
    configured = is_auth_configured()

    if configured and not is_authenticated():
        if request.is_json:
            return jsonify({"error": "authentication required"}), 401
        return redirect(url_for("main.login", next=next_url))

    if request.method == "GET":
        return render_template(
            "setup.html",
            error=None,
            next_url=next_url,
            configured=configured,
        )

    payload = request.get_json(silent=True) if request.is_json else None
    if payload:
        password = str(payload.get("password", ""))
        confirm = str(payload.get("confirm_password", payload.get("password", "")))
        next_url = sanitize_next_url(payload.get("next", next_url), _default_next_url())
    else:
        password = request.form.get("password", "")
        confirm = request.form.get("confirm_password", "")
        next_url = sanitize_next_url(request.form.get("next", next_url), _default_next_url())

    if not password or len(password) < 8:
        message = "Password must be at least 8 characters."
        if request.is_json:
            return jsonify({"error": message}), 400
        return render_template(
            "setup.html",
            error=message,
            next_url=next_url,
            configured=configured,
        ), 400

    if password != confirm:
        message = "Passwords do not match."
        if request.is_json:
            return jsonify({"error": message}), 400
        return render_template(
            "setup.html",
            error=message,
            next_url=next_url,
            configured=configured,
        ), 400

    if not set_password_from_plaintext(password):
        message = "Failed to save password hash."
        if request.is_json:
            return jsonify({"error": message}), 500
        return render_template(
            "setup.html",
            error=message,
            next_url=next_url,
            configured=configured,
        ), 500

    login_user()
    current_app.logger.info("Web login password configured from setup page.")
    if request.is_json:
        return jsonify({"status": "ok", "redirect": next_url})
    return redirect(next_url)


@bp.route("/")
def index():
    """Serves the main HTML page."""
    return render_template("index.html")


@bp.route("/joystick")
def joystick():
    """Serves the joystick control page."""
    return render_template("joystick.html")


@bp.route("/course")
def course_view():
    """Serves the course visualization page."""
    return render_template("course_view.html")


@bp.route("/api/auth/status")
def auth_status():
    return jsonify(
        {
            "authenticated": bool(is_authenticated()),
            "setup_required": not is_auth_configured(),
        }
    )


@bp.route("/api/navigation/status")
def get_navigation_status():
    """Get current navigation status as JSON."""
    if motor_interface.nav_controller:
        return jsonify(motor_interface.nav_controller.get_position())
    return jsonify({"error": "Navigation not initialized"}), 503


@bp.route("/api/navigation/goto_center", methods=["POST"])
def api_goto_center():
    """Navigate to center of course."""
    if motor_interface.nav_controller:
        motor_interface.nav_controller.go_to_center()
        return jsonify({"status": "navigating", "target": "center"})
    return jsonify({"error": "Navigation not initialized"}), 503


@bp.route("/api/navigation/goto_bucket/<color>", methods=["POST"])
def api_goto_bucket(color):
    """Navigate to specified bucket."""
    if motor_interface.nav_controller:
        bucket_pos = Config.get_bucket_position(color)
        if bucket_pos:
            motor_interface.nav_controller.go_to_bucket(color)
            return jsonify(
                {"status": "navigating", "target": color, "position": bucket_pos}
            )
        return jsonify({"error": f"Unknown bucket color: {color}"}), 400
    return jsonify({"error": "Navigation not initialized"}), 503


@bp.route("/api/navigation/goto_point", methods=["POST"])
def api_goto_point():
    """Navigate to arbitrary point."""
    if not motor_interface.nav_controller:
        return jsonify({"error": "Navigation not initialized"}), 503

    payload = request.get_json(silent=True)
    if not payload:
        return jsonify({"error": "Missing JSON body"}), 400

    try:
        x = float(payload.get("x"))
        y = float(payload.get("y"))
    except (ValueError, TypeError):
        return jsonify({"error": "Invalid coordinates"}), 400

    motor_interface.nav_controller.go_to_point(x, y)
    return jsonify({"status": "navigating", "target": f"POINT ({x:.1f}, {y:.1f})"})


@bp.route("/api/course/info")
def get_course_info():
    """Get course layout information."""
    return jsonify(
        {
            "dimensions": {"width": Config.COURSE_WIDTH, "height": Config.COURSE_HEIGHT},
            "buckets": Config.BUCKETS,
            "center": Config.CENTER,
            "start_position": Config.START_POSITION,
        }
    )


@bp.route("/api/calibrate", methods=["POST"])
def calibrate():
    """Calibrate gyro and reset position."""
    if not motor_interface.nav_controller:
        return jsonify({"error": "Navigation not initialized"}), 503

    try:
        motor_interface.nav_controller.calibrate()
        return jsonify(
            {
                "status": "calibrated",
                "position": Config.START_POSITION,
                "heading": Config.START_HEADING,
            }
        )
    except Exception as exc:
        current_app.logger.exception("Calibration failed: %s", exc)
        return jsonify({"error": "Calibration failed"}), 500


@bp.route("/api/shutdown", methods=["POST"])
def shutdown():
    """Gracefully shutdown the system - stops motors and exits the server."""
    logger = current_app.logger
    logger.warning("Shutdown requested via web interface")

    # Stop the motor interface (this will save logs via C program)
    motor_interface.stop()

    # Schedule server shutdown
    def shutdown_server():
        time.sleep(0.5)  # Give time for response to be sent
        logger.warning("Terminating server process...")
        os.kill(os.getpid(), signal.SIGTERM)

    threading.Thread(target=shutdown_server, daemon=True).start()

    return jsonify(
        {"status": "shutting_down", "message": "System is shutting down..."}
    )
