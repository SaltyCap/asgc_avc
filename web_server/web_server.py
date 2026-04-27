import os
import logging
import signal
import sys

from app import create_app, motor_interface
from app.gpio_interface import gpio_interface

app = create_app()
logger = logging.getLogger(__name__)


def _clean_shutdown(signum=None, frame=None):
    """Called on SIGTERM/SIGINT — stop hardware before the process exits."""
    logger.info("Shutdown signal received (sig=%s) — cleaning up hardware.", signum)
    try:
        gpio_interface.stop()
    except Exception:
        pass
    try:
        motor_interface.stop()
    except Exception:
        pass
    sys.exit(0)


# Register so that `kill <pid>` (SIGTERM from start_all.sh) triggers clean teardown
# instead of letting the OS destroy memory while lgpio threads are still running.
signal.signal(signal.SIGTERM, _clean_shutdown)
signal.signal(signal.SIGINT,  _clean_shutdown)

if __name__ == "__main__":
    host = app.config.get("BIND_HOST", "0.0.0.0")
    port = int(app.config.get("BIND_PORT", 5000))

    cert_path = os.path.join(os.path.dirname(__file__), "cert.pem")
    key_path = os.path.join(os.path.dirname(__file__), "key.pem")

    # Check for SSL
    ssl_context = None
    if os.path.exists(cert_path) and os.path.exists(key_path):
        ssl_context = (cert_path, key_path)
        logger.info(f"Starting server on https://{host}:{port}")
    else:
        logger.info(f"Starting server on http://{host}:{port}")
        logger.warning("SSL certificates not found. Microphone may not work in browsers.")

    logger.info("Connect to this address from your phone's browser.")

    import ssl
    from werkzeug.serving import WSGIRequestHandler

    class QuietRequestHandler(WSGIRequestHandler):
        def log_error(self, format, *args):
            # Suppress specific SSL noise caused by HTTP requests to HTTPS dev server
            message = format % args
            if "ssl.SSLError: [SSL: RECORD_LAYER_FAILURE]" in message:
                return
            super().log_error(format, *args)

    try:
        app.run(
            host=host,
            port=port,
            debug=False,
            use_reloader=False,
            ssl_context=ssl_context,
            request_handler=QuietRequestHandler,
        )
    except (KeyboardInterrupt, SystemExit):
        logger.info("Keyboard interrupt / exit received.")
    finally:
        logger.info("Shutting down...")
        try:
            gpio_interface.stop()
        except Exception:
            pass
        try:
            motor_interface.stop()
        except Exception:
            pass

