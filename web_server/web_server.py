import os

from app import create_app, motor_interface

app = create_app()

if __name__ == "__main__":
    host = app.config.get("BIND_HOST", "0.0.0.0")
    port = int(app.config.get("BIND_PORT", 5000))

    cert_path = os.path.join(os.path.dirname(__file__), "cert.pem")
    key_path = os.path.join(os.path.dirname(__file__), "key.pem")

    # Check for SSL
    ssl_context = None
    if os.path.exists(cert_path) and os.path.exists(key_path):
        ssl_context = (cert_path, key_path)
        print(f"Starting server on https://{host}:{port}")
    else:
        print(f"Starting server on http://{host}:{port}")
        print("\nWarning: SSL certificates not found. Microphone may not work in browsers.")

    print("Connect to this address from your phone's browser.")

    try:
        app.run(
            host=host,
            port=port,
            debug=False,
            use_reloader=False,
            ssl_context=ssl_context,
        )
    finally:
        print("\nShutting down...")
        motor_interface.stop()
