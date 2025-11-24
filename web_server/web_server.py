import os
import sys
from app import create_app, motor_interface

# Ensure we can import from current directory
sys.path.append(os.path.dirname(__file__))

app = create_app()

if __name__ == '__main__':
    print("Starting server on https://0.0.0.0:5000")
    print("Connect to this address from your phone's browser.")
    
    # Check for SSL
    ssl_context = None
    if os.path.exists('cert.pem') and os.path.exists('key.pem'):
        ssl_context = ('cert.pem', 'key.pem')
    else:
        print("\n⚠️  Warning: SSL certificates not found! Microphone may not work.")

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False,
                ssl_context=ssl_context)
    finally:
        print("\nShutting down...")
        motor_interface.stop()
