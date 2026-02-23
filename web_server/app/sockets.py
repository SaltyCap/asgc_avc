import json
import threading

import vosk
from flask import request
from flask_sock import Sock

from .auth import is_auth_configured, is_origin_allowed, is_websocket_authenticated
from .config import Config
from .motor_interface import motor_interface
from .voice_command import VoiceCommandProcessor

sock = Sock()

# Global model variable (loaded in create_app or lazily)
model = None
_model_lock = threading.Lock()

# Global set of connected motor control WebSocket clients
motor_clients = set()
motor_clients_lock = threading.Lock()


def _parse_boolean_flag(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0

    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"invalid boolean flag: {value!r}")


def _reject_socket(ws, reason):
    try:
        ws.send(json.dumps({"type": "error", "message": reason}))
    except Exception:
        pass
    ws.close()


def _authorize_socket(ws):
    if not is_origin_allowed(request):
        _reject_socket(ws, "origin not allowed")
        return False
    if not is_auth_configured():
        _reject_socket(ws, "password setup required")
        return False
    if not is_websocket_authenticated(request):
        _reject_socket(ws, "authentication required")
        return False
    return True

def init_model():
    global model
    with _model_lock:
        if model is not None:
            return True

        print("Loading Vosk model...")
        try:
            # Set log level to reduce Vosk verbosity
            vosk.SetLogLevel(-1)
            model = vosk.Model(Config.MODEL_PATH)
            print("Model loaded successfully.")
            return True
        except Exception as e:
            print(f"Error loading model: {e}")
            print(f"Please make sure the 'model' folder is at {Config.MODEL_PATH}")
            model = None
            return False


@sock.route("/audio")
def audio_socket(ws):
    """Handles the WebSocket connection for audio streaming."""
    if not _authorize_socket(ws):
        return

    print("Client connected.")

    if not init_model():
        print("Vosk model not loaded. Voice control unavailable.")
        _reject_socket(ws, "voice model unavailable")
        return

    # Suppress Vosk warnings about runtime graphs
    vosk.SetLogLevel(-1)

    # Try to create recognizer with vocabulary constraint
    try:
        recognizer = vosk.KaldiRecognizer(model, 16000, Config.VOCABULARY)
    except Exception:
        print("Model doesn't support vocabulary constraint, using full recognition")
        recognizer = vosk.KaldiRecognizer(model, 16000)

    recognizer.SetMaxAlternatives(0)
    recognizer.SetWords(False)
    recording = False
    last_partial = ""
    
    # Initialize voice processor
    voice_processor = VoiceCommandProcessor(motor_interface.nav_controller)

    try:
        while True:
            message = ws.receive()
            if message is None:
                break

            if isinstance(message, str):
                if message == "start":
                    recording = True
                    # Reset recognizer state instead of creating new one
                    recognizer.Reset()
                    last_partial = ""
                    # Ensure processor has latest controller reference
                    voice_processor.nav_controller = motor_interface.nav_controller
                    print("\n--- Recording Started ---")
                elif message == "stop":
                    recording = False
                    print("\n--- Recording Stopped ---")
                    final_result = json.loads(recognizer.FinalResult())
                    if final_result.get("text"):
                        final_text = final_result["text"]
                        print(f"Final: {final_text}\n")
                        ws.send(json.dumps({"type": "final", "text": final_text}))
                        voice_processor.process_command(final_text)

            elif isinstance(message, bytes) and recording:
                # Pass bytes directly - no numpy conversion needed
                if recognizer.AcceptWaveform(message):
                    result = json.loads(recognizer.Result())
                    if result.get("text"):
                        final_text = result["text"]
                        print(f"\nFinal: {final_text}")
                        ws.send(json.dumps({"type": "final", "text": final_text}))
                        voice_processor.process_command(final_text)
                else:
                    # Only send partial if it changed (reduces WebSocket traffic)
                    partial_result = json.loads(recognizer.PartialResult())
                    partial_text = partial_result.get("partial", "")
                    if partial_text and partial_text != last_partial:
                        last_partial = partial_text
                        print(f"Partial: {partial_text}", end="\r")
                        ws.send(json.dumps({"type": "partial", "text": partial_text}))

    except Exception as e:
        print(f"An error occurred or client disconnected: {e}")
    finally:
        print("Client disconnected.")


def _broadcast_motor_message(message):
    dead_clients = []
    with motor_clients_lock:
        clients = list(motor_clients)

    for client in clients:
        try:
            client.send(message)
        except Exception as e:
            print(f"Failed to broadcast to client: {e}")
            dead_clients.append(client)

    if dead_clients:
        with motor_clients_lock:
            for client in dead_clients:
                motor_clients.discard(client)


@sock.route("/motor")
def motor_socket(ws):
    """Handles WebSocket connection for motor control."""
    if not _authorize_socket(ws):
        return

    print("Motor control client connected.")

    # Add client to the set of connected clients
    with motor_clients_lock:
        motor_clients.add(ws)

    # Control mode:
    # - 'joystick' = direct PWM commands
    # - 'voice' = voice/navigation commands
    # - 'ml' = C-side ML inference mode
    # Default to 'voice' mode (index.html and course_view.html)
    control_mode = "voice"

    # Initialize helper
    voice_processor = VoiceCommandProcessor(motor_interface.nav_controller)

    # Tell newly connected client which mode this socket starts in.
    ws.send(json.dumps({"type": "mode_set", "mode": control_mode}))

    try:
        while True:
            message = ws.receive()
            if message is None:
                break

            if isinstance(message, str):
                try:
                    data = json.loads(message)
                    msg_type = data.get("type")

                    # Handle mode selection from client
                    if msg_type == "set_mode":
                        mode = data.get("mode", "voice")
                        if mode in ("joystick", "voice", "ml"):
                            previous_mode = control_mode
                            nav_state = ""
                            if motor_interface.nav_controller is not None:
                                nav_state = str(getattr(motor_interface.nav_controller, "state", ""))
                            c_is_ml = nav_state.upper() == "ML"
                            if mode == "ml":
                                if not motor_interface.send_command("ml_mode"):
                                    ws.send(
                                        json.dumps(
                                            {
                                                "type": "error",
                                                "message": "Motor controller offline; cannot enter ML mode",
                                            }
                                        )
                                    )
                                    continue
                            elif previous_mode == "ml" or c_is_ml:
                                # Leave C-side ML state and return to regular NAV_IDLE.
                                if not motor_interface.send_command("regular_mode"):
                                    ws.send(
                                        json.dumps(
                                            {
                                                "type": "error",
                                                "message": "Motor controller offline; cannot exit ML mode",
                                            }
                                        )
                                    )
                                    continue
                                # Fallback for older motor binaries that don't yet implement
                                # 'regular_mode': neutral pulse exits NAV_ML into NAV_IDLE.
                                motor_interface.send_command("pulse 1500000 1500000")
                            control_mode = mode
                            print(f"Control mode set to: {control_mode}")
                            ws.send(json.dumps({"type": "mode_set", "mode": control_mode}))
                        continue

                    # Explicit ML mode trigger (for clients that don't use set_mode)
                    if msg_type == "ml_mode":
                        if motor_interface.send_command("ml_mode"):
                            control_mode = "ml"
                            ws.send(json.dumps({"type": "mode_set", "mode": control_mode}))
                        else:
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Motor controller offline; cannot enter ML mode",
                                    }
                                )
                            )
                        continue

                    # Handle speed setting (works in both modes)
                    if msg_type == "set_speed":
                        speed_percent = data.get("speed_percent", 100)
                        speed_percent = max(0, min(100, int(speed_percent)))
                        if motor_interface.nav_controller:
                            motor_interface.nav_controller.set_speed_percent(speed_percent / 100.0)
                        print(f"Speed set to: {speed_percent}%")
                        ws.send(
                            json.dumps(
                                {"type": "speed_set", "speed_percent": speed_percent}
                            )
                        )
                        continue

                    # Handle PWM settings (works in both modes)
                    if msg_type == "set_pwm":
                        min_pwm_percent = data.get("min_pwm_percent", 45)
                        max_pwm_percent = data.get("max_pwm_percent", 80)

                        # Validate range 0-100
                        min_pwm_percent = max(0, min(100, int(min_pwm_percent)))
                        max_pwm_percent = max(0, min(100, int(max_pwm_percent)))

                        # Send command to C program
                        motor_interface.send_command(f"setpwm {min_pwm_percent} {max_pwm_percent}")
                        print(f"PWM settings: Min={min_pwm_percent}%, Max={max_pwm_percent}%")

                        # Broadcast to all connected motor control clients
                        pwm_message = json.dumps(
                            {
                                "type": "pwm_set",
                                "min_pwm_percent": min_pwm_percent,
                                "max_pwm_percent": max_pwm_percent,
                            }
                        )
                        _broadcast_motor_message(pwm_message)
                        continue

                    # Future sensor pipeline inputs for ML pick-and-place workflow.
                    if msg_type == "ml_ball_acquired":
                        raw_acquired = data.get("acquired", data.get("value"))
                        if raw_acquired is None:
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Missing 'acquired' field for ml_ball_acquired",
                                    }
                                )
                            )
                            continue

                        try:
                            acquired = _parse_boolean_flag(raw_acquired)
                        except ValueError:
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "acquired must be true/false or 1/0",
                                    }
                                )
                            )
                            continue

                        if motor_interface.send_command(f"ml_ball_acquired {1 if acquired else 0}"):
                            ws.send(json.dumps({"type": "ml_ball_acquired_set", "acquired": acquired}))
                        else:
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Motor controller offline; cannot set ml_ball_acquired",
                                    }
                                )
                            )
                        continue

                    if msg_type == "ml_ball_color":
                        color = str(data.get("color", "")).strip().lower()
                        if color == "clear":
                            color = "none"
                        if color not in ("red", "yellow", "blue", "green", "none"):
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "color must be red, yellow, blue, green, or none",
                                    }
                                )
                            )
                            continue

                        if motor_interface.send_command(f"ml_ball_color {color}"):
                            ws.send(json.dumps({"type": "ml_ball_color_set", "color": color}))
                        else:
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Motor controller offline; cannot set ml_ball_color",
                                    }
                                )
                            )
                        continue

                    # Handle commands based on control mode.
                    # Use if/elif here to remain compatible with Python 3.9.
                    if control_mode == "joystick":
                        # Joystick mode: only allow direct PWM control.
                        if msg_type == "joystick":
                            # Use exact pulse width values from client (in nanoseconds).
                            left_ns = data.get("leftNs", 1500000)
                            right_ns = data.get("rightNs", 1500000)

                            # Clamp to valid pulse width range for safety.
                            left_ns = max(1000000, min(2000000, int(left_ns)))
                            right_ns = max(1000000, min(2000000, int(right_ns)))

                            motor_interface.send_command(f"pulse {left_ns} {right_ns}")

                        elif msg_type == "stop":
                            motor_interface.send_command("stop")

                        else:
                            print(f"Command '{msg_type}' not allowed in joystick mode")
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Only PWM control allowed in joystick mode",
                                    }
                                )
                            )
                            continue

                    elif control_mode == "voice":
                        # Voice mode: only allow voice commands and navigation.
                        if msg_type == "voice":
                            command = data.get("command", "").lower()
                            # Ensure processor has latest controller reference.
                            voice_processor.nav_controller = motor_interface.nav_controller
                            voice_processor.process_command(command)

                        elif msg_type == "stop":
                            motor_interface.send_command("stop")

                        elif msg_type == "joystick":
                            print("Joystick commands not allowed in voice mode")
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Joystick control not allowed in voice mode",
                                    }
                                )
                            )
                            continue

                        else:
                            print(f"Unknown command type: {msg_type}")

                    elif control_mode == "ml":
                        if msg_type == "stop":
                            motor_interface.send_command("stop")
                        else:
                            print(f"Command '{msg_type}' not allowed in ml mode")
                            ws.send(
                                json.dumps(
                                    {
                                        "type": "error",
                                        "message": "Only stop, set_pwm, set_speed, ml_ball_acquired, and ml_ball_color are allowed in ML mode",
                                    }
                                )
                            )
                            continue

                    ws.send(json.dumps({"type": "ack", "received": data}))

                except json.JSONDecodeError:
                    print(f"Invalid JSON received: {message}")
                except Exception as e:
                    print(f"Error processing motor command: {e}")

    except Exception as e:
        print(f"Motor control client error or disconnected: {e}")
    finally:
        # Remove client from the set of connected clients
        with motor_clients_lock:
            motor_clients.discard(ws)
        print("Motor control client disconnected.")
