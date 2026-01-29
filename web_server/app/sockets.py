import json
import vosk
from flask_sock import Sock
from .config import Config
from .motor_interface import motor_interface
from .voice_command import VoiceCommandProcessor

sock = Sock()

# Global model variable (loaded in create_app or lazily)
model = None

# Global set of connected motor control WebSocket clients
motor_clients = set()

def init_model():
    global model
    print("Loading Vosk model...")
    try:
        # Set log level to reduce Vosk verbosity
        vosk.SetLogLevel(-1)
        model = vosk.Model(Config.MODEL_PATH)
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Error loading model: {e}")
        print(f"Please make sure the 'model' folder is at {Config.MODEL_PATH}")
        model = None

@sock.route('/audio')
def audio_socket(ws):
    """Handles the WebSocket connection for audio streaming."""
    print("Client connected.")

    if not model:
        print("Vosk model not loaded. Voice control unavailable.")
        ws.close()
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

            if isinstance(message, str):
                if message == 'start':
                    recording = True
                    # Reset recognizer state instead of creating new one
                    recognizer.Reset()
                    last_partial = ""
                    # Ensure processor has latest controller reference
                    voice_processor.nav_controller = motor_interface.nav_controller
                    print("\n--- Recording Started ---")
                elif message == 'stop':
                    recording = False
                    print("\n--- Recording Stopped ---")
                    final_result = json.loads(recognizer.FinalResult())
                    if final_result.get('text'):
                        final_text = final_result['text']
                        print(f"Final: {final_text}\n")
                        ws.send(json.dumps({'type': 'final', 'text': final_text}))
                        voice_processor.process_command(final_text)

            elif isinstance(message, bytes) and recording:
                # Pass bytes directly - no numpy conversion needed
                if recognizer.AcceptWaveform(message):
                    result = json.loads(recognizer.Result())
                    if result.get('text'):
                        final_text = result['text']
                        print(f"\nFinal: {final_text}")
                        ws.send(json.dumps({'type': 'final', 'text': final_text}))
                        voice_processor.process_command(final_text)
                else:
                    # Only send partial if it changed (reduces WebSocket traffic)
                    partial_result = json.loads(recognizer.PartialResult())
                    partial_text = partial_result.get('partial', '')
                    if partial_text and partial_text != last_partial:
                        last_partial = partial_text
                        print(f"Partial: {partial_text}", end='\r')
                        ws.send(json.dumps({'type': 'partial', 'text': partial_text}))

    except Exception as e:
        print(f"An error occurred or client disconnected: {e}")
    finally:
        print("Client disconnected.")

@sock.route('/motor')
def motor_socket(ws):
    """Handles WebSocket connection for motor control."""
    print("Motor control client connected.")

    # Add client to the set of connected clients
    motor_clients.add(ws)

    # Control mode: 'joystick' = direct PWM, 'voice' = voice/navigation commands
    # Default to 'voice' mode (index.html and course_view.html)
    control_mode = 'voice'
    
    # Initialize helper
    voice_processor = VoiceCommandProcessor(motor_interface.nav_controller)

    try:
        while True:
            message = ws.receive()

            if isinstance(message, str):
                try:
                    data = json.loads(message)
                    msg_type = data.get('type')

                    # Handle mode selection from client
                    if msg_type == 'set_mode':
                        mode = data.get('mode', 'voice')
                        if mode in ('joystick', 'voice'):
                            control_mode = mode
                            print(f"Control mode set to: {control_mode}")
                            ws.send(json.dumps({'type': 'mode_set', 'mode': control_mode}))
                        continue

                    # Handle speed setting (works in both modes)
                    if msg_type == 'set_speed':
                        speed_percent = data.get('speed_percent', 100)
                        speed_percent = max(0, min(100, int(speed_percent)))
                        if motor_interface.nav_controller:
                            motor_interface.nav_controller.set_speed_multiplier(speed_percent / 100.0)
                        print(f"Speed multiplier set to: {speed_percent}%")
                        ws.send(json.dumps({'type': 'speed_set', 'speed_percent': speed_percent}))
                        continue

                    # Handle PWM settings (works in both modes)
                    if msg_type == 'set_pwm':
                        min_pwm = data.get('min_pwm', 45)
                        max_pwm = data.get('max_pwm', 80)
                        min_pwm = max(20, min(100, int(min_pwm)))
                        max_pwm = max(20, min(100, int(max_pwm)))
                        # Send command to C program
                        motor_interface.send_command(f"setpwm {min_pwm} {max_pwm}")
                        print(f"PWM settings: Min={min_pwm}%, Max={max_pwm}%")

                        # Broadcast to all connected motor control clients
                        pwm_message = json.dumps({'type': 'pwm_set', 'min_pwm': min_pwm, 'max_pwm': max_pwm})
                        for client in list(motor_clients):  # Use list() to avoid modification during iteration
                            try:
                                client.send(pwm_message)
                            except Exception as e:
                                print(f"Failed to broadcast PWM to client: {e}")
                                motor_clients.discard(client)
                        continue

                    # Handle commands based on control mode
                    match control_mode:
                        case 'joystick':
                            # Joystick mode: only allow direct PWM control
                            if msg_type == 'joystick':
                                # Use exact pulse width values from client (in nanoseconds)
                                left_ns = data.get('leftNs', 1500000)
                                right_ns = data.get('rightNs', 1500000)

                                # Clamp to valid pulse width range for safety
                                left_ns = max(1000000, min(2000000, int(left_ns)))
                                right_ns = max(1000000, min(2000000, int(right_ns)))

                                motor_interface.send_command(f"pulse {left_ns} {right_ns}")

                            elif msg_type == 'stop':
                                motor_interface.send_command("stop")

                            else:
                                print(f"Command '{msg_type}' not allowed in joystick mode")
                                ws.send(json.dumps({'type': 'error', 'message': 'Only PWM control allowed in joystick mode'}))
                                continue

                        case 'voice':
                            # Voice mode: only allow voice commands and navigation
                            if msg_type == 'voice':
                                command = data.get('command', '').lower()
                                # Ensure processor has latest controller reference
                                voice_processor.nav_controller = motor_interface.nav_controller
                                voice_processor.process_command(command)

                            elif msg_type == 'stop':
                                motor_interface.send_command("stop")

                            elif msg_type == 'joystick':
                                print("Joystick commands not allowed in voice mode")
                                ws.send(json.dumps({'type': 'error', 'message': 'Joystick control not allowed in voice mode'}))
                                continue

                            else:
                                print(f"Unknown command type: {msg_type}")

                    ws.send(json.dumps({'type': 'ack', 'received': data}))

                except json.JSONDecodeError:
                    print(f"Invalid JSON received: {message}")
                except Exception as e:
                    print(f"Error processing motor command: {e}")

    except Exception as e:
        print(f"Motor control client error or disconnected: {e}")
    finally:
        # Remove client from the set of connected clients
        motor_clients.discard(ws)
        print("Motor control client disconnected.")
