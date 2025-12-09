import json
import vosk
from flask_sock import Sock
from .config import Config
from .motor_interface import motor_interface

sock = Sock()

# Global model variable (loaded in create_app or lazily)
model = None

# Vocabulary for voice commands (includes sound-alike words)
VOCABULARY = '["red", "read", "bread", "wed", "blue", "blew", "green", "yellow", "yell", "center", "middle", "centre", "stop", "clear", "forward", "back", "backward", "reverse", "left", "right", "motor", "one", "two", "start", "reset", "position", "[unk]"]'

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
    # Fall back to standard recognizer if model doesn't support it
    try:
        recognizer = vosk.KaldiRecognizer(model, 16000, VOCABULARY)
    except Exception:
        print("Model doesn't support vocabulary constraint, using full recognition")
        recognizer = vosk.KaldiRecognizer(model, 16000)

    recognizer.SetMaxAlternatives(0)
    recognizer.SetWords(False)
    recording = False
    last_partial = ""

    try:
        while True:
            message = ws.receive()

            if isinstance(message, str):
                if message == 'start':
                    recording = True
                    # Reset recognizer state instead of creating new one
                    recognizer.Reset()
                    last_partial = ""
                    print("\n--- Recording Started ---")
                elif message == 'stop':
                    recording = False
                    print("\n--- Recording Stopped ---")
                    final_result = json.loads(recognizer.FinalResult())
                    if final_result.get('text'):
                        final_text = final_result['text']
                        print(f"Final: {final_text}\n")
                        ws.send(json.dumps({'type': 'final', 'text': final_text}))
                        process_voice_command(final_text)

            elif isinstance(message, bytes) and recording:
                # Pass bytes directly - no numpy conversion needed
                if recognizer.AcceptWaveform(message):
                    result = json.loads(recognizer.Result())
                    if result.get('text'):
                        final_text = result['text']
                        print(f"\nFinal: {final_text}")
                        ws.send(json.dumps({'type': 'final', 'text': final_text}))
                        process_voice_command(final_text)
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

    # Control mode: 'joystick' = direct PWM, 'voice' = voice/navigation commands
    # Default to 'voice' mode (index.html and course_view.html)
    control_mode = 'voice'

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
                        nav_controller.set_speed_multiplier(speed_percent / 100.0)
                        print(f"Speed multiplier set to: {speed_percent}%")
                        ws.send(json.dumps({'type': 'speed_set', 'speed_percent': speed_percent}))
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
                                process_voice_command(command)

                            elif msg_type == 'stop':
                                motor_interface.send_command("stop")

                            elif msg_type == 'select_motor':
                                motor = data.get('motor', 1)
                                motor_interface.send_command(str(motor))

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
        print("Motor control client disconnected.")

def process_voice_command(command):
    """Process voice commands and convert to motor commands.

    Parses each word in the command string and queues valid commands.
    Example: "red blue green" will queue 3 separate commands.
    """
    nav_controller = motor_interface.nav_controller
    command = command.strip().lower()
    words = command.split()

    print(f"[VOICE COMMAND] '{command}'")

    # Valid commands that can be queued (colors + center)
    # Includes sound-alike aliases for better recognition
    queue_commands = {
        'red': 'red',
        'read': 'red',
        'bread': 'red',
        'wed': 'red',
        'blue': 'blue',
        'blew': 'blue',
        'green': 'green',
        'yellow': 'yellow',
        'yell': 'yellow',
        'center': 'center',
        'middle': 'center',
        'centre': 'center',
    }

    # Immediate action commands (not queued, executed right away)
    immediate_commands = {'clear', 'stop', 'reset'}

    queued_count = 0

    if not nav_controller:
        print("[VOICE] ERROR: Navigation controller not initialized!")
        return

    for word in words:
        print(f"[VOICE] Processing word: '{word}'")

        # Check for immediate commands first
        if word == 'clear':
            nav_controller.clear_queue()
            print("[VOICE] Queue cleared")
            return  # Stop processing after clear

        elif word == 'stop':
            nav_controller.clear_queue()
            print("[VOICE] Queue stopped and cleared")
            return  # Stop processing after stop

        elif word == 'start':
            nav_controller.start_queue()
            print("[VOICE] Queue started")
            return  # Stop processing after start

        # Check if word is a valid queue command
        elif word in queue_commands:
            target = queue_commands[word]
            print(f"[VOICE] Found valid command: '{word}' -> '{target}'", flush=True)
            try:
                if target == 'center':
                    print(f"[VOICE] Calling go_to_center()...", flush=True)
                    nav_controller.go_to_center()
                    print(f"[VOICE] go_to_center() returned", flush=True)
                else:
                    print(f"[VOICE] Calling go_to_bucket('{target}')...", flush=True)
                    nav_controller.go_to_bucket(target)
                    print(f"[VOICE] go_to_bucket() returned", flush=True)
                queued_count += 1
                print(f"[VOICE] Successfully queued: {target}", flush=True)
            except Exception as e:
                import traceback
                print(f"[VOICE] ERROR queueing {target}: {e}", flush=True)
                traceback.print_exc()
        else:
            print(f"[VOICE] Skipping unknown word: '{word}'")

    if queued_count > 0:
        print(f"[VOICE] Total queued: {queued_count} commands")
    elif not any(word in immediate_commands for word in words):
        print(f"[VOICE] No valid commands found in: '{command}'")
