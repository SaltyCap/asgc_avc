import json
import threading
import logging
import time

import vosk
from flask import request
from flask_sock import Sock

from .auth import is_auth_configured, is_origin_allowed, is_websocket_authenticated
from .ball_cycle_manager import (
    ball_cycle_manager,
    _SCOOP_QUS_UP,
    _SCOOP_QUS_DOWN,
    _SCOOP_DOWN_STEPS,
    _SCOOP_DOWN_STEP_DELAY,
    _SCOOP_UP_STEPS,
    _SCOOP_UP_STEP_DELAY,
)
from .config import Config
from .gpio_interface import gpio_interface
from .motor_interface import motor_interface
from .voice_command import VoiceCommandProcessor

sock = Sock()
logger = logging.getLogger(__name__)

# Global model variable (loaded in create_app or lazily)
model = None
_model_lock = threading.Lock()

# Global set of connected motor control WebSocket clients
motor_clients = set()
motor_clients_lock = threading.Lock()

# Last-known PWM state — broadcast to new clients on connect so they sync immediately
_current_pwm = {"min_percent": 25, "max_percent": 25}
_current_pwm_lock = threading.Lock()


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

        logger.info("Loading Vosk model...")
        try:
            # Set log level to reduce Vosk verbosity
            vosk.SetLogLevel(-1)
            model = vosk.Model(Config.MODEL_PATH)
            logger.info("Model loaded successfully.")
            return True
        except Exception as e:
            logger.error(f"Error loading model: {e}")
            logger.error(f"Please make sure the 'model' folder is at {Config.MODEL_PATH}")
            model = None
            return False


@sock.route("/audio")
def audio_socket(ws):
    """Handles the WebSocket connection for audio streaming."""
    if not _authorize_socket(ws):
        return

    logger.info("Audio client connected.")

    if not init_model():
        logger.error("Vosk model not loaded. Voice control unavailable.")
        _reject_socket(ws, "voice model unavailable")
        return

    # Suppress Vosk warnings about runtime graphs
    vosk.SetLogLevel(-1)

    # Try to create recognizer with vocabulary constraint
    try:
        recognizer = vosk.KaldiRecognizer(model, 16000, Config.VOCABULARY)
    except Exception:
        logger.warning("Model doesn't support vocabulary constraint, using full recognition")
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
                    logger.info("--- Recording Started ---")
                elif message == "stop":
                    recording = False
                    logger.info("--- Recording Stopped ---")
                    final_result = json.loads(recognizer.FinalResult())
                    if final_result.get("text"):
                        final_text = final_result["text"]
                        logger.info(f"Final: {final_text}")
                        ws.send(json.dumps({"type": "final", "text": final_text}))
                        voice_processor.process_command(final_text)

            elif isinstance(message, bytes) and recording:
                # Pass bytes directly - no numpy conversion needed
                if recognizer.AcceptWaveform(message):
                    result = json.loads(recognizer.Result())
                    if result.get("text"):
                        final_text = result["text"]
                        logger.info(f"Final: {final_text}")
                        ws.send(json.dumps({"type": "final", "text": final_text}))
                        voice_processor.process_command(final_text)
                else:
                    # Only send partial if it changed (reduces WebSocket traffic)
                    partial_result = json.loads(recognizer.PartialResult())
                    partial_text = partial_result.get("partial", "")
                    if partial_text and partial_text != last_partial:
                        last_partial = partial_text
                        logger.debug(f"Partial: {partial_text}")
                        ws.send(json.dumps({"type": "partial", "text": partial_text}))

    except Exception as e:
        # 1000 = normal closure, 1001 = going away, 1005 = no status (browser tab close)
        e_str = str(e)
        if "1000" in e_str or "1001" in e_str or "1005" in e_str:
            logger.info(f"Audio client disconnected (normal closure): {e}")
        else:
            logger.error(f"An error occurred or audio client disconnected: {e}")
    finally:
        logger.info("Audio client disconnected.")


def _broadcast_motor_message(message):
    dead_clients = []
    with motor_clients_lock:
        clients = list(motor_clients)

    for client in clients:
        try:
            client.send(message)
        except Exception as e:
            logger.error(f"Failed to broadcast to client: {e}")
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

    logger.info("Motor control client connected.")

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

    # Push current PWM state so new clients sync immediately without overwriting it.
    with _current_pwm_lock:
        ws.send(json.dumps({
            "type": "pwm_set",
            "min_pwm_percent": _current_pwm["min_percent"],
            "max_pwm_percent": _current_pwm["max_percent"],
        }))

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
                            logger.info(f"Control mode set to: {control_mode}")
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
                        logger.info(f"Speed set to: {speed_percent}%")
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

                        # Update stored PWM state
                        with _current_pwm_lock:
                            _current_pwm["min_percent"] = min_pwm_percent
                            _current_pwm["max_percent"] = max_pwm_percent

                        # Send command to C program
                        motor_interface.send_command(f"setpwm {min_pwm_percent} {max_pwm_percent}")
                        logger.info(f"PWM settings: Min={min_pwm_percent}%, Max={max_pwm_percent}%")

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

                        elif msg_type == "uart_servo":
                            position = data.get("position", "")
                            low  = int(data.get("low",  0)) & 0x7F
                            high = int(data.get("high", 0)) & 0x7F

                            if position == "down":
                                # Slew DOWN slowly in a background thread so it doesn't slam
                                def _slew_down():
                                    step_size = (_SCOOP_QUS_DOWN - _SCOOP_QUS_UP) / _SCOOP_DOWN_STEPS
                                    for i in range(1, _SCOOP_DOWN_STEPS + 1):
                                        target_qus = int(_SCOOP_QUS_UP + step_size * i)
                                        s_low  = target_qus & 0x7F
                                        s_high = (target_qus >> 7) & 0x7F
                                        motor_interface.send_command(
                                            f"uart_servo 0x{s_low:02X} 0x{s_high:02X}"
                                        )
                                        time.sleep(_SCOOP_DOWN_STEP_DELAY)
                                threading.Thread(target=_slew_down, daemon=True,
                                                 name="scoop-slew-down").start()
                            else:
                                # UP: slew slowly in a background thread so it doesn't jerk
                                def _slew_up():
                                    step_size = (_SCOOP_QUS_DOWN - _SCOOP_QUS_UP) / _SCOOP_UP_STEPS
                                    for i in range(_SCOOP_UP_STEPS, -1, -1):
                                        target_qus = int(_SCOOP_QUS_UP + step_size * i)
                                        s_low  = target_qus & 0x7F
                                        s_high = (target_qus >> 7) & 0x7F
                                        motor_interface.send_command(
                                            f"uart_servo 0x{s_low:02X} 0x{s_high:02X}"
                                        )
                                        time.sleep(_SCOOP_UP_STEP_DELAY)
                                threading.Thread(target=_slew_up, daemon=True,
                                                 name="scoop-slew-up").start()

                        elif msg_type == "stop":
                            motor_interface.send_command("stop")

                        else:
                            logger.warning(f"Command '{msg_type}' not allowed in joystick mode")
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
                            logger.warning("Joystick commands not allowed in voice mode")
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
                            logger.warning(f"Unknown command type: {msg_type}")

                    elif control_mode == "ml":
                        if msg_type == "stop":
                            motor_interface.send_command("stop")
                        else:
                            logger.warning(f"Command '{msg_type}' not allowed in ml mode")
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
                    logger.error(f"Invalid JSON received: {message}")
                except Exception as e:
                    logger.exception(f"Error processing motor command: {e}")

    except Exception as e:
        # 1000 = normal closure, 1001 = going away, 1005 = no status (browser tab close)
        e_str = str(e)
        if "1000" in e_str or "1001" in e_str or "1005" in e_str:
            logger.info(f"Motor control client disconnected (normal closure): {e}")
        else:
            logger.error(f"Motor control client error: {e}")
    finally:
        # Remove client from the set of connected clients
        with motor_clients_lock:
            motor_clients.discard(ws)
        logger.info("Motor control client disconnected.")


@sock.route("/gpio")
def gpio_socket(ws):
    """WebSocket for GPIO motor control and limit-switch status streaming."""
    if not _authorize_socket(ws):
        return

    logger.info("GPIO client connected.")

    # Send initial state immediately
    def _send_state():
        try:
            switches = gpio_interface.get_switch_states()
            motors   = gpio_interface.get_motor_states()
            ws.send(json.dumps({
                "type":            "gpio_state",
                "roller_switch":   switches["roller_switch"],
                "conveyor_switch": switches["conveyor_switch"],
                "roller_motor":    motors["roller_motor"],
                "conveyor_motor":  motors["conveyor_motor"],
            }))
        except Exception:
            pass

    _send_state()

    # Register a callback so switch changes are pushed immediately
    _client_alive = [True]

    def _on_switch_change(roller, conveyor):
        if not _client_alive[0]:
            return
        try:
            motors = gpio_interface.get_motor_states()
            ws.send(json.dumps({
                "type":            "gpio_state",
                "roller_switch":   roller,
                "conveyor_switch": conveyor,
                "roller_motor":    motors["roller_motor"],
                "conveyor_motor":  motors["conveyor_motor"],
            }))
        except Exception:
            _client_alive[0] = False

    gpio_interface.register_switch_callback(_on_switch_change)

    # ── Ball-cycle progress forwarding ────────────────────────────────────
    def _on_cycle_progress(phase, message, done, error):
        """Forward BallCycleManager events to this GPIO WebSocket client."""
        if not _client_alive[0]:
            return
        try:
            ws.send(json.dumps({
                "type":    "ball_cycle_progress",
                "phase":   phase.name,
                "message": message,
                "done":    done,
                "error":   error,
            }))
        except Exception:
            _client_alive[0] = False

    ball_cycle_manager.register_progress_callback(_on_cycle_progress)

    # ── Dry-run state ──────────────────────────────────────────────────────
    _dry_run_cancel = [False]
    _dry_run_thread = [None]

    def _send_dry_run(step, message, done=False, error=False):
        """Push a dry-run progress event to this client."""
        if not _client_alive[0]:
            return
        try:
            ws.send(json.dumps({
                "type":    "dry_run_progress",
                "step":    step,
                "message": message,
                "done":    done,
                "error":   error,
            }))
        except Exception:
            _client_alive[0] = False

    def _run_dry_run_sequence():
        """
        Ball dry-run sequence (runs in a background thread):

        1. Start roller motor.
        2. Wait for roller switch to trigger.
        3. Run roller 3 extra seconds, then stop.
        4. Move scoop to UP position via UART servo.
        5. Start conveyor motor.
        6. Wait for conveyor switch to trigger (30 s timeout).
        7. Keep conveyor running 1.5 s after switch, then STOP conveyor.
        8. Wait 10 s (ball settling / scoop-down delay).
        9. Run conveyor 10 s to dispense ball, then stop.
        """
        import time as _time

        def cancelled():
            return _dry_run_cancel[0] or not _client_alive[0]

        # ── Step 1: start roller ────────────────────────────────────────
        _send_dry_run(1, "Starting roller motor…")
        gpio_interface.set_roller_motor(True)
        _send_state()

        # ── Step 2: wait for roller switch ──────────────────────────────
        _send_dry_run(2, "Waiting for roller switch (ball in scoop)…")
        while not cancelled():
            states = gpio_interface.get_switch_states()
            if states["roller_switch"]:
                break
            _time.sleep(0.05)

        if cancelled():
            gpio_interface.set_roller_motor(False)
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            _send_state()
            return

        _send_dry_run(2, "Roller switch triggered — running 3 more seconds…")

        # ── Step 3: extra 3 s then stop roller ──────────────────────────
        deadline = _time.monotonic() + 3.0
        while _time.monotonic() < deadline:
            if cancelled():
                break
            _time.sleep(0.05)

        gpio_interface.set_roller_motor(False)
        _send_state()

        if cancelled():
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            return

        _send_dry_run(3, "Roller stopped. Moving scoop to UP position…")

        # ── Step 4: scoop UP via UART servo ───────────────────────────────────
        # Slew UP slowly (mirrors the DOWN slew) so the scoop doesn’t jerk.
        step_size = (_SCOOP_QUS_DOWN - _SCOOP_QUS_UP) / _SCOOP_UP_STEPS
        for i in range(_SCOOP_UP_STEPS, -1, -1):
            if cancelled():
                _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
                return
            target_qus = int(_SCOOP_QUS_UP + step_size * i)
            s_low  = target_qus & 0x7F
            s_high = (target_qus >> 7) & 0x7F
            motor_interface.send_command(f"uart_servo 0x{s_low:02X} 0x{s_high:02X}")
            _time.sleep(_SCOOP_UP_STEP_DELAY)

        if cancelled():
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            return

        # ── Step 5: start conveyor ───────────────────────────────────────
        _send_dry_run(4, "Scoop up. Starting conveyor motor…")
        gpio_interface.set_conveyor_motor(True)
        _send_state()

        # ── Step 6: wait for conveyor switch (30 s timeout) ────────────
        _send_dry_run(5, "Waiting for conveyor switch (ball at sensor)...")
        conv_deadline  = _time.monotonic() + 30.0
        next_log_time  = _time.monotonic() + 1.0
        conv_timed_out = False

        while not cancelled():
            if _time.monotonic() >= conv_deadline:
                conv_timed_out = True
                _send_dry_run(5, "Conveyor switch timeout - stopping conveyor anyway...")
                logger.warning("[DRY RUN] Conveyor switch timeout after 30 s")
                break
            states = gpio_interface.get_switch_states()
            conv = states["conveyor_switch"]
            if _time.monotonic() >= next_log_time:
                logger.debug("[DRY RUN] Waiting for conveyor switch - state: %s", conv)
                next_log_time = _time.monotonic() + 1.0
            if conv:
                logger.info("[DRY RUN] Conveyor switch TRIGGERED.")
                break
            _time.sleep(0.05)

        if cancelled():
            gpio_interface.set_conveyor_motor(False)
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            _send_state()
            return

        # ── Step 7: keep conveyor running 1.5 s then STOP ───────────────
        if not conv_timed_out:
            _send_dry_run(6, "Conveyor switch triggered - running 1.5 s more then stopping...")
        deadline = _time.monotonic() + 1.5
        while _time.monotonic() < deadline:
            if cancelled():
                break
            _time.sleep(0.05)

        gpio_interface.set_conveyor_motor(False)
        _send_state()

        if cancelled():
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            return

        # ── Step 7b: slew scoop DOWN while waiting ───────────────────────
        _send_dry_run(7, "Conveyor stopped. Slewing scoop DOWN...")
        step_size = (_SCOOP_QUS_DOWN - _SCOOP_QUS_UP) / _SCOOP_DOWN_STEPS
        for i in range(1, _SCOOP_DOWN_STEPS + 1):
            if cancelled():
                _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
                return
            target_qus = int(_SCOOP_QUS_UP + step_size * i)
            s_low  = target_qus & 0x7F
            s_high = (target_qus >> 7) & 0x7F
            motor_interface.send_command(f"uart_servo 0x{s_low:02X} 0x{s_high:02X}")
            _time.sleep(_SCOOP_DOWN_STEP_DELAY)

        if cancelled():
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            return

        # ── Step 8: wait remainder of 10 s before restarting conveyor ────
        _send_dry_run(7, "Scoop down. Waiting for dispense window...")
        deadline = _time.monotonic() + 10.0
        while _time.monotonic() < deadline:
            if cancelled():
                break
            _time.sleep(0.05)

        if cancelled():
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            return

        # ── Step 9: dispense - run conveyor 10 s ────────────────────────
        _send_dry_run(8, "Dispensing - running conveyor 10 seconds...")
        gpio_interface.set_conveyor_motor(True)
        _send_state()
        deadline = _time.monotonic() + 10.0
        while _time.monotonic() < deadline:
            if cancelled():
                break
            _time.sleep(0.05)

        gpio_interface.set_conveyor_motor(False)
        _send_state()

        if cancelled():
            _send_dry_run(0, "Dry run cancelled.", done=True, error=True)
            return

        _send_dry_run(9, "Ball dry run complete!", done=True)
        logger.info("Ball dry run sequence completed successfully.")

    try:
        while True:
            message = ws.receive()
            if message is None:
                break

            if isinstance(message, str):
                try:
                    data = json.loads(message)
                    msg_type = data.get("type")

                    if msg_type == "set_roller_motor":
                        state = bool(data.get("state", False))
                        gpio_interface.set_roller_motor(state)
                        _send_state()

                    elif msg_type == "set_conveyor_motor":
                        state = bool(data.get("state", False))
                        gpio_interface.set_conveyor_motor(state)
                        _send_state()

                    elif msg_type == "start_dry_run":
                        # Only allow one dry run at a time
                        if _dry_run_thread[0] and _dry_run_thread[0].is_alive():
                            ws.send(json.dumps({
                                "type":    "dry_run_progress",
                                "step":    0,
                                "message": "Dry run already in progress.",
                                "done":    False,
                                "error":   True,
                            }))
                        else:
                            _dry_run_cancel[0] = False
                            t = threading.Thread(
                                target=_run_dry_run_sequence,
                                daemon=True,
                                name="dry-run",
                            )
                            _dry_run_thread[0] = t
                            t.start()

                    elif msg_type == "cancel_dry_run":
                        _dry_run_cancel[0] = True
                        logger.info("Dry run cancellation requested.")

                    elif msg_type == "cancel_ball_cycle":
                        ball_cycle_manager.cancel()
                        logger.info("Ball cycle cancellation requested via GPIO socket.")

                    elif msg_type == "ping":
                        ws.send(json.dumps({"type": "pong"}))

                    else:
                        logger.warning("Unknown GPIO command: %s", msg_type)

                except json.JSONDecodeError:
                    logger.error("Invalid JSON on GPIO socket: %s", message)
                except Exception:
                    logger.exception("Error processing GPIO command")

    except Exception as e:
        e_str = str(e)
        if "1000" in e_str or "1001" in e_str or "1005" in e_str:
            logger.info("GPIO client disconnected (normal closure): %s", e)
        else:
            logger.error("GPIO client error: %s", e)
    finally:
        _client_alive[0] = False
        _dry_run_cancel[0] = True   # stop any in-flight dry run
        ball_cycle_manager.unregister_progress_callback(_on_cycle_progress)
        gpio_interface.unregister_switch_callback(_on_switch_change)
        logger.info("GPIO client disconnected.")


@sock.route("/tuning_telemetry")
def tuning_telemetry_socket(ws):
    """WebSocket for high-frequency PID tuning telemetry."""
    if not _authorize_socket(ws):
        return

    from .tuning_manager import pid_tuning_manager
    logger.info("Tuning telemetry client connected.")

    try:
        while True:
            # Send telemetry JSON
            telemetry = pid_tuning_manager.get_telemetry()
            ws.send(json.dumps({
                "type": "telemetry",
                "heading": telemetry["heading"],
                "target_heading": telemetry.get("target_heading"),
                "time": telemetry["time"]
            }))
            time.sleep(0.1) # 10Hz
    except Exception as e:
        e_str = str(e)
        if "1000" in e_str or "1001" in e_str or "1005" in e_str or "Broken pipe" in e_str:
            pass # normal
        else:
            logger.error("Tuning telemetry client error: %s", e)
    finally:
        logger.info("Tuning telemetry client disconnected.")
