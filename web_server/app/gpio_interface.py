"""
gpio_interface.py — GPIO control for the ball-handling mechanism on Pi 5.

Pin assignments
---------------
GPIO  9  – Roller limit switch (ball in scoop)   INPUT  pull-up, see _ROLLER_SWITCH_INVERT
GPIO 25  – Conveyor limit switch (ball at sensor) INPUT  pull-up, see _CONVEYOR_SWITCH_INVERT
GPIO  8  – Roller motor control                   OUTPUT (inverted: LOW = ON, HIGH = OFF)
GPIO 11  – Conveyor motor control                 OUTPUT (inverted: LOW = ON, HIGH = OFF)

Switch wiring note (2026-04-15):
  If a switch is wired normally-open  (NO) with a pull-up, the line sits HIGH
  when open and is pulled LOW when pressed  → LOW = triggered  (INVERT = False).
  If a switch is wired normally-closed (NC) with a pull-up, the line sits LOW
  at rest and goes HIGH when actuated      → HIGH = triggered  (INVERT = True).

Set _ROLLER_SWITCH_INVERT / _CONVEYOR_SWITCH_INVERT to match the physical wiring.

NOTE (2026-04-15): Both motor outputs are inverted (active-low).  GPIO 8 is
the roller and GPIO 11 is the conveyor.
"""

import logging
import threading
import time

logger = logging.getLogger(__name__)

# ── Pin numbers (BCM / GPIO numbering) ─────────────────────────────────────
PIN_ROLLER_SWITCH   = 9    # limit switch: ball in scoop
PIN_CONVEYOR_SWITCH = 25   # limit switch: ball at colour sensor
PIN_ROLLER_MOTOR    = 8    # inverted output: LOW = motor ON, HIGH = motor OFF
PIN_CONVEYOR_MOTOR  = 11   # inverted output: LOW = motor ON, HIGH = motor OFF

# Polling interval for limit-switch reads (seconds)
_POLL_INTERVAL = 0.05  # 20 Hz

# ── Switch polarity — set True if the switch is normally-closed (NC) ────────
# NC wiring: line sits LOW at rest, goes HIGH when ball actuates the switch.
# NO wiring: line sits HIGH at rest, goes LOW when ball actuates the switch.
_ROLLER_SWITCH_INVERT   = False  # False = NO wiring (LOW when triggered)
_CONVEYOR_SWITCH_INVERT = False  # False = NO wiring (LOW when triggered)


try:
    import lgpio
    _LGPIO_AVAILABLE = True
except ImportError:
    _LGPIO_AVAILABLE = False
    logger.warning("lgpio not available — GPIO interface running in simulation mode")


class GpioInterface:
    """Manages the roller and conveyor GPIO pins."""

    def __init__(self):
        self._handle = None          # lgpio chip handle
        self._lock = threading.Lock()
        self._running = False
        self._poll_thread = None

        # Motor states
        self._roller_on = False
        self._conveyor_on = False

        # Switch states (True = ball present / switch triggered)
        self._roller_switch = False
        self._conveyor_switch = False

        # Callbacks: called with (roller_triggered: bool, conveyor_triggered: bool)
        self._switch_callbacks = []

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Open the GPIO chip and configure pins.  Safe to call multiple times."""
        if self._running:
            return True

        if not _LGPIO_AVAILABLE:
            logger.warning("lgpio unavailable — GPIO interface in simulation mode")
            self._running = True
            self._start_polling()
            return True

        try:
            self._handle = lgpio.gpiochip_open(4)  # Pi 5: RP1 chip is gpiochip4

            # Output pins — motor FET gates (inverted/active-low: start HIGH / off)
            lgpio.gpio_claim_output(self._handle, PIN_ROLLER_MOTOR,   1)
            lgpio.gpio_claim_output(self._handle, PIN_CONVEYOR_MOTOR, 1)

            # Input pins — internal pull-up enabled; both switches are active-low.
            # Line sits HIGH when switch is open; pulled LOW when ball is present.
            lgpio.gpio_claim_input(self._handle, PIN_ROLLER_SWITCH,   lgpio.SET_PULL_UP)
            lgpio.gpio_claim_input(self._handle, PIN_CONVEYOR_SWITCH, lgpio.SET_PULL_UP)

            self._running = True
            self._start_polling()
            logger.info(
                "GPIO interface started — roller switch GPIO%d, conveyor switch GPIO%d, "
                "roller motor GPIO%d, conveyor motor GPIO%d",
                PIN_ROLLER_SWITCH, PIN_CONVEYOR_SWITCH,
                PIN_ROLLER_MOTOR, PIN_CONVEYOR_MOTOR,
            )
            return True
        except Exception:
            logger.exception("Failed to initialise GPIO")
            return False

    def stop(self):
        """Release GPIO resources.  Safe to call multiple times / from any thread."""
        # Signal the poll loop to exit first
        self._running = False
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)

        # Atomically grab-and-clear the handle under the lock so the poll
        # thread (which also takes the lock when reading switches) can never
        # race gpio_read() against gpiochip_close().
        with self._lock:
            handle = self._handle
            self._handle = None

        if handle is not None and _LGPIO_AVAILABLE:
            try:
                # Inverted outputs: write HIGH to ensure motors are OFF on shutdown
                lgpio.gpio_write(handle, PIN_ROLLER_MOTOR,   1)
                lgpio.gpio_write(handle, PIN_CONVEYOR_MOTOR, 1)
            except Exception:
                pass
            try:
                lgpio.gpiochip_close(handle)
            except Exception:
                pass
        logger.info("GPIO interface stopped")

    def __del__(self):
        """Last-resort cleanup called by the GC — prevents double-free segfault."""
        # Only attempt cleanup if stop() hasn't already cleared the handle.
        handle = getattr(self, "_handle", None)
        if handle is not None and _LGPIO_AVAILABLE:
            self._handle = None
            try:
                lgpio.gpiochip_close(handle)
            except Exception:
                pass

    # ── Motor control ────────────────────────────────────────────────────────

    def set_roller_motor(self, state: bool) -> bool:
        """Turn the roller motor on (True) or off (False).
        GPIO 8 is inverted/active-low: write LOW (0) to turn ON, HIGH (1) to turn OFF.
        """
        with self._lock:
            self._roller_on = state
            if self._handle is not None and _LGPIO_AVAILABLE:
                try:
                    # Inverted logic: motor ON → write 0, motor OFF → write 1
                    lgpio.gpio_write(self._handle, PIN_ROLLER_MOTOR, 0 if state else 1)
                except Exception:
                    logger.exception("Failed to write roller motor pin")
                    return False
            logger.info("Roller motor → %s", "ON" if state else "OFF")
            return True

    def set_conveyor_motor(self, state: bool) -> bool:
        """Turn the conveyor motor on (True) or off (False).
        GPIO 11 is inverted/active-low: write LOW (0) to turn ON, HIGH (1) to turn OFF.
        """
        with self._lock:
            self._conveyor_on = state
            if self._handle is not None and _LGPIO_AVAILABLE:
                try:
                    # Inverted logic: motor ON → write 0, motor OFF → write 1
                    lgpio.gpio_write(self._handle, PIN_CONVEYOR_MOTOR, 0 if state else 1)
                except Exception:
                    logger.exception("Failed to write conveyor motor pin")
                    return False
            logger.info("Conveyor motor → %s", "ON" if state else "OFF")
            return True

    def get_motor_states(self) -> dict:
        with self._lock:
            return {
                "roller_motor":   self._roller_on,
                "conveyor_motor": self._conveyor_on,
            }

    # ── Limit switch reads ───────────────────────────────────────────────────

    def get_switch_states(self) -> dict:
        """Return the current (debounced) switch states."""
        with self._lock:
            return {
                "roller_switch":   self._roller_switch,
                "conveyor_switch": self._conveyor_switch,
            }

    def register_switch_callback(self, cb):
        """Register a function(roller: bool, conveyor: bool) for switch-change events."""
        self._switch_callbacks.append(cb)

    def unregister_switch_callback(self, cb):
        try:
            self._switch_callbacks.remove(cb)
        except ValueError:
            pass

    # ── Internal polling ─────────────────────────────────────────────────────

    def _start_polling(self):
        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="gpio-poll"
        )
        self._poll_thread.start()

    def _read_switch(self, pin: int, invert: bool = False) -> bool:
        """
        Read a limit switch pin and return True when the switch is TRIGGERED.

        invert=False (normally-open,  NO): LOW  (0) = triggered (active-low).
        invert=True  (normally-closed, NC): HIGH (1) = triggered (active-high).
        """
        if not _LGPIO_AVAILABLE:
            return False
        # Grab handle under lock so we never race against stop() clearing it.
        with self._lock:
            handle = self._handle
        if handle is None:
            return False
        try:
            raw = lgpio.gpio_read(handle, pin)
            triggered = (raw == 1) if invert else (raw == 0)
            return triggered
        except Exception:
            return False

    def _poll_loop(self):
        prev_roller   = None
        prev_conveyor = None

        while self._running:
            roller_state   = self._read_switch(PIN_ROLLER_SWITCH,   _ROLLER_SWITCH_INVERT)
            conveyor_state = self._read_switch(PIN_CONVEYOR_SWITCH, _CONVEYOR_SWITCH_INVERT)

            changed = roller_state != prev_roller or conveyor_state != prev_conveyor

            with self._lock:
                self._roller_switch   = roller_state
                self._conveyor_switch = conveyor_state

            if changed:
                prev_roller   = roller_state
                prev_conveyor = conveyor_state
                logger.info("Switch changed -> Roller=%s (Pin %s), Conveyor=%s (Pin %s)", roller_state, PIN_ROLLER_SWITCH, conveyor_state, PIN_CONVEYOR_SWITCH)
                for cb in list(self._switch_callbacks):
                    try:
                        cb(roller_state, conveyor_state)
                    except Exception:
                        logger.exception("Error in GPIO switch callback")

            time.sleep(_POLL_INTERVAL)


# Global singleton
gpio_interface = GpioInterface()
