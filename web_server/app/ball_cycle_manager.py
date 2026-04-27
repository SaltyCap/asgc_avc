"""
ball_cycle_manager.py — Orchestrates the full ball-retrieval cycle.

Per-queued-bucket-word sequence:
  1. Start roller motor, then drive toward CENTER.
  2. Continue driving until the roller switch triggers (ball in scoop).
     The robot keeps moving while the roller runs — no static wait.
  3. Stop driving.  Run roller 3 extra seconds then stop.
  4. Move scoop UP via UART servo.
  5. Start conveyor motor; wait until conveyor switch triggers (ball at sensor).
  6. Drop scoop DOWN via UART servo.
  7. Stop conveyor.
  8. Drive to the queued bucket (bucket_flag=1 → C code does backup manoeuvre).
  9. Wait until nav IDLE.
 10. Run conveyor 15 seconds to dispense ball into bucket.
 11. Return to center (no roller) — robot is always back at center after each ball.
 12. Repeat from step 1 for the next queued bucket.

Usage:
    from .ball_cycle_manager import ball_cycle_manager
    ball_cycle_manager.init(nav_controller, gpio_interface, motor_interface)
    ball_cycle_manager.enqueue_bucket("red")   # from VoiceCommandProcessor
    ball_cycle_manager.start()                 # from "start" voice command
    ball_cycle_manager.cancel()                # from "stop" voice command
"""

import logging
import threading
import time
from collections import deque
from enum import Enum, auto

logger = logging.getLogger(__name__)

# ── Servo pulse byte constants (matches joystick.html / uart.h) ─────────────
_SCOOP_UP_LOW   = 0x00  # pulse_us = 544  → scoop raised (+4° from previous 500 µs)
_SCOOP_UP_HIGH  = 0x11
_SCOOP_DOWN_LOW  = 0x70  # pulse_us = 1500 → scoop lowered
_SCOOP_DOWN_HIGH = 0x2E

# ── Timing constants ─────────────────────────────────────────────────────────
_ROLLER_EXTRA_S         = 3.0    # extra roller run after switch triggers
_SCOOP_UP_DELAY_S       = 1.5    # settling delay after roller stops before scoop moves UP
_SCOOP_TRAVEL_S         = 0.5    # pause after servo command for scoop to travel (UP)
_CONVEYOR_STOP_DELAY_S  = 1.5    # wait after conveyor switch before stopping conveyor
_CONVEYOR_RESTART_DELAY_S = 10.0 # wait after conveyor stops before restarting for dispense
_DISPENSE_S             = 15.0   # conveyor run time to dispense ball into bucket
_ROLLER_SWITCH_TIMEOUT_S = 3.0   # max wait for roller switch before proceeding without ball
_NAV_POLL_S             = 0.05   # how often to poll nav state
_NAV_START_WAIT_S       = 1.0    # delay before polling so C code leaves IDLE first

# ── Scoop slew control ──────────────────────────────────────────────────────────
# The scoop slams when sent directly to UP or DOWN in one command. We step
# through intermediate Maestro targets to slow the travel in both directions.
_SCOOP_DOWN_STEPS      = 20    # incremental steps from UP → DOWN
_SCOOP_DOWN_STEP_DELAY = 0.10  # seconds between each step (~2 s total travel)
_SCOOP_UP_STEPS        = 20    # incremental steps from DOWN → UP
_SCOOP_UP_STEP_DELAY   = 0.10  # seconds between each step (~2 s total travel)
# Quarter-μs targets:  UP=1824 (456 μs / -4° from original 500 μs), DOWN=6000 (1500 μs)
_SCOOP_QUS_UP   = 1824
_SCOOP_QUS_DOWN = 6000


class CyclePhase(Enum):
    IDLE                = auto()
    GOING_TO_CENTER     = auto()
    PICKING_UP_BALL     = auto()
    GOING_TO_BUCKET     = auto()
    DISPENSING          = auto()
    RETURNING_TO_CENTER = auto()


class BallCycleManager:
    """
    Singleton that owns the full pick-and-place cycle for each voice-queued
    bucket target.
    """

    def __init__(self):
        self._nav_controller   = None
        self._gpio_interface   = None
        self._motor_interface  = None

        self._bucket_queue: deque[str] = deque()
        self._queue_lock = threading.Lock()

        self._cancel  = threading.Event()
        self._thread  = None
        self._running = False

        self.phase = CyclePhase.IDLE

        # Callbacks: called with (phase: CyclePhase, message: str, done: bool, error: bool)
        self._progress_callbacks = []

    # ── Public API ───────────────────────────────────────────────────────────

    def init(self, nav_controller, gpio_interface, motor_interface):
        """Wire up external dependencies (call once from create_app)."""
        self._nav_controller  = nav_controller
        self._gpio_interface  = gpio_interface
        self._motor_interface = motor_interface
        logger.info("[BALL CYCLE] Manager initialised.")

    def enqueue_bucket(self, color: str):
        """Add a bucket color to the queue.  Thread-safe."""
        with self._queue_lock:
            self._bucket_queue.append(color.lower())
        logger.info("[BALL CYCLE] Enqueued bucket: %s  (queue depth: %d)",
                    color, len(self._bucket_queue))
        self._notify(CyclePhase.IDLE,
                     f"Queued bucket '{color}'. Say 'start' to begin.", done=False)

    def start(self):
        """Start the cycle thread if not already running."""
        if self._running and self._thread and self._thread.is_alive():
            logger.info("[BALL CYCLE] Already running.")
            return
        if not self._bucket_queue:
            logger.info("[BALL CYCLE] start() called with empty queue — nothing to do.")
            return
        self._cancel.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._cycle_loop,
            daemon=True,
            name="ball-cycle",
        )
        self._thread.start()
        logger.info("[BALL CYCLE] Cycle started.")

    def cancel(self):
        """Signal cancellation and stop all motors."""
        self._cancel.set()
        with self._queue_lock:
            self._bucket_queue.clear()
        self._safe_stop_motors()
        logger.info("[BALL CYCLE] Cancelled.")
        self._notify(CyclePhase.IDLE, "Ball cycle cancelled.", done=True, error=True)

    def clear_queue(self):
        """Clear pending queue without stopping a running pick-up."""
        with self._queue_lock:
            self._bucket_queue.clear()
        logger.info("[BALL CYCLE] Queue cleared.")

    def register_progress_callback(self, cb):
        """Register cb(phase, message, done, error) for progress events."""
        self._progress_callbacks.append(cb)

    def unregister_progress_callback(self, cb):
        try:
            self._progress_callbacks.remove(cb)
        except ValueError:
            pass

    def get_phase(self) -> str:
        return self.phase.name

    # ── Internal cycle loop ──────────────────────────────────────────────────

    def _cycle_loop(self):
        """Main loop: process each queued bucket one at a time."""
        try:
            while not self._cancel.is_set():
                with self._queue_lock:
                    if not self._bucket_queue:
                        break
                    color = self._bucket_queue.popleft()
                
                self.current_color = color
                logger.info("[BALL CYCLE] === Starting cycle for bucket: %s ===", color)
                success = self._run_one_cycle(color)
                self.current_color = None
                
                if not success or self._cancel.is_set():
                    break

            if not self._cancel.is_set():
                self._notify(CyclePhase.IDLE, "✅ All ball cycles complete!", done=True)
                logger.info("[BALL CYCLE] All cycles complete.")
        except Exception:
            logger.exception("[BALL CYCLE] Unexpected error in cycle loop.")
            self._notify(CyclePhase.IDLE, "Ball cycle error — check logs.", done=True, error=True)
        finally:
            # Stop GPIO motors (roller/conveyor) but do NOT send 'stop' to the
            # drive — the car should be free to move after the queue finishes.
            try:
                if self._gpio_interface:
                    self._gpio_interface.set_roller_motor(False)
                    self._gpio_interface.set_conveyor_motor(False)
            except Exception:
                logger.exception("[BALL CYCLE] Error stopping GPIO motors in finally")
            self._running = False
            self.phase = CyclePhase.IDLE

    def _run_one_cycle(self, color: str) -> bool:
        """Execute one full pick-and-place cycle.  Returns True on success."""

        # ── Phase A: start roller + navigate to center ──────────────────────
        self.phase = CyclePhase.GOING_TO_CENTER
        self._notify(CyclePhase.GOING_TO_CENTER,
                     f"[{color.upper()}] Starting roller and driving to center…")

        # Start roller motor first so it is already spinning as we approach
        if self._gpio_interface:
            self._gpio_interface.set_roller_motor(True)
        else:
            logger.warning("[BALL CYCLE] gpio_interface not set — roller motor skipped")

        # Send navigation command to center
        if self._nav_controller:
            self._nav_controller.go_to_center()
        else:
            logger.warning("[BALL CYCLE] nav_controller not set — nav skipped")

        # ── Phase B: pick up ball (while driving) ───────────────────────────
        self.phase = CyclePhase.PICKING_UP_BALL
        self._notify(CyclePhase.PICKING_UP_BALL,
                     f"[{color.upper()}] Roller spinning — waiting for ball in scoop…")

        # Wait for roller switch while the robot is driving toward center.
        # The robot keeps moving while the roller runs — no static wait.
        ball_detected = self._wait_for_roller_switch()

        if self._cancel.is_set():
            self._safe_stop_motors()
            return False

        if not ball_detected:
            # Timeout — no ball picked up.  Stop the roller but continue
            # the sequence so the car still navigates to the bucket.
            if self._gpio_interface:
                self._gpio_interface.set_roller_motor(False)
            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] ⚠️ No ball detected ({_ROLLER_SWITCH_TIMEOUT_S:.0f}s timeout) — driving to bucket anyway.")
        else:
            # Ball detected — stop driving immediately
            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] Ball detected! Stopping drive, running roller {_ROLLER_EXTRA_S:.0f}s more…")
            if self._motor_interface:
                self._motor_interface.send_command("stop")

            # Run roller extra seconds
            if not self._sleep(_ROLLER_EXTRA_S):
                self._safe_stop_motors()
                return False

            # Stop roller
            if self._gpio_interface:
                self._gpio_interface.set_roller_motor(False)

            # Brief settling delay so the ball can come to rest before the scoop moves
            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] Roller stopped. Waiting {_SCOOP_UP_DELAY_S:.1f}s for ball to settle…")
            if not self._sleep(_SCOOP_UP_DELAY_S):
                self._safe_stop_motors()
                return False

            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] Moving scoop UP…")

            # Scoop UP — slew slowly so it doesn’t jerk
            if not self._scoop_slew_up():
                self._safe_stop_motors()
                return False

            # Start conveyor
            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] Scoop up. Starting conveyor — waiting for ball at sensor…")
            if self._gpio_interface:
                self._gpio_interface.set_conveyor_motor(True)

            # Wait for conveyor switch (ball reaches sensor)
            if not self._wait_for_conveyor_switch():
                self._safe_stop_motors()
                return False

            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] Conveyor switch triggered — waiting {_CONVEYOR_STOP_DELAY_S:.1f}s before stopping conveyor…")

            # Wait 1.5 s with conveyor still running, then stop it
            if not self._sleep(_CONVEYOR_STOP_DELAY_S):
                self._safe_stop_motors()
                return False

            if self._gpio_interface:
                self._gpio_interface.set_conveyor_motor(False)

            self._notify(CyclePhase.PICKING_UP_BALL,
                         f"[{color.upper()}] Conveyor stopped. Dropping scoop DOWN while waiting {_CONVEYOR_RESTART_DELAY_S:.0f}s…")

            # Scoop DOWN — slew slowly so it doesn't slam
            if not self._scoop_slew_down():
                self._safe_stop_motors()
                return False

            # Wait remaining time before restarting the conveyor for dispensing
            if not self._sleep(_CONVEYOR_RESTART_DELAY_S):
                self._safe_stop_motors()
                return False

        self._notify(CyclePhase.PICKING_UP_BALL,
                     f"[{color.upper()}] Navigating to {color.upper()} bucket…")

        # ── Phase C: navigate to bucket ─────────────────────────────────────
        self.phase = CyclePhase.GOING_TO_BUCKET
        if self._nav_controller:
            self._nav_controller.go_to_bucket(color)
        else:
            logger.warning("[BALL CYCLE] nav_controller not set — bucket nav skipped")

        # Wait for nav to reach bucket (C code handles backup in BUCKET_BACKUP state)
        if not self._wait_for_nav_idle():
            self._safe_stop_motors()
            return False

        # ── Phase D: dispense (only if a ball was picked up) ────────────────
        if ball_detected:
            self._notify(CyclePhase.GOING_TO_BUCKET,
                         f"[{color.upper()}] At bucket. Dispensing ball ({_DISPENSE_S:.0f}s)…")

            self.phase = CyclePhase.DISPENSING
            if self._gpio_interface:
                self._gpio_interface.set_conveyor_motor(True)

            if not self._sleep(_DISPENSE_S):
                if self._gpio_interface:
                    self._gpio_interface.set_conveyor_motor(False)
                return False

            if self._gpio_interface:
                self._gpio_interface.set_conveyor_motor(False)

            self._notify(CyclePhase.DISPENSING,
                         f"[{color.upper()}] ✅ Ball dispensed into {color.upper()} bucket!")
        else:
            self._notify(CyclePhase.GOING_TO_BUCKET,
                         f"[{color.upper()}] At {color.upper()} bucket (no ball to dispense).")

        # ── Phase E: return to center after every dispense ──────────────────
        # The robot picks up one ball at a time, so it must always return to
        # center — both for the next pick-up AND in case this was the last ball.
        self.phase = CyclePhase.RETURNING_TO_CENTER
        self._notify(CyclePhase.RETURNING_TO_CENTER,
                     f"[{color.upper()}] Returning to center…")

        if self._nav_controller:
            self._nav_controller.go_to_center()

        if not self._wait_for_nav_idle():
            return False  # cancelled while returning

        self._notify(CyclePhase.RETURNING_TO_CENTER,
                     f"[{color.upper()}] Back at center.")
        logger.info("[BALL CYCLE] Cycle complete for bucket: %s", color)
        return True

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _wait_for_roller_switch(self, timeout=_ROLLER_SWITCH_TIMEOUT_S):
        """
        Block until the roller switch triggers, timeout expires, or the cycle
        is cancelled.  The robot is driving during this wait — we do NOT
        navigate here; the caller already sent a goto command.
        Returns True on trigger or timeout (sequence continues), False on cancel.
        """
        deadline = time.monotonic() + timeout
        log_interval = 5.0
        next_log = time.monotonic() + log_interval
        while not self._cancel.is_set():
            if time.monotonic() >= deadline:
                logger.warning(
                    "[BALL CYCLE] Roller switch timeout after %.1fs — "
                    "proceeding without ball.", timeout
                )
                return False  # Skip this queue item; don't hang forever
            if self._gpio_interface:
                states = self._gpio_interface.get_switch_states()
                if states.get("roller_switch"):
                    logger.info("[BALL CYCLE] Roller switch TRIGGERED.")
                    return True
            if time.monotonic() >= next_log:
                logger.debug("[BALL CYCLE] Waiting for roller switch (%.0fs remaining)…",
                             deadline - time.monotonic())
                next_log = time.monotonic() + log_interval
            time.sleep(_NAV_POLL_S)
        return False

    def _wait_for_conveyor_switch(self, timeout=30.0):
        """Block until conveyor switch triggers, timeout expires, or cancelled."""
        deadline = time.monotonic() + timeout
        log_interval = 1.0   # log switch state once per second
        next_log = time.monotonic() + log_interval
        while not self._cancel.is_set():
            if time.monotonic() >= deadline:
                logger.warning(
                    "[BALL CYCLE] Conveyor switch timeout after %.1fs — "
                    "stopping conveyor anyway and continuing.", timeout
                )
                return True   # treat timeout as triggered so the sequence continues
            if self._gpio_interface:
                states = self._gpio_interface.get_switch_states()
                conv = states.get("conveyor_switch", False)
                if time.monotonic() >= next_log:
                    logger.debug(
                        "[BALL CYCLE] Waiting for conveyor switch — current state: %s", conv
                    )
                    next_log = time.monotonic() + log_interval
                if conv:
                    logger.info("[BALL CYCLE] Conveyor switch TRIGGERED.")
                    return True
            time.sleep(_NAV_POLL_S)
        return False

    def _wait_for_nav_idle(self, timeout=120.0):
        """
        Block until the nav controller reports IDLE, meaning the current
        goto command has completed.  Waits a short startup delay first so
        the C code has time to transition away from IDLE before we poll.
        Returns True on IDLE, False on cancel or timeout.
        """
        if not self._nav_controller:
            return True  # no nav — treat as immediate success

        # Brief startup wait so C exits IDLE before we start polling
        if not self._sleep(_NAV_START_WAIT_S):
            return False

        deadline = time.monotonic() + timeout
        while not self._cancel.is_set():
            if time.monotonic() >= deadline:
                logger.warning(
                    "[BALL CYCLE] _wait_for_nav_idle timeout after %.1fs — continuing anyway.",
                    timeout
                )
                return True  # treat as arrived so the sequence doesn't hang
            state = getattr(self._nav_controller, "state", "IDLE")
            if state == "IDLE":
                return True
            time.sleep(_NAV_POLL_S)
        return False

    def _scoop_slew_up(self) -> bool:
        """
        Gradually move the scoop from DOWN to UP over _SCOOP_UP_STEPS steps
        so it doesn't jerk.  Returns False if cancelled mid-way.
        """
        if not self._motor_interface:
            return True
        step_size = (_SCOOP_QUS_DOWN - _SCOOP_QUS_UP) / _SCOOP_UP_STEPS
        for i in range(_SCOOP_UP_STEPS, -1, -1):
            if self._cancel.is_set():
                return False
            target_qus = int(_SCOOP_QUS_UP + step_size * i)
            low  = target_qus & 0x7F
            high = (target_qus >> 7) & 0x7F
            self._motor_interface.send_command(
                f"uart_servo 0x{low:02X} 0x{high:02X}"
            )
            time.sleep(_SCOOP_UP_STEP_DELAY)
        logger.debug("[BALL CYCLE] Scoop slew UP complete.")
        return True

    def _scoop_slew_down(self) -> bool:
        """
        Gradually move the scoop from UP to DOWN over _SCOOP_DOWN_STEPS steps
        so it doesn't slam.  Returns False if cancelled mid-way.
        """
        if not self._motor_interface:
            return True
        step_size = (_SCOOP_QUS_DOWN - _SCOOP_QUS_UP) / _SCOOP_DOWN_STEPS
        for i in range(1, _SCOOP_DOWN_STEPS + 1):
            if self._cancel.is_set():
                return False
            target_qus = int(_SCOOP_QUS_UP + step_size * i)
            low  = target_qus & 0x7F
            high = (target_qus >> 7) & 0x7F
            self._motor_interface.send_command(
                f"uart_servo 0x{low:02X} 0x{high:02X}"
            )
            time.sleep(_SCOOP_DOWN_STEP_DELAY)
        logger.debug("[BALL CYCLE] Scoop slew DOWN complete.")
        return True

    def _sleep(self, seconds: float) -> bool:
        """Sleep for `seconds`, returning False early if cancelled."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return False
            time.sleep(_NAV_POLL_S)
        return True

    def _safe_stop_motors(self):
        """Turn off roller and conveyor — safe to call from any state."""
        try:
            if self._gpio_interface:
                self._gpio_interface.set_roller_motor(False)
                self._gpio_interface.set_conveyor_motor(False)
        except Exception:
            logger.exception("[BALL CYCLE] Error stopping GPIO motors")
        try:
            if self._motor_interface:
                self._motor_interface.send_command("stop")
        except Exception:
            logger.exception("[BALL CYCLE] Error stopping motor interface")

    def _notify(self, phase: CyclePhase, message: str,
                done: bool = False, error: bool = False):
        """Fire all registered progress callbacks."""
        logger.info("[BALL CYCLE] %s", message)
        for cb in list(self._progress_callbacks):
            try:
                cb(phase, message, done, error)
            except Exception:
                logger.exception("[BALL CYCLE] Error in progress callback")


# Global singleton
ball_cycle_manager = BallCycleManager()
