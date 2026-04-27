"""
Navigation Controller (Thin Client)
Delegates all path planning and odometry to the C motor controller.
"""
import threading
import logging
from dataclasses import dataclass

try:
    from course_config import CENTER, START_HEADING, START_POSITION, get_bucket_position
except ImportError:
    from web_server.course_config import (
        CENTER,
        START_HEADING,
        START_POSITION,
        get_bucket_position,
    )

logger = logging.getLogger(__name__)

@dataclass
class NavigationCommand:
    """A queued navigation command"""
    command_type: str  # "bucket", "center"
    target: str  # color name or "center"
    position: tuple  # (x, y)


class CoordinatedNavigationController:
    def __init__(self, send_command_callback):
        self.send_command = send_command_callback
        self._lock = threading.RLock()

        # Mirror state from C process
        self.x = START_POSITION[0]
        self.y = START_POSITION[1]
        self.heading = START_HEADING
        self.state = "IDLE"

        self.command_queue = []
        self.queue_running = False

    def get_position(self):
        """Return current status dictionary."""
        with self._lock:
            queue_snapshot = [{"target": cmd.target} for cmd in self.command_queue]
            current_target = (
                self.command_queue[0].position
                if (self.queue_running and self.command_queue)
                else None
            )

        return {
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "state": self.state,
            "mode": "c_planning",
            "queue_running": self.queue_running,
            "queue": queue_snapshot,
            "current_target": current_target,
        }

    def set_speed_percent(self, speed_percent: float):
        """Send speed command to C (0.0 = 0%, 1.0 = 100%)."""
        speed_percent = max(0.0, min(1.0, float(speed_percent)))
        self._send_command(f"speed {speed_percent:.2f}")

    def go_to_center(self):
        """Immediately dispatch a goto-center command to the C process.

        Called by BallCycleManager which manages its own sequencing and polls
        for NAV_IDLE between steps — so we must fire the command directly
        rather than relying on start_queue() being called afterwards.
        """
        x, y = CENTER
        logger.info("[NAV] Dispatching goto CENTER (%.2f, %.2f)", x, y)
        self._send_command(f"goto {x:.2f} {y:.2f} 0")

    def go_to_bucket(self, color: str):
        """Immediately dispatch a goto-bucket command to the C process.

        Same rationale as go_to_center — direct dispatch for BallCycleManager.
        """
        pos = get_bucket_position(color)
        if pos:
            x, y = pos
            logger.info("[NAV] Dispatching goto %s bucket (%.2f, %.2f)", color.upper(), x, y)
            self._send_command(f"goto {x:.2f} {y:.2f} 1")

    def go_to_point(self, x, y):
        """Immediately dispatch a goto-point command to the C process."""
        logger.info("[NAV] Dispatching goto POINT (%.2f, %.2f)", x, y)
        self._send_command(f"goto {x:.2f} {y:.2f} 0")

    # --- Queue Management ---
    def queue_command(self, cmd):
        with self._lock:
            self.command_queue.append(cmd)
        logger.info(f"[NAV] Queued: {cmd.target}")

    def start_queue(self):
        should_process = False
        with self._lock:
            if not self.queue_running and self.command_queue:
                self.queue_running = True
                should_process = True

        if should_process:
            self._process_next_command()

    def clear_queue(self):
        with self._lock:
            self.command_queue = []
            self.queue_running = False
        self._send_command("stop")  # Also stop C process

    def reset_position(self, x=None, y=None, heading=None):
        """Reset position in C code."""
        if x is None:
            x = START_POSITION[0]
        if y is None:
            y = START_POSITION[1]
        if heading is None:
            heading = START_HEADING

        if self._send_command(f"setpos {x:.2f} {y:.2f} {heading:.2f}"):
            # Update local mirror immediately
            with self._lock:
                self.x = x
                self.y = y
                self.heading = heading

    def calibrate(self):
        """Calibrate gyro and reset to start position."""
        if self._send_command("calibrate"):
            # Update local mirror immediately
            with self._lock:
                self.x = START_POSITION[0]
                self.y = START_POSITION[1]
                self.heading = START_HEADING

    def _process_next_command(self):
        with self._lock:
            if not self.queue_running:
                return

            if not self.command_queue:
                self.queue_running = False
                return

            cmd = self.command_queue[0]
            bucket_flag = 1 if cmd.command_type == "bucket" else 0
            command = f"goto {cmd.position[0]:.2f} {cmd.position[1]:.2f} {bucket_flag}"

            # Keep dispatch under lock so queue clear/start cannot interleave command sends.
            if not self._send_command(command):
                self.queue_running = False
                logger.error("[NAV] Failed to dispatch command because motor process is unavailable")
                return

        logger.info(f"[NAV] Executing: {cmd.target} -> {cmd.position}")

    # --- Feedback Handling (called from motor_interface) ---

    def handle_status_update(self, x, y, heading, state_code):
        """Called when C prints STATUS x y h s"""
        finished_command = None
        trigger_next = False
        queue_complete = False

        with self._lock:
            self.x = x
            self.y = y
            self.heading = heading

            # Keep this mapping aligned with NavState in c_code/include/common.h.
            states = {
                0: "IDLE",
                1: "TURNING",
                2: "DRIVING",
                3: "PLANNING",
                4: "BUCKET_ROTATE",
                5: "BUCKET_BACKUP",
                6: "ML",
            }
            new_state = states.get(state_code, "UNKNOWN")
            previous_state = self.state
            self.state = new_state

            # Check if we finished a move (transition from NON-IDLE to IDLE)
            if previous_state != "IDLE" and new_state == "IDLE" and self.queue_running:
                if self.command_queue:
                    finished_command = self.command_queue.pop(0)

                if self.command_queue:
                    trigger_next = True
                else:
                    self.queue_running = False
                    queue_complete = True

        if finished_command:
            logger.info(f"[NAV] Finished: {finished_command.target}")

        if trigger_next:
            self._process_next_command()
        elif queue_complete:
            logger.info("[NAV] Queue Complete")

    def _send_command(self, command):
        """Send command through callback; returns True on accepted command."""
        try:
            return bool(self.send_command(command))
        except Exception as exc:
            logger.error(f"[NAV] Failed to send command '{command}': {exc}")
            return False
