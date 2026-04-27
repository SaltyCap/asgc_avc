import logging
import os
import queue
import subprocess
import threading
import time

from .config import Config

logger = logging.getLogger(__name__)


class MotorInterface:
    def __init__(self):
        self.process = None
        self.command_queue = queue.Queue(maxsize=Config.MAX_MOTOR_COMMAND_QUEUE)
        self.lock = threading.Lock()
        self.nav_controller = None
        self.running = False
        self._last_offline_warning_ts = 0.0

    def start(self, nav_controller=None):
        """Starts the motor control subprocess."""
        self.nav_controller = nav_controller

        with self.lock:
            if self._is_process_alive_locked():
                self.running = True
                return True

        self._clear_pending_commands()

        motor_path, project_root = self._resolve_motor_path()
        if not os.path.exists(motor_path):
            logger.error(f"Motor control program not found at {motor_path}")
            return False
        if not os.access(motor_path, os.X_OK):
            logger.error(f"Motor control program is not executable: {motor_path}")
            return False

        try:
            with self.lock:
                self.process = subprocess.Popen(
                    [motor_path],
                    cwd=project_root,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1
                )
                self.running = True
            logger.info(f"Motor control process started (PID: {self.process.pid})")

            # Start threads
            threading.Thread(target=self._read_output, daemon=True, name="motor-stdout").start()
            threading.Thread(target=self._read_errors, daemon=True, name="motor-stderr").start()
            threading.Thread(target=self._send_commands, daemon=True, name="motor-stdin").start()

            return True
        except Exception as e:
            with self.lock:
                self.running = False
                self.process = None
            logger.exception(f"Failed to start motor control: {e}")
            return False

    def stop(self):
        """Stops the motor control subprocess."""
        process = self.process
        if not process:
            return

        # Send shutdown commands directly before stopping command threads.
        try:
            if process.stdin and process.poll() is None:
                process.stdin.write("stop\n")
                process.stdin.write("q\n")
                process.stdin.flush()
        except Exception as e:
            logger.warning(f"Failed to send graceful shutdown commands: {e}")

        self.running = False
        self._clear_pending_commands()

        try:
            process.wait(timeout=3)
        except KeyboardInterrupt:
            # ^C during shutdown — kill the process immediately and re-raise
            # so the outer finally block in web_server.py still completes.
            process.kill()
            try:
                process.wait(timeout=1)
            except Exception:
                pass
            with self.lock:
                if self.process is process:
                    self.process = None
            logger.info("Motor control process killed (KeyboardInterrupt)")
            raise
        except Exception:
            process.terminate()
            try:
                process.wait(timeout=2)
            except Exception:
                process.kill()

        with self.lock:
            if self.process is process:
                self.process = None
        logger.info("Motor control process stopped")

    def send_command(self, command):
        """Queue a command to be sent to the motor control program."""
        if not command:
            return False

        if not self._is_process_alive():
            now = time.monotonic()
            if now - self._last_offline_warning_ts > 2.0:
                logger.warning(f"Dropping command while motor process is offline: {command}")
                self._last_offline_warning_ts = now
            return False

        try:
            self.command_queue.put_nowait(command)
            return True
        except queue.Full:
            logger.warning("Command queue full; dropping command")
            return False

    def _send_commands(self):
        """Sends queued commands to motor control process."""
        while self.running:
            try:
                process = self.process
                if not process or process.poll() is not None:
                    self.running = False
                    break
                command = self.command_queue.get(timeout=0.1)
                if command and process.stdin:
                    process.stdin.write(command + "\n")
                    process.stdin.flush()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error sending command to motor: {e}")
                self.running = False
                break

    def _read_output(self):
        """Reads and prints output from motor control process."""
        while self.running:
            try:
                process = self.process
                if not process or process.poll() is not None or not process.stdout:
                    self.running = False
                    break
                line = process.stdout.readline()
                if line:
                    line = line.strip()
                    logger.debug(f"[MOTOR] {line}")
                    self._handle_motor_feedback(line)
                elif process.poll() is not None:
                    self.running = False
                    break
            except Exception as e:
                logger.error(f"Error reading motor output: {e}")
                self.running = False
                break

    def _read_errors(self):
        """Continuously drain stderr to prevent child process pipe blockage."""
        while self.running:
            try:
                process = self.process
                if not process or process.poll() is not None or not process.stderr:
                    self.running = False
                    break
                line = process.stderr.readline()
                if line:
                    line = line.strip()
                    if not line:
                        continue
                        
                    # Parse log level if present
                    if line.startswith("DEBUG:"):
                        logger.debug(f"[MOTOR] {line[6:].strip()}")
                    elif line.startswith("INFO:"):
                        logger.info(f"[MOTOR] {line[5:].strip()}")
                    elif line.startswith("WARNING:"):
                        logger.warning(f"[MOTOR] {line[8:].strip()}")
                    elif line.startswith("ERROR:"):
                        logger.error(f"[MOTOR] {line[6:].strip()}")
                    else:
                        # Default to error for raw stderr output
                        logger.error(f"[MOTOR STDERR] {line}")
                elif process.poll() is not None:
                    self.running = False
                    break
            except Exception as e:
                logger.error(f"Error reading motor stderr: {e}")
                self.running = False
                break

    def _clear_pending_commands(self):
        """Drop queued commands that no longer apply."""
        while True:
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break

    def _resolve_motor_path(self):
        app_dir = os.path.dirname(os.path.abspath(__file__))
        web_server_dir = os.path.dirname(app_dir)
        project_root = os.path.dirname(web_server_dir)
        # The C binary uses relative paths like "../logs" that are relative to
        # the c_code/ directory, so we must run it from there.
        c_code_dir = os.path.join(project_root, "c_code")

        configured_path = Config.get_motor_control_path()
        if os.path.isabs(configured_path):
            return configured_path, c_code_dir

        candidates = [
            os.path.join(project_root, "c_code", configured_path),
            os.path.abspath(os.path.join(web_server_dir, "..", "c_code", configured_path)),
        ]
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate, c_code_dir
        return candidates[0], c_code_dir

    def _is_process_alive_locked(self):
        return bool(self.process and self.process.poll() is None)

    def _is_process_alive(self):
        with self.lock:
            return self._is_process_alive_locked()

    def _handle_motor_feedback(self, line):
        """Parses motor feedback and updates navigation controller."""
        # Check for READY message and send initial speed setting
        if line.startswith("READY"):
            # Send initial speed to match web interface slider default (25%)
            self.send_command("speed 0.25")
            logger.info("Motor initialized speed to 25% (matching slider default)")

        if not self.nav_controller:
            return

        parts = line.split()
        if not parts:
            return

        if parts[0] == "STATUS" and len(parts) >= 5:
            try:
                x = float(parts[1])
                y = float(parts[2])
                h = float(parts[3])
                s = int(parts[4])
                if hasattr(self.nav_controller, "handle_status_update"):
                    self.nav_controller.handle_status_update(x, y, h, s)
            except ValueError:
                pass


# Global instance
motor_interface = MotorInterface()
