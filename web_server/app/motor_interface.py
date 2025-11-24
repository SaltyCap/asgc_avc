import subprocess
import threading
import queue
import os
import sys
from .config import Config

class MotorInterface:
    def __init__(self):
        self.process = None
        self.command_queue = queue.Queue()
        self.lock = threading.Lock()
        self.nav_controller = None
        self.running = False

    def start(self, nav_controller=None):
        """Starts the motor control subprocess."""
        self.nav_controller = nav_controller
        
        # Resolve absolute path to motor control program
        # Config.get_motor_control_path() returns relative path from WebServer root
        # We are in WebServer/app, so we need to go up one level
        base_dir = os.path.dirname(os.path.dirname(__file__))
        motor_path = os.path.join(base_dir, Config.get_motor_control_path().replace("../", ""))
        
        # Fix path resolution if it's still relative to "c_code" which is a sibling of web_server
        # The original code used os.path.dirname(__file__) which was web_server/web_server.py
        # So "../c_code" meant going to parent of web_server.
        # Here, base_dir is web_server. Parent is project root.
        project_root = os.path.dirname(base_dir)
        motor_exec_name = os.path.basename(Config.get_motor_control_path())
        motor_path = os.path.join(project_root, "c_code", motor_exec_name)

        if not os.path.exists(motor_path):
            print(f"ERROR: Motor control program not found at {motor_path}")
            return False

        try:
            with self.lock:
                self.process = subprocess.Popen(
                    ['sudo', motor_path],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1
                )
            self.running = True
            print(f"Motor control process started (PID: {self.process.pid})")

            # Start threads
            threading.Thread(target=self._read_output, daemon=True).start()
            threading.Thread(target=self._send_commands, daemon=True).start()

            return True
        except Exception as e:
            print(f"Failed to start motor control: {e}")
            return False

    def stop(self):
        """Stops the motor control subprocess."""
        self.running = False
        if self.process:
            try:
                self.send_command("stopall")
                self.send_command("q")
                self.process.wait(timeout=3)
            except:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except:
                    self.process.kill()
            self.process = None
            print("Motor control process stopped")

    def send_command(self, command):
        """Queue a command to be sent to the motor control program."""
        self.command_queue.put(command)

    def _send_commands(self):
        """Sends queued commands to motor control process."""
        while self.running and self.process and self.process.poll() is None:
            try:
                command = self.command_queue.get(timeout=0.1)
                if command and self.process and self.process.stdin:
                    self.process.stdin.write(command + '\n')
                    self.process.stdin.flush()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error sending command to motor: {e}")
                break

    def _read_output(self):
        """Reads and prints output from motor control process."""
        while self.running and self.process and self.process.poll() is None:
            try:
                line = self.process.stdout.readline()
                if line:
                    line = line.strip()
                    self._handle_motor_feedback(line)
            except Exception as e:
                print(f"Error reading motor output: {e}")
                break

    def _handle_motor_feedback(self, line):
        """Parses motor feedback and updates navigation controller."""
        if not self.nav_controller:
            return

        parts = line.split()
        if not parts:
            return

        if parts[0] == "COORDINATED_COMPLETE":
            if hasattr(self.nav_controller, 'handle_coordinated_complete'):
                self.nav_controller.handle_coordinated_complete()

        elif parts[0] == "ENCODER" and len(parts) >= 4:
            try:
                motor_id = int(parts[1])
                total_counts = int(parts[2])
                current_angle = int(parts[3])
                if hasattr(self.nav_controller, 'update_encoder_data'):
                    self.nav_controller.update_encoder_data(motor_id, total_counts, current_angle)
            except (ValueError, IndexError):
                pass

        elif parts[0] == "COMPLETE" and len(parts) >= 3:
            try:
                motor_id = int(parts[1])
                final_counts = int(parts[2])
                if hasattr(self.nav_controller, 'handle_motor_complete'):
                    self.nav_controller.handle_motor_complete(motor_id, final_counts)
            except (ValueError, IndexError):
                pass

# Global instance
motor_interface = MotorInterface()
