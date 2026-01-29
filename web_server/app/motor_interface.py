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
        # We assume the project structure:
        # project_root/
        #   web_server/
        #     app/
        #       motor_interface.py
        #   c_code/
        #     asgc_motor_control
        
        # Determine project root based on this file's location
        app_dir = os.path.dirname(os.path.abspath(__file__))
        web_server_dir = os.path.dirname(app_dir)
        project_root = os.path.dirname(web_server_dir)
        
        motor_exec_name = os.path.basename(Config.get_motor_control_path())
        motor_path = os.path.join(project_root, "c_code", motor_exec_name)
        
        # Verify the file exists and is executable
        if not os.path.exists(motor_path):
            # Fallback for different CWD scenarios
            fallback_path = os.path.abspath(os.path.join(web_server_dir, "../c_code", motor_exec_name))
            if os.path.exists(fallback_path):
                motor_path = fallback_path

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

        if parts[0] == "STATUS" and len(parts) >= 5:
            try:
                x = float(parts[1])
                y = float(parts[2])
                h = float(parts[3])
                s = int(parts[4])
                if hasattr(self.nav_controller, 'handle_status_update'):
                    self.nav_controller.handle_status_update(x, y, h, s)
            except ValueError:
                pass



# Global instance
motor_interface = MotorInterface()
