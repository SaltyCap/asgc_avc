"""
Navigation Controller (Thin Client)
Delegates all path planning and odometry to the C motor controller.
"""
from typing import Optional, List
import time
from dataclasses import dataclass
from course_config import *

@dataclass
class NavigationCommand:
    """A queued navigation command"""
    command_type: str  # 'bucket', 'center'
    target: str  # color name or 'center'
    position: tuple  # (x, y)

class CoordinatedNavigationController:
    def __init__(self, send_command_callback):
        self.send_command = send_command_callback
        
        # Mirror state from C process
        self.x = START_POSITION[0]
        self.y = START_POSITION[1]
        self.heading = START_HEADING
        self.state = "IDLE" 
        self.encoder_age_ms = 0
        
        self.command_queue = []
        self.queue_running = False

    def get_position(self):
        """Return current status dictionary."""
        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'state': self.state,
            'mode': 'c_planning',
            'queue_running': self.queue_running,
            'queue': [{'target': c.target} for c in self.command_queue],
            'current_target': self.command_queue[0].position if (self.queue_running and self.command_queue) else None
        }
        
    def set_speed_multiplier(self, multiplier: float):
        """Send speed command to C."""
        self.send_command(f"speed {multiplier:.2f}")

    def go_to_center(self):
        """Queue center command."""
        self.queue_command(NavigationCommand('center', 'CENTER', CENTER))

    def go_to_bucket(self, color: str):
        """Queue bucket command."""
        pos = get_bucket_position(color)
        if pos:
            self.queue_command(NavigationCommand('bucket', color.upper(), pos))

    # --- Queue Management ---
    def queue_command(self, cmd):
        self.command_queue.append(cmd)
        print(f"[NAV] Queued: {cmd.target}")

    def start_queue(self):
        if not self.queue_running and self.command_queue:
            self.queue_running = True
            self._process_next_command()
            
    def clear_queue(self):
        self.command_queue = []
        self.queue_running = False
        self.send_command("stop") # Also stop C process

    def reset_position(self, x=None, y=None, heading=None):
        """Reset position in C code."""
        if x is None: x = START_POSITION[0]
        if y is None: y = START_POSITION[1]
        if heading is None: heading = START_HEADING
        
        self.send_command(f"setpos {x:.2f} {y:.2f} {heading:.2f}")
        # Update local mirror immediately
        self.x = x
        self.y = y
        self.heading = heading

    def _process_next_command(self):
        if not self.command_queue:
            self.queue_running = False
            return
            
        cmd = self.command_queue[0]
        # Send GOTO to C
        self.send_command(f"goto {cmd.position[0]:.2f} {cmd.position[1]:.2f}")
        print(f"[NAV] Executing: {cmd.target} -> {cmd.position}")

    # --- Feedback Handling (called from motor_interface) ---
    
    def handle_status_update(self, x, y, heading, state_code):
        """Called when C prints STATUS x y h s"""
        self.x = x
        self.y = y
        self.heading = heading
        
        # Map C state code to string
        states = {0: "IDLE", 1: "TURNING", 2: "DRIVING", 3: "PLANNING"}
        new_state = states.get(state_code, "UNKNOWN")
        
        # Check if we finished a move (transition from NON-IDLE to IDLE)
        if self.state != "IDLE" and new_state == "IDLE" and self.queue_running:
            # Command finished
            if self.command_queue:
                finished = self.command_queue.pop(0)
                print(f"[NAV] Finished: {finished.target}")
                
            # Trigger next
            if self.command_queue:
                # Small delay? C is fast enough.
                self._process_next_command()
            else:
                self.queue_running = False
                print("[NAV] Queue Complete")
                
        self.state = new_state

    def update_encoder_data(self, *args):
        pass # Ignored, C handles this now

    def handle_motor_complete(self, *args):
        pass # Ignored, C handles logic
