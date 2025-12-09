"""
Navigation Controller with Coordinated Dual-Motor Control and Closed-Loop Feedback

Features:
- Coordinated differential drive (both motors work together)
- Closed-loop PID control with real-time encoder feedback
- Real-time path correction
- Synchronized motor movements
- Command queue for sequential navigation
- Much higher accuracy than dead reckoning

This controller works with mc2_coordinated.c for best results.
"""
import math
import asyncio
import time
import threading
from queue import Queue
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, Callable, List
from course_config import *

class NavigationState(Enum):
    IDLE = "idle"
    TURNING = "turning"
    DRIVING = "driving"
    WAITING = "waiting"
    COMPLETED = "completed"
    ERROR = "error"

@dataclass
class NavigationCommand:
    """A queued navigation command"""
    command_type: str  # 'bucket', 'center'
    target: str  # color name or 'center'
    position: Tuple[float, float]  # target coordinates

@dataclass
class EncoderData:
    """Real-time encoder data"""
    motor_id: int
    total_counts: int
    current_angle: int
    timestamp: float

class CoordinatedNavigationController:
    def __init__(self, send_command_callback: Callable[[str], None]):
        """
        Initialize coordinated navigation controller.

        Args:
            send_command_callback: Function to send commands to motor controller
        """
        self.send_command = send_command_callback

        # Current position and heading
        self.x = START_POSITION[0]
        self.y = START_POSITION[1]
        self.heading = START_HEADING

        # Encoder data for both motors
        self.left_encoder: Optional[EncoderData] = None
        self.right_encoder: Optional[EncoderData] = None

        # Navigation state
        self.state = NavigationState.IDLE
        self.current_target: Optional[Tuple[float, float]] = None
        self.navigation_mode = 'manual'

        # Command queue
        self.command_queue: List[NavigationCommand] = []
        self.queue_lock = threading.Lock()
        self.queue_running = False
        self.queue_stop_flag = False

        # Movement tracking
        self.movement_start_time = 0
        self.expected_duration = 0

        # Performance tracking
        self.last_update_time = time.time()
        self.update_count = 0

        # Differential drive parameters (from course_config)
        self.wheelbase_feet = WHEELBASE_INCHES / INCHES_PER_FOOT
        self.wheel_circumference_feet = WHEEL_CIRCUMFERENCE_INCHES / INCHES_PER_FOOT

        # Speed multiplier (0.0 to 1.0, controlled by throttle slider)
        self.speed_multiplier = 1.0

    def set_speed_multiplier(self, multiplier: float):
        """
        Set the speed multiplier (0.0 to 1.0).
        This is controlled by the throttle slider on the web interface.
        """
        self.speed_multiplier = max(0.0, min(1.0, multiplier))
        print(f"[NAV] Speed multiplier set to {self.speed_multiplier:.2f} ({int(self.speed_multiplier * 100)}%)")

    def update_encoder_data(self, motor_id: int, total_counts: int, current_angle: int):
        """
        Update encoder data from motor controller.
        Called automatically when ENCODER messages are received.
        """
        data = EncoderData(
            motor_id=motor_id,
            total_counts=total_counts,
            current_angle=current_angle,
            timestamp=time.time()
        )

        if motor_id == 0:
            self.left_encoder = data
        elif motor_id == 1:
            self.right_encoder = data

        # Update odometry using both encoders
        self._update_odometry()

        self.update_count += 1

    def _update_odometry(self):
        """Update position using differential drive kinematics."""
        if not self.left_encoder or not self.right_encoder:
            return

        # Convert encoder counts to distance traveled
        left_dist = counts_to_feet(self.left_encoder.total_counts)
        right_dist = counts_to_feet(self.right_encoder.total_counts)

        # Differential drive kinematics
        center_dist = (left_dist + right_dist) / 2.0

        # Change in heading
        delta_heading_rad = (right_dist - left_dist) / self.wheelbase_feet
        delta_heading_deg = math.degrees(delta_heading_rad)

        # Update heading
        old_heading = self.heading
        self.heading = (self.heading + delta_heading_deg) % 360

        # Update position (use average heading during movement)
        avg_heading_rad = math.radians((old_heading + self.heading) / 2.0)
        self.x += center_dist * math.cos(avg_heading_rad)
        self.y += center_dist * math.sin(avg_heading_rad)

        self.last_update_time = time.time()

    def reset_position(self, x: Optional[float] = None,
                      y: Optional[float] = None,
                      heading: Optional[float] = None):
        """Reset robot position and heading."""
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if heading is not None:
            self.heading = heading

        self.left_encoder = None
        self.right_encoder = None

    def get_position(self):
        """Get current position and heading with encoder status."""
        encoder_age = self._get_encoder_age_ms()
        update_rate = self._get_update_rate()

        # Get queue info
        with self.queue_lock:
            queue_list = [{'target': cmd.target, 'position': cmd.position}
                         for cmd in self.command_queue]

        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'target': self.current_target,
            'mode': self.navigation_mode,
            'state': self.state.value,
            'encoder_age_ms': encoder_age,
            'update_rate_hz': update_rate,
            'left_counts': self.left_encoder.total_counts if self.left_encoder else 0,
            'right_counts': self.right_encoder.total_counts if self.right_encoder else 0,
            'control_type': 'coordinated_closed_loop',
            'queue': queue_list,
            'queue_running': self.queue_running
        }

    def _get_encoder_age_ms(self) -> float:
        """Get age of encoder data in milliseconds."""
        now = time.time()
        ages = []

        if self.left_encoder:
            ages.append((now - self.left_encoder.timestamp) * 1000)
        if self.right_encoder:
            ages.append((now - self.right_encoder.timestamp) * 1000)

        return max(ages) if ages else 999

    def _get_update_rate(self) -> float:
        """Calculate encoder update rate."""
        elapsed = time.time() - self.last_update_time
        if elapsed > 0 and self.update_count > 0:
            return self.update_count / elapsed
        return 0

    def calculate_distance_to_point(self, target_x: float, target_y: float) -> float:
        """Calculate distance to target point."""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.sqrt(dx*dx + dy*dy)

    def calculate_bearing_to_point(self, target_x: float, target_y: float) -> float:
        """Calculate bearing to target point."""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.degrees(math.atan2(dy, dx)) % 360

    def calculate_turn_angle(self, target_heading: float) -> float:
        """Calculate shortest turn angle to reach target heading."""
        diff = target_heading - self.heading
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    async def turn_to_heading_async(self, target_heading: float) -> bool:
        """
        Turn to face target heading using coordinated control.
        Uses differential drive: one wheel forward, other backward.
        """
        turn_angle = self.calculate_turn_angle(target_heading)

        if abs(turn_angle) < HEADING_TOLERANCE:
            return True

        self.state = NavigationState.TURNING
        print(f"[NAV] Turning {turn_angle:.1f}° to heading {target_heading:.1f}°")

        # Calculate arc length for turn
        # Arc = (angle/360) * 2π * (wheelbase/2)
        turn_radius = self.wheelbase_feet / 2.0
        arc_length = abs(turn_angle) * (math.pi / 180.0) * turn_radius

        # Convert to encoder counts
        turn_counts = feet_to_counts(arc_length)

        # Send coordinated turn command using arc with speed multiplier
        # Positive turn_angle = turn right (left +, right -)
        # Negative turn_angle = turn left (left -, right +)
        # Use arc command to apply speed multiplier: arc <left> <right> <speed>
        turn_speed = 0.8 * self.speed_multiplier  # 0.8 is base turn speed factor
        if turn_angle > 0:
            self.send_command(f"arc {turn_counts} {-turn_counts} {turn_speed}")
        else:
            self.send_command(f"arc {-turn_counts} {turn_counts} {turn_speed}")

        # Estimate completion time
        self.movement_start_time = time.time()
        self.expected_duration = abs(turn_angle) / 45.0 * 2.0  # ~2 sec per 45°

        # Wait for completion
        timeout = self.expected_duration + 5.0  # Extra time for safety
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.state == NavigationState.COMPLETED:
                print(f"[NAV] Turn completed: now at {self.heading:.1f}°")
                return True

            # Check if we're close enough (encoder feedback)
            current_turn = self.calculate_turn_angle(target_heading)
            if abs(current_turn) < HEADING_TOLERANCE:
                print(f"[NAV] Turn target reached via feedback")
                return True

            await asyncio.sleep(0.05)

        # Timeout
        print(f"[NAV] Turn timeout - at {self.heading:.1f}°, wanted {target_heading:.1f}°")
        self.state = NavigationState.ERROR
        return False

    async def drive_distance_async(self, distance_feet: float) -> bool:
        """
        Drive straight using coordinated control.
        Both wheels move together with PID control for straight line.
        """
        self.state = NavigationState.DRIVING
        print(f"[NAV] Driving {distance_feet:.2f} feet")

        # Convert distance to encoder counts
        drive_counts = feet_to_counts(distance_feet)

        # Send coordinated drive command using arc with speed multiplier
        # Use arc command to apply speed multiplier: arc <left> <right> <speed>
        # For straight driving, both wheels get the same counts
        drive_speed = 1.0 * self.speed_multiplier  # 1.0 is base drive speed factor
        self.send_command(f"arc {drive_counts} {drive_counts} {drive_speed}")

        # Estimate completion time
        self.movement_start_time = time.time()
        self.expected_duration = abs(distance_feet) / 2.0 * 1.5  # ~1.5 sec per 2 feet

        # Wait for completion
        timeout = self.expected_duration + 10.0
        start_time = time.time()

        start_x, start_y = self.x, self.y

        while time.time() - start_time < timeout:
            if self.state == NavigationState.COMPLETED:
                actual_dist = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
                print(f"[NAV] Drive completed: traveled {actual_dist:.2f} ft")
                return True

            # Check if we've traveled the distance (encoder feedback)
            if self.left_encoder and self.right_encoder:
                avg_counts = (abs(self.left_encoder.total_counts) +
                            abs(self.right_encoder.total_counts)) / 2.0
                traveled = counts_to_feet(avg_counts)

                if traveled >= abs(distance_feet) * 0.95:  # 95% threshold
                    print(f"[NAV] Drive target reached via feedback: {traveled:.2f} ft")
                    return True

            await asyncio.sleep(0.05)

        # Timeout
        print(f"[NAV] Drive timeout")
        self.state = NavigationState.ERROR
        return False

    async def navigate_to_point_async(self, target_x: float, target_y: float) -> bool:
        """
        Navigate to target point using coordinated control.
        1. Turn to face target
        2. Drive straight to target
        3. Closed-loop control provides accuracy
        """
        self.current_target = (target_x, target_y)

        distance = self.calculate_distance_to_point(target_x, target_y)
        bearing = self.calculate_bearing_to_point(target_x, target_y)

        print(f"\n[NAV] ═══ NAVIGATING TO ({target_x}, {target_y}) ═══")
        print(f"[NAV] Current: ({self.x:.2f}, {self.y:.2f}) @ {self.heading:.1f}°")
        print(f"[NAV] Distance: {distance:.2f} ft, Bearing: {bearing:.1f}°")

        if distance < POSITION_TOLERANCE:
            print("[NAV] Already at target!")
            self.state = NavigationState.COMPLETED
            return True

        # Step 1: Turn to face target
        print(f"[NAV] Step 1: Turn to bearing {bearing:.1f}°")
        success = await self.turn_to_heading_async(bearing)
        if not success:
            print("[NAV] ✗ Turn failed")
            return False

        await asyncio.sleep(0.5)  # Brief pause between movements

        # Step 2: Drive to target
        print(f"[NAV] Step 2: Drive {distance:.2f} feet")
        success = await self.drive_distance_async(distance)
        if not success:
            print("[NAV] ✗ Drive failed")
            return False

        # Verify arrival
        final_distance = self.calculate_distance_to_point(target_x, target_y)
        print(f"[NAV] ✓ Navigation complete! Final distance: {final_distance:.2f} ft")

        self.state = NavigationState.COMPLETED
        return True

    def _run_async_in_thread(self, coro):
        """Run an async coroutine in a new thread with its own event loop."""
        def run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(coro)
            finally:
                loop.close()

        thread = threading.Thread(target=run, daemon=True)
        thread.start()

    # ========== QUEUE MANAGEMENT ==========

    def queue_command(self, command: NavigationCommand):
        """Add a command to the queue (max 5 commands). Does NOT auto-start."""
        with self.queue_lock:
            # Limit queue to 5 commands
            if len(self.command_queue) >= 5:
                print(f"[QUEUE] Full! Ignoring: {command.target}")
                return

            self.command_queue.append(command)
            print(f"[QUEUE] Added: {command.target} -> Queue size: {len(self.command_queue)}")
            self._print_queue()
        # Queue does NOT auto-start - user must say "start" to begin execution

    def start_queue(self):
        """Start executing the queue. Called when user says 'start'."""
        with self.queue_lock:
            if self.queue_running:
                print("[QUEUE] Already running")
                return
            if not self.command_queue:
                print("[QUEUE] Nothing to start - queue is empty")
                return
            print(f"[QUEUE] Starting execution with {len(self.command_queue)} commands")
        self._start_queue_processor()

    def clear_queue(self):
        """Clear all pending commands from queue."""
        with self.queue_lock:
            count = len(self.command_queue)
            self.command_queue.clear()
            print(f"[QUEUE] Cleared {count} commands")
        self.queue_stop_flag = True
        self.stop()

    def get_queue(self) -> List[dict]:
        """Get current queue as list of dicts."""
        with self.queue_lock:
            return [{'target': cmd.target, 'position': cmd.position}
                   for cmd in self.command_queue]

    def _print_queue(self):
        """Print current queue status. Must be called with queue_lock held."""
        # Note: No lock here - caller must hold queue_lock
        if not self.command_queue:
            print("[QUEUE] Empty")
            return
        print("[QUEUE] Pending commands:")
        for i, cmd in enumerate(self.command_queue):
            print(f"  {i+1}. {cmd.target} @ {cmd.position}")

    def _start_queue_processor(self):
        """Start the queue processor in a background thread."""
        with self.queue_lock:
            if self.queue_running:
                return
            # Set running flag BEFORE starting thread to prevent race condition
            self.queue_running = True
            self.queue_stop_flag = False
        self._run_async_in_thread(self._process_queue())

    async def _process_queue(self):
        """Process commands from the queue sequentially."""
        # Note: queue_running is already set True by _start_queue_processor()
        print("[QUEUE] Processor started")

        try:
            while True:
                # Get next command
                cmd = None
                with self.queue_lock:
                    if self.command_queue and not self.queue_stop_flag:
                        cmd = self.command_queue.pop(0)

                if cmd is None or self.queue_stop_flag:
                    break

                print(f"\n[QUEUE] ═══ Processing: {cmd.target} ═══")
                self.navigation_mode = f'goto_{cmd.command_type}'
                self.current_target = cmd.position

                # Navigate to the target
                success = await self.navigate_to_point_async(cmd.position[0], cmd.position[1])

                if self.queue_stop_flag:
                    print("[QUEUE] Stopped by user")
                    break

                if success:
                    print(f"[QUEUE] ✓ Completed: {cmd.target}")
                else:
                    print(f"[QUEUE] ✗ Failed: {cmd.target}")

                # Brief pause between commands
                await asyncio.sleep(0.5)

        finally:
            self.queue_running = False
            self.navigation_mode = 'manual'
            self.current_target = None
            print("[QUEUE] Processor stopped")

    # ========== NAVIGATION COMMANDS ==========

    def go_to_center(self, callback: Optional[Callable] = None):
        """Add center navigation to queue."""
        cmd = NavigationCommand(
            command_type='center',
            target='CENTER',
            position=CENTER
        )
        self.queue_command(cmd)

    def go_to_bucket(self, color: str, callback: Optional[Callable] = None):
        """Add bucket navigation to queue."""
        bucket_pos = get_bucket_position(color)
        if bucket_pos is None:
            print(f"[NAV] ✗ Unknown bucket color: {color}")
            return False

        cmd = NavigationCommand(
            command_type='bucket',
            target=color.upper(),
            position=bucket_pos
        )
        self.queue_command(cmd)
        return True

    def stop(self):
        """Emergency stop - halts all motors immediately."""
        print("[NAV] ⚠ EMERGENCY STOP")
        self.state = NavigationState.IDLE
        self.navigation_mode = 'manual'
        self.send_command("stopall")

    def handle_coordinated_complete(self):
        """
        Called when motor controller reports COORDINATED_COMPLETE.
        Signals that the current movement is done.
        """
        if self.state in [NavigationState.TURNING, NavigationState.DRIVING]:
            self.state = NavigationState.COMPLETED

    def get_status_string(self) -> str:
        """Get detailed status string."""
        status = f"╔════════════════════════════════════════╗\n"
        status += f"║  COORDINATED NAVIGATION STATUS         ║\n"
        status += f"╚════════════════════════════════════════╝\n"
        status += f"Position: ({self.x:.2f}, {self.y:.2f}) ft\n"
        status += f"Heading: {self.heading:.1f}°\n"
        status += f"State: {self.state.value}\n"
        status += f"Mode: {self.navigation_mode}\n"
        status += f"Control: Closed-loop PID + Differential Drive\n"
        status += f"Update Rate: {self._get_update_rate():.1f} Hz\n"
        status += f"Encoder Age: {self._get_encoder_age_ms():.1f} ms\n"

        if self.left_encoder:
            status += f"Left Wheel: {self.left_encoder.total_counts} counts\n"
        if self.right_encoder:
            status += f"Right Wheel: {self.right_encoder.total_counts} counts\n"

        if self.current_target:
            dist = self.calculate_distance_to_point(self.current_target[0],
                                                     self.current_target[1])
            status += f"Target: {self.current_target}\n"
            status += f"Distance to Target: {dist:.2f} ft\n"

        return status
