import logging
from .config import Config

logger = logging.getLogger(__name__)

class VoiceCommandProcessor:
    """
    Processes voice command strings into actionable navigation commands.

    Bucket-color words (red, blue, green, yellow) are routed to
    BallCycleManager, which handles the full pick-and-place sequence:
      drive to center (roller spinning) → detect ball → pick up →
      drive to bucket → dispense → repeat.

    "start"     — start / resume the ball-cycle queue.
    "stop/clear"— cancel the cycle and clear the queue.
    """
    def __init__(self, nav_controller):
        self.nav_controller = nav_controller

    def process_command(self, command_text):
        """
        Parses a voice command string and executes or queues actions.
        Returns a tuple (queued_count, executed_immediate).
        """
        if not self.nav_controller:
            logger.error("Navigation controller not initialized!")
            return 0, False

        command_text = command_text.strip().lower()
        words = command_text.split()
        
        logger.info(f"[VOICE COMMAND] '{command_text}'")

        queued_count = 0
        executed_immediate = False
        immediate_word = None

        for word in words:
            if word in Config.IMMEDIATE_COMMANDS:
                immediate_word = word
            elif word in Config.COMMAND_ALIASES:
                target = Config.COMMAND_ALIASES[word]
                if self._queue_target_command(target):
                    queued_count += 1

        if immediate_word:
            self._handle_immediate_command(immediate_word)
            executed_immediate = True

        if queued_count > 0:
            logger.info(f"Total queued: {queued_count} commands")
        elif not executed_immediate:
            logger.info(f"No valid commands found in: '{command_text}'")
            
        return queued_count, executed_immediate

    def _handle_immediate_command(self, word):
        """Executes immediate commands like stop, start, clear."""
        logger.info(f"Executing immediate command: '{word}'")

        # Import here to avoid circular imports at module load time
        from .ball_cycle_manager import ball_cycle_manager

        if word == 'clear':
            ball_cycle_manager.clear_queue()
            self.nav_controller.clear_queue()
            logger.info("Queue cleared")
            
        elif word == 'stop':
            ball_cycle_manager.cancel()
            self.nav_controller.clear_queue()
            logger.info("Ball cycle stopped and queue cleared")
            
        elif word == 'start':
            # Start the ball-cycle manager (processes enqueued bucket targets).
            # Clear the nav display queue first — BCM dispatches goto commands
            # directly to the C process and manages sequencing itself, so the
            # nav queue is only used for UI slot display.
            with ball_cycle_manager._queue_lock:
                has_targets = len(ball_cycle_manager._bucket_queue) > 0
            if has_targets:
                ball_cycle_manager.start()
                logger.info("Ball cycle started")
            else:
                logger.warning("start: BallCycleManager queue is empty — say color words first")

        elif word == 'reset':
            ball_cycle_manager.cancel()
            self.nav_controller.reset_position(
                x=Config.START_POSITION[0],
                y=Config.START_POSITION[1],
                heading=Config.START_HEADING
            )
            logger.info(f"Position reset to {Config.START_POSITION} @ {Config.START_HEADING}°")
            
        elif word == 'calibrate':
            ball_cycle_manager.cancel()
            self.nav_controller.calibrate()
            logger.info(f"Gyro calibrated and position reset to {Config.START_POSITION} @ {Config.START_HEADING}°")

    def _queue_target_command(self, target):
        """
        Queues a navigation target.

        Bucket colors (red, blue, green, yellow) go into the BallCycleManager
        for the full pick-and-place sequence.  Say 'start' to begin.

        'center' navigates directly — the car drives to center immediately
        without starting a ball cycle.
        """
        try:
            logger.info(f"Found target: '{target}'")
            if target == 'center':
                # Direct center navigation — does not start a ball cycle
                self.nav_controller.go_to_center()
            else:
                # Bucket target → enqueue into BCM; user must say 'start' to run
                from .ball_cycle_manager import ball_cycle_manager
                ball_cycle_manager.enqueue_bucket(target)

            logger.info(f"Successfully queued: {target}")
            return True
        except Exception as e:
            logger.exception(f"ERROR queueing {target}: {e}")
            return False
