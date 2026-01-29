from .config import Config

class VoiceCommandProcessor:
    """
    Processes voice command strings into actionable navigation commands.
    """
    def __init__(self, nav_controller):
        self.nav_controller = nav_controller

    def process_command(self, command_text):
        """
        Parses a voice command string and executes or queues actions.
        Returns a tuple (queued_count, executed_immediate).
        """
        if not self.nav_controller:
            print("[VOICE] ERROR: Navigation controller not initialized!")
            return 0, False

        command_text = command_text.strip().lower()
        words = command_text.split()
        
        print(f"[VOICE COMMAND] '{command_text}'")

        queued_count = 0
        executed_immediate = False

        for word in words:
            # Check for immediate commands first
            if word in Config.IMMEDIATE_COMMANDS:
                self._handle_immediate_command(word)
                executed_immediate = True
                return queued_count, executed_immediate # Stop processing after immediate command

            # Check if word is a valid target alias
            elif word in Config.COMMAND_ALIASES:
                target = Config.COMMAND_ALIASES[word]
                if self._queue_target_command(target):
                    queued_count += 1
            else:
                # Ignore unknown words (or could log them)
                pass

        if queued_count > 0:
            print(f"[VOICE] Total queued: {queued_count} commands")
        elif not executed_immediate:
            print(f"[VOICE] No valid commands found in: '{command_text}'")
            
        return queued_count, executed_immediate

    def _handle_immediate_command(self, word):
        """Executes immediate commands like stop, start, clear."""
        print(f"[VOICE] Executing immediate command: '{word}'")
        
        if word == 'clear':
            self.nav_controller.clear_queue()
            print("[VOICE] Queue cleared")
            
        elif word == 'stop':
            self.nav_controller.clear_queue()
            print("[VOICE] Queue stopped and cleared")
            
        elif word == 'start':
            self.nav_controller.start_queue()
            print("[VOICE] Queue started")
            
        elif word == 'reset':
            self.nav_controller.reset_position(
                x=Config.START_POSITION[0],
                y=Config.START_POSITION[1],
                heading=Config.START_HEADING
            )
            print(f"[VOICE] Position reset to {Config.START_POSITION} @ {Config.START_HEADING}°")

    def _queue_target_command(self, target):
        """Queues a navigation command for a specific target."""
        try:
            print(f"[VOICE] Found target: '{target}'", flush=True)
            if target == 'center':
                self.nav_controller.go_to_center()
            else:
                # Assume it's a bucket color
                self.nav_controller.go_to_bucket(target)
            
            print(f"[VOICE] Successfully queued: {target}", flush=True)
            return True
        except Exception as e:
            import traceback
            print(f"[VOICE] ERROR queueing {target}: {e}", flush=True)
            traceback.print_exc()
            return False
