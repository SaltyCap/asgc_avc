import sys
import logging
from web_server.app.config import Config
from web_server.app.voice_command import VoiceCommandProcessor

logging.basicConfig(level=logging.DEBUG)

class DummyNavController:
    def __init__(self):
        self.queue = []
    def go_to_center(self):
        print("NAV: go_to_center")
    def queue_command(self, cmd):
        self.queue.append(cmd)
        print(f"NAV: queued {cmd}")
    def clear_queue(self):
        self.queue = []
        print("NAV: clear_queue")

nav = DummyNavController()
vp = VoiceCommandProcessor(nav)

print(vp.process_command("red start"))
