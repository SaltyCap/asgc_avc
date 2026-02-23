import math
import os

try:
    from course_config import (
        BUCKETS,
        CENTER,
        COURSE_HEIGHT,
        COURSE_WIDTH,
        START_HEADING,
        START_POSITION,
    )
except ImportError:
    from web_server.course_config import (
        BUCKETS,
        CENTER,
        COURSE_HEIGHT,
        COURSE_WIDTH,
        START_HEADING,
        START_POSITION,
    )

class Config:
    # Course configuration (imported from course_config.py)
    COURSE_WIDTH = COURSE_WIDTH
    COURSE_HEIGHT = COURSE_HEIGHT
    BUCKETS = BUCKETS
    CENTER = CENTER
    START_POSITION = START_POSITION
    START_HEADING = START_HEADING  # Now linked to course_config.py

    # Runtime/network
    BIND_HOST = os.getenv("BIND_HOST", "0.0.0.0")
    BIND_PORT = int(os.getenv("BIND_PORT", 5001))
    SECRET_KEY = os.getenv("ASGC_SECRET_KEY", os.urandom(32).hex())

    # Web authentication
    AUTH_USERNAME = os.getenv("ASGC_WEB_USERNAME", "admin")
    AUTH_PASSWORD_ENV = "ASGC_WEB_PASSWORD"
    AUTH_PASSWORD_HASH_ENV = "ASGC_WEB_PASSWORD_HASH"
    AUTH_PASSWORD_HASH_FILE = os.getenv(
        "ASGC_WEB_PASSWORD_HASH_FILE",
        os.path.join(os.path.dirname(os.path.dirname(__file__)), ".secrets", "web_password.hash"),
    )
    PASSWORD_HASH_ALGORITHM = "pbkdf2_sha256"
    PASSWORD_HASH_ITERATIONS = max(120000, int(os.getenv("ASGC_PASSWORD_HASH_ITERATIONS", "260000")))
    SESSION_COOKIE_HTTPONLY = True
    SESSION_COOKIE_SAMESITE = "Lax"
    SESSION_COOKIE_SECURE = os.getenv("ASGC_SESSION_COOKIE_SECURE", "").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    ALLOWED_ORIGINS = tuple(
        origin.strip()
        for origin in os.getenv("ASGC_ALLOWED_ORIGINS", "").split(",")
        if origin.strip()
    )

    # Robot physical parameters (synchronized with c_code/include/common.h)
    WHEEL_DIAMETER_INCHES = 5.3
    WHEELBASE_INCHES = 16.0

    # Conversion factors
    INCHES_PER_FOOT = 12
    COUNTS_PER_REV = 4096

    # Navigation parameters
    DEFAULT_SPEED = 30
    TURN_SPEED = 25
    POSITION_TOLERANCE = 1.0
    HEADING_TOLERANCE = 5.0

    # Derived values
    WHEEL_CIRCUMFERENCE_INCHES = math.pi * WHEEL_DIAMETER_INCHES
    COUNTS_PER_INCH = COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES
    COUNTS_PER_FOOT = COUNTS_PER_INCH * INCHES_PER_FOOT

    # Paths
    MODEL_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "model")
    MAX_MOTOR_COMMAND_QUEUE = max(10, int(os.getenv("ASGC_MAX_MOTOR_QUEUE", "200")))
    
    @classmethod
    def get_motor_control_path(cls):
        return os.getenv("ASGC_MOTOR_EXECUTABLE", "asgc_motor_control")

    @classmethod
    def get_bucket_position(cls, color):
        """Get position of bucket by color name."""
        color = color.lower().strip()
        return cls.BUCKETS.get(color)

    @classmethod
    def feet_to_counts(cls, feet):
        """Convert feet to encoder counts."""
        return int(feet * cls.COUNTS_PER_FOOT)

    @classmethod
    def counts_to_feet(cls, counts):
        """Convert encoder counts to feet."""
        return counts / cls.COUNTS_PER_FOOT

    @classmethod
    def calculate_turn_counts(cls, degrees):
        """Calculate encoder counts needed to turn by specified degrees."""
        arc_length_inches = (abs(degrees) / 360.0) * 3.14159 * cls.WHEELBASE_INCHES
        return int(arc_length_inches * cls.COUNTS_PER_INCH)

    # Voice Command Vocabulary (moved from sockets.py)
    # Includes sound-alike words for robust recognition
    VOCABULARY_LIST = [
        "red", "read", "bread", "wed", 
        "blue", "blew", 
        "green", 
        "yellow", "yell", 
        "center", "middle", "centre", 
        "stop", "clear", 
        "forward", "back", "backward", "reverse", 
        "left", "right", 
        "motor", "one", "two", 
        "start", "reset", "position", "calibrate",
        "[unk]"
    ]
    
    # Format required by Vosk: '["word1", "word2", ...]'
    VOCABULARY = str(VOCABULARY_LIST).replace("'", '"')

    # Command Aliases mapping (alias -> canonical command)
    COMMAND_ALIASES = {
        'red': 'red', 'read': 'red', 'bread': 'red', 'wed': 'red',
        'blue': 'blue', 'blew': 'blue',
        'green': 'green',
        'yellow': 'yellow', 'yell': 'yellow',
        'center': 'center', 'middle': 'center', 'centre': 'center',
    }

    # Immediate action commands (not queued)
    IMMEDIATE_COMMANDS = {'clear', 'stop', 'reset', 'start', 'calibrate'}
