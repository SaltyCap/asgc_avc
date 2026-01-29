import os

class Config:
    # Course dimensions (feet) - 30x30 play area
    COURSE_WIDTH = 30
    COURSE_HEIGHT = 30

    # Bucket locations (feet)
    BUCKETS = {
        'red': (0, 0),
        'yellow': (0, 30),
        'blue': (30, 30),
        'green': (30, 0)
    }

    # Course center
    CENTER = (15, 15)

    # Starting position
    START_POSITION = (0, 15)
    START_HEADING = 90

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
    WHEEL_CIRCUMFERENCE_INCHES = 3.14159 * WHEEL_DIAMETER_INCHES
    COUNTS_PER_INCH = COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES
    COUNTS_PER_FOOT = COUNTS_PER_INCH * INCHES_PER_FOOT

    # Paths
    MODEL_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "model")
    
    @classmethod
    def get_motor_control_path(cls):
        return "../c_code/asgc_motor_control"

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
        "start", "reset", "position", 
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
    IMMEDIATE_COMMANDS = {'clear', 'stop', 'reset', 'start'}
