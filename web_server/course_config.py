# Course Configuration for Autonomous Vehicle Challenge

# Performance Settings
USE_COORDINATED_CONTROL = True  # Use coordinated dual-motor with closed-loop PID (BEST)
USE_OPTIMIZED_DAEMON = False    # Use optimized daemon (if not using coordinated)
CACHE_CALCULATIONS = True       # Cache distance/bearing calculations

# Control mode explanation:
# - USE_COORDINATED_CONTROL=True: Best accuracy, closed-loop PID, differential drive
# - USE_OPTIMIZED_DAEMON=True: Fast async control, encoder feedback
# - Both False: Original simple control (for debugging)

# Course dimensions (feet)
COURSE_WIDTH = 35
COURSE_HEIGHT = 35

# Bucket locations (feet) - corners of the 30x30 play area
BUCKETS = {
    'red': (0, 0),
    'yellow': (0, 30),
    'blue': (30, 30),
    'green': (30, 0)
}

# Course center
CENTER = (15, 15)

# Starting position (between yellow and red, 15 feet from center)
START_POSITION = (0, 15)
START_HEADING = 90  # degrees, facing right (toward center)

# Robot physical parameters (adjust based on your robot)
WHEEL_DIAMETER_INCHES = 4.0  # Wheel diameter in inches
WHEELBASE_INCHES = 12.0      # Distance between wheels in inches

# Conversion factors
INCHES_PER_FOOT = 12
COUNTS_PER_REV = 4096  # AS5600 encoder resolution

# Navigation parameters
DEFAULT_SPEED = 30  # Speed percentage for navigation
TURN_SPEED = 25     # Speed for turning
POSITION_TOLERANCE = 1.0  # feet - acceptable position error
HEADING_TOLERANCE = 5.0   # degrees - acceptable heading error

# Calculate derived values
WHEEL_CIRCUMFERENCE_INCHES = 3.14159 * WHEEL_DIAMETER_INCHES
COUNTS_PER_INCH = COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES
COUNTS_PER_FOOT = COUNTS_PER_INCH * INCHES_PER_FOOT

def get_bucket_position(color):
    """Get position of bucket by color name."""
    color = color.lower().strip()
    return BUCKETS.get(color)

def feet_to_counts(feet):
    """Convert feet to encoder counts."""
    return int(feet * COUNTS_PER_FOOT)

def counts_to_feet(counts):
    """Convert encoder counts to feet."""
    return counts / COUNTS_PER_FOOT

def calculate_turn_counts(degrees):
    """Calculate encoder counts needed to turn by specified degrees."""
    # Arc length = (degrees/360) * 2 * pi * radius
    # For differential drive, radius = wheelbase/2
    arc_length_inches = (abs(degrees) / 360.0) * 3.14159 * WHEELBASE_INCHES
    return int(arc_length_inches * COUNTS_PER_INCH)
