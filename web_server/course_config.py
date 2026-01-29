"""
Course Configuration
Shared configuration for navigation and field geometry.
Mirrors values from app/config.py for direct import usage.
"""

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

def get_bucket_position(color):
    """Get position of bucket by color name."""
    color = color.lower().strip()
    return BUCKETS.get(color)
