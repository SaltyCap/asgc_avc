from flask import Blueprint, render_template, jsonify, request
from .config import Config
from .motor_interface import motor_interface

# We will need a way to access the nav_controller. 
# Since we haven't defined where it lives yet, let's assume it's accessible via motor_interface 
# or we import a global from the package (circular import risk).
# Better: attach it to motor_interface or use a shared state module.
# For now, let's access it through motor_interface.nav_controller

bp = Blueprint('main', __name__)

@bp.route('/')
def index():
    """Serves the main HTML page."""
    return render_template('index.html')

@bp.route('/joystick')
def joystick():
    """Serves the joystick control page."""
    return render_template('joystick.html')

@bp.route('/course')
def course_view():
    """Serves the course visualization page."""
    return render_template('course_view.html')

@bp.route('/api/navigation/status')
def get_navigation_status():
    """Get current navigation status as JSON."""
    if motor_interface.nav_controller:
        return jsonify(motor_interface.nav_controller.get_position())
    else:
        return jsonify({'error': 'Navigation not initialized'}), 503

@bp.route('/api/navigation/goto_center', methods=['POST'])
def api_goto_center():
    """Navigate to center of course."""
    if motor_interface.nav_controller:
        motor_interface.nav_controller.go_to_center()
        return jsonify({'status': 'navigating', 'target': 'center'})
    else:
        return jsonify({'error': 'Navigation not initialized'}), 503

@bp.route('/api/navigation/goto_bucket/<color>', methods=['POST'])
def api_goto_bucket(color):
    """Navigate to specified bucket."""
    if motor_interface.nav_controller:
        bucket_pos = Config.get_bucket_position(color)
        if bucket_pos:
            motor_interface.nav_controller.go_to_bucket(color)
            return jsonify({'status': 'navigating', 'target': color, 'position': bucket_pos})
        else:
            return jsonify({'error': f'Unknown bucket color: {color}'}), 400
    else:
        return jsonify({'error': 'Navigation not initialized'}), 503

@bp.route('/api/course/info')
def get_course_info():
    """Get course layout information."""
    return jsonify({
        'dimensions': {'width': Config.COURSE_WIDTH, 'height': Config.COURSE_HEIGHT},
        'buckets': Config.BUCKETS,
        'center': Config.CENTER,
        'start_position': Config.START_POSITION
    })
