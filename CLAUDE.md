# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an autonomous vehicle control system for the Arkansas Space Grant Commission (ASGC) competition. The system combines a C-based motor controller with a Python Flask web server to provide voice-controlled navigation on a 30x30 foot course.

**Architecture**: Two-process design
- **C Program** (`c_code/mc2_coordinated`): Low-level motor control with PID, I2C encoder feedback, PWM output
- **Python Web Server** (`web_server/`): Flask application with voice recognition (Vosk), navigation logic, WebSocket communication

The processes communicate via stdin/stdout pipes, with the Python server sending commands and receiving encoder feedback from the C program.

## Common Commands

### Building and Running

```bash
# Build C motor controller
cd c_code && make

# Clean build artifacts
cd c_code && make clean

# Start entire system (recommended)
./start_all.sh
# This script:
# 1. Checks/starts pigpiod daemon
# 2. Sets I2C speed to 400kHz
# 3. Builds C code
# 4. Creates/activates Python venv
# 5. Generates SSL certificates (if missing)
# 6. Downloads Vosk model (if missing, ~600MB)
# 7. Starts web server (which launches motor controller)

# Start web server only (if C code already built)
cd web_server
source venv/bin/activate
python web_server.py
```

### Network Modes

The Pi can operate in two network modes:

**WiFi Mode** (for development with SSH access):
- Connects to existing WiFi network
- Access web server at Pi's WiFi IP address (check with `hostname -I`)
- Example: `https://192.168.0.82:5000`

**Hotspot Mode** (for field operation without WiFi):
```bash
# Enable hotspot (WARNING: will disconnect SSH if connected via WiFi)
./enable_hotspot.sh

# This creates WiFi network:
#   SSID: ASGC_Robot
#   Password: robotcontrol
#   Pi IP: 10.42.0.1

# Connect your phone/laptop to ASGC_Robot network
# Access: https://10.42.0.1:5000

# To switch back to WiFi:
./disable_hotspot.sh
```

See `HOTSPOT_SETUP.md` for detailed troubleshooting and configuration options.

### Testing

There are no automated test files in this repository. Testing is done manually through the web interfaces.

## System Architecture

### Inter-Process Communication

**Python → C**: Commands sent via stdin
- Format: Text commands like `drive 5000`, `turn 2000`, `stopall`
- See `web_server/app/motor_interface.py` for command sending
- Commands are queued and sent by `MotorInterface._send_commands()` thread

**C → Python**: Feedback via stdout
- `ENCODER <motor_id> <total_counts> <current_angle>` - Real-time encoder updates
- `COORDINATED_COMPLETE` - Movement finished
- `COMPLETE <motor_id> <final_counts>` - Individual motor complete
- Parsed by `MotorInterface._handle_motor_feedback()` in `web_server/app/motor_interface.py`

### Control Flow

1. **Voice Command**: User says "red" → Vosk recognizes in `web_server/app/sockets.py`
2. **Queue Command**: Added to navigation queue in `navigation_coordinated.py`
3. **Navigation**: Queue processor calculates path, converts to encoder counts
4. **Motor Commands**: Sent to C program via `motor_interface.send_command()`
5. **Execution**: C program runs PID control loop, sends encoder feedback
6. **Odometry Update**: Python updates robot position from encoder data
7. **Completion**: C signals `COORDINATED_COMPLETE`, next command executes

### Key Components

**Navigation Controller** (`web_server/navigation_coordinated.py`)
- `CoordinatedNavigationController`: Main navigation logic
- Command queue system (max 5 commands)
- Differential drive kinematics for odometry
- Closed-loop path planning with turn-then-drive approach
- State machine: IDLE → TURNING → DRIVING → COMPLETED

**Motor Controller** (`c_code/src/main.c`)
- Coordinated dual-motor control thread (1kHz loop)
- PID controllers for each motor (`c_code/src/pid.c`)
- AS5600 I2C magnetic encoder reading (`c_code/src/i2c.c`)
- PWM output via pigpio library (`c_code/src/motor.c`)

**Voice Recognition** (`web_server/app/sockets.py`)
- Vosk offline speech recognition
- Constrained vocabulary for faster recognition
- WebSocket `/audio` endpoint for streaming audio
- Processes commands in `process_voice_command()`

**Motor Interface** (`web_server/app/motor_interface.py`)
- Subprocess management for C program
- Background threads for bidirectional communication
- Encoder feedback routing to navigation controller

## Configuration

All tunable parameters are in `web_server/course_config.py`:

**Critical calibration values** (adjust for your robot):
- `WHEEL_DIAMETER_INCHES`: Affects distance accuracy
- `WHEELBASE_INCHES`: Affects turn accuracy
- `START_POSITION` and `START_HEADING`: Robot's initial pose

**Navigation parameters**:
- `DEFAULT_SPEED`: Driving speed (0-100%)
- `TURN_SPEED`: Turning speed (0-100%)
- `POSITION_TOLERANCE`: When target is "reached" (feet)
- `HEADING_TOLERANCE`: Acceptable heading error (degrees)

**Course layout**:
- `BUCKETS`: Corner positions (red, yellow, blue, green)
- `CENTER`: Center point (15, 15)

## Important Implementation Details

### Coordinate System
- Origin (0, 0) at red bucket (bottom-left)
- X increases to the right (east)
- Y increases upward (north)
- Heading: 0° = north, 90° = east, 180° = south, 270° = west
- Robot starts at (0, 15) facing 90° (east toward center)

### Motor Control Strategy
The C program uses **coordinated differential drive**:
- `drive <counts>`: Both motors forward/backward with PID to keep straight
- `turn <counts>`: Differential turn (one wheel forward, one backward)
- Closed-loop PID continuously corrects for motor differences
- Encoder feedback enables real-time position tracking

### Navigation Strategy
Two-step approach for each waypoint:
1. **Turn**: Rotate to face target bearing
2. **Drive**: Move straight to target distance

This is simpler than continuous path following but requires accurate turns. The PID controller compensates for mechanical variations.

### Voice Command Queue
- Commands are **queued, not executed immediately**
- Maximum 5 commands in queue
- FIFO execution (first in, first out)
- Say "clear" to empty queue
- Say "stop" to halt current movement and clear queue
- Queue status visible in web UI as 5 color-coded slots

## File Organization

```
c_code/
  src/          - C source files
  include/      - C header files
  Makefile      - Build configuration
  mc2_coordinated - Compiled binary (gitignored)

web_server/
  app/
    __init__.py          - Flask app factory, initializes nav controller
    config.py            - Flask config (SSL, paths)
    motor_interface.py   - C program subprocess management
    routes.py            - REST API endpoints
    sockets.py           - WebSocket handlers + voice recognition
  static/       - CSS, JavaScript
  templates/    - HTML (index.html, joystick.html, course_view.html)
  navigation_coordinated.py - Navigation logic with queue
  course_config.py       - Robot/course parameters
  web_server.py          - Entry point
  venv/         - Python virtual environment (create manually)
```

## SSL Certificates

The web server requires HTTPS for browser microphone access. Place `cert.pem` and `key.pem` in `web_server/` directory:

```bash
cd web_server
openssl req -x509 -newkey rsa:4096 -nodes -out cert.pem -keyout key.pem -days 365
```

Browser will show security warning (self-signed cert) - accept to continue. The `start_all.sh` script automatically generates these if they don't exist.

## Hardware Dependencies

- **Raspberry Pi 5** (or compatible with 40-pin GPIO)
- **pigpio**: Must be running for PWM (`sudo pigpiod`)
  - C program will fail if pigpio daemon not running
  - PWM pins: GPIO 12 (left motor), GPIO 13 (right motor)
- **I2C**: Enabled in `/boot/config.txt` with `dtparam=i2c_arm=on,i2c_arm_baudrate=400000`
  - AS5600 encoder on default I2C bus (usually `/dev/i2c-1`)
- **Vosk model**: Automatically downloaded by `start_all.sh` if not present (~600MB)
  - Model: `vosk-model-small-en-us-0.15` (optimized for ARM/Raspberry Pi)
  - Manual download: https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  - Extract to `web_server/model/` directory

## Python Virtual Environment

The project requires a Python virtual environment in `web_server/venv/`. If not present:

```bash
cd web_server
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Dependencies are listed in `requirements.txt` (Flask, Flask-Sock, Vosk).

## Common Issues

**"Virtual environment 'venv' not found"**: Create venv as shown above

**"Motor control program not found"**: Run `make` in `c_code/` directory

**Segfault on startup or "not a Raspberry Pi" warnings**: Usually Vosk model incompatibility. The `start_all.sh` script automatically downloads the ARM-optimized model (`vosk-model-small-en-us-0.15`, ~600MB). If you manually downloaded a different model, delete it and let the script download the correct one.

**"pigpio daemon not running"**: The C program requires pigpiod for PWM control. Run `sudo pigpiod` or let `start_all.sh` start it automatically.

**Motors don't respond**:
- Check `sudo` permissions (C program needs root for GPIO)
- Verify pigpio daemon is running: `sudo pigpiod`
- Check I2C with `i2cdetect -y 1` (should see device at 0x36)

**Voice recognition doesn't work**:
- Browser needs HTTPS (check cert.pem/key.pem exist)
- Accept browser security warning
- Grant microphone permissions

**Robot drives wrong direction**: Verify `START_HEADING` in `course_config.py` matches physical heading

**Position drifts over time**: Calibrate `WHEEL_DIAMETER_INCHES` and `WHEELBASE_INCHES` by measuring actual distance/rotation
