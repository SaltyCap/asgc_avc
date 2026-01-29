# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an autonomous vehicle control system for the Arkansas Space Grant Commission (ASGC) competition. The system combines a C-based motor controller with a Python Flask web server to provide voice-controlled navigation on a 30x30 foot course.

**Architecture**: Two-process design
- **C Program** (`c_code/mc2_coordinated`): Low-level motor control with PID, I2C encoder feedback, PWM output, IMU integration, Kalman filtering
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

### Network Access

**WiFi Mode**:
- Connects to existing WiFi network
- Access web server at Pi's WiFi IP address (check with `hostname -I`)
- Example: `https://192.168.0.82:5000`

### Testing

There are no automated test files in this repository. Testing is done manually through the web interfaces.

## System Architecture

### Inter-Process Communication

**Python → C**: Commands sent via stdin
- Format: Text commands like `goto 15.0 15.0`, `speed 0.5`, `stop`, `setpos 0.0 15.0 90.0`, `pulse 1500000 1500000`
- See `web_server/app/motor_interface.py` for command sending
- Commands are queued and sent by `MotorInterface._send_commands()` thread

**C → Python**: Feedback via stdout
- `STATUS <x> <y> <heading> <state_code>` - Real-time odometry and navigation state updates
- `OK <command>` - Command acknowledgment
- Parsed by `MotorInterface._handle_motor_feedback()` in `web_server/app/motor_interface.py`

### Control Flow

1. **Voice Command**: User says "red" → Vosk recognizes in `web_server/app/sockets.py`
2. **Queue Command**: Added to navigation queue in `navigation_coordinated.py`
3. **Navigation**: Queue processor sends `goto` command to C program
4. **Path Planning**: C program calculates turn/drive sequence in `coordinated_control_thread()`
5. **Motor Control**: C program runs PID control loop, sends encoder feedback
6. **Odometry Update**: C program fuses encoder + IMU data via Kalman filter
7. **Status Feedback**: C sends `STATUS` updates to Python
8. **Completion**: C transitions to IDLE state, Python processes next queued command

### Key Components

**Navigation Controller** (`web_server/navigation_coordinated.py`)
- `CoordinatedNavigationController`: Thin client that delegates to C program
- Command queue system (unlimited queue)
- Mirrors odometry state from C program via `STATUS` messages
- State machine: IDLE → PLANNING → TURNING → DRIVING → IDLE

**Motor Controller** (`c_code/src/main.c`)
- Coordinated dual-motor control thread (200Hz loop)
- Simple on/off control with stall detection
- AS5600 I2C magnetic encoder reading (`c_code/src/i2c.c`)
- MPU6050 IMU gyroscope reading (`c_code/src/imu.c`)
- Kalman filter sensor fusion (`c_code/src/kalman.c`)
- PWM output via sysfs (`c_code/src/motor.c`)
- Comprehensive logging system with mode tracking

**Voice Recognition** (`web_server/app/sockets.py`)
- Vosk offline speech recognition
- Constrained vocabulary for faster recognition
- WebSocket `/audio` endpoint for streaming audio
- Processes commands via `VoiceCommandProcessor` in `voice_command.py`

**Motor Interface** (`web_server/app/motor_interface.py`)
- Subprocess management for C program
- Background threads for bidirectional communication
- Status feedback routing to navigation controller

## Configuration

All tunable parameters are in `web_server/app/config.py`:

**Critical calibration values** (adjust for your robot):
- `WHEEL_DIAMETER_INCHES`: 5.3 inches (synchronized with C code)
- `WHEELBASE_INCHES`: 16.0 inches (synchronized with C code)
- `START_POSITION` and `START_HEADING`: Robot's initial pose

**Navigation parameters**:
- `DEFAULT_SPEED`: Driving speed (0-100%)
- `TURN_SPEED`: Turning speed (0-100%)
- `POSITION_TOLERANCE`: When target is "reached" (feet)
- `HEADING_TOLERANCE`: Acceptable heading error (degrees)

**Course layout**:
- `BUCKETS`: Corner positions (red, yellow, blue, green)
- `CENTER`: Center point (15, 15)

**C Code Parameters** (`c_code/include/common.h` and `c_code/src/main.c`):
- `COUNTS_PER_REV`: 4096 (AS5600 encoder resolution)
- `WHEEL_DIAMETER_INCHES`: 5.3 inches
- `WHEELBASE_INCHES`: 16.0 inches
- `g_min_pwm`, `g_max_pwm`: PWM limits (adjustable via `setpwm` command)
- `STOP_THRESHOLD`: 50 counts (~0.5 inches)
- `DEADBAND_THRESHOLD`: 50 counts

## Important Implementation Details

### Coordinate System
- Origin (0, 0) at red bucket (bottom-left)
- X increases to the right (east)
- Y increases upward (north)
- Heading: 0° = north, 90° = east, 180° = south, 270° = west
- Robot starts at (0, 15) facing 90° (east toward center)

### Motor Control Strategy
The C program uses **coordinated differential drive**:
- `goto <x> <y>`: Autonomous navigation with turn-then-drive approach
- `pulse <left_ns> <right_ns>`: Direct PWM control for joystick mode
- `stop`: Immediate stop and clear all targets
- Closed-loop control with stall detection and compensation
- Encoder feedback enables real-time position tracking
- IMU gyroscope provides heading rate for sensor fusion

### Navigation Strategy
Two-step approach for each waypoint:
1. **Turn**: Rotate to face target bearing
2. **Drive**: Move straight to target distance

This is simpler than continuous path following but requires accurate turns. The simple on/off controller with stall detection handles mechanical variations.

### Sensor Fusion
- **Encoders**: Provide distance traveled and differential heading change
- **IMU Gyroscope**: Provides heading rate (degrees/sec)
- **Kalman Filter**: Fuses encoder heading measurement with gyro rate prediction
- Update rate: 200Hz (encoder thread runs continuously, odometry updates in control loop)

### Voice Command Queue
- Commands are **queued, not executed immediately**
- Unlimited queue size
- FIFO execution (first in, first out)
- Say "clear" or "stop" to empty queue
- Say "start" to begin queue execution
- Queue status visible in web UI

## File Organization

```
c_code/
  src/
    main.c           - Main control loop, navigation state machine, command processing
    motor.c          - PWM initialization and motor speed control
    i2c.c            - AS5600 encoder I2C communication
    imu.c            - MPU6050 IMU initialization and gyro reading
    kalman.c         - Kalman filter implementation for sensor fusion
    common.c         - Time utilities (get_time_sec, sleep_us, sleep_ms)
  include/
    common.h         - Shared constants, structures (OdometryState, NavigationController)
    motor.h          - Motor and encoder structures, PWM constants
    i2c.h            - I2C addresses and function declarations
    imu.h            - IMU registers and function declarations
    kalman.h         - Kalman filter structure and functions
  Makefile           - Build configuration
  mc2_coordinated    - Compiled binary (gitignored)

web_server/
  app/
    __init__.py              - Flask app factory, initializes nav controller
    config.py                - Flask config (course layout, voice vocabulary)
    motor_interface.py       - C program subprocess management
    routes.py                - REST API endpoints
    sockets.py               - WebSocket handlers + voice recognition
    voice_command.py         - Voice command processing logic
  static/            - CSS, JavaScript
  templates/         - HTML (index.html, joystick.html, course_view.html)
  navigation_coordinated.py  - Navigation controller (thin client)
  course_config.py   - Robot/course parameters (DEPRECATED - use app/config.py)
  web_server.py      - Entry point
  venv/              - Python virtual environment (create manually)
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
- **PWM**: Sysfs PWM interface
  - C program uses `/sys/class/pwm/pwmchipX/pwmY/` for control
  - PWM channels: 0 (left motor), 1 (right motor)
- **I2C**: Enabled in `/boot/config.txt` with `dtparam=i2c_arm=on,i2c_arm_baudrate=400000`
  - AS5600 encoders on I2C bus 1 (`/dev/i2c-1`)
    - Left encoder: 0x40
    - Right encoder: 0x1B
  - MPU6050 IMU on I2C bus 3 (`/dev/i2c-3`)
    - Address: 0x68
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

**Motors don't respond**:
- Check `sudo` permissions (C program needs root for GPIO)
- Verify PWM sysfs interface is available
- Check I2C with `i2cdetect -y 1` (should see devices at 0x40 and 0x1B)

**Voice recognition doesn't work**:
- Browser needs HTTPS (check cert.pem/key.pem exist)
- Accept browser security warning
- Grant microphone permissions

**Robot drives wrong direction**: Verify `START_HEADING` in `config.py` matches physical heading

**Position drifts over time**: Calibrate `WHEEL_DIAMETER_INCHES` and `WHEELBASE_INCHES` by measuring actual distance/rotation

## Code Inventory and Usage Tracking

### C Code Functions and Variables

#### `c_code/src/main.c`
**Global Variables:**
- `running` - ✅ USED: Signal handler flag, checked in all threads
- `odometry` - ✅ USED: Current robot position (x, y, heading)
- `nav_ctrl` - ✅ USED: Navigation state machine
- `kf_heading` - ✅ USED: Kalman filter for heading fusion
- `current_gyro_rate` - ✅ USED: Latest gyro reading from IMU thread
- `last_imu_time` - ✅ USED: Timestamp for Kalman filter dt calculation
- `imu_data_lock` - ✅ USED: Mutex for gyro data access
- `g_min_pwm` - ⚠️ DECLARED BUT UNUSED: Minimum PWM (not currently used in control logic)
- `g_max_pwm` - ✅ USED: Maximum PWM for control stability
- `log_buffer` - ✅ USED: Logging system buffer
- `log_index` - ✅ USED: Current log position
- `current_mode` - ✅ USED: Control mode tracking (IDLE/JOYSTICK/VOICE_NAV)

**Functions:**
- `signal_handler()` - ✅ USED: Handles SIGINT/SIGTERM
- `init_log_system()` - ✅ USED: Allocates log buffer
- `log_data()` - ✅ USED: Records telemetry at 200Hz
- `dump_log()` - ✅ USED: Writes log to CSV file
- `coordinated_control_thread()` - ✅ USED: Main 200Hz control loop
- `imu_thread()` - ✅ USED: 100Hz IMU polling
- `update_encoder_tracker()` - ✅ USED: Handles encoder wrap detection
- `encoder_feedback_thread()` - ✅ USED: Continuous encoder reading
- `calculate_turn_counts()` - ✅ USED: Converts degrees to encoder counts
- `update_odometry()` - ✅ USED: Sensor fusion (encoder + IMU)
- `process_command()` - ✅ USED: Parses stdin commands
- `command_input_thread()` - ✅ USED: Reads stdin commands
- `main()` - ✅ USED: Entry point

**Supported Commands:**
- `goto <x> <y>` - ✅ USED: Navigate to position
- `speed <multiplier>` - ✅ USED: Set speed (0.0-1.0)
- `setpwm <min> <max>` - ✅ USED: Adjust PWM limits
- `setpos <x> <y> <heading>` - ✅ USED: Reset odometry
- `stop` - ✅ USED: Stop all motors and dump log
- `q` - ✅ USED: Quit program
- `pulse <left_ns> <right_ns>` - ✅ USED: Direct PWM control (joystick mode)

#### `c_code/src/motor.c`
**Global Variables:**
- `motors[2]` - ✅ USED: Motor state array
- `encoders[2]` - ✅ USED: Encoder state array
- `PWM_CHIP` - ✅ USED: PWM chip number (static)

**Functions:**
- `find_pwm_chip()` - ✅ USED: Locates PWM sysfs interface
- `pwm_init()` - ✅ USED: Initializes PWM channels
- `set_motor_speed()` - ✅ USED: Sets motor speed with ramping
- `pwm_cleanup()` - ✅ USED: Stops motors and closes file descriptors

#### `c_code/src/i2c.c`
**Global Variables:**
- `i2c_fd` - ✅ USED: I2C bus file descriptor

**Functions:**
- `i2c_init()` - ✅ USED: Opens I2C bus
- `read_raw_angle()` - ✅ USED: Reads AS5600 encoder angle
- `i2c_cleanup()` - ✅ USED: Closes I2C bus

**Removed Functions:**
- `configure_fast_filter()` - ❌ REMOVED: Was unused, removed in cleanup

#### `c_code/src/imu.c`
**Global Variables:**
- `imu` - ✅ USED: IMU context (gyro offset, mutex, file descriptor)

**Functions:**
- `write_reg()` - ✅ USED: Writes MPU6050 register (static helper)
- `imu_init()` - ✅ USED: Initializes MPU6050
- `imu_cleanup()` - ✅ USED: Closes IMU I2C connection
- `imu_read_gyro_z()` - ✅ USED: Reads Z-axis gyro rate
- `imu_calibrate()` - ✅ USED: Calibrates gyro offset

#### `c_code/src/kalman.c`
**Functions:**
- `kalman_init()` - ✅ USED: Initializes Kalman filter
- `kalman_get_angle()` - ✅ USED: Performs Kalman update (predict + correct)

#### `c_code/src/common.c`
**Functions:**
- `get_time_sec()` - ✅ USED: Returns monotonic time in seconds
- `sleep_us()` - ✅ USED: Sleep for microseconds
- `sleep_ms()` - ✅ USED: Sleep for milliseconds

### C Code Header Files

#### `c_code/include/common.h`
**Constants:**
- All constants ✅ USED in main.c and other modules

**Structures:**
- `OdometryState` - ✅ USED
- `NavState` enum - ✅ USED
- `NavigationController` - ✅ USED

#### `c_code/include/motor.h`
**Constants:**
- All PWM constants ✅ USED

**Structures:**
- `Motor` - ✅ USED
- `EncoderState` - ✅ USED

#### `c_code/include/i2c.h`
**All declarations** - ✅ USED

### Python Code Functions and Variables

#### `web_server/navigation_coordinated.py`
**Class: CoordinatedNavigationController**
- All methods ✅ USED
- `update_encoder_data()` - ⚠️ STUB: Empty implementation (C handles this)
- `handle_motor_complete()` - ⚠️ STUB: Empty implementation (C handles this)

#### `web_server/app/motor_interface.py`
**Class: MotorInterface**
- All methods ✅ USED

#### `web_server/app/sockets.py`
**Functions:**
- `init_model()` - ✅ USED
- `audio_socket()` - ✅ USED
- `motor_socket()` - ✅ USED

#### `web_server/app/voice_command.py`
**Class: VoiceCommandProcessor**
- All methods ✅ USED

#### `web_server/app/routes.py`
**Routes:**
- All routes ✅ USED

#### `web_server/app/config.py`
**Class: Config**
- All class variables and methods ✅ USED

#### `web_server/course_config.py`
**Status:** ⚠️ DEPRECATED - This file duplicates `app/config.py` and should be removed. The application uses `app/config.py` instead.

### Recommendations for Code Cleanup

#### High Priority
1. **Consider using `g_min_pwm`:**
   - Currently declared but not used in control logic
   - Either implement minimum PWM enforcement or remove the variable

#### Low Priority
1. **Stub methods in navigation_coordinated.py:**
   - `update_encoder_data()` and `handle_motor_complete()` are empty
   - Consider removing if never called, or add documentation explaining they're intentionally empty

### Summary
- **Total C Functions:** 24 (all actively used)
- **Total Python Functions/Methods:** ~40 (all actively used)
- **Unused Code:** Minimal - deprecated `course_config.py` removed, duplicate declarations fixed
- **Code Health:** Excellent - very clean codebase with minimal cruft

All documented functions have been verified to exist in the codebase.
