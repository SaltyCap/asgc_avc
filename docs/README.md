# ASGC Autonomous Vehicle Challenge - Complete System

This repository contains a complete autonomous vehicle control system with voice-controlled navigation for the Arkansas Space Grant Commission competition on a 30x30 foot course.

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│                    USER INTERFACES                      │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐ │
│  │ Voice Control│   │   Joystick   │   │  Course View │ │
│  │  + Queue     │   │   (Touch)    │   │    (Map)     │ │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘ │
└─────────┼──────────────────┼──────────────────┼─────────┘
          │                  │                  │
          └──────────────────┼──────────────────┘
                             │ HTTPS/WebSocket
                    ┌────────▼────────┐
                    │  Flask Web      │
                    │  Server         │
                    │  - Voice (Vosk) │
                    │  - Navigation   │
                    │  - Cmd Queue    │
                    └────────┬────────┘
                             │ stdin/stdout
                    ┌────────▼────────┐
                    │  Motor Control  │
                    │  (C Program)    │
                    │  - PWM Control  │
                    │  - I2C Encoder  │
                    └─────────────────┘
                             │
                    ┌────────▼────────┐
                    │   HARDWARE      │
                    │  - 2x Motors    │
                    │  - AS5600       │
                    │  - PWM ESCs     │
                    └─────────────────┘
```

## Key Features

### Command Queue System
- **5-slot visual queue** - See pending commands at a glance
- **Voice-controlled queue** - Say colors to queue destinations
- **Sequential execution** - Commands process one at a time
- **Clear/Stop controls** - Voice or button to clear queue

### Closed-Loop Navigation
- **PID control** - Real-time error correction
- **Differential drive** - Coordinated dual-motor control
- **Encoder feedback** - High-accuracy positioning
- **Automatic drift compensation**

### Voice Recognition
- **Offline processing** - Vosk speech recognition (no internet required)
- **Low-latency** - Fast command recognition
- **Custom vocabulary** - Optimized for navigation commands

## Quick Start

### 1. Hardware Requirements
- Raspberry Pi 5 (or compatible)
- 2x Brushless motors with ESCs
- AS5600 magnetic encoder (I2C)
- PWM pins (GPIO 12, 13)

### 2. Start the System

```bash
cd ~/antigravity
./start_all.sh
```

This script:
1. Compiles the C motor controller
2. Activates Python virtual environment
3. Starts the web server with motor control

### 3. Access Interfaces

- **Voice Control + Queue**: https://your-ip:5000/
- **Joystick**: https://your-ip:5000/joystick
- **Course View**: https://your-ip:5000/course

## Project Structure

```
antigravity/
├── start_all.sh              # Main startup script
│
├── c_code/
│   ├── src/
│   │   ├── mc2_coordinated.c # Motor control with PID
│   │   └── ...
│   ├── mc2_coordinated       # Compiled binary
│   └── Makefile
│
├── web_server/
│   ├── web_server.py         # Main Flask server
│   ├── navigation_coordinated.py  # Navigation + queue
│   ├── course_config.py      # Course parameters
│   ├── model/                # Vosk model (vosk-model-en-us-0.22)
│   ├── cert.pem              # SSL certificate
│   ├── key.pem               # SSL key
│   ├── app/
│   │   ├── __init__.py
│   │   ├── routes.py         # HTTP routes
│   │   └── sockets.py        # WebSocket handlers + voice
│   ├── static/
│   │   ├── css/style.css
│   │   └── js/main.js
│   └── templates/
│       ├── index.html        # Voice control + queue UI
│       ├── joystick.html     # Manual control
│       └── course_view.html  # Course map visualization
│
└── docs/
    ├── README.md             # This file
    └── ...
```

## Voice Commands

### Navigation (Queue Commands)
| Command | Action |
|---------|--------|
| "red" | Queue navigation to red bucket (0, 0) |
| "yellow" | Queue navigation to yellow bucket (0, 30) |
| "blue" | Queue navigation to blue bucket (30, 30) |
| "green" | Queue navigation to green bucket (30, 0) |
| "center" | Queue navigation to center (15, 15) |

### Control Commands
| Command | Action |
|---------|--------|
| "stop" | Emergency stop + clear queue |
| "clear" | Clear all queued commands |
| "reset position" | Reset robot position to start |

### Manual Movement
| Command | Action |
|---------|--------|
| "forward" | Move forward |
| "back" / "backward" | Move backward |
| "left" | Turn left |
| "right" | Turn right |

## Command Queue

The voice control page shows a 5-slot command queue:

```
[ 1 ]  [ 2 ]  [ 3 ]  [ 4 ]  [ 5 ]
 RED   BLUE  GREEN   ---    ---
```

- **Slot 1**: Currently executing (pulsing animation)
- **Slots 2-5**: Pending commands
- **Empty slots**: Show "---"
- **Color-coded**: Each slot matches its target color

### Queue Behavior
1. Say a color to add it to the queue
2. Commands execute in order (FIFO)
3. When command 1 completes, everything shifts up
4. Maximum 5 commands can be queued
5. "clear" removes all pending commands
6. "stop" stops current + clears queue

## Course Layout

```
    Yellow (0,30)              Blue (30,30)
         ●─────────────────────────●
         │                         │
         │                         │
         │           ○             │  30 ft
         │       (15,15)           │
         │        Center           │
         │                         │
    ──── X ────────────────────────●
   Start Red (0,0)            Green (30,0)
  (0,15)
              30 ft
```

- **Play area**: 30x30 feet
- **Visual border**: 2.5 ft padding (display only)
- **Start position**: (0, 15) facing east (90°)
- **Center**: (15, 15)

## Configuration

### course_config.py

```python
# Robot physical parameters
WHEEL_DIAMETER_INCHES = 4.0      # Calibrate this!
WHEELBASE_INCHES = 12.0          # Calibrate this!

# Navigation settings
DEFAULT_SPEED = 30               # Driving speed %
TURN_SPEED = 25                  # Turning speed %
POSITION_TOLERANCE = 1.0         # feet
HEADING_TOLERANCE = 5.0          # degrees

# Course layout
PLAY_AREA = 30                   # 30x30 foot course
BUCKETS = {
    'red': (0, 0),
    'yellow': (0, 30),
    'blue': (30, 30),
    'green': (30, 0)
}
CENTER = (15, 15)
START_POSITION = (0, 15)
START_HEADING = 90               # Facing east
```

## API Reference

### REST Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/navigation/status` | GET | Robot position, heading, queue status |
| `/api/navigation/goto_center` | POST | Queue navigation to center |
| `/api/navigation/goto_bucket/:color` | POST | Queue navigation to bucket |
| `/api/course/info` | GET | Course configuration |

### WebSocket Endpoints

| Endpoint | Description |
|----------|-------------|
| `/audio` | Voice recognition streaming |
| `/motor` | Motor control + voice commands |

### Navigation Status Response

```json
{
  "x": 15.0,
  "y": 15.0,
  "heading": 90.0,
  "target": [30, 30],
  "mode": "goto_bucket",
  "state": "driving",
  "queue": [
    {"target": "GREEN", "position": [30, 0]},
    {"target": "CENTER", "position": [15, 15]}
  ],
  "queue_running": true
}
```

## User Interfaces

### Voice Control (index.html)
- Start/Stop car control
- Start/Stop voice recognition
- Live transcript display
- **Command queue visualization (5 slots)**
- **STOP / CLEAR / RESET buttons**
- Navigation links to Joystick and Course

### Course View (course_view.html)
- Real-time course map (30x30)
- Robot position and heading
- Bucket locations (color-coded)
- Target line when navigating
- STOP button
- Position/Heading/Mode display

### Joystick (joystick.html)
- Touch/click joystick control
- Real-time motor control
- Connection status

## Calibration

### 1. Wheel Diameter
```bash
# Command 10 revolutions
# In motor WebSocket: send "r10"
# Measure actual distance traveled
# Adjust WHEEL_DIAMETER_INCHES
```

### 2. Wheelbase
```bash
# Command 360° turn
# Verify robot returns to same heading
# Adjust WHEELBASE_INCHES
```

### 3. Test Navigation
```bash
cd ~/antigravity/web_server
python3 test_navigation.py
```

## Troubleshooting

### Voice Recognition
- **Commands not recognized**: Check VOCABULARY in sockets.py
- **Poor accuracy**: Speak clearly, reduce background noise
- **No response**: Check microphone permissions in browser

### Navigation
- **Robot doesn't move**: Check motor controller connection
- **Wrong direction**: Verify START_HEADING, check motor wiring
- **Position drift**: Calibrate wheel diameter and wheelbase
- **Queue not updating**: Check WebSocket connection

### Motor Control
- **Motors don't respond**: Check sudo, I2C, PWM setup
- **Segfault**: Check Vosk model compatibility
- **Connection lost**: Motor controller auto-restarts

## Safety

- **Emergency stop** on all interfaces
- **Motors stop** on WebSocket disconnect
- **Graceful shutdown** on Ctrl+C
- **Queue clear** on stop command

## Technology Stack

- **Motor Control**: C with pigpio (PWM, I2C)
- **Web Server**: Flask + Flask-Sock
- **Voice Recognition**: Vosk (vosk-model-en-us-0.22, 1.8GB)
- **Frontend**: Vanilla JS, CSS3
- **Communication**: WebSocket, REST API

## License

Educational use for ASGC Autonomous Vehicle Challenge.

---

**Ready to compete!**

1. Run `./start_all.sh`
2. Open https://your-ip:5000/
3. Start voice recognition
4. Say "center" then a color
5. Watch the robot navigate!
