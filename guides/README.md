# ASGC Autonomous Vehicle Challenge - Complete System

This repository contains a complete autonomous vehicle control system with voice-controlled navigation for the Arkansas Space Grant Commission competition on a 30x30 foot course.

## 🚀 Quick Start

### 1. Hardware Requirements
- **Raspberry Pi 5** (or compatible)
- **2x Brushless DC Motors** with ESCs
- **AS5600 Magnetic Encoders** (I2C)
- **Power**: LiPo battery for motors, USB-C/BEC for Pi

### 2. Startup
The system includes a single startup script that builds the C code, sets up the Python environment, and launches the server.

```bash
cd ~/asgc_avc
./start_all.sh
```


### 3. Access Interfaces
Once running, connect to the Pi via WiFi and access:
- **Voice Control**: `https://<PI_IP>:5000/`
- **Joystick**: `https://<PI_IP>:5000/joystick`
- **Course Map**: `https://<PI_IP>:5000/course`

> **Note**: Accept the browser security warning (self-signed certificate) to enable microphone access.

**Finding Pi's IP Address:**
- Use `hostname -I` on the Pi
- Example: `https://192.168.1.100:5000`

---

## 🎙️ Voice Commands

The system uses **Vosk** for offline speech recognition. Commands are queued and executed sequentially.

### Navigation (Adds to Queue)
| Command | Action |
|---------|--------|
| **"Red"** | Queue navigation to Red bucket (0, 0) |
| **"Yellow"** | Queue navigation to Yellow bucket (0, 30) |
| **"Blue"** | Queue navigation to Blue bucket (30, 30) |
| **"Green"** | Queue navigation to Green bucket (30, 0) |
| **"Center"** | Queue navigation to Center (15, 15) |

### Control (Immediate)
| Command | Action |
|---------|--------|
| **"Start"** | Execute the queued commands |
| **"Stop"** | Emergency stop and clear queue |
| **"Clear"** | Clear all pending commands |
| **"Reset"** | Reset internal position to (0,15) |

---

## 🏗️ System Architecture

### Overview
The system consists of two main processes communicating via standard I/O pipes:

1.  **Python Web Server** (`web_server/`)
    -   **Framework**: Flask + Flask-Sock
    -   **Role**: Handles user interface, voice recognition, path planning, and command queuing.
    -   **Voice Logic**: `app/voice_command.py` processes speech into intent.
    -   **Communication**: Sends text commands (`goto x y`) to the C program.

2.  **C Motor Controller** (`c_code/`)
    -   **Role**: Real-time motor control, PID loops, odometry, and hardware interfacing.
    -   **Feedback**: Sends position updates (`STATUS x y h s`) back to Python.
    -   **Control Loop**: Runs at 200Hz for smooth operation.

### Data Flow
`User Speech` → `Vosk (Python)` → `Command Queue` → `Motor Interface` → `stdin` → `C Program` → `PWM/I2C` → `Motors`

---

## 🔧 Configuration & Tuning

Configuration constants are centralized in `web_server/app/config.py`.

### Physical Robot Calibration
Update these values to match your specific hardware for accurate navigation:
```python
WHEEL_DIAMETER_INCHES = 4.0      # Measure carefully!
WHEELBASE_INCHES = 12.0          # Distance between wheel centers
```

### Course Dimensions
The course is a 30' x 30' grid.
- **Red**: (0, 0)
- **Yellow**: (0, 30)
- **Blue**: (30, 30)
- **Green**: (30, 0)
- **Start**: (0, 15) facing East (90°)

---

## 🛠️ Troubleshooting

### Voice Recognition Issues
- **"Microphone permission denied"**: Ensure you are using **HTTPS**. Browser safety rules block mic access on HTTP sites (except localhost).
- **"Vosk model not found"**: Run `./start_all.sh` again to auto-download the model.

### Navigation Issues
- **Robot spins in place**: Check `WHEELBASE_INCHES` or motor polarity.
- **Distances are wrong**: Calibrate `WHEEL_DIAMETER_INCHES`.
- **Drifting**: Ensure wheels are not slipping and encoders are securely mounted.

### Motor Issues
- **No movement**: Check battery voltage and ESC initialization.
- **"pigpio connection failed"**: Ensure `pigpiod` is running (started by `./start_all.sh`).

---

## 📂 Project Structure

```
asgc_avc/
├── start_all.sh              # MASTER STARTUP SCRIPT
├── c_code/                   # Motor Control (C)
│   ├── src/                  # Source files (main.c, motor.c, pid.c)
│   ├── include/              # Headers
│   └── Makefile              # Build system
├── web_server/               # Web Application (Python)
│   ├── web_server.py         # Entry point
│   ├── app/
│   │   ├── config.py         # Configuration & Constants
│   │   ├── voice_command.py  # Voice logic
│   │   └── ...
│   ├── static/               # JS/CSS
│   └── templates/            # HTML
└── guides/                   # Documentation
    ├── README.md             # This file
    └── CLAUDE.md             # AI coding guidelines
```
