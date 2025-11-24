# Motor Control Integration Guide

## Overview

The C motor control program and Python web server are integrated via stdin/stdout communication. The web server provides three interfaces:

1. **Voice Control + Queue** - Speak commands to queue navigation targets
2. **Joystick Interface** - Visual touch controls for manual driving
3. **Course View** - Real-time map showing robot position

## Architecture

```
┌─────────────────┐
│  Web Browser    │ (Phone/Computer)
│  - Voice + Queue│
│  - Joystick     │
│  - Course Map   │
└────────┬────────┘
         │ WebSocket (HTTPS/WSS)
         ▼
┌─────────────────┐
│  Web Server     │ (Python/Flask)
│  - WebSocket    │
│  - Voice (Vosk) │
│  - Navigation   │
│  - Cmd Queue    │
└────────┬────────┘
         │ stdin/stdout (subprocess)
         ▼
┌─────────────────┐
│  Motor Control  │ (C Program)
│  - PWM Control  │
│  - I2C Encoder  │
│  - PID Control  │
└─────────────────┘
```

## How It Works

### Startup Script (`start_all.sh`)
```bash
#!/bin/bash
cd ~/antigravity/c_code && make clean && make
cd ~/antigravity/web_server
source venv/bin/activate
sudo ./venv/bin/python web_server.py
```

### Web Server (`web_server.py`)
- Starts `mc2_coordinated` as a subprocess with `sudo`
- Manages WebSocket endpoints:
  - `/audio` - Voice recognition streaming
  - `/motor` - Motor control + voice commands
- Translates voice/joystick inputs into motor commands
- Manages navigation queue (5-slot FIFO)
- Sends commands to C program via stdin

### Motor Control (`mc2_coordinated.c`)
- Runs as subprocess under web server
- Receives commands via stdin:
  - `1` or `2` - Select motor
  - `r<number>` - Spin motor revolutions
  - `drive <counts>` - Coordinated forward drive
  - `turn <counts>` - Coordinated turn
  - `stop` - Stop selected motor
  - `stopall` - Emergency stop all
  - `q` - Quit

### Navigation Controller (`navigation_coordinated.py`)
- Manages 5-slot command queue
- Processes commands sequentially
- Calculates bearing and distance to targets
- Sends turn/drive commands to motor controller
- Tracks robot position via encoder feedback

## Voice Commands

### Navigation (Queue Commands)
| Command | Action |
|---------|--------|
| "red" | Queue → red bucket (0, 0) |
| "yellow" | Queue → yellow bucket (0, 30) |
| "blue" | Queue → blue bucket (30, 30) |
| "green" | Queue → green bucket (30, 0) |
| "center" | Queue → center (15, 15) |

### Control Commands
| Command | Action |
|---------|--------|
| "stop" | Emergency stop + clear queue |
| "clear" | Clear all queued commands |
| "reset position" | Reset to start position |

### Manual Movement
| Command | Action |
|---------|--------|
| "forward" | Drive forward |
| "back" / "backward" | Drive backward |
| "left" | Turn left |
| "right" | Turn right |

## Command Queue System

The voice control page displays a 5-slot queue:

```
[ 1 ]  [ 2 ]  [ 3 ]  [ 4 ]  [ 5 ]
 RED   BLUE  GREEN   ---    ---
```

- Slot 1: Currently executing (pulsing)
- Slots 2-5: Pending commands
- Commands shift up as each completes
- Max 5 commands queued at once

## Running the System

### Prerequisites

1. **Motor control compiled:**
   ```bash
   cd ~/antigravity/c_code
   make
   ```

2. **Python venv with dependencies:**
   ```bash
   cd ~/antigravity/web_server
   python3 -m venv venv
   source venv/bin/activate
   pip install flask flask-sock vosk numpy
   ```

3. **SSL certificates:**
   ```bash
   cd ~/antigravity/web_server
   openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
   ```

4. **Vosk model** in `web_server/model/`

### Starting the Server

```bash
cd ~/antigravity
./start_all.sh
```

### Accessing Interfaces

| Interface | URL |
|-----------|-----|
| Voice + Queue | https://\<ip\>:5000/ |
| Joystick | https://\<ip\>:5000/joystick |
| Course Map | https://\<ip\>:5000/course |

## WebSocket Communication

### /motor Endpoint

**Receive from client:**
```json
{"type": "joystick", "x": 50, "y": 75}
{"type": "stop"}
{"type": "voice", "command": "red"}
```

**Send to client:**
```json
{"type": "status", "message": "Command received"}
```

### /audio Endpoint

**Receive:** Raw 16-bit PCM audio at 16kHz

**Send:**
```json
{"type": "partial", "text": "re"}
{"type": "final", "text": "red"}
```

## Code Flow

### Voice Command Flow
```
Browser Mic → /audio WebSocket → Vosk Recognition
    → process_voice_command() → nav_controller.go_to_bucket()
    → queue_command() → _process_queue()
    → send_command("turn X") → motor_interface.send_command()
    → mc2_coordinated stdin
```

### Joystick Flow
```
Browser Touch → /motor WebSocket → motor_interface.send_command()
    → mc2_coordinated stdin → PWM output
```

## File Structure

```
antigravity/
├── start_all.sh
├── c_code/
│   ├── src/mc2_coordinated.c
│   ├── mc2_coordinated (binary)
│   └── Makefile
└── web_server/
    ├── web_server.py
    ├── navigation_coordinated.py
    ├── course_config.py
    ├── app/
    │   ├── routes.py
    │   └── sockets.py
    └── templates/
        ├── index.html (voice + queue)
        ├── joystick.html
        └── course_view.html
```

## Troubleshooting

### Motor control not starting
- Check `mc2_coordinated` exists in `c_code/`
- Ensure running with `sudo`
- Verify I2C and PWM configured

### WebSocket fails
- Check SSL certificates
- Verify port 5000 open
- Accept self-signed cert in browser

### Voice not recognized
- Check microphone permissions
- Verify Vosk model in `model/`
- Check VOCABULARY in `sockets.py`

### Queue not updating
- Check `/api/navigation/status` returns data
- Verify motor WebSocket connected
- Check browser console for errors

## Safety Features

1. **Emergency Stop** - Voice "stop" or button clears queue + stops motors
2. **Connection Monitor** - Auto-reconnect WebSocket
3. **Graceful Shutdown** - Ctrl+C stops motors and subprocess
4. **Queue Limit** - Max 5 commands prevents overload
