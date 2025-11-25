# Autonomous Vehicle Challenge - Navigation Guide

## Challenge Overview

Your robot must navigate a 30×30 foot course with colored buckets at each corner and complete the following mission:

1. **Start Position**: Left side between yellow and red buckets (0, 15), facing east
2. **Task 1**: Navigate to the center of the course (15, 15)
3. **Task 2**: Wait for voice command indicating bucket color
4. **Task 3**: Navigate to the specified colored bucket

## Course Layout

```
30×30 foot play area with 2.5ft visual buffer

       0        15        30
   30  ┌─────────────────────┐
       │                     │
       │ YELLOW      BLUE    │
       │  (0,30)●───●(30,30) │
       │       │     │       │
   15 X│START  │  ●  │       │  X = Start (0,15)
       │       │(15,15)      │
       │       │CENTER       │
       │  RED ●─────● GREEN  │
    0  │(0,0)      (30,0)    │
       └─────────────────────┘
```

### Bucket Positions
- **Red**: (0, 0) - Bottom left
- **Yellow**: (0, 30) - Top left
- **Blue**: (30, 30) - Top right
- **Green**: (30, 0) - Bottom right

### Key Locations
- **Start**: (0, 15) - Left edge, midway between red and yellow
- **Center**: (15, 15) - Middle of play area
- **Play Area**: 30×30 feet

## Voice Command Queue System

### How It Works
1. Say a color command to add it to the queue
2. Commands execute in order (FIFO)
3. Maximum 5 commands can be queued
4. Visual display shows queue status

### Queue Display (on Voice Control page)
```
[ 1 ]  [ 2 ]  [ 3 ]  [ 4 ]  [ 5 ]
 RED   BLUE  GREEN   ---    ---
```

- Slot 1: Currently executing (pulsing animation)
- Slots 2-5: Pending commands
- Empty slots: Show "---"

### Voice Commands

**Navigation (adds to queue):**
- **"red"** - Queue navigation to red bucket
- **"yellow"** - Queue navigation to yellow bucket
- **"blue"** - Queue navigation to blue bucket
- **"green"** - Queue navigation to green bucket
- **"center"** - Queue navigation to center

**Control:**
- **"stop"** - Emergency stop + clear queue
- **"clear"** - Clear all queued commands
- **"reset position"** - Reset to start position

**Manual Movement:**
- **"forward"** / **"back"** - Manual driving
- **"left"** / **"right"** - Manual turning

## Web Interfaces

### 1. Voice Control + Queue (`/`)
- Voice recognition with Vosk
- **5-slot command queue display**
- STOP / CLEAR / RESET buttons
- Transcript display
- Best for competition (hands-free)

### 2. Course View (`/course`)
- Real-time course map (30×30)
- Robot position and heading indicator
- Bucket locations (color-coded)
- Target line when navigating
- STOP button
- Position/Heading/Mode status

### 3. Joystick Control (`/joystick`)
- Manual control interface
- Touch/click joystick
- Good for testing and manual override

## Running the System

### 1. Start the Server
```bash
cd ~/asgc_avc
./start_all.sh
```

### 2. Access the Interfaces
```
https://<your-ip>:5000/          # Voice control + queue
https://<your-ip>:5000/course    # Course visualization
https://<your-ip>:5000/joystick  # Manual joystick
```

### 3. Competition Sequence

**Setup Phase:**
1. Place robot at start position (0, 15) facing east
2. Open Course View to verify position
3. Say "reset position" to calibrate
4. Verify robot shows heading ~90°

**Run Phase:**
1. Say **"center"** - Robot queues navigation to (15, 15)
2. Watch queue display - slot 1 shows "CENTER" (pulsing)
3. Robot navigates to center
4. When complete, say bucket color: **"blue"**
5. Robot queues and navigates to blue bucket
6. Mission complete!

**Advanced - Queue Multiple:**
1. Say "center" then immediately "blue"
2. Queue shows: [CENTER, BLUE, ---, ---, ---]
3. Robot executes in order automatically

**Emergency:**
- Say **"stop"** - Stops motors and clears queue
- Say **"clear"** - Clears queue but doesn't stop current command

## Configuration

### course_config.py

```python
# Robot physical parameters - CALIBRATE THESE!
WHEEL_DIAMETER_INCHES = 4.0
WHEELBASE_INCHES = 12.0

# Navigation speeds
DEFAULT_SPEED = 30        # Driving speed %
TURN_SPEED = 25           # Turning speed %

# Tolerances
POSITION_TOLERANCE = 1.0   # feet
HEADING_TOLERANCE = 5.0    # degrees

# Course layout
PLAY_AREA = 30             # 30x30 feet
CENTER = (15, 15)
START_POSITION = (0, 15)
START_HEADING = 90         # Facing east

BUCKETS = {
    'red': (0, 0),
    'yellow': (0, 30),
    'blue': (30, 30),
    'green': (30, 0)
}
```

## API Endpoints

```
GET  /api/navigation/status      # Position, heading, queue status
POST /api/navigation/goto_center # Queue center navigation
POST /api/navigation/goto_bucket/:color  # Queue bucket navigation
GET  /api/course/info            # Course configuration
```

### Status Response Example
```json
{
  "x": 15.0,
  "y": 15.0,
  "heading": 90.0,
  "target": [30, 30],
  "mode": "goto_bucket",
  "state": "driving",
  "queue": [
    {"target": "GREEN", "position": [30, 0]}
  ],
  "queue_running": true
}
```

## Calibration

### 1. Wheel Diameter
```bash
# Command 10 revolutions via joystick or motor WebSocket
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
cd ~/asgc_avc/web_server
python3 test_navigation.py
```

## Navigation Algorithm

### Current Implementation
1. Calculate bearing to target point
2. Calculate turn angle (shortest path)
3. Turn to face target
4. Calculate distance to target
5. Drive straight to target
6. Repeat for next queued command

### Queue Processing
```
queue_command() → add to queue (max 5)
                → start processor if not running

_process_queue() → pop command from front
                 → set as current_target
                 → navigate_to_point_async()
                 → loop until queue empty
```

## Troubleshooting

### Queue not updating
- Check browser console for WebSocket errors
- Verify `/api/navigation/status` returns queue data
- Ensure motor WebSocket connected

### Robot doesn't move
- Check motor controller connection
- Verify C program running (check start_all.sh output)
- Test manual commands via joystick

### Position drifts
- Calibrate wheel diameter
- Calibrate wheelbase
- Check encoder connections

### Voice not recognized
- Check microphone permissions
- Speak clearly: "RED", "BLUE", etc.
- Check VOCABULARY in sockets.py

### Wrong direction
- Verify START_HEADING (90° = east)
- Check motor wiring
- Swap motor assignments if reversed

## File Locations

```
home/
├── start_all.sh
├── c_code/
│   └── src/mc2_coordinated.c
├── web_server/
│   ├── web_server.py
│   ├── navigation_coordinated.py  # Queue + navigation
│   ├── course_config.py
│   ├── app/
│   │   ├── routes.py
│   │   └── sockets.py             # Voice command processing
│   └── templates/
│       ├── index.html             # Voice + queue UI
│       ├── joystick.html
│       └── course_view.html
└── docs/
    └── README.md
```

## Competition Strategy

### Recommended Setup
1. Open Voice Control page on phone (for commands)
2. Open Course View on tablet (for monitoring)
3. Keep Joystick page ready as backup

### Execution
1. Place robot at start, say "reset position"
2. Say "center" - watch queue fill slot 1
3. Watch robot navigate on course view
4. When at center, say target color
5. Robot completes mission

### Backup Plan
- Use Joystick for manual control if needed
- "stop" voice command always available
- STOP button on all interfaces


