# ASGC AVC Guide

This is the single current guide for this repository.

## Purpose

The system drives a robot autonomously to course targets using:

- Web UI for voice and queue control
- Python Flask + WebSocket backend
- C real-time motor, encoder, and odometry control

## Repo Layout

- `web_server/`: UI, Flask routes, WebSocket handling, navigation queue
- `c_code/`: motor control, encoder polling, odometry, state machine
- `logs/`: generated runtime CSV logs (created at runtime)
- `start_all.sh`: full startup script

## Quick Start

1. From repo root, run:

```bash
./start_all.sh
```

2. Open on phone or laptop:

```text
https://<robot-ip>:5000
```

3. Main pages:
- `/` voice + queue control
- `/joystick` direct PWM joystick control
- `/course` course visualization + status

## Voice Command Flow

1. In `/`, press `Voice Start` and speak.
2. Audio goes to WebSocket `/audio`.
3. Backend speech recognition maps words to commands.
4. Target words are queued (`red`, `yellow`, `blue`, `green`, `center`).
5. Immediate words execute instantly:
- `start`: begin queue execution
- `stop`: stop and clear queue
- `clear`: clear queue
- `reset`: reset odometry to start position
- `calibrate`: gyro calibrate + reset position

## Autonomous Navigation Flow

1. Python sends `goto x y is_bucket` to C process.
2. C control loop plans and executes:
- turn-to-heading
- straight drive with ramping
- wheel synchronization correction
3. For bucket targets, C performs:
- bucket zone detect
- 180 degree rotate
- backup to about `0.25 ft` from bucket
4. C emits `STATUS x y heading state` at `50 Hz`.
5. Python updates UI and queue progression from that status.

## Core Runtime Rates

- Encoder/sensor feedback loop: `1000 Hz` (`c_code/src/main.c`)
- Control loop/state machine: `500 Hz` (`c_code/src/main.c`)
- UI status polling: `500 ms` (`web_server/static/js/main.js`)

## Control and Tuning

- UI PWM slider sends:
- `set_pwm` for min/max pulse limits
- `set_speed` for navigation speed percent
- Joystick mode sends direct `pulse <left_ns> <right_ns>`.
- Voice/queue mode sends high-level navigation commands.

## C Command Reference

Commands accepted on C stdin:

- `goto <x> <y> [is_bucket]`
- `speed <0.0..1.0>`
- `setpwm <min_percent> <max_percent>`
- `setpos <x> <y> <heading_deg>`
- `calibrate`
- `pulse <left_ns> <right_ns>`
- `ml_mode`
- `ml_ball_acquired <0|1>`
- `ml_ball_color <red|yellow|blue|green|none>`
- `stop`
- `q`

## Logging

Logs are buffered in memory and written to CSV:

- persistent logs: `../logs/motor_log_<mode>_<timestamp>.csv`
- quick-access copy: `/dev/shm/motor_log_<mode>_latest.csv`

`stop` and shutdown both flush logs.

## Hardware Notes Used by Current Code

- Left AS5600: `/dev/i2c-3`, address `0x40`
- Right AS5600: `/dev/i2c-1`, address `0x1B`
- IMU MPU6050: `/dev/i2c-2`, address `0x68`

## Troubleshooting

- If voice is unavailable, verify:
- SSL certs exist in `web_server/`
- Vosk model exists at `web_server/model`
- If robot does not move:
- confirm C binary built: `c_code/asgc_motor_control`
- confirm process starts with permissions for PWM and I2C
- If odometry drifts:
- run `calibrate` while robot is stationary
- check encoder count direction signs in `c_code/src/main.c`
