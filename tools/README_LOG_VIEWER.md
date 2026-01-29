# Log Viewer

Interactive graphical viewer for ASGC motor control logs.

## Features

- **File Selection**: Browse and select any CSV log file
- **8 Interactive Plots**:
  1. Motor PWM commands (left/right)
  2. Encoder raw angles from I2C
  3. Left motor target vs actual counts
  4. Right motor target vs actual counts
  5. MPU6050 gyro Z-axis data
  6. Fused heading (Kalman filtered)
  7. X/Y position over time
  8. 2D path visualization with start/end markers
- **Auto-scaling**: All plots automatically scale to fit data
- **Navigation State Shading**: Background colors show IDLE/TURNING/DRIVING/GOTO states
- **Interactive Controls**: Zoom, pan, reset view
- **Reload/New File**: Quick buttons to refresh or load different logs

## Usage

**Easy way (recommended):**
```bash
# From the logs directory
./view_logs.sh
```

**Manual way:**
```bash
# Activate venv and run
source venv/bin/activate
python3 log_viewer.py
```

## Requirements

- Python 3
- pandas
- matplotlib
- tkinter (usually pre-installed)

## Controls

- **Left click + drag**: Pan the view
- **Right click + drag**: Zoom in/out
- **Home button**: Reset to default view
- **Reload button**: Refresh current file
- **New File button**: Load a different log

## Log Format

Expected CSV columns:
- `time`, `mode`, `pwm_l`, `i2c_l`, `pwm_r`, `i2c_r`
- `target_l`, `actual_l`, `target_r`, `actual_r`
- `gyro_z`, `odom_x`, `odom_y`, `odom_heading`, `nav_state`
