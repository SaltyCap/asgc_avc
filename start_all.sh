#!/bin/bash

# AVC Project Startup Script

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Starting AVC Project..."

# 0. Set I2C Speed
echo "Setting I2C speed to 400kHz..."
if sudo dtparam i2c_arm_baudrate=400000; then
    echo "I2C speed set."
else
    echo "Warning: Failed to set I2C speed. Please ensure 'dtparam=i2c_arm=on,i2c_arm_baudrate=400000' is in /boot/config.txt"
fi

# 1. Build C Code
echo "Building C motor control code..."
cd "$SCRIPT_DIR/c_code"
if make; then
    echo "C code built successfully."
else
    echo "Error: Failed to build C code."
    exit 1
fi

# 2. Start Web Server
echo "Starting Web Server..."
cd "$SCRIPT_DIR/web_server"

# Check if venv exists
if [ -d "venv" ]; then
    source venv/bin/activate
else
    echo "Error: Virtual environment 'venv' not found in web_server directory."
    exit 1
fi

# Run the server
# Note: The server needs to be run with python, and it will internally use sudo for the motor process
python web_server.py
