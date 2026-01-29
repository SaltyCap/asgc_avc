#!/bin/bash
# Log Viewer Launcher Script
# Activates virtual environment and runs the log viewer

cd "$(dirname "$0")"

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Creating..."
    python3 -m venv venv
    echo "Installing dependencies..."
    ./venv/bin/pip install pandas matplotlib
fi

# Run the log viewer with venv Python
./venv/bin/python3 log_viewer.py
