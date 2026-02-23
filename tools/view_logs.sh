#!/bin/bash
# Log Viewer Launcher Script

cd "$(dirname "$0")"

# Check if venv exists, create if not
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    
    # Activate virtual environment
    source venv/bin/activate
    
    echo "Installing required packages..."
    pip install pandas matplotlib
else
    # Activate existing virtual environment
    source venv/bin/activate
fi

# Check if required packages are installed in venv
if ! python -c "import pandas, matplotlib" 2>/dev/null; then
    echo "Installing missing packages..."
    pip install pandas matplotlib
fi

# Run the log viewer
python log_viewer.py
