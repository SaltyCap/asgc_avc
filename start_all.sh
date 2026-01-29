#!/bin/bash

# AVC Project Startup Script

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Starting AVC Project..."

# 0. Check and start pigpiod daemon
# pigpiod check removed as it is not used by the current C implementation


# 1. Set I2C Speed
echo "Setting I2C speed to 400kHz..."
if sudo dtparam i2c_arm_baudrate=400000; then
    echo "I2C speed set."
else
    echo "Warning: Failed to set I2C speed. Please ensure 'dtparam=i2c_arm=on,i2c_arm_baudrate=400000' is in /boot/config.txt"
fi

# 2. Build C Code
echo "Building C motor control code..."
cd "$SCRIPT_DIR/c_code"
if make; then
    echo "C code built successfully."
else
    echo "Error: Failed to build C code."
    exit 1
fi

# 3. Setup Python Virtual Environment
echo "Setting up Python environment..."
cd "$SCRIPT_DIR/web_server"

# Check if venv exists, create if not
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    if python3 -m venv venv; then
        echo "Virtual environment created successfully."
    else
        echo "Error: Failed to create virtual environment."
        exit 1
    fi

    # Install dependencies
    echo "Installing Python dependencies..."
    source venv/bin/activate
    if pip install -r requirements.txt; then
        echo "Dependencies installed successfully."
    else
        echo "Error: Failed to install dependencies."
        exit 1
    fi
else
    echo "Virtual environment found."
    source venv/bin/activate
fi

# 4. Check for SSL Certificates
echo "Checking for SSL certificates..."
if [ ! -f "cert.pem" ] || [ ! -f "key.pem" ]; then
    echo "SSL certificates not found. Generating self-signed certificates..."
    if openssl req -x509 -newkey rsa:4096 -nodes -out cert.pem -keyout key.pem -days 365 -subj "/C=US/ST=Arkansas/L=Conway/O=ASGC/CN=localhost" 2>/dev/null; then
        echo "SSL certificates generated successfully."
        echo "Note: Your browser will show a security warning. This is expected for self-signed certificates."
    else
        echo "Warning: Failed to generate SSL certificates. Microphone access may not work."
        echo "You can manually generate them with:"
        echo "  openssl req -x509 -newkey rsa:4096 -nodes -out cert.pem -keyout key.pem -days 365"
    fi
else
    echo "SSL certificates found."
fi

# 5. Check for Vosk Model
echo "Checking for Vosk speech recognition model..."
MODEL_DIR="model"
MODEL_NAME="vosk-model-small-en-us-0.15"
MODEL_URL="https://alphacephei.com/vosk/models/${MODEL_NAME}.zip"

if [ ! -d "$MODEL_DIR" ]; then
    echo "Vosk model not found. Downloading ${MODEL_NAME} (600MB)..."
    echo "This may take several minutes depending on your connection..."

    # Download the model
    if wget -q --show-progress "$MODEL_URL" -O "${MODEL_NAME}.zip"; then
        echo "Download complete. Extracting..."
        if unzip -q "${MODEL_NAME}.zip"; then
            # Rename to 'model' for consistency
            mv "$MODEL_NAME" "$MODEL_DIR"
            rm "${MODEL_NAME}.zip"
            echo "Vosk model installed successfully."
        else
            echo "Error: Failed to extract model archive."
            rm -f "${MODEL_NAME}.zip"
            echo "You can manually download and extract the model:"
            echo "  wget $MODEL_URL"
            echo "  unzip ${MODEL_NAME}.zip"
            echo "  mv $MODEL_NAME $MODEL_DIR"
            exit 1
        fi
    else
        echo "Error: Failed to download Vosk model."
        echo "Please check your internet connection and try again."
        echo "You can manually download from: $MODEL_URL"
        exit 1
    fi
else
    echo "Vosk model found."
fi

# 6. Start Web Server
echo "Starting Web Server..."
# Note: The server needs to be run with python3, and it will internally use sudo for the motor process
python3 web_server.py
