#!/bin/bash
# Script to train the ML controller model using conda/miniforge
# Usage:
#   ./runtraining.sh [args for train.py]

cd "$(dirname "$0")/.."

# Determine which python3 to use.
PYTHON3="python3"
if python3 -c "import pandas, numpy, sklearn, matplotlib" 2>/dev/null; then
    : # system python3 is fine
elif [[ -x "$HOME/miniforge3/bin/python3" ]] && \
     "$HOME/miniforge3/bin/python3" -c "import pandas, numpy, sklearn, matplotlib" 2>/dev/null; then
    PYTHON3="$HOME/miniforge3/bin/python3"
else
    # Last resort: set up conda and install missing packages
    if ! command -v conda &>/dev/null; then
        if [[ -x "$HOME/miniforge3/bin/conda" ]]; then
            export PATH="$HOME/miniforge3/bin:$PATH"
        else
            echo "conda not found. Auto-installing Miniforge3..."
            ARCH="$(uname -m)"
            case "$ARCH" in
                aarch64|arm64) MINI_ARCH="aarch64" ;;
                x86_64)        MINI_ARCH="x86_64"  ;;
                *)             echo "Unsupported architecture: $ARCH"; exit 1 ;;
            esac
            MINI_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${MINI_ARCH}.sh"
            MINI_INSTALLER="/tmp/Miniforge3-Linux-${MINI_ARCH}.sh"
            echo "Downloading Miniforge3 from $MINI_URL ..."
            if command -v curl &>/dev/null; then
                curl -fsSL "$MINI_URL" -o "$MINI_INSTALLER"
            elif command -v wget &>/dev/null; then
                wget -q "$MINI_URL" -O "$MINI_INSTALLER"
            else
                echo "Error: neither curl nor wget found. Please install one and retry."
                exit 1
            fi
            echo "Installing Miniforge3 to ~/miniforge3 ..."
            bash "$MINI_INSTALLER" -b -p "$HOME/miniforge3"
            rm -f "$MINI_INSTALLER"
            export PATH="$HOME/miniforge3/bin:$PATH"
            echo "Miniforge3 installed."
        fi
    fi
    eval "$(conda shell.bash hook)"
    conda activate base
    if ! "$HOME/miniforge3/bin/python3" -c "import pandas, numpy, sklearn, matplotlib" 2>/dev/null; then
        echo "Installing required packages for training..."
        conda install -y pandas numpy scikit-learn matplotlib
    fi
    PYTHON3="$HOME/miniforge3/bin/python3"
fi

echo "Running training script with $PYTHON3..."
"$PYTHON3" ml_training/train.py "$@"
