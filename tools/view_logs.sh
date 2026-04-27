#!/bin/bash
# Log Viewer Launcher Script
# Uses conda/miniforge Python for better binary compatibility
# Usage:
#   ./view_logs.sh                 # interactive file picker
#   ./view_logs.sh latest [N]      # open latest N logs (default 1)
#   ./view_logs.sh tuning [N]      # open latest N voice logs (default 5)
#   ./view_logs.sh mode <name> [N] # open latest N logs for mode: voice|joystick|ml
#   ./view_logs.sh validate_ml      # validate latest log for ML controller behavior
#   ./view_logs.sh validate_ml out_back # validate latest log incl. 10ft out-and-back checks
#   ./view_logs.sh /path/a.csv ... # open specific files

cd "$(dirname "$0")"

# Fast path: validator does not need conda or plotting deps.
if [[ "${1:-}" == "validate_ml" ]]; then
    if [[ "${2:-}" == "out_back" ]]; then
        python3 validate_hil_log.py --expect-ml --expect-out-back
    else
        python3 validate_hil_log.py --expect-ml
    fi
    exit $?
fi

# Determine which python3 to use.
# Fast path: if the system python3 already has the required packages, use it directly.
# This avoids slow conda initialization entirely.
PYTHON3="python3"
if python3 -c "import pandas, matplotlib, tkinter" 2>/dev/null; then
    : # system python3 is fine — nothing to do
elif [[ -x "$HOME/miniforge3/bin/python3" ]] && \
     "$HOME/miniforge3/bin/python3" -c "import pandas, matplotlib, tkinter" 2>/dev/null; then
    # Miniforge is installed and has the packages — use it directly (no conda activate needed)
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
    if ! "$HOME/miniforge3/bin/python3" -c "import pandas, matplotlib" 2>/dev/null; then
        echo "Installing required packages..."
        conda install -y pandas matplotlib
    fi
    PYTHON3="$HOME/miniforge3/bin/python3"
fi

MODE="interactive"
COUNT=""
MODE_FILTER=""
VALIDATE_OUT_BACK="0"
shift_args=()

case "${1:-}" in
    latest)
        MODE="latest"
        COUNT="${2:-1}"
        MODE_FILTER="${3:-any}"
        ;;
    tuning)
        MODE="latest"
        COUNT="${2:-5}"
        MODE_FILTER="voice"
        ;;
    mode)
        MODE="latest"
        MODE_FILTER="${2:-voice}"
        COUNT="${3:-3}"
        ;;
    validate_ml)
        MODE="validate_ml"
        if [[ "${2:-}" == "out_back" ]]; then
            VALIDATE_OUT_BACK="1"
        fi
        ;;
    "")
        MODE="interactive"
        ;;
    *)
        MODE="files"
        shift_args=("$@")
        ;;
esac

# Move empty log files (header-only, 0 data rows) to trash before opening viewer
LOG_DIR="$(cd "$(dirname "$0")/../logs" 2>/dev/null && pwd)"
TRASH_DIR="${LOG_DIR}/.trash"

_trash_empty_csvs() {
    local dir="$1"
    local prefix="${2:-}"
    [[ -d "$dir" ]] || return
    while IFS= read -r -d '' csv; do
        # Count lines; a header-only file has exactly 1 non-empty line
        local lines
        lines=$(grep -c . "$csv" 2>/dev/null || echo 0)
        if [[ "$lines" -le 1 ]]; then
            mkdir -p "$TRASH_DIR"
            local base; base=$(basename "$csv")
            local dest="${TRASH_DIR}/${base}"
            local n=1
            while [[ -e "$dest" ]]; do
                local stem="${base%.*}" ext="${base##*.}"
                dest="${TRASH_DIR}/${stem}_${n}.${ext}"
                (( n++ ))
            done
            mv "$csv" "$dest"
            echo "Trashed empty log: ${base} -> ${dest}"
        fi
    done < <(find "$dir" -maxdepth 1 -name "${prefix}*.csv" -print0 2>/dev/null)
}

if [[ -n "$LOG_DIR" ]]; then
    _trash_empty_csvs "$LOG_DIR"
fi
_trash_empty_csvs "/dev/shm" "motor_log_"

# Run the log viewer
if [[ "$MODE" == "latest" ]]; then
    "$PYTHON3" log_viewer.py --latest "$COUNT" --mode "$MODE_FILTER"
elif [[ "$MODE" == "validate_ml" ]]; then
    if [[ "$VALIDATE_OUT_BACK" == "1" ]]; then
        "$PYTHON3" validate_hil_log.py --expect-ml --expect-out-back
    else
        "$PYTHON3" validate_hil_log.py --expect-ml
    fi
elif [[ "$MODE" == "files" ]]; then
    "$PYTHON3" log_viewer.py "${shift_args[@]}"
else
    "$PYTHON3" log_viewer.py
fi
