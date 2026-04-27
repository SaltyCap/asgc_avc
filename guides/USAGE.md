# ML Motor Control - Usage Guide

This guide explains how to collect data, train the model, and run the robot in ML mode.

## 1. Data Collection (Phase 1)
Run the robot in standard modes (Joystick or PID Navigation) to collect training data.
- **Start Logging**: The system automatically logs data to `logs/` whenever it is running.
- **Driving**: Drive the robot manually or run existing PID test patterns.
- **Stop**: Send the `stop` command to dump logs to CSV files.

## 2. Model Training (Phase 2)
Use the Python scripts to train the model on your collected logs.

### Prerequisites
- Python 3.x
- `scikit-learn`, `pandas`, `numpy` (Install via `pip install -r ml_training/requirements.txt`)

### Training Steps
1.  **Transfer Logs**: Ensure your CSV logs are in the `logs/` directory (relative to `ml_training`).
2.  **Run Training**:
    ```bash
    python3 ml_training/train.py
    ```
    - This reads all CSVs in `logs/`.
    - Trains a Neural Network (MLP) to predict PWM from sensors.
    - **Auto-Exports**: Generates `c_code/src/inference_weights.c` automatically.
    - **Note:** The script will also verify the model works by running a dummy inference.

## 3. Deployment (Phase 3)
Compile the C code with the new weights.

1.  **Compile**:
    ```bash
    cd c_code
    make clean
    make
    ```
2.  **Run**:
    ```bash
    ./asgc_motor_control
    ```

## 4. Operational Mode
To activate the Neural Network control:

1.  **Command**: Send `ml_mode` via the command line or web interface.
    - Web UI: click **ML MODE** on the main page.
    - WebSocket: send `{\"type\": \"ml_mode\"}` (or `{\"type\": \"set_mode\", \"mode\": \"ml\"}`) to `/motor`.
2.  **Behavior**:
    - On each ML test cycle, the robot first navigates to course center (`15, 15`).
    - It then runs a pickup sweep that covers a 30-inch diameter circle at center.
    - The sweep stops immediately once `ball acquired` is reported (or after sweep coverage is complete).
    - Once both inputs are provided (`ball acquired` + `ball color`), it navigates to the matching bucket and performs the bucket drop sequence.
    - After drop completion, it resets and returns to center for the next cycle.
3.  **Ball Inputs**:
    - CLI: `ml_ball_acquired 1` and `ml_ball_color red|yellow|blue|green|none`
    - WebSocket (`/motor`):
      - `{\"type\": \"ml_ball_acquired\", \"acquired\": true}`
      - `{\"type\": \"ml_ball_color\", \"color\": \"red\"}`
4.  **Safety**: The standard failsafes (stop on disconnect, etc.) still apply. To stop, send `stop`.

## Troubleshooting
- **Weights not updating?** Make sure you ran `train.py` *before* compiling `make`. The C compiler needs the new `inference_weights.c` file.
- **Latency?** The inference engine runs in <10μs. If the loop is slow, check logging overhead.
