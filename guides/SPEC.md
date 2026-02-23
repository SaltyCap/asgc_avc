# SPEC.md — ML Motor Control Integration

Status: **FINALIZED**

## 1. Executive Summary
Develop and integrate a Machine Learning (ML) model to control two PWM values for motors, replacing or augmenting the existing PID control. The system must support high-speed (20kHz) time-synced data logging for training and operate the inference loop at 60Hz or faster.

## 2. Goals
1.  **High-Speed Data Logging**: Capture telemetry (encoders, PWM, IMU) at 20kHz to create a high-fidelity dataset.
2.  **ML Model Control**: specific neural network to predict PWM values based on state.
3.  **Performance**: Inference loop must run >60Hz on Raspberry Pi 5.
4.  **Safety**: Maintain safety stops and stall detection.

## 3. Requirements

### 3.1 Data Logging
- **Frequency**: 20kHz (Time-synced).
- **Data Points**:
  - Timestamp (us precision)
  - Left/Right Encoder Counts (Raw & Relative)
  - Left/Right PWM Output
  - IMU Data (Gyro Z)
  - Target State (Desired Velocity/Position)
- **Storage**: Buffer in RAM, dump to CSV on stop.

### 3.2 Machine Learning Model
- **Framework**: TensorFlow / Keras (Training), TensorFlow Lite Micro or ONNX Runtime (Inference in C++).
- **Inputs**:
  - Command Word (e.g., GOTO, STOP, TURN)
  - Current Location (x, y, heading)
  - Desired Location (target_x, target_y)
  - Current PWM Values (feedback)
  - Current Velocity/Encoder Delta
- **Outputs**:
  - Left Motor PWM
  - Right Motor PWM
- **Latency**: < 16ms per inference.

### 3.3 System Integration
- **Platform**: Raspberry Pi 5
- **Language**: C/C++ for control loop and inference.
- **Existing Codebase**: `c_code/asgc_motor_control`

## 4. Constraints
- Must run on Raspberry Pi 5.
- Must coexist with existing `web_server`.
- Safety features (emergency stop) must override ML control.

## 5. Non-Goals
- replacing the high-level path planning (A*), only low-level motor control.
