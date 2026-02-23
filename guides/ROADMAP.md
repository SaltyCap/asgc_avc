# Roadmap — ML Motor Control

## Phase 1: High-Speed Logging & Data Collection
**Goal**: Enable 20kHz logging and collect training data.
- [x] Optimize `logging.c` for 20kHz writes (use large RAM buffer).
- [x] Update `control_loop.c` to trigger logging at 20kHz.
- [x] Verify timing accuracy.
- [ ] Collect dataset from manual/PID driving.

## Phase 2: Model Architecture & Training
**Goal**: Design and train the ML model.
- [x] Define model architecture in Python (Keras).
- [x] Preprocess collected data.
- [x] Train model to predict PWM.
- [x] Convert model to C++ compatible format (TFLite/ONNX).

## Phase 3: Inference Engine Integration
**Goal**: Integrate Model into C control loop.
- [x] Add inference.c/h (Simple MLP runtime).
- [x] Integrate inference into control_loop.c.
- [x] Add ML control mode to main.c (command_processor.c).
- [x] Verify inference speed.

## Phase 4: Validation & Tuning
**Goal**: Tuning and real-world testing.
- [ ] Bench test with safe PWM limits.
- [ ] Compare ML performance vs PID.
- [ ] Tune model hyperparameters if needed.
