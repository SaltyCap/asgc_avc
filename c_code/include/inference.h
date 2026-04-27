#ifndef INFERENCE_H
#define INFERENCE_H

/**
 * Run MLP Inference
 * 
 * Performs a forward pass through the hardcoded neural network.
 * 
 * @param inputs Array of normalized input features (Range 0.0 - 1.0)
 * @param outputs Array to store model output predictions
 *        (trained on normalized targets, but linear output may exceed 0.0-1.0)
 */
void run_inference(const float* inputs, float* outputs);

// Fitted MinMax scaler bounds exported by ml_training/train.py
extern const float FEATURE_MIN[5];
extern const float FEATURE_MAX[5];
extern const float TARGET_MIN[2];
extern const float TARGET_MAX[2];

#endif
