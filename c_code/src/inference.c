#include "../include/inference.h"
#include <math.h>

// Include the auto-generated weights and biases
#include "inference_weights.c"

// Helper: ReLU Activation
static inline float relu(float x) {
    return (x > 0.0f) ? x : 0.0f;
}

void run_inference(const float* inputs, float* outputs) {
    // ---------------------------------------------------------
    // Layer 1: Input -> Hidden 1
    // ---------------------------------------------------------
    float h1[sizeof(LAYER_1_BIAS) / sizeof(float)];
    int n_h1 = sizeof(LAYER_1_BIAS) / sizeof(float);
    int n_inputs = sizeof(LAYER_1_WEIGHTS) / sizeof(LAYER_1_WEIGHTS[0]);
    
    for (int i = 0; i < n_h1; i++) {
        float sum = LAYER_1_BIAS[i];
        for (int j = 0; j < n_inputs; j++) {
            sum += inputs[j] * LAYER_1_WEIGHTS[j][i];
        }
        h1[i] = relu(sum);
    }

    // ---------------------------------------------------------
    // Layer 2: Hidden 1 -> Hidden 2
    // ---------------------------------------------------------
    float h2[sizeof(LAYER_2_BIAS) / sizeof(float)];
    int n_h2 = sizeof(LAYER_2_BIAS) / sizeof(float);
    
    for (int i = 0; i < n_h2; i++) {
        float sum = LAYER_2_BIAS[i];
        for (int j = 0; j < n_h1; j++) {
            sum += h1[j] * LAYER_2_WEIGHTS[j][i];
        }
        h2[i] = relu(sum);
    }

    // ---------------------------------------------------------
    // Layer 3: Hidden 2 -> Output
    // ---------------------------------------------------------
    int n_out = sizeof(LAYER_3_BIAS) / sizeof(float);
    
    for (int i = 0; i < n_out; i++) {
        float sum = LAYER_3_BIAS[i];
        for (int j = 0; j < n_h2; j++) {
            sum += h2[j] * LAYER_3_WEIGHTS[j][i];
        }
        // Linear activation for output (regression)
        outputs[i] = sum;
    }
}
