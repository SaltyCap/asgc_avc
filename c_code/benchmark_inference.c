#include <stdio.h>
#include <time.h>
#include "include/inference.h"

// Mock implementation of inference.c details if needed, but we link against the object file.
// We just need the header.

int main() {
    float inputs[5] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    float outputs[2];
    
    printf("Benchmarking Inference Engine...\n");
    
    clock_t start = clock();
    int iterations = 1000000;
    
    for (int i = 0; i < iterations; i++) {
        // Change inputs slightly to avoid compiler optimizing away
        inputs[0] = (float)(i % 100) / 100.0f;
        run_inference(inputs, outputs);
    }
    
    clock_t end = clock();
    double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    
    printf("Ran %d iterations in %.6f seconds\n", iterations, cpu_time_used);
    printf("Average latency: %.6f us per inference\n", (cpu_time_used * 1000000.0) / iterations);
    
    if ((cpu_time_used / iterations) < 0.001) {
        printf("PASS: Latency < 1ms\n");
    } else {
        printf("FAIL: Latency >= 1ms\n");
    }
    
    printf("Sample Output: %f %f\n", outputs[0], outputs[1]);
    
    return 0;
}
