#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    // PID state
    double integral;
    int32_t last_error;

    // Current state
    int32_t total_counts;
    int32_t target_counts;
    int16_t current_raw_angle;
    int16_t last_raw_angle;
    int16_t start_raw_angle;

    // Control
    int has_target;
} PIDController;

// PID Parameters
#define KP 0.8
#define KI 0.02
#define KD 0.1
#define MAX_INTEGRAL 1000

void update_total_counts(PIDController *pid, int16_t raw_angle);
int calculate_pid_speed(PIDController *pid, int32_t current_counts, double dt);

#endif
