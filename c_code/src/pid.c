#include "../include/pid.h"
#include "../include/common.h"
#include <stdlib.h>
#include <math.h>

void update_total_counts(PIDController *pid, int16_t raw_angle) {
    if (pid->last_raw_angle >= 0) {
        if (pid->last_raw_angle > 3500 && raw_angle < 500) {
            pid->total_counts += COUNTS_PER_REV;
        } else if (pid->last_raw_angle < 500 && raw_angle > 3500) {
            pid->total_counts -= COUNTS_PER_REV;
        }
    }
    pid->last_raw_angle = raw_angle;
}

int calculate_pid_speed(PIDController *pid, int32_t current_counts, double dt) {
    int32_t error = pid->target_counts - current_counts;

    // Proportional term
    double p_term = KP * error;

    // Integral term (with anti-windup)
    pid->integral += error * dt;
    if (pid->integral > MAX_INTEGRAL) pid->integral = MAX_INTEGRAL;
    if (pid->integral < -MAX_INTEGRAL) pid->integral = -MAX_INTEGRAL;
    double i_term = KI * pid->integral;

    // Derivative term
    double d_term = 0;
    if (dt > 0) {
        d_term = KD * (error - pid->last_error) / dt;
    }
    pid->last_error = error;

    // Combined PID output
    double output = p_term + i_term + d_term;

    // Convert to speed percentage (-100 to 100)
    int speed = (int)output;
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // Minimum speed to overcome friction (deadband)
    if (abs(speed) > 0 && abs(speed) < 15) {
        speed = (speed > 0) ? 15 : -15;
    }

    return speed;
}
