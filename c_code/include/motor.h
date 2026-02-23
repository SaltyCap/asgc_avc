#ifndef MOTOR_H
#define MOTOR_H

#include <pthread.h>
#include <stdint.h>
#include "common.h" // For OdometryState and NavigationController

// PWM configuration
#define PWM_CHIP_INDEX 2     // Usually 2 on Raspberry Pi 5
#define PWM_CHANNEL_LEFT 1   // GPIO 13
#define PWM_CHANNEL_RIGHT 0  // GPIO 12
#define PWM_PERIOD_NS 2500000
#define NEUTRAL_NS 1500000
#define FORWARD_MAX_NS 2000000
#define REVERSE_MAX_NS 1000000

typedef struct {
    int id;
    int pwm_duty_fd;
    int pwm_enable_fd;
    int last_pulse_ns;   // Last PWM pulse width sent (for logging)
    pthread_mutex_t lock;
} Motor;

typedef struct {
    int32_t total_counts;      // Accumulated encoder counts
    int16_t current_raw_angle; // Current 0-4095 angle from AS5600
    int16_t last_raw_angle;    // Previous raw angle for delta calculation

    int32_t target_counts;     // Target relative distance
    int32_t move_start_counts; // Total counts at start of current move
    int has_target;            // Non-zero when target tracking is active
    double ramp_start_time;    // Time when ramping started (0.0 = not started)
} EncoderState;

// Global arrays
extern Motor motors[2];
extern EncoderState encoders[2];

extern OdometryState odometry;
extern NavigationController nav_ctrl;

int pwm_init(void);
void pwm_cleanup(void);
void set_motor_pwm(int motor_id, int pulse_ns);

#endif
