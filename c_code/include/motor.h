#ifndef MOTOR_H
#define MOTOR_H

#include <pthread.h>

// PWM Configuration
#define PWM_CHANNEL_LEFT 0   // GPIO 12
#define PWM_CHANNEL_RIGHT 1  // GPIO 13
#define PWM_PERIOD_NS 2500000
#define NEUTRAL_NS 1400000
#define FORWARD_MAX_NS 2000000
#define REVERSE_MAX_NS 1000000

typedef struct {
    int id;
    int pwm_duty_fd;
    int pwm_enable_fd;
    int current_speed;
    pthread_mutex_t lock;
} Motor;

// Global motor array
extern Motor motors[2];

int pwm_init(void);
void pwm_cleanup(void);
void set_motor_speed(int motor_id, int speed_percent);

#endif
