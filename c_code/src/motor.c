#include "../include/motor.h"
#include "../include/common.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

Motor motors[2];
EncoderState encoders[2];
// Internal static variables
static int PWM_CHIP = -1;

static int find_pwm_chip(void) {
    char path[256];
    for (int i = 0; i < 10; i++) {
        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d", i);
        if (access(path, F_OK) == 0) return i;
    }
    return -1;
}

int pwm_init(void) {
    char path[256];
    int fd;
    int channels[2] = {PWM_CHANNEL_LEFT, PWM_CHANNEL_RIGHT};

    PWM_CHIP = find_pwm_chip();
    if (PWM_CHIP < 0) return -1;

    for (int i = 0; i < 2; i++) {
        motors[i].id = i;
        int channel = channels[i];

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d", PWM_CHIP, channel);
        if (access(path, F_OK) != 0) {
            snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", PWM_CHIP);
            fd = open(path, O_WRONLY);
            if (fd < 0) return -1;
            dprintf(fd, "%d", channel);
            close(fd);
            sleep_us(100000);
        }

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/period", PWM_CHIP, channel);
        fd = open(path, O_WRONLY);
        if (fd < 0) return -1;
        dprintf(fd, "%d", PWM_PERIOD_NS);
        close(fd);

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", PWM_CHIP, channel);
        motors[i].pwm_duty_fd = open(path, O_WRONLY);
        if (motors[i].pwm_duty_fd < 0) return -1;
        dprintf(motors[i].pwm_duty_fd, "%d", NEUTRAL_NS);

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/enable", PWM_CHIP, channel);
        motors[i].pwm_enable_fd = open(path, O_WRONLY);
        if (motors[i].pwm_enable_fd < 0) {
            close(motors[i].pwm_duty_fd);
            return -1;
        }
        dprintf(motors[i].pwm_enable_fd, "1");

        motors[i].last_pulse_ns = NEUTRAL_NS;
        motors[i].last_speed_update_time = 0;

        pthread_mutex_init(&motors[i].lock, NULL);
    }
    return 0;
}



void set_motor_speed(int motor_id, int speed_percent, int immediate) {
    if (speed_percent > 100) speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    // Convert target speed_percent to target_pulse_ns
    // Account for ESC deadband: forward starts at 1550us, reverse at 1450us
    int target_pulse_ns;
    if (speed_percent > 0) {
        // Map 0-100% to FORWARD_START_NS to FORWARD_MAX_NS
        target_pulse_ns = FORWARD_START_NS + (speed_percent * (FORWARD_MAX_NS - FORWARD_START_NS)) / 100;
    } else if (speed_percent < 0) {
        // Map 0-(-100%) to REVERSE_START_NS to REVERSE_MAX_NS
        target_pulse_ns = REVERSE_START_NS - (abs(speed_percent) * (REVERSE_START_NS - REVERSE_MAX_NS)) / 100;
    } else {
        target_pulse_ns = NEUTRAL_NS;
    }

    // Explicit Check: Clamp to absolute limits
    if (target_pulse_ns > FORWARD_MAX_NS) target_pulse_ns = FORWARD_MAX_NS;
    if (target_pulse_ns < REVERSE_MAX_NS) target_pulse_ns = REVERSE_MAX_NS;

    // Ramp rate limiting (Nanoseconds domain)
    // Limits the rate of change of the pulse width to prevent sudden jerks
    // 500,000 ns range / 3 seconds = ~166,667 ns/sec
    #define RAMP_NS_PER_SEC 166667.0

    double current_time = get_time_sec();
    double dt = current_time - motors[motor_id].last_speed_update_time;
    int current_pulse_ns = motors[motor_id].last_pulse_ns;

    if (!immediate && dt > 0 && motors[motor_id].last_speed_update_time > 0) {
        int diff = target_pulse_ns - current_pulse_ns;
        int max_change = (int)(RAMP_NS_PER_SEC * dt); 
        
        if (max_change < 1) max_change = 1; // Ensure some movement

        if (abs(diff) > max_change) {
            if (diff > 0) {
                current_pulse_ns += max_change;
            } else {
                current_pulse_ns -= max_change;
            }
        } else {
            current_pulse_ns = target_pulse_ns;
        }
    } else {
        // Immediate update or first run
        current_pulse_ns = target_pulse_ns;
    }
    
    // Save state
    motors[motor_id].last_pulse_ns = current_pulse_ns;
    motors[motor_id].last_speed_update_time = current_time;

    // Apply Output
    int final_output_ns = current_pulse_ns;



    if (motors[motor_id].pwm_duty_fd >= 0) {
        lseek(motors[motor_id].pwm_duty_fd, 0, SEEK_SET);
        dprintf(motors[motor_id].pwm_duty_fd, "%d", final_output_ns);
    }
    motors[motor_id].current_speed = speed_percent; // Store target for logging
}

void pwm_cleanup(void) {
    for (int i = 0; i < 2; i++) {
        if (motors[i].pwm_duty_fd >= 0) {
            lseek(motors[i].pwm_duty_fd, 0, SEEK_SET);
            dprintf(motors[i].pwm_duty_fd, "%d", NEUTRAL_NS);
            close(motors[i].pwm_duty_fd);
        }
        if (motors[i].pwm_enable_fd >= 0) {
            lseek(motors[i].pwm_enable_fd, 0, SEEK_SET);
            dprintf(motors[i].pwm_enable_fd, "0");
            close(motors[i].pwm_enable_fd);
        }
        pthread_mutex_destroy(&motors[i].lock);
    }
}

// Motor state accessor functions
int8_t get_left_motor_state(void) {
    int8_t state;
    pthread_mutex_lock(&motors[0].lock);
    state = encoders[0].motor_state;
    pthread_mutex_unlock(&motors[0].lock);
    return state;
}

int8_t get_right_motor_state(void) {
    int8_t state;
    pthread_mutex_lock(&motors[1].lock);
    state = encoders[1].motor_state;
    pthread_mutex_unlock(&motors[1].lock);
    return state;
}

int32_t get_left_rotation_count(void) {
    int32_t count;
    pthread_mutex_lock(&motors[0].lock);
    count = encoders[0].rotation_count;
    pthread_mutex_unlock(&motors[0].lock);
    return count;
}

int32_t get_right_rotation_count(void) {
    int32_t count;
    pthread_mutex_lock(&motors[1].lock);
    count = encoders[1].rotation_count;
    pthread_mutex_unlock(&motors[1].lock);
    return count;
}

int32_t get_left_position(void) {
    int32_t position;
    pthread_mutex_lock(&motors[0].lock);
    int32_t base = COUNTS_PER_REV * encoders[0].rotation_count;
    int32_t offset = encoders[0].current_raw_angle - encoders[0].start_raw_angle;
    position = base + offset;
    pthread_mutex_unlock(&motors[0].lock);
    return position;
}

int32_t get_right_position(void) {
    int32_t position;
    pthread_mutex_lock(&motors[1].lock);
    int32_t base = COUNTS_PER_REV * encoders[1].rotation_count;
    int32_t offset = encoders[1].current_raw_angle - encoders[1].start_raw_angle;
    position = base + offset;
    pthread_mutex_unlock(&motors[1].lock);
    return position;
}

