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
    int i;

    PWM_CHIP = find_pwm_chip();
    if (PWM_CHIP < 0) return -1;

    for (i = 0; i < 2; i++) {
        motors[i].id = i;
        motors[i].pwm_duty_fd = -1;
        motors[i].pwm_enable_fd = -1;
        motors[i].last_pulse_ns = NEUTRAL_NS;
        pthread_mutex_init(&motors[i].lock, NULL);
    }

    for (i = 0; i < 2; i++) {
        int channel = channels[i];

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d", PWM_CHIP, channel);
        if (access(path, F_OK) != 0) {
            snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", PWM_CHIP);
            fd = open(path, O_WRONLY);
            if (fd < 0) goto fail;
            dprintf(fd, "%d", channel);
            close(fd);
            sleep_us(100000);
        }

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/period", PWM_CHIP, channel);
        fd = open(path, O_WRONLY);
        if (fd < 0) goto fail;
        dprintf(fd, "%d", PWM_PERIOD_NS);
        close(fd);

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", PWM_CHIP, channel);
        motors[i].pwm_duty_fd = open(path, O_WRONLY);
        if (motors[i].pwm_duty_fd < 0) goto fail;
        dprintf(motors[i].pwm_duty_fd, "%d", NEUTRAL_NS);

        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/enable", PWM_CHIP, channel);
        motors[i].pwm_enable_fd = open(path, O_WRONLY);
        if (motors[i].pwm_enable_fd < 0) {
            close(motors[i].pwm_duty_fd);
            motors[i].pwm_duty_fd = -1;
            goto fail;
        }
        dprintf(motors[i].pwm_enable_fd, "1");
    }
    return 0;

fail:
    pwm_cleanup();
    return -1;
}



void set_motor_pwm(int motor_id, int pulse_ns) {
    if (motor_id < 0 || motor_id >= 2) {
        return;
    }

    // Explicit Check: Clamp to absolute limits
    if (pulse_ns > FORWARD_MAX_NS) pulse_ns = FORWARD_MAX_NS;
    if (pulse_ns < REVERSE_MAX_NS) pulse_ns = REVERSE_MAX_NS;

    motors[motor_id].last_pulse_ns = pulse_ns;

    // Write to PWM hardware
    if (motors[motor_id].pwm_duty_fd >= 0) {
        lseek(motors[motor_id].pwm_duty_fd, 0, SEEK_SET);
        dprintf(motors[motor_id].pwm_duty_fd, "%d", pulse_ns);
    }
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
