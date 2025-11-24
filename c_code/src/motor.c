#include "../include/motor.h"
#include "../include/common.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

Motor motors[2];
int PWM_CHIP = -1;

int find_pwm_chip(void) {
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

        pthread_mutex_init(&motors[i].lock, NULL);
    }
    return 0;
}

void set_motor_speed(int motor_id, int speed_percent) {
    if (speed_percent > 100) speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    int pulse_ns;
    if (speed_percent > 0) {
        pulse_ns = NEUTRAL_NS + (speed_percent * (FORWARD_MAX_NS - NEUTRAL_NS)) / 100;
    } else if (speed_percent < 0) {
        pulse_ns = NEUTRAL_NS - (abs(speed_percent) * (NEUTRAL_NS - REVERSE_MAX_NS)) / 100;
    } else {
        pulse_ns = NEUTRAL_NS;
    }

    lseek(motors[motor_id].pwm_duty_fd, 0, SEEK_SET);
    dprintf(motors[motor_id].pwm_duty_fd, "%d", pulse_ns);

    motors[motor_id].current_speed = speed_percent;
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
