#include "../include/common.h"
#include "../include/motor.h"
#include "../include/i2c.h"
#include "../include/pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

volatile int running = 1;

// PID Controllers for each motor
PIDController pids[2];

// Coordinated movement state
typedef struct {
    int active;
    int32_t left_target;
    int32_t right_target;
    double speed_factor;
    pthread_mutex_t lock;
} CoordinatedMove;

CoordinatedMove coord_move = {.active = 0};

void signal_handler(int sig) {
    (void)sig;
    running = 0;
}

// --- Coordinated Control Thread ---
void* coordinated_control_thread(void* arg) {
    (void)arg;
    double last_time = get_time_sec();
    int sleep_us = 1000000 / 1000; // 1kHz

    while (running) {
        double current_time = get_time_sec();
        double dt = current_time - last_time;
        last_time = current_time;

        pthread_mutex_lock(&coord_move.lock);

        if (coord_move.active) {
            int left_done = 0, right_done = 0;

            // Left motor (0)
            pthread_mutex_lock(&motors[0].lock);
            if (pids[0].has_target) {
                int32_t left_current = pids[0].total_counts +
                                       (pids[0].current_raw_angle - pids[0].start_raw_angle);
                int32_t left_error = pids[0].target_counts - left_current;

                if (abs(left_error) <= STOP_THRESHOLD) {
                    set_motor_speed(0, 0);
                    pids[0].has_target = 0;
                    left_done = 1;
                } else {
                    int speed = calculate_pid_speed(&pids[0], left_current, dt);
                    speed = (int)(speed * coord_move.speed_factor);
                    set_motor_speed(0, speed);
                }
            } else {
                left_done = 1;
            }
            pthread_mutex_unlock(&motors[0].lock);

            // Right motor (1)
            pthread_mutex_lock(&motors[1].lock);
            if (pids[1].has_target) {
                int32_t right_current = pids[1].total_counts +
                                        (pids[1].current_raw_angle - pids[1].start_raw_angle);
                int32_t right_error = pids[1].target_counts - right_current;

                if (abs(right_error) <= STOP_THRESHOLD) {
                    set_motor_speed(1, 0);
                    pids[1].has_target = 0;
                    right_done = 1;
                } else {
                    int speed = calculate_pid_speed(&pids[1], right_current, dt);
                    speed = (int)(speed * coord_move.speed_factor);
                    set_motor_speed(1, speed);
                }
            } else {
                right_done = 1;
            }
            pthread_mutex_unlock(&motors[1].lock);

            if (left_done && right_done) {
                coord_move.active = 0;
                printf("COORDINATED_COMPLETE\n");
                fflush(stdout);
            }
        }

        pthread_mutex_unlock(&coord_move.lock);
        usleep(sleep_us);
    }
    return NULL;
}

// --- Encoder feedback thread ---
void* encoder_feedback_thread(void* arg) {
    (void)arg;
    int sleep_us = 50; // 20kHz

    while (running) {
        // Read both encoders as close together as possible for synchronized data
        int16_t left_angle = read_raw_angle(0);   // Left encoder (0x40)
        int16_t right_angle = read_raw_angle(1);  // Right encoder (0x1B)

        // Process left motor encoder
        if (left_angle >= 0) {
            pthread_mutex_lock(&motors[0].lock);
            pids[0].current_raw_angle = left_angle;
            update_total_counts(&pids[0], left_angle);

            int32_t current_counts = pids[0].total_counts +
                                    (left_angle - pids[0].start_raw_angle);

            printf("ENCODER 0 %d %d\n", current_counts, left_angle);
            pthread_mutex_unlock(&motors[0].lock);
        }

        // Process right motor encoder
        if (right_angle >= 0) {
            pthread_mutex_lock(&motors[1].lock);
            pids[1].current_raw_angle = right_angle;
            update_total_counts(&pids[1], right_angle);

            int32_t current_counts = pids[1].total_counts +
                                    (right_angle - pids[1].start_raw_angle);

            printf("ENCODER 1 %d %d\n", current_counts, right_angle);
            pthread_mutex_unlock(&motors[1].lock);
        }

        fflush(stdout);
        usleep(sleep_us);
    }
    return NULL;
}

// --- Command processing ---
void start_coordinated_move(int32_t left_counts, int32_t right_counts, double speed_factor) {
    pthread_mutex_lock(&coord_move.lock);

    // Set targets
    pthread_mutex_lock(&motors[0].lock);
    pids[0].target_counts = left_counts;
    pids[0].has_target = 1;
    pids[0].total_counts = 0;
    pids[0].start_raw_angle = pids[0].current_raw_angle;
    pids[0].integral = 0;
    pids[0].last_error = left_counts;
    pthread_mutex_unlock(&motors[0].lock);

    pthread_mutex_lock(&motors[1].lock);
    pids[1].target_counts = right_counts;
    pids[1].has_target = 1;
    pids[1].total_counts = 0;
    pids[1].start_raw_angle = pids[1].current_raw_angle;
    pids[1].integral = 0;
    pids[1].last_error = right_counts;
    pthread_mutex_unlock(&motors[1].lock);

    coord_move.left_target = left_counts;
    coord_move.right_target = right_counts;
    coord_move.speed_factor = speed_factor;
    coord_move.active = 1;

    pthread_mutex_unlock(&coord_move.lock);

    printf("OK coordinated L:%d R:%d\n", left_counts, right_counts);
    fflush(stdout);
}

void process_command(char* cmd) {
    cmd[strcspn(cmd, "\n")] = 0;

    if (strncasecmp(cmd, "drive", 5) == 0) {
        int32_t counts;
        if (sscanf(cmd + 5, "%d", &counts) == 1) {
            start_coordinated_move(counts, counts, 1.0);
        }
    }
    else if (strncasecmp(cmd, "turn", 4) == 0) {
        int32_t counts;
        if (sscanf(cmd + 4, "%d", &counts) == 1) {
            start_coordinated_move(counts, -counts, 0.8);
        }
    }
    else if (strncasecmp(cmd, "arc", 3) == 0) {
        int32_t left, right;
        double speed = 1.0;
        int parsed = sscanf(cmd + 3, "%d %d %lf", &left, &right, &speed);
        if (parsed >= 2) {
            start_coordinated_move(left, right, speed);
        }
    }
    else if (strncasecmp(cmd, "stop", 4) == 0) {
        pthread_mutex_lock(&coord_move.lock);
        coord_move.active = 0;
        for (int i = 0; i < 2; i++) {
            pthread_mutex_lock(&motors[i].lock);
            pids[i].has_target = 0;
            set_motor_speed(i, 0);
            pthread_mutex_unlock(&motors[i].lock);
        }
        pthread_mutex_unlock(&coord_move.lock);
        printf("OK stopall\n");
        fflush(stdout);
    }
    else if (strcasecmp(cmd, "q") == 0) {
        running = 0;
        printf("OK quit\n");
        fflush(stdout);
    }
    // Direct PWM control for tank steering: pwm <left_speed> <right_speed>
    // Speed values are -100 to 100 (percent)
    else if (strncasecmp(cmd, "pwm", 3) == 0) {
        int left_speed, right_speed;
        if (sscanf(cmd + 3, "%d %d", &left_speed, &right_speed) == 2) {
            // Disable coordinated control when using direct PWM
            pthread_mutex_lock(&coord_move.lock);
            coord_move.active = 0;
            pthread_mutex_unlock(&coord_move.lock);

            // Disable PID targets
            pthread_mutex_lock(&motors[0].lock);
            pids[0].has_target = 0;
            pthread_mutex_unlock(&motors[0].lock);

            pthread_mutex_lock(&motors[1].lock);
            pids[1].has_target = 0;
            pthread_mutex_unlock(&motors[1].lock);

            // Set direct PWM speeds
            set_motor_speed(0, left_speed);
            set_motor_speed(1, right_speed);

            printf("OK pwm L:%d R:%d\n", left_speed, right_speed);
            fflush(stdout);
        }
    }
    // Raw pulse width control: pulse <left_ns> <right_ns>
    // Values are pulse width in nanoseconds (e.g., 1000000 to 2000000)
    else if (strncasecmp(cmd, "pulse", 5) == 0) {
        int left_ns, right_ns;
        if (sscanf(cmd + 5, "%d %d", &left_ns, &right_ns) == 2) {
            // Disable coordinated control
            pthread_mutex_lock(&coord_move.lock);
            coord_move.active = 0;
            pthread_mutex_unlock(&coord_move.lock);

            // Disable PID targets
            pthread_mutex_lock(&motors[0].lock);
            pids[0].has_target = 0;
            pthread_mutex_unlock(&motors[0].lock);

            pthread_mutex_lock(&motors[1].lock);
            pids[1].has_target = 0;
            pthread_mutex_unlock(&motors[1].lock);

            // Clamp pulse widths to valid range
            if (left_ns < REVERSE_MAX_NS) left_ns = REVERSE_MAX_NS;
            if (left_ns > FORWARD_MAX_NS) left_ns = FORWARD_MAX_NS;
            if (right_ns < REVERSE_MAX_NS) right_ns = REVERSE_MAX_NS;
            if (right_ns > FORWARD_MAX_NS) right_ns = FORWARD_MAX_NS;

            // Write pulse widths directly
            lseek(motors[0].pwm_duty_fd, 0, SEEK_SET);
            dprintf(motors[0].pwm_duty_fd, "%d", left_ns);
            lseek(motors[1].pwm_duty_fd, 0, SEEK_SET);
            dprintf(motors[1].pwm_duty_fd, "%d", right_ns);

            printf("OK pulse L:%d R:%d\n", left_ns, right_ns);
            fflush(stdout);
        }
    }
    // Simple manual control (legacy support)
    else if (cmd[0] == '1' || cmd[0] == '2') {
         // Not fully implemented in this refactor for brevity,
         // but could be added if manual individual motor control is needed.
         // For now, we focus on coordinated control.
    }
    else if (cmd[0] == 'r') {
        // Manual revs
    }
}

void* command_input_thread(void* arg) {
    (void)arg;
    char buffer[256];
    while (running && fgets(buffer, sizeof(buffer), stdin) != NULL) {
        process_command(buffer);
    }
    running = 0;
    return NULL;
}

int main(void) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (i2c_init() < 0) {
        fprintf(stderr, "ERROR: I2C init failed\n");
        return 1;
    }

    if (pwm_init() < 0) {
        fprintf(stderr, "ERROR: PWM init failed\n");
        i2c_cleanup();
        return 1;
    }

    pthread_mutex_init(&coord_move.lock, NULL);

    fprintf(stderr, "Arming ESCs...\n");
    fflush(stderr);
    sleep(2);

    printf("READY coordinated\n");
    fflush(stdout);

    pthread_t feedback_thread, control_thread, input_thread;

    pthread_create(&feedback_thread, NULL, encoder_feedback_thread, NULL);
    pthread_create(&control_thread, NULL, coordinated_control_thread, NULL);
    pthread_create(&input_thread, NULL, command_input_thread, NULL);

    pthread_join(input_thread, NULL);
    pthread_join(feedback_thread, NULL);
    pthread_join(control_thread, NULL);

    pwm_cleanup();
    i2c_cleanup();

    return 0;
}
