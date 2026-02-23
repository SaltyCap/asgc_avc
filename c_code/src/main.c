#include "../include/command_processor.h"
#include "../include/common.h"
#include "../include/control_loop.h"
#include "../include/i2c.h"
#include "../include/imu.h"
#include "../include/logging.h"
#include "../include/motor.h"
#include "../include/odometry.h"
#include "../include/runtime_state.h"
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

int main(void) {
    // Set up signal handlers for graceful shutdown.
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize I2C bus for sensors.
    if (i2c_init() < 0) {
        fprintf(stderr, "ERROR: I2C init failed\n");
        return 1;
    }

    // Initialize PWM for motor control.
    if (pwm_init() < 0) {
        fprintf(stderr, "ERROR: PWM init failed\n");
        i2c_cleanup();
        return 1;
    }

    // Initialize telemetry logging system.
    init_log_system();

    // Initialize IMU (optional, system continues without it).
    if (imu_init() < 0) {
        fprintf(stderr, "WARNING: IMU init failed (check wiring to I2C3). Continuing without IMU.\n");
    } else {
        imu_calibrate(500); // 2.5 second calibration for better accuracy.
    }

    last_imu_time = get_time_sec();
    last_imu_sample_time = last_imu_time;

    // Initialize encoders.
    for (int i = 0; i < 2; i++) {
        encoders[i].total_counts = 0;
        encoders[i].current_raw_angle = 0;
        encoders[i].last_raw_angle = -1; // Marks first sample as uninitialized.
        encoders[i].target_counts = 0;
        encoders[i].has_target = 0;
    }

    fprintf(stderr, "Arming ESCs...\n");
    fflush(stderr);
    sleep(2);

    printf("READY coordinated\n");
    fflush(stdout);

    // Start 20kHz logging thread
    if (start_logging_thread() < 0) {
        fprintf(stderr, "ERROR: Failed to start logging thread\n");
    }

    pthread_t feedback_thread, control_thread, input_thread;
    pthread_create(&feedback_thread, NULL, encoder_feedback_thread, NULL);
    pthread_create(&control_thread, NULL, coordinated_control_thread, NULL);
    pthread_create(&input_thread, NULL, command_input_thread, NULL);

    pthread_join(feedback_thread, NULL);
    pthread_join(control_thread, NULL);
    // fgets() can block indefinitely on stdin; cancel input thread so
    // signal-driven shutdown cannot hang waiting for command input.
    pthread_cancel(input_thread);
    pthread_join(input_thread, NULL);

    if (shutdown_signal_received) {
        fprintf(stderr, "Shutdown requested by signal, flushing logs.\n");
    }
    stop_logging_thread();
    dump_log();

    pwm_cleanup();
    imu_cleanup();
    i2c_cleanup();

    return 0;
}
