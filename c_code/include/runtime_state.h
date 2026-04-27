#ifndef RUNTIME_STATE_H
#define RUNTIME_STATE_H

#include <pthread.h>
#include <signal.h>
#include "common.h"

// Process lifecycle state
extern volatile sig_atomic_t running;
extern volatile sig_atomic_t shutdown_signal_received;

// Shared controller and odometry state
extern pthread_mutex_t state_lock;
extern OdometryState odometry;
extern NavigationController nav_ctrl;

// Shared IMU data used by odometry/logging
extern pthread_mutex_t imu_data_lock;
extern double current_gyro_rate;
extern double last_imu_sample_time;
extern double last_imu_time;

// Runtime speed/PWM limits updated from web commands
extern int g_min_pwm_ns;
extern int g_max_pwm_ns;
extern int g_speed_percent;

// Gyro dead zone measured at startup calibration (deg/sec).
// Used by odometry.c (rate filter) and control_loop.c (heading correction).
extern double g_gyro_rate_dead_zone;

void signal_handler(int sig);

#endif
