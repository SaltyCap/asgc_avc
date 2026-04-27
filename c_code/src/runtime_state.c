#include "../include/runtime_state.h"

// Process lifecycle state
volatile sig_atomic_t running = 1;
volatile sig_atomic_t shutdown_signal_received = 0;

// Shared controller and odometry state
pthread_mutex_t state_lock = PTHREAD_MUTEX_INITIALIZER;
OdometryState odometry = {START_X, START_Y, START_HEADING, 0, 0};
NavigationController nav_ctrl = {
    .state = NAV_IDLE,
    .target_x = 0.0,
    .target_y = 0.0,
    .target_heading = 0.0,
    .target_distance = 0.0,
    .is_bucket_target = 0,
    .bucket_x = 0.0,
    .bucket_y = 0.0,
    .is_backing_up = 0,
    .ml_test_enabled = 0,
    .ml_stage = ML_STAGE_INACTIVE,
    .ml_ball_acquired = 0,
    .ml_ball_color = BALL_COLOR_NONE,
    .ml_sweep_waypoint_index = 0,
};

// Shared IMU data used by odometry/logging
pthread_mutex_t imu_data_lock = PTHREAD_MUTEX_INITIALIZER;
double current_gyro_rate = 0.0;
double last_imu_sample_time = 0.0;
double last_imu_time = 0.0;

// Runtime speed/PWM limits updated from web commands
int g_min_pwm_ns = 1400000;  // Minimum PWM pulse width (1400us)
int g_max_pwm_ns = 1600000;  // Maximum PWM pulse width (1600us)
int g_speed_percent = 25;    // Navigation speed scale (0-100)

// Gyro dead zone computed at startup calibration (deg/sec).
// Safe default; overwritten by imu_calibrate() on startup.
double g_gyro_rate_dead_zone = 0.5;

void signal_handler(int sig) {
    (void)sig;
    running = 0;
    shutdown_signal_received = 1;
}
