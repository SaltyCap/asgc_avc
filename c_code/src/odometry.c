#include "../include/odometry.h"
#include "../include/common.h"
#include "../include/runtime_state.h"
#include "../include/sensors.h"
#include <math.h>
#include <pthread.h>
#include <unistd.h>

// Encoder direction mapping. Flip to -1 if a wheel's counts move opposite
// the expected forward direction during bring-up.
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN 1

// Clamp odometry timestep to avoid integration spikes.
#define MAX_ODOM_DT_SEC 0.05

void update_encoder_rotation(EncoderState *enc, int16_t raw_angle, int motor_id) {
    if (enc->last_raw_angle < 0) {
        enc->last_raw_angle = raw_angle;
        enc->current_raw_angle = raw_angle;
        enc->total_counts = 0;
        return;
    }

    int32_t delta = raw_angle - enc->last_raw_angle;
    if (delta > COUNTS_PER_REV / 2) {
        delta -= COUNTS_PER_REV;
    } else if (delta < -COUNTS_PER_REV / 2) {
        delta += COUNTS_PER_REV;
    }

    int encoder_sign = (motor_id == 0) ? LEFT_ENCODER_SIGN : RIGHT_ENCODER_SIGN;
    enc->total_counts += encoder_sign * delta;
    enc->last_raw_angle = raw_angle;
    enc->current_raw_angle = raw_angle;
}

void* encoder_feedback_thread(void* arg) {
    (void)arg;
    const int target_period_us = 1000000 / 1000; // 1kHz encoder-focused loop

    while (running) {
        double loop_start = get_time_sec();

        // Read one synchronized sensor snapshot each cycle.
        SensorData sensors = read_all_sensors();
        int16_t left_angle = sensors.left_encoder;
        int16_t right_angle = sensors.right_encoder;

        // Update gyro data only when a fresh IMU sample is available.
        if (sensors.imu_valid) {
            pthread_mutex_lock(&imu_data_lock);
            current_gyro_rate = sensors.gyro_z;
            last_imu_sample_time = sensors.timestamp;
            pthread_mutex_unlock(&imu_data_lock);
        }

        if (left_angle >= 0) {
            pthread_mutex_lock(&motors[0].lock);
            update_encoder_rotation(&encoders[0], left_angle, 0);
            pthread_mutex_unlock(&motors[0].lock);
        }

        if (right_angle >= 0) {
            pthread_mutex_lock(&motors[1].lock);
            update_encoder_rotation(&encoders[1], right_angle, 1);
            pthread_mutex_unlock(&motors[1].lock);
        }

        update_odometry();
        int elapsed_us = (int)((get_time_sec() - loop_start) * 1e6);
        int remaining_us = target_period_us - elapsed_us;
        if (remaining_us > 0) {
            usleep(remaining_us);
        }
    }
    return NULL;
}

int32_t calculate_turn_counts(double degrees) {
    // Preserve sign so turn direction (CW vs CCW) follows the requested angle.
    double arc_length = (degrees / 360.0) * M_PI * WHEELBASE_INCHES;
    return (int32_t)(arc_length * COUNTS_PER_INCH);
}

// --- Encoder + Gyro Odometry ---
void update_odometry(void) {
    static int first_update = 1;

    pthread_mutex_lock(&state_lock);
    double current_time = get_time_sec();
    double dt = current_time - last_imu_time;
    if (dt < 0.0) dt = 0.0;
    if (dt > MAX_ODOM_DT_SEC) dt = MAX_ODOM_DT_SEC;
    last_imu_time = current_time;

    // Initialize tracking on first update to prevent position jump.
    if (first_update) {
        odometry.last_left_total = encoders[0].total_counts;
        odometry.last_right_total = encoders[1].total_counts;
        first_update = 0;
        pthread_mutex_unlock(&state_lock);
        return;
    }

    int32_t d_left = encoders[0].total_counts - odometry.last_left_total;
    int32_t d_right = encoders[1].total_counts - odometry.last_right_total;
    odometry.last_left_total = encoders[0].total_counts;
    odometry.last_right_total = encoders[1].total_counts;

    double dist_left = d_left / COUNTS_PER_FOOT;
    double dist_right = d_right / COUNTS_PER_FOOT;
    double center_dist = (dist_left + dist_right) / 2.0;

    // Get gyro rate and reject stale/noise samples.
    pthread_mutex_lock(&imu_data_lock);
    double gyro_rate = current_gyro_rate;
    double imu_age = current_time - last_imu_sample_time;
    pthread_mutex_unlock(&imu_data_lock);

    if (imu_age > 0.2) {
        gyro_rate = 0.0;
    }
    if (fabs(gyro_rate) < 0.25) {
        gyro_rate = 0.0;
    }

    // Integrate gyro heading every cycle regardless of encoder movement.
    // The gyro is the authoritative heading source; encoder counts are only
    // used for distance (x/y) integration, not heading.
    double delta_heading = gyro_rate * dt;

    double new_heading = odometry.heading + delta_heading;
    double avg_heading_rad = (odometry.heading + new_heading) / 2.0 * (M_PI / 180.0);

    odometry.x += center_dist * cos(avg_heading_rad);
    odometry.y += center_dist * sin(avg_heading_rad);
    odometry.heading = new_heading;

    while (odometry.heading >= 360.0) odometry.heading -= 360.0;
    while (odometry.heading < 0.0) odometry.heading += 360.0;

    pthread_mutex_unlock(&state_lock);
}
