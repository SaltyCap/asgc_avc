#include "../include/control_loop.h"
#include "../include/common.h"
#include "../include/logging.h"
#include "../include/motor.h"
#include "../include/odometry.h"
#include "../include/runtime_state.h"
#include "../include/inference.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define STALL_PROGRESS_COUNTS 20
#define STALL_TIMEOUT_SEC 3.0
#define CONTROL_LOOP_HZ 500
#define CONTROL_DT_MIN_SEC 0.0005
#define CONTROL_DT_MAX_SEC 0.05
#define ML_SWEEP_TURNS 5.0
#define ML_SWEEP_POINTS_PER_TURN 12
#define ML_SWEEP_WAYPOINTS ((int)(ML_SWEEP_TURNS * ML_SWEEP_POINTS_PER_TURN))

// Heading correction gain (counts of differential correction per degree of heading error).
// Increase to correct drift faster; decrease if it causes weaving.
#define HEADING_CORRECTION_GAIN 2.5
// Hard cap so a large heading error can't overwhelm the drive PID.
#define MAX_HEADING_CORRECTION_COUNTS 200
// Gyro-based turn: declare the turn complete when within this many degrees.
#define TURN_HEADING_DONE_DEG 1.5
// Estimated drive-leg duration used to convert the measured gyro rate dead zone
// into a heading-error dead zone (seconds).  Increase if legs are longer.
#define HEADING_CORRECTION_DRIVE_TIME_EST_SEC 8.0

typedef struct {
    double kp;
    double ki;
    double kd_velocity;
    double ka_accel;
    double integral_limit;
    double velocity_stop_threshold;
} WheelPidConfig;

typedef struct {
    double integral;
    double last_error;
    int initialized;
} WheelPidState;

static WheelPidConfig drive_pid_cfg = {
    .kp = 34.0,
    .ki = 2.2,
    .kd_velocity = 18.0,
    .ka_accel = 0.06,
    .integral_limit = 25000.0,
    .velocity_stop_threshold = 260.0
};

static WheelPidConfig turn_pid_cfg = {
    .kp = 60.0,
    .ki = 5.0,
    .kd_velocity = 30.0,
    .ka_accel = 0.08,
    .integral_limit = 40000.0,
    .velocity_stop_threshold = 300.0
};

static pthread_mutex_t pid_cfg_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t nav_controller_mode_lock = PTHREAD_MUTEX_INITIALIZER;
static NavControllerMode nav_controller_mode = NAV_CONTROLLER_PID;
static WheelPidState wheel_pid[2];

static int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static double clamp_double(double value, double min_value, double max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void reset_wheel_pid_state(int motor_id) {
    if (motor_id < 0 || motor_id >= 2) {
        return;
    }
    wheel_pid[motor_id].integral = 0.0;
    wheel_pid[motor_id].last_error = 0.0;
    wheel_pid[motor_id].initialized = 0;
}

void control_reset_pid_states(void) {
    reset_wheel_pid_state(0);
    reset_wheel_pid_state(1);
}

static double compute_wheel_pid_output(int motor_id,
                                       int32_t error_counts,
                                       double velocity_counts_per_sec,
                                       double acceleration_counts_per_sec2,
                                       const WheelPidConfig* cfg,
                                       double dt_sec) {
    if (!cfg || motor_id < 0 || motor_id >= 2) {
        return 0.0;
    }

    WheelPidState* pid = &wheel_pid[motor_id];
    if (!pid->initialized) {
        pid->last_error = (double)error_counts;
        pid->integral = 0.0;
        pid->initialized = 1;
    }

    if ((((double)error_counts) >= 0.0 && pid->last_error < 0.0) ||
        (((double)error_counts) <= 0.0 && pid->last_error > 0.0)) {
        pid->integral = 0.0;
    }

    pid->integral += ((double)error_counts) * dt_sec;
    pid->integral = clamp_double(pid->integral, -cfg->integral_limit, cfg->integral_limit);

    double effort = (cfg->kp * (double)error_counts) +
                    (cfg->ki * pid->integral) -
                    (cfg->kd_velocity * velocity_counts_per_sec) -
                    (cfg->ka_accel * acceleration_counts_per_sec2);

    pid->last_error = (double)error_counts;
    return effort;
}

static double normalize_angle_diff(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

int control_set_nav_controller_mode(NavControllerMode mode) {
    if (mode != NAV_CONTROLLER_PID && mode != NAV_CONTROLLER_ML) {
        return 0;
    }
    pthread_mutex_lock(&nav_controller_mode_lock);
    nav_controller_mode = mode;
    pthread_mutex_unlock(&nav_controller_mode_lock);
    return 1;
}

NavControllerMode control_get_nav_controller_mode(void) {
    pthread_mutex_lock(&nav_controller_mode_lock);
    NavControllerMode mode = nav_controller_mode;
    pthread_mutex_unlock(&nav_controller_mode_lock);
    return mode;
}

static int compute_ml_control_pwm(int32_t target_l,
                                  int32_t actual_l,
                                  int32_t target_r,
                                  int32_t actual_r,
                                  int* out_pwm_l,
                                  int* out_pwm_r) {
    if (!out_pwm_l || !out_pwm_r) {
        return 0;
    }

    float inputs[5];
    float outputs[2];
    float gyro = 0.0f;

    pthread_mutex_lock(&imu_data_lock);
    gyro = (float)current_gyro_rate;
    pthread_mutex_unlock(&imu_data_lock);

    float raw_inputs[5] = {
        (float)target_l,
        (float)actual_l,
        (float)target_r,
        (float)actual_r,
        gyro
    };

    for (int i = 0; i < 5; i++) {
        float denom = FEATURE_MAX[i] - FEATURE_MIN[i];
        if (fabsf(denom) < 1e-6f) {
            inputs[i] = 0.0f;
        } else {
            inputs[i] = (raw_inputs[i] - FEATURE_MIN[i]) / denom;
        }
        if (inputs[i] < 0.0f) inputs[i] = 0.0f;
        if (inputs[i] > 1.0f) inputs[i] = 1.0f;
    }

    run_inference(inputs, outputs);

    for (int i = 0; i < 2; i++) {
        if (!isfinite(outputs[i])) {
            outputs[i] = 0.5f;
        }
        if (outputs[i] < 0.0f) outputs[i] = 0.0f;
        if (outputs[i] > 1.0f) outputs[i] = 1.0f;
    }

    float pwm_l_f = TARGET_MIN[0] + outputs[0] * (TARGET_MAX[0] - TARGET_MIN[0]);
    float pwm_r_f = TARGET_MIN[1] + outputs[1] * (TARGET_MAX[1] - TARGET_MIN[1]);
    *out_pwm_l = (int)(pwm_l_f + 0.5f);
    *out_pwm_r = (int)(pwm_r_f + 0.5f);
    return 1;
}

static WheelPidConfig get_pid_config_copy(PidProfile profile) {
    WheelPidConfig cfg;
    pthread_mutex_lock(&pid_cfg_lock);
    cfg = (profile == PID_PROFILE_TURN) ? turn_pid_cfg : drive_pid_cfg;
    pthread_mutex_unlock(&pid_cfg_lock);
    return cfg;
}

int control_set_pid_gains(PidProfile profile,
                          double kp,
                          double ki,
                          double kd_velocity,
                          double ka_accel,
                          double velocity_stop_threshold) {
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd_velocity) ||
        !isfinite(ka_accel) || !isfinite(velocity_stop_threshold)) {
        return 0;
    }
    if (kp < 0.0 || ki < 0.0 || kd_velocity < 0.0 || ka_accel < 0.0 ||
        velocity_stop_threshold < 0.0) {
        return 0;
    }

    pthread_mutex_lock(&pid_cfg_lock);
    WheelPidConfig* cfg = (profile == PID_PROFILE_TURN) ? &turn_pid_cfg : &drive_pid_cfg;
    cfg->kp = kp;
    cfg->ki = ki;
    cfg->kd_velocity = kd_velocity;
    cfg->ka_accel = ka_accel;
    cfg->velocity_stop_threshold = velocity_stop_threshold;
    pthread_mutex_unlock(&pid_cfg_lock);

    return 1;
}

int control_get_pid_gains(PidProfile profile,
                          double* kp,
                          double* ki,
                          double* kd_velocity,
                          double* ka_accel,
                          double* velocity_stop_threshold) {
    if (!kp || !ki || !kd_velocity || !ka_accel || !velocity_stop_threshold) {
        return 0;
    }

    pthread_mutex_lock(&pid_cfg_lock);
    const WheelPidConfig* cfg = (profile == PID_PROFILE_TURN) ? &turn_pid_cfg : &drive_pid_cfg;
    *kp = cfg->kp;
    *ki = cfg->ki;
    *kd_velocity = cfg->kd_velocity;
    *ka_accel = cfg->ka_accel;
    *velocity_stop_threshold = cfg->velocity_stop_threshold;
    pthread_mutex_unlock(&pid_cfg_lock);

    return 1;
}

static int ml_bucket_coordinates(BallColor color, double* out_x, double* out_y) {
    if (!out_x || !out_y) {
        return 0;
    }

    switch (color) {
        case BALL_COLOR_RED:
            *out_x = BUCKET_RED_X;
            *out_y = BUCKET_RED_Y;
            return 1;
        case BALL_COLOR_YELLOW:
            *out_x = BUCKET_YELLOW_X;
            *out_y = BUCKET_YELLOW_Y;
            return 1;
        case BALL_COLOR_BLUE:
            *out_x = BUCKET_BLUE_X;
            *out_y = BUCKET_BLUE_Y;
            return 1;
        case BALL_COLOR_GREEN:
            *out_x = BUCKET_GREEN_X;
            *out_y = BUCKET_GREEN_Y;
            return 1;
        case BALL_COLOR_NONE:
        default:
            return 0;
    }
}

static void stop_motion_targets_locked(void) {
    for (int i = 0; i < 2; i++) {
        pthread_mutex_lock(&motors[i].lock);
        encoders[i].move_start_counts = encoders[i].total_counts;
        encoders[i].target_counts = 0;
        encoders[i].has_target = 0;
        encoders[i].ramp_start_time = 0.0;
        set_motor_pwm(i, NEUTRAL_NS);
        pthread_mutex_unlock(&motors[i].lock);
        reset_wheel_pid_state(i);
    }
}

static int ml_set_next_pickup_waypoint_locked(void) {
    if (nav_ctrl.ml_sweep_waypoint_index >= ML_SWEEP_WAYPOINTS) {
        return 0;
    }

    int waypoint = nav_ctrl.ml_sweep_waypoint_index;
    nav_ctrl.ml_sweep_waypoint_index++;

    double progress = (double)(waypoint + 1) / (double)ML_SWEEP_WAYPOINTS;
    double theta = progress * (ML_SWEEP_TURNS * 2.0 * M_PI);
    double radius = progress * ML_PICKUP_CIRCLE_RADIUS_FT;

    nav_ctrl.target_x = COURSE_CENTER_X + radius * cos(theta);
    nav_ctrl.target_y = COURSE_CENTER_Y + radius * sin(theta);
    nav_ctrl.is_bucket_target = 0;
    nav_ctrl.bucket_x = 0.0;
    nav_ctrl.bucket_y = 0.0;
    nav_ctrl.is_backing_up = 0;
    nav_ctrl.target_distance = 0.0;
    nav_ctrl.state = NAV_GOTO;
    return 1;
}

static int ml_try_dispatch_bucket_locked(void) {
    double bucket_x = 0.0;
    double bucket_y = 0.0;

    if (!nav_ctrl.ml_test_enabled) {
        return 0;
    }
    if (nav_ctrl.ml_stage != ML_STAGE_WAIT_BALL &&
        nav_ctrl.ml_stage != ML_STAGE_PICKUP_SWEEP) {
        return 0;
    }
    if (!nav_ctrl.ml_ball_acquired) {
        return 0;
    }
    if (!ml_bucket_coordinates(nav_ctrl.ml_ball_color, &bucket_x, &bucket_y)) {
        return 0;
    }

    nav_ctrl.target_x = bucket_x;
    nav_ctrl.target_y = bucket_y;
    nav_ctrl.is_bucket_target = 1;
    nav_ctrl.bucket_x = bucket_x;
    nav_ctrl.bucket_y = bucket_y;
    nav_ctrl.state = NAV_GOTO;
    nav_ctrl.ml_stage = ML_STAGE_TO_BUCKET;
    nav_ctrl.ml_sweep_waypoint_index = 0;
    return 1;
}

void* coordinated_control_thread(void* arg) {
    (void)arg;
    const int target_period_us = 1000000 / CONTROL_LOOP_HZ;
    double stall_last_progress_time = 0.0;
    int32_t stall_last_progress_sum = 0;
    double last_control_time = get_time_sec();

    printf("Control loop running at %dHz (logging %dHz)\n", CONTROL_LOOP_HZ, LOG_RATE_HZ);

    while (running) {
        double loop_start = get_time_sec();
        double current_time = loop_start;

        pthread_mutex_lock(&state_lock);

        switch (nav_ctrl.state) {
            case NAV_IDLE:
                stall_last_progress_time = 0.0;
                stall_last_progress_sum = 0;
                control_reset_pid_states();
                last_control_time = current_time;
                break;

            case NAV_GOTO: {
                if (nav_ctrl.ml_test_enabled &&
                    nav_ctrl.ml_stage == ML_STAGE_PICKUP_SWEEP &&
                    nav_ctrl.ml_ball_acquired) {
                    stall_last_progress_time = 0.0;
                    stall_last_progress_sum = 0;

                    if (ml_try_dispatch_bucket_locked()) {
                        printf("ML_TARGET_BUCKET\n");
                    } else {
                        nav_ctrl.ml_stage = ML_STAGE_WAIT_BALL;
                        nav_ctrl.state = NAV_IDLE;
                        printf("ML_BALL_ACQUIRED_WAIT_COLOR\n");
                    }
                    fflush(stdout);
                    printf("STATUS %.2f %.2f %.2f %d\n",
                           odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                    break;
                }

                // Determine next step: turn or drive.
                double dx = nav_ctrl.target_x - odometry.x;
                double dy = nav_ctrl.target_y - odometry.y;
                double target_heading = atan2(dy, dx) * 180.0 / M_PI;
                if (target_heading < 0) target_heading += 360.0;

                double heading_diff = normalize_angle_diff(target_heading - odometry.heading);
                double distance = sqrt(dx * dx + dy * dy);
                double arrival_tolerance = nav_ctrl.is_bucket_target ? 1.5 : 0.5;

                if (distance < arrival_tolerance) {
                    if (nav_ctrl.is_bucket_target) {
                        printf("BUCKET_ZONE\n");
                        fflush(stdout);
                        nav_ctrl.state = NAV_BUCKET_ROTATE;
                        stall_last_progress_time = 0.0;
                        stall_last_progress_sum = 0;
                        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    } else if (nav_ctrl.ml_test_enabled && nav_ctrl.ml_stage == ML_STAGE_TO_CENTER) {
                        nav_ctrl.ml_stage = ML_STAGE_PICKUP_SWEEP;
                        nav_ctrl.ml_sweep_waypoint_index = 0;
                        nav_ctrl.is_bucket_target = 0;
                        nav_ctrl.bucket_x = 0.0;
                        nav_ctrl.bucket_y = 0.0;
                        nav_ctrl.is_backing_up = 0;
                        stall_last_progress_time = 0.0;
                        stall_last_progress_sum = 0;

                        printf("ML_CENTER_REACHED\n");
                        if (ml_try_dispatch_bucket_locked()) {
                            printf("ML_TARGET_BUCKET\n");
                        } else if (ml_set_next_pickup_waypoint_locked()) {
                            printf("ML_PICKUP_SWEEP_START\n");
                        } else {
                            nav_ctrl.ml_stage = ML_STAGE_WAIT_BALL;
                            nav_ctrl.state = NAV_IDLE;
                            printf("ML_WAIT_BALL\n");
                        }
                        fflush(stdout);
                        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    } else if (nav_ctrl.ml_test_enabled && nav_ctrl.ml_stage == ML_STAGE_PICKUP_SWEEP) {
                        stall_last_progress_time = 0.0;
                        stall_last_progress_sum = 0;

                        if (ml_try_dispatch_bucket_locked()) {
                            printf("ML_TARGET_BUCKET\n");
                        } else if (ml_set_next_pickup_waypoint_locked()) {
                            printf("ML_PICKUP_SWEEP_STEP %d\n", nav_ctrl.ml_sweep_waypoint_index);
                        } else {
                            nav_ctrl.ml_stage = ML_STAGE_WAIT_BALL;
                            nav_ctrl.state = NAV_IDLE;
                            printf("ML_PICKUP_SWEEP_COMPLETE\n");
                            printf("ML_WAIT_BALL\n");
                        }
                        fflush(stdout);
                        printf("STATUS %.2f %.2f %.2f %d\n",
                               odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    } else {
                        printf("ARRIVED\n");
                        fflush(stdout);
                        nav_ctrl.state = NAV_IDLE;
                        stall_last_progress_time = 0.0;
                        stall_last_progress_sum = 0;
                        control_reset_pid_states();
                        last_control_time = current_time;
                        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    }
                } else if (fabs(heading_diff) > 2.0) {
                    nav_ctrl.state = NAV_TURNING;
                    nav_ctrl.target_heading = target_heading;

                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].move_start_counts = encoders[0].total_counts;
                    encoders[0].target_counts = -calculate_turn_counts(heading_diff);
                    encoders[0].has_target = 1;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].move_start_counts = encoders[1].total_counts;
                    encoders[1].target_counts = calculate_turn_counts(heading_diff);
                    encoders[1].has_target = 1;
                    pthread_mutex_unlock(&motors[1].lock);

                    stall_last_progress_time = current_time;
                    stall_last_progress_sum = 0;
                    control_reset_pid_states();
                    last_control_time = current_time;
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                } else {
                    nav_ctrl.state = NAV_DRIVING;
                    nav_ctrl.target_distance = distance;
                    nav_ctrl.target_heading = target_heading; // saved for heading correction

                    int32_t counts = (int32_t)(distance * COUNTS_PER_FOOT);
                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].move_start_counts = encoders[0].total_counts;
                    encoders[0].target_counts = counts;
                    encoders[0].has_target = 1;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].move_start_counts = encoders[1].total_counts;
                    encoders[1].target_counts = counts;
                    encoders[1].has_target = 1;
                    pthread_mutex_unlock(&motors[1].lock);

                    stall_last_progress_time = current_time;
                    stall_last_progress_sum = 0;
                    control_reset_pid_states();
                    last_control_time = current_time;
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                }
                break;
            }

            case NAV_TURNING:
            case NAV_DRIVING: {
                int left_done = 0, right_done = 0;
                int left_active = 0, right_active = 0;
                int32_t left_relative_counts = 0;
                int32_t right_relative_counts = 0;
                int next_left_pwm = NEUTRAL_NS;
                int next_right_pwm = NEUTRAL_NS;

                if (nav_ctrl.ml_test_enabled &&
                    nav_ctrl.ml_stage == ML_STAGE_PICKUP_SWEEP &&
                    nav_ctrl.ml_ball_acquired) {
                    stop_motion_targets_locked();
                    stall_last_progress_time = 0.0;
                    stall_last_progress_sum = 0;

                    if (ml_try_dispatch_bucket_locked()) {
                        printf("ML_TARGET_BUCKET\n");
                    } else {
                        nav_ctrl.ml_stage = ML_STAGE_WAIT_BALL;
                        nav_ctrl.state = NAV_IDLE;
                        printf("ML_BALL_ACQUIRED_WAIT_COLOR\n");
                    }
                    control_reset_pid_states();
                    last_control_time = current_time;
                    fflush(stdout);
                    printf("STATUS %.2f %.2f %.2f %d\n",
                           odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                    break;
                }

                // Apply user speed scaling on top of PWM limits.
                int speed_percent = g_speed_percent;
                if (speed_percent <= 0) speed_percent = 1;
                if (speed_percent > 100) speed_percent = 100;
                double speed_scale = speed_percent / 100.0;
                int max_pwm_ns = NEUTRAL_NS + (int)((g_max_pwm_ns - NEUTRAL_NS) * speed_scale);
                int min_pwm_ns = NEUTRAL_NS - (int)((NEUTRAL_NS - g_min_pwm_ns) * speed_scale);
                double dt = current_time - last_control_time;
                if (dt < CONTROL_DT_MIN_SEC) dt = CONTROL_DT_MIN_SEC;
                if (dt > CONTROL_DT_MAX_SEC) dt = CONTROL_DT_MAX_SEC;
                last_control_time = current_time;
                NavControllerMode controller_mode = control_get_nav_controller_mode();
                int use_ml_controller = (controller_mode == NAV_CONTROLLER_ML);
                int32_t left_target_counts = 0;
                int32_t right_target_counts = 0;
                int32_t left_actual_counts = 0;
                int32_t right_actual_counts = 0;
                int32_t left_error_counts = 0;
                int32_t right_error_counts = 0;
                double left_velocity = 0.0;
                double right_velocity = 0.0;
                double left_acceleration = 0.0;
                double right_acceleration = 0.0;
                int left_has_target_snapshot = 0;
                int right_has_target_snapshot = 0;
                int32_t left_target_snapshot = 0;
                int32_t right_target_snapshot = 0;

                pthread_mutex_lock(&motors[0].lock);
                left_has_target_snapshot = encoders[0].has_target;
                left_target_snapshot = encoders[0].target_counts;
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                right_has_target_snapshot = encoders[1].has_target;
                right_target_snapshot = encoders[1].target_counts;
                pthread_mutex_unlock(&motors[1].lock);

                // Treat opposite-sign wheel targets as turning even if state sync lags briefly.
                int opposite_target_signs =
                    left_has_target_snapshot &&
                    right_has_target_snapshot &&
                    ((left_target_snapshot > 0 && right_target_snapshot < 0) ||
                     (left_target_snapshot < 0 && right_target_snapshot > 0));
                PidProfile profile =
                    (nav_ctrl.state == NAV_TURNING || opposite_target_signs)
                        ? PID_PROFILE_TURN
                        : PID_PROFILE_DRIVE;
                WheelPidConfig pid_cfg = get_pid_config_copy(profile);

                // --- Heading correction for straight driving (Option B) ---
                // Only active during NAV_DRIVING; turned off during turns.
                // The dead zone is computed from the gyro noise floor measured at startup:
                //   dead_zone_deg = g_gyro_rate_dead_zone * estimated_drive_time
                // This prevents gyro bias that sneaks through the rate filter in odometry.c
                // from causing the car to weave on a straight leg.
                // The coordinate system is the same as the course map: 0deg = East (+X),
                // matching atan2(dy,dx), so there is no frame mismatch.
                // positive heading_err => car drifted CCW => steer CW (right)
                //   => subtract from left error, add to right error
                int32_t heading_correction_counts = 0;
                if (nav_ctrl.state == NAV_DRIVING && !use_ml_controller) {
                    double heading_err = normalize_angle_diff(
                        nav_ctrl.target_heading - odometry.heading);
                    // Compute dead zone from measured gyro noise floor.
                    double dead_zone_deg = g_gyro_rate_dead_zone * HEADING_CORRECTION_DRIVE_TIME_EST_SEC;
                    if (dead_zone_deg < 1.0) dead_zone_deg = 1.0;
                    if (dead_zone_deg > 5.0) dead_zone_deg = 5.0;
                    // Dead zone: ignore small errors that are within gyro drift range.
                    if (fabs(heading_err) > dead_zone_deg) {
                        double active_err = heading_err - (heading_err > 0 ?
                             dead_zone_deg : -dead_zone_deg);
                        double raw_corr = active_err * HEADING_CORRECTION_GAIN;
                        if (raw_corr >  MAX_HEADING_CORRECTION_COUNTS) raw_corr =  MAX_HEADING_CORRECTION_COUNTS;
                        if (raw_corr < -MAX_HEADING_CORRECTION_COUNTS) raw_corr = -MAX_HEADING_CORRECTION_COUNTS;
                        heading_correction_counts = (int32_t)lround(raw_corr);
                        
                        // If the robot has overshot and the PID is commanding reverse thrust,
                        // steering kinematics are inverted. We must invert the differential 
                        // correction, otherwise the robot will steer the wrong way and spin out.
                        if (left_error_counts + right_error_counts < 0) {
                            heading_correction_counts = -heading_correction_counts;
                        }
                    }
                }

                // --- Gyro-based heading feedback for turning ---
                // Use the gyro-integrated heading (odometry.heading) as the primary
                // error signal during NAV_TURNING so that wheel slip doesn't cause
                // the turn to stop short.  The error is converted to equivalent
                // encoder counts using the same sign convention as the initial target
                // assignment (left wheel = -counts, right wheel = +counts).
                int gyro_turn_done = 0;
                int32_t gyro_turn_left_error  = 0;
                int32_t gyro_turn_right_error = 0;
                if ((nav_ctrl.state == NAV_TURNING || opposite_target_signs) && !use_ml_controller) {
                    double heading_err = normalize_angle_diff(
                        nav_ctrl.target_heading - odometry.heading);
                    int32_t arc_counts = calculate_turn_counts(heading_err);
                    gyro_turn_left_error  = -arc_counts;
                    gyro_turn_right_error =  arc_counts;

                    if (fabs(heading_err) < TURN_HEADING_DONE_DEG) {
                        double vel_l = 0.0, vel_r = 0.0;
                        pthread_mutex_lock(&motors[0].lock);
                        vel_l = encoders[0].velocity_counts_per_sec;
                        pthread_mutex_unlock(&motors[0].lock);
                        pthread_mutex_lock(&motors[1].lock);
                        vel_r = encoders[1].velocity_counts_per_sec;
                        pthread_mutex_unlock(&motors[1].lock);

                        if (fabs(vel_l) < pid_cfg.velocity_stop_threshold &&
                            fabs(vel_r) < pid_cfg.velocity_stop_threshold) {
                            gyro_turn_done = 1;
                        }
                    }
                }

                pthread_mutex_lock(&motors[0].lock);
                if (encoders[0].has_target) {
                    int32_t current_relative = encoders[0].total_counts - encoders[0].move_start_counts;
                    left_relative_counts = current_relative;
                    left_target_counts = encoders[0].target_counts;
                    left_actual_counts = current_relative;
                    left_active = 1;
                    left_error_counts = left_target_counts - current_relative;
                    left_velocity = encoders[0].velocity_counts_per_sec;
                    left_acceleration = encoders[0].acceleration_counts_per_sec2;

                    if (!use_ml_controller) {
                        if (nav_ctrl.state == NAV_TURNING || opposite_target_signs) {
                            if (gyro_turn_done) {
                                next_left_pwm = NEUTRAL_NS;
                                encoders[0].has_target = 0;
                                encoders[0].ramp_start_time = 0.0;
                                left_done = 1;
                                reset_wheel_pid_state(0);
                            } else {
                                // Gyro-based error: actual rotation measured, not wheel distance.
                                double pid_effort_ns = compute_wheel_pid_output(
                                    0, gyro_turn_left_error, left_velocity, left_acceleration, &pid_cfg, dt);
                                next_left_pwm = NEUTRAL_NS + (int)lround(pid_effort_ns);
                            }
                        } else {
                            // Drive mode: encoder error + heading correction.
                            int32_t left_pid_error = left_error_counts - heading_correction_counts;
                            double pid_effort_ns = compute_wheel_pid_output(
                                0, left_pid_error, left_velocity, left_acceleration, &pid_cfg, dt);
                            next_left_pwm = NEUTRAL_NS + (int)lround(pid_effort_ns);
                        }
                    }

                    // Stop check for drive mode only (turn mode is handled above).
                    if (!(nav_ctrl.state == NAV_TURNING || opposite_target_signs)) {
                        if (abs(left_error_counts) < STOP_THRESHOLD &&
                            fabs(left_velocity) < pid_cfg.velocity_stop_threshold) {
                            next_left_pwm = NEUTRAL_NS;
                            encoders[0].has_target = 0;
                            encoders[0].ramp_start_time = 0.0;
                            left_done = 1;
                            reset_wheel_pid_state(0);
                        }
                    }
                } else {
                    next_left_pwm = NEUTRAL_NS;
                    left_done = 1;
                    reset_wheel_pid_state(0);
                }
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                if (encoders[1].has_target) {
                    int32_t current_relative = encoders[1].total_counts - encoders[1].move_start_counts;
                    right_relative_counts = current_relative;
                    right_target_counts = encoders[1].target_counts;
                    right_actual_counts = current_relative;
                    right_active = 1;
                    right_error_counts = right_target_counts - current_relative;
                    right_velocity = encoders[1].velocity_counts_per_sec;
                    right_acceleration = encoders[1].acceleration_counts_per_sec2;

                    if (!use_ml_controller) {
                        if (nav_ctrl.state == NAV_TURNING || opposite_target_signs) {
                            if (gyro_turn_done) {
                                next_right_pwm = NEUTRAL_NS;
                                encoders[1].has_target = 0;
                                encoders[1].ramp_start_time = 0.0;
                                right_done = 1;
                                reset_wheel_pid_state(1);
                            } else {
                                // Gyro-based error: actual rotation measured, not wheel distance.
                                double pid_effort_ns = compute_wheel_pid_output(
                                    1, gyro_turn_right_error, right_velocity, right_acceleration, &pid_cfg, dt);
                                next_right_pwm = NEUTRAL_NS + (int)lround(pid_effort_ns);
                            }
                        } else {
                            // Drive mode: encoder error + heading correction.
                            int32_t right_pid_error = right_error_counts + heading_correction_counts;
                            double pid_effort_ns = compute_wheel_pid_output(
                                1, right_pid_error, right_velocity, right_acceleration, &pid_cfg, dt);
                            next_right_pwm = NEUTRAL_NS + (int)lround(pid_effort_ns);
                        }
                    }

                    // Stop check for drive mode only (turn mode is handled above).
                    if (!(nav_ctrl.state == NAV_TURNING || opposite_target_signs)) {
                        if (abs(right_error_counts) < STOP_THRESHOLD &&
                            fabs(right_velocity) < pid_cfg.velocity_stop_threshold) {
                            next_right_pwm = NEUTRAL_NS;
                            encoders[1].has_target = 0;
                            encoders[1].ramp_start_time = 0.0;
                            right_done = 1;
                            reset_wheel_pid_state(1);
                        }
                    }
                } else {
                    next_right_pwm = NEUTRAL_NS;
                    right_done = 1;
                    reset_wheel_pid_state(1);
                }
                pthread_mutex_unlock(&motors[1].lock);

                if (use_ml_controller &&
                    (left_active || right_active) &&
                    !(left_done && right_done)) {
                    int ml_pwm_l = NEUTRAL_NS;
                    int ml_pwm_r = NEUTRAL_NS;
                    if (compute_ml_control_pwm(
                            left_target_counts,
                            left_actual_counts,
                            right_target_counts,
                            right_actual_counts,
                            &ml_pwm_l,
                            &ml_pwm_r)) {
                        next_left_pwm = (left_active && !left_done) ? ml_pwm_l : NEUTRAL_NS;
                        next_right_pwm = (right_active && !right_done) ? ml_pwm_r : NEUTRAL_NS;
                    } else {
                        if (left_active && !left_done) {
                            next_left_pwm = NEUTRAL_NS;
                        }
                        if (right_active && !right_done) {
                            next_right_pwm = NEUTRAL_NS;
                        }
                    }
                }

                // Stall recovery: if progress does not change for too long, stop and re-plan.
                if ((left_active || right_active) && !(left_done && right_done)) {
                    int32_t progress_sum = abs(left_relative_counts) + abs(right_relative_counts);

                    if (stall_last_progress_time == 0.0) {
                        stall_last_progress_time = current_time;
                        stall_last_progress_sum = progress_sum;
                    }

                    if ((progress_sum - stall_last_progress_sum) >= STALL_PROGRESS_COUNTS) {
                        stall_last_progress_time = current_time;
                        stall_last_progress_sum = progress_sum;
                    } else if ((current_time - stall_last_progress_time) > STALL_TIMEOUT_SEC) {
                        pthread_mutex_lock(&motors[0].lock);
                        encoders[0].has_target = 0;
                        encoders[0].ramp_start_time = 0.0;
                        pthread_mutex_unlock(&motors[0].lock);

                        pthread_mutex_lock(&motors[1].lock);
                        encoders[1].has_target = 0;
                        encoders[1].ramp_start_time = 0.0;
                        pthread_mutex_unlock(&motors[1].lock);

                        next_left_pwm = NEUTRAL_NS;
                        next_right_pwm = NEUTRAL_NS;
                        left_done = 1;
                        right_done = 1;

                        if (nav_ctrl.is_backing_up) {
                            nav_ctrl.state = NAV_IDLE;
                            nav_ctrl.is_backing_up = 0;
                            nav_ctrl.is_bucket_target = 0;
                            nav_ctrl.bucket_x = 0.0;
                            nav_ctrl.bucket_y = 0.0;
                            printf("ARRIVED_BUMP\n");
                        } else if (nav_ctrl.is_bucket_target && nav_ctrl.state == NAV_TURNING) {
                            // Stalled during the 180 degree bucket rotation. Proceed to backup.
                            double dx = nav_ctrl.bucket_x - odometry.x;
                            double dy = nav_ctrl.bucket_y - odometry.y;
                            double dist_to_bucket = sqrt(dx * dx + dy * dy);

                            if (dist_to_bucket < 2.0) {
                                nav_ctrl.state = NAV_BUCKET_BACKUP;
                            } else {
                                nav_ctrl.state = NAV_GOTO;
                            }
                        } else {
                            nav_ctrl.state = NAV_GOTO;
                        }

                        stall_last_progress_time = current_time;
                        stall_last_progress_sum = 0;
                        control_reset_pid_states();
                        last_control_time = current_time;

                        printf("STALL_RECOVERY\n");
                        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    }
                }

                next_left_pwm = clamp_int(next_left_pwm, min_pwm_ns, max_pwm_ns);
                next_right_pwm = clamp_int(next_right_pwm, min_pwm_ns, max_pwm_ns);

                pthread_mutex_lock(&motors[0].lock);
                set_motor_pwm(0, next_left_pwm);
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                set_motor_pwm(1, next_right_pwm);
                pthread_mutex_unlock(&motors[1].lock);

                if (left_done && right_done && nav_ctrl.state != NAV_GOTO) {
                    stall_last_progress_time = 0.0;
                    stall_last_progress_sum = 0;
                    if (nav_ctrl.is_bucket_target && nav_ctrl.state == NAV_TURNING) {
                        double dx = nav_ctrl.bucket_x - odometry.x;
                        double dy = nav_ctrl.bucket_y - odometry.y;
                        double dist_to_bucket = sqrt(dx * dx + dy * dy);

                        if (dist_to_bucket < 2.0) {
                            nav_ctrl.state = NAV_BUCKET_BACKUP;
                        } else {
                            nav_ctrl.state = NAV_GOTO;
                        }
                    } else if (nav_ctrl.is_backing_up) {
                        nav_ctrl.is_backing_up = 0;
                        nav_ctrl.is_bucket_target = 0;
                        nav_ctrl.bucket_x = 0.0;
                        nav_ctrl.bucket_y = 0.0;
                        nav_ctrl.state = NAV_IDLE;
                        printf("ARRIVED\n");
                    } else {
                        if (nav_ctrl.isolated_turn) {
                            nav_ctrl.isolated_turn = 0;
                            nav_ctrl.state = NAV_IDLE;
                        } else {
                            nav_ctrl.state = NAV_GOTO;
                        }
                    }
                    control_reset_pid_states();
                    last_control_time = current_time;

                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                }
                break;
            }

            case NAV_BUCKET_ROTATE: {
                double target_heading = odometry.heading + 180.0;
                if (target_heading >= 360.0) target_heading -= 360.0;
                double heading_diff = normalize_angle_diff(target_heading - odometry.heading);

                pthread_mutex_lock(&motors[0].lock);
                encoders[0].move_start_counts = encoders[0].total_counts;
                encoders[0].target_counts = -calculate_turn_counts(heading_diff);
                encoders[0].has_target = 1;
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                encoders[1].move_start_counts = encoders[1].total_counts;
                encoders[1].target_counts = calculate_turn_counts(heading_diff);
                encoders[1].has_target = 1;
                pthread_mutex_unlock(&motors[1].lock);

                nav_ctrl.state = NAV_TURNING;
                nav_ctrl.target_heading = target_heading;
                stall_last_progress_time = current_time;
                stall_last_progress_sum = 0;
                control_reset_pid_states();
                last_control_time = current_time;

                printf("ROTATING_180\n");
                printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                fflush(stdout);
                break;
            }

            case NAV_BUCKET_BACKUP: {
                double dx = nav_ctrl.bucket_x - odometry.x;
                double dy = nav_ctrl.bucket_y - odometry.y;
                double current_distance = sqrt(dx * dx + dy * dy);
                double backup_distance = current_distance - 0.25;

                if (backup_distance > 0.1) {
                    int32_t counts = -(int32_t)(backup_distance * COUNTS_PER_FOOT);

                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].move_start_counts = encoders[0].total_counts;
                    encoders[0].target_counts = counts;
                    encoders[0].has_target = 1;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].move_start_counts = encoders[1].total_counts;
                    encoders[1].target_counts = counts;
                    encoders[1].has_target = 1;
                    pthread_mutex_unlock(&motors[1].lock);

                    nav_ctrl.state = NAV_DRIVING;
                    nav_ctrl.target_distance = backup_distance;
                    nav_ctrl.is_backing_up = 1;
                    stall_last_progress_time = current_time;
                    stall_last_progress_sum = 0;
                    control_reset_pid_states();
                    last_control_time = current_time;

                    printf("BACKING_UP %.2f ft\n", backup_distance);
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                } else {
                    printf("ARRIVED\n");
                    nav_ctrl.is_bucket_target = 0;
                    nav_ctrl.bucket_x = 0.0;
                    nav_ctrl.bucket_y = 0.0;

                    if (nav_ctrl.ml_test_enabled && nav_ctrl.ml_stage == ML_STAGE_TO_BUCKET) {
                        nav_ctrl.ml_ball_acquired = 0;
                        nav_ctrl.ml_ball_color = BALL_COLOR_NONE;
                        nav_ctrl.ml_stage = ML_STAGE_TO_CENTER;
                        nav_ctrl.ml_sweep_waypoint_index = 0;
                        nav_ctrl.target_x = COURSE_CENTER_X;
                        nav_ctrl.target_y = COURSE_CENTER_Y;
                        nav_ctrl.state = NAV_GOTO;

                        printf("ML_DROP_COMPLETE\n");
                        printf("ML_STAGE to_center\n");
                    } else {
                        nav_ctrl.ml_sweep_waypoint_index = 0;
                        nav_ctrl.state = NAV_IDLE;
                    }

                    stall_last_progress_time = 0.0;
                    stall_last_progress_sum = 0;
                    control_reset_pid_states();
                    last_control_time = current_time;

                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                }
                break;
            }

            case NAV_ML: {
                pthread_mutex_lock(&motors[0].lock);
                int32_t target_l = encoders[0].target_counts;
                int32_t actual_l = encoders[0].total_counts - encoders[0].move_start_counts;
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                int32_t target_r = encoders[1].target_counts;
                int32_t actual_r = encoders[1].total_counts - encoders[1].move_start_counts;
                pthread_mutex_unlock(&motors[1].lock);

                int pwm_l = NEUTRAL_NS;
                int pwm_r = NEUTRAL_NS;
                compute_ml_control_pwm(target_l, actual_l, target_r, actual_r, &pwm_l, &pwm_r);

                // Keep ML outputs inside operator-configured limits and speed scaling.
                int speed_percent = g_speed_percent;
                if (speed_percent <= 0) speed_percent = 1;
                if (speed_percent > 100) speed_percent = 100;
                float speed_scale = speed_percent / 100.0f;
                int max_pwm_ns = NEUTRAL_NS + (int)((g_max_pwm_ns - NEUTRAL_NS) * speed_scale);
                int min_pwm_ns = NEUTRAL_NS - (int)((NEUTRAL_NS - g_min_pwm_ns) * speed_scale);
                pwm_l = clamp_int(pwm_l, min_pwm_ns, max_pwm_ns);
                pwm_r = clamp_int(pwm_r, min_pwm_ns, max_pwm_ns);

                pthread_mutex_lock(&motors[0].lock);
                set_motor_pwm(0, pwm_l);
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                set_motor_pwm(1, pwm_r);
                pthread_mutex_unlock(&motors[1].lock);
                break;
            }

        }

        static int status_counter = 0;
        if (status_counter++ % 10 == 0) {
            printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
            fflush(stdout);
        }
        pthread_mutex_unlock(&state_lock);

        int elapsed_us = (int)((get_time_sec() - loop_start) * 1e6);
        int remaining_us = target_period_us - elapsed_us;
        if (remaining_us > 0) {
            usleep(remaining_us);
        }
    }
    return NULL;
}
