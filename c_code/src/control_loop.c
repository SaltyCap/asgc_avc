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

// Motor ramping time constants.
// Using smooth ramps prevents mechanical stress and improves control stability.
#define RAMP_UP_TIME 1.0
#define DECEL_ZONE_FEET 1.0
#define DECEL_ZONE_COUNTS ((int32_t)(DECEL_ZONE_FEET * COUNTS_PER_FOOT))
#define STRAIGHT_SYNC_KP_NS_PER_COUNT 18
#define STRAIGHT_SYNC_MAX_NS 25000
#define STRAIGHT_SYNC_DEADBAND_COUNTS 60
#define STALL_PROGRESS_COUNTS 20
#define STALL_TIMEOUT_SEC 3.0
#define CONTROL_LOOP_HZ 500
#define ML_SWEEP_TURNS 5.0
#define ML_SWEEP_POINTS_PER_TURN 12
#define ML_SWEEP_WAYPOINTS ((int)(ML_SWEEP_TURNS * ML_SWEEP_POINTS_PER_TURN))

static int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static double normalize_angle_diff(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
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

    printf("Control loop running at %dHz (logging %dHz)\n", CONTROL_LOOP_HZ, LOG_RATE_HZ);

    while (running) {
        double loop_start = get_time_sec();
        double current_time = loop_start;

        pthread_mutex_lock(&state_lock);

        switch (nav_ctrl.state) {
            case NAV_IDLE:
                stall_last_progress_time = 0.0;
                stall_last_progress_sum = 0;
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
                double arrival_tolerance = nav_ctrl.is_bucket_target ? 1.5 : 2.0;

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
                        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    }
                } else if (fabs(heading_diff) > 5.0) {
                    nav_ctrl.state = NAV_TURNING;
                    nav_ctrl.target_heading = target_heading;

                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].move_start_counts = encoders[0].total_counts;
                    encoders[0].target_counts = calculate_turn_counts(heading_diff);
                    encoders[0].has_target = 1;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].move_start_counts = encoders[1].total_counts;
                    encoders[1].target_counts = -calculate_turn_counts(heading_diff);
                    encoders[1].has_target = 1;
                    pthread_mutex_unlock(&motors[1].lock);

                    stall_last_progress_time = current_time;
                    stall_last_progress_sum = 0;
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                } else {
                    nav_ctrl.state = NAV_DRIVING;
                    nav_ctrl.target_distance = distance;
                    nav_ctrl.target_heading = target_heading;  // Gyro correction reference.

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
                int32_t left_target_counts = 0;
                int32_t right_target_counts = 0;
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

                // --- NAV_TURNING: PWM is driven by encoder direction but completion
                //     is determined by gyro heading reaching nav_ctrl.target_heading. ---
                if (nav_ctrl.state == NAV_TURNING) {
                    // Check gyro-based heading error to decide if turn is done.
                    double gyro_heading_err = normalize_angle_diff(
                        nav_ctrl.target_heading - odometry.heading);

                    if (fabs(gyro_heading_err) <= 2.0) {
                        // Target heading reached — stop both motors immediately.
                        pthread_mutex_lock(&motors[0].lock);
                        encoders[0].has_target = 0;
                        encoders[0].ramp_start_time = 0.0;
                        set_motor_pwm(0, NEUTRAL_NS);
                        pthread_mutex_unlock(&motors[0].lock);

                        pthread_mutex_lock(&motors[1].lock);
                        encoders[1].has_target = 0;
                        encoders[1].ramp_start_time = 0.0;
                        set_motor_pwm(1, NEUTRAL_NS);
                        pthread_mutex_unlock(&motors[1].lock);

                        left_done = right_done = 1;
                        stall_last_progress_time = 0.0;
                        stall_last_progress_sum = 0;

                        if (nav_ctrl.is_bucket_target) {
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

                        printf("TURN_DONE gyro_hdg=%.2f target=%.2f\n",
                               odometry.heading, nav_ctrl.target_heading);
                        printf("STATUS %.2f %.2f %.2f %d\n",
                               odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                        break;  // Exit this state case immediately.
                    }

                    // Turn still in progress — scale PWM based on remaining heading error.
                    // Use encoder targets only for PWM direction (set during NAV_GOTO).
                    double turn_scale = fabs(gyro_heading_err) / 40.0;
                    if (turn_scale > 1.0) turn_scale = 1.0;
                    if (turn_scale < 0.90) turn_scale = 0.90;  // Minimum crawl to finish.
                    int pwm_range = max_pwm_ns - NEUTRAL_NS;
                    int turn_pwm_step = (int)(pwm_range * turn_scale);

                    // Direction: positive heading_err = CCW turn needed
                    //   (left motor reverse, right motor forward for CCW).
                    if (gyro_heading_err > 0) {
                        // CCW (left turn)
                        next_left_pwm  = NEUTRAL_NS - turn_pwm_step;
                        next_right_pwm = NEUTRAL_NS + turn_pwm_step;
                    } else {
                        // CW (right turn)
                        next_left_pwm  = NEUTRAL_NS + turn_pwm_step;
                        next_right_pwm = NEUTRAL_NS - turn_pwm_step;
                    }

                    // Mark encoders active so stall watchdog works.
                    pthread_mutex_lock(&motors[0].lock);
                    left_relative_counts = encoders[0].total_counts - encoders[0].move_start_counts;
                    left_target_counts   = encoders[0].target_counts;
                    left_active          = encoders[0].has_target;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    right_relative_counts = encoders[1].total_counts - encoders[1].move_start_counts;
                    right_target_counts   = encoders[1].target_counts;
                    right_active          = encoders[1].has_target;
                    pthread_mutex_unlock(&motors[1].lock);

                } else {
                    // --- NAV_DRIVING: encoder counts determine distance completion. ---

                    pthread_mutex_lock(&motors[0].lock);
                    if (encoders[0].has_target) {
                        int32_t current_relative = encoders[0].total_counts - encoders[0].move_start_counts;
                        left_relative_counts = current_relative;
                        left_target_counts = encoders[0].target_counts;
                        left_active = 1;
                        int32_t error = left_target_counts - current_relative;

                        if (abs(error) < STOP_THRESHOLD) {
                            next_left_pwm = NEUTRAL_NS;
                            encoders[0].has_target = 0;
                            encoders[0].ramp_start_time = 0.0;
                            left_done = 1;
                        } else {
                            double ramp_now = current_time;
                            if (encoders[0].ramp_start_time == 0.0) {
                                encoders[0].ramp_start_time = ramp_now;
                            }

                            double elapsed = ramp_now - encoders[0].ramp_start_time;
                            double accel_factor = elapsed / RAMP_UP_TIME;
                            if (accel_factor > 1.0) accel_factor = 1.0;

                            double decel_factor = 1.0;
                            int32_t abs_error = abs(error);
                            if (abs_error < DECEL_ZONE_COUNTS) {
                                decel_factor = (double)abs_error / DECEL_ZONE_COUNTS;
                                if (decel_factor < 0.2) decel_factor = 0.2;
                            }

                            double ramp_factor = (accel_factor < decel_factor) ? accel_factor : decel_factor;
                            if (error > 0) {
                                int pwm_range = max_pwm_ns - NEUTRAL_NS;
                                next_left_pwm = NEUTRAL_NS + (int)(pwm_range * ramp_factor);
                            } else {
                                int pwm_range = NEUTRAL_NS - min_pwm_ns;
                                next_left_pwm = NEUTRAL_NS - (int)(pwm_range * ramp_factor);
                            }
                        }
                    } else {
                        next_left_pwm = NEUTRAL_NS;
                        left_done = 1;
                    }
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    if (encoders[1].has_target) {
                        int32_t current_relative = encoders[1].total_counts - encoders[1].move_start_counts;
                        right_relative_counts = current_relative;
                        right_target_counts = encoders[1].target_counts;
                        right_active = 1;
                        int32_t error = right_target_counts - current_relative;

                        if (abs(error) < STOP_THRESHOLD) {
                            next_right_pwm = NEUTRAL_NS;
                            encoders[1].has_target = 0;
                            encoders[1].ramp_start_time = 0.0;
                            right_done = 1;
                        } else {
                            double ramp_now = current_time;
                            if (encoders[1].ramp_start_time == 0.0) {
                                encoders[1].ramp_start_time = ramp_now;
                            }

                            double elapsed = ramp_now - encoders[1].ramp_start_time;
                            double accel_factor = elapsed / RAMP_UP_TIME;
                            if (accel_factor > 1.0) accel_factor = 1.0;

                            double decel_factor = 1.0;
                            int32_t abs_error = abs(error);
                            if (abs_error < DECEL_ZONE_COUNTS) {
                                decel_factor = (double)abs_error / DECEL_ZONE_COUNTS;
                                if (decel_factor < 0.65) decel_factor = 0.65;
                            }

                            double ramp_factor = (accel_factor < decel_factor) ? accel_factor : decel_factor;
                            if (error > 0) {
                                int pwm_range = max_pwm_ns - NEUTRAL_NS;
                                next_right_pwm = NEUTRAL_NS + (int)(pwm_range * ramp_factor);
                            } else {
                                int pwm_range = NEUTRAL_NS - min_pwm_ns;
                                next_right_pwm = NEUTRAL_NS - (int)(pwm_range * ramp_factor);
                            }
                        }
                    } else {
                        next_right_pwm = NEUTRAL_NS;
                        right_done = 1;
                    }
                    pthread_mutex_unlock(&motors[1].lock);
                }  // end NAV_DRIVING encoder block

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
                        nav_ctrl.state = NAV_GOTO;
                        stall_last_progress_time = current_time;
                        stall_last_progress_sum = 0;

                        printf("STALL_RECOVERY\n");
                        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    }
                }

                // During straight driving, apply gyro heading correction only.
                // Encoder sync is intentionally not used — gyro is the sole
                // source of heading error for steering correction.
                if (nav_ctrl.state == NAV_DRIVING && left_active && right_active && !(left_done && right_done)) {
                    double drive_heading_err = normalize_angle_diff(
                        nav_ctrl.target_heading - odometry.heading);
                    if (fabs(drive_heading_err) > 1.5) {
                        // 1000 ns per degree of error, capped.
                        int gyro_correction = (int)(drive_heading_err * 1000.0);
                        gyro_correction = clamp_int(gyro_correction, -STRAIGHT_SYNC_MAX_NS, STRAIGHT_SYNC_MAX_NS);
                        int motion_sign = (left_target_counts < 0 && right_target_counts < 0) ? -1 : 1;
                        next_left_pwm  -= motion_sign * gyro_correction;
                        next_right_pwm += motion_sign * gyro_correction;
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
                    } else {
                        nav_ctrl.state = NAV_GOTO;
                    }

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
                encoders[0].target_counts = calculate_turn_counts(heading_diff);
                encoders[0].has_target = 1;
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                encoders[1].move_start_counts = encoders[1].total_counts;
                encoders[1].target_counts = -calculate_turn_counts(heading_diff);
                encoders[1].has_target = 1;
                pthread_mutex_unlock(&motors[1].lock);

                nav_ctrl.state = NAV_TURNING;
                nav_ctrl.target_heading = target_heading;
                stall_last_progress_time = current_time;
                stall_last_progress_sum = 0;

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
                    stall_last_progress_time = current_time;
                    stall_last_progress_sum = 0;

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

                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                }
                break;
            }

            case NAV_ML: {
                float inputs[5];   // target_l, actual_l, target_r, actual_r, gyro_z
                float outputs[2];  // normalized pwm_l, pwm_r

                pthread_mutex_lock(&motors[0].lock);
                int32_t target_l = encoders[0].target_counts;
                int32_t actual_l = encoders[0].total_counts - encoders[0].move_start_counts;
                pthread_mutex_unlock(&motors[0].lock);

                pthread_mutex_lock(&motors[1].lock);
                int32_t target_r = encoders[1].target_counts;
                int32_t actual_r = encoders[1].total_counts - encoders[1].move_start_counts;
                pthread_mutex_unlock(&motors[1].lock);

                pthread_mutex_lock(&imu_data_lock);
                float gyro = (float)current_gyro_rate;
                pthread_mutex_unlock(&imu_data_lock);

                float raw_inputs[5] = {
                    (float)target_l,
                    (float)actual_l,
                    (float)target_r,
                    (float)actual_r,
                    gyro
                };

                // Normalize using fitted scaler bounds exported from training.
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

                // Output layer is linear regression; clamp normalized predictions
                // before de-normalizing to PWM nanoseconds.
                for (int i = 0; i < 2; i++) {
                    if (!isfinite(outputs[i])) {
                        outputs[i] = 0.5f; // fall back to mid-range if inference is invalid
                    }
                    if (outputs[i] < 0.0f) outputs[i] = 0.0f;
                    if (outputs[i] > 1.0f) outputs[i] = 1.0f;
                }

                float pwm_l_f = TARGET_MIN[0] + outputs[0] * (TARGET_MAX[0] - TARGET_MIN[0]);
                float pwm_r_f = TARGET_MIN[1] + outputs[1] * (TARGET_MAX[1] - TARGET_MIN[1]);
                int pwm_l = (int)(pwm_l_f + 0.5f);
                int pwm_r = (int)(pwm_r_f + 0.5f);

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
