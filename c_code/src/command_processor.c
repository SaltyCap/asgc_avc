#include "../include/command_processor.h"
#include "../include/imu.h"
#include "../include/logging.h"
#include "../include/motor.h"
#include "../include/runtime_state.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>

static void disable_ml_workflow_locked(void) {
    nav_ctrl.ml_test_enabled = 0;
    nav_ctrl.ml_stage = ML_STAGE_INACTIVE;
    nav_ctrl.ml_ball_acquired = 0;
    nav_ctrl.ml_ball_color = BALL_COLOR_NONE;
    nav_ctrl.ml_sweep_waypoint_index = 0;
}

static void stop_motion_targets(void) {
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

static const char* ball_color_name(BallColor color) {
    switch (color) {
        case BALL_COLOR_RED:
            return "red";
        case BALL_COLOR_YELLOW:
            return "yellow";
        case BALL_COLOR_BLUE:
            return "blue";
        case BALL_COLOR_GREEN:
            return "green";
        case BALL_COLOR_NONE:
        default:
            return "none";
    }
}

static int ball_color_from_token(const char* token, BallColor* out_color, double* out_x, double* out_y) {
    if (!token || !out_color || !out_x || !out_y) {
        return 0;
    }

    if (strcasecmp(token, "red") == 0) {
        *out_color = BALL_COLOR_RED;
        *out_x = BUCKET_RED_X;
        *out_y = BUCKET_RED_Y;
        return 1;
    }
    if (strcasecmp(token, "yellow") == 0) {
        *out_color = BALL_COLOR_YELLOW;
        *out_x = BUCKET_YELLOW_X;
        *out_y = BUCKET_YELLOW_Y;
        return 1;
    }
    if (strcasecmp(token, "blue") == 0) {
        *out_color = BALL_COLOR_BLUE;
        *out_x = BUCKET_BLUE_X;
        *out_y = BUCKET_BLUE_Y;
        return 1;
    }
    if (strcasecmp(token, "green") == 0) {
        *out_color = BALL_COLOR_GREEN;
        *out_x = BUCKET_GREEN_X;
        *out_y = BUCKET_GREEN_Y;
        return 1;
    }
    if (strcasecmp(token, "none") == 0 || strcasecmp(token, "clear") == 0) {
        *out_color = BALL_COLOR_NONE;
        *out_x = 0.0;
        *out_y = 0.0;
        return 1;
    }

    return 0;
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

    switch (nav_ctrl.ml_ball_color) {
        case BALL_COLOR_RED:
            bucket_x = BUCKET_RED_X;
            bucket_y = BUCKET_RED_Y;
            break;
        case BALL_COLOR_YELLOW:
            bucket_x = BUCKET_YELLOW_X;
            bucket_y = BUCKET_YELLOW_Y;
            break;
        case BALL_COLOR_BLUE:
            bucket_x = BUCKET_BLUE_X;
            bucket_y = BUCKET_BLUE_Y;
            break;
        case BALL_COLOR_GREEN:
            bucket_x = BUCKET_GREEN_X;
            bucket_y = BUCKET_GREEN_Y;
            break;
        case BALL_COLOR_NONE:
        default:
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

void process_command(char* cmd) {
    cmd[strcspn(cmd, "\n")] = 0;

    fprintf(stderr, "DEBUG: Received command: '%s'\n", cmd);
    fflush(stderr);

    if (strncasecmp(cmd, "goto", 4) == 0) {
        double x, y;
        int is_bucket = 0;
        int parsed = sscanf(cmd + 4, "%lf %lf %d", &x, &y, &is_bucket);

        if (parsed >= 2) {
            set_control_mode(MODE_VOICE_NAV);
            double status_x, status_y, status_h;
            int status_state;
            pthread_mutex_lock(&state_lock);
            disable_ml_workflow_locked();
            nav_ctrl.target_x = x;
            nav_ctrl.target_y = y;
            nav_ctrl.is_bucket_target = (is_bucket != 0);

            if (nav_ctrl.is_bucket_target) {
                nav_ctrl.bucket_x = x;
                nav_ctrl.bucket_y = y;
            } else {
                nav_ctrl.bucket_x = 0.0;
                nav_ctrl.bucket_y = 0.0;
            }

            nav_ctrl.state = NAV_GOTO;
            status_x = odometry.x;
            status_y = odometry.y;
            status_h = odometry.heading;
            status_state = nav_ctrl.state;
            pthread_mutex_unlock(&state_lock);
            printf("OK goto %.2f %.2f (bucket=%d)\n", x, y, is_bucket);
            fflush(stdout);

            printf("STATUS %.2f %.2f %.2f %d\n", status_x, status_y, status_h, status_state);
            fflush(stdout);
        }
    } else if (strncasecmp(cmd, "setpwm", 6) == 0) {
        int min_pwm_percent, max_pwm_percent;
        if (sscanf(cmd + 6, "%d %d", &min_pwm_percent, &max_pwm_percent) == 2) {
            if (min_pwm_percent < 0) min_pwm_percent = 0;
            if (min_pwm_percent > 100) min_pwm_percent = 100;
            if (max_pwm_percent < 0) max_pwm_percent = 0;
            if (max_pwm_percent > 100) max_pwm_percent = 100;
            if (min_pwm_percent > max_pwm_percent) {
                int temp = min_pwm_percent;
                min_pwm_percent = max_pwm_percent;
                max_pwm_percent = temp;
            }

            pthread_mutex_lock(&state_lock);
            g_min_pwm_ns = NEUTRAL_NS - ((NEUTRAL_NS - REVERSE_MAX_NS) * min_pwm_percent / 100);
            g_max_pwm_ns = NEUTRAL_NS + ((FORWARD_MAX_NS - NEUTRAL_NS) * max_pwm_percent / 100);
            int min_ns = g_min_pwm_ns;
            int max_ns = g_max_pwm_ns;
            pthread_mutex_unlock(&state_lock);

            printf("OK setpwm %d %d (min=%dns, max=%dns)\n", min_pwm_percent, max_pwm_percent, min_ns, max_ns);
            fflush(stdout);
        }
    } else if (strncasecmp(cmd, "speed", 5) == 0) {
        double speed_scale;
        if (sscanf(cmd + 5, "%lf", &speed_scale) == 1) {
            if (speed_scale < 0.0) speed_scale = 0.0;
            if (speed_scale > 1.0) speed_scale = 1.0;
            int requested_percent = (int)(speed_scale * 100.0 + 0.5);
            if (requested_percent <= 0) requested_percent = 1;
            pthread_mutex_lock(&state_lock);
            g_speed_percent = requested_percent;
            int applied_percent = g_speed_percent;
            pthread_mutex_unlock(&state_lock);
            printf("OK speed %.2f (%d%%)\n", speed_scale, applied_percent);
            fflush(stdout);
        }
    } else if (strncasecmp(cmd, "setpos", 6) == 0) {
        double x, y, h;
        if (sscanf(cmd + 6, "%lf %lf %lf", &x, &y, &h) == 3) {
            pthread_mutex_lock(&state_lock);
            odometry.x = x;
            odometry.y = y;
            odometry.heading = h;
            odometry.last_left_total = encoders[0].total_counts;
            odometry.last_right_total = encoders[1].total_counts;
            int status_state = nav_ctrl.state;
            pthread_mutex_unlock(&state_lock);
            printf("OK setpos %.2f %.2f %.2f\n", x, y, h);
            fflush(stdout);

            printf("STATUS %.2f %.2f %.2f %d\n", x, y, h, status_state);
            fflush(stdout);
        }
    } else if (strncasecmp(cmd, "calibrate", 9) == 0) {
        imu_calibrate(500);

        pthread_mutex_lock(&state_lock);
        odometry.x = 0.0;
        odometry.y = 15.0;
        odometry.heading = 0.0;
        odometry.last_left_total = encoders[0].total_counts;
        odometry.last_right_total = encoders[1].total_counts;
        int status_state = nav_ctrl.state;
        pthread_mutex_unlock(&state_lock);

        printf("OK calibrate\n");
        fflush(stdout);
        printf("STATUS %.2f %.2f %.2f %d\n", 0.0, 15.0, 0.0, status_state);
        fflush(stdout);
    } else if (strncasecmp(cmd, "regular_mode", 12) == 0) {
        set_control_mode(MODE_IDLE);

        pthread_mutex_lock(&state_lock);
        disable_ml_workflow_locked();
        nav_ctrl.state = NAV_IDLE;
        nav_ctrl.is_bucket_target = 0;
        nav_ctrl.bucket_x = 0.0;
        nav_ctrl.bucket_y = 0.0;
        double status_x = odometry.x;
        double status_y = odometry.y;
        double status_h = odometry.heading;
        int status_state = nav_ctrl.state;
        pthread_mutex_unlock(&state_lock);

        for (int i = 0; i < 2; i++) {
            pthread_mutex_lock(&motors[i].lock);
            encoders[i].has_target = 0;
            set_motor_pwm(i, NEUTRAL_NS);
            pthread_mutex_unlock(&motors[i].lock);
        }

        printf("OK regular_mode\n");
        fflush(stdout);
        printf("STATUS %.2f %.2f %.2f %d\n", status_x, status_y, status_h, status_state);
        fflush(stdout);
    } else if (strncasecmp(cmd, "stop", 4) == 0 || strncasecmp(cmd, "stopall", 7) == 0) {
        set_control_mode(MODE_IDLE);
        pthread_mutex_lock(&state_lock);
        disable_ml_workflow_locked();
        nav_ctrl.state = NAV_IDLE;
        nav_ctrl.is_bucket_target = 0;
        nav_ctrl.bucket_x = 0.0;
        nav_ctrl.bucket_y = 0.0;
        pthread_mutex_unlock(&state_lock);
        for (int i = 0; i < 2; i++) {
            pthread_mutex_lock(&motors[i].lock);
            encoders[i].has_target = 0;
            set_motor_pwm(i, NEUTRAL_NS);
            pthread_mutex_unlock(&motors[i].lock);
        }
        dump_log();
        init_log_system();
        printf("OK stopall (log dumped)\n");
        fflush(stdout);
    } else if (strcasecmp(cmd, "q") == 0) {
        running = 0;
        printf("OK quit\n");
        fflush(stdout);
    } else if (strncasecmp(cmd, "pulse", 5) == 0) {
        int left_ns, right_ns;
        if (sscanf(cmd + 5, "%d %d", &left_ns, &right_ns) == 2) {
            set_control_mode(MODE_JOYSTICK);

            pthread_mutex_lock(&state_lock);
            disable_ml_workflow_locked();
            nav_ctrl.state = NAV_IDLE;
            nav_ctrl.is_bucket_target = 0;
            nav_ctrl.bucket_x = 0.0;
            nav_ctrl.bucket_y = 0.0;
            pthread_mutex_unlock(&state_lock);

            pthread_mutex_lock(&motors[0].lock);
            encoders[0].has_target = 0;
            pthread_mutex_unlock(&motors[0].lock);

            pthread_mutex_lock(&motors[1].lock);
            encoders[1].has_target = 0;
            pthread_mutex_unlock(&motors[1].lock);

            if (left_ns < REVERSE_MAX_NS) left_ns = REVERSE_MAX_NS;
            if (left_ns > FORWARD_MAX_NS) left_ns = FORWARD_MAX_NS;
            if (right_ns < REVERSE_MAX_NS) right_ns = REVERSE_MAX_NS;
            if (right_ns > FORWARD_MAX_NS) right_ns = FORWARD_MAX_NS;

            pthread_mutex_lock(&motors[0].lock);
            set_motor_pwm(0, left_ns);
            pthread_mutex_unlock(&motors[0].lock);

            pthread_mutex_lock(&motors[1].lock);
            set_motor_pwm(1, right_ns);
            pthread_mutex_unlock(&motors[1].lock);

        printf("OK pulse L:%d R:%d\n", left_ns, right_ns);
        fflush(stdout);
        }
    } else if (strncasecmp(cmd, "ml_mode", 7) == 0) {
        set_control_mode(MODE_ML);

        // Clear any active motion command and start ML workflow from center.
        stop_motion_targets();

        pthread_mutex_lock(&state_lock);
        nav_ctrl.state = NAV_GOTO;
        nav_ctrl.target_x = COURSE_CENTER_X;
        nav_ctrl.target_y = COURSE_CENTER_Y;
        nav_ctrl.target_heading = 0.0;
        nav_ctrl.target_distance = 0.0;
        nav_ctrl.is_bucket_target = 0;
        nav_ctrl.bucket_x = 0.0;
        nav_ctrl.bucket_y = 0.0;
        nav_ctrl.ml_test_enabled = 1;
        nav_ctrl.ml_stage = ML_STAGE_TO_CENTER;
        nav_ctrl.ml_ball_acquired = 0;
        nav_ctrl.ml_ball_color = BALL_COLOR_NONE;
        nav_ctrl.ml_sweep_waypoint_index = 0;
        double status_x = odometry.x;
        double status_y = odometry.y;
        double status_h = odometry.heading;
        int status_state = nav_ctrl.state;
        pthread_mutex_unlock(&state_lock);

        printf("OK ml_mode\n");
        fflush(stdout);
        printf("ML_STAGE to_center\n");
        fflush(stdout);
        printf("STATUS %.2f %.2f %.2f %d\n", status_x, status_y, status_h, status_state);
        fflush(stdout);
    } else if (strncasecmp(cmd, "ml_ball_acquired", 16) == 0) {
        char token[16];
        if (sscanf(cmd + 16, "%15s", token) != 1) {
            printf("ERR ml_ball_acquired expects 0/1\n");
            fflush(stdout);
            return;
        }

        int acquired = 0;
        if (strcasecmp(token, "1") == 0 || strcasecmp(token, "true") == 0 ||
            strcasecmp(token, "yes") == 0 || strcasecmp(token, "on") == 0) {
            acquired = 1;
        } else if (strcasecmp(token, "0") == 0 || strcasecmp(token, "false") == 0 ||
                   strcasecmp(token, "no") == 0 || strcasecmp(token, "off") == 0) {
            acquired = 0;
        } else {
            printf("ERR ml_ball_acquired expects 0/1\n");
            fflush(stdout);
            return;
        }

        pthread_mutex_lock(&state_lock);
        nav_ctrl.ml_ball_acquired = acquired;
        int dispatched = ml_try_dispatch_bucket_locked();
        BallColor assigned_color = nav_ctrl.ml_ball_color;
        double status_x = odometry.x;
        double status_y = odometry.y;
        double status_h = odometry.heading;
        int status_state = nav_ctrl.state;
        pthread_mutex_unlock(&state_lock);

        printf("OK ml_ball_acquired %d\n", acquired);
        fflush(stdout);
        if (dispatched) {
            printf("ML_TARGET_BUCKET %s\n", ball_color_name(assigned_color));
            fflush(stdout);
        }
        printf("STATUS %.2f %.2f %.2f %d\n", status_x, status_y, status_h, status_state);
        fflush(stdout);
    } else if (strncasecmp(cmd, "ml_ball_color", 13) == 0) {
        char color_token[16];
        if (sscanf(cmd + 13, "%15s", color_token) != 1) {
            printf("ERR ml_ball_color expects red|yellow|blue|green|none\n");
            fflush(stdout);
            return;
        }

        BallColor color = BALL_COLOR_NONE;
        double bucket_x = 0.0;
        double bucket_y = 0.0;
        if (!ball_color_from_token(color_token, &color, &bucket_x, &bucket_y)) {
            printf("ERR ml_ball_color expects red|yellow|blue|green|none\n");
            fflush(stdout);
            return;
        }

        pthread_mutex_lock(&state_lock);
        nav_ctrl.ml_ball_color = color;
        int dispatched = ml_try_dispatch_bucket_locked();
        double status_x = odometry.x;
        double status_y = odometry.y;
        double status_h = odometry.heading;
        int status_state = nav_ctrl.state;
        pthread_mutex_unlock(&state_lock);

        printf("OK ml_ball_color %s\n", ball_color_name(color));
        fflush(stdout);
        if (dispatched) {
            printf("ML_TARGET_BUCKET %s\n", ball_color_name(color));
            fflush(stdout);
        }
        printf("STATUS %.2f %.2f %.2f %d\n", status_x, status_y, status_h, status_state);
        fflush(stdout);
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
