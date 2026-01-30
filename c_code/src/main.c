#include "../include/common.h"
#include "../include/motor.h"
#include "../include/i2c.h"
#include "../include/imu.h"
#include "../include/kalman.h"
#include "../include/sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

volatile int running = 1;

#define START_X 0.0
#define START_Y 15.0
#define START_HEADING 90.0

OdometryState odometry = {START_X, START_Y, START_HEADING, 0, 0}; // Start at (0, 15), Heading 90
NavigationController nav_ctrl = {NAV_IDLE, 0, 0, 0, 0, 0.3}; // Default 30% speed
KalmanFilter kf_heading;
double current_gyro_rate = 0.0;
double last_imu_time = 0.0;
pthread_mutex_t imu_data_lock = PTHREAD_MUTEX_INITIALIZER;

// Global PWM control parameters (can be adjusted via setpwm command)
int g_min_pwm = 45;  // Minimum PWM to overcome friction
int g_max_pwm = 80;  // Maximum PWM for control stability

void dump_log(); // Forward declare
int32_t calculate_turn_counts(double degrees);
void update_odometry(void);


void signal_handler(int sig) {
    (void)sig;
    running = 0;
    dump_log();
}

// --- Logging System ---
// --- Logging System ---
#define LOG_SIZE 1000000 // ~48MB RAM for logs, ~1.4 hrs at 200Hz. Reduced from 15M to prevent OOM.

// Control modes for logging
typedef enum {
    MODE_IDLE = 0,
    MODE_JOYSTICK = 1,    // Direct pulse commands
    MODE_VOICE_NAV = 2    // Autonomous goto commands
} ControlMode;

typedef struct {
    double time;
    int32_t target_l;
    int32_t actual_l;
    int pulse_l; 
    int raw_l;
    int32_t target_r;
    int32_t actual_r;
    int pulse_r; 
    int raw_r;
    char mode; // Control mode: 0=IDLE, 1=JOYSTICK, 2=VOICE_NAV
    
    // IMU data
    double gyro_z;        // Z-axis gyro rate (degrees/sec)
    
    // Odometry data
    double odom_x;        // X position (feet)
    double odom_y;        // Y position (feet)
    double odom_heading;  // Heading (degrees)
    
    // Navigation state
    char nav_state;       // 0=IDLE, 1=PLANNING, 2=TURNING, 3=DRIVING
} LogEntry;

LogEntry *log_buffer = NULL;
int log_index = 0;
ControlMode current_mode = MODE_IDLE;

void init_log_system() {
    log_buffer = (LogEntry*)malloc(sizeof(LogEntry) * LOG_SIZE);
    if (!log_buffer) {
        printf("ERROR: Failed to allocate 500MB log buffer\n");
    } else {
        printf("Allocated log buffer (%d entries)\n", LOG_SIZE);
    }
}

void log_data(double time) {
    if (!log_buffer || log_index >= LOG_SIZE) return;

    // Capture state safely
    LogEntry *entry = &log_buffer[log_index];
    entry->time = time;
    entry->mode = (char)current_mode;

    pthread_mutex_lock(&motors[0].lock);
    entry->target_l = encoders[0].target_counts;
    entry->actual_l = encoders[0].total_counts + (encoders[0].current_raw_angle - encoders[0].start_raw_angle);
    entry->pulse_l = motors[0].last_pulse_ns;
    entry->raw_l = encoders[0].current_raw_angle;
    pthread_mutex_unlock(&motors[0].lock);

    pthread_mutex_lock(&motors[1].lock);
    entry->target_r = encoders[1].target_counts;
    entry->actual_r = encoders[1].total_counts + (encoders[1].current_raw_angle - encoders[1].start_raw_angle);
    entry->pulse_r = motors[1].last_pulse_ns;
    entry->raw_r = encoders[1].current_raw_angle;
    pthread_mutex_unlock(&motors[1].lock);

    // Capture IMU data
    pthread_mutex_lock(&imu_data_lock);
    entry->gyro_z = current_gyro_rate;
    pthread_mutex_unlock(&imu_data_lock);

    // Capture odometry data (odometry is updated in coordinated_control_thread)
    entry->odom_x = odometry.x;
    entry->odom_y = odometry.y;
    entry->odom_heading = odometry.heading;

    // Capture navigation state
    entry->nav_state = (char)nav_ctrl.state;

    log_index++;
}

void dump_log() {
    if (!log_buffer) return;

    // Generate timestamp for unique filename
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char filename[512];
    char temp_filename[512];

    // Count entries by mode to determine primary mode
    int joystick_count = 0;
    int voice_count = 0;
    for (int i = 0; i < log_index; i++) {
        if (log_buffer[i].mode == MODE_JOYSTICK) joystick_count++;
        else if (log_buffer[i].mode == MODE_VOICE_NAV) voice_count++;
    }

    // Determine primary mode and create appropriate filename
    const char *mode_str = (joystick_count > voice_count) ? "joystick" : "voice";

    // Permanent log directory (relative to project root)
    // Check if file exists and auto-increment to prevent overwriting
    int file_counter = 0;
    while (1) {
        if (file_counter == 0) {
            snprintf(filename, sizeof(filename),
                     "../logs/motor_log_%s_%04d%02d%02d_%02d%02d%02d.csv",
                     mode_str, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                     t->tm_hour, t->tm_min, t->tm_sec);
        } else {
            snprintf(filename, sizeof(filename),
                     "../logs/motor_log_%s_%04d%02d%02d_%02d%02d%02d_%d.csv",
                     mode_str, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                     t->tm_hour, t->tm_min, t->tm_sec, file_counter);
        }

        // Check if file exists
        FILE *test = fopen(filename, "r");
        if (!test) {
            // File doesn't exist, we can use this filename
            break;
        }
        fclose(test);
        file_counter++;

        // Safety: don't loop forever
        if (file_counter > 1000) {
            fprintf(stderr, "ERROR: Too many log files with same timestamp\n");
            return;
        }
    }

    // Also save to RAM disk for quick access during session
    snprintf(temp_filename, sizeof(temp_filename),
             "/dev/shm/motor_log_%s_latest.csv", mode_str);

    FILE *f = fopen(filename, "w");
    if (!f) {
        fprintf(stderr, "ERROR: Could not open log file %s\n", filename);
        return;
    }

    // Header with all telemetry data
    fprintf(f, "time,mode,pwm_l,i2c_l,pwm_r,i2c_r,target_l,actual_l,target_r,actual_r,gyro_z,odom_x,odom_y,odom_heading,nav_state\n");

    // Write data with all telemetry fields
    const char *mode_names[] = {"IDLE", "JOYSTICK", "VOICE"};
    const char *nav_state_names[] = {"IDLE", "TURNING", "DRIVING", "GOTO"};
    for (int i = 0; i < log_index; i++) {
        fprintf(f, "%.4f,%s,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.2f,%s\n",
            log_buffer[i].time,
            mode_names[(int)log_buffer[i].mode],
            log_buffer[i].pulse_l, log_buffer[i].raw_l,
            log_buffer[i].pulse_r, log_buffer[i].raw_r,
            log_buffer[i].target_l, log_buffer[i].actual_l,
            log_buffer[i].target_r, log_buffer[i].actual_r,
            log_buffer[i].gyro_z,
            log_buffer[i].odom_x,
            log_buffer[i].odom_y,
            log_buffer[i].odom_heading,
            nav_state_names[(int)log_buffer[i].nav_state]);
    }
    fclose(f);
    printf("Saved %d log entries to %s\n", log_index, filename);
    printf("  Joystick entries: %d, Voice navigation entries: %d\n", joystick_count, voice_count);

    // Also save a copy to RAM disk for quick access
    FILE *f_temp = fopen(temp_filename, "w");
    if (f_temp) {
        fprintf(f_temp, "time,mode,pwm_l,i2c_l,pwm_r,i2c_r,target_l,actual_l,target_r,actual_r,gyro_z,odom_x,odom_y,odom_heading,nav_state\n");
        const char *mode_names[] = {"IDLE", "JOYSTICK", "VOICE"};
        const char *nav_state_names[] = {"IDLE", "TURNING", "DRIVING", "GOTO"};
        for (int i = 0; i < log_index; i++) {
            fprintf(f_temp, "%.4f,%s,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.2f,%s\n",
                log_buffer[i].time,
                mode_names[(int)log_buffer[i].mode],
                log_buffer[i].pulse_l, log_buffer[i].raw_l,
                log_buffer[i].pulse_r, log_buffer[i].raw_r,
                log_buffer[i].target_l, log_buffer[i].actual_l,
                log_buffer[i].target_r, log_buffer[i].actual_r,
                log_buffer[i].gyro_z,
                log_buffer[i].odom_x,
                log_buffer[i].odom_y,
                log_buffer[i].odom_heading,
                nav_state_names[(int)log_buffer[i].nav_state]);
        }
        fclose(f_temp);
        printf("  Quick access copy: %s\n", temp_filename);
    }

    // Free buffer after dumping to save memory if we were to continue (though we exit usually)
    free(log_buffer);
    log_buffer = NULL;
}

// --- Coordinated Control Thread ---
void* coordinated_control_thread(void* arg) {
    (void)arg;
    int sleep_us = 1000000 / 200; // 200Hz control loop
    
    printf("Control loop running at 200Hz\n");

    while (running) {
        double current_time = get_time_sec();


        switch (nav_ctrl.state) {
            case NAV_IDLE:
                // Do nothing
                break;

            case NAV_GOTO: {
                // Determine next step: Turn or Drive
                double dx = nav_ctrl.target_x - odometry.x;
                double dy = nav_ctrl.target_y - odometry.y;
                double target_heading = atan2(dy, dx) * 180.0 / M_PI;
                if (target_heading < 0) target_heading += 360.0;

                double heading_diff = target_heading - odometry.heading;
                while (heading_diff > 180) heading_diff -= 360;
                while (heading_diff < -180) heading_diff += 360;

                double distance = sqrt(dx*dx + dy*dy);

                if (distance < 1.0) { // Tolerance 1ft
                    printf("ARRIVED\n");
                    fflush(stdout);
                    nav_ctrl.state = NAV_IDLE;

                    // Send immediate STATUS update so Python knows we arrived
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                } else if (fabs(heading_diff) > 5.0) { // Turn required
                    nav_ctrl.state = NAV_TURNING;
                    nav_ctrl.target_heading = target_heading;

                    // Reset Encoders for local move
                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].start_raw_angle = encoders[0].current_raw_angle;
                    encoders[0].total_counts = 0;
                    encoders[0].target_counts = calculate_turn_counts(heading_diff);
                    encoders[0].has_target = 1;
                    encoders[0].stall_count = 0;
                    encoders[0].stall_check_time = current_time;
                    encoders[0].stall_last_position = 0;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].start_raw_angle = encoders[1].current_raw_angle;
                    encoders[1].total_counts = 0;
                    encoders[1].target_counts = -calculate_turn_counts(heading_diff); // Differential
                    encoders[1].has_target = 1;
                    encoders[1].stall_count = 0;
                    encoders[1].stall_check_time = current_time;
                    encoders[1].stall_last_position = 0;
                    pthread_mutex_unlock(&motors[1].lock);

                    // Send immediate STATUS to notify Python we started turning
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);

                } else { // Drive required
                    nav_ctrl.state = NAV_DRIVING;
                    nav_ctrl.target_distance = distance;

                    // Reset Encoders for local move
                    int32_t counts = (int32_t)(distance * COUNTS_PER_FOOT);
                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].start_raw_angle = encoders[0].current_raw_angle;
                    encoders[0].total_counts = 0;
                    encoders[0].target_counts = counts;
                    encoders[0].has_target = 1;
                    encoders[0].stall_count = 0;
                    encoders[0].stall_check_time = current_time;
                    encoders[0].stall_last_position = 0;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].start_raw_angle = encoders[1].current_raw_angle;
                    encoders[1].total_counts = 0;
                    encoders[1].target_counts = counts;
                    encoders[1].has_target = 1;
                    encoders[1].stall_count = 0;
                    encoders[1].stall_check_time = current_time;
                    encoders[1].stall_last_position = 0;
                    pthread_mutex_unlock(&motors[1].lock);

                    // Send immediate STATUS to notify Python we started driving
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                }
                break;
            }

            case NAV_TURNING:
            case NAV_DRIVING: {
                int left_done = 0, right_done = 0;

                // Simple on/off control - no proportional deceleration
                // Use global PWM limits (can be adjusted via setpwm command)
                // Apply speed multiplier from slider (0.0 - 1.0)
                int MAX_PWM = (int)(g_max_pwm * nav_ctrl.speed_multiplier);
                if (MAX_PWM < g_min_pwm) MAX_PWM = g_min_pwm; // Ensure we can move

                // Check Left
                 pthread_mutex_lock(&motors[0].lock);
                 if (encoders[0].has_target) {
                    int32_t current = encoders[0].total_counts + (encoders[0].current_raw_angle - encoders[0].start_raw_angle);
                    int32_t error = encoders[0].target_counts - current;

                    if (abs(error) < STOP_THRESHOLD) {
                        // Within stop threshold - we're done
                        set_motor_speed(0, 0, 1);
                        encoders[0].has_target = 0;
                        encoders[0].stall_count = 0;
                        left_done = 1;
                    } else if (abs(error) < DEADBAND_THRESHOLD && encoders[0].stall_count == 0) {
                        // Within deadband and not stalled - close enough, stop
                        set_motor_speed(0, 0, 1);
                        encoders[0].has_target = 0;
                        left_done = 1;
                    } else {
                        // Stall detection
                        if (current_time - encoders[0].stall_check_time > 0.5) {
                            int32_t position_change = abs(current - encoders[0].stall_last_position);
                            if (position_change < 20 && abs(error) > 100) {
                                encoders[0].stall_count++;
                                fprintf(stderr, "Left motor stalled (count: %d), error: %d\n", encoders[0].stall_count, error);
                            } else {
                                encoders[0].stall_count = 0;
                            }
                            encoders[0].stall_last_position = current;
                            encoders[0].stall_check_time = current_time;
                        }

                        // Proportional control with slowdown zone of 2000 counts
                        int pwm;
                        if (abs(error) < 2000) {
                            double scale = (double)abs(error) / 2000.0;
                            if (scale < 0.2) scale = 0.2; // Min 20% speed to keep moving
                            pwm = (int)(MAX_PWM * scale);
                            if (error < 0) pwm = -pwm;
                            
                            // Ensure we overcome friction (min absolute PWM)
                            if (abs(pwm) < g_min_pwm) pwm = (pwm > 0) ? g_min_pwm : -g_min_pwm;
                        } else {
                            pwm = (error > 0) ? MAX_PWM : -MAX_PWM;
                        }

                        // Stall compensation - boost power if stuck
                        int boost = encoders[0].stall_count * 10;
                        if (pwm > 0) {
                            pwm += boost;
                            if (pwm > 100) pwm = 100;
                        } else {
                            pwm -= boost;
                            if (pwm < -100) pwm = -100;
                        }

                        set_motor_speed(0, pwm, 1);
                    }
                 } else {
                     set_motor_speed(0, 0, 1);
                     left_done = 1;
                 }
                 pthread_mutex_unlock(&motors[0].lock);

                 // Check Right
                 pthread_mutex_lock(&motors[1].lock);
                 if (encoders[1].has_target) {
                    int32_t current = encoders[1].total_counts + (encoders[1].current_raw_angle - encoders[1].start_raw_angle);
                    int32_t error = encoders[1].target_counts - current;

                    if (abs(error) < STOP_THRESHOLD) {
                        // Within stop threshold - we're done
                        set_motor_speed(1, 0, 1);
                        encoders[1].has_target = 0;
                        encoders[1].stall_count = 0;
                        right_done = 1;
                    } else if (abs(error) < DEADBAND_THRESHOLD && encoders[1].stall_count == 0) {
                        // Within deadband and not stalled - close enough, stop
                        set_motor_speed(1, 0, 1);
                        encoders[1].has_target = 0;
                        right_done = 1;
                    } else {
                        // Stall detection
                        if (current_time - encoders[1].stall_check_time > 0.5) {
                            int32_t position_change = abs(current - encoders[1].stall_last_position);
                            if (position_change < 20 && abs(error) > 100) {
                                encoders[1].stall_count++;
                                fprintf(stderr, "Right motor stalled (count: %d), error: %d\n", encoders[1].stall_count, error);
                            } else {
                                encoders[1].stall_count = 0;
                            }
                            encoders[1].stall_last_position = current;
                            encoders[1].stall_check_time = current_time;
                        }

                        // Proportional control with slowdown zone of 2000 counts
                        int pwm;
                        if (abs(error) < 2000) {
                            double scale = (double)abs(error) / 2000.0;
                            if (scale < 0.2) scale = 0.2; // Min 20% speed to keep moving
                            pwm = (int)(MAX_PWM * scale);
                            if (error < 0) pwm = -pwm;
                            
                            // Ensure we overcome friction (min absolute PWM)
                            if (abs(pwm) < g_min_pwm) pwm = (pwm > 0) ? g_min_pwm : -g_min_pwm;
                        } else {
                            pwm = (error > 0) ? MAX_PWM : -MAX_PWM;
                        }

                        // Stall compensation - boost power if stuck
                        int boost = encoders[1].stall_count * 10;
                        if (pwm > 0) {
                            pwm += boost;
                            if (pwm > 100) pwm = 100;
                        } else {
                            pwm -= boost;
                            if (pwm < -100) pwm = -100;
                        }

                        set_motor_speed(1, pwm, 1);
                    }
                 } else {
                     set_motor_speed(1, 0, 1);
                     right_done = 1;
                 }
                 pthread_mutex_unlock(&motors[1].lock);

                 if (left_done && right_done) {
                     nav_ctrl.state = NAV_GOTO; // Re-evaluate

                     // Send immediate STATUS to notify Python of state change
                     printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                     fflush(stdout);
                 }
                 break;
            }
        }

        static int status_counter = 0;
        if (status_counter++ % 10 == 0) { // Approx 20Hz (200Hz loop) - Increased from 10Hz for faster response
             printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
             fflush(stdout);
        }

        // Log telemetry
        log_data(current_time);
        
        usleep(sleep_us);
    }
    return NULL;
}

// --- Encoder feedback thread ---
void update_encoder_tracker(EncoderState *enc, int16_t raw_angle) {
    if (enc->last_raw_angle >= 0) {
        if (enc->last_raw_angle > 3500 && raw_angle < 500) {
            enc->total_counts += COUNTS_PER_REV;
        } else if (enc->last_raw_angle < 500 && raw_angle > 3500) {
            enc->total_counts -= COUNTS_PER_REV;
        }
    }
    enc->last_raw_angle = raw_angle;
    enc->current_raw_angle = raw_angle;
}

void* encoder_feedback_thread(void* arg) {
    (void)arg;

    while (running) {
        // Read all sensors simultaneously (IMU on I2C3, encoders on I2C1)
        SensorData sensors = read_all_sensors();
        
        if (!sensors.valid) {
            // If read failed, skip this iteration
            continue;
        }
        
        int16_t left_angle = sensors.left_encoder;
        int16_t right_angle = sensors.right_encoder;
        
        // Update gyro data for odometry
        pthread_mutex_lock(&imu_data_lock);
        current_gyro_rate = sensors.gyro_z;
        pthread_mutex_unlock(&imu_data_lock);

        // Process left motor encoder
        if (left_angle >= 0) {
            pthread_mutex_lock(&motors[0].lock);
            
            // Safety: Initialize last_raw_angle on first valid read
            if (encoders[0].last_raw_angle < 0) {
                encoders[0].last_raw_angle = left_angle;
                encoders[0].current_raw_angle = left_angle;
            }
            
            update_encoder_tracker(&encoders[0], left_angle);
            pthread_mutex_unlock(&motors[0].lock);
        }

        // Process right motor encoder
        if (right_angle >= 0) {
            pthread_mutex_lock(&motors[1].lock);

            // Safety: Initialize last_raw_angle on first valid read
            if (encoders[1].last_raw_angle < 0) {
                encoders[1].last_raw_angle = right_angle;
                encoders[1].current_raw_angle = right_angle;
            }

            update_encoder_tracker(&encoders[1], right_angle);
            pthread_mutex_unlock(&motors[1].lock);
        }

        update_odometry();

    }
    return NULL;
}

int32_t calculate_turn_counts(double degrees) {
    double arc_length = (fabs(degrees) / 360.0) * M_PI * WHEELBASE_INCHES;
    return (int32_t)(arc_length * COUNTS_PER_INCH);
}

// --- Fusion Odometry ---
void update_odometry(void) {
    double current_time = get_time_sec();
    double dt = current_time - last_imu_time;
    last_imu_time = current_time;

    // 1. Get Encoder Data (Distance Change)
    // Delta counts since last check (Note: assumes we are called frequently enough that we don't wrap int32)
    int32_t d_left = encoders[0].total_counts - odometry.last_left_total;
    int32_t d_right = encoders[1].total_counts - odometry.last_right_total;
    
    odometry.last_left_total = encoders[0].total_counts;
    odometry.last_right_total = encoders[1].total_counts;

    double dist_left = d_left / COUNTS_PER_FOOT;
    double dist_right = d_right / COUNTS_PER_FOOT;

    double center_dist = (dist_left + dist_right) / 2.0;


    


    // 3. Get Gyro Rate (Process)
    pthread_mutex_lock(&imu_data_lock);
    double gyro_rate = current_gyro_rate;
    pthread_mutex_unlock(&imu_data_lock);

    // 4. IMU Integration (No Sensor Fusion)
    // Simply integrate gyro rate to get new heading
    double dt_seconds = dt;
    double delta_heading = gyro_rate * dt_seconds;
    
    // Update heading
    double new_heading = odometry.heading + delta_heading;

    // 5. Update Odometry State
    // Use the average heading during the interval for position update
    double avg_heading_rad = (odometry.heading + new_heading) / 2.0 * (M_PI/180.0);
    
    odometry.x += center_dist * cos(avg_heading_rad);
    odometry.y += center_dist * sin(avg_heading_rad);
    
    odometry.heading = new_heading;
    
    // Normalize heading 0-360
    while(odometry.heading >= 360.0) odometry.heading -= 360.0;
    while(odometry.heading < 0.0) odometry.heading += 360.0;
    
    // Update Kalman state just to keep it in sync if we switch back later, 
    // though it's not used for the result anymore
    kf_heading.angle = odometry.heading;
}

// --- Command processing ---
void process_command(char* cmd) {
    cmd[strcspn(cmd, "\n")] = 0;

    // Debug logging to trace command reception
    fprintf(stderr, "DEBUG: Received command: '%s'\n", cmd);
    fflush(stderr);

    if (strncasecmp(cmd, "goto", 4) == 0) {
        double x, y;
        if (sscanf(cmd + 4, "%lf %lf", &x, &y) == 2) {
            current_mode = MODE_VOICE_NAV; // Voice control mode
            nav_ctrl.target_x = x;
            nav_ctrl.target_y = y;
            nav_ctrl.state = NAV_GOTO;
            printf("OK goto %.2f %.2f\n", x, y);
            fflush(stdout);

            // Send immediate STATUS update so Python knows state changed
            printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
            fflush(stdout);
        }
    }
    else if (strncasecmp(cmd, "speed", 5) == 0) {
        double s;
        if (sscanf(cmd + 5, "%lf", &s) == 1) {
            if (s < 0.0) s = 0.0;
            if (s > 1.0) s = 1.0;
            nav_ctrl.speed_multiplier = s;
            printf("OK speed %.2f\n", s);
            fflush(stdout);
        }
    }
    else if (strncasecmp(cmd, "setpwm", 6) == 0) {
        int min_pwm, max_pwm;
        if (sscanf(cmd + 6, "%d %d", &min_pwm, &max_pwm) == 2) {
            // Validate ranges
            if (min_pwm < 20) min_pwm = 20;
            if (min_pwm > 100) min_pwm = 100;
            if (max_pwm < 20) max_pwm = 20;
            if (max_pwm > 100) max_pwm = 100;
            if (min_pwm > max_pwm) {
                int temp = min_pwm;
                min_pwm = max_pwm;
                max_pwm = temp;
            }

            g_min_pwm = min_pwm;
            g_max_pwm = max_pwm;
            printf("OK setpwm %d %d\n", min_pwm, max_pwm);
            fflush(stdout);
        }
    }
    else if (strncasecmp(cmd, "setpos", 6) == 0) {
        double x, y, h;
        if (sscanf(cmd + 6, "%lf %lf %lf", &x, &y, &h) == 3) {
            odometry.x = x;
            odometry.y = y;
            odometry.heading = h;
            // Also reset accumulation to avoid jumps
            odometry.last_left_total = encoders[0].total_counts;
            odometry.last_right_total = encoders[1].total_counts;
            printf("OK setpos %.2f %.2f %.2f\n", x, y, h);
            fflush(stdout);

            // Send immediate STATUS update
            printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
            fflush(stdout);
        }
    }
    else if (strncasecmp(cmd, "stop", 4) == 0) {
        current_mode = MODE_IDLE; // Stopped/idle mode
        nav_ctrl.state = NAV_IDLE;
        for (int i = 0; i < 2; i++) {
            pthread_mutex_lock(&motors[i].lock);
            encoders[i].has_target = 0;
            set_motor_speed(i, 0, 1); // IMMEDIATE STOP
            pthread_mutex_unlock(&motors[i].lock);
        }
        // Force log dump
        dump_log();
        log_index = 0; // Reset log index
        printf("OK stopall (log dumped)\n");
        fflush(stdout);
    }
    else if (strcasecmp(cmd, "q") == 0) {
        running = 0;
        printf("OK quit\n");
        fflush(stdout);
    }
    // Raw pulse width control: pulse <left_ns> <right_ns>
    else if (strncasecmp(cmd, "pulse", 5) == 0) {
        int left_ns, right_ns;
        if (sscanf(cmd + 5, "%d %d", &left_ns, &right_ns) == 2) {
            current_mode = MODE_JOYSTICK; // Joystick/manual control mode

            // Disable navigation
            nav_ctrl.state = NAV_IDLE;

            // Disable PID targets
            pthread_mutex_lock(&motors[0].lock);
            encoders[0].has_target = 0;
            pthread_mutex_unlock(&motors[0].lock);

            pthread_mutex_lock(&motors[1].lock);
            encoders[1].has_target = 0;
            pthread_mutex_unlock(&motors[1].lock);

            // Clamp pulse widths to valid range
            if (left_ns < REVERSE_MAX_NS) left_ns = REVERSE_MAX_NS;
            if (left_ns > FORWARD_MAX_NS) left_ns = FORWARD_MAX_NS;
            if (right_ns < REVERSE_MAX_NS) right_ns = REVERSE_MAX_NS;
            if (right_ns > FORWARD_MAX_NS) right_ns = FORWARD_MAX_NS;



            // Write pulse widths directly (Protected by locks)
            pthread_mutex_lock(&motors[0].lock);
            lseek(motors[0].pwm_duty_fd, 0, SEEK_SET);
            dprintf(motors[0].pwm_duty_fd, "%d", left_ns);
            motors[0].last_pulse_ns = left_ns;
            pthread_mutex_unlock(&motors[0].lock);

            pthread_mutex_lock(&motors[1].lock);
            lseek(motors[1].pwm_duty_fd, 0, SEEK_SET);
            dprintf(motors[1].pwm_duty_fd, "%d", right_ns);
            motors[1].last_pulse_ns = right_ns;
            pthread_mutex_unlock(&motors[1].lock);



            printf("OK pulse L:%d R:%d\n", left_ns, right_ns);
            fflush(stdout);
        }
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

    init_log_system();

    // Initialize IMU
    if (imu_init() < 0) {
        fprintf(stderr, "WARNING: IMU init failed (check wiring to I2C3). Continuing without IMU.\n");
    } else {
        imu_calibrate(200); // 1 second calibration
    }

    // Initialize Kalman Filter
    kalman_init(&kf_heading);
    kf_heading.angle = 90.0; // Initialize with start heading
    last_imu_time = get_time_sec();
    
    // Initialize Encoders
    for(int i=0; i<2; i++) {
        encoders[i].total_counts = 0;
        encoders[i].current_raw_angle = 0;
        encoders[i].last_raw_angle = -1; // Flag as invalid
        encoders[i].start_raw_angle = 0;
        encoders[i].target_counts = 0;
        encoders[i].has_target = 0;
        encoders[i].stall_last_position = 0;
        encoders[i].stall_check_time = 0;
        encoders[i].stall_count = 0;
    }

    pthread_mutex_init(&motors[0].lock, NULL);
    pthread_mutex_init(&motors[1].lock, NULL);
    // pthread_mutex_init(&coord_move.lock, NULL); // Removed legacy lock


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
