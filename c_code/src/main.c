#include "../include/common.h"
#include "../include/motor.h"
#include "../include/i2c.h"
#include "../include/imu.h"
#include "../include/kalman.h"
#include "../include/sensors.h"
#include "../include/logging.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

// Global state
volatile int running = 1;

// Odometry and navigation state
OdometryState odometry = {START_X, START_Y, START_HEADING, 0, 0};
NavigationController nav_ctrl = {NAV_IDLE, 0, 0, 0, 0, 0.15, 0, 0, 0};

// IMU and Kalman filter state
KalmanFilter kf_heading;
double current_gyro_rate = 0.0;
double last_imu_time = 0.0;
pthread_mutex_t imu_data_lock = PTHREAD_MUTEX_INITIALIZER;

// Global PWM limits from slider (nanoseconds)
// These values are set by the web interface and control max motor speed
int g_min_pwm_ns = 1400000;  // Minimum PWM pulse width (1400µs)
int g_max_pwm_ns = 1600000;  // Maximum PWM pulse width (1600µs)

// Motor ramping time constants
// Using smooth ramps prevents mechanical stress and improves control stability
#define RAMP_UP_TIME 3.0    // Seconds to ramp from neutral to max speed
#define RAMP_DOWN_TIME 3.0  // Seconds to ramp from max to neutral
#define DECEL_ZONE_FEET 3.0 // Start decelerating when within 3 feet of target
#define DECEL_ZONE_COUNTS ((int32_t)(DECEL_ZONE_FEET * COUNTS_PER_FOOT))

// Forward declarations
int32_t calculate_turn_counts(double degrees);
void update_odometry(void);


/**
 * Signal handler for graceful shutdown
 * 
 * Handles SIGINT (Ctrl+C) and SIGTERM by stopping the control loop
 * and dumping telemetry logs before exit.
 */
void signal_handler(int sig) {
    (void)sig;
    running = 0;
    dump_log();
}

/**
 * Coordinated control thread
 * 
 * Main control loop running at 500Hz. Handles navigation state machine,
 * motor control with smooth ramping, and telemetry logging.
 * 
 * Navigation states:
 * - NAV_IDLE: No active navigation
 * - NAV_GOTO: Planning phase, determines if turn or drive is needed
 * - NAV_TURNING: Executing differential turn to target heading
 * - NAV_DRIVING: Executing straight-line drive to target position
 * - NAV_BUCKET_ROTATE: Special bucket approach - rotate 180 degrees
 * - NAV_BUCKET_BACKUP: Special bucket approach - back up to 0.25ft
 */
void* coordinated_control_thread(void* arg) {
    (void)arg;
    const int sleep_us = 1000000 / 500; // 500Hz control loop
    
    printf("Control loop running at 500Hz\n");

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

                // Use different tolerance based on whether this is a bucket target
                double arrival_tolerance = nav_ctrl.is_bucket_target ? 1.5 : 1.0;

                if (distance < arrival_tolerance) {
                    if (nav_ctrl.is_bucket_target) {
                        // Bucket approach: transition to rotation phase
                        printf("BUCKET_ZONE\\n");
                        fflush(stdout);
                        nav_ctrl.state = NAV_BUCKET_ROTATE;
                        
                        // Send immediate STATUS update
                        printf("STATUS %.2f %.2f %.2f %d\\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    } else {
                        // Normal arrival (center or other targets)
                        printf("ARRIVED\\n");
                        fflush(stdout);
                        nav_ctrl.state = NAV_IDLE;

                        // Send immediate STATUS update so Python knows we arrived
                        printf("STATUS %.2f %.2f %.2f %d\\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                        fflush(stdout);
                    }
                } else if (fabs(heading_diff) > 5.0) { // Turn required
                    nav_ctrl.state = NAV_TURNING;
                    nav_ctrl.target_heading = target_heading;

                    // Reset Encoders for local move
                    pthread_mutex_lock(&motors[0].lock);
                    encoders[0].move_start_counts = encoders[0].total_counts; // Capture start position
                    encoders[0].target_counts = calculate_turn_counts(heading_diff);
                    encoders[0].has_target = 1;
                    pthread_mutex_unlock(&motors[0].lock);

                    pthread_mutex_lock(&motors[1].lock);
                    encoders[1].move_start_counts = encoders[1].total_counts; // Capture start position
                    encoders[1].target_counts = -calculate_turn_counts(heading_diff); // Differential
                    encoders[1].has_target = 1;
                    pthread_mutex_unlock(&motors[1].lock);

                    // Send immediate STATUS to notify Python we started turning
                    printf("STATUS %.2f %.2f %.2f %d\\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);

                } else { // Drive required
                    nav_ctrl.state = NAV_DRIVING;
                    nav_ctrl.target_distance = distance;

                    // Reset Encoders for local move
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

                    // Send immediate STATUS to notify Python we started driving
                    printf("STATUS %.2f %.2f %.2f %d\\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                }
                break;
            }

            case NAV_TURNING:
            case NAV_DRIVING: {
                int left_done = 0, right_done = 0;

                // PWM limits are already in nanoseconds from slider
                // Apply speed multiplier by scaling the range around neutral
                int range_ns = g_max_pwm_ns - NEUTRAL_NS;
                int MAX_PWM_NS = NEUTRAL_NS + (int)(range_ns * nav_ctrl.speed_percent);
                
                range_ns = NEUTRAL_NS - g_min_pwm_ns;
                int MIN_PWM_NS = NEUTRAL_NS - (int)(range_ns * nav_ctrl.speed_percent);
                
                // Ensure we have valid values
                if (MAX_PWM_NS < NEUTRAL_NS + 1000) MAX_PWM_NS = NEUTRAL_NS + 1000;
                if (MIN_PWM_NS > NEUTRAL_NS - 1000) MIN_PWM_NS = NEUTRAL_NS - 1000;

                // Check Left
                 pthread_mutex_lock(&motors[0].lock);
                 if (encoders[0].has_target) {
                     // Calculate relative position and error
                    int32_t current_relative = (encoders[0].total_counts + (encoders[0].current_raw_angle - encoders[0].start_raw_angle)) - encoders[0].move_start_counts;
                    int32_t error = encoders[0].target_counts - current_relative;

                    if (abs(error) < STOP_THRESHOLD) {
                        // Within stop threshold - we're done
                        set_motor_speed(0, 0, 1);
                        encoders[0].has_target = 0;
                        encoders[0].ramp_start_time = 0.0; // Reset ramp
                        left_done = 1;
                    } else {
                        double current_time = get_time_sec();
                        
                        // Smooth ramping control with acceleration and deceleration
                        // Initialize ramp start time if not set
                        if (encoders[0].ramp_start_time == 0.0) {
                            encoders[0].ramp_start_time = current_time;
                        }
                        
                        // Acceleration phase: ramp up from 0 to full speed
                        double elapsed = current_time - encoders[0].ramp_start_time;
                        double accel_factor = elapsed / RAMP_UP_TIME;
                        if (accel_factor > 1.0) accel_factor = 1.0; // Cap at 100%
                        
                        // Deceleration phase: slow down when approaching target
                        double decel_factor = 1.0;
                        int32_t abs_error = abs(error);
                        if (abs_error < DECEL_ZONE_COUNTS) {
                            // Within deceleration zone: reduce speed proportionally
                            decel_factor = (double)abs_error / DECEL_ZONE_COUNTS;
                            // Ensure minimum speed to prevent stalling before reaching target
                            if (decel_factor < 0.2) decel_factor = 0.2;
                        }
                        
                        // Combined ramp factor: use minimum of accel and decel
                        double ramp_factor = (accel_factor < decel_factor) ? accel_factor : decel_factor;
                        
                        // Determine direction and target pulse width
                        int target_pwm_ns;
                        if (error > 0) {
                            // Forward: ramp from NEUTRAL to MAX_PWM_NS
                            int pwm_range = MAX_PWM_NS - NEUTRAL_NS;
                            target_pwm_ns = NEUTRAL_NS + (int)(pwm_range * ramp_factor);
                        } else {
                            // Reverse: ramp from NEUTRAL to MIN_PWM_NS
                            int pwm_range = NEUTRAL_NS - MIN_PWM_NS;
                            target_pwm_ns = NEUTRAL_NS - (int)(pwm_range * ramp_factor);
                        }
                        
                        // Convert to percentage for set_motor_speed
                        int pwm_percent;
                        if (target_pwm_ns > NEUTRAL_NS) {
                            // Forward
                            pwm_percent = ((target_pwm_ns - NEUTRAL_NS) * 100) / (FORWARD_MAX_NS - NEUTRAL_NS);
                        } else if (target_pwm_ns < NEUTRAL_NS) {
                            // Reverse
                            pwm_percent = -((NEUTRAL_NS - target_pwm_ns) * 100) / (NEUTRAL_NS - REVERSE_MAX_NS);
                        } else {
                            pwm_percent = 0;
                        }

                        set_motor_speed(0, pwm_percent, 1);
                    }
                 } else {
                     set_motor_speed(0, 0, 1);
                     left_done = 1;
                 }
                 pthread_mutex_unlock(&motors[0].lock);

                 // Check Right
                 pthread_mutex_lock(&motors[1].lock);
                 if (encoders[1].has_target) {
                    // Calculate relative position and error
                    int32_t current_relative = (encoders[1].total_counts + (encoders[1].current_raw_angle - encoders[1].start_raw_angle)) - encoders[1].move_start_counts;
                    int32_t error = encoders[1].target_counts - current_relative;

                    if (abs(error) < STOP_THRESHOLD) {
                        // Within stop threshold - we're done
                        set_motor_speed(1, 0, 1);
                        encoders[1].has_target = 0;
                        encoders[1].ramp_start_time = 0.0; // Reset ramp
                        right_done = 1;
                    } else {
                        double current_time = get_time_sec(); 
                        
                        // Smooth ramping control with acceleration and deceleration
                        // Initialize ramp start time if not set
                        if (encoders[1].ramp_start_time == 0.0) {
                            encoders[1].ramp_start_time = current_time;
                        }
                        
                        // Acceleration phase: ramp up from 0 to full speed
                        double elapsed = current_time - encoders[1].ramp_start_time;
                        double accel_factor = elapsed / RAMP_UP_TIME;
                        if (accel_factor > 1.0) accel_factor = 1.0; // Cap at 100%
                        
                        // Deceleration phase: slow down when approaching target
                        double decel_factor = 1.0;
                        int32_t abs_error = abs(error);
                        if (abs_error < DECEL_ZONE_COUNTS) {
                            // Within deceleration zone: reduce speed proportionally
                            decel_factor = (double)abs_error / DECEL_ZONE_COUNTS;
                            // Ensure minimum speed to prevent stalling before reaching target
                            if (decel_factor < 0.2) decel_factor = 0.2;
                        }
                        
                        // Combined ramp factor: use minimum of accel and decel
                        double ramp_factor = (accel_factor < decel_factor) ? accel_factor : decel_factor;
                        
                        // Determine direction and target pulse width
                        int target_pwm_ns;
                        if (error > 0) {
                            // Forward: ramp from NEUTRAL to MAX_PWM_NS
                            int pwm_range = MAX_PWM_NS - NEUTRAL_NS;
                            target_pwm_ns = NEUTRAL_NS + (int)(pwm_range * ramp_factor);
                        } else {
                            // Reverse: ramp from NEUTRAL to MIN_PWM_NS
                            int pwm_range = NEUTRAL_NS - MIN_PWM_NS;
                            target_pwm_ns = NEUTRAL_NS - (int)(pwm_range * ramp_factor);
                        }
                        
                        // Convert to percentage for set_motor_speed
                        int pwm_percent;
                        if (target_pwm_ns > NEUTRAL_NS) {
                            // Forward
                            pwm_percent = ((target_pwm_ns - NEUTRAL_NS) * 100) / (FORWARD_MAX_NS - NEUTRAL_NS);
                        } else if (target_pwm_ns < NEUTRAL_NS) {
                            // Reverse
                            pwm_percent = -((NEUTRAL_NS - target_pwm_ns) * 100) / (NEUTRAL_NS - REVERSE_MAX_NS);
                        } else {
                            pwm_percent = 0;
                        }

                        set_motor_speed(1, pwm_percent, 1);
                    }
                 } else {
                     set_motor_speed(1, 0, 1);
                     right_done = 1;
                 }
                 pthread_mutex_unlock(&motors[1].lock);

                 if (left_done && right_done) {
                     // Check if we just finished the 180-degree rotation for bucket approach
                     if (nav_ctrl.is_bucket_target && nav_ctrl.state == NAV_TURNING) {
                         // Check if we're rotating at the bucket (not initial approach turn)
                         double dx = nav_ctrl.bucket_x - odometry.x;
                         double dy = nav_ctrl.bucket_y - odometry.y;
                         double dist_to_bucket = sqrt(dx*dx + dy*dy);
                         
                         if (dist_to_bucket < 2.0) { // We're near the bucket (within 2ft)
                             // Transition to backup phase
                             nav_ctrl.state = NAV_BUCKET_BACKUP;
                         } else {
                             // Still approaching, continue normal navigation
                             nav_ctrl.state = NAV_GOTO;
                         }
                     } else {
                         // Normal re-evaluation
                         nav_ctrl.state = NAV_GOTO;
                     }

                     // Send immediate STATUS to notify Python of state change
                     printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                     fflush(stdout);
                 }
                 break;
            }

            case NAV_BUCKET_ROTATE: {
                // Rotate 180 degrees (shortest direction)
                double target_heading = odometry.heading + 180.0;
                if (target_heading >= 360.0) target_heading -= 360.0;
                
                // Calculate heading difference (shortest path)
                double heading_diff = target_heading - odometry.heading;
                while (heading_diff > 180) heading_diff -= 360;
                while (heading_diff < -180) heading_diff += 360;

                // Set up rotation
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

                // Transition to executing the rotation
                nav_ctrl.state = NAV_TURNING;
                nav_ctrl.target_heading = target_heading;

                printf("ROTATING_180\n");
                printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                fflush(stdout);
                break;
            }

            case NAV_BUCKET_BACKUP: {
                // Calculate distance from current position to bucket
                double dx = nav_ctrl.bucket_x - odometry.x;
                double dy = nav_ctrl.bucket_y - odometry.y;
                double current_distance = sqrt(dx*dx + dy*dy);
                
                // Calculate backup distance needed to reach 0.25ft from bucket
                double backup_distance = current_distance - 0.25;
                
                if (backup_distance > 0.1) { // Only backup if we need to move more than 0.1ft
                    // Set up reverse motion (negative counts)
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

                    // Transition to driving (which handles both forward and reverse)
                    nav_ctrl.state = NAV_DRIVING;
                    nav_ctrl.target_distance = backup_distance;

                    printf("BACKING_UP %.2f ft\n", backup_distance);
                    printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
                    fflush(stdout);
                } else {
                    // Already close enough, finish
                    printf("ARRIVED\n");
                    nav_ctrl.state = NAV_IDLE;
                    nav_ctrl.is_bucket_target = 0; // Clear bucket flag
                    
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
// Helper function to get motor state from PWM pulse width
int8_t get_motor_state(int pwm_ns) {
    if (pwm_ns > NEUTRAL_NS + 10000) return 1;   // Forward (>1510µs, 10µs hysteresis)
    if (pwm_ns < NEUTRAL_NS - 10000) return -1;  // Reverse (<1490µs, 10µs hysteresis)
    return 0;  // Neutral
}

// Helper function to calculate current position based on rotation count and raw angle
int32_t calculate_position(EncoderState *enc) {
    // Formula: 4095 * rotation_count ± current_value
    // For forward: add current angle
    // For reverse: subtract current angle (rotations already negative)
    int32_t base = COUNTS_PER_REV * enc->rotation_count;
    int32_t offset = enc->current_raw_angle - enc->start_raw_angle;
    
    return base + offset;
}

void update_encoder_rotation(EncoderState *enc, int16_t raw_angle, int motor_id) {
    // Update motor state from PWM
    int pwm_ns = motors[motor_id].last_pulse_ns;
    enc->motor_state = get_motor_state(pwm_ns);
    
    // Initialize on first valid read
    // Capture the initial encoder position so we effectively start at 0
    if (enc->last_raw_angle < 0) {
        enc->last_raw_angle = raw_angle;
        enc->current_raw_angle = raw_angle;
        enc->start_raw_angle = raw_angle;  // Set offset to current position
        enc->last_motor_state = enc->motor_state;
        return;
    }
    
    // Detect rotation completion by monitoring boundary crossings
    // Both motors use the same logic (encoders are not inverted)
    if (enc->motor_state == 1) {
        // Forward motion: crossing from high (>3000) to low (<1000)
        if (enc->last_raw_angle > 3000 && raw_angle < 1000) {
            enc->rotation_count++;
        }
    } else if (enc->motor_state == -1) {
        // Reverse motion: crossing from low (<1000) to high (>3000)
        if (enc->last_raw_angle < 1000 && raw_angle > 3000) {
            enc->rotation_count--;
        }
    }
    
    // Update state
    enc->last_raw_angle = raw_angle;
    enc->current_raw_angle = raw_angle;
    enc->last_motor_state = enc->motor_state;
    
    // Update total_counts for compatibility (will be phased out)
    enc->total_counts = calculate_position(enc);
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
            update_encoder_rotation(&encoders[0], left_angle, 0);
            pthread_mutex_unlock(&motors[0].lock);
        }

        // Process right motor encoder
        if (right_angle >= 0) {
            pthread_mutex_lock(&motors[1].lock);
            update_encoder_rotation(&encoders[1], right_angle, 1);
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
    static int first_update = 1;
    
    double current_time = get_time_sec();
    double dt = current_time - last_imu_time;
    last_imu_time = current_time;

    // Initialize odometry tracking on first update to prevent position jump
    if (first_update) {
        odometry.last_left_total = encoders[0].total_counts;
        odometry.last_right_total = encoders[1].total_counts;
        first_update = 0;
        return;  // Skip first update to avoid spurious delta
    }

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

    // Apply gyro deadband to prevent drift when stationary
    // Ignore gyro readings less than 0.5 deg/sec
    if (fabs(gyro_rate) < 0.25) {
        gyro_rate = 0.0;
    }

    // 4. IMU Integration
    // Only integrate gyro if robot is actually moving (encoders changing)
    // This prevents heading drift when stationary
    double dt_seconds = dt;
    double delta_heading = 0.0;
    
    // Check if robot is moving (either wheel has moved)
    if (fabs(center_dist) > 0.001) {  // More than 0.001 feet movement
        delta_heading = gyro_rate * dt_seconds;
    }
    
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

/**
 * Process incoming command from stdin
 * 
 * Parses and executes commands from the Python backend including:
 * - goto: Autonomous navigation to coordinates
 * - speed: Set navigation speed multiplier
 * - setpwm: Set PWM limits from slider
 * - setpos: Manually set odometry position
 * - calibrate: Calibrate IMU and reset position
 * - stop: Emergency stop and log dump
 * - pulse: Direct PWM control (joystick mode)
 */
void process_command(char* cmd) {
    cmd[strcspn(cmd, "\n")] = 0;

    // Debug logging to trace command reception
    fprintf(stderr, "DEBUG: Received command: '%s'\n", cmd);
    fflush(stderr);

    if (strncasecmp(cmd, "goto", 4) == 0) {
        double x, y;
        int is_bucket = 0;
        
        // Try to parse with optional bucket flag: "goto x y [is_bucket]"
        int parsed = sscanf(cmd + 4, "%lf %lf %d", &x, &y, &is_bucket);
        
        if (parsed >= 2) { // At least x and y were parsed
            set_control_mode(MODE_VOICE_NAV);
            nav_ctrl.target_x = x;
            nav_ctrl.target_y = y;
            nav_ctrl.is_bucket_target = is_bucket;
            
            if (is_bucket) {
                // Store actual bucket coordinates for backup calculation
                nav_ctrl.bucket_x = x;
                nav_ctrl.bucket_y = y;
            }
            
            nav_ctrl.state = NAV_GOTO;
            printf("OK goto %.2f %.2f (bucket=%d)\n", x, y, is_bucket);
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
            nav_ctrl.speed_percent = s;
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

            // Convert percentage values to nanoseconds
            // JavaScript sends percentages, we convert to pulse widths
            // min_pwm and max_pwm are 0-100, representing range from neutral
            
            // For reverse (min): percentage of range from NEUTRAL to REVERSE_MAX
            g_min_pwm_ns = NEUTRAL_NS - ((NEUTRAL_NS - REVERSE_MAX_NS) * min_pwm / 100);
            
            // For forward (max): percentage of range from NEUTRAL to FORWARD_MAX
            g_max_pwm_ns = NEUTRAL_NS + ((FORWARD_MAX_NS - NEUTRAL_NS) * max_pwm / 100);
            
            printf("OK setpwm %d %d (min=%dns, max=%dns)\n", min_pwm, max_pwm, g_min_pwm_ns, g_max_pwm_ns);
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
    else if (strncasecmp(cmd, "calibrate", 9) == 0) {
        // Calibrate IMU and reset position to start
        imu_calibrate(500);  // 2.5 second calibration for better accuracy
        
        // Reset to start position (0, 15, 0)
        odometry.x = 0.0;
        odometry.y = 15.0;
        odometry.heading = 0.0;
        
        // Reset encoder offsets to current position (zero the encoders)
        pthread_mutex_lock(&motors[0].lock);
        encoders[0].start_raw_angle = encoders[0].current_raw_angle;
        pthread_mutex_unlock(&motors[0].lock);
        
        pthread_mutex_lock(&motors[1].lock);
        encoders[1].start_raw_angle = encoders[1].current_raw_angle;
        pthread_mutex_unlock(&motors[1].lock);
        
        // Reset accumulation to avoid jumps
        odometry.last_left_total = encoders[0].total_counts;
        odometry.last_right_total = encoders[1].total_counts;
        
        printf("OK calibrate\n");
        fflush(stdout);
        
        // Send immediate STATUS update
        printf("STATUS %.2f %.2f %.2f %d\n", odometry.x, odometry.y, odometry.heading, nav_ctrl.state);
        fflush(stdout);
    }
    else if (strncasecmp(cmd, "stop", 4) == 0) {
        set_control_mode(MODE_IDLE);
        nav_ctrl.state = NAV_IDLE;
        for (int i = 0; i < 2; i++) {
            pthread_mutex_lock(&motors[i].lock);
            encoders[i].has_target = 0;
            set_motor_speed(i, 0, 1); // IMMEDIATE STOP
            pthread_mutex_unlock(&motors[i].lock);
        }
        // Force log dump and reset for new session
        dump_log();
        init_log_system();
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
            set_control_mode(MODE_JOYSTICK);

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

/**
 * Command input thread
 * 
 * Reads commands from stdin and processes them. Runs until EOF or
 * the running flag is cleared.
 */
void* command_input_thread(void* arg) {
    (void)arg;
    char buffer[256];
    while (running && fgets(buffer, sizeof(buffer), stdin) != NULL) {
        process_command(buffer);
    }
    running = 0;
    return NULL;
}

/**
 * Main entry point
 * 
 * Initializes all subsystems (I2C, PWM, IMU, logging), spawns control threads,
 * and waits for shutdown signal.
 */
int main(void) {
    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize I2C bus for sensors
    if (i2c_init() < 0) {
        fprintf(stderr, "ERROR: I2C init failed\n");
        return 1;
    }

    // Initialize PWM for motor control
    if (pwm_init() < 0) {
        fprintf(stderr, "ERROR: PWM init failed\n");
        i2c_cleanup();
        return 1;
    }

    // Initialize telemetry logging system
    init_log_system();

    // Initialize IMU (optional, system continues without it)
    if (imu_init() < 0) {
        fprintf(stderr, "WARNING: IMU init failed (check wiring to I2C3). Continuing without IMU.\n");
    } else {
        imu_calibrate(500); // 2.5 second calibration for better accuracy
    }

    // Initialize Kalman Filter for heading estimation
    kalman_init(&kf_heading);
    kf_heading.angle = START_HEADING;
    last_imu_time = get_time_sec();
    
    // Initialize Encoders
    for(int i=0; i<2; i++) {
        encoders[i].total_counts = 0;
        encoders[i].current_raw_angle = 0;
        encoders[i].last_raw_angle = -1; // Flag as invalid
        encoders[i].start_raw_angle = 0;
        
    // Rotation-based tracking fields
        encoders[i].rotation_count = 0;
        encoders[i].motor_state = 0;
        encoders[i].last_motor_state = 0;
        
        encoders[i].target_counts = 0;
        encoders[i].has_target = 0;
    }

    pthread_mutex_init(&motors[0].lock, NULL);
    pthread_mutex_init(&motors[1].lock, NULL);


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
