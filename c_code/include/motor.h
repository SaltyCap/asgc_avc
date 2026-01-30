#ifndef MOTOR_H
#define MOTOR_H

#include <pthread.h>
#include <stdint.h>
#include "common.h" // For OdometryState and NavigationController

// PWM Configuration
#define PWM_CHANNEL_LEFT 0   // GPIO 12 
#define PWM_CHANNEL_RIGHT 1  // GPIO 13
#define PWM_PERIOD_NS 2500000
#define NEUTRAL_NS 1500000
#define FORWARD_START_NS 1500000  // No Dead Band
#define FORWARD_MAX_NS 2000000
#define REVERSE_START_NS 1500000  // No Dead Band
#define REVERSE_MAX_NS 1000000



typedef struct {
    int id;
    int pwm_duty_fd;
    int pwm_enable_fd;
    int current_speed;
    int last_pulse_ns;           // For ramp rate limiting (in nanoseconds)
    double last_speed_update_time;  // For ramp rate limiting
    pthread_mutex_t lock;
} Motor;

typedef struct {
    int32_t total_counts;      // Accumulated counts (mult-turn) - DEPRECATED, use rotation-based calculation
    int16_t current_raw_angle; // Current 0-4095 angle
    int16_t start_raw_angle;   // Angle at start of move
    
    // Rotation-based tracking
    int32_t rotation_count;    // Number of complete rotations
    int8_t motor_state;        // Current motor direction: -1 (reverse), 0 (neutral), 1 (forward)
    int8_t last_motor_state;   // Previous motor direction for rotation detection
    int16_t last_raw_angle;    // Previous raw angle for rotation boundary detection

    int32_t target_counts;     // Target relative distance
    int32_t move_start_counts; // Total counts at start of current move (for relative tracking)
    int has_target;            // Flag

    // Stall detection
    int32_t stall_last_position; // Position at last stall check
    double stall_check_time;     // Time of last stall check
    int stall_count;             // Number of consecutive stalls
} EncoderState;

// Global arrays
extern Motor motors[2];
extern EncoderState encoders[2];

// Note: OdometryState and NavigationController moved to common.h

extern OdometryState odometry;
extern NavigationController nav_ctrl;

int pwm_init(void);
void pwm_cleanup(void);
void set_motor_speed(int motor_id, int speed_percent, int immediate);

// Motor state accessor functions
int8_t get_left_motor_state(void);   // Returns -1 (reverse), 0 (neutral), 1 (forward)
int8_t get_right_motor_state(void);  // Returns -1 (reverse), 0 (neutral), 1 (forward)
int32_t get_left_rotation_count(void);
int32_t get_right_rotation_count(void);
int32_t get_left_position(void);     // Returns 4095 * rotation_count + current_value
int32_t get_right_position(void);    // Returns 4095 * rotation_count + current_value



#endif
