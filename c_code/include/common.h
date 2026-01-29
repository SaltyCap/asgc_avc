#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <time.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constants
#define COUNTS_PER_REV 4096
#define STOP_THRESHOLD 50          // Stop when within 50 counts (~0.5 inches)
#define DEADBAND_THRESHOLD 50      // Don't reverse if within 50 counts (~0.5 inches)

// Robot Physical Constants (from course_config.py)
#define WHEEL_DIAMETER_INCHES 5.3
#define WHEELBASE_INCHES 16.0
#define INCHES_PER_FOOT 12
#define WHEEL_CIRCUMFERENCE_INCHES (M_PI * WHEEL_DIAMETER_INCHES)
#define COUNTS_PER_INCH (COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES)
#define COUNTS_PER_FOOT (COUNTS_PER_INCH * INCHES_PER_FOOT)


// Time utilities
// Time utilities
double get_time_sec(void);
void sleep_us(uint32_t microseconds);
void sleep_ms(uint32_t ms);

// --- State Structures ---

// Odometry State
typedef struct {
    double x;           // feet
    double y;           // feet
    double heading;     // degrees
    int32_t last_left_total;
    int32_t last_right_total;
} OdometryState;

// Navigation State Machine State
typedef enum {
    NAV_IDLE,
    NAV_TURNING,
    NAV_DRIVING,
    NAV_GOTO        // Meta-state: planning move to target
} NavState;

// Navigation Controller State
typedef struct {
    NavState state;
    double target_x;
    double target_y;
    double target_heading;  // For TURN state
    double target_distance; // For DRIVE state
    double speed_multiplier; // 0.0 to 1.0 from slider
} NavigationController;

#endif
