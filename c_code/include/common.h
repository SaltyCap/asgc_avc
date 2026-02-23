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
#define STOP_THRESHOLD 200          // Stop when within 200 counts (~0.8 inches)

// Robot Physical Constants (from course_config.py)
#define WHEEL_DIAMETER_INCHES 5.3
#define WHEELBASE_INCHES 16.0
#define INCHES_PER_FOOT 12
#define WHEEL_CIRCUMFERENCE_INCHES (M_PI * WHEEL_DIAMETER_INCHES)
#define COUNTS_PER_INCH (COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES)
#define COUNTS_PER_FOOT (COUNTS_PER_INCH * INCHES_PER_FOOT)

// Starting Configuration (Must match course_config.py)
#define START_X 0.0
#define START_Y 15.0
#define START_HEADING 0.0

// Course landmarks (Must match course_config.py)
#define COURSE_CENTER_X 15.0
#define COURSE_CENTER_Y 15.0
#define BUCKET_RED_X 0.0
#define BUCKET_RED_Y 0.0
#define BUCKET_YELLOW_X 0.0
#define BUCKET_YELLOW_Y 30.0
#define BUCKET_BLUE_X 30.0
#define BUCKET_BLUE_Y 30.0
#define BUCKET_GREEN_X 30.0
#define BUCKET_GREEN_Y 0.0

// ML pickup search area at course center (30in diameter).
#define ML_PICKUP_CIRCLE_DIAMETER_FT (30.0 / 12.0)
#define ML_PICKUP_CIRCLE_RADIUS_FT (ML_PICKUP_CIRCLE_DIAMETER_FT / 2.0)

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
    NAV_GOTO,           // Meta-state: planning move to target
    NAV_BUCKET_ROTATE,   // Rotating 180 degrees at bucket
    NAV_BUCKET_BACKUP,    // Backing up to 0.25ft from bucket
    NAV_ML               // ML Inference Control
} NavState;

typedef enum {
    BALL_COLOR_NONE = 0,
    BALL_COLOR_RED = 1,
    BALL_COLOR_YELLOW = 2,
    BALL_COLOR_BLUE = 3,
    BALL_COLOR_GREEN = 4
} BallColor;

typedef enum {
    ML_STAGE_INACTIVE = 0,
    ML_STAGE_TO_CENTER = 1,
    ML_STAGE_PICKUP_SWEEP = 2,
    ML_STAGE_WAIT_BALL = 3,
    ML_STAGE_TO_BUCKET = 4
} MLStage;

// Navigation Controller State
typedef struct {
    NavState state;
    double target_x;
    double target_y;
    double target_heading;  // For TURN state
    double target_distance; // For DRIVE state

    int is_bucket_target;   // 1 if navigating to a colored bucket
    double bucket_x;        // Actual bucket coordinates
    double bucket_y;

    int ml_test_enabled;    // 1 while ML pick-and-place workflow is active
    MLStage ml_stage;       // Current ML workflow stage
    int ml_ball_acquired;   // 1 after external pickup confirmation
    BallColor ml_ball_color;// Color of currently held ball
    int ml_sweep_waypoint_index; // Current pickup sweep waypoint
} NavigationController;

#endif
