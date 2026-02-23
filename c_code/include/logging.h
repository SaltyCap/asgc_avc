#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>

#define LOG_RATE_HZ 100

/**
 * Control modes for telemetry logging
 * 
 * Tracks the current control mode to categorize log entries and
 * determine appropriate log file naming.
 */
typedef enum {
    MODE_IDLE = 0,        // System idle, no active control
    MODE_JOYSTICK = 1,    // Direct pulse width commands from joystick/manual control
    MODE_VOICE_NAV = 2,   // Autonomous navigation from voice/goto commands
    MODE_ML = 3           // ML inference-based motor control
} ControlMode;

/**
 * Single telemetry log entry
 * 
 * Captures complete system state at a single point in time including
 * motor commands, encoder feedback, IMU data, odometry, and navigation state.
 * Logged at a fixed configured rate (100Hz by default).
 */
typedef struct {
    double time;          // Timestamp in seconds since program start
    
    // Motor control and encoder feedback
    int32_t target_l;     // Left motor target position (encoder counts)
    int32_t actual_l;     // Left motor actual position (encoder counts)
    int pulse_l;          // Left motor PWM pulse width (nanoseconds)
    int raw_l;            // Left encoder raw angle (0-4095)
    int32_t target_r;     // Right motor target position (encoder counts)
    int32_t actual_r;     // Right motor actual position (encoder counts)
    int pulse_r;          // Right motor PWM pulse width (nanoseconds)
    int raw_r;            // Right encoder raw angle (0-4095)
    
    char mode;            // Control mode: 0=IDLE, 1=JOYSTICK, 2=VOICE_NAV, 3=ML
    
    // IMU data
    double gyro_z;        // Z-axis gyro rate (degrees/sec)
    
    // Odometry data
    double odom_x;        // X position (feet)
    double odom_y;        // Y position (feet)
    double odom_heading;  // Heading (degrees, 0-360)
    
    // Navigation state
    char nav_state;       // Navigation state machine state
} LogEntry;

/**
 * Initialize the logging system
 * 
 * Allocates memory for the log buffer. Must be called before any logging
 * operations. Prints error message if allocation fails but does not exit.
 */
void init_log_system(void);

/**
 * Log current system state
 * 
 * Captures a snapshot of all telemetry data and stores it in the log buffer.
 * Should be called at regular intervals (typically 100Hz) from the control loop.
 * Silently ignores calls if buffer is full or not initialized.
 * 
 * @param time Current timestamp in seconds since program start
 */
void log_data(double time);

/**
 * Write log buffer to CSV files
 * 
 * Writes all logged entries to timestamped CSV files in ../logs/ directory
 * and creates a quick-access copy in /dev/shm/. Automatically determines
 * filename based on primary control mode (joystick vs voice navigation).
 * Frees the log buffer after writing.
 * 
 * Safe to call multiple times or when buffer is empty.
 */
void dump_log(void);

/**
 * Set the current control mode
 * 
 * Updates the control mode for subsequent log entries. Used to track
 * whether the system is in manual joystick control or autonomous navigation.
 * 
 * @param mode New control mode to set
 */
void set_control_mode(ControlMode mode);

/**
 * Get the current control mode
 * 
 * @return Current control mode
 */
ControlMode get_control_mode(void);


/**
 * Start the high-speed (20kHz) logging thread
 * 
 * Spawns a dedicated thread that samples system state every 50μs.
 * Must be called after init_log_system().
 * 
 * @return 0 on success, < 0 on failure
 */
int start_logging_thread(void);

/**
 * Stop the logging thread
 * 
 * Signals the logging thread to terminate and joins it.
 */
void stop_logging_thread(void);

#endif
