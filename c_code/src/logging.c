#include "../include/logging.h"
#include "../include/motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

// Maximum number of log entries before buffer is full (~48MB RAM, ~1.4 hrs at 500Hz)
// Reduced from 15M to prevent out-of-memory issues
#define LOG_SIZE 1000000

// Maximum number of duplicate files to check before giving up
#define MAX_FILE_DUPLICATES 1000

// Log buffer and state
static LogEntry *log_buffer = NULL;
static int log_index = 0;
static ControlMode current_mode = MODE_IDLE;

// External references to motor and encoder state (defined in motor.h)
extern Motor motors[2];
extern EncoderState encoders[2];
extern OdometryState odometry;
extern NavigationController nav_ctrl;
extern pthread_mutex_t imu_data_lock;
extern double current_gyro_rate;

void init_log_system(void) {
    log_buffer = (LogEntry*)malloc(sizeof(LogEntry) * LOG_SIZE);
    if (!log_buffer) {
        printf("ERROR: Failed to allocate log buffer (%zu bytes)\n", 
               sizeof(LogEntry) * LOG_SIZE);
    } else {
        printf("Allocated log buffer (%d entries, %zu MB)\n", 
               LOG_SIZE, (sizeof(LogEntry) * LOG_SIZE) / (1024 * 1024));
    }
    log_index = 0;
}

void log_data(double time) {
    // Silently ignore if buffer not initialized or full
    if (!log_buffer || log_index >= LOG_SIZE) {
        return;
    }

    LogEntry *entry = &log_buffer[log_index];
    entry->time = time;
    entry->mode = (char)current_mode;

    // Capture left motor state (thread-safe)
    pthread_mutex_lock(&motors[0].lock);
    entry->target_l = encoders[0].target_counts;
    entry->actual_l = encoders[0].total_counts + 
                      (encoders[0].current_raw_angle - encoders[0].start_raw_angle);
    entry->pulse_l = motors[0].last_pulse_ns;
    entry->raw_l = encoders[0].current_raw_angle;
    pthread_mutex_unlock(&motors[0].lock);

    // Capture right motor state (thread-safe)
    pthread_mutex_lock(&motors[1].lock);
    entry->target_r = encoders[1].target_counts;
    entry->actual_r = encoders[1].total_counts + 
                      (encoders[1].current_raw_angle - encoders[1].start_raw_angle);
    entry->pulse_r = motors[1].last_pulse_ns;
    entry->raw_r = encoders[1].current_raw_angle;
    pthread_mutex_unlock(&motors[1].lock);

    // Capture IMU data (thread-safe)
    pthread_mutex_lock(&imu_data_lock);
    entry->gyro_z = current_gyro_rate;
    pthread_mutex_unlock(&imu_data_lock);

    // Capture odometry data (updated by coordinated_control_thread)
    entry->odom_x = odometry.x;
    entry->odom_y = odometry.y;
    entry->odom_heading = odometry.heading;

    // Capture navigation state
    entry->nav_state = (char)nav_ctrl.state;

    log_index++;
}

/**
 * Write log entries to a file in CSV format
 * 
 * Helper function to reduce code duplication. Writes CSV header and all
 * log entries to the specified file.
 * 
 * @param f File pointer to write to (must be open for writing)
 */
static void write_log_csv(FILE *f) {
    // CSV header with all telemetry columns
    fprintf(f, "time,mode,pwm_l,i2c_l,pwm_r,i2c_r,target_l,actual_l,target_r,actual_r,"
               "gyro_z,odom_x,odom_y,odom_heading,nav_state\n");

    // Mode and navigation state name mappings
    const char *mode_names[] = {"IDLE", "JOYSTICK", "VOICE"};
    const char *nav_state_names[] = {
        "IDLE", "TURNING", "DRIVING", "GOTO", 
        "BUCKET_APPROACH", "BUCKET_ROTATE", "BUCKET_BACKUP"
    };

    // Write all log entries
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
}

void dump_log(void) {
    if (!log_buffer) {
        return;
    }

    // Generate timestamp for unique filename
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char filename[512];
    char temp_filename[512];

    // Count entries by mode to determine primary mode for filename
    int joystick_count = 0;
    int voice_count = 0;
    for (int i = 0; i < log_index; i++) {
        if (log_buffer[i].mode == MODE_JOYSTICK) {
            joystick_count++;
        } else if (log_buffer[i].mode == MODE_VOICE_NAV) {
            voice_count++;
        }
    }

    // Determine primary mode (whichever has more entries)
    const char *mode_str = (joystick_count > voice_count) ? "joystick" : "voice";

    // Find unique filename by checking for duplicates
    // Auto-increment counter if file already exists to prevent overwriting
    int duplicate_file_counter = 0;
    while (1) {
        if (duplicate_file_counter == 0) {
            snprintf(filename, sizeof(filename),
                     "../logs/motor_log_%s_%04d%02d%02d_%02d%02d%02d.csv",
                     mode_str, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                     t->tm_hour, t->tm_min, t->tm_sec);
        } else {
            snprintf(filename, sizeof(filename),
                     "../logs/motor_log_%s_%04d%02d%02d_%02d%02d%02d_%d.csv",
                     mode_str, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                     t->tm_hour, t->tm_min, t->tm_sec, duplicate_file_counter);
        }

        // Check if file exists
        FILE *test = fopen(filename, "r");
        if (!test) {
            // File doesn't exist, we can use this filename
            break;
        }
        fclose(test);
        duplicate_file_counter++;

        // Safety: prevent infinite loop
        if (duplicate_file_counter > MAX_FILE_DUPLICATES) {
            fprintf(stderr, "ERROR: Too many log files with same timestamp\n");
            return;
        }
    }

    // Write permanent log file
    FILE *f = fopen(filename, "w");
    if (!f) {
        fprintf(stderr, "ERROR: Could not open log file %s\n", filename);
        return;
    }

    write_log_csv(f);
    fclose(f);

    printf("Saved %d log entries to %s\n", log_index, filename);
    printf("  Joystick entries: %d, Voice navigation entries: %d\n", 
           joystick_count, voice_count);

    // Also save to RAM disk for quick access during session
    // This allows real-time log analysis without disk I/O delays
    snprintf(temp_filename, sizeof(temp_filename),
             "/dev/shm/motor_log_%s_latest.csv", mode_str);

    FILE *f_temp = fopen(temp_filename, "w");
    if (f_temp) {
        write_log_csv(f_temp);
        fclose(f_temp);
        printf("  Quick access copy: %s\n", temp_filename);
    }

    // Free buffer after dumping to save memory
    // Note: This prevents further logging until init_log_system() is called again
    free(log_buffer);
    log_buffer = NULL;
    log_index = 0;
}

void set_control_mode(ControlMode mode) {
    current_mode = mode;
}

ControlMode get_control_mode(void) {
    return current_mode;
}
