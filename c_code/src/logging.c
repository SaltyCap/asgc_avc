#include "../include/logging.h"
#include "../include/control_loop.h"
#include "../include/motor.h"
#include "../include/runtime_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

// 6M entries at 20kHz = ~5 minutes of high-speed telemetry (~528MB RAM)
#define LOG_SIZE 6000000

// Maximum number of duplicate files to check before giving up
#define MAX_FILE_DUPLICATES 1000

// Log buffer and state
static LogEntry *log_buffer = NULL;
static int log_index = 0;
static ControlMode current_mode = MODE_IDLE;
static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;

static const char* control_mode_name(char mode) {
    switch ((ControlMode)mode) {
        case MODE_IDLE:
            return "IDLE";
        case MODE_JOYSTICK:
            return "JOYSTICK";
        case MODE_VOICE_NAV:
            return "VOICE";
        case MODE_ML:
            return "ML";
        default:
            return "UNKNOWN";
    }
}

static const char* nav_state_name(char nav_state) {
    switch ((NavState)nav_state) {
        case NAV_IDLE:
            return "IDLE";
        case NAV_TURNING:
            return "TURNING";
        case NAV_DRIVING:
            return "DRIVING";
        case NAV_GOTO:
            return "GOTO";
        case NAV_BUCKET_ROTATE:
            return "BUCKET_ROTATE";
        case NAV_BUCKET_BACKUP:
            return "BUCKET_BACKUP";
        case NAV_ML:
            return "ML";
        default:
            return "UNKNOWN";
    }
}

static const char* nav_controller_mode_name(char mode) {
    return (mode == (char)NAV_CONTROLLER_ML) ? "ML" : "PID";
}

void init_log_system(void) {
    pthread_mutex_lock(&log_lock);
    if (log_buffer) {
        free(log_buffer);
        log_buffer = NULL;
    }
    log_buffer = (LogEntry*)malloc(sizeof(LogEntry) * LOG_SIZE);
    if (!log_buffer) {
        printf("ERROR: Failed to allocate log buffer (%zu bytes)\n", 
               sizeof(LogEntry) * LOG_SIZE);
    } else {
        printf("Allocated log buffer (%d entries, %zu MB)\n", 
               LOG_SIZE, (sizeof(LogEntry) * LOG_SIZE) / (1024 * 1024));
    }
    log_index = 0;
    pthread_mutex_unlock(&log_lock);
}

void log_data(double time) {
    LogEntry snapshot = {0};
    snapshot.time = time;

    // Maintain lock order compatibility with control loop: state -> motor.
    pthread_mutex_lock(&state_lock);
    snapshot.odom_x = odometry.x;
    snapshot.odom_y = odometry.y;
    snapshot.odom_heading = odometry.heading;
    snapshot.nav_state = (char)nav_ctrl.state;
    pthread_mutex_unlock(&state_lock);

    pthread_mutex_lock(&log_lock);
    snapshot.mode = (char)current_mode;
    pthread_mutex_unlock(&log_lock);
    snapshot.nav_controller_mode = (char)control_get_nav_controller_mode();

    // Capture left motor state (thread-safe).
    pthread_mutex_lock(&motors[0].lock);
    snapshot.target_l = encoders[0].target_counts;
    snapshot.actual_l = encoders[0].total_counts - encoders[0].move_start_counts;
    snapshot.pulse_l = motors[0].last_pulse_ns;
    snapshot.raw_l = encoders[0].current_raw_angle;
    snapshot.vel_l = encoders[0].velocity_counts_per_sec;
    snapshot.accel_l = encoders[0].acceleration_counts_per_sec2;
    pthread_mutex_unlock(&motors[0].lock);

    // Capture right motor state (thread-safe).
    pthread_mutex_lock(&motors[1].lock);
    snapshot.target_r = encoders[1].target_counts;
    snapshot.actual_r = encoders[1].total_counts - encoders[1].move_start_counts;
    snapshot.pulse_r = motors[1].last_pulse_ns;
    snapshot.raw_r = encoders[1].current_raw_angle;
    snapshot.vel_r = encoders[1].velocity_counts_per_sec;
    snapshot.accel_r = encoders[1].acceleration_counts_per_sec2;
    pthread_mutex_unlock(&motors[1].lock);

    // Capture IMU data (thread-safe). Keep lock ordering compatible with
    // update_odometry() to avoid lock inversion.
    pthread_mutex_lock(&imu_data_lock);
    snapshot.gyro_z = current_gyro_rate;
    pthread_mutex_unlock(&imu_data_lock);

    // Commit snapshot to buffer.
    pthread_mutex_lock(&log_lock);
    if (log_buffer && log_index < LOG_SIZE) {
        log_buffer[log_index] = snapshot;
        log_index++;
    }
    pthread_mutex_unlock(&log_lock);
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
    fprintf(f, "time,mode,nav_controller_mode,pwm_l,i2c_l,pwm_r,i2c_r,target_l,actual_l,target_r,actual_r,"
               "vel_l,accel_l,vel_r,accel_r,gyro_z,odom_x,odom_y,odom_heading,nav_state\n");

    // Write all log entries
    for (int i = 0; i < log_index; i++) {
        fprintf(f, "%.4f,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%s\n",
            log_buffer[i].time,
            control_mode_name(log_buffer[i].mode),
            nav_controller_mode_name(log_buffer[i].nav_controller_mode),
            log_buffer[i].pulse_l, log_buffer[i].raw_l,
            log_buffer[i].pulse_r, log_buffer[i].raw_r,
            log_buffer[i].target_l, log_buffer[i].actual_l,
            log_buffer[i].target_r, log_buffer[i].actual_r,
            log_buffer[i].vel_l, log_buffer[i].accel_l,
            log_buffer[i].vel_r, log_buffer[i].accel_r,
            log_buffer[i].gyro_z,
            log_buffer[i].odom_x,
            log_buffer[i].odom_y,
            log_buffer[i].odom_heading,
            nav_state_name(log_buffer[i].nav_state));
    }
}

void dump_log(void) {
    pthread_mutex_lock(&log_lock);
    if (!log_buffer) {
        pthread_mutex_unlock(&log_lock);
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
    int ml_count = 0;
    for (int i = 0; i < log_index; i++) {
        if (log_buffer[i].mode == MODE_JOYSTICK) {
            joystick_count++;
        } else if (log_buffer[i].mode == MODE_VOICE_NAV) {
            voice_count++;
        } else if (log_buffer[i].mode == MODE_ML) {
            ml_count++;
        }
    }

    // Determine primary mode (whichever has more entries)
    const char *mode_str = "voice";
    int max_count = voice_count;
    if (joystick_count > max_count) {
        mode_str = "joystick";
        max_count = joystick_count;
    }
    if (ml_count > max_count) {
        mode_str = "ml";
    }

    // Ensure the persistent log directory exists.
    if (mkdir("../logs", 0755) < 0 && errno != EEXIST) {
        fprintf(stderr, "ERROR: Could not create ../logs directory: %s\n", strerror(errno));
        pthread_mutex_unlock(&log_lock);
        return;
    }

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
            pthread_mutex_unlock(&log_lock);
            return;
        }
    }

    // Write permanent log file
    FILE *f = fopen(filename, "w");
    if (!f) {
        fprintf(stderr, "ERROR: Could not open log file %s\n", filename);
        pthread_mutex_unlock(&log_lock);
        return;
    }

    write_log_csv(f);
    fclose(f);

    printf("Saved %d log entries to %s\n", log_index, filename);
    printf("  Joystick entries: %d, Voice navigation entries: %d, ML entries: %d\n", 
           joystick_count, voice_count, ml_count);

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
    pthread_mutex_unlock(&log_lock);
}

void set_control_mode(ControlMode mode) {
    pthread_mutex_lock(&log_lock);
    current_mode = mode;
    pthread_mutex_unlock(&log_lock);
}

ControlMode get_control_mode(void) {
    pthread_mutex_lock(&log_lock);
    ControlMode mode = current_mode;
    pthread_mutex_unlock(&log_lock);
    return mode;
}

static pthread_t logging_thread_id;
static int logging_thread_running = 0;

static void* logging_thread(void* arg) {
    (void)arg;
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 50000; // 50 microseconds = 20kHz

    printf("Logging thread started at 20kHz\n");

    while (logging_thread_running) {
        log_data(get_time_sec());
        nanosleep(&ts, NULL);
    }
    return NULL;
}

int start_logging_thread(void) {
    if (logging_thread_running) {
        return 0; // Already running
    }

    logging_thread_running = 1;
    if (pthread_create(&logging_thread_id, NULL, logging_thread, NULL) != 0) {
        perror("Failed to create logging thread");
        logging_thread_running = 0;
        return -1;
    }
    return 0;
}

void stop_logging_thread(void) {
    if (!logging_thread_running) {
        return;
    }

    logging_thread_running = 0;
    pthread_join(logging_thread_id, NULL);
    printf("Logging thread stopped\n");
}
