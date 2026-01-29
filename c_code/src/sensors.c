#include "../include/sensors.h"
#include "../include/i2c.h"
#include "../include/imu.h"
#include "../include/common.h"
#include <pthread.h>
#include <stdint.h>

// Thread data structures for parallel reads
typedef struct {
    int16_t angle;
    int success;
} EncoderThreadData;

typedef struct {
    double gyro_z;
    int success;
} IMUThreadData;

// Thread function to read left encoder (I2C3)
static void* read_left_encoder_thread(void* arg) {
    EncoderThreadData* data = (EncoderThreadData*)arg;
    data->angle = read_raw_angle(0);  // Motor ID 0 = left encoder on Bus 3 (via i2c1_fd)
    data->success = (data->angle >= 0) ? 1 : 0;
    return NULL;
}

// Thread function to read right encoder (I2C1)
static void* read_right_encoder_thread(void* arg) {
    EncoderThreadData* data = (EncoderThreadData*)arg;
    data->angle = read_raw_angle(1);  // Motor ID 1 = right encoder on Bus 1 (via i2c2_fd)
    data->success = (data->angle >= 0) ? 1 : 0;
    return NULL;
}

// Thread function to read IMU (I2C2)
static void* read_imu_thread(void* arg) {
    IMUThreadData* data = (IMUThreadData*)arg;
    data->gyro_z = imu_read_gyro_z();
    data->success = 1;  // imu_read_gyro_z returns 0.0 on error, which is valid
    return NULL;
}

// Read all sensors simultaneously using three threads
// Left encoder on I2C1, right encoder on I2C2, IMU on I2C3
// All three I2C buses accessed in parallel for maximum performance
SensorData read_all_sensors(void) {
    SensorData result = {0, 0, 0.0, 0.0, 0};
    
    // Capture timestamp BEFORE starting reads for precise synchronization
    // This ensures all sensor data corresponds to the same time instant
    result.timestamp = get_time_sec();
    
    // Thread handles for all three sensors
    pthread_t left_thread, right_thread, imu_thread;
    EncoderThreadData left_data = {-1, 0};
    EncoderThreadData right_data = {-1, 0};
    IMUThreadData imu_data = {0.0, 0};
    
    // Launch all three threads simultaneously
    // Each accesses a different I2C bus - true parallel execution
    pthread_create(&left_thread, NULL, read_left_encoder_thread, &left_data);
    pthread_create(&right_thread, NULL, read_right_encoder_thread, &right_data);
    pthread_create(&imu_thread, NULL, read_imu_thread, &imu_data);
    
    // Wait for all three to complete
    pthread_join(left_thread, NULL);
    pthread_join(right_thread, NULL);
    pthread_join(imu_thread, NULL);
    
    // Combine results
    result.left_encoder = left_data.angle;
    result.right_encoder = right_data.angle;
    result.gyro_z = imu_data.gyro_z;
    result.valid = left_data.success && right_data.success && imu_data.success;
    
    return result;
}
