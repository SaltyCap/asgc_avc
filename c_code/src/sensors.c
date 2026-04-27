#include "../include/sensors.h"
#include "../include/i2c.h"
#include "../include/imu.h"
#include "../include/common.h"
#include <stdint.h>

// Encoder loop targets 1kHz; divider 10 yields ~100Hz IMU polling.
#define IMU_POLL_DIVIDER 10

// Read all sensors once per cycle.
// Direct reads avoid per-cycle thread creation overhead in the real-time loop.
SensorData read_all_sensors(void) {
    static double last_gyro_z = 0.0;
    static unsigned int imu_cycle = 0;
    SensorData result = {0, 0, 0.0, 0.0, 0};
    
    // Capture timestamp BEFORE starting reads for precise synchronization
    // This ensures all sensor data corresponds to the same time instant
    result.timestamp = get_time_sec();
    
    int16_t left_angle = read_raw_angle(0);   // Left encoder on I2C3
    int16_t right_angle = read_raw_angle(1);  // Right encoder on I2C1

    // Poll IMU less often so encoder polling can run faster and more consistently.
    if ((imu_cycle++ % IMU_POLL_DIVIDER) == 0) {
        double imu_sample = 0.0;
        if (imu_read_gyro_z_sample(&imu_sample)) {
            last_gyro_z = imu_sample;
            result.imu_valid = 1;
        }
    }
    
    // Combine results
    result.left_encoder = left_angle;
    result.right_encoder = right_angle;
    result.gyro_z = last_gyro_z;
    
    return result;
}
