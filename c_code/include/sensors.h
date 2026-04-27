#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// Combined sensor data structure
typedef struct {
    int16_t left_encoder;   // Left motor encoder raw angle
    int16_t right_encoder;  // Right motor encoder raw angle
    double gyro_z;          // Latest IMU Z-axis gyro rate (degrees/sec)
    double timestamp;       // Timestamp when sensors were read (seconds)
    int imu_valid;          // 1 if gyro_z was refreshed this cycle
} SensorData;

// Read one low-latency sensor snapshot:
//   - Left Encoder:  I2C3 (/dev/i2c-3)
//   - Right Encoder: I2C1 (/dev/i2c-1)
//   - IMU (MPU6050): I2C2 (/dev/i2c-2)
// Encoder reads are always attempted each call. IMU may be decimated.
SensorData read_all_sensors(void);

#endif
