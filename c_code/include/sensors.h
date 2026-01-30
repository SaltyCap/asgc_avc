#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// Combined sensor data structure
typedef struct {
    int16_t left_encoder;   // Left motor encoder raw angle
    int16_t right_encoder;  // Right motor encoder raw angle
    double gyro_z;          // IMU Z-axis gyro rate (degrees/sec)
    double timestamp;       // Timestamp when sensors were read (seconds)
    int valid;              // 1 if all reads successful, 0 otherwise
} SensorData;

// Read all sensors simultaneously using three separate I2C buses:
//   - Left Encoder:  I2C3 (/dev/i2c-3)
//   - Right Encoder: I2C1 (/dev/i2c-1)
//   - IMU (MPU6050): I2C2 (/dev/i2c-2)
// All three buses accessed in parallel for maximum performance
// Timestamp is captured at the start of the read for precise synchronization
SensorData read_all_sensors(void);

#endif
