#ifndef IMU_H
#define IMU_H

#include <pthread.h>

// I2C Configuration for IMU
// IMU is on /dev/i2c-2 (Bus 2)
#define IMU_I2C_BUS "/dev/i2c-2"
#define MPU6050_ADDR 0x68

// Register Map
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

typedef struct {
    double z_gyro_offset;
    pthread_mutex_t lock;
    int i2c_fd;
} IMUContext;

extern IMUContext imu;

// Functions
int imu_init(void);
void imu_cleanup(void);
double imu_read_gyro_z(void);
void imu_calibrate(int samples);

#endif
