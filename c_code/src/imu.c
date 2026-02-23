#include "../include/imu.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>

IMUContext imu = {0.0, PTHREAD_MUTEX_INITIALIZER, -1};

static int write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(imu.i2c_fd, buf, 2) != 2) return -1;
    return 0;
}

static int read_raw_gyro_z_locked(int16_t *raw_z) {
    uint8_t reg = GYRO_ZOUT_H;
    uint8_t buf[2];

    if (write(imu.i2c_fd, &reg, 1) != 1) {
        return -1;
    }
    if (read(imu.i2c_fd, buf, 2) != 2) {
        return -1;
    }

    *raw_z = (int16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

static int imu_read_raw_rate(double *raw_rate_out) {
    if (!raw_rate_out || imu.i2c_fd < 0) {
        return 0;
    }

    int16_t raw_z = 0;
    pthread_mutex_lock(&imu.lock);
    int ok = (read_raw_gyro_z_locked(&raw_z) == 0);
    pthread_mutex_unlock(&imu.lock);
    if (!ok) {
        return 0;
    }

    *raw_rate_out = (double)raw_z / 131.0;
    return 1;
}

int imu_init(void) {
    imu.i2c_fd = open(IMU_I2C_BUS, O_RDWR);
    if (imu.i2c_fd < 0) {
        perror("Failed to open IMU I2C bus");
        return -1;
    }

    if (ioctl(imu.i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(imu.i2c_fd);
        imu.i2c_fd = -1;
        return -1;
    }

    // Wake up MPU-6050 (clear sleep bit)
    if (write_reg(PWR_MGMT_1, 0x00) < 0) {
        perror("IMU register write failed: PWR_MGMT_1");
        close(imu.i2c_fd);
        imu.i2c_fd = -1;
        return -1;
    }
    usleep(100000); // Wait for wake up

    // Set sample rate divider to 9.
    // Gyro output rate is 1kHz when DLPF is enabled, so:
    // Sample Rate = 1000 / (1 + 9) = 100Hz
    if (write_reg(SMPLRT_DIV, 0x09) < 0) {
        perror("IMU register write failed: SMPLRT_DIV");
        close(imu.i2c_fd);
        imu.i2c_fd = -1;
        return -1;
    }

    // Set DLPF (Digital Low Pass Filter) to Bandwidth 44Hz (Config 3)
    // This reduces noise significantly. Delay ~4.9ms.
    if (write_reg(CONFIG, 0x03) < 0) {
        perror("IMU register write failed: CONFIG");
        close(imu.i2c_fd);
        imu.i2c_fd = -1;
        return -1;
    }

    // Set Gyro Range to +/- 250 degrees/sec (FS_SEL=0)
    // LSB Sensitivity = 131 LSB/dps
    if (write_reg(GYRO_CONFIG, 0x00) < 0) {
        perror("IMU register write failed: GYRO_CONFIG");
        close(imu.i2c_fd);
        imu.i2c_fd = -1;
        return -1;
    }

    printf("IMU: MPU6050 Initialized on %s\n", IMU_I2C_BUS);
    return 0;
}

void imu_cleanup(void) {
    if (imu.i2c_fd >= 0) {
        close(imu.i2c_fd);
        imu.i2c_fd = -1;
    }
}

double imu_read_gyro_z(void) {
    double gyro_z = 0.0;
    if (!imu_read_gyro_z_sample(&gyro_z)) {
        return 0.0;
    }
    return gyro_z;
}

int imu_read_gyro_z_sample(double *gyro_z_out) {
    if (!gyro_z_out) {
        return 0;
    }
    double raw_rate = 0.0;
    if (!imu_read_raw_rate(&raw_rate)) {
        return 0;
    }
    *gyro_z_out = -(raw_rate - imu.z_gyro_offset);
    return 1;
}

void imu_calibrate(int samples) {
    if (imu.i2c_fd < 0) return;
    if (samples <= 0) return;

    printf("IMU: Calibrating Gyro (Do not move robot)...\n");
    
    // Allow gyro to settle after power-up
    usleep(500000); // 500ms settling time
    
    double sum = 0.0;
    
    // Discard first 200 readings to ensure gyro is stable.
    for(int i=0; i<200; i++) {
        double ignored = 0.0;
        (void)imu_read_raw_rate(&ignored);
        usleep(5000);
    }

    // Calibrate in raw-rate space, then apply sign convention in imu_read_gyro_z_sample().
    int valid_samples = 0;
    for (int i = 0; i < samples; i++) {
        double raw_rate = 0.0;
        if (imu_read_raw_rate(&raw_rate)) {
            sum += raw_rate;
            valid_samples++;
        }
        usleep(5000); // 200Hz sampling
    }

    if (valid_samples == 0) {
        printf("IMU: Calibration failed (no valid samples)\n");
        return;
    }

    imu.z_gyro_offset = sum / valid_samples;
    printf("IMU: Calibration Complete. Offset: %.4f dps\n", imu.z_gyro_offset);
}
