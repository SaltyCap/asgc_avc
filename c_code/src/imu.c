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

int imu_init(void) {
    imu.i2c_fd = open(IMU_I2C_BUS, O_RDWR);
    if (imu.i2c_fd < 0) {
        perror("Failed to open I2C3 bus for IMU");
        return -1;
    }

    if (ioctl(imu.i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(imu.i2c_fd);
        return -1;
    }

    // Wake up MPU-6050 (clear sleep bit)
    if (write_reg(PWR_MGMT_1, 0x00) < 0) return -1;
    usleep(100000); // Wait for wake up

    // Set sample rate divider to 0 (Sample Rate = Gyro Rate / (1 + 0))
    // Gyro Rate is 8kHz if DLPF disabled, 1kHz if enabled.
    // We will enable DLPF, so base is 1kHz. Div 0 = 1kHz sample rate.
    write_reg(SMPLRT_DIV, 0x07); 

    // Set DLPF (Digital Low Pass Filter) to Bandwidth 44Hz (Config 3)
    // This reduces noise significantly. Delay ~4.9ms.
    write_reg(CONFIG, 0x03);

    // Set Gyro Range to +/- 250 degrees/sec (FS_SEL=0)
    // LSB Sensitivity = 131 LSB/dps
    write_reg(GYRO_CONFIG, 0x00);

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
    if (imu.i2c_fd < 0) return 0.0;

    uint8_t reg = GYRO_ZOUT_H;
    uint8_t buf[2];

    pthread_mutex_lock(&imu.lock);
    
    if (write(imu.i2c_fd, &reg, 1) != 1) {
        pthread_mutex_unlock(&imu.lock);
        return 0.0;
    }
    
    if (read(imu.i2c_fd, buf, 2) != 2) {
        pthread_mutex_unlock(&imu.lock);
        return 0.0;
    }
    
    pthread_mutex_unlock(&imu.lock);

    // Join high and low bytes
    int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);

    // Convert to degrees per second
    // Sensitivity for +/- 250dps is 131 LSB/dps
    double gyro_z = (double)raw_z / 131.0;

    return -(gyro_z - imu.z_gyro_offset);
}

void imu_calibrate(int samples) {
    if (imu.i2c_fd < 0) return;

    printf("IMU: Calibrating Gyro (Do not move robot)...\n");
    
    // Allow gyro to settle after power-up
    usleep(500000); // 500ms settling time
    
    double sum = 0.0;
    
    // Discard first 200 readings to ensure gyro is stable
    for(int i=0; i<200; i++) {
        imu_read_gyro_z(); 
        usleep(5000);
    }

    // Temporary zero offset for calibration
    imu.z_gyro_offset = 0.0; 

    for (int i = 0; i < samples; i++) {
        sum += imu_read_gyro_z();
        usleep(5000); // 200Hz sampling
    }

    imu.z_gyro_offset = sum / samples;
    printf("IMU: Calibration Complete. Offset: %.4f dps\n", imu.z_gyro_offset);
}
