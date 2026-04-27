#include "../include/i2c.h"
#include "../include/common.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// Separate file descriptors for each I2C bus
int i2c1_fd = -1;  // Left encoder
int i2c2_fd = -1;  // Right encoder

int i2c_init(void) {
    // Open Bus for Left Encoder (Now on I2C-3)
    // We reuse the i2c1_fd/i2c2_fd variables but map them correctly
    // i2c1_fd -> Left Encoder (Bus 3)
    i2c1_fd = open(I2C1_BUS, O_RDWR);
    if (i2c1_fd < 0) {
        perror("Failed to open Left Encoder bus");
        return -1;
    }

    // Each encoder has a dedicated bus/device fd, so bind slave address once.
    if (ioctl(i2c1_fd, I2C_SLAVE, AS5600_LEFT_ADDRESS) < 0) {
        perror("Failed to select Left Encoder I2C address");
        close(i2c1_fd);
        i2c1_fd = -1;
        return -1;
    }
    
    // Open Bus for Right Encoder (Now on I2C-1)
    // i2c2_fd -> Right Encoder (Bus 1)
    i2c2_fd = open(I2C2_BUS, O_RDWR);
    if (i2c2_fd < 0) {
        perror("Failed to open Right Encoder bus");
        close(i2c1_fd);
        i2c1_fd = -1;
        return -1;
    }

    if (ioctl(i2c2_fd, I2C_SLAVE, AS5600_RIGHT_ADDRESS) < 0) {
        perror("Failed to select Right Encoder I2C address");
        close(i2c2_fd);
        i2c2_fd = -1;
        close(i2c1_fd);
        i2c1_fd = -1;
        return -1;
    }
    
    printf("I2C: Opened Left(Bus 3) and Right(Bus 1)\n");
    return 0;
}

int16_t read_raw_angle(int motor_id) {
    if (motor_id != 0 && motor_id != 1) {
        return -1;
    }

    // Select bus based on motor ID (slave address already configured in init)
    int fd = (motor_id == 0) ? i2c1_fd : i2c2_fd;

    if (fd < 0) return -1;

    // Read raw angle register (0x0C-0x0D) in a single transaction
    uint8_t reg = 0x0C; // REG_RAW_ANGLE_H
    uint8_t buf[2];

    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;

    return ((buf[0] & 0x0F) << 8) | buf[1];
}

void i2c_cleanup(void) {
    if (i2c1_fd >= 0) {
        close(i2c1_fd);
        i2c1_fd = -1;
    }
    if (i2c2_fd >= 0) {
        close(i2c2_fd);
        i2c2_fd = -1;
    }
}
