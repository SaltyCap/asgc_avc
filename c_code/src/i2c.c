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
    i2c1_fd = open("/dev/i2c-3", O_RDWR);
    if (i2c1_fd < 0) {
        perror("Failed to open Left Encoder bus (I2C-3)");
        return -1;
    }
    
    // Open Bus for Right Encoder (Now on I2C-1)
    // i2c2_fd -> Right Encoder (Bus 1)
    i2c2_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c2_fd < 0) {
        perror("Failed to open Right Encoder bus (I2C-1)");
        close(i2c1_fd);
        return -1;
    }
    
    printf("I2C: Opened Left(Bus 3) and Right(Bus 1)\n");
    return 0;
}

int16_t read_raw_angle(int motor_id) {
    // Select bus and address based on motor ID
    int fd = (motor_id == 0) ? i2c1_fd : i2c2_fd;
    int address = (motor_id == 0) ? AS5600_LEFT_ADDRESS : AS5600_RIGHT_ADDRESS;
    
    if (fd < 0) return -1;

    // Set the I2C slave address
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        return -1;
    }

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
