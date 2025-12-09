#include "../include/i2c.h"
#include "../include/common.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int i2c_fd = -1;

int i2c_init(void) {
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) return -1;
    return 0;
}

int16_t read_raw_angle(int motor_id) {
    // Select the correct sensor address based on motor ID
    // motor_id 0 = left (0x40), motor_id 1 = right (0x1B)
    const int address = (motor_id == 0) ? AS5600_LEFT_ADDRESS : AS5600_RIGHT_ADDRESS;

    // Set the I2C slave address for this read
    // This ioctl is fast (~1-2 microseconds) but we minimize other overhead
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        return -1;
    }

    // Read raw angle register (0x0C-0x0D) in a single transaction
    uint8_t reg = 0x0C; // REG_RAW_ANGLE_H
    uint8_t buf[2];

    if (write(i2c_fd, &reg, 1) != 1) return -1;
    if (read(i2c_fd, buf, 2) != 2) return -1;

    return ((buf[0] & 0x0F) << 8) | buf[1];
}

int configure_fast_filter(void) {
    // Implementation optional if we want to keep it simple, 
    // but good for performance.
    return 0;
}

void i2c_cleanup(void) {
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}
