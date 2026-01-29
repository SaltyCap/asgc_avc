#ifndef I2C_H
#define I2C_H

#include <stdint.h>

// I2C Bus Configuration - Updated based on hardware detection
#define I2C1_BUS "/dev/i2c-3"      // Left Encoder (0x40)
#define I2C2_BUS "/dev/i2c-1"      // Right Encoder (0x1B)
#define AS5600_LEFT_ADDRESS 0x40   // Left encoder
#define AS5600_RIGHT_ADDRESS 0x1B  // Right encoder

int i2c_init(void);
void i2c_cleanup(void);
int16_t read_raw_angle(int motor_id);

#endif
