#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_BUS "/dev/i2c-1"
#define AS5600_ADDRESS 0x36

int i2c_init(void);
void i2c_cleanup(void);
int16_t read_raw_angle(void);
int configure_fast_filter(void);

#endif
