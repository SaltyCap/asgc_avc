# I2C Configuration Update Summary

## ✓ Completed Changes

### 1. Device Tree Configuration (`/boot/firmware/config.txt`)

Updated I2C bus configuration with new GPIO pin assignments:

```bash
# I2C1: GPIO 2/3 (Pins 3/5) - Adjacent ✓
dtparam=i2c_arm=on,i2c_arm_baudrate=400000

# I2C2: GPIO 4/5 (Pins 7/29)
dtoverlay=i2c2-pi5,pins_4_5,baudrate=400000

# I2C3: GPIO 6/7 (Pins 31/26) - Adjacent ✓
dtoverlay=i2c3-pi5,pins_6_7,baudrate=400000
```

### 2. C Code Header Files Updated

#### [`include/i2c.h`](file:///home/asgc/asgc_avc/c_code/include/i2c.h)
- Added GPIO pin documentation for I2C1 (GPIO 2/3) and I2C2 (GPIO 4/5)
- Left encoder on I2C1, Right encoder on I2C2

#### [`include/imu.h`](file:///home/asgc/asgc_avc/c_code/include/imu.h)
- Added GPIO pin documentation for I2C3 (GPIO 6/7)
- MPU6050 IMU on I2C3

#### [`include/sensors.h`](file:///home/asgc/asgc_avc/c_code/include/sensors.h)
- Updated sensor documentation with complete GPIO pin assignments
- All three buses documented with physical pin numbers

### 3. Compilation Verified
✓ Code compiles successfully with no errors or warnings

## Pin Assignment Summary

| Device | Bus | GPIO Pins | Physical Pins | Adjacent? |
|--------|-----|-----------|---------------|-----------|
| Left Encoder | I2C1 | 2/3 | 3/5 | ✓ Yes |
| Right Encoder | I2C2 | 4/5 | 7/29 | Partial |
| IMU (MPU6050) | I2C3 | 6/7 | 31/26 | ✓ Yes |

## Avoided Pins (As Requested)
- ✓ UART pins (GPIO 14/15) - Not used
- ✓ PWM pins (GPIO 12/13) - Not used

## Next Steps

### 1. Reboot to Apply Device Tree Changes
```bash
sudo reboot
```

### 2. After Reboot - Verify I2C Buses
```bash
# Check buses exist
ls -l /dev/i2c-*

# Detect devices
i2cdetect -y 1  # Should show 0x40 (left encoder)
i2cdetect -y 2  # Should show 0x1B (right encoder)
i2cdetect -y 3  # Should show 0x68 (IMU)
```

### 3. Test the System
```bash
cd /home/asgc/asgc_avc/c_code
sudo ./asgc_motor_control
```

## Documentation Created

1. **[`I2C_PIN_CONFIGURATION.md`](file:///home/asgc/asgc_avc/I2C_PIN_CONFIGURATION.md)** - Detailed pin configuration reference
2. **[`I2C_WIRING_GUIDE.md`](file:///home/asgc/asgc_avc/I2C_WIRING_GUIDE.md)** - Complete wiring guide with troubleshooting

## Important Notes

- All buses configured at **400kHz** for fast communication
- Two buses (I2C1 and I2C3) have **adjacent physical pins** for easy wiring
- I2C2 uses pins 7 and 29 which are not adjacent - use longer jumper wires
- No code logic changes were needed - only documentation updates
- GPIO 19 and 26 (from your original request) cannot be used for I2C on Pi 5

## Configuration Files Changed

1. `/boot/firmware/config.txt` - Device tree overlays
2. `/home/asgc/asgc_avc/c_code/include/i2c.h` - I2C1/I2C2 documentation
3. `/home/asgc/asgc_avc/c_code/include/imu.h` - I2C3 documentation
4. `/home/asgc/asgc_avc/c_code/include/sensors.h` - Complete sensor documentation
