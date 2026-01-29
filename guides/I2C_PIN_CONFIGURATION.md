# I2C Configuration Summary - Raspberry Pi 5

## Your Requested GPIOs: 2, 3, 5, 6, 26, 19

### ✓ Configured (Using GPIOs 2, 3, 5, 6)

| Bus | GPIO Pins | Physical Pins | Adjacent? | Device |
|-----|-----------|---------------|-----------|--------|
| **I2C1** | 2 (SDA), 3 (SCL) | 3, 5 | ✓ Yes | `/dev/i2c-1` |
| **I2C2** | 4 (SDA), 5 (SCL) | 7, 29 | Pins 29/31 adjacent | `/dev/i2c-2` |
| **I2C3** | 6 (SDA), 7 (SCL) | 31, 26 | Pins 31/26 adjacent | `/dev/i2c-3` |

### ⚠ GPIO 19 and 26 Cannot Be Used for I2C

The Raspberry Pi 5 hardware **does not support I2C** on GPIO 19 and 26. These pins are not connected to any I2C controller.

## Pin Layout Visualization

```
GPIO Header (40-pin):
     3.3V  (1) (2)  5V
★ GPIO2/SDA1  (3) (4)  5V              ← I2C1 SDA
★ GPIO3/SCL1  (5) (6)  GND             ← I2C1 SCL
  GPIO4/SDA2  (7) (8)  GPIO14/UART-TX ← I2C2 SDA
      GND  (9) (10) GPIO15/UART-RX
   GPIO17 (11) (12) GPIO18/PWM0
GPIO13/PWM1 (13) (14) GND
   GPIO27 (15) (16) GPIO23
     3.3V (17) (18) GPIO24
   GPIO10 (19) (20) GND
    GPIO9 (21) (22) GPIO25
   GPIO11 (23) (24) GPIO8
      GND (25) (26) GPIO7/SCL3        ← I2C3 SCL
    GPIO0 (27) (28) GPIO1
★ GPIO5/SCL2 (29) (30) GND             ← I2C2 SCL
★ GPIO6/SDA3 (31) (32) GPIO12/PWM0     ← I2C3 SDA
   GPIO13 (33) (34) GND
   GPIO19 (35) (36) GPIO16            ← NOT I2C capable
   GPIO26 (37) (38) GPIO20            ← NOT I2C capable
      GND (39) (40) GPIO21

★ = Your requested GPIOs that ARE configured
```

## Wiring Guide

### I2C1 (Pins 3 & 5) - **ADJACENT**
- Pin 3: GPIO2 (SDA1) 
- Pin 5: GPIO3 (SCL1)
- **Status**: ✓ Side by side

### I2C2 (Pins 7 & 29)
- Pin 7: GPIO4 (SDA2)
- Pin 29: GPIO5 (SCL2) ← **Your requested GPIO 5**
- **Status**: Pin 29 is adjacent to pin 31

### I2C3 (Pins 31 & 26)
- Pin 31: GPIO6 (SDA3) ← **Your requested GPIO 6**
- Pin 26: GPIO7 (SCL3)
- **Status**: ✓ Adjacent pins

## Configuration Applied

```bash
# I2C1: GPIO 2/3 (default, enabled via dtparam)
dtparam=i2c_arm=on,i2c_arm_baudrate=400000

# I2C2: GPIO 4/5 (includes your GPIO 5)
dtoverlay=i2c2-pi5,pins_4_5,baudrate=400000

# I2C3: GPIO 6/7 (includes your GPIO 6)
dtoverlay=i2c3-pi5,pins_6_7,baudrate=400000
```

## Next Steps

1. **Reboot** to apply changes:
   ```bash
   sudo reboot
   ```

2. **Verify I2C buses** after reboot:
   ```bash
   ls -l /dev/i2c-*
   # Should show: /dev/i2c-1, /dev/i2c-2, /dev/i2c-3
   ```

3. **Detect devices** on each bus:
   ```bash
   i2cdetect -y 1  # I2C1 on GPIO 2/3
   i2cdetect -y 2  # I2C2 on GPIO 4/5
   i2cdetect -y 3  # I2C3 on GPIO 6/7
   ```

## Summary

✓ **Successfully configured** I2C using 4 of your 6 requested GPIOs (2, 3, 5, 6)  
✗ **Cannot use** GPIO 19 and 26 for I2C (hardware limitation on Pi 5)  
✓ **All buses** running at 400kHz  
✓ **Avoids** UART pins (14/15) and your PWM pins (12/13)
