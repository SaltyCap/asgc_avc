# I2C Wiring Guide - ASGC AVC Robot

## Hardware Configuration Summary

### I2C Bus Assignments

| Bus | GPIO Pins | Physical Pins | Device | I2C Address |
|-----|-----------|---------------|--------|-------------|
| **I2C1** | GPIO 2 (SDA), GPIO 3 (SCL) | Pins 3, 5 | Left Encoder (AS5600) | 0x40 |
| **I2C2** | GPIO 4 (SDA), GPIO 5 (SCL) | Pins 7, 29 | Right Encoder (AS5600) | 0x1B |
| **I2C3** | GPIO 6 (SDA), GPIO 7 (SCL) | Pins 31, 26 | IMU (MPU6050) | 0x68 |

All buses configured at **400kHz** for fast communication.

## Wiring Connections

### Left Encoder (AS5600) → I2C1
```
AS5600 Left    →    Raspberry Pi 5
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VCC            →    Pin 1 or 17 (3.3V)
GND            →    Pin 6 or 9 (GND)
SDA            →    Pin 3 (GPIO 2) ★
SCL            →    Pin 5 (GPIO 3) ★
```

### Right Encoder (AS5600) → I2C2
```
AS5600 Right   →    Raspberry Pi 5
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VCC            →    Pin 1 or 17 (3.3V)
GND            →    Pin 6 or 9 (GND)
SDA            →    Pin 7 (GPIO 4) ★
SCL            →    Pin 29 (GPIO 5) ★
```

### IMU (MPU6050) → I2C3
```
MPU6050        →    Raspberry Pi 5
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VCC            →    Pin 1 or 17 (3.3V)
GND            →    Pin 6 or 9 (GND)
SDA            →    Pin 31 (GPIO 6) ★
SCL            →    Pin 26 (GPIO 7) ★
```

## GPIO Header Pinout Reference

```
        Raspberry Pi 5 - 40 Pin GPIO Header
        ===================================

     3.3V  (1) (2)  5V    ← Power for sensors
★ GPIO2/SDA1  (3) (4)  5V    ← I2C1 SDA (Left Encoder)
★ GPIO3/SCL1  (5) (6)  GND   ← I2C1 SCL (Left Encoder)
★ GPIO4/SDA2  (7) (8)  GPIO14/UART-TX ← I2C2 SDA (Right Encoder)
      GND  (9) (10) GPIO15/UART-RX
   GPIO17 (11) (12) GPIO18/PWM0
GPIO13/PWM1 (13) (14) GND
   GPIO27 (15) (16) GPIO23
     3.3V (17) (18) GPIO24
   GPIO10 (19) (20) GND
    GPIO9 (21) (22) GPIO25
   GPIO11 (23) (24) GPIO8
      GND (25) (26) GPIO7/SCL3 ← I2C3 SCL (IMU)
    GPIO0 (27) (28) GPIO1
★ GPIO5/SCL2 (29) (30) GND   ← I2C2 SCL (Right Encoder)
★ GPIO6/SDA3 (31) (32) GPIO12/PWM0 ← I2C3 SDA (IMU)
   GPIO13 (33) (34) GND
   GPIO19 (35) (36) GPIO16
   GPIO26 (37) (38) GPIO20
      GND (39) (40) GPIO21

★ = I2C pins in use
```

## Physical Pin Adjacency

### ✓ Adjacent Pins (Easy Wiring)
- **I2C1**: Pins 3 & 5 are **side by side**
- **I2C3**: Pins 31 & 26 are **adjacent** (31 is next to 26 vertically)

### ⚠ Non-Adjacent Pins
- **I2C2**: Pin 7 (top of header) and Pin 29 (middle of header)
  - Use longer jumper wires or ribbon cable

## Code References

### Header Files Updated
- [`include/i2c.h`](file:///home/asgc/asgc_avc/c_code/include/i2c.h) - I2C1/I2C2 bus definitions
- [`include/imu.h`](file:///home/asgc/asgc_avc/c_code/include/imu.h) - I2C3 bus definition
- [`include/sensors.h`](file:///home/asgc/asgc_avc/c_code/include/sensors.h) - Sensor documentation

### Device Tree Configuration
- [`/boot/firmware/config.txt`](file:///boot/firmware/config.txt) - dtoverlay settings

## Verification Steps

### 1. Reboot to Apply Changes
```bash
sudo reboot
```

### 2. Check I2C Buses Exist
```bash
ls -l /dev/i2c-*
# Expected output:
# /dev/i2c-1
# /dev/i2c-2
# /dev/i2c-3
```

### 3. Detect Devices on Each Bus
```bash
# Left encoder on I2C1 (should show device at 0x40)
i2cdetect -y 1

# Right encoder on I2C2 (should show device at 0x1B)
i2cdetect -y 2

# IMU on I2C3 (should show device at 0x68)
i2cdetect -y 3
```

### 4. Rebuild and Test C Code
```bash
cd /home/asgc/asgc_avc/c_code
make clean
make
sudo ./bin/main
```

## Troubleshooting

### Device Not Detected
1. Check physical connections (SDA, SCL, VCC, GND)
2. Verify 3.3V power is present
3. Check I2C address with `i2cdetect -y <bus_number>`
4. Ensure pull-up resistors are present (usually built into sensors)

### Bus Not Available
1. Verify `/boot/firmware/config.txt` has correct dtoverlay lines
2. Reboot after config changes
3. Check `dmesg | grep i2c` for errors

### Wrong Data or Communication Errors
1. Verify correct bus assignment in code
2. Check for loose connections
3. Reduce I2C speed if needed (change baudrate in config.txt)
4. Ensure wires are not too long (keep under 30cm for 400kHz)

## Notes

- **Pull-up resistors**: Most I2C sensors have built-in pull-ups. If you experience issues, you may need external 4.7kΩ pull-ups on SDA/SCL lines.
- **Wire length**: Keep I2C wires short for 400kHz operation. Use twisted pair or shielded cable for longer runs.
- **Power**: All sensors use 3.3V. Do NOT connect to 5V pins.
- **Parallel operation**: The three-bus design allows simultaneous sensor reads for maximum performance.
