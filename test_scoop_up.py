#!/usr/bin/env python3
"""
test_scoop_up.py
-----------------
Sends a single scoop UP position command directly over the Maestro UART.
Run this ONLY while start_all.sh / motor_control is stopped, to avoid port conflicts.

Usage:
    sudo python3 test_scoop_up.py [qus]

    qus  = quarter-microsecond target (default: 2178 = current UP + 8 degrees)
           2000 = original UP (500 us)
           2178 = +8 degrees from original
           1822 = -8 degrees from original

Examples:
    sudo python3 test_scoop_up.py          # test +8 deg (2178 qus)
    sudo python3 test_scoop_up.py 2000     # original UP position
    sudo python3 test_scoop_up.py 2200     # a bit more than +8 deg
"""

import sys
import time

UART_DEVICE  = "/dev/ttyAMA0"
UART_BAUD    = 9600
DEVICE_NUM   = 0x0C   # Maestro device number (default)

# Quarter-microsecond targets
QUS_ORIGINAL_UP = 2000   # 500 us  — current "up" in code
QUS_DOWN        = 6000   # 1500 us — full down

def qus_to_bytes(qus):
    """Convert a quarter-us value to Maestro low/high bytes."""
    low  = qus & 0x7F
    high = (qus >> 7) & 0x7F
    return low, high

def send_target(ser, channel, qus):
    low, high = qus_to_bytes(qus)
    # Maestro compact protocol: 0xAA  device  0x04  channel  low  high
    packet = bytes([0xAA, DEVICE_NUM, 0x04, channel, low, high])
    ser.write(packet)
    us = qus / 4
    print(f"  ch{channel}: {qus} qus = {us:.1f} µs  (low=0x{low:02X} high=0x{high:02X})")

def main():
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed.  Run: pip install pyserial")
        sys.exit(1)

    target_qus = int(sys.argv[1]) if len(sys.argv) > 1 else 2178

    low, high = qus_to_bytes(target_qus)
    us = target_qus / 4
    deg_delta = (target_qus - QUS_ORIGINAL_UP) / 22.2

    print("=" * 50)
    print(f"  Scoop UP position test")
    print(f"  Target : {target_qus} qus = {us:.1f} µs")
    print(f"  Bytes  : low=0x{low:02X}  high=0x{high:02X}")
    print(f"  Delta  : {deg_delta:+.1f}° from original UP")
    print("=" * 50)

    try:
        ser = serial.Serial(UART_DEVICE, UART_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"\nERROR opening {UART_DEVICE}: {e}")
        print("Is the motor_control process still running? Stop it first.")
        sys.exit(1)

    print("\nSending to both servo channels...")
    send_target(ser, 0, target_qus)
    time.sleep(0.1)
    send_target(ser, 1, target_qus)

    time.sleep(1.0)   # give servo time to move before we exit
    ser.close()
    print("\nDone. Did the scoop move to the right position?")

if __name__ == "__main__":
    main()
