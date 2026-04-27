#!/usr/bin/env python3
"""
test_scoop_direction.py
-----------------------
Nudges the scoop between the OLD UP (500 µs) and NEW UP (544 µs)
so you can confirm which direction 544 µs moves the scoop.

Sequence:
  1. Move to OLD UP (500 µs) — baseline
  2. Wait 2 s
  3. Move to NEW UP (544 µs) — watch direction
  4. Wait 2 s
  5. Return to OLD UP (500 µs)

Motor process must be stopped before running this.
"""

import sys
import time
import serial

UART_DEVICE = "/dev/ttyAMA0"
BAUD = 9600
PULSE_CENTER = 1500  # µs


def send_position(ser, us):
    qus = us * 4
    low  = qus & 0x7F
    high = (qus >> 7) & 0x7F
    mir_us = max(500, min(2500, 2 * PULSE_CENTER - us))
    mir_qus = mir_us * 4
    mir_low  = mir_qus & 0x7F
    mir_high = (mir_qus >> 7) & 0x7F
    pkt0 = bytes([0x84, 0, low  & 0x7F, high  & 0x7F])
    pkt1 = bytes([0x84, 1, mir_low & 0x7F, mir_high & 0x7F])
    ser.write(pkt0)
    ser.write(pkt1)
    ser.flush()
    print(f"  → CH0: {us} µs  (low=0x{low:02X}, high=0x{high:02X})   CH1 mirror: {mir_us} µs")


def main():
    print(f"Opening {UART_DEVICE}…")
    try:
        ser = serial.Serial(UART_DEVICE, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        print("Stop the motor/web-server process first.")
        sys.exit(1)

    print("\n[1] Moving to OLD UP (500 µs — current baseline)…")
    send_position(ser, 500)
    time.sleep(2.0)

    print("\n[2] Moving to NEW UP (544 µs — +4°)…")
    print("     Watch: does the scoop go HIGHER or lower?")
    send_position(ser, 544)
    time.sleep(3.0)

    print("\n[3] Back to OLD UP (500 µs)…")
    send_position(ser, 500)
    time.sleep(1.0)

    ser.close()
    print("\nDone. Tell me: did 544 µs move the scoop higher or lower than 500 µs?")


if __name__ == "__main__":
    main()
