#!/usr/bin/env python3
"""
test_conveyor_limit_switch.py
------------------------------
Standalone test for the conveyor limit-switch timing fix.

Sequence:
  1. Start conveyor motor (GPIO 11, active-low).
  2. Poll conveyor switch (GPIO 25) with a 30-second TIMEOUT.
     — Prints raw GPIO value every second so you can see polarity live.
     — If timeout expires the motor is stopped anyway (same behaviour as
       ball_cycle_manager.py after the 2026-04-22 timeout fix).
  3. When switch triggers (or timeout) → prints timestamp.
  4. Conveyor keeps running for 1.5 s.
  5. Stops conveyor → prints elapsed time and PASS/FAIL verdict.
  6. Cleans up and exits.

Run with:
    sudo python3 test_conveyor_limit_switch.py

Switch polarity flags (edit to match your wiring):
    SWITCH_INVERT = False  → Normally-Open  (NO): LOW  when triggered
    SWITCH_INVERT = True   → Normally-Closed (NC): HIGH when triggered
"""

import sys
import time

# ── Pin assignments (must match gpio_interface.py) ───────────────────────────
PIN_CONVEYOR_SWITCH = 25   # INPUT  pull-up
PIN_CONVEYOR_MOTOR  = 11   # OUTPUT inverted: LOW = ON, HIGH = OFF

# ── Timing ───────────────────────────────────────────────────────────────────
CONVEYOR_STOP_DELAY_S  = 1.5   # keep running this long after switch triggers
SWITCH_WAIT_TIMEOUT_S  = 30.0  # max time to wait for switch before giving up
POLL_INTERVAL          = 0.05  # 20 Hz switch polling
LOG_INTERVAL           = 1.0   # print raw switch state this often while waiting

# ── Switch polarity (edit to match gpio_interface.py _CONVEYOR_SWITCH_INVERT) ─
SWITCH_INVERT = False   # False = NO (active-low),  True = NC (active-high)

# ── lgpio setup ──────────────────────────────────────────────────────────────
try:
    import lgpio
except ImportError:
    print("ERROR: lgpio not installed.  Run:  pip install lgpio")
    sys.exit(1)


def read_switch(handle, pin, invert=False) -> bool:
    """Return True when the switch is TRIGGERED (accounts for polarity)."""
    raw = lgpio.gpio_read(handle, pin)
    return (raw == 1) if invert else (raw == 0)


def main():
    polarity_str = "NC (active-high)" if SWITCH_INVERT else "NO (active-low)"
    print("=" * 60)
    print("  Conveyor limit-switch timing test")
    print(f"  Conveyor motor  : GPIO {PIN_CONVEYOR_MOTOR}  (active-low output)")
    print(f"  Limit switch    : GPIO {PIN_CONVEYOR_SWITCH} — {polarity_str}")
    print(f"  Switch timeout  : {SWITCH_WAIT_TIMEOUT_S:.0f} s")
    print(f"  Stop delay      : {CONVEYOR_STOP_DELAY_S:.1f} s after switch/timeout")
    print("=" * 60)

    handle = lgpio.gpiochip_open(4)   # Pi 5: RP1 chip = gpiochip4

    try:
        # Configure pins
        lgpio.gpio_claim_output(handle, PIN_CONVEYOR_MOTOR,  1)   # start OFF (HIGH)
        lgpio.gpio_claim_input( handle, PIN_CONVEYOR_SWITCH, lgpio.SET_PULL_UP)

        # ── Step 1: start conveyor ───────────────────────────────────────────
        print("\n[1] Starting conveyor motor…")
        lgpio.gpio_write(handle, PIN_CONVEYOR_MOTOR, 0)   # LOW = ON
        t_start = time.monotonic()
        print("    Motor ON ✓  (press the conveyor limit switch now)")

        # ── Step 2: poll for switch trigger with timeout ─────────────────────
        print(f"\n[2] Waiting up to {SWITCH_WAIT_TIMEOUT_S:.0f}s for limit switch…")
        print("    (raw GPIO value printed every second — 0=LOW, 1=HIGH)\n")

        deadline      = t_start + SWITCH_WAIT_TIMEOUT_S
        next_log_time = time.monotonic() + LOG_INTERVAL
        timed_out     = False
        triggered     = False

        while True:
            now = time.monotonic()

            # Enforce timeout
            if now >= deadline:
                timed_out = True
                print(f"\n    ⚠️  TIMEOUT after {SWITCH_WAIT_TIMEOUT_S:.0f}s — "
                      "switch never triggered (stopping conveyor anyway).")
                break

            raw = lgpio.gpio_read(handle, PIN_CONVEYOR_SWITCH)
            triggered = read_switch(handle, PIN_CONVEYOR_SWITCH, SWITCH_INVERT)

            # Periodic status log
            if now >= next_log_time:
                remaining = deadline - now
                status = "TRIGGERED ✅" if triggered else "idle"
                print(f"    GPIO{PIN_CONVEYOR_SWITCH} raw={raw}  "
                      f"triggered={triggered}  status={status}  "
                      f"remaining={remaining:.1f}s")
                next_log_time = now + LOG_INTERVAL

            if triggered:
                break

            time.sleep(POLL_INTERVAL)

        t_trigger = time.monotonic()
        elapsed_to_trigger = t_trigger - t_start

        if not timed_out:
            print(f"\n    ✅ Limit switch triggered after {elapsed_to_trigger:.2f}s")
        else:
            raw_now = lgpio.gpio_read(handle, PIN_CONVEYOR_SWITCH)
            print(f"    (Last raw GPIO value: {raw_now} — "
                  "if this never changed, check SWITCH_INVERT polarity setting)")

        # ── Step 3: keep running for 1.5 s ──────────────────────────────────
        print(f"\n[3] Conveyor keeps running for {CONVEYOR_STOP_DELAY_S:.1f}s…")
        stop_deadline = t_trigger + CONVEYOR_STOP_DELAY_S
        while time.monotonic() < stop_deadline:
            time.sleep(0.01)

        # ── Step 4: stop conveyor ────────────────────────────────────────────
        lgpio.gpio_write(handle, PIN_CONVEYOR_MOTOR, 1)   # HIGH = OFF
        t_stop = time.monotonic()
        actual_delay = t_stop - t_trigger

        print(f"    ✅ Conveyor stopped!")
        print(f"       Delay after switch/timeout = {actual_delay:.3f}s  "
              f"(target {CONVEYOR_STOP_DELAY_S:.1f}s)")

        if timed_out:
            print("\n⚠️  INCONCLUSIVE — switch timed out. Check wiring and SWITCH_INVERT flag.")
        elif abs(actual_delay - CONVEYOR_STOP_DELAY_S) < 0.1:
            print("\n✅ PASS — stop delay within 100 ms of target")
        else:
            print(f"\n⚠️  WARN — stop delay off by "
                  f"{abs(actual_delay - CONVEYOR_STOP_DELAY_S)*1000:.0f} ms")

        print("\n── Polarity hint ─────────────────────────────────────────")
        print("  If the switch NEVER triggered, try flipping:")
        print("    SWITCH_INVERT = True   (if your switch is Normally-Closed)")
        print("  And update gpio_interface.py:  _CONVEYOR_SWITCH_INVERT = True")
        print("──────────────────────────────────────────────────────────")

    except KeyboardInterrupt:
        print("\nInterrupted — stopping motor and cleaning up.")
    finally:
        # Always leave motor OFF
        try:
            lgpio.gpio_write(handle, PIN_CONVEYOR_MOTOR, 1)
        except Exception:
            pass
        lgpio.gpiochip_close(handle)
        print("GPIO released. Done.")


if __name__ == "__main__":
    main()
