#!/usr/bin/env python3
"""
Hardware-in-loop log validator for ML navigation controller checks.

This script reads the latest motor CSV log (or a user-specified file),
verifies schema/telemetry health, and runs ML-focused sanity checks.
"""

import argparse
import csv
import glob
import math
import os
import sys
from collections import Counter


NEUTRAL_NS = 1_500_000

REQUIRED_COLUMNS = [
    "time",
    "mode",
    "pwm_l",
    "i2c_l",
    "pwm_r",
    "i2c_r",
    "target_l",
    "actual_l",
    "target_r",
    "actual_r",
    "vel_l",
    "accel_l",
    "vel_r",
    "accel_r",
    "gyro_z",
    "odom_x",
    "odom_y",
    "odom_heading",
    "nav_state",
]

NUMERIC_COLUMNS = [
    "time",
    "pwm_l",
    "i2c_l",
    "pwm_r",
    "i2c_r",
    "target_l",
    "actual_l",
    "target_r",
    "actual_r",
    "vel_l",
    "accel_l",
    "vel_r",
    "accel_r",
    "gyro_z",
    "odom_x",
    "odom_y",
    "odom_heading",
]

TURN_STATES = {"TURNING"}
DRIVE_STATES = {"DRIVING"}
MOTION_STATES = TURN_STATES | DRIVE_STATES


def _sign(value: float) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def _latest_log(log_dir: str, mode_filter: str) -> str:
    pattern = os.path.join(log_dir, "motor_log_*.csv")
    files = glob.glob(pattern)
    if mode_filter != "any":
        token = f"_{mode_filter.lower()}_"
        files = [path for path in files if token in os.path.basename(path).lower()]
    if not files:
        raise FileNotFoundError(f"No log files found in {log_dir!r} (mode={mode_filter})")
    files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return files[0]


def _load_rows(path: str):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV is missing header row")

        missing = [col for col in REQUIRED_COLUMNS if col not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV missing required columns: {', '.join(missing)}")

        rows = []
        parse_errors = 0
        for row in reader:
            try:
                for key in NUMERIC_COLUMNS:
                    row[key] = float(row[key])
                row["mode"] = str(row["mode"]).strip().upper()
                row["nav_state"] = str(row["nav_state"]).strip().upper()
                row["nav_controller_mode"] = str(row.get("nav_controller_mode", "")).strip().upper()
                rows.append(row)
            except Exception:
                parse_errors += 1

    if not rows:
        raise ValueError("No valid data rows found after parsing")
    return rows, parse_errors


def _emit(status: str, message: str):
    print(f"[{status}] {message}")


def _check(condition: bool, ok_message: str, fail_message: str):
    if condition:
        _emit("PASS", ok_message)
        return True
    _emit("FAIL", fail_message)
    return False


def _fraction(rows, predicate):
    if not rows:
        return 0.0
    passed = sum(1 for row in rows if predicate(row))
    return passed / float(len(rows))


def validate(args) -> int:
    path = args.log_file or _latest_log(args.log_dir, args.mode)
    rows, parse_errors = _load_rows(path)

    _emit("INFO", f"Log file: {path}")
    _emit("INFO", f"Rows parsed: {len(rows)}")
    _emit("INFO", f"Rows dropped (parse errors): {parse_errors}")

    mode_counts = Counter(row["mode"] for row in rows)
    state_counts = Counter(row["nav_state"] for row in rows)
    controller_present = "nav_controller_mode" in rows[0]
    controller_counts = Counter(
        row["nav_controller_mode"] for row in rows if row["nav_controller_mode"]
    )

    _emit("INFO", f"Control mode counts: {dict(mode_counts)}")
    _emit("INFO", f"Nav state counts: {dict(state_counts)}")
    if controller_counts:
        _emit("INFO", f"Nav controller mode counts: {dict(controller_counts)}")
    else:
        _emit("INFO", "Nav controller mode counts: unavailable (older log schema)")

    checks_ok = True

    checks_ok &= _check(
        len(rows) >= args.min_rows,
        f"Row count >= {args.min_rows}",
        f"Row count too small ({len(rows)} < {args.min_rows})",
    )

    if args.expect_ml:
        checks_ok &= _check(
            "nav_controller_mode" in rows[0] and bool(controller_counts),
            "nav_controller_mode column present with values",
            "nav_controller_mode not present or empty; rebuild and collect a new log",
        )
        ml_rows = [row for row in rows if row["nav_controller_mode"] == "ML"]
        checks_ok &= _check(
            len(ml_rows) >= args.min_ml_rows,
            f"ML controller rows >= {args.min_ml_rows}",
            f"Not enough ML controller rows ({len(ml_rows)} < {args.min_ml_rows})",
        )
        analysis_rows = [row for row in ml_rows if row["nav_state"] in MOTION_STATES]
    else:
        analysis_rows = [row for row in rows if row["nav_state"] in MOTION_STATES]

    checks_ok &= _check(
        len(analysis_rows) > 0,
        "Found motion rows in TURNING/DRIVING",
        "No TURNING/DRIVING rows found",
    )

    if analysis_rows:
        pwm_active_ratio = _fraction(
            analysis_rows,
            lambda row: (
                abs(row["pwm_l"] - NEUTRAL_NS) > args.pwm_delta_ns
                or abs(row["pwm_r"] - NEUTRAL_NS) > args.pwm_delta_ns
            ),
        )
        checks_ok &= _check(
            pwm_active_ratio >= args.min_pwm_active_ratio,
            f"PWM active ratio OK ({pwm_active_ratio:.2%})",
            (
                "PWM active ratio too low "
                f"({pwm_active_ratio:.2%} < {args.min_pwm_active_ratio:.2%})"
            ),
        )

        vel_active_ratio = _fraction(
            analysis_rows,
            lambda row: abs(row["vel_l"]) > args.vel_eps or abs(row["vel_r"]) > args.vel_eps,
        )
        accel_active_ratio = _fraction(
            analysis_rows,
            lambda row: abs(row["accel_l"]) > args.accel_eps or abs(row["accel_r"]) > args.accel_eps,
        )
        checks_ok &= _check(
            vel_active_ratio >= args.min_dynamics_ratio,
            f"Velocity activity ratio OK ({vel_active_ratio:.2%})",
            (
                "Velocity activity ratio too low "
                f"({vel_active_ratio:.2%} < {args.min_dynamics_ratio:.2%})"
            ),
        )
        checks_ok &= _check(
            accel_active_ratio >= args.min_dynamics_ratio,
            f"Acceleration activity ratio OK ({accel_active_ratio:.2%})",
            (
                "Acceleration activity ratio too low "
                f"({accel_active_ratio:.2%} < {args.min_dynamics_ratio:.2%})"
            ),
        )

    turning_rows = [row for row in analysis_rows if row["nav_state"] in TURN_STATES]
    if turning_rows:
        opposite_sign_ratio = _fraction(
            turning_rows,
            lambda row: _sign(row["target_l"]) != _sign(row["target_r"]),
        )
        checks_ok &= _check(
            opposite_sign_ratio >= args.min_turn_sign_ratio,
            f"Turning target sign pattern OK ({opposite_sign_ratio:.2%})",
            (
                "Turning target sign pattern weak "
                f"({opposite_sign_ratio:.2%} < {args.min_turn_sign_ratio:.2%})"
            ),
        )

    driving_rows = [row for row in analysis_rows if row["nav_state"] in DRIVE_STATES]
    if driving_rows:
        same_sign_ratio = _fraction(
            driving_rows,
            lambda row: _sign(row["target_l"]) == _sign(row["target_r"]),
        )
        checks_ok &= _check(
            same_sign_ratio >= args.min_drive_sign_ratio,
            f"Driving target sign pattern OK ({same_sign_ratio:.2%})",
            (
                "Driving target sign pattern weak "
                f"({same_sign_ratio:.2%} < {args.min_drive_sign_ratio:.2%})"
            ),
        )

    if args.expect_out_back and rows:
        start_x = rows[0]["odom_x"]
        start_y = rows[0]["odom_y"]
        max_dist = 0.0
        for row in rows:
            dist = math.hypot(row["odom_x"] - start_x, row["odom_y"] - start_y)
            if dist > max_dist:
                max_dist = dist
        end_dist = math.hypot(rows[-1]["odom_x"] - start_x, rows[-1]["odom_y"] - start_y)

        checks_ok &= _check(
            max_dist >= args.min_out_distance_ft,
            f"Out leg distance OK ({max_dist:.2f} ft)",
            f"Out leg distance too short ({max_dist:.2f} ft < {args.min_out_distance_ft:.2f} ft)",
        )
        checks_ok &= _check(
            end_dist <= args.max_return_error_ft,
            f"Return-to-start error OK ({end_dist:.2f} ft)",
            (
                "Return-to-start error too high "
                f"({end_dist:.2f} ft > {args.max_return_error_ft:.2f} ft)"
            ),
        )

    if checks_ok:
        _emit("RESULT", "PASS")
        return 0
    _emit("RESULT", "FAIL")
    return 1


def parse_args():
    parser = argparse.ArgumentParser(description="Validate HIL CSV telemetry logs.")
    parser.add_argument("--log-file", default="", help="Path to a specific CSV log file.")
    parser.add_argument(
        "--log-dir",
        default=os.path.join(os.path.dirname(__file__), "..", "logs"),
        help="Directory containing motor_log_*.csv files.",
    )
    parser.add_argument(
        "--mode",
        default="any",
        choices=["any", "voice", "joystick", "ml"],
        help="Filename mode filter when selecting latest log.",
    )
    parser.add_argument("--expect-ml", action="store_true", help="Require ML controller rows.")
    parser.add_argument(
        "--expect-out-back",
        action="store_true",
        help="Check for out-and-back distance behavior.",
    )
    parser.add_argument("--min-rows", type=int, default=200, help="Minimum total rows required.")
    parser.add_argument("--min-ml-rows", type=int, default=50, help="Minimum ML rows required.")
    parser.add_argument("--pwm-delta-ns", type=float, default=20_000.0, help="Delta from neutral to count as active PWM.")
    parser.add_argument("--vel-eps", type=float, default=1.0, help="Velocity magnitude threshold for activity.")
    parser.add_argument("--accel-eps", type=float, default=1.0, help="Acceleration magnitude threshold for activity.")
    parser.add_argument(
        "--min-pwm-active-ratio",
        type=float,
        default=0.10,
        help="Minimum active-PWM ratio in motion rows.",
    )
    parser.add_argument(
        "--min-dynamics-ratio",
        type=float,
        default=0.10,
        help="Minimum active velocity/acceleration ratio in motion rows.",
    )
    parser.add_argument(
        "--min-turn-sign-ratio",
        type=float,
        default=0.70,
        help="Minimum ratio of opposite wheel target signs during turning.",
    )
    parser.add_argument(
        "--min-drive-sign-ratio",
        type=float,
        default=0.70,
        help="Minimum ratio of same wheel target signs during driving.",
    )
    parser.add_argument(
        "--min-out-distance-ft",
        type=float,
        default=8.0,
        help="Minimum expected max distance from start for out-and-back check.",
    )
    parser.add_argument(
        "--max-return-error-ft",
        type=float,
        default=3.0,
        help="Maximum allowed end distance from start for out-and-back check.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        return validate(args)
    except Exception as exc:
        _emit("ERROR", str(exc))
        return 2


if __name__ == "__main__":
    sys.exit(main())
