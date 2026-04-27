import threading
import time
import math
import json
import os

from .config import Config
from .motor_interface import motor_interface


class PidTuningManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._worker = None
        self._stop_event = threading.Event()

        self._active_test = None
        self._phase = "idle"
        self._last_result = ""
        self._last_error = ""
        self._updated_at = time.time()
        self._target_heading = None
        
        self._autotune_worker = None
        self._autotune_pause_event = threading.Event()
        self._autotune_cancel_event = threading.Event()
        self._autotune_state = "idle"
        self._autotune_profile = None
        self._best_cost = float('inf')

        self._profiles = {
            "turn": {
                "kp": 40.0,
                "ki": 2.8,
                "kd_velocity": 21.0,
                "ka_accel": 0.08,
                "velocity_stop_threshold": 300.0,
            },
            "drive": {
                "kp": 34.0,
                "ki": 2.2,
                "kd_velocity": 18.0,
                "ka_accel": 0.06,
                "velocity_stop_threshold": 260.0,
            },
        }
        self._nav_controller_mode = "pid"
        self._profile_path = Config.get_tuning_profile_path()

        self._load_profile_from_disk(strict=False)

    def snapshot(self):
        with self._lock:
            running = bool(self._worker and self._worker.is_alive())
            return {
                "running": running,
                "active_test": self._active_test,
                "phase": self._phase,
                "last_result": self._last_result,
                "last_error": self._last_error,
                "updated_at": self._updated_at,
                "nav_controller_mode": self._nav_controller_mode,
                "profile_path": self._profile_path,
                "autotune_state": self._autotune_state,
                "autotune_profile": self._autotune_profile,
                "autotune_best_cost": self._best_cost if self._best_cost != float('inf') else None,
                "profiles": {
                    "turn": dict(self._profiles["turn"]),
                    "drive": dict(self._profiles["drive"]),
                },
            }

    def get_telemetry(self):
        nav = motor_interface.nav_controller
        if not nav:
            return {"heading": 0.0, "target_heading": None, "time": time.time()}
        
        pos = nav.get_position()
        return {
            "heading": pos.get("heading", 0.0),
            "target_heading": self._target_heading,
            "time": time.time()
        }

    def set_profile(self, profile_name, gains):
        profile = self._normalize_profile(profile_name)
        clean = self._validate_gains(gains)
        # Always update the in-memory profile first so that save_profile() will
        # persist the new values even if the motor controller is currently offline.
        with self._lock:
            self._profiles[profile] = clean
            self._last_result = f"{profile} gains updated"
            self._last_error = ""
            self._updated_at = time.time()
        # Best-effort: push the new gains to the running motor controller.
        # A failure here is non-fatal; the values are already stored in memory
        # and can be synced next time the motor is restarted or a test is run.
        try:
            self._send_pid_profile(profile, clean)
        except Exception as exc:
            with self._lock:
                self._last_error = f"motor offline – gains saved in memory only: {exc}"
                self._updated_at = time.time()

    def set_nav_controller_mode(self, mode_name):
        mode = self._normalize_controller_mode(mode_name)
        self._send_command(f"nav_controller {mode}")
        with self._lock:
            self._nav_controller_mode = mode
            self._last_result = f"controller mode set to {mode}"
            self._last_error = ""
            self._updated_at = time.time()

    def save_profile(self):
        with self._lock:
            payload = {
                "nav_controller_mode": self._nav_controller_mode,
                "profiles": {
                    "turn": dict(self._profiles["turn"]),
                    "drive": dict(self._profiles["drive"]),
                },
            }
            profile_path = self._profile_path

        profile_dir = os.path.dirname(profile_path)
        if profile_dir:
            os.makedirs(profile_dir, exist_ok=True)
        with open(profile_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)

        with self._lock:
            self._last_result = f"profile saved to {profile_path}"
            self._last_error = ""
            self._updated_at = time.time()

    def load_profile(self):
        self._load_profile_from_disk(strict=True)
        self._sync_profiles_to_motor()
        with self._lock:
            mode = self._nav_controller_mode
        self._send_command(f"nav_controller {mode}")
        with self._lock:
            self._last_result = "profile loaded from disk"
            self._last_error = ""
            self._updated_at = time.time()

    def stop_test(self):
        with self._lock:
            worker = self._worker
            running = bool(worker and worker.is_alive())
            self._stop_event.set()
            self._autotune_cancel_event.set()
            if self._autotune_pause_event.is_set():
                self._autotune_pause_event.clear()
            self._phase = "stopping"
            self._updated_at = time.time()

        motor_interface.send_command("stop")
        return running

    def start_turning_test(self, angle_deg=90.0):
        angle = float(angle_deg)
        if angle <= 0.0:
            raise ValueError("turning test angle must be > 0")
        return self._start_test(
            "turning",
            lambda: self._run_turning_test(angle),
        )

    def start_straight_test(self):
        return self._start_test("straight", self._run_straight_test)

    def _start_test(self, name, target):
        with self._lock:
            if self._worker and self._worker.is_alive():
                return False
            self._active_test = name
            self._phase = "starting"
            self._last_result = ""
            self._last_error = ""
            self._updated_at = time.time()
            self._stop_event.clear()

            self._worker = threading.Thread(
                target=self._test_runner,
                args=(target,),
                daemon=True,
                name=f"pid-tuning-{name}",
            )
            self._worker.start()
            return True

    def _test_runner(self, target):
        try:
            self._prepare_for_test()
            target()
            self._set_phase("complete")
            self._set_result("test completed")
        except Exception as exc:
            self._set_error(str(exc))
            self._set_phase("failed")
        finally:
            with self._lock:
                self._active_test = None
                self._updated_at = time.time()

    def start_autotune(self, profile):
        profile = self._normalize_profile(profile)
        with self._lock:
            if self._autotune_worker and self._autotune_worker.is_alive():
                return False
            if self._worker and self._worker.is_alive():
                return False
            self._autotune_profile = profile
            self._autotune_state = "starting"
            self._best_cost = float('inf')
            self._last_result = ""
            self._last_error = ""
            self._updated_at = time.time()
            self._autotune_cancel_event.clear()
            self._autotune_pause_event.clear()

            self._autotune_worker = threading.Thread(
                target=self._autotune_runner,
                args=(profile,),
                daemon=True,
                name=f"pid-autotuning-{profile}",
            )
            self._autotune_worker.start()
            return True

    def pause_autotune(self):
        with self._lock:
            if self._autotune_worker and self._autotune_worker.is_alive():
                self._autotune_pause_event.set()
                self._last_result = "autotune pause requested"
                self._updated_at = time.time()
                return True
            return False

    def resume_autotune(self):
        with self._lock:
            if self._autotune_worker and self._autotune_worker.is_alive():
                self._autotune_pause_event.clear()
                self._last_result = "autotune resumed"
                self._updated_at = time.time()
                return True
            return False

    def cancel_autotune(self):
        with self._lock:
            running = bool(self._autotune_worker and self._autotune_worker.is_alive())
            self._autotune_cancel_event.set()
            # If it's paused, we must un-pause it so the thread can wake up and exit
            self._autotune_pause_event.clear()
            self._autotune_state = "cancelling"
            self._updated_at = time.time()

        motor_interface.send_command("stop")
        return running

    def _set_autotune_state(self, state):
        with self._lock:
            self._autotune_state = state
            self._updated_at = time.time()

    def _check_autotune_events(self):
        if self._autotune_cancel_event.is_set():
            raise RuntimeError("autotuning cancelled")
        if self._autotune_pause_event.is_set():
            self._set_autotune_state("paused")
            while self._autotune_pause_event.is_set():
                if self._autotune_cancel_event.is_set():
                    raise RuntimeError("autotuning cancelled")
                time.sleep(0.1)
            self._set_autotune_state("running")

    def _autotune_runner(self, profile):
        try:
            self._set_autotune_state("running")
            # Tunable parameters: kp, ki, kd_velocity
            param_names = ["kp", "ki", "kd_velocity"]
            
            with self._lock:
                gains = dict(self._profiles[profile])
            
            p = [gains[n] for n in param_names]
            best_p = list(p)
            dp = [max(1.0, 0.25 * x) if x > 0 else 0.5 for x in p] # Initial dp: 25% of value
            
            def eval_cost(curr_p):
                self._check_autotune_events()
                new_gains = gains.copy()
                for i, name in enumerate(param_names):
                    if curr_p[i] < 0: 
                        curr_p[i] = 0.0 # PID params shouldn't be negative here
                    new_gains[name] = curr_p[i]
                
                self.set_profile(profile, new_gains)
                try:
                    self._stop_event.clear()
                    self._prepare_for_test()
                    if profile == "turn":
                        cost = self._run_turning_test(90.0)
                    else:
                        cost = self._run_straight_test()
                    return cost
                except Exception:
                    # Timeout or motion failed -> massive penalty
                    return 1000.0

            best_err = eval_cost(p)
            with self._lock:
                self._best_cost = best_err
            
            tol = 0.6
            iters = 0
            while sum(dp) > tol:
                self._check_autotune_events()
                for i in range(len(p)):
                    p[i] += dp[i]
                    err = eval_cost(p)
                    
                    if err < best_err:
                        best_err = err
                        best_p = list(p)
                        with self._lock:
                            self._best_cost = best_err
                        dp[i] *= 1.1
                    else:
                        p[i] -= 2 * dp[i]
                        err = eval_cost(p)
                        
                        if err < best_err:
                            best_err = err
                            best_p = list(p)
                            with self._lock:
                                self._best_cost = best_err
                            dp[i] *= 1.1
                        else:
                            p[i] += dp[i]
                            dp[i] *= 0.9
                
                iters += 1
                if iters > 30: # Prevent infinite loop
                    break
                    
            self._set_autotune_state("complete")
            # Restore best_p to the active profile before saving
            final_gains = gains.copy()
            for i, name in enumerate(param_names):
                final_gains[name] = best_p[i]
            self.set_profile(profile, final_gains)

            self.save_profile()
            self._set_result(f"Autotune complete. Best cost: {best_err:.2f}s")
            
        except Exception as exc:
            self._set_error(f"Autotune error: {exc}")
            self._set_autotune_state("failed")
            # Attempt to restore best_p on failure or cancellation
            try:
                final_gains = gains.copy()
                for i, name in enumerate(param_names):
                    final_gains[name] = best_p[i]
                self.set_profile(profile, final_gains)
            except Exception:
                pass
        finally:
            with self._lock:
                self._autotune_profile = None
                self._updated_at = time.time()

    def _prepare_for_test(self):
        self._set_phase("preparing")
        nav = motor_interface.nav_controller
        if nav is None:
            raise RuntimeError("navigation controller is not initialized")

        nav.clear_queue()

        if not motor_interface.send_command("regular_mode"):
            raise RuntimeError("motor controller is offline")

        motor_interface.send_command("stop")

        # Tuning tests are intended to evaluate PID gains, so force PID mode.
        forced_pid = False
        with self._lock:
            if self._nav_controller_mode != "pid":
                self._nav_controller_mode = "pid"
                forced_pid = True
        if forced_pid:
            self._set_result("controller mode forced to pid for tuning test")

        self._sync_profiles_to_motor()

    def _sync_profiles_to_motor(self):
        with self._lock:
            turn = dict(self._profiles["turn"])
            drive = dict(self._profiles["drive"])
            mode = self._nav_controller_mode
        self._send_pid_profile("turn", turn)
        self._send_pid_profile("drive", drive)
        self._send_command(f"nav_controller {mode}")

    def _run_turning_test(self, angle_deg):
        self._set_phase("turning: reset pose")
        self._reset_pose()
        self._check_cancel()

        nav = motor_interface.nav_controller
        current_h = nav.get_position().get("heading", Config.START_HEADING)

        self._set_phase(f"turning: +{angle_deg:.1f} deg")
        self._target_heading = current_h + angle_deg
        self._send_command(f"turn {angle_deg:.2f}")
        t1 = self._wait_for_motion_complete("turn +")
        self._target_heading = None
        
        current_h = nav.get_position().get("heading", Config.START_HEADING)
        self._set_phase(f"turning: -{angle_deg:.1f} deg")
        self._target_heading = current_h - angle_deg
        self._send_command(f"turn {-angle_deg:.2f}")
        t2 = self._wait_for_motion_complete("turn -")
        self._target_heading = None
        return t1 + t2

    def _run_straight_test(self):
        nav = motor_interface.nav_controller
        start_h = float(nav.get_position().get("heading", Config.START_HEADING)) if nav else float(Config.START_HEADING)
        start_x = float(Config.START_POSITION[0])
        start_y = float(Config.START_POSITION[1])
        
        heading_rad = math.radians(start_h)
        # Drive 10ft so integral term has time to wind up and settle
        target_x = start_x + (10.0 * math.cos(heading_rad))
        target_y = start_y + (10.0 * math.sin(heading_rad))

        self._set_phase("straight: reset pose")
        self._send_command(f"setpos {start_x:.2f} {start_y:.2f} {start_h:.2f}")
        time.sleep(0.25)
        self._check_cancel()

        self._set_phase("straight: drive +10ft")
        self._target_heading = start_h
        self._send_command(f"goto {target_x:.2f} {target_y:.2f} 0")
        t1 = self._wait_for_motion_complete("drive forward")
        self._target_heading = None
        
        nav = motor_interface.nav_controller
        current_h = nav.get_position().get("heading", start_h)

        self._set_phase("straight: turn 180")
        self._target_heading = current_h + 180.0
        self._send_command("turn 180.00")
        t2 = self._wait_for_motion_complete("turn 180")
        self._target_heading = None
        
        current_h = nav.get_position().get("heading", start_h)

        self._set_phase("straight: drive -10ft")
        self._target_heading = current_h
        self._send_command(f"goto {start_x:.2f} {start_y:.2f} 0")
        t3 = self._wait_for_motion_complete("drive back")
        self._target_heading = None
        return t1 + t2 + t3

    def _reset_pose(self):
        nav = motor_interface.nav_controller
        current_h = float(nav.get_position().get("heading", Config.START_HEADING)) if nav else float(Config.START_HEADING)
        start_x = float(Config.START_POSITION[0])
        start_y = float(Config.START_POSITION[1])
        
        self._send_command(f"setpos {start_x:.2f} {start_y:.2f} {current_h:.2f}")
        time.sleep(0.25)

    def _wait_for_motion_complete(self, step_name, timeout_sec=60.0):
        start = time.monotonic()
        saw_motion = False
        idle_since = None

        while True:
            self._check_cancel()
            now = time.monotonic()
            if now - start > timeout_sec:
                raise RuntimeError(f"{step_name}: timeout waiting for completion")

            state = self._current_nav_state()
            if state and state != "IDLE":
                saw_motion = True
                idle_since = None
            elif saw_motion:
                if idle_since is None:
                    idle_since = now
                elif now - idle_since >= 0.35:
                    return idle_since - start
            elif now - start > 3.0:
                raise RuntimeError(f"{step_name}: no motion detected")

            time.sleep(0.05)

    def _current_nav_state(self):
        nav = motor_interface.nav_controller
        if nav is None:
            raise RuntimeError("navigation controller unavailable")
        status = nav.get_position()
        return str(status.get("state", "UNKNOWN")).upper()

    def _normalize_profile(self, profile_name):
        token = str(profile_name).strip().lower()
        if token in ("turn", "turning"):
            return "turn"
        if token in ("drive", "straight"):
            return "drive"
        raise ValueError("profile must be 'turn' or 'drive'")

    def _normalize_controller_mode(self, mode_name):
        token = str(mode_name).strip().lower()
        if token == "pid":
            return "pid"
        if token in ("ml", "model"):
            return "ml"
        raise ValueError("controller mode must be 'pid' or 'ml'")

    def _validate_gains(self, gains):
        required = ["kp", "ki", "kd_velocity", "ka_accel", "velocity_stop_threshold"]
        clean = {}
        for key in required:
            if key not in gains:
                raise ValueError(f"missing gain: {key}")
            value = float(gains[key])
            if value < 0.0:
                raise ValueError(f"{key} must be >= 0")
            clean[key] = value
        return clean

    def _send_pid_profile(self, profile, gains):
        command = (
            f"pid_set {profile} {gains['kp']:.6f} {gains['ki']:.6f} "
            f"{gains['kd_velocity']:.6f} {gains['ka_accel']:.6f} "
            f"{gains['velocity_stop_threshold']:.6f}"
        )
        self._send_command(command)

    def _send_command(self, command):
        if not motor_interface.send_command(command):
            raise RuntimeError(f"failed to send command: {command}")

    def _check_cancel(self):
        if self._stop_event.is_set():
            raise RuntimeError("test cancelled")

    def _set_phase(self, phase):
        with self._lock:
            self._phase = phase
            self._updated_at = time.time()

    def _set_result(self, message):
        with self._lock:
            self._last_result = message
            self._last_error = ""
            self._updated_at = time.time()

    def _set_error(self, message):
        with self._lock:
            self._last_error = message
            self._last_result = ""
            self._updated_at = time.time()

    def _load_profile_from_disk(self, strict=False):
        profile_path = self._profile_path
        if not profile_path or not os.path.exists(profile_path):
            if strict:
                raise FileNotFoundError(profile_path)
            return

        with open(profile_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)

        mode = self._normalize_controller_mode(payload.get("nav_controller_mode", "pid"))
        profiles = payload.get("profiles", {})
        turn = self._validate_gains(profiles.get("turn", self._profiles["turn"]))
        drive = self._validate_gains(profiles.get("drive", self._profiles["drive"]))

        with self._lock:
            self._nav_controller_mode = mode
            self._profiles["turn"] = turn
            self._profiles["drive"] = drive


pid_tuning_manager = PidTuningManager()
