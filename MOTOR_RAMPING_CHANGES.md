# Motor Control Smooth Ramping Implementation

## Summary of Changes

Successfully replaced bang-bang motor control with smooth ramping system. The motors now gradually accelerate and decelerate over 3 seconds, providing gentler and more controlled movement.

## Files Modified

### 1. `/home/asgc/asgc_avc/c_code/src/main.c`

#### Removed:
- `g_min_pwm` global variable (line 25)
- Bang-bang control logic (lines 399-416 for left motor, 461-478 for right motor)
- Power boost/stall compensation logic
- Deadband threshold checks (lines 377-381 for left, 441-445 for right)

#### Added:
- **Smooth ramp constants:**
  ```c
  #define RAMP_UP_TIME 3.0    // 3 seconds to ramp from neutral to max
  #define RAMP_DOWN_TIME 3.0  // 3 seconds to ramp from max to neutral
  ```

- **Smooth ramping control logic** for both motors:
  - Tracks `ramp_start_time` to measure elapsed time since movement started
  - Calculates `ramp_factor` as `elapsed / RAMP_UP_TIME` (capped at 1.0)
  - Applies PWM as: `pwm = direction * MAX_PWM * ramp_factor`
  - Gradually increases from 0% to MAX_PWM over 3 seconds
  - Resets `ramp_start_time` to 0.0 when target is reached
  - Maintains stall detection with immediate full power if stalled (>2 stall counts)

#### Modified:
- `setpwm` command handler now only sets `g_max_pwm` (removed `g_min_pwm` assignment)
- PWM range check changed from `if (MAX_PWM < g_min_pwm)` to `if (MAX_PWM < 1)`

### 2. `/home/asgc/asgc_avc/c_code/include/common.h`

#### Removed:
- `DEADBAND_THRESHOLD` definition (was 200 counts)

### 3. `/home/asgc/asgc_avc/c_code/include/motor.h`

#### Added:
- New field in `EncoderState` structure:
  ```c
  double ramp_start_time;  // Time when ramping started (0.0 = not started)
  ```

## How It Works

### Acceleration (Ramp Up)
1. When a new target is set, `ramp_start_time` is initialized to current time
2. Each control loop iteration calculates elapsed time since ramp start
3. PWM increases linearly from 0% to MAX_PWM over 3 seconds
4. Formula: `pwm = direction * MAX_PWM * (elapsed / 3.0)`

### Deceleration (Ramp Down)
- Motors naturally decelerate as they approach the target
- When within STOP_THRESHOLD (200 counts ≈ 0.8 inches), motors stop immediately
- `ramp_start_time` is reset to 0.0 for next movement

### Stall Handling
- Stall detection remains active
- If stalled more than 2 times, bypass ramping and apply full power immediately
- This ensures the robot doesn't get stuck due to gentle ramping

## PWM Slider Behavior

The slider in `index.html` now controls the **full PWM range** around 1500µs neutral:
- **0%**: min=1500µs, max=1500µs (no movement)
- **50%**: min=1250µs, max=1750µs (half range)
- **100%**: min=1000µs, max=2000µs (full range)

The `g_max_pwm` variable stores the percentage from the slider and is used to scale the ramped PWM output.

## Benefits

1. **Smoother operation**: No more sudden jerks or oscillations
2. **Gentler on hardware**: Gradual acceleration reduces mechanical stress
3. **Better control**: Predictable ramping behavior
4. **Maintained safety**: Stall detection still provides full power when needed

## Testing Recommendations

1. Test with low slider values (20-30%) first to verify gentle ramping
2. Gradually increase slider to observe scaling behavior
3. Monitor logs to verify smooth PWM transitions
4. Test stall recovery to ensure full power kicks in when needed
