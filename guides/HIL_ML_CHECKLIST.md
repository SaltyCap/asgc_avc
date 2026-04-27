# HIL ML Controller Checklist

This checklist verifies that ML controller mode drives navigation output while preserving queue/navigation behavior.

## 1) Preflight (robot host)

1. Put robot on stand (wheels clear), then limit power to a safe value from UI (for example 20-25%).
2. Build and start stack:

```bash
cd /Users/christianmorton/Desktop/asgc_avc_v1_ml_backup
./start_all.sh
```

3. Open:
- Main UI: `https://<robot-host>:5001/`
- Tuning UI: `https://<robot-host>:5001/tuning`

## 2) Turning check (first)

1. In `/tuning`, set controller to `Use ML`.
2. Run `Turning Test` with `90 deg`.
3. Confirm phase progresses:
- `turning: reset pose`
- `turning: +90.0 deg`
- `turning: -90.0 deg`
- `complete`

## 3) Straight check (second)

1. In `/tuning`, keep controller in `Use ML`.
2. Run `Run 10ft Out-and-Back`.
3. Confirm phase progresses:
- `straight: reset pose`
- `straight: drive +10ft`
- `straight: turn 180`
- `straight: drive -10ft`
- `complete`

## 4) Save profile

1. Click `Save Profile`.
2. Optional: click `Load Profile` and verify values/mode restore.

## 5) Log validation

Stop run to flush logs, then validate:

```bash
cd /Users/christianmorton/Desktop/asgc_avc_v1_ml_backup/tools
./view_logs.sh validate_ml out_back
```

Expected result:
- `[RESULT] PASS`

Validation checks include:
- `nav_controller_mode=ML` rows are present.
- Motion rows in `TURNING/DRIVING` exist.
- PWM activity is non-trivial during motion.
- Encoder velocity and acceleration are active (not flat zero).
- Target sign pattern is sane for turning/driving.
- Out-and-back behavior reaches outward distance and returns near start.

