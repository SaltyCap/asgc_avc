#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>

typedef enum {
    PID_PROFILE_DRIVE = 0,
    PID_PROFILE_TURN = 1
} PidProfile;

typedef enum {
    NAV_CONTROLLER_PID = 0,
    NAV_CONTROLLER_ML = 1
} NavControllerMode;

void* coordinated_control_thread(void* arg);

int control_set_pid_gains(PidProfile profile,
                          double kp,
                          double ki,
                          double kd_velocity,
                          double ka_accel,
                          double velocity_stop_threshold);

int control_get_pid_gains(PidProfile profile,
                          double* kp,
                          double* ki,
                          double* kd_velocity,
                          double* ka_accel,
                          double* velocity_stop_threshold);

int control_set_nav_controller_mode(NavControllerMode mode);
NavControllerMode control_get_nav_controller_mode(void);
void control_reset_pid_states(void);

#endif
