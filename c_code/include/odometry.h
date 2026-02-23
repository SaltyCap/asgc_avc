#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include "motor.h"

void update_encoder_rotation(EncoderState *enc, int16_t raw_angle, int motor_id);
void* encoder_feedback_thread(void* arg);
int32_t calculate_turn_counts(double degrees);
void update_odometry(void);

#endif
