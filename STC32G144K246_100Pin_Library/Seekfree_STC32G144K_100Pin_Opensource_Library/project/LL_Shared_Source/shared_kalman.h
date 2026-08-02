#ifndef SHARED_KALMAN_H
#define SHARED_KALMAN_H

#include "zf_common_typedef.h"

/*
 * Two-state angle Kalman filter.
 *
 * The state is [angle, gyro_bias].  The prediction input is gyro rate and
 * the measurement input is an angle derived from the accelerometer.
 */
typedef struct
{
    float angle;
    float bias;
    float rate;
    float p00;
    float p01;
    float p10;
    float p11;
    float q_angle;
    float q_bias;
    float r_measure;
    uint8 initialized;
} shared_kalman_t;

void shared_kalman_init(shared_kalman_t *filter,
        float angle,
        float q_angle,
        float q_bias,
        float r_measure);
void shared_kalman_reset(shared_kalman_t *filter, float angle);
float shared_kalman_update(shared_kalman_t *filter,
        float measured_angle,
        float gyro_rate,
        float dt);
float shared_kalman_get_angle(const shared_kalman_t *filter);

#endif
