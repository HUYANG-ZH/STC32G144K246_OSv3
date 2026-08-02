#include "shared_kalman.h"
#include "sys_tfpu.h"

void shared_kalman_init(shared_kalman_t *filter,
        float angle,
        float q_angle,
        float q_bias,
        float r_measure)
{
    if(NULL == filter)
    {
        return;
    }

    filter->angle = angle;
    filter->bias = 0.0f;
    filter->rate = 0.0f;
    filter->p00 = 0.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 0.0f;
    filter->q_angle = q_angle;
    filter->q_bias = q_bias;
    filter->r_measure = r_measure;
    filter->initialized = 1U;
}

void shared_kalman_reset(shared_kalman_t *filter, float angle)
{
    if(NULL == filter)
    {
        return;
    }

    filter->angle = angle;
    filter->bias = 0.0f;
    filter->rate = 0.0f;
    filter->p00 = 0.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 0.0f;
    filter->initialized = 1U;
}

float shared_kalman_update(shared_kalman_t *filter,
        float measured_angle,
        float gyro_rate,
        float dt)
{
    float p00_temp;
    float p01_temp;
    float s;
    float k0;
    float k1;
    float innovation;

    if(NULL == filter)
    {
        return 0.0f;
    }
    if(0U == filter->initialized)
    {
        shared_kalman_init(filter, measured_angle,
                filter->q_angle, filter->q_bias, filter->r_measure);
        return filter->angle;
    }
    if(dt <= 0.0f)
    {
        dt = 0.0001f;
    }

    /* Time update: angle is driven by the bias-corrected gyro rate. */
    filter->rate = tfpu_sub(gyro_rate, filter->bias);
    filter->angle = tfpu_add(filter->angle, tfpu_mul(dt, filter->rate));
    filter->p00 = tfpu_add(filter->p00,
            tfpu_mul(dt, tfpu_add(tfpu_sub(tfpu_mul(dt, filter->p11), filter->p01),
                    tfpu_add(tfpu_sub(0.0f, filter->p10), filter->q_angle))));
    filter->p01 = tfpu_sub(filter->p01, tfpu_mul(dt, filter->p11));
    filter->p10 = tfpu_sub(filter->p10, tfpu_mul(dt, filter->p11));
    filter->p11 = tfpu_add(filter->p11, tfpu_mul(filter->q_bias, dt));

    /* Measurement update: accelerometer-derived angle corrects drift. */
    s = tfpu_add(filter->p00, filter->r_measure);
    if(s <= 0.0f)
    {
        return filter->angle;
    }
    k0 = tfpu_div(filter->p00, s);
    k1 = tfpu_div(filter->p10, s);
    innovation = tfpu_sub(measured_angle, filter->angle);
    filter->angle = tfpu_add(filter->angle, tfpu_mul(k0, innovation));
    filter->bias = tfpu_add(filter->bias, tfpu_mul(k1, innovation));

    p00_temp = filter->p00;
    p01_temp = filter->p01;
    filter->p00 = tfpu_sub(filter->p00, tfpu_mul(k0, p00_temp));
    filter->p01 = tfpu_sub(filter->p01, tfpu_mul(k0, p01_temp));
    filter->p10 = tfpu_sub(filter->p10, tfpu_mul(k1, p00_temp));
    filter->p11 = tfpu_sub(filter->p11, tfpu_mul(k1, p01_temp));

    return filter->angle;
}

float shared_kalman_get_angle(const shared_kalman_t *filter)
{
    if(NULL == filter)
    {
        return 0.0f;
    }

    return filter->angle;
}
