#ifndef APP_MOTION_POSTPROCESS_H
#define APP_MOTION_POSTPROCESS_H

#include "zf_common_typedef.h"

#ifndef APP_MOTION_POSTPROCESS_PIT
#define APP_MOTION_POSTPROCESS_PIT            TIM9_PIT
#endif

#ifndef APP_MOTION_POSTPROCESS_PERIOD_MS
#define APP_MOTION_POSTPROCESS_PERIOD_MS      (5U)
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float enable;
} app_motion_postprocess_config_t;

typedef struct
{
    float raw_error;
    float feedforward;
    float processed_error;
    float linear_mps;
    float target_yaw_rate_radps;
    float actual_yaw_rate_radps;
    float target_differential_speed;
    float load_left;
    float load_right;
    float left_target_mps;
    float right_target_mps;
    float enabled;
} app_motion_postprocess_data_t;

extern app_motion_postprocess_config_t app_motion_postprocess_config;

void app_motion_postprocess_init(void);
void app_motion_postprocess_get_data(app_motion_postprocess_data_t *out_data);

#endif
