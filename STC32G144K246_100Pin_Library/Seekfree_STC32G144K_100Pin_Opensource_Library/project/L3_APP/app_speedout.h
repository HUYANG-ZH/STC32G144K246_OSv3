#ifndef APP_SPEEDOUT_H
#define APP_SPEEDOUT_H

#include "zf_common_typedef.h"

#ifndef APP_SPEEDOUT_PIT
#define APP_SPEEDOUT_PIT                 TIM5_PIT
#endif

#ifndef APP_SPEEDOUT_PERIOD_MS
#define APP_SPEEDOUT_PERIOD_MS           (1U)
#endif

typedef struct
{
    float target_mps;
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} app_speedout_pid_config_t;

typedef struct
{
    app_speedout_pid_config_t left;
    app_speedout_pid_config_t right;
} app_speedout_config_t;

typedef struct
{
    float left_target_mps;
    float right_target_mps;
    float left_actual_mps;
    float right_actual_mps;
    float left_pwm;
    float right_pwm;
    float enabled;
} app_speedout_data_t;

extern app_speedout_config_t app_speedout_config;
extern app_speedout_data_t app_speedout_data;

void app_speedout_init(void);
void app_speedout_debug(void);
void app_speedout_stop(void);
void app_speedout_start(void);
void app_speedout_set_target(float left_mps, float right_mps);
void app_speedout_get_data(app_speedout_data_t *out_data);

#endif
