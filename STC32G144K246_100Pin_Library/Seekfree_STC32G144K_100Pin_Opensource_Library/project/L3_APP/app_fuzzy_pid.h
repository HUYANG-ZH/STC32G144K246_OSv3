#ifndef APP_FUZZY_PID_H
#define APP_FUZZY_PID_H

#include "zf_common_typedef.h"

typedef struct
{
    float error_scale;
    float error_delta_scale;
    float kp_scale;
    float kd_scale;
} app_fuzzy_pid_config_t;

typedef struct
{
    float delta_kp;
    float delta_kd;
} app_fuzzy_pid_data_t;

extern app_fuzzy_pid_config_t app_fuzzy_pid_config;

void app_fuzzy_pid_init(void);
void app_fuzzy_pid_update(float error, float error_delta, app_fuzzy_pid_data_t *out_data);

#endif
