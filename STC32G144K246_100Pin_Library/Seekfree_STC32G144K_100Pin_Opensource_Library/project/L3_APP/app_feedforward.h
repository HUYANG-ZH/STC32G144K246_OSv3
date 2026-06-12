#ifndef APP_FEEDFORWARD_H
#define APP_FEEDFORWARD_H

#include "zf_common_typedef.h"

#ifndef APP_FEEDFORWARD_TASK_ID
#define APP_FEEDFORWARD_TASK_ID             (0U)
#endif

#ifndef APP_FEEDFORWARD_PERIOD_MS
#define APP_FEEDFORWARD_PERIOD_MS           (5U)
#endif

typedef struct
{
    float kff;
    float kd;
    float denominator_bias;
} app_feedforward_config_t;

typedef struct
{
    float curvature;
    float curvature_rate;
    float feedforward;
} app_feedforward_data_t;

extern app_feedforward_config_t app_feedforward_config;

void app_feedforward_init(void);
void app_feedforward_get_data(app_feedforward_data_t *out_data);

#endif
