#ifndef APP_MOTION_PREPROCESS_H
#define APP_MOTION_PREPROCESS_H

#include "zf_common_typedef.h"

#ifndef APP_MOTION_PREPROCESS_PIT
#define APP_MOTION_PREPROCESS_PIT              TIM6_PIT
#endif

#ifndef APP_MOTION_PREPROCESS_PERIOD_MS
#define APP_MOTION_PREPROCESS_PERIOD_MS        (5U)
#endif

typedef struct
{
    float linear_mps;
    float yaw_rate_gain;
} app_motion_preprocess_config_t;

typedef struct
{
    float x_error;
    float y_error;
    float line_error;
    float linear_mps;
    float target_yaw_rate_rps;
} app_motion_preprocess_data_t;

extern app_motion_preprocess_config_t app_motion_preprocess_config;

void app_motion_preprocess_init(void);
/* 由实时控制链调用；初始化阶段仅允许一次无阻塞的快照预热。 */
void app_motion_preprocess_control_step(void);
void app_motion_preprocess_get_data(app_motion_preprocess_data_t *out_data);

#endif
