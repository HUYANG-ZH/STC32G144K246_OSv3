#ifndef SERVICE_SPEED_H
#define SERVICE_SPEED_H

#include "zf_common_typedef.h"

typedef struct
{
    float left_mps;
    float right_mps;
    /* 编码器积分里程: 原始脉冲累加(米), 累加在 TIM3 1ms 采样中断内 */
    float odo_left_m;
    float odo_right_m;
    float odo_total_m;
} service_speed_data_t;

void service_speed_init(void);
void service_speed_debug(void);
void service_speed_update(void);
void service_speed_get(service_speed_data_t *out_speed);
void service_speed_odometer_reset(void);

#endif
