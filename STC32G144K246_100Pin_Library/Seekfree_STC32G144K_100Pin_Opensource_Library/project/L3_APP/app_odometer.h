#ifndef APP_ODOMETER_H
#define APP_ODOMETER_H

#include "zf_common_typedef.h"

/* 编码器积分里程计展示模块:
   里程累加在 service_speed TIM3 1ms 采样中断内完成(原始脉冲积分);
   本模块负责刷新无线展示缓存 + 提供清零动作与只读访问。 */
void app_odometer_init(void);
void app_odometer_task(void);
void app_odometer_reset(void);
float app_odometer_get_total_m(void);

#endif
