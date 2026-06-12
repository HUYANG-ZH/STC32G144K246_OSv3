#ifndef APP_SPEED_PLAN_H
#define APP_SPEED_PLAN_H

#include "zf_common_typedef.h"

#ifndef APP_SPEED_PLAN_TASK_ID
#define APP_SPEED_PLAN_TASK_ID             (2U)
#endif

#ifndef APP_SPEED_PLAN_PERIOD_MS
#define APP_SPEED_PLAN_PERIOD_MS           (5U)
#endif

void app_speed_plan_init(void);
float app_speed_plan_get_linear_mps(void);

#endif
