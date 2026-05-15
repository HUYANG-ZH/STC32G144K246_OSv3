#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include "zf_common_typedef.h"

#define APP_SCHEDULER_TASK_MAX (10U)

void app_scheduler_init(void);
uint8 app_scheduler_add(uint8 id, void (*task_func)(void), uint8 priority, uint16 period_ms);
void app_scheduler_run(void);

#endif
