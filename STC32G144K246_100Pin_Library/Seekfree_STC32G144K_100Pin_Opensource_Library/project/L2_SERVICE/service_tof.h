#ifndef SERVICE_TOF_H
#define SERVICE_TOF_H

#include "zf_common_typedef.h"

/* Board configuration: enable the installed ToF module and its IIC service. */
#ifndef SERVICE_TOF_ENABLE
#define SERVICE_TOF_ENABLE             (1U)
#endif

#define SERVICE_TOF_PERIOD_TICK        (333UL)   /* 约30Hz, 0.1ms/tick */

#ifndef SERVICE_TOF_DEBUG_PRINT_ENABLE
#define SERVICE_TOF_DEBUG_PRINT_ENABLE (0U)      /* 1: 30Hz 同步 printf 距离数据 (VOFA+ 文本协议) */
#endif

void service_tof_init(void);
void service_tof_task(void);
uint8 service_tof_is_ready(void);
uint8 service_tof_get_last_error(void);
uint16 service_tof_get_distance_mm(void);
uint8 service_tof_get_range_status(void);

#endif
