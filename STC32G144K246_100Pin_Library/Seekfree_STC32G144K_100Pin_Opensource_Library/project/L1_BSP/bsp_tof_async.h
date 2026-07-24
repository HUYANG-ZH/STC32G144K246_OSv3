#ifndef BSP_TOF_ASYNC_H
#define BSP_TOF_ASYNC_H

#include "zf_common_typedef.h"

uint8 bsp_tof_async_init(void);
void bsp_tof_async_process(void);
uint8 bsp_tof_async_is_ready(void);
uint8 bsp_tof_async_request_sample(void);
uint16 bsp_tof_async_get_distance_mm(void);
uint8 bsp_tof_async_get_last_error(void);

#endif
