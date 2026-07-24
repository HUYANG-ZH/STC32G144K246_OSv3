#ifndef BSP_TOF_H
#define BSP_TOF_H

#include "zf_common_typedef.h"

#define BSP_TOF_DRIVER_DL1A             (1U)
#define BSP_TOF_DRIVER_DL1B             (2U)

#ifndef BSP_TOF_DRIVER
#define BSP_TOF_DRIVER                  BSP_TOF_DRIVER_DL1B
#endif

#define BSP_TOF_INVALID_DISTANCE_MM    (8192U)

uint8 bsp_tof_init(void);
void bsp_tof_process(void);
uint8 bsp_tof_is_ready(void);
uint8 bsp_tof_request_sample(void);
uint16 bsp_tof_get_distance_mm(void);
uint8 bsp_tof_get_last_error(void);

#endif
