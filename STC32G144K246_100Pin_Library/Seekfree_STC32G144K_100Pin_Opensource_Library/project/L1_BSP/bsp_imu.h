#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "zf_common_typedef.h"

#define DEV_ID_IMU                (0x01U)

void bsp_imu_init(void);
float bsp_imu_read_gyro_z(void);
uint8 bsp_imu_is_ready(void);

#endif
