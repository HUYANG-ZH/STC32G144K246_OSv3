#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "zf_common_typedef.h"

#define DEV_ID_IMU                (0x01U)

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} bsp_imu_gyro_t;

void bsp_imu_init(void);
void bsp_imu_read_gyro(bsp_imu_gyro_t *out_data);
float bsp_imu_read_gyro_z(void);
uint8 bsp_imu_is_ready(void);

#endif
