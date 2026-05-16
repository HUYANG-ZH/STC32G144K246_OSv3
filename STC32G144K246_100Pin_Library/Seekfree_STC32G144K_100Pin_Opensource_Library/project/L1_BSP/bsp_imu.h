#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "zf_common_typedef.h"

#define DEV_ID_IMU                (0x01U)

typedef struct
{
    float imu_gyro_x;
    float imu_gyro_y;
    float imu_gyro_z;
    float imu_acc_x;
    float imu_acc_y;
    float imu_acc_z;
    float imu_temperture;
    float imu_quat_x;
    float imu_quat_y;
    float imu_quat_z;
    float imu_quat_w;
    float imu_roll_deg;
    float imu_pitch_deg;
    float imu_yaw_deg;
} imu_data_t;

void bsp_imu_init(void);
void bsp_imu_debug(void);
void bsp_imu_read(imu_data_t *out_data);
void bsp_imu_get_temperature(float *temperature);
uint8 bsp_imu_is_ready(void);

#endif
