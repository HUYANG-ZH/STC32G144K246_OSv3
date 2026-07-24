#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "zf_common_typedef.h"

#define DEV_ID_IMU                (0x01U)

typedef struct
{
    int16 temperature_raw;
    int16 gyro_x_raw;
    int16 gyro_y_raw;
    int16 gyro_z_raw;
    int16 acc_x_raw;
    int16 acc_y_raw;
    int16 acc_z_raw;
    uint32 sequence;
    uint8 valid;
} bsp_imu_sample_t;

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} bsp_imu_gyro_t;

void bsp_imu_init(void);
void bsp_imu_bootstrap_process(void);
uint8 bsp_imu_request_sample(void);
uint8 bsp_imu_get_latest_sample(bsp_imu_sample_t *out_data);
uint32 bsp_imu_get_dma_error_count(void);
void bsp_imu_read_gyro(bsp_imu_gyro_t *out_data);
float bsp_imu_read_gyro_z(void);
uint8 bsp_imu_is_ready(void);

#endif
