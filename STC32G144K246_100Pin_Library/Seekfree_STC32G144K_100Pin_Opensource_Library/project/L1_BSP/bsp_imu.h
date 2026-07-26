#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "zf_common_typedef.h"

#define DEV_ID_IMU                (0x01U)

typedef struct
{
    int16 gyro_x_raw;
    int16 gyro_y_raw;
    int16 gyro_z_raw;
    int16 acc_x_raw;
    int16 acc_y_raw;
    int16 acc_z_raw;
    /* IMU660RC SFLP quaternion in the legacy vendor order [q0..q3].
       FIFO supplies vector xyz; q0..q3 intentionally retain the established
       board-facing order until installation-axis verification is complete. */
    float quaternion_0;
    float quaternion_1;
    float quaternion_2;
    float quaternion_3;
    /* Euler angles derived from the quaternion, in degrees. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    /* Every field above comes from one FIFO DMA frame and shares this metadata. */
    uint32 sequence;
    uint32 drdy_tick;
    uint32 timestamp_tick;
    uint8 valid;
} bsp_imu_sample_t;

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} bsp_imu_gyro_t;

void bsp_imu_init(void);
void bsp_imu_task(void);
/* Legacy name retained for existing callers. */
void bsp_imu_bootstrap_process(void);
uint8 bsp_imu_request_sample(void);
uint8 bsp_imu_get_latest_sample(bsp_imu_sample_t *out_data);
uint32 bsp_imu_get_dma_error_count(void);
void bsp_imu_read_gyro(bsp_imu_gyro_t *out_data);
float bsp_imu_read_gyro_z(void);
uint8 bsp_imu_is_ready(void);

#endif
