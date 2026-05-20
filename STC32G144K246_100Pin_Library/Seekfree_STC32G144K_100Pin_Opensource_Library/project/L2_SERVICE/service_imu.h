#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H

#include "zf_common_typedef.h"

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
    float roll;
    float pitch;
    float yaw;
} service_imu_data_t;

void service_imu_init(void);
void service_imu_debug(void);
uint8 service_imu_task(void);
void service_imu_get_data(service_imu_data_t *out_imu);

#endif
