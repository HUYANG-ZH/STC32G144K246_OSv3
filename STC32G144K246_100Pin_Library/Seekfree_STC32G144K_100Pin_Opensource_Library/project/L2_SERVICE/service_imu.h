#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H

#include "zf_common_typedef.h"

typedef struct
{
    int16 temperature_raw;
    int16 gyro_x_raw;
    int16 gyro_y_raw;
    int16 gyro_z_raw;
    int16 acc_x_raw;
    int16 acc_y_raw;
    int16 acc_z_raw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    uint32 sequence;
    uint8 valid;
} service_imu_sample_t;

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint32 sequence;
} service_imu_gyro_t;

void service_imu_init(void);
void service_imu_task(void);
void service_imu_update(void);
void service_imu_start_calibration(void);
uint8 service_imu_calibration_is_complete(void);
void service_imu_calibrate_gyro_x(void);
void service_imu_calibrate_gyro_z(void);
void service_imu_read_gyro(service_imu_gyro_t *out_data);
float service_imu_read_gyro_z(void);
uint8 service_imu_get_latest_sample(service_imu_sample_t *out_data);

#endif
