#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include "zf_common_typedef.h"
#include "service_imu.h"

#ifndef APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT
#define APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT    (0.5f)
#endif

typedef struct
{
    float gyro_z;
    /* Vendor SFLP order [q0..q3]; not labelled x/y/z/w until installation is verified. */
    float quaternion_0;
    float quaternion_1;
    float quaternion_2;
    float quaternion_3;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32 sequence;
    uint8 valid;
} app_attitude_data_t;

void app_attitude_init(void);
void app_attitude_update(const service_imu_sample_t *imu);
void app_attitude_task(void);
void app_attitude_get_data(app_attitude_data_t *out_data);

#endif
