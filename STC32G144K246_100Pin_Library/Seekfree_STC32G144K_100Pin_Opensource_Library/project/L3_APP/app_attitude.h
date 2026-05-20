#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include "zf_common_typedef.h"

#ifndef APP_ATTITUDE_ACC_LPF_ALPHA_DEFAULT
#define APP_ATTITUDE_ACC_LPF_ALPHA_DEFAULT     (0.5f)
#endif

#ifndef APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT
#define APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT    (0.5f)
#endif

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} app_attitude_data_t;

void app_attitude_init(void);
void app_attitude_task(void);
void app_attitude_get_data(app_attitude_data_t *out_data);

#endif
