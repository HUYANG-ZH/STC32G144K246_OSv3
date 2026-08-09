#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include "zf_common_typedef.h"
#include "service_imu.h"

#ifndef APP_ATTITUDE_KALMAN_Q_ANGLE_DEFAULT
#define APP_ATTITUDE_KALMAN_Q_ANGLE_DEFAULT   (0.001f)
#endif

#ifndef APP_ATTITUDE_KALMAN_Q_BIAS_DEFAULT
#define APP_ATTITUDE_KALMAN_Q_BIAS_DEFAULT    (0.003f)
#endif

#ifndef APP_ATTITUDE_KALMAN_R_MEASURE_DEFAULT
#define APP_ATTITUDE_KALMAN_R_MEASURE_DEFAULT (0.03f)
#endif

#ifndef APP_ATTITUDE_GYRO_Z_LPF_ALPHA_DEFAULT
#define APP_ATTITUDE_GYRO_Z_LPF_ALPHA_DEFAULT (0.5f)
#endif

/* pitch(0-360 全象限) 一阶低通系数 */
#ifndef APP_ATTITUDE_PITCH_LPF_ALPHA_DEFAULT
#define APP_ATTITUDE_PITCH_LPF_ALPHA_DEFAULT   (0.3f)
#endif

#ifndef APP_ATTITUDE_DT_MAX_SECOND
#define APP_ATTITUDE_DT_MAX_SECOND             (0.020f)
#endif

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_z;
    uint32 sequence;
    uint32 timestamp_tick;
    uint8 valid;
} app_attitude_data_t;

void app_attitude_init(void);
void app_attitude_update(const service_imu_sample_t *imu);
void app_attitude_task(void);
void app_attitude_get_data(app_attitude_data_t *out_data);

#endif
