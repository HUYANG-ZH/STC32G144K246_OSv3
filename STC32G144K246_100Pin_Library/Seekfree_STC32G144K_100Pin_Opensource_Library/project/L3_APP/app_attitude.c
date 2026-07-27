#include "zf_common_headfile.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "app_attitude.h"

static shared_lpf_t attitude_gyro_z_lpf;
static volatile app_attitude_data_t attitude_data;

void app_attitude_init(void)
{
    float gyro_z;

    gyro_z = service_imu_read_gyro_z();
    shared_lpf_init(&attitude_gyro_z_lpf, APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT, gyro_z);
    attitude_data.gyro_z = gyro_z;
}

void app_attitude_update(const service_imu_sample_t *imu)
{
    uint8 ea_backup;
    float gyro_z;

    if(NULL == imu)
    {
        return;
    }

    gyro_z = shared_lpf_update(&attitude_gyro_z_lpf, imu->gyro_z);
    ea_backup = EA;
    EA = 0;
    attitude_data.gyro_z = gyro_z;
    EA = ea_backup;
}

void app_attitude_task(void)
{
    uint8 ea_backup;
    float gyro_z;

    gyro_z = shared_lpf_update(&attitude_gyro_z_lpf, service_imu_read_gyro_z());

    ea_backup = EA;
    EA = 0;
    attitude_data.gyro_z = gyro_z;
    EA = ea_backup;
}

void app_attitude_get_data(app_attitude_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    out_data->gyro_z = attitude_data.gyro_z;
    EA = ea_backup;
}
