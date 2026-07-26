#include "zf_common_headfile.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "app_attitude.h"

static shared_lpf_t attitude_gyro_z_lpf;
static volatile app_attitude_data_t attitude_data;
static uint32 attitude_last_sequence = 0UL;

void app_attitude_init(void)
{
    service_imu_sample_t imu;
    float gyro_z = 0.0f;

    if(0U != service_imu_get_latest_sample(&imu))
    {
        gyro_z = imu.gyro_z;
    }
    shared_lpf_init(&attitude_gyro_z_lpf, APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT, gyro_z);
    attitude_data.gyro_z = gyro_z;
    attitude_data.quaternion_0 = 0.0f;
    attitude_data.quaternion_1 = 0.0f;
    attitude_data.quaternion_2 = 0.0f;
    attitude_data.quaternion_3 = 0.0f;
    attitude_data.roll_deg = 0.0f;
    attitude_data.pitch_deg = 0.0f;
    attitude_data.yaw_deg = 0.0f;
    attitude_data.sequence = 0UL;
    attitude_data.valid = 0U;
    attitude_last_sequence = 0UL;

    if(0U != service_imu_get_latest_sample(&imu))
    {
        app_attitude_update(&imu);
    }
}

void app_attitude_update(const service_imu_sample_t *imu)
{
    uint8 ea_backup;
    float gyro_z;

    if((NULL == imu) || (0U == imu->valid) ||
       (imu->sequence == attitude_last_sequence))
    {
        return;
    }

    gyro_z = shared_lpf_update(&attitude_gyro_z_lpf, imu->gyro_z);
    ea_backup = EA;
    EA = 0;
    attitude_data.gyro_z = gyro_z;
    attitude_data.quaternion_0 = imu->quaternion_0;
    attitude_data.quaternion_1 = imu->quaternion_1;
    attitude_data.quaternion_2 = imu->quaternion_2;
    attitude_data.quaternion_3 = imu->quaternion_3;
    attitude_data.roll_deg = imu->roll_deg;
    attitude_data.pitch_deg = imu->pitch_deg;
    attitude_data.yaw_deg = imu->yaw_deg;
    attitude_data.sequence = imu->sequence;
    attitude_data.valid = imu->valid;
    attitude_last_sequence = imu->sequence;
    EA = ea_backup;
}

void app_attitude_task(void)
{
    service_imu_sample_t imu;

    if(0U != service_imu_get_latest_sample(&imu))
    {
        app_attitude_update(&imu);
    }
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
    *out_data = attitude_data;
    EA = ea_backup;
}
