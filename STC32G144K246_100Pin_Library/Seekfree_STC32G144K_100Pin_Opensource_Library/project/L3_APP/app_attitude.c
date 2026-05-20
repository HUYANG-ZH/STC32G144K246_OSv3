#include "zf_common_headfile.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "app_attitude.h"

static shared_lpf_t attitude_acc_x_lpf;
static shared_lpf_t attitude_acc_y_lpf;
static shared_lpf_t attitude_acc_z_lpf;
static shared_lpf_t attitude_gyro_x_lpf;
static shared_lpf_t attitude_gyro_y_lpf;
static shared_lpf_t attitude_gyro_z_lpf;
static volatile app_attitude_data_t attitude_data;

static void app_attitude_copy_to_output(const service_imu_data_t *imu_data);

void app_attitude_init(void)
{
    service_imu_data_t imu_data;

    service_imu_get_data(&imu_data);

    shared_lpf_init(&attitude_acc_x_lpf, APP_ATTITUDE_ACC_LPF_ALPHA_DEFAULT, imu_data.acc_x);
    shared_lpf_init(&attitude_acc_y_lpf, APP_ATTITUDE_ACC_LPF_ALPHA_DEFAULT, imu_data.acc_y);
    shared_lpf_init(&attitude_acc_z_lpf, APP_ATTITUDE_ACC_LPF_ALPHA_DEFAULT, imu_data.acc_z);
    shared_lpf_init(&attitude_gyro_x_lpf, APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT, imu_data.gyro_x);
    shared_lpf_init(&attitude_gyro_y_lpf, APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT, imu_data.gyro_y);
    shared_lpf_init(&attitude_gyro_z_lpf, APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT, imu_data.gyro_z);

    app_attitude_copy_to_output(&imu_data);
}

void app_attitude_task(void)
{
    service_imu_data_t imu_data;

    service_imu_get_data(&imu_data);
    app_attitude_copy_to_output(&imu_data);
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
    out_data->roll = attitude_data.roll;
    out_data->pitch = attitude_data.pitch;
    out_data->yaw = attitude_data.yaw;
    out_data->acc_x = attitude_data.acc_x;
    out_data->acc_y = attitude_data.acc_y;
    out_data->acc_z = attitude_data.acc_z;
    out_data->gyro_x = attitude_data.gyro_x;
    out_data->gyro_y = attitude_data.gyro_y;
    out_data->gyro_z = attitude_data.gyro_z;
    EA = ea_backup;
}

static void app_attitude_copy_to_output(const service_imu_data_t *imu_data)
{
    uint8 ea_backup;
    app_attitude_data_t output;

    output.roll = imu_data->roll;
    output.pitch = imu_data->pitch;
    output.yaw = imu_data->yaw;
    output.acc_x = shared_lpf_update(&attitude_acc_x_lpf, imu_data->acc_x);
    output.acc_y = shared_lpf_update(&attitude_acc_y_lpf, imu_data->acc_y);
    output.acc_z = shared_lpf_update(&attitude_acc_z_lpf, imu_data->acc_z);
    output.gyro_x = shared_lpf_update(&attitude_gyro_x_lpf, imu_data->gyro_x);
    output.gyro_y = shared_lpf_update(&attitude_gyro_y_lpf, imu_data->gyro_y);
    output.gyro_z = shared_lpf_update(&attitude_gyro_z_lpf, imu_data->gyro_z);

    ea_backup = EA;
    EA = 0;
    attitude_data.roll = output.roll;
    attitude_data.pitch = output.pitch;
    attitude_data.yaw = output.yaw;
    attitude_data.acc_x = output.acc_x;
    attitude_data.acc_y = output.acc_y;
    attitude_data.acc_z = output.acc_z;
    attitude_data.gyro_x = output.gyro_x;
    attitude_data.gyro_y = output.gyro_y;
    attitude_data.gyro_z = output.gyro_z;
    EA = ea_backup;
}
