#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_imu.h"

void service_imu_init(void)
{
    bsp_imu_init();
}

uint8 service_imu_task(void)
{
    return bsp_imu_task();
}

void service_imu_debug(void)
{
    service_imu_data_t testdata;
    printf("[imu:get]\r\n");
    service_imu_get_data(&testdata);
    printf("[imu:gx/gy/gz] %f,%f,%f\r\n", testdata.gyro_x,testdata.gyro_y,testdata.gyro_z);
}

void service_imu_get_data(service_imu_data_t *out_imu)
{
    imu_data_t bsp_data;

    if(NULL == out_imu)
    {
        return;
    }

    bsp_imu_read(&bsp_data);

    out_imu->acc_x = bsp_data.imu_acc_x;
    out_imu->acc_y = bsp_data.imu_acc_y;
    out_imu->acc_z = bsp_data.imu_acc_z;
    out_imu->gyro_x = bsp_data.imu_gyro_x;
    out_imu->gyro_y = bsp_data.imu_gyro_y;
    out_imu->gyro_z = bsp_data.imu_gyro_z;
    out_imu->quat_x = bsp_data.imu_quat_x;
    out_imu->quat_y = bsp_data.imu_quat_y;
    out_imu->quat_z = bsp_data.imu_quat_z;
    out_imu->quat_w = bsp_data.imu_quat_w;
    out_imu->roll = bsp_data.imu_roll_deg;
    out_imu->pitch = bsp_data.imu_pitch_deg;
    out_imu->yaw = bsp_data.imu_yaw_deg;
}
