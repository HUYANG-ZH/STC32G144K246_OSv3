#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_imu.h"

void service_imu_init(void)
{
    bsp_imu_init();
}

float service_imu_read_gyro_z(void)
{
    return bsp_imu_read_gyro_z();
}
