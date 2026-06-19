#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_delay.h"
#include "service_wireless_uart.h"
#include "service_imu.h"

#define SERVICE_IMU_CALIBRATE_COUNT     (500U)
#define SERVICE_IMU_CALIBRATE_INTERVAL  (10U)

static float gyro_z_offset = 0.0f;

void service_imu_init(void)
{
    bsp_imu_init();
}

void service_imu_calibrate_gyro_z(void)
{
    uint16 i;
    float sum = 0.0f;

    wprint("gz calibrating...\r\n");
    for(i = 0U; i < SERVICE_IMU_CALIBRATE_COUNT; i++)
    {
        sum += bsp_imu_read_gyro_z();
        service_delay_ms(SERVICE_IMU_CALIBRATE_INTERVAL);
    }

    gyro_z_offset = sum / (float)SERVICE_IMU_CALIBRATE_COUNT;
    wprint("gz offset=%.3f\r\n", gyro_z_offset);
}

float service_imu_read_gyro_z(void)
{
    return bsp_imu_read_gyro_z() - gyro_z_offset;
}
