#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "bsp_include.h"
#include "service_delay.h"
#include "service_wireless_uart.h"
#include "service_imu.h"

#define SERVICE_IMU_CALIBRATE_COUNT     (200U)
#define SERVICE_IMU_CALIBRATE_INTERVAL  (10U)

static float gyro_x_offset = 0.0f;
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

void service_imu_calibrate_gyro_x(void)
{
    uint16 i;
    float sum = 0.0f;
    bsp_imu_gyro_t raw;

    wprint("gx calibrating...\r\n");
    for(i = 0U; i < SERVICE_IMU_CALIBRATE_COUNT; i++)
    {
        bsp_imu_read_gyro(&raw);
        sum += raw.gyro_x;
        service_delay_ms(SERVICE_IMU_CALIBRATE_INTERVAL);
    }

    gyro_x_offset = sum / (float)SERVICE_IMU_CALIBRATE_COUNT;
    wprint("gx offset=%.3f\r\n", gyro_x_offset);
}

void service_imu_read_gyro(service_imu_gyro_t *out_data)
{
    bsp_imu_gyro_t raw;

    if(NULL == out_data)
    {
        return;
    }

    bsp_imu_read_gyro(&raw);
    out_data->gyro_x = tfpu_sub(raw.gyro_x, gyro_x_offset);
    out_data->gyro_y = raw.gyro_y;
    out_data->gyro_z = tfpu_sub(raw.gyro_z, gyro_z_offset);
}

float service_imu_read_gyro_z(void)
{
    return tfpu_sub(bsp_imu_read_gyro_z(), gyro_z_offset);
}
