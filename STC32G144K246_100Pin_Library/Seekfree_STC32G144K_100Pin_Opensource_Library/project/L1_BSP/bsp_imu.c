#include "zf_common_headfile.h"
#include "bsp_imu.h"

#ifndef BSP_IMU_QUARTERNION_RATE
#define BSP_IMU_QUARTERNION_RATE        (IMU660RC_QUARTERNION_DISABLE)
#endif

static volatile uint8 s_bsp_imu_ready = 0U;

void bsp_imu_init(void)
{
    (void)imu660rc_init(BSP_IMU_QUARTERNION_RATE);
    s_bsp_imu_ready = 1U;
}

void bsp_imu_read_gyro(bsp_imu_gyro_t *out_data)
{
    uint8 datas[6];
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;

    if(NULL == out_data)
    {
        return;
    }

    IMU660RC_CS(0);
    spi_read_8bit_registers(IMU660RC_SPI, IMU660RC_OUTX_L_G | IMU660RC_SPI_R, datas, 6U);
    IMU660RC_CS(1);

    gyro_x = (int16)(((uint16)datas[1] << 8) | datas[0]);
    gyro_y = (int16)(((uint16)datas[3] << 8) | datas[2]);
    gyro_z = (int16)(((uint16)datas[5] << 8) | datas[4]);

    out_data->gyro_x = imu660rc_gyro_transition(gyro_x);
    out_data->gyro_y = imu660rc_gyro_transition(gyro_y);
    out_data->gyro_z = imu660rc_gyro_transition(gyro_z);
}

float bsp_imu_read_gyro_z(void)
{
    uint8 datas[2];
    int16 gyro_z;

    IMU660RC_CS(0);
    spi_read_8bit_registers(IMU660RC_SPI, IMU660RC_OUTZ_L_G | IMU660RC_SPI_R, datas, 2U);
    IMU660RC_CS(1);

    gyro_z = (int16)(((uint16)datas[1] << 8) | datas[0]);
    return imu660rc_gyro_transition(gyro_z);
}

uint8 bsp_imu_is_ready(void)
{
    return s_bsp_imu_ready;
}
