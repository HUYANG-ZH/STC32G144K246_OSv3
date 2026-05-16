#include "zf_common_headfile.h"
#include "bsp_imu.h"

static volatile uint8 s_bsp_imu_ready = 0U;

void bsp_imu_init(void)
{
    imu660rc_init(IMU660RC_QUARTERNION_480HZ);
    s_bsp_imu_ready = 1U;
}

void bsp_imu_debug(void)
{
}

void bsp_imu_read(imu_data_t *out_data)
{
    uint8 ea_backup;
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;

    ea_backup = EA;
    EA = 0;
    acc_x = imu660rc_acc_x;
    acc_y = imu660rc_acc_y;
    acc_z = imu660rc_acc_z;
    gyro_x = imu660rc_gyro_x;
    gyro_y = imu660rc_gyro_y;
    gyro_z = imu660rc_gyro_z;
    out_data->imu_quat_x = imu660rc_quarternion[0];
    out_data->imu_quat_y = imu660rc_quarternion[1];
    out_data->imu_quat_z = imu660rc_quarternion[2];
    out_data->imu_quat_w = imu660rc_quarternion[3];
    out_data->imu_roll_deg = imu660rc_roll;
    out_data->imu_pitch_deg = imu660rc_pitch;
    out_data->imu_yaw_deg = imu660rc_yaw;
    EA = ea_backup;

    out_data->imu_acc_x = imu660rc_acc_transition(acc_x);
    out_data->imu_acc_y = imu660rc_acc_transition(acc_y);
    out_data->imu_acc_z = imu660rc_acc_transition(acc_z);
    out_data->imu_gyro_x = imu660rc_gyro_transition(gyro_x);
    out_data->imu_gyro_y = imu660rc_gyro_transition(gyro_y);
    out_data->imu_gyro_z = imu660rc_gyro_transition(gyro_z);
    bsp_imu_get_temperature(&out_data->imu_temperture);
}

void bsp_imu_get_temperature(float *temperature)
{
    uint8 datas[2];
    int16 raw_temperature;

    IMU660RC_CS(0);
    spi_read_8bit_registers(IMU660RC_SPI, IMU660RC_OUT_TEMP_L | IMU660RC_SPI_R, datas, 2U);
    IMU660RC_CS(1);
    raw_temperature = (int16)(((uint16)datas[1] << 8) | datas[0]);
    *temperature = ((float)raw_temperature / 256.0f) + 25.0f;
}

uint8 bsp_imu_is_ready(void)
{
    return s_bsp_imu_ready;
}
