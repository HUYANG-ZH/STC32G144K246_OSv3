#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} service_imu_gyro_t;

void service_imu_init(void);
void service_imu_calibrate_gyro_z(void);
void service_imu_read_gyro(service_imu_gyro_t *out_data);
float service_imu_read_gyro_z(void);

#endif
