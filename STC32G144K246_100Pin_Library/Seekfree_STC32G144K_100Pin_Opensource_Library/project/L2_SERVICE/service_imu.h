#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H

void service_imu_init(void);
void service_imu_calibrate_gyro_z(void);
float service_imu_read_gyro_z(void);

#endif
