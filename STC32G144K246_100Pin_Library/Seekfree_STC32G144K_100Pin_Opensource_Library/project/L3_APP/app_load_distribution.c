#include "zf_common_headfile.h"
#include "service_packet.h"
#include "app_feedforward.h"
#include "app_motion_preprocess.h"
#include "app_load_distribution.h"

#define CAR_WIDTH_M                                   (0.15f)     // 车辆宽度，单位 m
#define CAR_HEIGHT_M                                  (0.05f)     // 车辆重心高度，单位 m
#define GRAVITY_MPS2                                  (3 * 9.81f)     // 实际重力加速度，单位 m/s^2

void app_load_distribution_init(void)
{
    
}

void app_load_distribution_process(app_load_distribution_data_t *datas)
{
    datas->ay = datas->straight_current * (datas->turn_current * 0.01745f);
    datas->LTC = (datas->ay * CAR_HEIGHT_M) / (GRAVITY_MPS2 * CAR_WIDTH_M);
    // 此处极性未确定
    datas->load_left = datas->Target_differential_speed * (0.5 + datas->LTC);
    datas->load_right = datas->Target_differential_speed * (0.5 + datas->LTC);
}