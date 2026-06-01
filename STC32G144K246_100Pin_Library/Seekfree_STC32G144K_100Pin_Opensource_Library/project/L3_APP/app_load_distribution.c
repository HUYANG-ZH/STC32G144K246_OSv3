#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "app_load_distribution.h"

#define APP_LOAD_DISTRIBUTION_CAR_WIDTH_M            (0.15f)       // 车辆宽度，单位 m
#define APP_LOAD_DISTRIBUTION_CAR_HEIGHT_M           (0.05f)       // 车辆重心高度，单位 m
#define APP_LOAD_DISTRIBUTION_GRAVITY_MPS2           (29.43f)      // 实际重力加速度，单位 m/s^2
#define APP_LOAD_DISTRIBUTION_LTC_LIMIT              (0.5f)        // 荷载转移比例限幅

static float app_load_distribution_limit_ltc(float ltc)
{
    float ltc_min;

    if(ltc > APP_LOAD_DISTRIBUTION_LTC_LIMIT)
    {
        return APP_LOAD_DISTRIBUTION_LTC_LIMIT;
    }

    ltc_min = tfpu_sub(0.0f, APP_LOAD_DISTRIBUTION_LTC_LIMIT);
    if(ltc < ltc_min)
    {
        return ltc_min;
    }

    return ltc;
}

void app_load_distribution_init(void)
{
}

void app_load_distribution_process(app_load_distribution_data_t *datas)
{
    float ltc_denominator;
    float left_ratio;
    float right_ratio;

    if(NULL == datas)
    {
        return;
    }

    // turn_current unit is rad/s, converted once in app_motion_postprocess.
    datas->ay = tfpu_mul(datas->straight_current, datas->turn_current);

    ltc_denominator = tfpu_mul(APP_LOAD_DISTRIBUTION_GRAVITY_MPS2, APP_LOAD_DISTRIBUTION_CAR_WIDTH_M);
    datas->LTC = tfpu_div(tfpu_mul(datas->ay, APP_LOAD_DISTRIBUTION_CAR_HEIGHT_M), ltc_denominator);
    datas->LTC = app_load_distribution_limit_ltc(datas->LTC);

    left_ratio = tfpu_sub(0.5f, datas->LTC);
    right_ratio = tfpu_add(0.5f, datas->LTC);

    datas->load_left = tfpu_mul(datas->Target_differential_speed, left_ratio);
    datas->load_right = tfpu_mul(datas->Target_differential_speed, right_ratio);
}
