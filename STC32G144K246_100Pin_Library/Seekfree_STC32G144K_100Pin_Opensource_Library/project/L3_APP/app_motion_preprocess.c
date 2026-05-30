#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_inductor_preprocess.h"
#include "app_motion_preprocess.h"

#define APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT      (1U)        // 无线变量单次注册数量
#define APP_MOTION_PREPROCESS_X_WEIGHT                 (0.3f)      // x轴差比融合权重
#define APP_MOTION_PREPROCESS_Y_WEIGHT                 (0.7f)      // y轴差比融合权重
#define APP_MOTION_PREPROCESS_SUM_MIN                  (0.001f)    // 差比计算分母最小值
#define APP_MOTION_PREPROCESS_DEFAULT_LINEAR_MPS       (0.0f)      // 默认直线速度，单位 m/s
#define APP_MOTION_PREPROCESS_DEFAULT_YAW_RATE_GAIN    (1.0f)      // 默认角速度增益，单位 rad/s

app_motion_preprocess_config_t app_motion_preprocess_config =
{
    APP_MOTION_PREPROCESS_DEFAULT_LINEAR_MPS,
    APP_MOTION_PREPROCESS_DEFAULT_YAW_RATE_GAIN
};

static volatile app_motion_preprocess_data_t motion_preprocess_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static void app_motion_preprocess_tick(void);

static float app_motion_preprocess_diff_ratio(float left_value, float right_value)
{
    float sum;

    sum = tfpu_add(left_value, right_value);
    if(APP_MOTION_PREPROCESS_SUM_MIN >= sum)
    {
        return 0.0f;
    }

    return tfpu_div(tfpu_sub(left_value, right_value), sum);
}

static void app_motion_preprocess_register_packet(void)
{
    (void)service_packet_add_variable("motion_pre_linear_mps",
            &app_motion_preprocess_config.linear_mps, APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("motion_pre_yaw_rate_gain",
            &app_motion_preprocess_config.yaw_rate_gain, APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运动前处理初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_motion_preprocess_init();
// 备注信息     使用 APP_MOTION_PREPROCESS_PIT 每 APP_MOTION_PREPROCESS_PERIOD_MS ms 更新一次
//-------------------------------------------------------------------------------------------------------------------
void app_motion_preprocess_init(void)
{
    app_motion_preprocess_register_packet();
    app_motion_preprocess_tick();
    pit_ms_init(APP_MOTION_PREPROCESS_PIT, APP_MOTION_PREPROCESS_PERIOD_MS, app_motion_preprocess_tick);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取运动前处理数据
// 参数说明     out_data        数据输出地址
// 返回参数     void
// 使用示例     app_motion_preprocess_get_data(&motion_data);
//-------------------------------------------------------------------------------------------------------------------
void app_motion_preprocess_get_data(app_motion_preprocess_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    out_data->x_error = motion_preprocess_data.x_error;
    out_data->y_error = motion_preprocess_data.y_error;
    out_data->line_error = motion_preprocess_data.line_error;
    out_data->linear_mps = motion_preprocess_data.linear_mps;
    out_data->target_yaw_rate_rps = motion_preprocess_data.target_yaw_rate_rps;
    EA = ea_backup;
}

static void app_motion_preprocess_tick(void)
{
    app_inductor_preprocess_data_t inductor_data;
    app_motion_preprocess_data_t output;

    app_inductor_preprocess_get_data(&inductor_data);

    output.y_error = app_motion_preprocess_diff_ratio(inductor_data.normalized[0], inductor_data.normalized[3]);
    output.x_error = app_motion_preprocess_diff_ratio(inductor_data.normalized[1], inductor_data.normalized[2]);
    output.line_error = tfpu_add(tfpu_mul(output.y_error, APP_MOTION_PREPROCESS_Y_WEIGHT),
            tfpu_mul(output.x_error, APP_MOTION_PREPROCESS_X_WEIGHT));
    output.linear_mps = app_motion_preprocess_config.linear_mps;
    output.target_yaw_rate_rps = tfpu_mul(app_motion_preprocess_config.yaw_rate_gain, output.line_error);

    motion_preprocess_data.x_error = output.x_error;
    motion_preprocess_data.y_error = output.y_error;
    motion_preprocess_data.line_error = output.line_error;
    motion_preprocess_data.linear_mps = output.linear_mps;
    motion_preprocess_data.target_yaw_rate_rps = output.target_yaw_rate_rps;
}
