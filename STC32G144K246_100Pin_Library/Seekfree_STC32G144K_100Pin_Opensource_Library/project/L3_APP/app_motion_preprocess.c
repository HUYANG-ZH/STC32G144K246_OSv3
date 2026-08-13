#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_inductor_preprocess.h"
#include "app_motion_preprocess.h"
#include "service_wireless_uart.h"

#define APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT      (1U)        // 无线变量单次注册数量
#define APP_MOTION_PREPROCESS_X_WEIGHT                 (0.25f)     // x轴差比融合权重
#define APP_MOTION_PREPROCESS_Y_WEIGHT                 (1.0f)      // y轴差比融合权重
#define APP_MOTION_PREPROCESS_SUM_MIN                  (0.001f)    // 差比计算分母最小值
#define APP_MOTION_PREPROCESS_DEFAULT_LINEAR_MPS       (0.0f)      // 默认直线速度，单位 m/s
#define APP_MOTION_PREPROCESS_DEFAULT_YAW_RATE_GAIN    (0.62f)     // 默认角速度增益，单位 rad/s
/* x 轴(前进方向)电感差比的抗噪参数(直道弱信号防噪声放大导致车体震颤):
   公式: x_error = (CH2-CH3)/(CH2+CH3+B) * max(CH2,CH3)/(max(CH2,CH3)+S0)
   B    = 分母偏置: 直道信号和的一半左右
   S0   = 强度门控半强度点: 直道单路强度的 2 倍, 弱信号压增益、强信号(弯道)保持敏感
   car2 量纲(0~100): B=8, S0=16, 经 200 组参数网格仿真直道降噪约 4.5x、出线约 19x
   car3 量纲(0~300 软上限, MIN 标定高): 经 6 场景 x 81 组 = 486 组网格仿真交叉验证,
   最优平衡点 B=24, S0=40(补偿后直道 std 全场景最小, 弯道压缩可用 xw 0.25->0.35 补偿) */
#define APP_MOTION_PREPROCESS_X_DEN_BIAS_DEFAULT      (24.0f)     // 差比分母偏置
#define APP_MOTION_PREPROCESS_X_GATE_S0_DEFAULT       (40.0f)     // 门控半强度点

app_motion_preprocess_config_t app_motion_preprocess_config =
{
    APP_MOTION_PREPROCESS_DEFAULT_LINEAR_MPS,
    APP_MOTION_PREPROCESS_DEFAULT_YAW_RATE_GAIN
};

static volatile app_motion_preprocess_data_t motion_preprocess_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* 差比融合权重无线可调, 宏定义值仅作默认 */
static volatile float motion_pre_x_weight = APP_MOTION_PREPROCESS_X_WEIGHT;
static volatile float motion_pre_y_weight = APP_MOTION_PREPROCESS_Y_WEIGHT;
/* x 轴差比抗噪参数无线可调, 宏定义值仅作默认 */
static volatile float motion_pre_x_den_bias = APP_MOTION_PREPROCESS_X_DEN_BIAS_DEFAULT;
static volatile float motion_pre_x_gate_s0 = APP_MOTION_PREPROCESS_X_GATE_S0_DEFAULT;

void app_motion_preprocess_control_step(void);

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
    (void)service_packet_add_variable("xw",
            (float *)&motion_pre_x_weight, APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("yw",
            (float *)&motion_pre_y_weight, APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("x_den_b",
            (float *)&motion_pre_x_den_bias, APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("x_gate_s0",
            (float *)&motion_pre_x_gate_s0, APP_MOTION_PREPROCESS_PACKET_SINGLE_COUNT);
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
    /* 首次仅发布当前快照；周期调度统一由 app_motion_postprocess 的 TIM6 控制链承担。 */
    app_motion_preprocess_control_step();
    #if __DBGFLAG__
    printf(">>[app_motion_preprocess_init]\r\n");
    wprint(">>[app_motion_preprocess_init]\r\n");
    #endif
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

void app_motion_preprocess_control_step(void)
{
    app_inductor_preprocess_data_t inductor_data;
    app_motion_preprocess_data_t output;

    app_inductor_preprocess_get_data(&inductor_data);

    output.y_error = app_motion_preprocess_diff_ratio(inductor_data.normalized[0], inductor_data.normalized[3]);
    /* x 轴(前进方向)差比抗噪公式:
       x_error = (CH2-CH3)/(CH2+CH3+x_den_b) * max(CH2,CH3)/(max(CH2,CH3)+x_gate_s0)
       直道弱信号时: 分母偏置防小分母放大 + 强度门控把增益压到接近 0 -> 噪声不进控制链
       弯道强信号时: 门控增益 -> 1, 偏置相对大分母可忽略 -> 差比保持敏感 */
    {
        float x_strength;
        float x_gate;
        float x_raw;

        x_strength = (inductor_data.normalized[1] > inductor_data.normalized[2]) ?
                inductor_data.normalized[1] : inductor_data.normalized[2];
        x_gate = tfpu_div(x_strength,
                tfpu_add(x_strength, motion_pre_x_gate_s0));
        x_raw = tfpu_div(tfpu_sub(inductor_data.normalized[1], inductor_data.normalized[2]),
                tfpu_add(tfpu_add(inductor_data.normalized[1], inductor_data.normalized[2]),
                motion_pre_x_den_bias));
        output.x_error = tfpu_mul(x_raw, x_gate);
    }

    output.line_error = tfpu_add(tfpu_mul(output.y_error, motion_pre_y_weight),
            tfpu_mul(output.x_error, motion_pre_x_weight));
    output.linear_mps = app_motion_preprocess_config.linear_mps;
    output.target_yaw_rate_rps = tfpu_mul(app_motion_preprocess_config.yaw_rate_gain, output.line_error);

    motion_preprocess_data.x_error = output.x_error;
    motion_preprocess_data.y_error = output.y_error;
    motion_preprocess_data.line_error = output.line_error;
    motion_preprocess_data.linear_mps = output.linear_mps;
    motion_preprocess_data.target_yaw_rate_rps = output.target_yaw_rate_rps;
}
