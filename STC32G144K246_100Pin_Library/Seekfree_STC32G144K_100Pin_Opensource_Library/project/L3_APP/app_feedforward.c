#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_inductor_preprocess.h"
#include "app_feedforward.h"
#include "app_element.h"
#include "service_wireless_uart.h"

#define APP_FEEDFORWARD_PACKET_SINGLE_COUNT       (1U)       // 无线变量单次注册数量
#define APP_FEEDFORWARD_DEFAULT_KFF              (0.0f)   // 默认前馈增益
#define APP_FEEDFORWARD_DEFAULT_KD               (0.0f)      // 默认前馈微分增益
#define APP_FEEDFORWARD_DEFAULT_DENOM_BIAS       (0.01f)     // 曲率分母偏置
#define APP_FEEDFORWARD_OUTPUT_LIMIT             (7.0f)      // 前馈输出限幅 m/s
#define APP_FEEDFORWARD_DYNAMIC_FULL_NORM        (80.0f)     // CH1/CH2归一化强度达到该值时给满前馈
#define APP_FEEDFORWARD_DYNAMIC_FULL_NORM_INV    (0.0125f)
/* 曲率强度门控半强度点(car2抗干扰设计同步): 与 x_error 抗噪公式同构,
   直道弱信号压缩曲率噪声, 弯道强信号保持敏感; 按car3重标后量纲取32(与x_gate_s0一致)
   实车修正: x_gate_s0 已降 20 缓解弯道延迟, 前馈门控同步降 20 保持一致放开节奏 */
#define APP_FEEDFORWARD_GATE_S0_DEFAULT          (20.0f)
#define APP_FEEDFORWARD_ROUNDABOUT_RAMP_MS       (500U)      // 环岛触发前馈关闭斜坡时间
#define APP_FEEDFORWARD_ROUNDABOUT_RAMP_STEP     \
    (APP_FEEDFORWARD_PERIOD_MS / (float)APP_FEEDFORWARD_ROUNDABOUT_RAMP_MS)

app_feedforward_config_t app_feedforward_config =
{
    APP_FEEDFORWARD_DEFAULT_KFF,
    APP_FEEDFORWARD_DEFAULT_KD,
    APP_FEEDFORWARD_DEFAULT_DENOM_BIAS
};

static volatile app_feedforward_data_t feedforward_data = {0.0f, 0.0f, 0.0f};
static float feedforward_last_curvature = 0.0f;
static uint8 feedforward_rate_ready = 0U;
static float feedforward_scale = 1.0f;
static float feedforward_dt_inv = 0.0f;
static volatile float feedforward_gate_s0 = APP_FEEDFORWARD_GATE_S0_DEFAULT;

void app_feedforward_control_step(void);

static void app_feedforward_register_packet(void)
{
    (void)service_packet_add_variable("feedforward_kff",
            &app_feedforward_config.kff, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("feedforward_kd",
            &app_feedforward_config.kd, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("ff_gate_s0",
            (float *)&feedforward_gate_s0, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     前馈模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_feedforward_init();
// 备注信息     使用 APP_FEEDFORWARD_PERIOD_MS ms 周期任务更新一次
//-------------------------------------------------------------------------------------------------------------------
void app_feedforward_init(void)
{
    app_feedforward_config.kff = APP_FEEDFORWARD_DEFAULT_KFF;
    app_feedforward_config.kd = APP_FEEDFORWARD_DEFAULT_KD;
    app_feedforward_config.denominator_bias = APP_FEEDFORWARD_DEFAULT_DENOM_BIAS;
    feedforward_data.curvature = 0.0f;
    feedforward_data.curvature_rate = 0.0f;
    feedforward_data.feedforward = 0.0f;
    feedforward_last_curvature = 0.0f;
    feedforward_rate_ready = 0U;
    feedforward_scale = 1.0f;
    feedforward_dt_inv = tfpu_div(1000.0f,
            tfpu_int2float((long)APP_FEEDFORWARD_PERIOD_MS));

    app_feedforward_register_packet();
    app_feedforward_control_step();
    /* 周期执行由同一条 TIM6 控制链统一排序，避免跨定时器的数据年龄不确定。 */
    #if __DBGFLAG__
    printf(">>[app_feedforward_init]\r\n");
    wprint(">>[app_feedforward_init]\r\n");
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取前馈数据
// 参数说明     out_data        数据输出地址
// 返回参数     void
// 使用示例     app_feedforward_get_data(&feedforward_data);
//-------------------------------------------------------------------------------------------------------------------
void app_feedforward_get_data(app_feedforward_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    *out_data = feedforward_data;
    EA = ea_backup;
}

void app_feedforward_control_step(void)
{
    app_inductor_preprocess_data_t inductor_data;
    float numerator;
    float denominator;
    float curvature;
    float curvature_rate;
    float strength;
    float strength_ratio;
    float dynamic_kff;
    float feedforward_p;
    float feedforward_d;

    app_inductor_preprocess_get_data(&inductor_data);

    numerator = tfpu_sub(inductor_data.normalized[1], inductor_data.normalized[2]);
    denominator = tfpu_add(tfpu_add(inductor_data.normalized[0], inductor_data.normalized[3]),
            app_feedforward_config.denominator_bias);
    curvature = tfpu_div(numerator, denominator);

    /* 曲率强度门控(car2抗干扰设计同步): 直道弱信号压缩曲率噪声, 弯道强信号保持敏感 */
    {
        float x_strength = (inductor_data.normalized[1] > inductor_data.normalized[2]) ?
                inductor_data.normalized[1] : inductor_data.normalized[2];
        float x_gate = tfpu_div(x_strength, tfpu_add(x_strength, feedforward_gate_s0));
        curvature = tfpu_mul(curvature, x_gate);
    }

    if(0U == feedforward_rate_ready)
    {
        curvature_rate = 0.0f;
        feedforward_rate_ready = 1U;
    }
    else
    {
        curvature_rate = tfpu_mul(tfpu_sub(curvature, feedforward_last_curvature),
                feedforward_dt_inv);
    }
    feedforward_last_curvature = curvature;

    strength = (inductor_data.normalized[1] > inductor_data.normalized[2]) ?
            inductor_data.normalized[1] : inductor_data.normalized[2];
    strength_ratio = tfpu_mul(strength, APP_FEEDFORWARD_DYNAMIC_FULL_NORM_INV);
    if(0.0f > strength_ratio)
    {
        strength_ratio = 0.0f;
    }
    else if(1.0f < strength_ratio)
    {
        strength_ratio = 1.0f;
    }

    dynamic_kff = tfpu_mul(app_feedforward_config.kff, strength_ratio);
    feedforward_p = tfpu_mul(dynamic_kff, curvature);
    feedforward_d = tfpu_mul(app_feedforward_config.kd, curvature_rate);

    {
        app_element_data_t element;
        app_element_get_data(&element);

        if((APP_ELEMENT_TYPE_CYLINDER == element.type) && (element.active >= 0.5f))
        {
            feedforward_scale = 0.0f;
        }
        else if(app_element_roundabout_feedforward_scale > 1.0f)
        {
            feedforward_scale = app_element_roundabout_feedforward_scale;
        }
        else
        {
            feedforward_scale += APP_FEEDFORWARD_ROUNDABOUT_RAMP_STEP;
            if(feedforward_scale > 1.0f)
            {
                feedforward_scale = 1.0f;
            }
        }
    }

    feedforward_data.curvature = curvature;
    feedforward_data.curvature_rate = curvature_rate;
    feedforward_data.feedforward = tfpu_mul(tfpu_add(feedforward_p, feedforward_d), feedforward_scale);

    if(feedforward_data.feedforward > APP_FEEDFORWARD_OUTPUT_LIMIT)
    {
        feedforward_data.feedforward = APP_FEEDFORWARD_OUTPUT_LIMIT;
    }
    else if(feedforward_data.feedforward < -APP_FEEDFORWARD_OUTPUT_LIMIT)
    {
        feedforward_data.feedforward = -APP_FEEDFORWARD_OUTPUT_LIMIT;
    }
}
