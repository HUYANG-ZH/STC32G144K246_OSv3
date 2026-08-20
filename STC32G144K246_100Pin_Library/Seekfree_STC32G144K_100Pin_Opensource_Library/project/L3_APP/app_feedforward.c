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
#define APP_FEEDFORWARD_OUTPUT_LIMIT             (5.0f)      // 前馈输出限幅 m/s
#define APP_FEEDFORWARD_GATE_ON_DEFAULT           (0.030f)
#define APP_FEEDFORWARD_GATE_OFF_DEFAULT          (0.018f)
#define APP_FEEDFORWARD_CURVATURE_LPF_ALPHA       (0.55f)
#define APP_FEEDFORWARD_PREVIEW_MS_DEFAULT        (15.0f)
#define APP_FEEDFORWARD_PREVIEW_LIMIT_DEFAULT     (0.120f)
#define APP_FEEDFORWARD_GATE_ENTER_CONFIRM        (1U)
#define APP_FEEDFORWARD_GATE_EXIT_CONFIRM         (2U)
#define APP_FEEDFORWARD_RISE_RATE_MPS2            (200.0f)
#define APP_FEEDFORWARD_FALL_RATE_MPS2            (250.0f)
#define APP_FEEDFORWARD_ROUNDABOUT_RAMP_MS       (50U)      // 环岛触发前馈关闭斜坡时间
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
static float feedforward_output = 0.0f;
static uint8 feedforward_gate_active = 0U;
static uint8 feedforward_gate_enter_count = 0U;
static uint8 feedforward_gate_exit_count = 0U;
static volatile float feedforward_gate_on = APP_FEEDFORWARD_GATE_ON_DEFAULT;
static volatile float feedforward_gate_off = APP_FEEDFORWARD_GATE_OFF_DEFAULT;
static volatile float feedforward_curvature_lpf_alpha = APP_FEEDFORWARD_CURVATURE_LPF_ALPHA;
static volatile float feedforward_preview_ms = APP_FEEDFORWARD_PREVIEW_MS_DEFAULT;
static volatile float feedforward_preview_limit = APP_FEEDFORWARD_PREVIEW_LIMIT_DEFAULT;

void app_feedforward_control_step(void);

static float app_feedforward_abs(float value)
{
    return (value < 0.0f) ? tfpu_sub(0.0f, value) : value;
}

static float app_feedforward_limit(float value, float min, float max)
{
    if(value < min)
    {
        return min;
    }
    if(value > max)
    {
        return max;
    }
    return value;
}

static uint8 app_feedforward_update_gate(float curvature)
{
    float on_threshold;
    float off_threshold;
    float abs_curvature;

    on_threshold = feedforward_gate_on;
    if(on_threshold <= 0.0f)
    {
        on_threshold = APP_FEEDFORWARD_GATE_ON_DEFAULT;
    }
    off_threshold = feedforward_gate_off;
    if(off_threshold < 0.0f)
    {
        off_threshold = 0.0f;
    }
    if(off_threshold >= on_threshold)
    {
        off_threshold = tfpu_mul(on_threshold, 0.6f);
    }

    abs_curvature = app_feedforward_abs(curvature);
    if(0U == feedforward_gate_active)
    {
        feedforward_gate_exit_count = 0U;
        if(abs_curvature >= on_threshold)
        {
            feedforward_gate_enter_count++;
            if(feedforward_gate_enter_count >= APP_FEEDFORWARD_GATE_ENTER_CONFIRM)
            {
                feedforward_gate_active = 1U;
                feedforward_gate_enter_count = 0U;
            }
        }
        else
        {
            feedforward_gate_enter_count = 0U;
        }
    }
    else
    {
        feedforward_gate_enter_count = 0U;
        if(abs_curvature <= off_threshold)
        {
            feedforward_gate_exit_count++;
            if(feedforward_gate_exit_count >= APP_FEEDFORWARD_GATE_EXIT_CONFIRM)
            {
                feedforward_gate_active = 0U;
                feedforward_gate_exit_count = 0U;
            }
        }
        else
        {
            feedforward_gate_exit_count = 0U;
        }
    }

    return feedforward_gate_active;
}

static float app_feedforward_ramp(float current, float target)
{
    float delta;
    float max_delta;

    delta = tfpu_sub(target, current);
    max_delta = tfpu_mul((delta >= 0.0f) ? APP_FEEDFORWARD_RISE_RATE_MPS2 :
            APP_FEEDFORWARD_FALL_RATE_MPS2,
            (float)APP_FEEDFORWARD_PERIOD_MS / 1000.0f);
    return tfpu_add(current, app_feedforward_limit(delta, -max_delta, max_delta));
}

static void app_feedforward_register_packet(void)
{
    (void)service_packet_add_variable("feedforward_kff",
            &app_feedforward_config.kff, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("feedforward_kd",
            &app_feedforward_config.kd, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("ff_gate_on",
            (float *)&feedforward_gate_on, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("ff_gate_off",
            (float *)&feedforward_gate_off, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("ff_curve_alpha",
            (float *)&feedforward_curvature_lpf_alpha, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("ff_preview_ms",
            (float *)&feedforward_preview_ms, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("ff_preview_lim",
            (float *)&feedforward_preview_limit, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
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
    feedforward_output = 0.0f;
    feedforward_gate_active = 0U;
    feedforward_gate_enter_count = 0U;
    feedforward_gate_exit_count = 0U;
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
    float curvature_raw;
    float curvature;
    float curvature_rate;
    float curvature_predict;
    float curvature_alpha;
    float preview_term;
    float preview_ms;
    float preview_limit;
    float feedforward_p;
    float feedforward_d;
    float feedforward_target;

    app_inductor_preprocess_get_data(&inductor_data);

    numerator = tfpu_sub(inductor_data.normalized[1], inductor_data.normalized[2]);
    denominator = tfpu_add(tfpu_add(inductor_data.normalized[0], inductor_data.normalized[3]),
            app_feedforward_config.denominator_bias);
    curvature_raw = tfpu_div(numerator, denominator);

    /* Light filtering only feeds the gate and derivative.  Once the gate is
       open, Kff is not attenuated by signal strength. */
    curvature_alpha = app_feedforward_limit(feedforward_curvature_lpf_alpha, 0.05f, 1.0f);
    curvature = tfpu_add(feedforward_last_curvature,
            tfpu_mul(curvature_alpha, tfpu_sub(curvature_raw, feedforward_last_curvature)));

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

    preview_ms = app_feedforward_limit(feedforward_preview_ms, 0.0f, 100.0f);
    preview_limit = app_feedforward_limit(feedforward_preview_limit, 0.0f, 0.5f);
    preview_term = tfpu_mul(curvature_rate, tfpu_mul(preview_ms, 0.001f));
    preview_term = app_feedforward_limit(preview_term, -preview_limit, preview_limit);
    curvature_predict = tfpu_add(curvature, preview_term);

    feedforward_p = tfpu_mul(app_feedforward_config.kff, curvature_predict);
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

    if(0U != app_feedforward_update_gate(curvature))
    {
        feedforward_target = tfpu_mul(tfpu_add(feedforward_p, feedforward_d), feedforward_scale);
    }
    else
    {
        feedforward_target = 0.0f;
    }
    feedforward_target = app_feedforward_limit(feedforward_target,
            -APP_FEEDFORWARD_OUTPUT_LIMIT, APP_FEEDFORWARD_OUTPUT_LIMIT);

    if(feedforward_scale <= 0.0f)
    {
        feedforward_output = 0.0f;
    }
    else
    {
        feedforward_output = app_feedforward_ramp(feedforward_output, feedforward_target);
    }

    feedforward_data.curvature = curvature;
    feedforward_data.curvature_rate = curvature_rate;
    feedforward_data.feedforward = feedforward_output;
}
