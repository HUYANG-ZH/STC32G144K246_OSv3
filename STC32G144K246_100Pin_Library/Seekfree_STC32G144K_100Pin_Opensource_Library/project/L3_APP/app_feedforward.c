#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_inductor_preprocess.h"
#include "app_scheduler.h"
#include "app_feedforward.h"

#define APP_FEEDFORWARD_PACKET_SINGLE_COUNT       (1U)       // 无线变量单次注册数量
#define APP_FEEDFORWARD_TASK_PRIORITY            (5U)        // 前馈计算任务优先级
#define APP_FEEDFORWARD_DEFAULT_KFF              (2.38f)     // 默认前馈增益
#define APP_FEEDFORWARD_DEFAULT_DENOM_BIAS       (0.01f)     // 曲率分母偏置

app_feedforward_config_t app_feedforward_config =
{
    APP_FEEDFORWARD_DEFAULT_KFF,
    APP_FEEDFORWARD_DEFAULT_DENOM_BIAS
};

static volatile app_feedforward_data_t feedforward_data = {0.0f, 0.0f};

static void app_feedforward_task(void);

static void app_feedforward_register_packet(void)
{
    (void)service_packet_add_variable("feedforward_kff",
            &app_feedforward_config.kff, APP_FEEDFORWARD_PACKET_SINGLE_COUNT);
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
    app_feedforward_config.denominator_bias = APP_FEEDFORWARD_DEFAULT_DENOM_BIAS;
    feedforward_data.curvature = 0.0f;
    feedforward_data.feedforward = 0.0f;

    app_feedforward_register_packet();
    app_feedforward_task();
    (void)app_scheduler_add(APP_FEEDFORWARD_TASK_ID, app_feedforward_task,
            APP_FEEDFORWARD_TASK_PRIORITY, APP_FEEDFORWARD_PERIOD_MS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取前馈数据
// 参数说明     out_data        数据输出地址
// 返回参数     void
// 使用示例     app_feedforward_get_data(&feedforward_data);
//-------------------------------------------------------------------------------------------------------------------
void app_feedforward_get_data(app_feedforward_data_t *out_data)
{
    if(NULL == out_data)
    {
        return;
    }

    *out_data = feedforward_data;
}

static void app_feedforward_task(void)
{
    app_inductor_preprocess_data_t inductor_data;
    float numerator;
    float denominator;
    float curvature;

    app_inductor_preprocess_get_data(&inductor_data);

    numerator = tfpu_sub(inductor_data.normalized[1], inductor_data.normalized[2]);
    denominator = tfpu_add(tfpu_add(inductor_data.normalized[0], inductor_data.normalized[3]),
            app_feedforward_config.denominator_bias);
    curvature = tfpu_div(numerator, denominator);

    feedforward_data.curvature = curvature;
    feedforward_data.feedforward = tfpu_mul(app_feedforward_config.kff, curvature);
}
