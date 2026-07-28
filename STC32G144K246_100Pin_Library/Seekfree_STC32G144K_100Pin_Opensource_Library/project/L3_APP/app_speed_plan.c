#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_feedforward.h"
#include "app_motion_preprocess.h"
#include "app_speed_plan.h"
#include "app_element.h"
#include "service_wireless_uart.h"

#define APP_SPEED_PLAN_PACKET_SINGLE_COUNT      (1U)
#define APP_SPEED_PLAN_DEFAULT_MIN_RATIO        (0.9f)
#define APP_SPEED_PLAN_ACCEL_LIMIT_MPS2         (80.0f)
#define APP_SPEED_PLAN_DECEL_LIMIT_MPS2         (80.0f)
#define APP_SPEED_PLAN_ERROR_MAX                (1.0f)
#define APP_SPEED_PLAN_CURVATURE_MAX            (1.0f)

static float speed_plan_min_ratio = APP_SPEED_PLAN_DEFAULT_MIN_RATIO;
static volatile float speed_plan_linear_mps = 0.0f;

void app_speed_plan_control_step(void);

static float app_speed_plan_abs(float value)
{
    return (0.0f > value) ? tfpu_sub(0.0f, value) : value;
}

static float app_speed_plan_limit(float value, float min, float max)
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

static float app_speed_plan_max(float a, float b)
{
    return (a > b) ? a : b;
}

static float app_speed_plan_norm_abs(float value, float max_value)
{
    float abs_value;

    if(0.0f >= max_value)
    {
        return 0.0f;
    }

    abs_value = app_speed_plan_abs(value);
    abs_value = app_speed_plan_limit(abs_value, 0.0f, max_value);

    if(1.0f == max_value)
    {
        return abs_value;
    }

    return tfpu_div(abs_value, max_value);
}

static float app_speed_plan_ramp(float current, float target)
{
    float delta;
    float limit;

    delta = tfpu_sub(target, current);
    limit = (0.0f < delta) ?
            tfpu_mul(APP_SPEED_PLAN_ACCEL_LIMIT_MPS2, (float)APP_SPEED_PLAN_PERIOD_MS / 1000.0f) :
            tfpu_mul(APP_SPEED_PLAN_DECEL_LIMIT_MPS2, (float)APP_SPEED_PLAN_PERIOD_MS / 1000.0f);

    if(delta > limit)
    {
        delta = limit;
    }
    else if(delta < tfpu_sub(0.0f, limit))
    {
        delta = tfpu_sub(0.0f, limit);
    }

    return tfpu_add(current, delta);
}

static void app_speed_plan_register_packet(void)
{
    (void)service_packet_add_variable("speed_plan_min_ratio",
            &speed_plan_min_ratio, APP_SPEED_PLAN_PACKET_SINGLE_COUNT);
}

void app_speed_plan_init(void)
{
    app_motion_preprocess_data_t motion_preprocess;

    app_motion_preprocess_get_data(&motion_preprocess);
    speed_plan_min_ratio = APP_SPEED_PLAN_DEFAULT_MIN_RATIO;
    speed_plan_linear_mps = motion_preprocess.linear_mps;

    app_speed_plan_register_packet();
    app_speed_plan_control_step();
    /* 周期执行由同一条 TIM6 控制链统一排序，避免跨定时器的数据年龄不确定。 */
    #if __DBGFLAG__
    printf(">>[app_speed_plan_init]\r\n");
    wprint(">>[app_speed_plan_init]\r\n");
    #endif
}

float app_speed_plan_get_linear_mps(void)
{
    return speed_plan_linear_mps;
}

void app_speed_plan_control_step(void)
{
    float line_error_rate;
    float curvature_rate;
    float brake_rate;
    float ratio;
    float target_raw_mps;
    app_motion_preprocess_data_t motion_preprocess;
    app_feedforward_data_t feedforward;

    app_motion_preprocess_get_data(&motion_preprocess);
    app_feedforward_get_data(&feedforward);

    line_error_rate = app_speed_plan_norm_abs(motion_preprocess.line_error, APP_SPEED_PLAN_ERROR_MAX);
    curvature_rate = app_speed_plan_norm_abs(feedforward.curvature, APP_SPEED_PLAN_CURVATURE_MAX);
    brake_rate = app_speed_plan_max(line_error_rate, curvature_rate);
    ratio = app_speed_plan_limit(speed_plan_min_ratio, 0.0f, 1.0f);
    target_raw_mps = tfpu_mul(motion_preprocess.linear_mps,
            tfpu_sub(1.0f, tfpu_mul(tfpu_sub(1.0f, ratio), brake_rate)));

    /* 跷跷板触发后立即降速到 0.8m/s，持续 160ms，跳过 ramp 实现硬立即 */
    if(0U != app_element_seesaw_is_slowdown_active())
    {
        speed_plan_linear_mps = app_element_seesaw_slowdown_speed_mps();
    }
    else
    {
        speed_plan_linear_mps = app_speed_plan_ramp(speed_plan_linear_mps, target_raw_mps);
    }
}
