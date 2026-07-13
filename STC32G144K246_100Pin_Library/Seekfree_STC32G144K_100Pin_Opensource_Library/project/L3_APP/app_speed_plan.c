#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_feedforward.h"
#include "app_motion_preprocess.h"
#include "app_scheduler.h"
#include "app_speed_plan.h"
#include "app_element.h"

#define APP_SPEED_PLAN_PACKET_SINGLE_COUNT      (1U)
#define APP_SPEED_PLAN_TASK_PRIORITY            (6U)
#define APP_SPEED_PLAN_DEFAULT_MIN_RATIO        (1.0f)
#define APP_SPEED_PLAN_ACCEL_LIMIT_MPS2         (80.0f)
#define APP_SPEED_PLAN_DECEL_LIMIT_MPS2         (80.0f)
#define APP_SPEED_PLAN_ERROR_MAX                (1.0f)
#define APP_SPEED_PLAN_CURVATURE_MAX            (1.0f)

static float speed_plan_min_ratio = APP_SPEED_PLAN_DEFAULT_MIN_RATIO;
static volatile float speed_plan_linear_mps = 0.0f;

static void app_speed_plan_task(void);

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
    app_speed_plan_task();
    (void)app_scheduler_add(APP_SPEED_PLAN_TASK_ID, app_speed_plan_task,
            APP_SPEED_PLAN_TASK_PRIORITY, APP_SPEED_PLAN_PERIOD_MS);
}

float app_speed_plan_get_linear_mps(void)
{
    return speed_plan_linear_mps;
}

static void app_speed_plan_task(void)
{
    float line_error_rate;
    float curvature_rate;
    float brake_rate;
    float ratio;
    float target_raw_mps;
    app_motion_preprocess_data_t motion_preprocess;
    app_feedforward_data_t feedforward;
    app_element_data_t element;

    app_motion_preprocess_get_data(&motion_preprocess);
    app_feedforward_get_data(&feedforward);
    app_element_get_data(&element);

    line_error_rate = app_speed_plan_norm_abs(motion_preprocess.line_error, APP_SPEED_PLAN_ERROR_MAX);
    curvature_rate = app_speed_plan_norm_abs(feedforward.curvature, APP_SPEED_PLAN_CURVATURE_MAX);
    brake_rate = app_speed_plan_max(line_error_rate, curvature_rate);
    ratio = app_speed_plan_limit(speed_plan_min_ratio, 0.0f, 1.0f);
    target_raw_mps = tfpu_mul(motion_preprocess.linear_mps,
            tfpu_sub(1.0f, tfpu_mul(tfpu_sub(1.0f, ratio), brake_rate)));

    if((APP_ELEMENT_TYPE_SEESAW == element.type) && (element.active >= 0.5f))
    {
        target_raw_mps = 1.0f;
    }

    speed_plan_linear_mps = app_speed_plan_ramp(speed_plan_linear_mps, target_raw_mps);
}
