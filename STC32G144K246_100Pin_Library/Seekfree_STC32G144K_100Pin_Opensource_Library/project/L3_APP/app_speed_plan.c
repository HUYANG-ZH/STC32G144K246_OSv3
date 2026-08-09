#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_packet.h"
#include "app_feedforward.h"
#include "app_motion_preprocess.h"
#include "app_speed_plan.h"
#include "app_element.h"
#include "app_attitude.h"
#include "app_speedout.h"
#include "service_negative_pressure.h"
#include "service_tof.h"
#include "service_wireless_uart.h"

#define APP_SPEED_PLAN_PACKET_SINGLE_COUNT      (1U)
#define APP_SPEED_PLAN_DEFAULT_MIN_RATIO        (0.9f)
#define APP_SPEED_PLAN_ACCEL_LIMIT_MPS2         (80.0f)
#define APP_SPEED_PLAN_DECEL_LIMIT_MPS2         (80.0f)
#define APP_SPEED_PLAN_ERROR_MAX                (1.0f)
#define APP_SPEED_PLAN_CURVATURE_MAX            (1.0f)

/* 负压生效量接口: 无线 negative_pressure 为基础量, 触发时叠加增量, 上限 90 */
#define APP_SPEED_PLAN_PRESSURE_BOOST_DEFAULT   (15.0f)     /* 触发时在基础量上增加的百分比 */
#define APP_SPEED_PLAN_PRESSURE_MAX             (90.0f)     /* 生效量上限 */
#define APP_SPEED_PLAN_PITCH_LIMIT_DEG          (30.0f)     /* pitch(0-360) 偏离水平 (30,330) 触发 */
#define APP_SPEED_PLAN_TOF_NEAR_DISTANCE_MM     (200U)      /* TOF 测距 <200mm 触发(预留) */

static volatile float speed_plan_min_ratio = APP_SPEED_PLAN_DEFAULT_MIN_RATIO;
static volatile float speed_plan_pressure_boost = APP_SPEED_PLAN_PRESSURE_BOOST_DEFAULT;
static uint8 speed_plan_pressure_boost_active = 0U;
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
            (float *)&speed_plan_min_ratio, APP_SPEED_PLAN_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("pressure_boost",
            (float *)&speed_plan_pressure_boost, APP_SPEED_PLAN_PACKET_SINGLE_COUNT);
}

/* TOF 接近检测: 测距 <200mm 判定为接近障碍, 触发负压增量。
   TOF 未就绪/测距异常时 service_tof_get_distance_mm() 返回 8192/7777, 均 >200, 不会误触发 */
static uint8 app_speed_plan_tof_near_obstacle(void)
{
    if(service_tof_get_distance_mm() < APP_SPEED_PLAN_TOF_NEAR_DISTANCE_MM)
    {
        return 1U;
    }
    return 0U;
}

/* 负压增量触发条件: TOF 接近(<200mm) 或 pitch(0-360) 偏离水平 (30,330) */
static uint8 app_speed_plan_pressure_boost_condition(void)
{
    app_attitude_data_t attitude;

    if(0U != app_speed_plan_tof_near_obstacle())
    {
        return 1U;
    }

    app_attitude_get_data(&attitude);
    /* pitch 已改为 0-360 语义: 偏离水平 (30, 330) 区间即触发(与原 ±30° 等效) */
    if((0U != attitude.valid) &&
       (attitude.pitch_deg > APP_SPEED_PLAN_PITCH_LIMIT_DEG) &&
       (attitude.pitch_deg < (360.0f - APP_SPEED_PLAN_PITCH_LIMIT_DEG)))
    {
        return 1U;
    }

    return 0U;
}

/* 负压生效量计算: 基础量(无线 negative_pressure) + 触发增量, 上限 90。
   触发期间接管负压输出, 恢复后还原基础量。调用方为 TIM5 前的 5ms 控制链。 */
static void app_speed_plan_update_pressure(void)
{
    uint8 active;
    float effective;
    app_speedout_data_t speedout;

    /* 安全联锁(IMU/电感故障)期间不接管负压: TIM5 联锁路径会强制置 0,
       这里再写入会把联锁击穿, 因此联锁期间直接放弃本周期负压管理 */
    if(0U != app_speedout_get_safety_inhibit())
    {
        speed_plan_pressure_boost_active = 0U;
        return;
    }

    /* 车辆未启动(无线 stop/急停后 enabled=0)时不接管负压:
       否则 stop 后触发条件(如 |pitch|>30°)成立时会每 5ms 把负压重新拉起 */
    app_speedout_get_data(&speedout);
    if(speedout.enabled < 0.5f)
    {
        speed_plan_pressure_boost_active = 0U;
        return;
    }

    active = app_speed_plan_pressure_boost_condition();
    if(0U != active)
    {
        effective = tfpu_add(tfpu_int2float((long)service_negative_pressure_get_config_percent()),
                speed_plan_pressure_boost);
        if(effective > APP_SPEED_PLAN_PRESSURE_MAX)
        {
            effective = APP_SPEED_PLAN_PRESSURE_MAX;
        }
        service_negative_pressure_set_percent((uint8)(effective + 0.5f));
    }
    else if(0U != speed_plan_pressure_boost_active)
    {
        service_negative_pressure_set_percent(service_negative_pressure_get_config_percent());
    }
    speed_plan_pressure_boost_active = active;
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

    /* 负压生效量接口: 基础量 + 触发增量(上限90) */
    app_speed_plan_update_pressure();
}
