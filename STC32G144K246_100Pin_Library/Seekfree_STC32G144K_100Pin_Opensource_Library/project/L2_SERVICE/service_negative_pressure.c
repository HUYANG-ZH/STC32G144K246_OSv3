#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_negative_pressure.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define NEGATIVE_PRESSURE_PERCENT_MAX       (100U)
#define NEGATIVE_PRESSURE_DUTY_MIN          (PWM_DUTY_MAX * 5UL / 100UL)
#define NEGATIVE_PRESSURE_DUTY_MAX          (PWM_DUTY_MAX * 10UL / 100UL)
#define NEGATIVE_PRESSURE_PACKET_COUNT      (1U)
/* 缓启动: 负压变化率不超过 10%/s (0.1ms/tick), 1 百分之一% 需要 10 tick */
#define NEGATIVE_PRESSURE_RAMP_PERCENT_PER_SEC  (10U)
#define NEGATIVE_PRESSURE_RAMP_TICK_PER_H       \
    (10000U / (NEGATIVE_PRESSURE_RAMP_PERCENT_PER_SEC * 100U))

static volatile uint8 negative_pressure_percent = 0U;
static float negative_pressure_percent_config = 0.0f;
static uint8 negative_pressure_config_percent = 0U;
static volatile uint8 negative_pressure_requested_percent = 0U;
static volatile uint8 negative_pressure_request_pending = 0U;
static volatile uint8 negative_pressure_control_override = 0U;
static uint32 negative_pressure_last_ramp_tick = 0UL;
static uint8 negative_pressure_ramp_remainder = 0U;
static uint16 negative_pressure_actual_h = 0U;   /* 斜坡实际值(百分之一%), 与取整发布的 percent 分离, 防止亚百分比进度丢失 */

static uint32 negative_pressure_percent_to_duty(uint8 percent)
{
    uint32 duty_range;

    if(NEGATIVE_PRESSURE_PERCENT_MAX < percent)
    {
        percent = NEGATIVE_PRESSURE_PERCENT_MAX;
    }

    duty_range = NEGATIVE_PRESSURE_DUTY_MAX - NEGATIVE_PRESSURE_DUTY_MIN;

    return NEGATIVE_PRESSURE_DUTY_MIN + (duty_range * percent / NEGATIVE_PRESSURE_PERCENT_MAX);
}

static uint8 negative_pressure_float_to_percent(float percent)
{
    if(0.0f >= percent)
    {
        return 0U;
    }

    if((float)NEGATIVE_PRESSURE_PERCENT_MAX <= percent)
    {
        return NEGATIVE_PRESSURE_PERCENT_MAX;
    }

    return (uint8)(percent + 0.5f);
}

static void service_negative_pressure_config_updated(void)
{
    uint8 percent;
    uint8 ea_backup;

    percent = negative_pressure_float_to_percent(negative_pressure_percent_config);
    ea_backup = EA;
    EA = 0;
    negative_pressure_config_percent = percent;
    negative_pressure_control_override = 0U;
    negative_pressure_requested_percent = percent;
    negative_pressure_request_pending = 1U;
    EA = ea_backup;
}

void service_negative_pressure_init(void)
{
    pwm_init(SERVICE_NEGATIVE_PRESSURE_PWM_CH, SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ, 0U);
    negative_pressure_percent = 0U;
    negative_pressure_percent_config = 0.0f;
    negative_pressure_config_percent = 0U;
    negative_pressure_requested_percent = 0U;
    negative_pressure_request_pending = 0U;
    negative_pressure_control_override = 0U;
    negative_pressure_last_ramp_tick = service_timetick_what();
    negative_pressure_ramp_remainder = 0U;
    negative_pressure_actual_h = 0U;
    (void)service_packet_add_variable_with_callback("negative_pressure", &negative_pressure_percent_config,
            NEGATIVE_PRESSURE_PACKET_COUNT, service_negative_pressure_config_updated);
    #if __DBGFLAG__
    printf(">>[service_negative_pressure_init]\r\n");
    wprint(">>[service_negative_pressure_init]\r\n");
    #endif
}

void service_negative_pressure_debug(void)
{
    printf("[pressure] : 30\r\n");
    service_negative_pressure_set_percent(30);
}

void service_negative_pressure_task(void)
{
    uint8 percent;
    uint8 ea_backup;

    percent = negative_pressure_float_to_percent(negative_pressure_percent_config);
    ea_backup = EA;
    EA = 0;
    if(0U != negative_pressure_control_override)
    {
        EA = ea_backup;
        return;
    }

    if(percent != negative_pressure_config_percent)
    {
        negative_pressure_config_percent = percent;
        negative_pressure_requested_percent = percent;
        negative_pressure_request_pending = 1U;
    }
    EA = ea_backup;
}

void service_negative_pressure_request_percent(uint8 percent)
{
    uint8 ea_backup;

    if(NEGATIVE_PRESSURE_PERCENT_MAX < percent)
    {
        percent = NEGATIVE_PRESSURE_PERCENT_MAX;
    }

    ea_backup = EA;
    EA = 0;
    negative_pressure_requested_percent = percent;
    negative_pressure_request_pending = 1U;
    EA = ea_backup;
}

void service_negative_pressure_apply_request(void)
{
    uint32 now;
    uint32 delta_tick;
    uint32 step_h;
    uint32 actual_h;
    uint32 target_h;
    uint8 next_percent;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    if(0U == negative_pressure_request_pending)
    {
        EA = ea_backup;
        return;
    }
    target_h = (uint32)negative_pressure_requested_percent * 100U;
    EA = ea_backup;
    actual_h = (uint32)negative_pressure_actual_h;

    /* 缓启动斜坡: 变化率 ≤5%/s, 按 0.1ms tick 定时, 整数定点(百分之一%) */
    now = service_timetick_what();
    delta_tick = now - negative_pressure_last_ramp_tick;
    negative_pressure_last_ramp_tick = now;

    step_h = delta_tick / NEGATIVE_PRESSURE_RAMP_TICK_PER_H;
    negative_pressure_ramp_remainder = (uint8)(negative_pressure_ramp_remainder +
            (delta_tick % NEGATIVE_PRESSURE_RAMP_TICK_PER_H));
    if(negative_pressure_ramp_remainder >= (uint8)NEGATIVE_PRESSURE_RAMP_TICK_PER_H)
    {
        negative_pressure_ramp_remainder = (uint8)(negative_pressure_ramp_remainder -
                NEGATIVE_PRESSURE_RAMP_TICK_PER_H);
        step_h++;
    }

    if(actual_h < target_h)
    {
        actual_h += step_h;
        if(actual_h > target_h)
        {
            actual_h = target_h;
        }
    }
    else if(actual_h > target_h)
    {
        if(step_h >= actual_h)
        {
            actual_h = target_h;
        }
        else
        {
            actual_h -= step_h;
            if(actual_h < target_h)
            {
                actual_h = target_h;
            }
        }
    }

    next_percent = (uint8)((actual_h + 50U) / 100U);
    if(next_percent != negative_pressure_percent)
    {
        pwm_set_duty(SERVICE_NEGATIVE_PRESSURE_PWM_CH, negative_pressure_percent_to_duty(next_percent));
    }
    negative_pressure_percent = next_percent;
    negative_pressure_actual_h = (uint16)actual_h;

    if(actual_h == target_h)
    {
        negative_pressure_request_pending = 0U;
    }
}

void service_negative_pressure_set_percent_immediate(uint8 percent)
{
    uint8 ea_backup;

    if(NEGATIVE_PRESSURE_PERCENT_MAX < percent)
    {
        percent = NEGATIVE_PRESSURE_PERCENT_MAX;
    }

    ea_backup = EA;
    EA = 0;
    negative_pressure_control_override = 1U;
    negative_pressure_requested_percent = percent;
    negative_pressure_request_pending = 0U;
    negative_pressure_percent = percent;
    negative_pressure_actual_h = (uint16)percent * 100U;
    EA = ea_backup;

    pwm_set_duty(SERVICE_NEGATIVE_PRESSURE_PWM_CH, negative_pressure_percent_to_duty(percent));
    negative_pressure_last_ramp_tick = service_timetick_what();
    negative_pressure_ramp_remainder = 0U;
}

void service_negative_pressure_set_percent(uint8 percent)
{
    uint8 ea_backup;

    /* A real-time controller owns this value until the packet write callback
     * explicitly accepts a new operator configuration. */
    ea_backup = EA;
    EA = 0;
    negative_pressure_control_override = 1U;
    if(NEGATIVE_PRESSURE_PERCENT_MAX < percent)
    {
        percent = NEGATIVE_PRESSURE_PERCENT_MAX;
    }
    negative_pressure_requested_percent = percent;
    negative_pressure_request_pending = 1U;
    EA = ea_backup;
}

uint8 service_negative_pressure_get_percent(void)
{
    return negative_pressure_percent;
}

uint8 service_negative_pressure_get_config_percent(void)
{
    return negative_pressure_float_to_percent(negative_pressure_percent_config);
}
