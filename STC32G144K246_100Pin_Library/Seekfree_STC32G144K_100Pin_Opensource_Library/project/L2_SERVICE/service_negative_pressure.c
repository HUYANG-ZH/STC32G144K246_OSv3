#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_negative_pressure.h"

#define NEGATIVE_PRESSURE_PERCENT_MAX       (100U)
#define NEGATIVE_PRESSURE_DUTY_MIN          (PWM_DUTY_MAX * 5UL / 100UL)
#define NEGATIVE_PRESSURE_DUTY_MAX          (PWM_DUTY_MAX * 10UL / 100UL)
#define NEGATIVE_PRESSURE_PACKET_COUNT      (1U)

static uint8 negative_pressure_percent = 0U;
static float negative_pressure_percent_config = 0.0f;
static uint8 negative_pressure_config_percent = 0U;
static volatile uint8 negative_pressure_requested_percent = 0U;
static volatile uint8 negative_pressure_request_pending = 0U;
static volatile uint8 negative_pressure_control_override = 0U;

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
    (void)service_packet_add_variable_with_callback("negative_pressure", &negative_pressure_percent_config,
            NEGATIVE_PRESSURE_PACKET_COUNT, service_negative_pressure_config_updated);
    #if __DBGFLAG__
    printf(">>[service_negative_pressure_init]\r\n");
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
    uint8 percent;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    if(0U == negative_pressure_request_pending)
    {
        EA = ea_backup;
        return;
    }
    percent = negative_pressure_requested_percent;
    negative_pressure_request_pending = 0U;
    EA = ea_backup;

    pwm_set_duty(SERVICE_NEGATIVE_PRESSURE_PWM_CH, negative_pressure_percent_to_duty(percent));
    negative_pressure_percent = percent;
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
