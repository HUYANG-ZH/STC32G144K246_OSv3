#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_negative_pressure.h"

#define NEGATIVE_PRESSURE_PERCENT_MAX       (100U)
#define NEGATIVE_PRESSURE_DUTY_MIN          (PWM_DUTY_MAX * 5UL / 100UL)
#define NEGATIVE_PRESSURE_DUTY_MAX          (PWM_DUTY_MAX * 10UL / 100UL)
#define NEGATIVE_PRESSURE_PACKET_COUNT      (1U)

static uint8 negative_pressure_percent = 0U;
static float negative_pressure_percent_config = 0.0f;

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

void service_negative_pressure_init(void)
{
    pwm_init(SERVICE_NEGATIVE_PRESSURE_PWM_CH, SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ, 0U);
    negative_pressure_percent = 0U;
    negative_pressure_percent_config = 0.0f;
    (void)service_packet_add_variable("negative_pressure", &negative_pressure_percent_config, NEGATIVE_PRESSURE_PACKET_COUNT);
}

void service_negative_pressure_debug(void)
{
    printf("[pressure] : 30\r\n");
    service_negative_pressure_set_percent(30);
}

void service_negative_pressure_task(void)
{
    uint8 percent;

    percent = negative_pressure_float_to_percent(negative_pressure_percent_config);
    if(percent != negative_pressure_percent)
    {
        service_negative_pressure_set_percent(percent);
    }
}

void service_negative_pressure_set_percent(uint8 percent)
{
    if(NEGATIVE_PRESSURE_PERCENT_MAX < percent)
    {
        percent = NEGATIVE_PRESSURE_PERCENT_MAX;
    }

    pwm_set_duty(SERVICE_NEGATIVE_PRESSURE_PWM_CH, negative_pressure_percent_to_duty(percent));
    negative_pressure_percent = percent;
    negative_pressure_percent_config = (float)percent;
}

uint8 service_negative_pressure_get_percent(void)
{
    return negative_pressure_percent;
}
