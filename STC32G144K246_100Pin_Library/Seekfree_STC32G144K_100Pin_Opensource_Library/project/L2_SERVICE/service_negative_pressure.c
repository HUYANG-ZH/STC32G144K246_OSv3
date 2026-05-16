#include "zf_common_headfile.h"
#include "service_negative_pressure.h"

#define NEGATIVE_PRESSURE_PERCENT_MAX       (100U)
#define NEGATIVE_PRESSURE_DUTY_MIN          (PWM_DUTY_MAX * 5UL / 100UL)
#define NEGATIVE_PRESSURE_DUTY_MAX          (PWM_DUTY_MAX * 10UL / 100UL)

static uint8 negative_pressure_percent = 0U;

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

void service_negative_pressure_init(void)
{
    pwm_init(SERVICE_NEGATIVE_PRESSURE_PWM_CH, SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ, 0U);
    negative_pressure_percent = 0U;
}

void service_negative_pressure_debug(void)
{
    printf("[pressure] : 30\r\n");
    service_negative_pressure_set_percent(30);
}

void service_negative_pressure_set_percent(uint8 percent)
{
    if(NEGATIVE_PRESSURE_PERCENT_MAX < percent)
    {
        percent = NEGATIVE_PRESSURE_PERCENT_MAX;
    }

    pwm_set_duty(SERVICE_NEGATIVE_PRESSURE_PWM_CH, negative_pressure_percent_to_duty(percent));
    negative_pressure_percent = percent;
}

uint8 service_negative_pressure_get_percent(void)
{
    return negative_pressure_percent;
}
