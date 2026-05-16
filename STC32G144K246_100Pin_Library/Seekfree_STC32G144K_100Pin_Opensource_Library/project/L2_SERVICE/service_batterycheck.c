#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_batterycheck.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define BATTERYCHECK_PERIOD_TICK        (20000UL)

static float batterycheck_voltage = 0.0f;
static uint32 batterycheck_last_tick = 0UL;

void service_batterycheck_init(void)
{
    bsp_battery_init();
    bsp_battery_vol(&batterycheck_voltage);
    batterycheck_last_tick = service_timetick_what();
}

void service_batterycheck_debug(void)
{
    printf("[batterycheck:voltage=%.4f]\r\n",batterycheck_voltage);
}

void service_batterycheck_task(void)
{
    uint32 now;

    now = service_timetick_what();
    if((uint32)(now - batterycheck_last_tick) >= BATTERYCHECK_PERIOD_TICK)
    {
        bsp_battery_vol(&batterycheck_voltage);
        printf("battery%f\r\n",batterycheck_voltage);
        batterycheck_last_tick = now;
    }

    if(batterycheck_voltage <= 11.0f)
    {
        printf("batterylow:%f\r\n",batterycheck_voltage);
        wprint("batterylow.\r\n");
    }
}

void service_batterycheck_get_voltage(float *voltage)
{
    service_batterycheck_task();
    if(NULL != voltage)
    {
        *voltage = batterycheck_voltage;
    }
}
