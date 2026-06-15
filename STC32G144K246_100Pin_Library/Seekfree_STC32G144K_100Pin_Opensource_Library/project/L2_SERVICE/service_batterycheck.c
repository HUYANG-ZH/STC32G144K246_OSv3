#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_batterycheck.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define BATTERYCHECK_PERIOD_TICK        (20000UL)

static float batterycheck_voltage = 0.0f;
static uint32 batterycheck_last_tick = 0UL;

static void service_batterycheck_update_now(void);
static void service_batterycheck_voltage_reply(void);

static void service_batterycheck_update_now(void)
{
    bsp_battery_vol(&batterycheck_voltage);
    batterycheck_last_tick = service_timetick_what();
}

static void service_batterycheck_voltage_reply(void)
{
    service_batterycheck_update_now();
    wprint("battery_voltage,%.3f\r\n", batterycheck_voltage);
}

void service_batterycheck_init(void)
{
    bsp_battery_init();
    service_batterycheck_update_now();
    (void)service_packet_add_action("battery_voltage", service_batterycheck_voltage_reply, 0UL);
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
        service_batterycheck_update_now();
    }

}

void service_batterycheck_get_voltage(float *voltage)
{
    service_batterycheck_update_now();
    if(NULL != voltage)
    {
        *voltage = batterycheck_voltage;
    }
}
