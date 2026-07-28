#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_batterycheck.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define BATTERYCHECK_PERIOD_TICK        (1000UL)
#define BATTERYCHECK_DMA_TIMEOUT_TICK   (100UL)

static float batterycheck_voltage = 0.0f;
static uint32 batterycheck_last_request_tick = 0UL;
static uint32 batterycheck_last_sequence = 0UL;
static uint8 batterycheck_valid = 0U;

static void service_batterycheck_refresh_snapshot(void);
static void service_batterycheck_request_next(void);
static void service_batterycheck_voltage_reply(void);

static void service_batterycheck_refresh_snapshot(void)
{
    uint16 rawdata;
    uint32 sequence;

    if(0U == bsp_battery_get_snapshot(&rawdata, &sequence))
    {
        return;
    }

    if((0U == batterycheck_valid) || (sequence != batterycheck_last_sequence))
    {
        bsp_battery_vol(&batterycheck_voltage);
        batterycheck_last_sequence = sequence;
        batterycheck_valid = 1U;
    }
}

static void service_batterycheck_request_next(void)
{
    if(0U != bsp_battery_request_sample())
    {
        batterycheck_last_request_tick = service_timetick_what();
    }
}

static void service_batterycheck_voltage_reply(void)
{
    service_batterycheck_refresh_snapshot();
    service_batterycheck_request_next();
    if(0U != batterycheck_valid)
    {
        wprint("battery_voltage,%.3f\r\n", batterycheck_voltage);
    }
    else
    {
        wprint("battery_voltage,pending\r\n");
    }
}

void service_batterycheck_init(void)
{
    bsp_battery_init();
    batterycheck_voltage = 0.0f;
    batterycheck_last_request_tick = service_timetick_what();
    batterycheck_last_sequence = 0UL;
    batterycheck_valid = 0U;
    (void)service_packet_add_action("battery_voltage", service_batterycheck_voltage_reply, 0UL);
    #if __DBGFLAG__
    printf(">>[service_batterycheck_init]\r\n");
    wprint(">>[service_batterycheck_init]\r\n");
    #endif
}

void service_batterycheck_debug(void)
{
    service_batterycheck_refresh_snapshot();
    if(0U != batterycheck_valid)
    {
        printf("[batterycheck:voltage=%.4f]\r\n", batterycheck_voltage);
    }
    else
    {
        printf("[batterycheck:pending]\r\n");
    }
}

void service_batterycheck_task(void)
{
    uint32 now;

    service_batterycheck_refresh_snapshot();
    now = service_timetick_what();
    if(0U != bsp_battery_is_busy())
    {
        if((uint32)(now - batterycheck_last_request_tick) >= BATTERYCHECK_DMA_TIMEOUT_TICK)
        {
            bsp_battery_recover();
            service_batterycheck_request_next();
        }
        return;
    }

    if((uint32)(now - batterycheck_last_request_tick) >= BATTERYCHECK_PERIOD_TICK)
    {
        service_batterycheck_request_next();
    }
}

uint8 service_batterycheck_is_valid(void)
{
    service_batterycheck_refresh_snapshot();
    return batterycheck_valid;
}

uint8 service_batterycheck_raw_is_valid(void)
{
    return bsp_battery_sample_is_valid();
}

void service_batterycheck_get_raw(uint16 *rawdata)
{
    bsp_battery_get_raw(rawdata);
}

uint8 service_batterycheck_get_raw_snapshot(uint16 *rawdata, uint32 *sequence)
{
    return bsp_battery_get_snapshot(rawdata, sequence);
}

void service_batterycheck_get_voltage(float *voltage)
{
    service_batterycheck_refresh_snapshot();
    if(NULL != voltage)
    {
        *voltage = batterycheck_voltage;
    }
}
