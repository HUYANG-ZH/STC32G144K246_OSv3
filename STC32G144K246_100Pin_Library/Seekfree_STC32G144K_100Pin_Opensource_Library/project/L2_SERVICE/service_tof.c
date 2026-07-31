#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_tof.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define SERVICE_TOF_SAMPLE_PERIOD_TICK      (500UL)

static uint32 service_tof_last_request_tick = 0UL;

void service_tof_init(void)
{
#if SERVICE_TOF_ENABLE
    (void)bsp_tof_init();
    service_tof_last_request_tick = service_timetick_what();
    #if __DBGFLAG__
    printf(">>[service_tof_init]\r\n");
    wprint(">>[service_tof_init]\r\n");
    #endif
#else
    service_tof_last_request_tick = 0UL;
#endif
}

void service_tof_task(void)
{
#if SERVICE_TOF_ENABLE
    uint32 now;

    bsp_tof_process();
    if(0U == bsp_tof_is_ready())
    {
        return;
    }

    now = service_timetick_what();
    if((uint32)(now - service_tof_last_request_tick) >= SERVICE_TOF_SAMPLE_PERIOD_TICK)
    {
        if(0U != bsp_tof_request_sample())
        {
            service_tof_last_request_tick = now;
        }
    }
#endif
}

uint8 service_tof_is_ready(void)
{
#if SERVICE_TOF_ENABLE
    return bsp_tof_is_ready();
#else
    return 0U;
#endif
}

uint8 service_tof_get_last_error(void)
{
#if SERVICE_TOF_ENABLE
    return bsp_tof_get_last_error();
#else
    return 0U;
#endif
}

uint16 service_tof_get_distance_mm(void)
{
#if SERVICE_TOF_ENABLE
    (void)bsp_tof_request_sample();
    return bsp_tof_get_distance_mm();
#else
    return BSP_TOF_INVALID_DISTANCE_MM;
#endif
}

uint8 service_tof_get_range_status(void)
{
#if SERVICE_TOF_ENABLE
    return bsp_tof_get_range_status();
#else
    return BSP_TOF_RANGE_STATUS_NO_UPDATE;
#endif
}
