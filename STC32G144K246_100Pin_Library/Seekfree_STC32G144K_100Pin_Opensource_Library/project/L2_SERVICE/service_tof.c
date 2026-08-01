#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_tof.h"
#include "service_timetick.h"

#define SERVICE_TOF_INVALID_DISTANCE_THRESHOLD_MM (1000U)

static uint32 service_tof_last_request_tick = 0UL;

void service_tof_init(void)
{
#if SERVICE_TOF_ENABLE
    (void)bsp_tof_init();
    service_tof_last_request_tick = service_timetick_what();
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
    if((uint32)(now - service_tof_last_request_tick) >= SERVICE_TOF_PERIOD_TICK)
    {
        if(0U != bsp_tof_request_sample())
        {
            service_tof_last_request_tick = now;
#if SERVICE_TOF_DEBUG_PRINT_ENABLE
            /* 与 30Hz 采样节拍同步输出, VOFA+ RawData 文本协议:
               首字段为通道名, 距离+range_status 双通道 */
            printf("tof_distance_mm,%u,tof_range_status,%u\r\n",
                    (unsigned int)bsp_tof_get_distance_mm(),
                    (unsigned int)bsp_tof_get_range_status());
#endif
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
    uint16 distance_mm;
    uint8 range_status;

    distance_mm = bsp_tof_get_distance_mm();
    range_status = bsp_tof_get_range_status();

    if((0U != range_status) ||
            (distance_mm > SERVICE_TOF_INVALID_DISTANCE_THRESHOLD_MM))
    {
        return 0U;
    }

    return distance_mm;
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
