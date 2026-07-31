#include "zf_common_headfile.h"
#include "bsp_tof.h"
#include "bsp_tof_async.h"

/* Public compatibility facade: every runtime operation is now a snapshot operation. */
uint8 bsp_tof_init(void)
{
    return bsp_tof_async_init();
}

void bsp_tof_process(void)
{
    bsp_tof_async_process();
}

uint8 bsp_tof_is_ready(void)
{
    return bsp_tof_async_is_ready();
}

uint8 bsp_tof_request_sample(void)
{
    return bsp_tof_async_request_sample();
}

uint16 bsp_tof_get_distance_mm(void)
{
    return bsp_tof_async_get_distance_mm();
}

uint8 bsp_tof_get_range_status(void)
{
    return bsp_tof_async_get_range_status();
}

uint8 bsp_tof_get_last_error(void)
{
    return bsp_tof_async_get_last_error();
}
