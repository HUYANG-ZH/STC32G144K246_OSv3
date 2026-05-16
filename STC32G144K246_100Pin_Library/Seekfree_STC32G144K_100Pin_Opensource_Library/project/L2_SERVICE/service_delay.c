#include "zf_common_headfile.h"
#include "service_timetick.h"
#include "service_delay.h"

#define SERVICE_DELAY_TICKS_PER_MS      (10UL)
#define SERVICE_DELAY_US_PER_TICK       (100UL)

static void service_delay_ticks(uint32 ticks)
{
    uint32 start_tick;

    if(0UL == ticks)
    {
        return;
    }

    start_tick = service_timetick_what();
    while((uint32)(service_timetick_what() - start_tick) < ticks)
    {
    }
}

void service_delay_ms(uint32 ms)
{
    service_delay_ticks(ms * SERVICE_DELAY_TICKS_PER_MS);
}

void service_delay_us(uint32 us)
{
    service_delay_ticks((us + SERVICE_DELAY_US_PER_TICK - 1UL) / SERVICE_DELAY_US_PER_TICK);
}
