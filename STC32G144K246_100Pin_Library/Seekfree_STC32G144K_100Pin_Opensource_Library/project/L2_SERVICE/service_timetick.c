#include "zf_common_headfile.h"
#include "service_timetick.h"

static volatile uint32 time;

static void timetick_new(void);

/*时间戳单位:0.1ms*/
void service_timetick_init(void)
{
    time = 0;
    pit_us_init(TIM0_PIT,100,timetick_new);
    #if __DBGFLAG__
    printf(">>[service_timetick_init]\r\n");
    #endif
}

void service_timetick_debug(void)
{
    printf("[timetick:time=%ld.]\r\n", service_timetick_what());
}

uint32 service_timetick_what(void)
{
    uint32 snapshot;
    uint8 ea_backup;

    /* `time` is advanced by TIM0.  Publish a complete 32-bit tick to both
     * background state machines and higher-priority control timers. */
    ea_backup = EA;
    EA = 0;
    snapshot = time;
    EA = ea_backup;
    return snapshot;
}

static void timetick_new(void)
{
    time++;
}
