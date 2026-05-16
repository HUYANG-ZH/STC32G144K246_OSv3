#include "zf_common_headfile.h"
static volatile uint32 time;

static void timetick_new(void);

/*时间戳单位:0.1ms*/
void service_timetick_init(void)
{
    time = 0;
    pit_us_init(TIM0_PIT,100,timetick_new);
}

void service_timetick_debug(void)
{
    printf("[timetick:time=%ld.]\r\n",time);
}

uint32 service_timetick_what(void)
{
    uint8 ea_backup;
    uint32 now;

    ea_backup = EA;
    EA = 0;
    now = time;
    EA = ea_backup;

    return now;
}

static void timetick_new(void)
{
    time++;
}
