#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_buzzer.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define SERVICE_BUZZER_TICKS_PER_MS        (10UL)
#define SERVICE_BUZZER_TIME_HALF_RANGE     (0x80000000UL)

static uint8 buzzer_is_on = 0U;
static uint8 buzzer_timing = 0U;
static uint32 buzzer_deadline_tick = 0UL;

static uint8 service_buzzer_time_reached(uint32 now, uint32 deadline)
{
    return ((uint32)(now - deadline) < SERVICE_BUZZER_TIME_HALF_RANGE);
}

void service_buzzer_init(void)
{
    bsp_buzzer_init();
    buzzer_is_on = 1U;
    buzzer_timing = 0U;
    buzzer_deadline_tick = 0UL;
    #if __DBGFLAG__
    printf(">>[service_buzzer_init]\r\n");
    wprint(">>[service_buzzer_init]\r\n");
    #endif
}

void service_buzzer_debug(void)
{
    printf("[buzzer:scheduled500ms]\r\n");
    service_buzzer_beep_ms(500);
}

void service_buzzer_beep_ms(uint32 duration_ms)
{
    if(0UL == duration_ms)
    {
        service_buzzer_stop();
        return;
    }

    bsp_buzzer_on();
    buzzer_is_on = 1U;
    buzzer_timing = 1U;
    buzzer_deadline_tick = service_timetick_what() + duration_ms * SERVICE_BUZZER_TICKS_PER_MS;
}

void service_buzzer_task(void)
{
    uint32 now;

    if(0U == buzzer_timing)
    {
        return;
    }

    now = service_timetick_what();
    if(0U != service_buzzer_time_reached(now, buzzer_deadline_tick))
    {
        service_buzzer_stop();
    }
}

void service_buzzer_stop(void)
{
    bsp_buzzer_off();
    buzzer_is_on = 0U;
    buzzer_timing = 0U;
}

uint8 service_buzzer_is_on(void)
{
    service_buzzer_task();
    return buzzer_is_on;
}
