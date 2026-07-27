#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_button.h"
#include "service_timetick.h"

#define BUTTON_POLL_INTERVAL_TICK          (100UL)
#define BUTTON_CLICK_INTERVAL_TICK         (300UL)

static uint32 s_last_poll_tick = 0UL;
static uint8 s_raw = 0U;
static uint8 s_debounced = 0U;
static uint8 s_last_debounced = 0U;
static uint8 s_clicked = 0U;
static uint32 s_last_click_tick = 0UL;

void service_button_init(void)
{
    gpio_init(IO_P32, GPI, GPIO_LOW, GPI_PULL_DOWN);
}

void service_button_task(void)
{
    uint32 now;

    now = service_timetick_what();
    if((uint32)(now - s_last_poll_tick) < BUTTON_POLL_INTERVAL_TICK)
    {
        return;
    }
    s_last_poll_tick = now;

    s_raw = gpio_get_level(IO_P32);

    if(s_raw == s_debounced)
    {
        if(s_debounced && !s_last_debounced)
        {
            s_clicked = 1U;
        }
        s_last_debounced = s_debounced;
    }
    else
    {
        s_debounced = s_raw;
    }
}

uint8 service_button_clicked(void)
{
    uint32 now;
    uint8 ret;

    ret = s_clicked;
    if(0U != ret)
    {
        now = service_timetick_what();
        if((uint32)(now - s_last_click_tick) < BUTTON_CLICK_INTERVAL_TICK)
        {
            ret = 0U;
        }
        else
        {
            s_last_click_tick = now;
        }
    }
    s_clicked = 0U;
    return ret;
}

uint8 service_button_get_state(void)
{
    return s_debounced;
}
