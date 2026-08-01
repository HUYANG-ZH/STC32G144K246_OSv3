#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_led.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "app_speedout.h"

#define SERVICE_LED_TICKS_PER_MS       (10UL)
#define SERVICE_LED_TIME_HALF_RANGE    (0x80000000UL)

#define SERVICE_LED_MODE_IDLE          (0U)
#define SERVICE_LED_MODE_ON_DELAY      (1U)
#define SERVICE_LED_MODE_BLINK         (2U)

#define SERVICE_LED_STOP_BLINK_ON_MS   (1000UL)
#define SERVICE_LED_STOP_BLINK_OFF_MS  (1000UL)

static uint8 led_is_on = 0U;
static uint8 led_mode = SERVICE_LED_MODE_IDLE;
static uint8 led_blink_phase_on = 0U;
static uint32 led_deadline_tick = 0UL;
static uint32 led_on_ticks = 0UL;
static uint32 led_off_ticks = 0UL;
static uint32 led_blink_remaining = 0UL;
static uint8 led_status_valid = 0U;
static uint8 led_status_started = 0U;

static uint8 service_led_time_reached(uint32 now, uint32 deadline)
{
    return ((uint32)(now - deadline) < SERVICE_LED_TIME_HALF_RANGE);
}

static void service_led_reset_schedule(void)
{
    led_mode = SERVICE_LED_MODE_IDLE;
    led_blink_phase_on = 0U;
    led_deadline_tick = 0UL;
    led_on_ticks = 0UL;
    led_off_ticks = 0UL;
    led_blink_remaining = 0UL;
}

void service_led_init(void)
{
    bsp_led_init();
    led_is_on = 0U;
    led_status_valid = 1U;
    led_status_started = 0U;
    service_led_reset_schedule();
    service_led_blink_ms(SERVICE_LED_STOP_BLINK_ON_MS,
            SERVICE_LED_STOP_BLINK_OFF_MS, 0UL);
    #if __DBGFLAG__
    printf(">>[service_led_init]\r\n");
    wprint(">>[service_led_init]\r\n");
    #endif
}

void service_led_debug(void)
{
    printf("[led:blink500msx3]\r\n");
    service_led_blink_ms(500UL, 500UL, 3UL);
}

void service_led_on(void)
{
    bsp_led_on();
    led_is_on = 1U;
    service_led_reset_schedule();
}

void service_led_off(void)
{
    bsp_led_off();
    led_is_on = 0U;
    service_led_reset_schedule();
}

void service_led_set(uint8 enable)
{
    if(0U != enable)
    {
        service_led_on();
    }
    else
    {
        service_led_off();
    }
}

void service_led_on_ms(uint32 duration_ms)
{
    if(0UL == duration_ms)
    {
        service_led_off();
        return;
    }

    bsp_led_on();
    led_is_on = 1U;
    led_mode = SERVICE_LED_MODE_ON_DELAY;
    led_blink_phase_on = 0U;
    led_deadline_tick = service_timetick_what() + duration_ms * SERVICE_LED_TICKS_PER_MS;
    led_on_ticks = 0UL;
    led_off_ticks = 0UL;
    led_blink_remaining = 0UL;
}

void service_led_delay_ms(uint32 duration_ms)
{
    service_led_on_ms(duration_ms);
}

void service_led_blink_ms(uint32 on_time_ms, uint32 off_time_ms, uint32 repeat_count)
{
    /* A zero-length phase cannot be scheduled in the foreground ticker. */
    if((0UL == on_time_ms) || (0UL == off_time_ms))
    {
        service_led_off();
        return;
    }

    bsp_led_on();
    led_is_on = 1U;
    led_mode = SERVICE_LED_MODE_BLINK;
    led_blink_phase_on = 1U;
    led_on_ticks = on_time_ms * SERVICE_LED_TICKS_PER_MS;
    led_off_ticks = off_time_ms * SERVICE_LED_TICKS_PER_MS;
    led_blink_remaining = repeat_count;
    led_deadline_tick = service_timetick_what() + led_on_ticks;
}

static void service_led_timing_task(void)
{
    uint32 now;

    if(SERVICE_LED_MODE_IDLE == led_mode)
    {
        return;
    }

    now = service_timetick_what();
    if(0U == service_led_time_reached(now, led_deadline_tick))
    {
        return;
    }

    if(SERVICE_LED_MODE_ON_DELAY == led_mode)
    {
        service_led_off();
        return;
    }

    if(0U != led_blink_phase_on)
    {
        bsp_led_off();
        led_is_on = 0U;
        led_blink_phase_on = 0U;
        led_deadline_tick = now + led_off_ticks;
    }
    else if((1UL == led_blink_remaining))
    {
        service_led_off();
    }
    else
    {
        if(0UL != led_blink_remaining)
        {
            led_blink_remaining--;
        }

        bsp_led_on();
        led_is_on = 1U;
        led_blink_phase_on = 1U;
        led_deadline_tick = now + led_on_ticks;
    }
}

void service_led_task(void)
{
    app_speedout_data_t speedout_status;
    uint8 started;

    app_speedout_get_data(&speedout_status);
    started = (speedout_status.enabled >= 0.5f) ? 1U : 0U;
    started = (0U != started) ? 1U : 0U;

    if((0U == led_status_valid) || (led_status_started != started))
    {
        led_status_valid = 1U;
        led_status_started = started;

        if(0U != started)
        {
            service_led_on();
        }
        else
        {
            service_led_blink_ms(SERVICE_LED_STOP_BLINK_ON_MS,
                    SERVICE_LED_STOP_BLINK_OFF_MS, 0UL);
        }
    }

    service_led_timing_task();
}

void service_led_stop(void)
{
    service_led_off();
}

uint8 service_led_is_on(void)
{
    service_led_timing_task();
    return led_is_on;
}
