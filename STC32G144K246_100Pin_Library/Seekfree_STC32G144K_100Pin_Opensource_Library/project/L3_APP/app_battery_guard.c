#include "zf_common_headfile.h"
#include "service_batterycheck.h"
#include "service_timetick.h"
#include "app_feedforward.h"
#include "app_motion_postprocess.h"
#include "app_speedout.h"
#include "app_battery_guard.h"

#define APP_BATTERY_GUARD_PIT                  TIM8_PIT
#define APP_BATTERY_GUARD_PERIOD_MS            (10U)
/* ceil(11.0 V * 4096 / 36.4737); compare raw ADC data in the timer. */
#define APP_BATTERY_GUARD_LOW_RAW              (1236U)
#define APP_BATTERY_GUARD_STARTUP_GRACE_TICK    (3000UL)
#define APP_BATTERY_GUARD_MAX_SAMPLE_AGE_TICK   (1500UL)

#define APP_BATTERY_GUARD_REASON_LOW_VOLTAGE    (1U)
#define APP_BATTERY_GUARD_REASON_STALE_SAMPLE   (2U)

static volatile uint8 battery_guard_triggered = 0U;
static volatile uint8 battery_guard_report_pending = 0U;
static volatile uint16 battery_guard_trip_raw = 0U;
static volatile uint8 battery_guard_reason = 0U;
static uint32 battery_guard_last_sequence = 0UL;
static uint32 battery_guard_last_fresh_tick = 0UL;
static uint32 battery_guard_start_tick = 0UL;
static uint8 battery_guard_sample_seen = 0U;

static void app_battery_guard_trip(uint16 rawdata, uint8 reason)
{
    if(0U != battery_guard_triggered)
    {
        return;
    }

    battery_guard_triggered = 1U;
    battery_guard_trip_raw = rawdata;
    battery_guard_reason = reason;
    battery_guard_report_pending = 1U;
    /* An interlock cannot be overridden by a concurrent START command. */
    app_speedout_set_safety_inhibit(APP_SPEEDOUT_SAFETY_BATTERY);
}

/* The only safety decision and motor stop request live in a high-priority timer. */
static void app_battery_guard_tick(void)
{
    uint16 rawdata = 0U;
    uint32 sequence;
    uint32 now;

    if(0U != battery_guard_triggered)
    {
        return;
    }

    now = service_timetick_what();
    if(0U != service_batterycheck_get_raw_snapshot(&rawdata, &sequence))
    {
        if((0U == battery_guard_sample_seen) || (sequence != battery_guard_last_sequence))
        {
            battery_guard_last_sequence = sequence;
            battery_guard_last_fresh_tick = now;
            battery_guard_sample_seen = 1U;
            if(rawdata < APP_BATTERY_GUARD_LOW_RAW)
            {
                app_battery_guard_trip(rawdata, APP_BATTERY_GUARD_REASON_LOW_VOLTAGE);
            }
            return;
        }
    }

    if((0U == battery_guard_sample_seen) &&
            ((uint32)(now - battery_guard_start_tick) >= APP_BATTERY_GUARD_STARTUP_GRACE_TICK))
    {
        app_battery_guard_trip(0U, APP_BATTERY_GUARD_REASON_STALE_SAMPLE);
    }
    else if((0U != battery_guard_sample_seen) &&
            ((uint32)(now - battery_guard_last_fresh_tick) >= APP_BATTERY_GUARD_MAX_SAMPLE_AGE_TICK))
    {
        app_battery_guard_trip(rawdata, APP_BATTERY_GUARD_REASON_STALE_SAMPLE);
    }
}

void app_battery_guard_init(void)
{
    battery_guard_triggered = 0U;
    battery_guard_report_pending = 0U;
    battery_guard_trip_raw = 0U;
    battery_guard_reason = 0U;
    battery_guard_last_sequence = 0UL;
    battery_guard_last_fresh_tick = service_timetick_what();
    battery_guard_start_tick = battery_guard_last_fresh_tick;
    battery_guard_sample_seen = 0U;
    pit_ms_init(APP_BATTERY_GUARD_PIT, APP_BATTERY_GUARD_PERIOD_MS, app_battery_guard_tick);
    interrupt_set_priority(TIM8_IRQn, 3U);
}

void app_battery_guard_pump_events(void)
{
}
