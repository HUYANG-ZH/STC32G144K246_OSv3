#include "zf_common_headfile.h"
#include "service_batterycheck.h"
#include "service_timetick.h"
#include "app_feedforward.h"
#include "app_log.h"
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

/* Formatting and telemetry are deliberately kept out of the safety ISR. */
void app_battery_guard_pump_events(void)
{
    uint8 ea_backup;
    uint8 report_pending;
    uint8 reason;
    uint16 trip_raw;
    float voltage;
    uint32 timestamp;
    app_motion_postprocess_data_t motion_post;
    app_speedout_data_t speedout;
    app_feedforward_data_t feedforward;

    ea_backup = EA;
    EA = 0;
    report_pending = battery_guard_report_pending;
    trip_raw = battery_guard_trip_raw;
    reason = battery_guard_reason;
    battery_guard_report_pending = 0U;
    EA = ea_backup;
    if(0U == report_pending)
    {
        return;
    }

    service_batterycheck_get_voltage(&voltage);
    timestamp = service_timetick_what();
    app_motion_postprocess_get_data(&motion_post);
    app_speedout_get_data(&speedout);
    app_feedforward_get_data(&feedforward);

    Wlog("UV,tick_0p1ms=%lu,reason=%u,raw=%u,v=%.3f\r\n", timestamp, reason, trip_raw, voltage);
    Wlog("MP,raw=%.3f,tar_y=%.3f,act_y=%.3f,diff=%.3f\r\n",
            motion_post.raw_error,
            motion_post.target_yaw_rate_radps,
            motion_post.actual_yaw_rate_radps,
            motion_post.target_differential_speed);
    Wlog("SO,lt=%.3f,la=%.3f,lp=%.3f,rt=%.3f,ra=%.3f,rp=%.3f\r\n",
            speedout.left_target_mps,
            speedout.left_actual_mps,
            speedout.left_pwm,
            speedout.right_target_mps,
            speedout.right_actual_mps,
            speedout.right_pwm);
    Wlog("FF,kff=%.3f,kd=%.3f,cur=%.3f,rate=%.3f,out=%.3f,post=%.3f\r\n",
            app_feedforward_config.kff,
            app_feedforward_config.kd,
            feedforward.curvature,
            feedforward.curvature_rate,
            feedforward.feedforward,
            motion_post.feedforward);
}
