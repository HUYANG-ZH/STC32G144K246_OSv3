#include "zf_common_headfile.h"
#include "service_batterycheck.h"
#include "service_negative_pressure.h"
#include "service_timetick.h"
#include "app_feedforward.h"
#include "app_log.h"
#include "app_motion_postprocess.h"
#include "app_scheduler.h"
#include "app_speedout.h"
#include "app_battery_guard.h"

#define APP_BATTERY_GUARD_TASK_ID              (3U)
#define APP_BATTERY_GUARD_TASK_PRIORITY        (9U)
#define APP_BATTERY_GUARD_PERIOD_MS            (10U)
#define APP_BATTERY_GUARD_LOW_VOLTAGE          (11.0f)

static uint8 battery_guard_triggered = 0U;

static void app_battery_guard_task(void)
{
    float voltage;
    uint32 timestamp;
    app_motion_postprocess_data_t motion_post;
    app_speedout_data_t speedout;
    app_feedforward_data_t feedforward;

    if(0U != battery_guard_triggered)
    {
        return;
    }

    service_batterycheck_get_voltage(&voltage);
    if(voltage >= APP_BATTERY_GUARD_LOW_VOLTAGE)
    {
        return;
    }

    battery_guard_triggered = 1U;
    timestamp = service_timetick_what();
    app_motion_postprocess_get_data(&motion_post);
    app_speedout_get_data(&speedout);
    app_feedforward_get_data(&feedforward);

    Wlog("UV,tick_0p1ms=%lu,v=%.3f\r\n", timestamp, voltage);
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

    app_speedout_stop();
    service_negative_pressure_set_percent(0U);
}

void app_battery_guard_init(void)
{
    battery_guard_triggered = 0U;
    (void)app_scheduler_add(APP_BATTERY_GUARD_TASK_ID,
            app_battery_guard_task,
            APP_BATTERY_GUARD_TASK_PRIORITY,
            APP_BATTERY_GUARD_PERIOD_MS);
}
