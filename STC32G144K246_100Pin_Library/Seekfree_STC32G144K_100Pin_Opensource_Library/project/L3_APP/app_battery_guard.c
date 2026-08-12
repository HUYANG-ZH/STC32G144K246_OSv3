#include "zf_common_headfile.h"
#include "service_batterycheck.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "app_battery_guard.h"

/* TIM9: TIM8 已被 UART8 波特率发生器占用, 不可用作 PIT */
#define APP_BATTERY_GUARD_PIT                  TIM9_PIT
#define APP_BATTERY_GUARD_PERIOD_MS            (10U)
/* 欠压阈值 10.4V(等价 raw ≈ 1191), 比较的是发布链路的滤波后电压 */
#define APP_BATTERY_GUARD_LOW_VOLTAGE          (10.4f)
/* 采样失效判定需覆盖主循环可能的长时间阻塞(如开机蜂鸣 500ms 阻塞期),
   正常节拍 100ms, 500ms 内无新帧才判失效 */
#define APP_BATTERY_GUARD_STARTUP_GRACE_TICK    (3000UL)
#define APP_BATTERY_GUARD_MAX_SAMPLE_AGE_TICK   (5000UL)
/* 欠压判定开机宽限: 首个样本后 500ms 内不判欠压, 防上电分压/ADC 瞬态误报 */
#define APP_BATTERY_GUARD_LOW_VOLTAGE_GRACE_TICK (5000UL)

#define APP_BATTERY_GUARD_REASON_LOW_VOLTAGE    (1U)
#define APP_BATTERY_GUARD_REASON_STALE_SAMPLE   (2U)

static volatile uint8 battery_guard_triggered = 0U;
static volatile uint8 battery_guard_report_pending = 0U;
static volatile float battery_guard_trip_voltage = 0.0f;
static volatile uint8 battery_guard_reason = 0U;
static uint32 battery_guard_last_sequence = 0UL;
static uint32 battery_guard_last_fresh_tick = 0UL;
static uint32 battery_guard_start_tick = 0UL;
static uint32 battery_guard_first_fresh_tick = 0UL;
static uint8 battery_guard_sample_seen = 0U;

static void app_battery_guard_trip(float voltage, uint8 reason)
{
    if(0U != battery_guard_triggered)
    {
        return;
    }

    battery_guard_triggered = 1U;
    battery_guard_trip_voltage = voltage;
    battery_guard_reason = reason;
    battery_guard_report_pending = 1U;
    /* 仅报警, 不对车辆做任何动作(不下发联锁/不停机/不关负压) */
}

/* 欠压/采样失效检测: 仅上报无线报警, 不影响电机与负压 */
static void app_battery_guard_tick(void)
{
    uint16 rawdata = 0U;
    uint32 sequence;
    uint32 now;
    float voltage;

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
            if(0U == battery_guard_sample_seen)
            {
                battery_guard_first_fresh_tick = now;
                battery_guard_sample_seen = 1U;
            }
            /* 使用发布链路的滤波后电压做欠压判断;
               滤波未就绪(主循环尚未发布首帧, 值仍为初值 0.0)时跳过,
               避免开机阻塞期内误判欠压触发联锁 */
            voltage = service_batterycheck_get_filtered_voltage();
            if(0.0f > voltage)
            {
                /* 防御: 电压链全程非负, 负值必为异常, 不参与判定 */
                voltage = 0.0f;
            }
            if((0U != service_batterycheck_filter_ready()) &&
                    ((uint32)(now - battery_guard_first_fresh_tick) >= APP_BATTERY_GUARD_LOW_VOLTAGE_GRACE_TICK) &&
                    (voltage < APP_BATTERY_GUARD_LOW_VOLTAGE))
            {
                app_battery_guard_trip(voltage, APP_BATTERY_GUARD_REASON_LOW_VOLTAGE);
            }
            return;
        }
    }

    if((0U == battery_guard_sample_seen) &&
            ((uint32)(now - battery_guard_start_tick) >= APP_BATTERY_GUARD_STARTUP_GRACE_TICK))
    {
        app_battery_guard_trip(0.0f, APP_BATTERY_GUARD_REASON_STALE_SAMPLE);
    }
    else if((0U != battery_guard_sample_seen) &&
            ((uint32)(now - battery_guard_last_fresh_tick) >= APP_BATTERY_GUARD_MAX_SAMPLE_AGE_TICK))
    {
        app_battery_guard_trip(0.0f, APP_BATTERY_GUARD_REASON_STALE_SAMPLE);
    }
}

void app_battery_guard_init(void)
{
    battery_guard_triggered = 0U;
    battery_guard_report_pending = 0U;
    battery_guard_trip_voltage = 0.0f;
    battery_guard_reason = 0U;
    battery_guard_last_sequence = 0UL;
    battery_guard_last_fresh_tick = service_timetick_what();
    battery_guard_start_tick = battery_guard_last_fresh_tick;
    battery_guard_first_fresh_tick = 0UL;
    battery_guard_sample_seen = 0U;
    pit_ms_init(APP_BATTERY_GUARD_PIT, APP_BATTERY_GUARD_PERIOD_MS, app_battery_guard_tick);
    interrupt_set_priority(TIM9_IRQn, 3U);
}

void app_battery_guard_pump_events(void)
{
    uint8 ea_backup;
    uint8 pending;
    uint8 reason;
    float voltage;

    ea_backup = EA;
    EA = 0;
    pending = battery_guard_report_pending;
    battery_guard_report_pending = 0U;
    reason = battery_guard_reason;
    voltage = battery_guard_trip_voltage;
    EA = ea_backup;

    if(0U == pending)
    {
        return;
    }

    /* 仅无线报警: reason 1=欠压 2=采样失效, 附报警电压 */
    wprint("battery_alarm,1.000,%u,%.2f\r\n",
            (unsigned int)reason, (double)voltage);
}
