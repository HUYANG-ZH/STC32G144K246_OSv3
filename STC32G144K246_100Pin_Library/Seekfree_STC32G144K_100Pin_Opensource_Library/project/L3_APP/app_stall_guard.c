#include "zf_common_headfile.h"
#include "service_wireless_uart.h"
#include "app_speedout.h"
#include "app_stall_guard.h"

#define APP_STALL_GUARD_TICKS_PER_MS        (1U)      /* 跟随 TIM5 1ms 周期 */
#define APP_STALL_GUARD_HOLD_TICKS          (APP_STALL_GUARD_HOLD_MS / APP_STALL_GUARD_TICKS_PER_MS)
#define APP_STALL_GUARD_GRACE_TICKS         (APP_STALL_GUARD_STARTUP_GRACE_MS / APP_STALL_GUARD_TICKS_PER_MS)
#define APP_STALL_GUARD_PWM_LIMIT           (APP_STALL_GUARD_PWM_RATIO * (float)PWM_DUTY_MAX)

static uint16 stall_left_hold = 0U;                 /* 左轮连续满足计数(1ms tick) */
static uint16 stall_right_hold = 0U;                /* 右轮连续满足计数(1ms tick) */
static uint16 stall_grace_count = 0U;               /* 启动宽限 1ms tick 计数 */
static volatile uint8 stall_triggered = 0U;         /* 触发锁存, 防止重复触发刷屏 */
static volatile uint8 stall_report_pending = 0U;    /* 主循环待上报标志 */
static uint8 stall_last_enabled = 0U;               /* enabled 边沿检测 */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机堵转保护初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_stall_guard_init();
// 备注信息     由 main 在 app_speedout_init() 之后调用
//-------------------------------------------------------------------------------------------------------------------
void app_stall_guard_init(void)
{
    stall_left_hold = 0U;
    stall_right_hold = 0U;
    stall_grace_count = 0U;
    stall_triggered = 0U;
    stall_report_pending = 0U;
    stall_last_enabled = 0U;
    #if __DBGFLAG__
    printf(">>[app_stall_guard_init]\r\n");
    wprint(">>[app_stall_guard_init]\r\n");
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机堵转检测步进 (TIM5 1ms 中断上下文)
// 参数说明     void
// 返回参数     void
// 使用示例     app_speedout_tick() 内调用
// 备注信息     只读 app_speedout_data 快照; 触发时投递急停邮箱,
//              同帧随后的 apply_command 即执行急停, 与无线 stop 路径一致
//-------------------------------------------------------------------------------------------------------------------
void app_stall_guard_tick(void)
{
    app_speedout_data_t speedout;
    uint8 enabled;
    uint8 left_pwm_high;
    uint8 right_pwm_high;
    uint8 left_speed_low;
    uint8 right_speed_low;
    uint8 trigger = 0U;
    uint8 ea_backup;

    app_speedout_get_data(&speedout);
    enabled = (speedout.enabled >= 0.5f) ? 1U : 0U;

    if(0U == enabled)
    {
        /* 停车/未启动: 复位全部状态, 急停生效后自动解锁, 无线 start 可重新武装 */
        stall_left_hold = 0U;
        stall_right_hold = 0U;
        stall_grace_count = 0U;
        stall_triggered = 0U;
        stall_last_enabled = 0U;
        return;
    }

    if(0U != stall_triggered)
    {
        /* 已触发, 急停命令已投递; 等待 enabled 归零解锁, 避免 500ms 周期重复触发 */
        return;
    }

    if(0U == stall_last_enabled)
    {
        /* enabled 上升沿: 启动宽限期开始 */
        stall_grace_count = 0U;
    }
    stall_last_enabled = 1U;

    if(stall_grace_count < APP_STALL_GUARD_GRACE_TICKS)
    {
        stall_grace_count++;
        stall_left_hold = 0U;
        stall_right_hold = 0U;
        return;
    }

    /* 条件 A: |PWM| >= 80% 满占空比 */
    left_pwm_high = ((speedout.left_pwm >= APP_STALL_GUARD_PWM_LIMIT) ||
            (speedout.left_pwm <= -APP_STALL_GUARD_PWM_LIMIT)) ? 1U : 0U;
    right_pwm_high = ((speedout.right_pwm >= APP_STALL_GUARD_PWM_LIMIT) ||
            (speedout.right_pwm <= -APP_STALL_GUARD_PWM_LIMIT)) ? 1U : 0U;

    /* 条件 B: |实际速度| < 2.5m/s (编码器未建立速度) */
    left_speed_low = ((speedout.left_actual_mps < APP_STALL_GUARD_SPEED_MPS) &&
            (speedout.left_actual_mps > -APP_STALL_GUARD_SPEED_MPS)) ? 1U : 0U;
    right_speed_low = ((speedout.right_actual_mps < APP_STALL_GUARD_SPEED_MPS) &&
            (speedout.right_actual_mps > -APP_STALL_GUARD_SPEED_MPS)) ? 1U : 0U;

    /* 条件 C: 连续保持 0.5s; 任一帧不满足即清零重计 */
    if((0U != left_pwm_high) && (0U != left_speed_low))
    {
        stall_left_hold++;
        if(stall_left_hold >= APP_STALL_GUARD_HOLD_TICKS)
        {
            trigger = 1U;
        }
    }
    else
    {
        stall_left_hold = 0U;
    }

    if((0U != right_pwm_high) && (0U != right_speed_low))
    {
        stall_right_hold++;
        if(stall_right_hold >= APP_STALL_GUARD_HOLD_TICKS)
        {
            trigger = 1U;
        }
    }
    else
    {
        stall_right_hold = 0U;
    }

    if(0U != trigger)
    {
        ea_backup = EA;
        EA = 0;
        stall_triggered = 1U;
        stall_report_pending = 1U;
        EA = ea_backup;
        stall_left_hold = 0U;
        stall_right_hold = 0U;
        /* 与无线急停完全相同的操作: 由 TIM5 同帧 apply_command 执行
           (电机停止 + 负压立即归零) */
        app_speedout_request_stop_all();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     堵转事件泵 - 主循环调用
// 参数说明     void
// 返回参数     void
// 使用示例     while(1) { app_stall_guard_pump_events(); }
// 备注信息     输出 stall_alarm 报警帧与 stop 回报帧, 与无线急停回报一致
//-------------------------------------------------------------------------------------------------------------------
void app_stall_guard_pump_events(void)
{
    uint8 ea_backup;
    uint8 pending;

    ea_backup = EA;
    EA = 0;
    pending = stall_report_pending;
    stall_report_pending = 0U;
    EA = ea_backup;

    if(0U == pending)
    {
        return;
    }

    wprint("stall_alarm,1.000\r\n");
    wprint("stop,0.000\r\n");
}
