#ifndef APP_STALL_GUARD_H
#define APP_STALL_GUARD_H

#include "zf_common_typedef.h"

/* 电机堵转(过载)保护:
   判定条件(左右轮独立, 任一满足即触发):
     1. 该侧 PID 输出占空比 |PWM| >= APP_STALL_GUARD_PWM_RATIO * PWM_DUTY_MAX
     2. 该侧实际速度 |v| < APP_STALL_GUARD_SPEED_MPS
     3. 上述两条件连续保持 APP_STALL_GUARD_HOLD_MS
   触发动作: 与无线急停($$T,stop@)完全一致
     - app_speedout_request_stop_all(): TIM5 执行电机停止 + 负压立即归零
     - wprint("stop,0.000\r\n") 回报帧
     - 附加 wprint("stall_alarm,1.000\r\n") 供上位机区分触发原因
   防误触发: 未启动(enabled<0.5)不检测; 启动后宽限
   APP_STALL_GUARD_STARTUP_GRACE_MS 内不计数; 触发后锁存, enabled 归零自动解锁。 */

#ifndef APP_STALL_GUARD_PWM_RATIO
#define APP_STALL_GUARD_PWM_RATIO          (0.80f)   /* PWM >= 80% 判高输出 */
#endif

#ifndef APP_STALL_GUARD_SPEED_MPS
#define APP_STALL_GUARD_SPEED_MPS          (2.5f)    /* |实际速度| < 2.5m/s 判未建立 */
#endif

#ifndef APP_STALL_GUARD_HOLD_MS
#define APP_STALL_GUARD_HOLD_MS            (500U)    /* 持续 0.5s 判堵转 */
#endif

#ifndef APP_STALL_GUARD_STARTUP_GRACE_MS
#define APP_STALL_GUARD_STARTUP_GRACE_MS   (300U)    /* 启动宽限, 防起步大油门误报 */
#endif

void app_stall_guard_init(void);
/* 由 TIM5(1ms) 速度环上下文调用; 只读 app_speedout_data, 不阻塞 */
void app_stall_guard_tick(void);
/* 主循环调用, 输出报警/stop 回报帧 */
void app_stall_guard_pump_events(void);

#endif
