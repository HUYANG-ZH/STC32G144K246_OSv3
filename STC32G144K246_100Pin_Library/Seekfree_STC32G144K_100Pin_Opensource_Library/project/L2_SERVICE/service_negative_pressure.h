#ifndef SERVICE_NEGATIVE_PRESSURE_H
#define SERVICE_NEGATIVE_PRESSURE_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

#ifndef SERVICE_NEGATIVE_PRESSURE_PWM_CH
#define SERVICE_NEGATIVE_PRESSURE_PWM_CH        PWMD_CH1_PB4
#endif

#ifndef SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ
#define SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ   (50U)
#endif

void service_negative_pressure_init(void);
void service_negative_pressure_debug(void);
void service_negative_pressure_task(void);
void service_negative_pressure_request_percent(uint8 percent);
/* Call only from the deterministic actuator timer (TIM5 in this project). */
void service_negative_pressure_apply_request(void);
void service_negative_pressure_set_percent(uint8 percent);
/* 安全路径专用: 立即切到目标值(绕过缓启动斜坡), 仅限急停/故障抑制场景 */
void service_negative_pressure_set_percent_immediate(uint8 percent);
uint8 service_negative_pressure_get_percent(void);
/* 读取无线配置的基础量(0~100), 只读不改动变量机制 */
uint8 service_negative_pressure_get_config_percent(void);

#endif
