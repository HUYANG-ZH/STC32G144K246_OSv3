#ifndef APP_SPEEDOUT_H
#define APP_SPEEDOUT_H

#include "zf_common_typedef.h"

#ifndef APP_SPEEDOUT_PIT
#define APP_SPEEDOUT_PIT                 TIM5_PIT
#endif

#ifndef APP_SPEEDOUT_PERIOD_MS
#define APP_SPEEDOUT_PERIOD_MS           (1U)
#endif

#define APP_SPEEDOUT_SAFETY_BATTERY       (0x01U)
#define APP_SPEEDOUT_SAFETY_IMU           (0x02U)
#define APP_SPEEDOUT_SAFETY_INDUCTOR      (0x04U)

typedef struct
{
    float target_mps;
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} app_speedout_pid_config_t;

typedef struct
{
    app_speedout_pid_config_t left;
    app_speedout_pid_config_t right;
} app_speedout_config_t;

typedef struct
{
    float left_target_mps;
    float right_target_mps;
    float left_actual_mps;
    float right_actual_mps;
    float left_pwm;
    float right_pwm;
    float enabled;
} app_speedout_data_t;

extern volatile app_speedout_config_t app_speedout_config;
extern volatile app_speedout_data_t app_speedout_data;

void app_speedout_init(void);
void app_speedout_debug(void);
/* 非实时上下文只能写请求邮箱；实际电机/PWM 状态变化由 TIM5 速度环执行。 */
void app_speedout_request_stop(void);
void app_speedout_request_stop_all(void);
void app_speedout_request_start(void);
void app_speedout_set_safety_inhibit(uint8 mask);
void app_speedout_clear_safety_inhibit(uint8 mask);
uint8 app_speedout_get_safety_inhibit(void);
void app_speedout_set_target(float left_mps, float right_mps);
void app_speedout_get_data(app_speedout_data_t *out_data);

#endif
