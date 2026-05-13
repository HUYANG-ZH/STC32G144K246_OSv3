#ifndef SHARED_PID_H
#define SHARED_PID_H

#include "zf_common_typedef.h"

typedef enum
{
    SHARED_PID_DIRECTION_DIRECT = 0,                                      // 正作用 输出增大时反馈增大
    SHARED_PID_DIRECTION_REVERSE,                                         // 反作用 输出增大时反馈减小
} shared_pid_direction_enum;

typedef enum
{
    SHARED_PID_DERIVATIVE_ON_FEEDBACK = 0,                                // 对反馈微分 减少目标突变带来的 D 项冲击
    SHARED_PID_DERIVATIVE_ON_ERROR,                                       // 对误差微分
} shared_pid_derivative_enum;

typedef struct
{
    float kp;                                                             // 比例系数
    float ki;                                                             // 积分系数
    float kd;                                                             // 微分系数

    uint8 output_limit_enable;                                            // 输出限幅使能 ZF_ENABLE/ZF_DISABLE
    float output_min;                                                     // 输出最小值
    float output_max;                                                     // 输出最大值

    uint8 delta_limit_enable;                                             // 单次输出增量限幅使能 ZF_ENABLE/ZF_DISABLE
    float delta_min;                                                      // 单次输出增量最小值
    float delta_max;                                                      // 单次输出增量最大值

    float deadband;                                                       // 死区 <= 0 时关闭
    float setpoint_rate;                                                  // 目标变化速率限制 单位/秒 <= 0 时关闭
    float integral_separation;                                            // 积分分离阈值 <= 0 时关闭
    uint8 conditional_integral_enable;                                    // 条件积分使能 ZF_ENABLE/ZF_DISABLE

    shared_pid_direction_enum direction;                                  // 控制方向
    shared_pid_derivative_enum derivative_mode;                           // 微分模式

    float output;                                                         // 当前输出
    float ramp_target;                                                    // 斜坡处理后的目标值
    float last_error;                                                     // 上次误差
    float prev_error;                                                     // 上上次误差
    float last_feedback;                                                  // 上次反馈
    float prev_feedback;                                                  // 上上次反馈
    uint8 initialized;                                                    // 状态初始化标志
    uint8 target_initialized;                                             // 目标斜坡初始化标志
    uint8 feedback_initialized;                                           // 反馈历史初始化标志
} shared_pid_t;

void shared_pid_init(shared_pid_t *pid);
void shared_pid_reset(shared_pid_t *pid, float output);
void shared_pid_reset_state(shared_pid_t *pid, float output, float target, float feedback);
float shared_pid_update(shared_pid_t *pid, float target, float feedback, float dt);
float shared_pid_update_error(shared_pid_t *pid, float error, float dt);
float shared_pid_get(const shared_pid_t *pid);

#endif
