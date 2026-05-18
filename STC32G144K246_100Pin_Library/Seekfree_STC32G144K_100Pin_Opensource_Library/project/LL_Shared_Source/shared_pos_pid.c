#include "sys_tfpu.h"
#include "shared_pos_pid.h"

static void shared_pos_pid_fix_config(shared_pos_pid_t *pid);
static void shared_pos_pid_fix_limit(float *min_value, float *max_value);
static float shared_pos_pid_abs(float value);
static float shared_pos_pid_clamp(float value, float min_value, float max_value);
static float shared_pos_pid_limit_output(const shared_pos_pid_t *pid, float output);
static float shared_pos_pid_limit_integral(const shared_pos_pid_t *pid, float integral);
static float shared_pos_pid_limit_delta(const shared_pos_pid_t *pid, float output);
static float shared_pos_pid_apply_deadband(float error, float deadband);
static float shared_pos_pid_get_direction_sign(const shared_pos_pid_t *pid);
static float shared_pos_pid_update_target(shared_pos_pid_t *pid, float target, float dt);
static float shared_pos_pid_update_common(shared_pos_pid_t *pid, float error, float feedback, float dt, uint8 feedback_valid);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化位置式 PID 控制器
// 参数说明     pid             PID 控制器对象指针
// 返回参数     void
// 使用示例     shared_pos_pid_init(&pid);
//-------------------------------------------------------------------------------------------------------------------
void shared_pos_pid_init(shared_pos_pid_t *pid)
{
    if(NULL == pid)
    {
        return;
    }

    shared_pos_pid_fix_config(pid);

    pid->output = shared_pos_pid_limit_output(pid, 0.0f);
    pid->ramp_target = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedback = 0.0f;
    pid->initialized = ZF_FALSE;
    pid->target_initialized = ZF_FALSE;
    pid->feedback_initialized = ZF_FALSE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位位置式 PID 控制器输出与历史状态
// 参数说明     pid             PID 控制器对象指针
// 参数说明     output          复位后的输出值
// 返回参数     void
// 使用示例     shared_pos_pid_reset(&pid, 0.0f);
//-------------------------------------------------------------------------------------------------------------------
void shared_pos_pid_reset(shared_pos_pid_t *pid, float output)
{
    if(NULL == pid)
    {
        return;
    }

    shared_pos_pid_fix_config(pid);

    pid->output = shared_pos_pid_limit_output(pid, output);
    pid->ramp_target = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedback = 0.0f;
    pid->initialized = ZF_FALSE;
    pid->target_initialized = ZF_FALSE;
    pid->feedback_initialized = ZF_FALSE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按指定目标和反馈复位位置式 PID 控制器状态
// 参数说明     pid             PID 控制器对象指针
// 参数说明     output          复位后的输出值
// 参数说明     target          当前目标值
// 参数说明     feedback        当前反馈值
// 返回参数     void
// 使用示例     shared_pos_pid_reset_state(&pid, 0.0f, target, feedback);
//-------------------------------------------------------------------------------------------------------------------
void shared_pos_pid_reset_state(shared_pos_pid_t *pid, float output, float target, float feedback)
{
    float error;
    float use_output;

    if(NULL == pid)
    {
        return;
    }

    shared_pos_pid_fix_config(pid);

    error = tfpu_mul(shared_pos_pid_get_direction_sign(pid), tfpu_sub(target, feedback));
    error = shared_pos_pid_apply_deadband(error, pid->deadband);
    use_output = shared_pos_pid_limit_output(pid, output);

    pid->output = use_output;
    pid->ramp_target = target;
    pid->integral = shared_pos_pid_limit_integral(pid, tfpu_sub(use_output, tfpu_mul(pid->kp, error)));
    pid->last_error = error;
    pid->last_feedback = feedback;
    pid->initialized = ZF_TRUE;
    pid->target_initialized = ZF_TRUE;
    pid->feedback_initialized = ZF_TRUE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过目标值与反馈值更新位置式 PID 控制器
// 参数说明     pid             PID 控制器对象指针
// 参数说明     target          目标值
// 参数说明     feedback        反馈值
// 参数说明     dt              距离上次更新的时间 单位秒
// 返回参数     float           本次 PID 输出值
// 使用示例     output = shared_pos_pid_update(&pid, target, feedback, 0.005f);
//-------------------------------------------------------------------------------------------------------------------
float shared_pos_pid_update(shared_pos_pid_t *pid, float target, float feedback, float dt)
{
    float use_target;
    float error;

    if(NULL == pid)
    {
        return 0.0f;
    }
    if(dt <= 0.0f)
    {
        return pid->output;
    }

    shared_pos_pid_fix_config(pid);

    use_target = shared_pos_pid_update_target(pid, target, dt);
    error = tfpu_mul(shared_pos_pid_get_direction_sign(pid), tfpu_sub(use_target, feedback));

    return shared_pos_pid_update_common(pid, error, feedback, dt, ZF_TRUE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过误差值更新位置式 PID 控制器
// 参数说明     pid             PID 控制器对象指针
// 参数说明     error           本次误差值
// 参数说明     dt              距离上次更新的时间 单位秒
// 返回参数     float           本次 PID 输出值
// 使用示例     output = shared_pos_pid_update_error(&pid, error, 0.005f);
//-------------------------------------------------------------------------------------------------------------------
float shared_pos_pid_update_error(shared_pos_pid_t *pid, float error, float dt)
{
    if(NULL == pid)
    {
        return 0.0f;
    }
    if(dt <= 0.0f)
    {
        return pid->output;
    }

    shared_pos_pid_fix_config(pid);

    pid->target_initialized = ZF_FALSE;
    pid->feedback_initialized = ZF_FALSE;

    return shared_pos_pid_update_common(pid, error, 0.0f, dt, ZF_FALSE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取位置式 PID 控制器当前输出值
// 参数说明     pid             PID 控制器对象指针
// 返回参数     float           当前输出值
// 使用示例     output = shared_pos_pid_get(&pid);
//-------------------------------------------------------------------------------------------------------------------
float shared_pos_pid_get(const shared_pos_pid_t *pid)
{
    if(NULL == pid)
    {
        return 0.0f;
    }

    return pid->output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     修正位置式 PID 控制器配置参数
// 参数说明     pid             PID 控制器对象指针
// 返回参数     void
// 使用示例     shared_pos_pid_fix_config(&pid);
//-------------------------------------------------------------------------------------------------------------------
static void shared_pos_pid_fix_config(shared_pos_pid_t *pid)
{
    if(NULL == pid)
    {
        return;
    }

    pid->output_limit_enable = pid->output_limit_enable ? ZF_ENABLE : ZF_DISABLE;
    pid->integral_limit_enable = pid->integral_limit_enable ? ZF_ENABLE : ZF_DISABLE;
    pid->delta_limit_enable = pid->delta_limit_enable ? ZF_ENABLE : ZF_DISABLE;
    pid->conditional_integral_enable = pid->conditional_integral_enable ? ZF_ENABLE : ZF_DISABLE;

    if(SHARED_POS_PID_DIRECTION_REVERSE != pid->direction)
    {
        pid->direction = SHARED_POS_PID_DIRECTION_DIRECT;
    }

    if(SHARED_POS_PID_DERIVATIVE_ON_ERROR != pid->derivative_mode)
    {
        pid->derivative_mode = SHARED_POS_PID_DERIVATIVE_ON_FEEDBACK;
    }

    if(pid->deadband < 0.0f)
    {
        pid->deadband = 0.0f;
    }
    if(pid->setpoint_rate < 0.0f)
    {
        pid->setpoint_rate = 0.0f;
    }
    if(pid->integral_separation < 0.0f)
    {
        pid->integral_separation = 0.0f;
    }

    shared_pos_pid_fix_limit(&pid->output_min, &pid->output_max);
    shared_pos_pid_fix_limit(&pid->integral_min, &pid->integral_max);
    shared_pos_pid_fix_limit(&pid->delta_min, &pid->delta_max);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     修正限幅上下限顺序
// 参数说明     min_value       最小值指针
// 参数说明     max_value       最大值指针
// 返回参数     void
// 使用示例     shared_pos_pid_fix_limit(&min_value, &max_value);
//-------------------------------------------------------------------------------------------------------------------
static void shared_pos_pid_fix_limit(float *min_value, float *max_value)
{
    float temp;

    if((NULL == min_value) || (NULL == max_value))
    {
        return;
    }

    if(*max_value < *min_value)
    {
        temp = *min_value;
        *min_value = *max_value;
        *max_value = temp;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取浮点数绝对值
// 参数说明     value           输入值
// 返回参数     float           绝对值
// 使用示例     value = shared_pos_pid_abs(value);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_abs(float value)
{
    if(value < 0.0f)
    {
        value = -value;
    }

    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     限制浮点数范围
// 参数说明     value           输入值
// 参数说明     min_value       最小值
// 参数说明     max_value       最大值
// 返回参数     float           限制后的值
// 使用示例     value = shared_pos_pid_clamp(value, min_value, max_value);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_clamp(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }
    else if(max_value < value)
    {
        value = max_value;
    }

    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据配置限制 PID 输出值
// 参数说明     pid             PID 控制器对象指针
// 参数说明     output          待限制的输出值
// 返回参数     float           限制后的输出值
// 使用示例     output = shared_pos_pid_limit_output(&pid, output);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_limit_output(const shared_pos_pid_t *pid, float output)
{
    if((NULL != pid) && (ZF_ENABLE == pid->output_limit_enable))
    {
        output = shared_pos_pid_clamp(output, pid->output_min, pid->output_max);
    }

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据配置限制 PID 积分项
// 参数说明     pid             PID 控制器对象指针
// 参数说明     integral        待限制的积分项
// 返回参数     float           限制后的积分项
// 使用示例     integral = shared_pos_pid_limit_integral(&pid, integral);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_limit_integral(const shared_pos_pid_t *pid, float integral)
{
    if((NULL != pid) && (ZF_ENABLE == pid->integral_limit_enable))
    {
        integral = shared_pos_pid_clamp(integral, pid->integral_min, pid->integral_max);
    }

    return integral;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据配置限制 PID 输出变化量
// 参数说明     pid             PID 控制器对象指针
// 参数说明     output          待限制的输出值
// 返回参数     float           限制后的输出值
// 使用示例     output = shared_pos_pid_limit_delta(&pid, output);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_limit_delta(const shared_pos_pid_t *pid, float output)
{
    float delta;

    if((NULL != pid) && (ZF_ENABLE == pid->delta_limit_enable))
    {
        delta = tfpu_sub(output, pid->output);
        delta = shared_pos_pid_clamp(delta, pid->delta_min, pid->delta_max);
        output = tfpu_add(pid->output, delta);
    }

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用 PID 误差死区
// 参数说明     error           原始误差
// 参数说明     deadband        死区范围
// 返回参数     float           处理后的误差
// 使用示例     error = shared_pos_pid_apply_deadband(error, deadband);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_apply_deadband(float error, float deadband)
{
    if((0.0f < deadband) && (shared_pos_pid_abs(error) <= deadband))
    {
        error = 0.0f;
    }

    return error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 PID 控制方向系数
// 参数说明     pid             PID 控制器对象指针
// 返回参数     float           控制方向系数
// 使用示例     sign = shared_pos_pid_get_direction_sign(&pid);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_get_direction_sign(const shared_pos_pid_t *pid)
{
    if((NULL != pid) && (SHARED_POS_PID_DIRECTION_REVERSE == pid->direction))
    {
        return -1.0f;
    }

    return 1.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新目标斜坡值
// 参数说明     pid             PID 控制器对象指针
// 参数说明     target          原始目标值
// 参数说明     dt              距离上次更新的时间 单位秒
// 返回参数     float           斜坡处理后的目标值
// 使用示例     target = shared_pos_pid_update_target(&pid, target, dt);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_update_target(shared_pos_pid_t *pid, float target, float dt)
{
    float max_step;
    float delta;

    if(NULL == pid)
    {
        return target;
    }

    if((ZF_FALSE == pid->target_initialized) || (pid->setpoint_rate <= 0.0f))
    {
        pid->ramp_target = target;
        pid->target_initialized = ZF_TRUE;
        return target;
    }

    max_step = tfpu_mul(pid->setpoint_rate, dt);
    delta = tfpu_sub(target, pid->ramp_target);

    if(shared_pos_pid_abs(delta) <= max_step)
    {
        pid->ramp_target = target;
    }
    else if(0.0f < delta)
    {
        pid->ramp_target = tfpu_add(pid->ramp_target, max_step);
    }
    else
    {
        pid->ramp_target = tfpu_sub(pid->ramp_target, max_step);
    }

    return pid->ramp_target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行位置式 PID 核心计算
// 参数说明     pid             PID 控制器对象指针
// 参数说明     error           本次误差值
// 参数说明     feedback        本次反馈值
// 参数说明     dt              距离上次更新的时间 单位秒
// 参数说明     feedback_valid  反馈值是否有效
// 返回参数     float           本次 PID 输出值
// 使用示例     output = shared_pos_pid_update_common(&pid, error, feedback, dt, ZF_TRUE);
//-------------------------------------------------------------------------------------------------------------------
static float shared_pos_pid_update_common(shared_pos_pid_t *pid, float error, float feedback, float dt, uint8 feedback_valid)
{
    uint8 first_update;
    uint8 integral_enable;
    float direction_sign;
    float p_output;
    float d_output;
    float i_delta;
    float output;

    if(NULL == pid)
    {
        return 0.0f;
    }

    first_update = (ZF_FALSE == pid->initialized) ? ZF_TRUE : ZF_FALSE;
    integral_enable = ZF_TRUE;
    direction_sign = shared_pos_pid_get_direction_sign(pid);

    error = shared_pos_pid_apply_deadband(error, pid->deadband);

    p_output = tfpu_mul(pid->kp, error);
    i_delta = tfpu_mul(tfpu_mul(pid->ki, error), dt);
    d_output = 0.0f;

    if((0.0f < pid->integral_separation) && (pid->integral_separation < shared_pos_pid_abs(error)))
    {
        integral_enable = ZF_FALSE;
    }

    if((ZF_ENABLE == pid->conditional_integral_enable) && (ZF_ENABLE == pid->output_limit_enable))
    {
        if(((pid->output_max <= pid->output) && (0.0f < i_delta)) ||
                ((pid->output <= pid->output_min) && (i_delta < 0.0f)))
        {
            integral_enable = ZF_FALSE;
        }
    }

    if(ZF_TRUE == integral_enable)
    {
        pid->integral = shared_pos_pid_limit_integral(pid, tfpu_add(pid->integral, i_delta));
    }

    if(ZF_FALSE == first_update)
    {
        if((ZF_TRUE == feedback_valid) && (SHARED_POS_PID_DERIVATIVE_ON_FEEDBACK == pid->derivative_mode))
        {
            if(ZF_TRUE == pid->feedback_initialized)
            {
                d_output = tfpu_div(tfpu_mul(tfpu_mul(tfpu_sub(0.0f, direction_sign), pid->kd),
                        tfpu_sub(feedback, pid->last_feedback)), dt);
            }
        }
        else
        {
            d_output = tfpu_div(tfpu_mul(pid->kd, tfpu_sub(error, pid->last_error)), dt);
        }
    }

    output = tfpu_add(tfpu_add(p_output, pid->integral), d_output);
    output = shared_pos_pid_limit_delta(pid, output);
    output = shared_pos_pid_limit_output(pid, output);
    pid->output = output;

    pid->last_error = error;
    if(ZF_TRUE == feedback_valid)
    {
        pid->last_feedback = feedback;
        pid->feedback_initialized = ZF_TRUE;
    }
    pid->initialized = ZF_TRUE;

    return pid->output;
}
