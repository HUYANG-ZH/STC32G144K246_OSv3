#include "zf_common_headfile.h"
#include "shared_pos_pid.h"
#include "service_motor.h"
#include "service_packet.h"
#include "service_speed.h"
#include "app_speedout.h"

#define APP_SPEEDOUT_PACKET_SINGLE_COUNT     (1U)
#define APP_SPEEDOUT_ENABLE_THRESHOLD        (0.5f)
#define APP_SPEEDOUT_DT_SECOND               ((float)APP_SPEEDOUT_PERIOD_MS / 1000.0f)
#define APP_SPEEDOUT_TARGET_LIMIT_MPS        (10.0f)
#define APP_SPEEDOUT_DEFAULT_LEFT_KP         (1500.0f)
#define APP_SPEEDOUT_DEFAULT_LEFT_KI         (20000.0f)
#define APP_SPEEDOUT_DEFAULT_RIGHT_KP        (1500.0f)
#define APP_SPEEDOUT_DEFAULT_RIGHT_KI        (20000.0f)
#define APP_SPEEDOUT_DEFAULT_KD              (0.0f)
#define APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT  (0.0f)
#define APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT    ((float)PWM_DUTY_MAX)

app_speedout_config_t app_speedout_config =
{
    {0.0f, APP_SPEEDOUT_DEFAULT_LEFT_KP, APP_SPEEDOUT_DEFAULT_LEFT_KI,
            APP_SPEEDOUT_DEFAULT_KD, APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT, APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT},
    {0.0f, APP_SPEEDOUT_DEFAULT_RIGHT_KP, APP_SPEEDOUT_DEFAULT_RIGHT_KI,
            APP_SPEEDOUT_DEFAULT_KD, APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT, APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT}
};

app_speedout_data_t app_speedout_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static shared_pos_pid_t speedout_left_pid;
static shared_pos_pid_t speedout_right_pid;
static uint8 speedout_last_enabled = 0U;

static void app_speedout_tick(void);
static void app_speedout_restart_left_pid(void);
static void app_speedout_restart_right_pid(void);

static float app_speedout_output_limit(app_speedout_pid_config_t *config)
{
    float real_limit;

    real_limit = (0.0f < config->output_limit) ? config->output_limit : (float)PWM_DUTY_MAX;
    if(real_limit > (float)PWM_DUTY_MAX)
    {
        real_limit = (float)PWM_DUTY_MAX;
    }

    return real_limit;
}

static float app_speedout_limit_target(float target)
{
    if(target > APP_SPEEDOUT_TARGET_LIMIT_MPS)
    {
        return APP_SPEEDOUT_TARGET_LIMIT_MPS;
    }

    if(target < -APP_SPEEDOUT_TARGET_LIMIT_MPS)
    {
        return -APP_SPEEDOUT_TARGET_LIMIT_MPS;
    }

    return target;
}

static void app_speedout_sync_pid(shared_pos_pid_t *pid, app_speedout_pid_config_t *config)
{
    float output_limit;

    output_limit = app_speedout_output_limit(config);

    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->output_limit_enable = ZF_ENABLE;
    pid->output_min = -output_limit;
    pid->output_max = output_limit;
    pid->integral_limit_enable = (0.0f < config->integral_limit) ? ZF_ENABLE : ZF_DISABLE;
    pid->integral_min = -config->integral_limit;
    pid->integral_max = config->integral_limit;
    pid->delta_limit_enable = ZF_DISABLE;
    pid->delta_min = 0.0f;
    pid->delta_max = 0.0f;
    pid->deadband = 0.0f;
    pid->setpoint_rate = 0.0f;
    pid->integral_separation = 0.0f;
    pid->conditional_integral_enable = ZF_ENABLE;
    pid->direction = SHARED_POS_PID_DIRECTION_DIRECT;
    pid->derivative_mode = SHARED_POS_PID_DERIVATIVE_ON_ERROR;
}

static void app_speedout_clear_pid(void)
{
    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left);
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right);
    shared_pos_pid_reset(&speedout_left_pid, 0.0f);
    shared_pos_pid_reset(&speedout_right_pid, 0.0f);
}

static void app_speedout_restart_left_pid(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left);
    shared_pos_pid_restart(&speedout_left_pid);
    EA = ea_backup;
}

static void app_speedout_restart_right_pid(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right);
    shared_pos_pid_restart(&speedout_right_pid);
    EA = ea_backup;
}

static void app_speedout_register_packet(void)
{
    (void)service_packet_add_variable("speed_left_target", &app_speedout_config.left.target_mps, APP_SPEEDOUT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable_with_callback("speed_left_kp", &app_speedout_config.left.kp,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_left_pid);
    (void)service_packet_add_variable_with_callback("speed_left_ki", &app_speedout_config.left.ki,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_left_pid);
    (void)service_packet_add_variable("speed_right_target", &app_speedout_config.right.target_mps, APP_SPEEDOUT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable_with_callback("speed_right_kp", &app_speedout_config.right.kp,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_right_pid);
    (void)service_packet_add_variable_with_callback("speed_right_ki", &app_speedout_config.right.ki,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_right_pid);
    (void)service_packet_add_action("speed_stop", app_speedout_stop, 0UL);
}

void app_speedout_init(void)
{
    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left);
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right);
    shared_pos_pid_init(&speedout_left_pid);
    shared_pos_pid_init(&speedout_right_pid);

    app_speedout_config.left.target_mps = 0.0f;
    app_speedout_config.right.target_mps = 0.0f;
    app_speedout_data.left_target_mps = 0.0f;
    app_speedout_data.right_target_mps = 0.0f;
    app_speedout_data.left_actual_mps = 0.0f;
    app_speedout_data.right_actual_mps = 0.0f;
    app_speedout_data.left_pwm = 0.0f;
    app_speedout_data.right_pwm = 0.0f;
    app_speedout_data.enabled = 1.0f;
    speedout_last_enabled = 1U;

    app_speedout_register_packet();
    pit_ms_init(APP_SPEEDOUT_PIT, APP_SPEEDOUT_PERIOD_MS, app_speedout_tick);
}

void app_speedout_debug(void)
{
    app_speedout_data_t datas;

    app_speedout_get_data(&datas);
    printf("%.3f,%.3f,%.0f\r\n",
            datas.right_target_mps, datas.right_actual_mps, datas.right_pwm);
}

void app_speedout_stop(void)
{
    app_speedout_config.left.target_mps = 0.0f;
    app_speedout_config.right.target_mps = 0.0f;
    app_speedout_data.left_target_mps = 0.0f;
    app_speedout_data.right_target_mps = 0.0f;
    app_speedout_data.left_pwm = 0.0f;
    app_speedout_data.right_pwm = 0.0f;
    app_speedout_data.enabled = 0.0f;
    speedout_last_enabled = 0U;
    app_speedout_clear_pid();

    service_motor_stop();
}

void app_speedout_set_target(float left_mps, float right_mps)
{
    app_speedout_config.left.target_mps = left_mps;
    app_speedout_config.right.target_mps = right_mps;
    app_speedout_data.left_target_mps = left_mps;
    app_speedout_data.right_target_mps = right_mps;
}

void app_speedout_get_data(app_speedout_data_t *out_data)
{
    if(NULL == out_data)
    {
        return;
    }

    *out_data = app_speedout_data;
}

static void app_speedout_tick(void)
{
    uint8 enabled;
    int32 left_pwm;
    int32 right_pwm;
    float left_target;
    float right_target;
    service_speed_data_t speed;

    service_speed_get(&speed);
    app_speedout_config.left.kd = APP_SPEEDOUT_DEFAULT_KD;
    app_speedout_config.right.kd = APP_SPEEDOUT_DEFAULT_KD;
    app_speedout_config.left.integral_limit = APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT;
    app_speedout_config.right.integral_limit = APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT;
    app_speedout_config.left.output_limit = APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT;
    app_speedout_config.right.output_limit = APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT;
    app_speedout_config.left.target_mps = app_speedout_limit_target(app_speedout_config.left.target_mps);
    app_speedout_config.right.target_mps = app_speedout_limit_target(app_speedout_config.right.target_mps);
    left_target = app_speedout_config.left.target_mps;
    right_target = app_speedout_config.right.target_mps;

    enabled = (app_speedout_data.enabled >= APP_SPEEDOUT_ENABLE_THRESHOLD) ? 1U : 0U;
    if(0U == enabled)
    {
        if(0U != speedout_last_enabled)
        {
            app_speedout_clear_pid();
            service_motor_stop();
        }

        speedout_last_enabled = 0U;
        app_speedout_data.left_actual_mps = speed.left_mps;
        app_speedout_data.right_actual_mps = speed.right_mps;
        app_speedout_data.left_target_mps = left_target;
        app_speedout_data.right_target_mps = right_target;
        app_speedout_data.left_pwm = 0.0f;
        app_speedout_data.right_pwm = 0.0f;
        app_speedout_data.enabled = 0.0f;
        return;
    }

    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left);
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right);

    left_pwm = (int32)shared_pos_pid_update(&speedout_left_pid,
            left_target, speed.left_mps, APP_SPEEDOUT_DT_SECOND);
    left_pwm = -left_pwm;
    right_pwm = (int32)shared_pos_pid_update(&speedout_right_pid,
            right_target, speed.right_mps, APP_SPEEDOUT_DT_SECOND);

    service_motor_set_pwm(left_pwm, right_pwm);

    speedout_last_enabled = 1U;
    app_speedout_data.left_target_mps = left_target;
    app_speedout_data.right_target_mps = right_target;
    app_speedout_data.left_actual_mps = speed.left_mps;
    app_speedout_data.right_actual_mps = speed.right_mps;
    app_speedout_data.left_pwm = (float)left_pwm;
    app_speedout_data.right_pwm = (float)right_pwm;
    app_speedout_data.enabled = 1.0f;
}
