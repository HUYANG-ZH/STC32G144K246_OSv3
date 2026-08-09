#include "zf_common_headfile.h"
#include "shared_pos_pid.h"
#include "service_motor.h"
#include "service_negative_pressure.h"
#include "service_packet.h"
#include "service_speed.h"
#include "service_wireless_uart.h"
#include "app_speedout.h"

#define APP_SPEEDOUT_PACKET_SINGLE_COUNT     (1U)
#define APP_SPEEDOUT_ENABLE_THRESHOLD        (0.5f)
#define APP_SPEEDOUT_DT_SECOND               ((float)APP_SPEEDOUT_PERIOD_MS / 1000.0f)
#define APP_SPEEDOUT_TARGET_LIMIT_MPS        (10.0f)
#define APP_SPEEDOUT_DEFAULT_LEFT_KP         (1570.0f)
#define APP_SPEEDOUT_DEFAULT_LEFT_KI         (40000.0f)
#define APP_SPEEDOUT_DEFAULT_RIGHT_KP        (1570.0f)
#define APP_SPEEDOUT_DEFAULT_RIGHT_KI        (40000.0f)
#define APP_SPEEDOUT_DEFAULT_KD              (0.0f)
#define APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT  (0.0f)
#define APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT    ((float)PWM_DUTY_MAX)
#define APP_SPEEDOUT_RIGHT_TARGET_SIGN       (1.0f)
#define APP_SPEEDOUT_LEFT_PID_DIRECTION      SHARED_POS_PID_DIRECTION_REVERSE
#define APP_SPEEDOUT_RIGHT_PID_DIRECTION     SHARED_POS_PID_DIRECTION_DIRECT

volatile app_speedout_config_t app_speedout_config =
{
    {0.0f, APP_SPEEDOUT_DEFAULT_LEFT_KP, APP_SPEEDOUT_DEFAULT_LEFT_KI,
            APP_SPEEDOUT_DEFAULT_KD, APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT, APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT},
    {0.0f, APP_SPEEDOUT_DEFAULT_RIGHT_KP, APP_SPEEDOUT_DEFAULT_RIGHT_KI,
            APP_SPEEDOUT_DEFAULT_KD, APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT, APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT}
};

volatile app_speedout_data_t app_speedout_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static shared_pos_pid_t speedout_left_pid;
static shared_pos_pid_t speedout_right_pid;
static uint8 speedout_last_enabled = 0U;

typedef enum
{
    APP_SPEEDOUT_COMMAND_NONE = 0,
    APP_SPEEDOUT_COMMAND_STOP,
    APP_SPEEDOUT_COMMAND_STOP_ALL,
    APP_SPEEDOUT_COMMAND_START,
} app_speedout_command_enum;

typedef struct
{
    float left_mps;
    float right_mps;
    uint32 sequence;
    uint8 owner;
} app_speedout_target_mailbox_t;

#define APP_SPEEDOUT_TARGET_OWNER_NONE        (0U)
#define APP_SPEEDOUT_TARGET_OWNER_CONTROL     (1U)
#define APP_SPEEDOUT_TARGET_OWNER_PACKET      (2U)

static volatile app_speedout_command_enum speedout_command = APP_SPEEDOUT_COMMAND_NONE;
static volatile app_speedout_target_mailbox_t xdata speedout_target_mailbox;
static float speedout_active_left_target = 0.0f;
static float speedout_active_right_target = 0.0f;
static uint32 speedout_target_applied_sequence = 0UL;
static uint8 speedout_target_active_owner = APP_SPEEDOUT_TARGET_OWNER_NONE;
static volatile uint8 speedout_safety_inhibit = 0U;

static void app_speedout_tick(void);
static void app_speedout_restart_left_pid(void);
static void app_speedout_restart_right_pid(void);
static void app_speedout_stop_all_action(void);
static void app_speedout_start_action(void);
static void app_speedout_apply_command(void);
static void app_speedout_start(void);
static void app_speedout_stop(void);
static void app_speedout_publish_target(float left_mps, float right_mps, uint8 owner);
static void app_speedout_packet_target_changed(void);
static void app_speedout_consume_target(void);

static float app_speedout_output_limit(volatile app_speedout_pid_config_t *config)
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

static void app_speedout_sync_pid(shared_pos_pid_t *pid, volatile app_speedout_pid_config_t *config,
        shared_pos_pid_direction_enum direction)
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
    pid->direction = direction;
    pid->derivative_mode = SHARED_POS_PID_DERIVATIVE_ON_ERROR;
}

static void app_speedout_clear_pid(void)
{
    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left, APP_SPEEDOUT_LEFT_PID_DIRECTION);
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right, APP_SPEEDOUT_RIGHT_PID_DIRECTION);
    shared_pos_pid_reset(&speedout_left_pid, 0.0f);
    shared_pos_pid_reset(&speedout_right_pid, 0.0f);
}

static void app_speedout_restart_left_pid(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left, APP_SPEEDOUT_LEFT_PID_DIRECTION);
    shared_pos_pid_restart(&speedout_left_pid);
    EA = ea_backup;
}

static void app_speedout_restart_right_pid(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right, APP_SPEEDOUT_RIGHT_PID_DIRECTION);
    shared_pos_pid_restart(&speedout_right_pid);
    EA = ea_backup;
}

static void app_speedout_register_packet(void)
{
    (void)service_packet_add_variable_with_callback("SLT", (float *)&app_speedout_config.left.target_mps,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_packet_target_changed);
    (void)service_packet_add_variable_with_callback("SLK", (float *)&app_speedout_config.left.kp,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_left_pid);
    (void)service_packet_add_variable_with_callback("SLI", (float *)&app_speedout_config.left.ki,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_left_pid);
    (void)service_packet_add_variable_with_callback("SRT", (float *)&app_speedout_config.right.target_mps,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_packet_target_changed);
    (void)service_packet_add_variable_with_callback("SRK", (float *)&app_speedout_config.right.kp,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_right_pid);
    (void)service_packet_add_variable_with_callback("SRI", (float *)&app_speedout_config.right.ki,
            APP_SPEEDOUT_PACKET_SINGLE_COUNT, app_speedout_restart_right_pid);
    (void)service_packet_add_action("stop", app_speedout_stop_all_action, 0UL);
    (void)service_packet_add_action("start", app_speedout_start_action, 0UL);
}

void app_speedout_init(void)
{
    app_speedout_config.left.kd = APP_SPEEDOUT_DEFAULT_KD;
    app_speedout_config.right.kd = APP_SPEEDOUT_DEFAULT_KD;
    app_speedout_config.left.integral_limit = APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT;
    app_speedout_config.right.integral_limit = APP_SPEEDOUT_DEFAULT_INTEGRAL_LIMIT;
    app_speedout_config.left.output_limit = APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT;
    app_speedout_config.right.output_limit = APP_SPEEDOUT_DEFAULT_OUTPUT_LIMIT;
    app_speedout_sync_pid(&speedout_left_pid, &app_speedout_config.left, APP_SPEEDOUT_LEFT_PID_DIRECTION);
    app_speedout_sync_pid(&speedout_right_pid, &app_speedout_config.right, APP_SPEEDOUT_RIGHT_PID_DIRECTION);
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
    app_speedout_data.enabled = 0.0f;
    speedout_last_enabled = 0U;
    speedout_command = APP_SPEEDOUT_COMMAND_NONE;
    speedout_target_mailbox.left_mps = 0.0f;
    speedout_target_mailbox.right_mps = 0.0f;
    speedout_target_mailbox.sequence = 0UL;
    speedout_target_mailbox.owner = APP_SPEEDOUT_TARGET_OWNER_NONE;
    speedout_active_left_target = 0.0f;
    speedout_active_right_target = 0.0f;
    speedout_target_applied_sequence = 0UL;
    speedout_target_active_owner = APP_SPEEDOUT_TARGET_OWNER_NONE;
    speedout_safety_inhibit = 0U;

    app_speedout_register_packet();
    pit_ms_init(APP_SPEEDOUT_PIT, APP_SPEEDOUT_PERIOD_MS, app_speedout_tick);
    interrupt_set_priority(TIM5_IRQn, 3U);
    #if __DBGFLAG__
    printf(">>[app_speedout_init]\r\n");
    wprint(">>[app_speedout_init]\r\n");
    #endif
}

void app_speedout_debug(void)
{
    app_speedout_data_t datas;

    app_speedout_get_data(&datas);
    printf("%.3f,%.3f,%.3f,%.3f\r\n",
            datas.left_target_mps, datas.right_target_mps,
            datas.left_actual_mps, datas.right_actual_mps);
}

static void app_speedout_stop(void)
{
    speedout_active_left_target = 0.0f;
    speedout_active_right_target = 0.0f;
    app_speedout_data.left_target_mps = 0.0f;
    app_speedout_data.right_target_mps = 0.0f;
    app_speedout_data.left_pwm = 0.0f;
    app_speedout_data.right_pwm = 0.0f;
    app_speedout_data.enabled = 0.0f;
    speedout_last_enabled = 0U;
    app_speedout_clear_pid();

    service_motor_stop();
}

void app_speedout_request_stop_all(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    speedout_command = APP_SPEEDOUT_COMMAND_STOP_ALL;
    EA = ea_backup;
}

void app_speedout_request_start(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    speedout_command = APP_SPEEDOUT_COMMAND_START;
    EA = ea_backup;
}

void app_speedout_set_safety_inhibit(uint8 mask)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    speedout_safety_inhibit |= mask;
    EA = ea_backup;
}

void app_speedout_clear_safety_inhibit(uint8 mask)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    speedout_safety_inhibit &= (uint8)(~mask);
    EA = ea_backup;
}

uint8 app_speedout_get_safety_inhibit(void)
{
    uint8 ea_backup;
    uint8 result;

    ea_backup = EA;
    EA = 0;
    result = speedout_safety_inhibit;
    EA = ea_backup;
    return result;
}

static void app_speedout_stop_all_action(void)
{
    app_speedout_request_stop_all();
    wprint("stop,0.000\r\n");
}

static void app_speedout_start_action(void)
{
    app_speedout_request_start();
    wprint("start,0.000\r\n");
}

static void app_speedout_start(void)
{
    speedout_active_left_target = 0.0f;
    speedout_active_right_target = 0.0f;
    app_speedout_data.left_target_mps = 0.0f;
    app_speedout_data.right_target_mps = 0.0f;
    app_speedout_data.left_pwm = 0.0f;
    app_speedout_data.right_pwm = 0.0f;
    app_speedout_data.enabled = 1.0f;
    speedout_last_enabled = 1U;
    app_speedout_clear_pid();

    service_motor_stop();
}

void app_speedout_set_target(float left_mps, float right_mps)
{
    app_speedout_publish_target(left_mps, right_mps, APP_SPEEDOUT_TARGET_OWNER_CONTROL);
}

static void app_speedout_publish_target(float left_mps, float right_mps, uint8 owner)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    speedout_target_mailbox.left_mps = left_mps;
    speedout_target_mailbox.right_mps = right_mps;
    speedout_target_mailbox.owner = owner;
    speedout_target_mailbox.sequence++;
    EA = ea_backup;
}

static void app_speedout_packet_target_changed(void)
{
    float left_mps;
    float right_mps;
    uint8 ea_backup;

    /* A packet may update either field independently, but TIM5 receives one
       coherent pair rather than observing two in-place float stores. */
    ea_backup = EA;
    EA = 0;
    left_mps = app_speedout_config.left.target_mps;
    right_mps = app_speedout_config.right.target_mps;
    EA = ea_backup;
    app_speedout_publish_target(left_mps, right_mps, APP_SPEEDOUT_TARGET_OWNER_PACKET);
}

static void app_speedout_consume_target(void)
{
    app_speedout_target_mailbox_t snapshot;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    snapshot.left_mps = speedout_target_mailbox.left_mps;
    snapshot.right_mps = speedout_target_mailbox.right_mps;
    snapshot.sequence = speedout_target_mailbox.sequence;
    snapshot.owner = speedout_target_mailbox.owner;
    EA = ea_backup;

    if(snapshot.sequence != speedout_target_applied_sequence)
    {
        speedout_active_left_target = app_speedout_limit_target(snapshot.left_mps);
        speedout_active_right_target = app_speedout_limit_target(snapshot.right_mps);
        speedout_target_applied_sequence = snapshot.sequence;
        speedout_target_active_owner = snapshot.owner;
    }
}

void app_speedout_get_data(app_speedout_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    *out_data = app_speedout_data;
    EA = ea_backup;
}

static void app_speedout_tick(void)
{
    uint8 enabled;
    int32 left_pwm;
    int32 right_pwm;
    float left_target;
    float right_target;
    service_speed_data_t speed;

    app_speedout_consume_target();
    app_speedout_apply_command();
    /* All runtime pressure PWM updates share TIM5 with motor actuation. */
    service_negative_pressure_apply_request();

    service_speed_get(&speed);
    left_target = speedout_active_left_target;
    right_target = speedout_active_right_target * APP_SPEEDOUT_RIGHT_TARGET_SIGN;

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

    left_pwm = (int32)shared_pos_pid_update(&speedout_left_pid,
            left_target, speed.left_mps, APP_SPEEDOUT_DT_SECOND);
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

static void app_speedout_apply_command(void)
{
    app_speedout_command_enum command;
    uint8 ea_backup;
    uint8 safety_inhibit;

    ea_backup = EA;
    EA = 0;
    command = speedout_command;
    speedout_command = APP_SPEEDOUT_COMMAND_NONE;
    safety_inhibit = speedout_safety_inhibit;
    EA = ea_backup;

    if(0U != safety_inhibit)
    {
        /* Safety faults dominate START and remain active until their owner
           explicitly clears the relevant interlock bit. */
        app_speedout_stop();
        service_negative_pressure_set_percent(0U);
        service_negative_pressure_apply_request();
    }
    else if(APP_SPEEDOUT_COMMAND_STOP == command)
    {
        app_speedout_stop();
    }
    else if(APP_SPEEDOUT_COMMAND_STOP_ALL == command)
    {
        app_speedout_stop();
        service_negative_pressure_set_percent(0U);
        service_negative_pressure_apply_request();
    }
    else if(APP_SPEEDOUT_COMMAND_START == command)
    {
        app_speedout_start();
    }
}
