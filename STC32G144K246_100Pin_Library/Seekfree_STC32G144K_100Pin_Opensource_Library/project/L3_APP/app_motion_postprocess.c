#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "shared_pos_pid.h"
#include "service_imu.h"
#include "service_packet.h"
#include "app_feedforward.h"
#include "app_load_distribution.h"
#include "app_motion_preprocess.h"
#include "app_speedout.h"
#include "app_scheduler.h"
#include "app_motion_postprocess.h"

#define APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT     (1U)
#define APP_MOTION_POSTPROCESS_DEFAULT_KP              (0.29f)
#define APP_MOTION_POSTPROCESS_DEFAULT_KI              (0.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_KD              (0.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_INTEGRAL_LIMIT  (1.2f)
#define APP_MOTION_POSTPROCESS_DEFAULT_OUTPUT_LIMIT    (5.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_ENABLE          (1.0f)
#define APP_MOTION_POSTPROCESS_GYRO_LPF_ALPHA_DEFAULT  (0.5f)
#define APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD        (0.5f)
#define APP_MOTION_POSTPROCESS_DT_SECOND               ((float)APP_MOTION_POSTPROCESS_PERIOD_MS / 1000.0f)
#define APP_MOTION_POSTPROCESS_DEG_TO_RAD              (0.0174532925f)
#define APP_MOTION_POSTPROCESS_LEFT_STRAIGHT_SIGN      (-1.0f)
#define APP_MOTION_POSTPROCESS_RIGHT_STRAIGHT_SIGN     (1.0f)
#define APP_MOTION_POSTPROCESS_GYRO_TASK_ID            (1U)
#define APP_MOTION_POSTPROCESS_GYRO_TASK_PRIORITY      (10U)
#define APP_MOTION_POSTPROCESS_GYRO_PERIOD_MS          (1U)

app_motion_postprocess_config_t app_motion_postprocess_config =
{
    APP_MOTION_POSTPROCESS_DEFAULT_KP,
    APP_MOTION_POSTPROCESS_DEFAULT_KI,
    APP_MOTION_POSTPROCESS_DEFAULT_KD,
    APP_MOTION_POSTPROCESS_DEFAULT_INTEGRAL_LIMIT,
    APP_MOTION_POSTPROCESS_DEFAULT_OUTPUT_LIMIT,
    APP_MOTION_POSTPROCESS_DEFAULT_ENABLE
};

static volatile app_motion_postprocess_data_t motion_postprocess_data =
{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static shared_pos_pid_t motion_postprocess_yaw_pid;
static shared_lpf_t motion_postprocess_gyro_z_lpf;
static volatile float motion_postprocess_gyro_z_filtered = 0.0f;
static uint8 motion_postprocess_last_enabled = 0U;

static void app_motion_postprocess_task(void);
static void app_motion_postprocess_restart_pid(void);
static void app_motion_postprocess_gyro_task(void);

static float app_motion_postprocess_output_limit(void)
{
    if(0.0f < app_motion_postprocess_config.output_limit)
    {
        return app_motion_postprocess_config.output_limit;
    }

    return APP_MOTION_POSTPROCESS_DEFAULT_OUTPUT_LIMIT;
}

static void app_motion_postprocess_sync_pid(void)
{
    float output_limit;
    float integral_limit;

    output_limit = app_motion_postprocess_output_limit();
    integral_limit = app_motion_postprocess_config.integral_limit;

    motion_postprocess_yaw_pid.kp = app_motion_postprocess_config.kp;
    motion_postprocess_yaw_pid.ki = app_motion_postprocess_config.ki;
    motion_postprocess_yaw_pid.kd = app_motion_postprocess_config.kd;
    motion_postprocess_yaw_pid.output_limit_enable = ZF_ENABLE;
    motion_postprocess_yaw_pid.output_min = tfpu_sub(0.0f, output_limit);
    motion_postprocess_yaw_pid.output_max = output_limit;
    motion_postprocess_yaw_pid.integral_limit_enable = (0.0f < integral_limit) ? ZF_ENABLE : ZF_DISABLE;
    motion_postprocess_yaw_pid.integral_min = tfpu_sub(0.0f, integral_limit);
    motion_postprocess_yaw_pid.integral_max = integral_limit;
    motion_postprocess_yaw_pid.delta_limit_enable = ZF_DISABLE;
    motion_postprocess_yaw_pid.delta_min = 0.0f;
    motion_postprocess_yaw_pid.delta_max = 0.0f;
    motion_postprocess_yaw_pid.deadband = 0.0f;
    motion_postprocess_yaw_pid.setpoint_rate = 0.0f;
    motion_postprocess_yaw_pid.integral_separation = 0.0f;
    motion_postprocess_yaw_pid.conditional_integral_enable = ZF_ENABLE;
    motion_postprocess_yaw_pid.direction = SHARED_POS_PID_DIRECTION_REVERSE;
    motion_postprocess_yaw_pid.derivative_mode = SHARED_POS_PID_DERIVATIVE_ON_ERROR;
}

static void app_motion_postprocess_clear_pid(void)
{
    app_motion_postprocess_sync_pid();
    shared_pos_pid_reset(&motion_postprocess_yaw_pid, 0.0f);
}

static void app_motion_postprocess_restart_pid(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    app_motion_postprocess_sync_pid();
    shared_pos_pid_restart(&motion_postprocess_yaw_pid);
    EA = ea_backup;
}

static void app_motion_postprocess_register_packet(void)
{
    (void)service_packet_add_variable_with_callback("motion_post_kp",
            &app_motion_postprocess_config.kp, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT,
            app_motion_postprocess_restart_pid);
    (void)service_packet_add_variable_with_callback("motion_post_ki",
            &app_motion_postprocess_config.ki, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT,
            app_motion_postprocess_restart_pid);
    (void)service_packet_add_variable_with_callback("motion_post_kd",
            &app_motion_postprocess_config.kd, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT,
            app_motion_postprocess_restart_pid);
    (void)service_packet_add_variable_with_callback("motion_post_integral_limit",
            &app_motion_postprocess_config.integral_limit, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT,
            app_motion_postprocess_restart_pid);
    (void)service_packet_add_variable_with_callback("motion_post_output_limit",
            &app_motion_postprocess_config.output_limit, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT,
            app_motion_postprocess_restart_pid);
    (void)service_packet_add_variable("motion_post_enable",
            &app_motion_postprocess_config.enable, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT);
}

static void app_motion_postprocess_publish(app_motion_postprocess_data_t *output)
{
    uint8 ea_backup;

    if(NULL == output)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    motion_postprocess_data = *output;
    EA = ea_backup;
}

static void app_motion_postprocess_gyro_task(void)
{
    float raw_gyro;
    uint8 ea_backup;

    raw_gyro = service_imu_read_gyro_z();

    ea_backup = EA;
    EA = 0;
    motion_postprocess_gyro_z_filtered = shared_lpf_update(&motion_postprocess_gyro_z_lpf, raw_gyro);
    EA = ea_backup;
}

void app_motion_postprocess_init(void)
{
    float gyro_z;
    app_motion_postprocess_data_t output;

    gyro_z = service_imu_read_gyro_z();
    shared_lpf_init(&motion_postprocess_gyro_z_lpf,
            APP_MOTION_POSTPROCESS_GYRO_LPF_ALPHA_DEFAULT,
            gyro_z);
    motion_postprocess_gyro_z_filtered = gyro_z;
    app_motion_postprocess_sync_pid();
    shared_pos_pid_init(&motion_postprocess_yaw_pid);
    motion_postprocess_last_enabled =
            (app_motion_postprocess_config.enable >= APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD) ? 1U : 0U;

    output.raw_error = 0.0f;
    output.feedforward = 0.0f;
    output.processed_error = 0.0f;
    output.linear_mps = 0.0f;
    output.target_yaw_rate_radps = 0.0f;
    output.actual_yaw_rate_radps = 0.0f;
    output.target_differential_speed = 0.0f;
    output.load_left = 0.0f;
    output.load_right = 0.0f;
    output.left_target_mps = 0.0f;
    output.right_target_mps = 0.0f;
    output.enabled = (0U != motion_postprocess_last_enabled) ? 1.0f : 0.0f;
    app_motion_postprocess_publish(&output);

    app_motion_postprocess_register_packet();
    (void)app_scheduler_add(APP_MOTION_POSTPROCESS_GYRO_TASK_ID,
            app_motion_postprocess_gyro_task,
            APP_MOTION_POSTPROCESS_GYRO_TASK_PRIORITY,
            APP_MOTION_POSTPROCESS_GYRO_PERIOD_MS);
    app_motion_postprocess_task();
    pit_ms_init(APP_MOTION_POSTPROCESS_PIT,
            APP_MOTION_POSTPROCESS_PERIOD_MS,
            app_motion_postprocess_task);
}

void app_motion_postprocess_get_data(app_motion_postprocess_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    *out_data = motion_postprocess_data;
    EA = ea_backup;
}

static void app_motion_postprocess_task(void)
{
    uint8 enabled;
    float raw_error;
    float feedforward_value;
    float processed_error;
    float linear_mps;
    float target_yaw_rate_radps;
    float actual_yaw_rate_radps;
    float gyro_z;
    float target_differential_speed;
    app_motion_preprocess_data_t motion_preprocess;
    app_feedforward_data_t feedforward;
    app_load_distribution_data_t load_distribution;
    app_motion_postprocess_data_t output;

    app_motion_preprocess_get_data(&motion_preprocess);
    app_feedforward_get_data(&feedforward);
    gyro_z = motion_postprocess_gyro_z_filtered;

    raw_error = motion_preprocess.line_error;
    feedforward_value = feedforward.feedforward;
    processed_error = tfpu_add(raw_error, feedforward_value);
    linear_mps = motion_preprocess.linear_mps;
    target_yaw_rate_radps = tfpu_mul(app_motion_preprocess_config.yaw_rate_gain, processed_error);
    actual_yaw_rate_radps = tfpu_mul(gyro_z, APP_MOTION_POSTPROCESS_DEG_TO_RAD);
    enabled = (app_motion_postprocess_config.enable >= APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD) ? 1U : 0U;

    if(0U == enabled)
    {
        if(0U != motion_postprocess_last_enabled)
        {
            app_motion_postprocess_clear_pid();
        }
        target_differential_speed = 0.0f;
        motion_postprocess_last_enabled = 0U;
    }
    else
    {
        app_motion_postprocess_sync_pid();
        target_differential_speed = shared_pos_pid_update(&motion_postprocess_yaw_pid,
                target_yaw_rate_radps, actual_yaw_rate_radps, APP_MOTION_POSTPROCESS_DT_SECOND);
        motion_postprocess_last_enabled = 1U;
    }

    load_distribution.load_left = 0.0f;
    load_distribution.load_right = 0.0f;
    load_distribution.straight_current = linear_mps;
    load_distribution.turn_current = actual_yaw_rate_radps;
    load_distribution.Target_differential_speed = target_differential_speed;
    load_distribution.Target_Anglespeed = target_yaw_rate_radps;
    load_distribution.ay = 0.0f;
    load_distribution.LTC = 0.0f;
    app_load_distribution_process(&load_distribution);

    output.raw_error = raw_error;
    output.feedforward = feedforward_value;
    output.processed_error = processed_error;
    output.linear_mps = linear_mps;
    output.target_yaw_rate_radps = target_yaw_rate_radps;
    output.actual_yaw_rate_radps = actual_yaw_rate_radps;
    output.target_differential_speed = target_differential_speed;
    output.load_left = load_distribution.load_left;
    output.load_right = load_distribution.load_right;
    output.left_target_mps = tfpu_add(tfpu_mul(APP_MOTION_POSTPROCESS_LEFT_STRAIGHT_SIGN, linear_mps),
            output.load_left);
    output.right_target_mps = tfpu_add(tfpu_mul(APP_MOTION_POSTPROCESS_RIGHT_STRAIGHT_SIGN, linear_mps),
            output.load_right);
    output.enabled = (0U != enabled) ? 1.0f : 0.0f;

    app_speedout_set_target(output.left_target_mps, output.right_target_mps);
    app_motion_postprocess_publish(&output);
}
