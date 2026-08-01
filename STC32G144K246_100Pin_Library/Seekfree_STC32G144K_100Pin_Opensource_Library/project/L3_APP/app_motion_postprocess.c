#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "shared_pos_pid.h"
#include "service_imu.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "app_attitude.h"
#include "app_element.h"
#include "app_feedforward.h"
#include "app_motion_preprocess.h"
#include "app_speed_plan.h"
#include "app_speedout.h"
#include "app_motion_postprocess.h"
#include "service_wireless_uart.h"

#define APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT     (1U)
#define APP_MOTION_POSTPROCESS_DEFAULT_KP              (0.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_KI              (0.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_KD              (0.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_INTEGRAL_LIMIT  (1.2f)
#define APP_MOTION_POSTPROCESS_DEFAULT_OUTPUT_LIMIT    (5.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_ENABLE          (1.0f)
#define APP_MOTION_POSTPROCESS_DEFAULT_RATE_LIMIT      (0.0f)      // 角速度目标变化率限幅 rad/s²，0表示关闭
#define APP_MOTION_POSTPROCESS_GYRO_LPF_ALPHA_DEFAULT  (0.5f)
#define APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD        (0.5f)
#define APP_MOTION_POSTPROCESS_DT_SECOND               ((float)APP_MOTION_POSTPROCESS_PERIOD_MS / 1000.0f)
#define APP_MOTION_POSTPROCESS_DEG_TO_RAD              (0.0174532925f)
#define APP_MOTION_POSTPROCESS_LEFT_STRAIGHT_SIGN      (1.0f)
#define APP_MOTION_POSTPROCESS_RIGHT_STRAIGHT_SIGN     (1.0f)
#define APP_MOTION_POSTPROCESS_DIFF_HALF               (0.5f)
#define APP_MOTION_POSTPROCESS_FEEDFORWARD_SIGN        (-1.0f)
#define APP_MOTION_POSTPROCESS_IMU_STARTUP_GRACE_TICK  (1000UL)
#define APP_MOTION_POSTPROCESS_IMU_MAX_AGE_TICK        (500UL)

app_motion_postprocess_config_t app_motion_postprocess_config =
{
    APP_MOTION_POSTPROCESS_DEFAULT_KP,
    APP_MOTION_POSTPROCESS_DEFAULT_KI,
    APP_MOTION_POSTPROCESS_DEFAULT_KD,
    APP_MOTION_POSTPROCESS_DEFAULT_INTEGRAL_LIMIT,
    APP_MOTION_POSTPROCESS_DEFAULT_OUTPUT_LIMIT,
    APP_MOTION_POSTPROCESS_DEFAULT_ENABLE,
    APP_MOTION_POSTPROCESS_DEFAULT_RATE_LIMIT
};

static volatile app_motion_postprocess_data_t motion_postprocess_data =
{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static shared_pos_pid_t motion_postprocess_yaw_pid;
static shared_lpf_t motion_postprocess_gyro_z_lpf;
static volatile float motion_postprocess_gyro_z_filtered = 0.0f;
static uint32 motion_postprocess_imu_last_sequence = 0U;
static uint32 motion_postprocess_imu_last_fresh_tick = 0UL;
static uint32 motion_postprocess_imu_start_tick = 0UL;
static uint8 motion_postprocess_imu_seen = 0U;
static uint8 motion_postprocess_imu_fault = 0U;
static uint8 motion_postprocess_last_enabled = 0U;
static float motion_post_target_yaw_rate_override = 0.0f;
static float motion_postprocess_yaw_debug_enable = 0.0f;
static float motion_postprocess_rate_limited_yaw_rate = 0.0f;
static uint8 motion_postprocess_rate_limit_ready = 0U;

void app_motion_postprocess_control_step(void);
static void app_motion_postprocess_compute_step(void);
static void app_motion_postprocess_restart_pid(void);
void app_motion_postprocess_imu_step(void);

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
    motion_postprocess_yaw_pid.derivative_mode = SHARED_POS_PID_DERIVATIVE_ON_FEEDBACK;
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
    (void)service_packet_add_variable("motion_post_target_yaw_rate",
            &motion_post_target_yaw_rate_override, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("motion_post_yaw_debug",
            &motion_postprocess_yaw_debug_enable, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("motion_post_rate_limit",
            &app_motion_postprocess_config.rate_limit, APP_MOTION_POSTPROCESS_PACKET_SINGLE_COUNT);
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

void app_motion_postprocess_imu_step(void)
{
    service_imu_sample_t imu;
    uint8 ea_backup;
    uint8 imu_available;
    uint32 now;

    service_imu_update();
    now = service_timetick_what();
    imu_available = service_imu_get_latest_sample(&imu);
    if(0U == imu_available)
    {
        imu.gyro_x = 0.0f;
        imu.gyro_y = 0.0f;
        imu.gyro_z = 0.0f;
        imu.sequence = 0UL;
        imu.valid = 0U;
    }

    /*
     * SPI DMA may complete slower than the 1 ms control tick.  A stale frame
     * must not be filtered repeatedly: doing so changes the LPF state without
     * any new physical observation and obscures a stalled acquisition chain.
     */
    if((0U != imu.valid) && (0U != imu.sequence) &&
       (imu.sequence != motion_postprocess_imu_last_sequence))
    {
        ea_backup = EA;
        EA = 0;
        motion_postprocess_gyro_z_filtered = shared_lpf_update(&motion_postprocess_gyro_z_lpf, imu.gyro_z);
        EA = ea_backup;
        motion_postprocess_imu_last_sequence = imu.sequence;
        motion_postprocess_imu_last_fresh_tick = imu.timestamp_tick;
        motion_postprocess_imu_seen = 1U;
        if(0U != motion_postprocess_imu_fault)
        {
            motion_postprocess_imu_fault = 0U;
            app_speedout_clear_safety_inhibit(APP_SPEEDOUT_SAFETY_IMU);
        }
    }

    if(((0U == motion_postprocess_imu_seen) &&
            ((uint32)(now - motion_postprocess_imu_start_tick) >= APP_MOTION_POSTPROCESS_IMU_STARTUP_GRACE_TICK)) ||
       ((0U != motion_postprocess_imu_seen) &&
            ((uint32)(now - motion_postprocess_imu_last_fresh_tick) >= APP_MOTION_POSTPROCESS_IMU_MAX_AGE_TICK)))
    {
        if(0U == motion_postprocess_imu_fault)
        {
            motion_postprocess_imu_fault = 1U;
            app_speedout_set_safety_inhibit(APP_SPEEDOUT_SAFETY_IMU);
        }
    }
    /* 角速度环调试模式下关闭元素识别, 防止台架旋转触发圆筒/环岛状态机 */
    if(motion_postprocess_yaw_debug_enable < APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD)
    {
        app_element_imu_task(&imu);
    }
    if(0U != imu_available)
    {
        app_attitude_update(&imu);
    }
}

void app_motion_postprocess_init(void)
{
    service_imu_sample_t imu;
    app_motion_postprocess_data_t output;
    uint32 now;

    if(0U == service_imu_get_latest_sample(&imu))
    {
        imu.gyro_z = 0.0f;
        imu.sequence = 0UL;
        imu.valid = 0U;
    }
    now = service_timetick_what();
    shared_lpf_init(&motion_postprocess_gyro_z_lpf,
            APP_MOTION_POSTPROCESS_GYRO_LPF_ALPHA_DEFAULT,
            imu.gyro_z);
    motion_postprocess_gyro_z_filtered = imu.gyro_z;
    motion_postprocess_imu_last_sequence = imu.sequence;
    motion_postprocess_imu_last_fresh_tick = (0U != imu.valid) ? imu.timestamp_tick : now;
    motion_postprocess_imu_start_tick = now;
    motion_postprocess_imu_seen = ((0U != imu.valid) && (0U != imu.sequence)) ? 1U : 0U;
    motion_postprocess_imu_fault = 0U;
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
    pit_ms_init(APP_MOTION_POSTPROCESS_PIT,
            APP_MOTION_POSTPROCESS_PERIOD_MS,
            app_motion_postprocess_control_step);
    interrupt_set_priority(TIM6_IRQn, 3U);
    pit_ms_init(APP_MOTION_POSTPROCESS_IMU_PIT,
            APP_MOTION_POSTPROCESS_IMU_PERIOD_MS,
            app_motion_postprocess_imu_step);
    interrupt_set_priority(TIM7_IRQn, 3U);
    #if __DBGFLAG__
    printf(">>[app_motion_postprocess_init]\r\n");
    wprint(">>[app_motion_postprocess_init]\r\n");
    #endif
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

/*
 * 单一 5 ms 控制链：按数据依赖顺序推进，禁止让这些控制计算经由主循环队列调度。
 * TIM5 的 1 ms 速度环先使用上一帧目标，TIM6 随后生成下一帧目标，时序稳定且可分析。
 */
void app_motion_postprocess_control_step(void)
{
    app_motion_preprocess_control_step();
    /* 角速度环调试模式下关闭电感元素检测 */
    if(motion_postprocess_yaw_debug_enable < APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD)
    {
        app_element_control_step();
    }
    app_feedforward_control_step();
    app_speed_plan_control_step();
    app_motion_postprocess_compute_step();
}

static void app_motion_postprocess_compute_step(void)
{
    uint8 enabled;
    float raw_error;
    float processed_error;
    float static_feedforward_speed;
    float feedback_yaw_rate_radps;
    float feedforward_differential_speed;
    float feedback_differential_speed;
    float linear_mps;
    float target_yaw_rate_radps;
    float actual_yaw_rate_radps;
    float gyro_z;
    float target_differential_speed;
    float half_differential_speed;
    app_motion_preprocess_data_t motion_preprocess;
    app_feedforward_data_t feedforward;
    app_motion_postprocess_data_t output;

    app_motion_preprocess_get_data(&motion_preprocess);
    app_feedforward_get_data(&feedforward);
    gyro_z = motion_postprocess_gyro_z_filtered;

    raw_error = motion_preprocess.line_error;
    processed_error = raw_error;
    static_feedforward_speed = feedforward.feedforward;
    linear_mps = app_speed_plan_get_linear_mps();

    if(motion_postprocess_yaw_debug_enable >= APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD)
    {
        /* 角速度环调试模式: 关闭电感导航, 目标角速度由无线
           $$W,motion_post_target_yaw_rate,%f@ 直接指派 (rad/s) */
        feedback_yaw_rate_radps = motion_post_target_yaw_rate_override;
        static_feedforward_speed = 0.0f;
    }
    else
    {
        feedback_yaw_rate_radps = tfpu_mul(app_motion_preprocess_config.yaw_rate_gain, raw_error);

        /* 角速度目标变化率限幅 */
        if((app_motion_postprocess_config.rate_limit > 0.0f) && (0U != motion_postprocess_rate_limit_ready))
        {
            float max_delta = tfpu_mul(app_motion_postprocess_config.rate_limit,
                    APP_MOTION_POSTPROCESS_DT_SECOND);
            float delta = tfpu_sub(feedback_yaw_rate_radps, motion_postprocess_rate_limited_yaw_rate);

            if(delta > max_delta)
            {
                delta = max_delta;
            }
            else if(delta < -max_delta)
            {
                delta = -max_delta;
            }

            feedback_yaw_rate_radps = tfpu_add(motion_postprocess_rate_limited_yaw_rate, delta);
        }
        motion_postprocess_rate_limited_yaw_rate = feedback_yaw_rate_radps;
        motion_postprocess_rate_limit_ready = 1U;

        /* 圆筒元素：降角速度增益+限幅+关前馈 */
        {
            app_element_data_t element;
            app_element_get_data(&element);
            if((APP_ELEMENT_TYPE_CYLINDER == element.type) && (element.active >= 0.5f))
            {
                float limit = APP_ELEMENT_CYLINDER_YAW_LIMIT;

                feedback_yaw_rate_radps = tfpu_mul(12.0f, raw_error);
                if(feedback_yaw_rate_radps > limit)
                {
                    feedback_yaw_rate_radps = limit;
                }
                else if(feedback_yaw_rate_radps < -limit)
                {
                    feedback_yaw_rate_radps = -limit;
                }

                static_feedforward_speed = 0.0f;
            }

        }

        target_yaw_rate_radps = feedback_yaw_rate_radps;

        /* 环岛元素：角速度偏置融合 */
        if(0U != app_element_roundabout_bias_active)
        {
            target_yaw_rate_radps = tfpu_add(target_yaw_rate_radps,
                    tfpu_mul(APP_ELEMENT_ROUNDABOUT_BIAS_BLEND, app_element_roundabout_bias_yaw_radps));
            feedback_yaw_rate_radps = target_yaw_rate_radps;
        }
    }

    feedforward_differential_speed = tfpu_mul(APP_MOTION_POSTPROCESS_FEEDFORWARD_SIGN,
            static_feedforward_speed);
    actual_yaw_rate_radps = tfpu_mul(gyro_z, APP_MOTION_POSTPROCESS_DEG_TO_RAD);

    enabled = (app_motion_postprocess_config.enable >= APP_MOTION_POSTPROCESS_ENABLE_THRESHOLD) ? 1U : 0U;

    if(0U == enabled)
    {
        if(0U != motion_postprocess_last_enabled)
        {
            app_motion_postprocess_clear_pid();
        }
        feedback_differential_speed = 0.0f;
        target_differential_speed = 0.0f;
        motion_postprocess_last_enabled = 0U;
        motion_postprocess_rate_limit_ready = 0U;
    }
    else
    {
        app_motion_postprocess_sync_pid();
        feedback_differential_speed = shared_pos_pid_update(&motion_postprocess_yaw_pid,
                feedback_yaw_rate_radps, actual_yaw_rate_radps, APP_MOTION_POSTPROCESS_DT_SECOND);
        target_differential_speed = tfpu_add(feedback_differential_speed, feedforward_differential_speed);
        motion_postprocess_last_enabled = 1U;
    }

    half_differential_speed = tfpu_mul(target_differential_speed, APP_MOTION_POSTPROCESS_DIFF_HALF);

    output.raw_error = raw_error;
    output.feedforward = feedforward_differential_speed;
    output.processed_error = processed_error;
    output.linear_mps = linear_mps;
    output.target_yaw_rate_radps = target_yaw_rate_radps;
    output.actual_yaw_rate_radps = actual_yaw_rate_radps;
    output.target_differential_speed = target_differential_speed;
    output.load_left = half_differential_speed;
    output.load_right = tfpu_sub(0.0f, half_differential_speed);
    output.left_target_mps = tfpu_add(tfpu_mul(APP_MOTION_POSTPROCESS_LEFT_STRAIGHT_SIGN, linear_mps),
            output.load_left);
    output.right_target_mps = tfpu_add(tfpu_mul(APP_MOTION_POSTPROCESS_RIGHT_STRAIGHT_SIGN, linear_mps),
            output.load_right);
    output.enabled = (0U != enabled) ? 1.0f : 0.0f;

    app_speedout_set_target(output.left_target_mps, output.right_target_mps);
    app_motion_postprocess_publish(&output);

    {
        static uint16 print_tick = 0U;
        print_tick++;
        if(print_tick >= 200U)
        {
            print_tick = 0U;
        }
    }
}
