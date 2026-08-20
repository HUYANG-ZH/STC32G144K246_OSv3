#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "service_speed.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "app_inductor_preprocess.h"
#include "app_speedout.h"
#include "service_negative_pressure.h"
#include "service_tof.h"
#include "app_feedforward.h"
#include "app_attitude.h"
#include "app_motion_preprocess.h"
#include "roundabout_priority_tree.h"
#include "app_element.h"

#define APP_ELEMENT_TICK_PER_MS                 (10UL)
#define APP_ELEMENT_CYLINDER_DEAD_MS            (1000UL)
#define APP_ELEMENT_CYLINDER_DEAD_TICK          (APP_ELEMENT_CYLINDER_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_YAW_LIMIT_MS       (350UL)        // 圆筒触发后转向限幅持续时间
#define APP_ELEMENT_CYLINDER_YAW_LIMIT_TICK     (APP_ELEMENT_CYLINDER_YAW_LIMIT_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_WINDOW_MS          (1000UL)
#define APP_ELEMENT_CYLINDER_WINDOW_TICK        (APP_ELEMENT_CYLINDER_WINDOW_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_BUCKET_MS          (10UL)
#define APP_ELEMENT_CYLINDER_BUCKET_TICK        (APP_ELEMENT_CYLINDER_BUCKET_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_BUCKET_COUNT       ((APP_ELEMENT_CYLINDER_WINDOW_MS / APP_ELEMENT_CYLINDER_BUCKET_MS) + 2U)
#define APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_MS  (50UL)
#define APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_TICK \
    (APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_TRIGGER_DEG        (220.0f)
#define APP_ELEMENT_CYLINDER_GYRO_DEADBAND      (15.0f)
#define APP_ELEMENT_CYLINDER_GYRO_LPF_ALPHA     (0.20f)
#define APP_ELEMENT_CYLINDER_SLOWDOWN_TRIGGER     (3U)
#define APP_ELEMENT_CYLINDER_SLOWDOWN_DELAY_MS    (2000UL)
#define APP_ELEMENT_CYLINDER_SLOWDOWN_SPEED_MPS   (1.7f)
#define APP_ELEMENT_CYLINDER_SLOWDOWN_KFF         (0.9f)
#define APP_ELEMENT_CYLINDER_SLOWDOWN_YAW_GAIN    (11.0f)
#define APP_ELEMENT_CYLINDER_SLOWDOWN_PRESSURE    (45U)
#define APP_ELEMENT_TICK_TO_SECOND              (0.0001f)
#define APP_ELEMENT_DEG_TO_RAD                  (0.0174532925f)
#define APP_ELEMENT_PACKET_SINGLE_COUNT         (1U)

#define APP_ELEMENT_SEESAW_CONFIRM_COUNT        (2U)
/* 注意: attitude.pitch_deg 已改为 0-360 语义, 重新启用跷跷板时
   原"pitch >= 15(仅抬头)"阈值判断需改为 (pitch > 15 && pitch < 180) */
#define APP_ELEMENT_SEESAW_PITCH_THRESHOLD_DEG  (15.0f)
#define APP_ELEMENT_SEESAW_DEAD_MS              (500UL)
#define APP_ELEMENT_SEESAW_DEAD_TICK            (APP_ELEMENT_SEESAW_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_SEESAW_ACTIVE_MS            (100UL)
#define APP_ELEMENT_SEESAW_ACTIVE_TICK          (APP_ELEMENT_SEESAW_ACTIVE_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_SEESAW_SCORE_THRESHOLD      (1020)
#define APP_ELEMENT_SEESAW_SLOWDOWN_MS          (160UL)
#define APP_ELEMENT_SEESAW_SLOWDOWN_TICK        (APP_ELEMENT_SEESAW_SLOWDOWN_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_SEESAW_SLOWDOWN_SPEED_MPS   (1.4f)

/* 连续 3 帧(500Hz检测下~6ms)判据成立才确认环岛 */
#define APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT    (3U)
#define APP_ELEMENT_ROUNDABOUT_TOF_TRIGGER_DISTANCE_MM (650U)
#define APP_ELEMENT_ROUNDABOUT_DEAD_MS          (200UL)
#define APP_ELEMENT_ROUNDABOUT_DEAD_TICK        (APP_ELEMENT_ROUNDABOUT_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
/* 环岛超时: 触发后 2s 内未结束则强制结束 */
#define APP_ELEMENT_ROUNDABOUT_TIMEOUT_MS       (2000UL)
#define APP_ELEMENT_ROUNDABOUT_TIMEOUT_TICK     (APP_ELEMENT_ROUNDABOUT_TIMEOUT_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_ROUNDABOUT_TASK_ID          (4U)
#define APP_ELEMENT_ROUNDABOUT_TASK_PRIORITY    (9U)
#define APP_ELEMENT_ROUNDABOUT_PERIOD_MS        (5U)
#define APP_ELEMENT_ROUNDABOUT_FSM_IDLE          (0U)
#define APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE        (1U)
#define APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT     (2U)
#define APP_ELEMENT_ROUNDABOUT_EXIT_DISTANCE_M   (1.0f)
/* EXIT_WAIT 超时兜底: 1.5s 内未完成距离积分则强制恢复空闲, 防状态机卡死 */
#define APP_ELEMENT_ROUNDABOUT_EXIT_WAIT_TIMEOUT_MS  (1500UL)
#define APP_ELEMENT_ROUNDABOUT_EXIT_WAIT_TIMEOUT_TICK (APP_ELEMENT_ROUNDABOUT_EXIT_WAIT_TIMEOUT_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_ROUNDABOUT_1_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_2_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_3_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_4_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_5_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_1_BIAS_DPS_DEFAULT (-900.0f)
#define APP_ELEMENT_ROUNDABOUT_2_BIAS_DPS_DEFAULT (1100.0f)
#define APP_ELEMENT_ROUNDABOUT_3_BIAS_DPS_DEFAULT (-800.0f)
#define APP_ELEMENT_ROUNDABOUT_4_BIAS_DPS_DEFAULT (600.0f)
#define APP_ELEMENT_ROUNDABOUT_5_BIAS_DPS_DEFAULT (-500.0f)
#define APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT (290.0f)
#define APP_ELEMENT_ROUNDABOUT_2_ANGLE_DEG_DEFAULT (270.0f)
#define APP_ELEMENT_ROUNDABOUT_3_ANGLE_DEG_DEFAULT (290.0f)
#define APP_ELEMENT_ROUNDABOUT_4_ANGLE_DEG_DEFAULT (270.0f)
#define APP_ELEMENT_ROUNDABOUT_5_ANGLE_DEG_DEFAULT (180.0f)
#define APP_ELEMENT_ROUNDABOUT_STOP_COUNT        (5U)        /* 第5环完成后停车 */
#define APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_MS           (40UL)
#define APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_TICK         (APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_DPS_DEFAULT  (0.0f)

app_element_config_t app_element_config =
{
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    0U,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    0U,
    0.0f,
    0U
};

static volatile app_element_data_t element_data =
{
    APP_ELEMENT_TYPE_NONE,
    APP_ELEMENT_STATE_IDLE,
    APP_ELEMENT_DIR_NONE,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

static shared_lpf_t element_gyro_x_lpf;
static float element_cylinder_bucket_angle_pos[APP_ELEMENT_CYLINDER_BUCKET_COUNT];
static float element_cylinder_bucket_angle_neg[APP_ELEMENT_CYLINDER_BUCKET_COUNT];
static uint32 element_cylinder_bucket_tick[APP_ELEMENT_CYLINDER_BUCKET_COUNT];
/* Raw timestamp is the integration clock; TIM7 itself only schedules work. */
static uint32 element_last_raw_timestamp_tick = 0U;
static uint8 element_raw_timestamp_valid = 0U;
static uint32 element_imu_last_sequence = 0U;
static uint32 element_cylinder_dead_start_tick = 0U;
static uint32 element_cylinder_yaw_limit_start_tick = 0U;
static uint8 element_cylinder_yaw_limit_active = 0U;
static float element_cylinder_angle_pos_deg = 0.0f;
static float element_cylinder_angle_neg_deg = 0.0f;
static float element_cylinder_gyro_x = 0.0f;
static float element_cylinder_event = 0.0f;
static uint8 element_cylinder_count = 0U;
static float element_cylinder_count_float = 0.0f;
static volatile uint8 element_cylinder_dead = 0U;
static uint8 element_gyro_x_lpf_ready = 0U;
static uint8 element_cylinder_bucket_head = 0U;
static uint8 element_cylinder_bucket_count = 0U;
static volatile uint8 element_seesaw_confirm = 0U;
static volatile uint8 element_seesaw_dead = 0U;
static uint32 element_seesaw_dead_start_tick = 0U;
static volatile uint8 element_seesaw_active = 0U;
static uint32 element_seesaw_active_start_tick = 0U;
static volatile float element_seesaw_event = 0.0f;
static volatile uint8 element_seesaw_slowdown_active = 0U;
static volatile uint32 element_seesaw_slowdown_start_tick = 0U;
static volatile uint8 element_cylinder_slowdown_pending = 0U;
static volatile uint32 element_cylinder_slowdown_deadline_tick = 0U;
static volatile uint8 element_roundabout_confirm = 0U;
static volatile uint32 element_roundabout_dead_start_tick = 0U;
static volatile uint8 element_roundabout_dead = 0U;
static volatile uint8 element_roundabout_count = 0U;
static volatile float element_roundabout_count_float = 0.0f;
static volatile uint8 element_roundabout_gx_high = 0U;
static volatile uint32 element_roundabout_gx_high_tick = 0U;
static volatile uint8 element_roundabout_gz_integrate = 0U;
static volatile float element_roundabout_gz_angle_deg = 0.0f;
static volatile uint8 element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
static float element_roundabout_distance_m = 0.0f;
static volatile uint32 element_roundabout_exit_wait_start_tick = 0U;
static uint32 element_roundabout_active_tick = 0U;

volatile float app_element_roundabout_bias_yaw_radps = 0.0f;
volatile uint8 app_element_roundabout_bias_active = 0U;
volatile float app_element_roundabout_feedforward_scale = 1.0f;
static float element_roundabout_ff_scale_1 = APP_ELEMENT_ROUNDABOUT_1_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_2 = APP_ELEMENT_ROUNDABOUT_2_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_3 = APP_ELEMENT_ROUNDABOUT_3_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_4 = APP_ELEMENT_ROUNDABOUT_4_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_5 = APP_ELEMENT_ROUNDABOUT_5_FF_SCALE_DEFAULT;
static float element_roundabout_bias_dps_1 = APP_ELEMENT_ROUNDABOUT_1_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_2 = APP_ELEMENT_ROUNDABOUT_2_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_3 = APP_ELEMENT_ROUNDABOUT_3_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_4 = APP_ELEMENT_ROUNDABOUT_4_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_5 = APP_ELEMENT_ROUNDABOUT_5_BIAS_DPS_DEFAULT;
static float element_roundabout_angle_deg_1 = APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_2 = APP_ELEMENT_ROUNDABOUT_2_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_3 = APP_ELEMENT_ROUNDABOUT_3_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_4 = APP_ELEMENT_ROUNDABOUT_4_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_5 = APP_ELEMENT_ROUNDABOUT_5_ANGLE_DEG_DEFAULT;
static volatile uint32 element_roundabout_bias_start_tick = 0U;
static volatile uint32 element_roundabout_bias_duration_tick = 0U;

/* —— 环岛硬实时中断改造：主循环写、TIM7中断读的共享数据 —— */
static volatile service_imu_gyro_t element_gyro_snapshot = {0.0f, 0.0f, 0.0f, 0U};
static volatile uint32 element_roundabout_last_tick = 0U;
static volatile uint8 element_roundabout_event_flags = 0U;
static volatile uint16 element_roundabout_event_count = 0U;   /* FOUND 事件携带的 count */
static volatile uint8 element_realtime_event_flags = 0U;
#define ELEMENT_RB_EVENT_FOUND    (0x01U)
#define ELEMENT_RB_EVENT_EXIT     (0x02U)
#define ELEMENT_RB_EVENT_READY    (0x04U)
#define ELEMENT_RB_EVENT_FINISH   (0x08U)
#define ELEMENT_RB_EVENT_END      (0x10U)
#define ELEMENT_RB_EVENT_TIMEOUT  (0x20U)
#define ELEMENT_EVENT_CYLINDER_FOUND       (0x01U)
#define ELEMENT_EVENT_CYLINDER_SLOWDOWN    (0x02U)
#define ELEMENT_EVENT_SEESAW_FOUND         (0x04U)

static void app_element_cylinder_state_reply(void);
static void app_element_roundabout_imu_step(uint8 sample_fresh, uint32 raw_delta_tick);
static void app_element_process_control_events(uint32 now);

static float app_element_roundabout_get_ff_scale(void)
{
    switch(element_roundabout_count)
    {
        case 1U:
            return element_roundabout_ff_scale_1;
        case 2U:
            return element_roundabout_ff_scale_2;
        case 3U:
            return element_roundabout_ff_scale_3;
        case 4U:
            return element_roundabout_ff_scale_4;
        case 5U:
            return element_roundabout_ff_scale_5;
        default:
            return 1.0f;
    }
}

static float app_element_roundabout_get_angle_deg(void)
{
    switch(element_roundabout_count)
    {
        case 1U:
            return element_roundabout_angle_deg_1;
        case 2U:
            return element_roundabout_angle_deg_2;
        case 3U:
            return element_roundabout_angle_deg_3;
        case 4U:
            return element_roundabout_angle_deg_4;
        case 5U:
            return element_roundabout_angle_deg_5;
        default:
            return APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT;
    }
}

static float app_element_roundabout_get_bias_dps(void)
{
    switch(element_roundabout_count)
    {
        case 1U:
            return element_roundabout_bias_dps_1;
        case 2U:
            return element_roundabout_bias_dps_2;
        case 3U:
            return element_roundabout_bias_dps_3;
        case 4U:
            return element_roundabout_bias_dps_4;
        case 5U:
            return element_roundabout_bias_dps_5;
        default:
            return 0.0f;
    }
}

static void app_element_roundabout_apply_runtime_config(void)
{
    float bias_dps;

    bias_dps = app_element_roundabout_get_bias_dps();
    app_element_roundabout_bias_yaw_radps = tfpu_mul(bias_dps, APP_ELEMENT_DEG_TO_RAD);
    app_element_roundabout_bias_active = (0.0f != bias_dps) ? 1U : 0U;
    app_element_roundabout_feedforward_scale = app_element_roundabout_get_ff_scale();
}

static void app_element_reset(void)
{
    uint8 i;

    element_data.type = APP_ELEMENT_TYPE_NONE;
    element_data.state = APP_ELEMENT_STATE_IDLE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 0.0f;
    element_data.gyro_x = 0.0f;
    element_data.gyro_y = 0.0f;
    element_data.gyro_z = 0.0f;
    element_last_raw_timestamp_tick = 0U;
    element_raw_timestamp_valid = 0U;
    element_imu_last_sequence = 0U;
    element_cylinder_dead_start_tick = 0U;
    element_cylinder_angle_pos_deg = 0.0f;
    element_cylinder_angle_neg_deg = 0.0f;
    element_cylinder_gyro_x = 0.0f;
    element_cylinder_event = 0.0f;
    element_cylinder_count = 0U;
    element_cylinder_count_float = 0.0f;
    element_cylinder_dead = 0U;
    element_gyro_x_lpf_ready = 0U;
    element_cylinder_bucket_head = 0U;
    element_cylinder_bucket_count = 0U;
    element_seesaw_confirm = 0U;
    element_seesaw_dead = 0U;
    element_seesaw_active = 0U;
    element_seesaw_event = 0.0f;
    element_seesaw_slowdown_active = 0U;
    element_seesaw_slowdown_start_tick = 0U;
    element_cylinder_slowdown_pending = 0U;
    element_cylinder_slowdown_deadline_tick = 0U;
    element_roundabout_confirm = 0U;
    element_roundabout_dead = 0U;
    element_roundabout_count = 0U;
    element_roundabout_count_float = 0.0f;
    element_roundabout_gx_high = 0U;
    element_roundabout_gz_integrate = 0U;
    element_roundabout_gz_angle_deg = 0.0f;
    element_roundabout_last_tick = 0U;
    element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
    element_roundabout_distance_m = 0.0f;
    element_roundabout_exit_wait_start_tick = 0U;
    element_roundabout_active_tick = 0U;
    app_element_roundabout_bias_yaw_radps = 0.0f;
    app_element_roundabout_bias_active = 0U;
    app_element_roundabout_feedforward_scale = 1.0f;
    element_roundabout_event_flags = 0U;
    element_roundabout_event_count = 0U;
    element_realtime_event_flags = 0U;
    for(i = 0U; i < APP_ELEMENT_CYLINDER_BUCKET_COUNT; i++)
    {
        element_cylinder_bucket_angle_pos[i] = 0.0f;
        element_cylinder_bucket_angle_neg[i] = 0.0f;
        element_cylinder_bucket_tick[i] = 0U;
    }
    shared_lpf_init(&element_gyro_x_lpf, APP_ELEMENT_CYLINDER_GYRO_LPF_ALPHA, 0.0f);
}

static void app_element_cylinder_clear(void)
{
    element_cylinder_angle_pos_deg = 0.0f;
    element_cylinder_angle_neg_deg = 0.0f;
    element_cylinder_bucket_head = 0U;
    element_cylinder_bucket_count = 0U;
}

static uint8 app_element_cylinder_next_bucket(uint8 index)
{
    index++;
    if(index >= APP_ELEMENT_CYLINDER_BUCKET_COUNT)
    {
        index = 0U;
    }

    return index;
}

static uint8 app_element_cylinder_tail_bucket(void)
{
    uint16 index;

    index = (uint16)element_cylinder_bucket_head + (uint16)element_cylinder_bucket_count - 1U;
    while(index >= APP_ELEMENT_CYLINDER_BUCKET_COUNT)
    {
        index -= APP_ELEMENT_CYLINDER_BUCKET_COUNT;
    }

    return (uint8)index;
}

static void app_element_cylinder_drop_head(void)
{
    element_cylinder_angle_pos_deg = tfpu_sub(element_cylinder_angle_pos_deg,
            element_cylinder_bucket_angle_pos[element_cylinder_bucket_head]);
    element_cylinder_angle_neg_deg = tfpu_sub(element_cylinder_angle_neg_deg,
            element_cylinder_bucket_angle_neg[element_cylinder_bucket_head]);
    if(element_cylinder_angle_pos_deg < 0.0f)
    {
        element_cylinder_angle_pos_deg = 0.0f;
    }
    if(element_cylinder_angle_neg_deg < 0.0f)
    {
        element_cylinder_angle_neg_deg = 0.0f;
    }
    element_cylinder_bucket_angle_pos[element_cylinder_bucket_head] = 0.0f;
    element_cylinder_bucket_angle_neg[element_cylinder_bucket_head] = 0.0f;
    element_cylinder_bucket_head = app_element_cylinder_next_bucket(element_cylinder_bucket_head);
    element_cylinder_bucket_count--;
}

static void app_element_cylinder_drop_old(uint32 now)
{
    while((0U != element_cylinder_bucket_count) &&
            ((uint32)(now - element_cylinder_bucket_tick[element_cylinder_bucket_head]) >
             APP_ELEMENT_CYLINDER_WINDOW_TICK))
    {
        app_element_cylinder_drop_head();
    }
}

static void app_element_cylinder_push(float delta_deg, uint32 now)
{
    uint8 tail;
    float angle_abs;

    if(0U == element_cylinder_bucket_count)
    {
        element_cylinder_bucket_head = 0U;
        element_cylinder_bucket_count = 1U;
        element_cylinder_bucket_angle_pos[0] = 0.0f;
        element_cylinder_bucket_angle_neg[0] = 0.0f;
        element_cylinder_bucket_tick[0] = now;
    }

    tail = app_element_cylinder_tail_bucket();
    if((uint32)(now - element_cylinder_bucket_tick[tail]) >= APP_ELEMENT_CYLINDER_BUCKET_TICK)
    {
        if(element_cylinder_bucket_count >= APP_ELEMENT_CYLINDER_BUCKET_COUNT)
        {
            app_element_cylinder_drop_head();
        }

        tail = app_element_cylinder_next_bucket(tail);
        element_cylinder_bucket_angle_pos[tail] = 0.0f;
        element_cylinder_bucket_angle_neg[tail] = 0.0f;
        element_cylinder_bucket_tick[tail] = now;
        element_cylinder_bucket_count++;
    }

    if(delta_deg >= 0.0f)
    {
        element_cylinder_bucket_angle_pos[tail] = tfpu_add(element_cylinder_bucket_angle_pos[tail], delta_deg);
        element_cylinder_angle_pos_deg = tfpu_add(element_cylinder_angle_pos_deg, delta_deg);
    }
    else
    {
        angle_abs = tfpu_sub(0.0f, delta_deg);
        element_cylinder_bucket_angle_neg[tail] = tfpu_add(element_cylinder_bucket_angle_neg[tail], angle_abs);
        element_cylinder_angle_neg_deg = tfpu_add(element_cylinder_angle_neg_deg, angle_abs);
    }
}

static uint8 app_element_cylinder_in_dead(uint32 now)
{
    /* 跷跷板死区内也阻挡圆筒 */
    if(0U != element_seesaw_dead)
    {
        return 1U;
    }

    if(0U == element_cylinder_dead)
    {
        return 0U;
    }

    if((uint32)(now - element_cylinder_dead_start_tick) >= APP_ELEMENT_CYLINDER_DEAD_TICK)
    {
        element_cylinder_dead = 0U;
        return 0U;
    }

    return 1U;
}

static void app_element_cylinder_slowdown(void)
{
    app_motion_preprocess_config.linear_mps = APP_ELEMENT_CYLINDER_SLOWDOWN_SPEED_MPS;
    app_feedforward_config.kff = APP_ELEMENT_CYLINDER_SLOWDOWN_KFF;
    app_motion_preprocess_config.yaw_rate_gain = APP_ELEMENT_CYLINDER_SLOWDOWN_YAW_GAIN;
    if(app_speedout_data.enabled > 0.0f)
    {
        service_negative_pressure_set_percent(APP_ELEMENT_CYLINDER_SLOWDOWN_PRESSURE);
    }
    element_realtime_event_flags |= ELEMENT_EVENT_CYLINDER_SLOWDOWN;
}

static void app_element_cylinder_found(int8 dir, float gyro_x, uint32 now)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    element_data.type = APP_ELEMENT_TYPE_CYLINDER;
    element_data.state = APP_ELEMENT_STATE_DONE;
    element_data.dir = (0 < dir) ? APP_ELEMENT_DIR_LEFT : APP_ELEMENT_DIR_RIGHT;
    element_data.active = 1.0f;
    element_data.gyro_x = gyro_x;
    EA = ea_backup;

    element_cylinder_dead = 1U;
    element_cylinder_dead_start_tick = now;
    element_cylinder_yaw_limit_active = 1U;
    element_cylinder_yaw_limit_start_tick = now;
    element_cylinder_event = (0 < dir) ? 1.0f : -1.0f;
    element_cylinder_count++;
    element_cylinder_count_float = (float)element_cylinder_count;
    if(APP_ELEMENT_CYLINDER_SLOWDOWN_TRIGGER == element_cylinder_count)
    {
        element_cylinder_slowdown_pending = 1U;
        element_cylinder_slowdown_deadline_tick = now +
                (uint32)APP_ELEMENT_CYLINDER_SLOWDOWN_DELAY_MS * APP_ELEMENT_TICK_PER_MS;
    }
    app_element_cylinder_clear();

    element_realtime_event_flags |= ELEMENT_EVENT_CYLINDER_FOUND;
}

static void app_element_process_control_events(uint32 now)
{
    if((0U != element_cylinder_slowdown_pending) &&
            ((int32)(now - element_cylinder_slowdown_deadline_tick) >= 0))
    {
        element_cylinder_slowdown_pending = 0U;
        app_element_cylinder_slowdown();
    }
}

static void app_element_cylinder_update(float gyro_x, uint32 delta_tick, uint32 now)
{
    float delta_deg;

    if(0U != app_element_cylinder_in_dead(now))
    {
        app_element_cylinder_clear();
        return;
    }
    app_element_cylinder_drop_old(now);
    element_cylinder_gyro_x = gyro_x;

    if((gyro_x <= APP_ELEMENT_CYLINDER_GYRO_DEADBAND) &&
            (gyro_x >= -APP_ELEMENT_CYLINDER_GYRO_DEADBAND))
    {
        return;
    }

    delta_deg = tfpu_mul(gyro_x, tfpu_mul(tfpu_int2float((long)delta_tick), APP_ELEMENT_TICK_TO_SECOND));
    app_element_cylinder_push(delta_deg, now);

    if(element_cylinder_angle_pos_deg >= APP_ELEMENT_CYLINDER_TRIGGER_DEG)
    {
        app_element_cylinder_found(1, gyro_x, now);
    }
    else if(element_cylinder_angle_neg_deg >= APP_ELEMENT_CYLINDER_TRIGGER_DEG)
    {
        app_element_cylinder_found(-1, gyro_x, now);
    }
}

static void app_element_cylinder_state_reply(void)
{
    wprint("cylinder_state,%.3f,%.3f,%.3f,%u,%.3f\r\n",
            element_cylinder_gyro_x,
            element_cylinder_angle_pos_deg,
            element_cylinder_angle_neg_deg,
            (uint16)element_cylinder_bucket_count,
            element_cylinder_event);
}

static void app_element_roundabout_found(uint32 now)
{
    uint8 ea_backup;

    /* 第5环已完成(停车)后不再触发新的环岛动作 */
    if(element_roundabout_count >= APP_ELEMENT_ROUNDABOUT_STOP_COUNT)
    {
        return;
    }

    element_roundabout_dead = 1U;
    element_roundabout_dead_start_tick = now;

    ea_backup = EA;
    EA = 0;
    element_data.type = APP_ELEMENT_TYPE_ROUNDABOUT;
    element_data.state = APP_ELEMENT_STATE_DONE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 1.0f;
    EA = ea_backup;

    element_roundabout_count++;
    element_roundabout_count_float = (float)element_roundabout_count;
    element_roundabout_event_count = (uint16)element_roundabout_count;
    element_roundabout_event_flags |= ELEMENT_RB_EVENT_FOUND;

    /* 按第几个环岛加载偏置/前馈配置并进入 gz 角度积分,
       退出由 imu_step 在角度达标时执行(第5环停车) */
    app_element_roundabout_apply_runtime_config();
    element_roundabout_bias_start_tick = now;
    element_roundabout_bias_duration_tick = 0xFFFFFFFFU;
    element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE;
    element_roundabout_active_tick = now;
    element_roundabout_gz_integrate = 1U;
    element_roundabout_gz_angle_deg = 0.0f;
}

void app_element_control_step(void)
{
    uint32 now;
    uint8 roundabout_detected;
    uint16 tof_distance_mm;
    app_inductor_preprocess_data_t inductor;

    now = service_timetick_what();

    if((0U != element_roundabout_dead) &&
            ((uint32)(now - element_roundabout_dead_start_tick) >= APP_ELEMENT_ROUNDABOUT_DEAD_TICK))
    {
        element_roundabout_dead = 0U;
        element_roundabout_confirm = 0U;
    }

    if(0U != element_roundabout_dead)
    {
        return;
    }

    /* 环岛判据: TOF 超出阈值 且 INT8 浅树模型(rapt, 14x7)判定: 输入 uint16 归一化值(内部钳位255) + TOF mm */
    tof_distance_mm = service_tof_get_distance_mm();
    /* Active roundabout model: rapt_r16, 65 trees x depth 5. */
    app_inductor_preprocess_get_data(&inductor);
    roundabout_detected = ((tof_distance_mm > APP_ELEMENT_ROUNDABOUT_TOF_TRIGGER_DISTANCE_MM) &&
            (0U != roundabout_priority_tree_predict(
                    (uint16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1],
                    (uint16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2],
                    (uint16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3],
                    (uint16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4],
                    tof_distance_mm))) ? 1U : 0U;

    if(APP_ELEMENT_ROUNDABOUT_FSM_IDLE != element_roundabout_fsm)
    {
        /* 环岛动作中(ACTIVE/EXIT_WAIT): 退出/停车由 imu_step 的 gz 角度积分控制,
           此处不再因判据消失而中断, 仅保留 2s 超时兜底防死锁 */
        if((APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE == element_roundabout_fsm) &&
                ((uint32)(now - element_roundabout_active_tick) >= APP_ELEMENT_ROUNDABOUT_TIMEOUT_TICK))
        {
            element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
            element_roundabout_dead = 1U;
            element_roundabout_dead_start_tick = now;
            element_roundabout_event_flags |= ELEMENT_RB_EVENT_TIMEOUT;
        }
        return;
    }

    if(0U != roundabout_detected)
    {
        if((0U == element_seesaw_active) && (0U == element_roundabout_gx_high))
        {
            element_roundabout_confirm++;
            if(element_roundabout_confirm >= APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT)
            {
                app_element_roundabout_found(now);
            }
        }
    }
    else
    {
        element_roundabout_confirm = 0U;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     环岛状态机 IMU 步进 - 硬实时执行(与圆筒共用 imu_task 输入)
// 参数说明     sample_fresh     本帧 IMU 采样是否为新帧
// 参数说明     raw_delta_tick   与上一帧的时间差(0.1ms tick)
// 返回参数     void
// 使用示例     app_element_roundabout_imu_step(sample_fresh, raw_delta_tick);
// 备注信息     gz 角度积分控制环岛退出; 第5环(APP_ELEMENT_ROUNDABOUT_STOP_COUNT)停车
//-------------------------------------------------------------------------------------------------------------------
static void app_element_roundabout_imu_step(uint8 sample_fresh, uint32 raw_delta_tick)
{
    service_imu_gyro_t gyro;
    uint8 ea_backup;
    uint32 now;
    uint32 control_delta_tick;

    gyro = element_gyro_snapshot;   /* volatile 读主循环快照 */

    now = service_timetick_what();
    if(0U == element_roundabout_last_tick)
    {
        element_roundabout_last_tick = now;
        return;
    }
    control_delta_tick = now - element_roundabout_last_tick;
    element_roundabout_last_tick = now;

    /* gx 横滚抑制(环岛检测阻挡条件): |gx|>200°/s 置位 20ms, 期间抑制环岛确认 */
    if((0U != sample_fresh) && ((gyro.gyro_x > 200.0f) || (gyro.gyro_x < -200.0f)))
    {
        element_roundabout_gx_high = 1U;
        element_roundabout_gx_high_tick = now;
    }
    else if((0U != element_roundabout_gx_high) &&
            ((uint32)(now - element_roundabout_gx_high_tick) > 20U * APP_ELEMENT_TICK_PER_MS))
    {
        element_roundabout_gx_high = 0U;
    }

    /* 环岛gz积分：达到当前环岛配置角度后退出，第5环直接停车 */
    if((0U != element_roundabout_gz_integrate) && (0U != sample_fresh) &&
       (0U != raw_delta_tick) &&
       (APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_TICK >= raw_delta_tick))
    {
        float target_angle_deg;
        float delta_angle = tfpu_mul(gyro.gyro_z,
                tfpu_mul(tfpu_int2float((long)raw_delta_tick), APP_ELEMENT_TICK_TO_SECOND));

        app_element_roundabout_apply_runtime_config();
        target_angle_deg = app_element_roundabout_get_angle_deg();
        if(delta_angle < 0.0f)
        {
            delta_angle = tfpu_sub(0.0f, delta_angle);
        }
        element_roundabout_gz_angle_deg = tfpu_add(element_roundabout_gz_angle_deg, delta_angle);
        if(element_roundabout_gz_angle_deg >= target_angle_deg)
        {
            element_roundabout_gz_integrate = 0U;
            element_roundabout_gz_angle_deg = 0.0f;
            app_element_roundabout_bias_active = 0U;
            app_element_roundabout_bias_yaw_radps = 0.0f;
            app_element_roundabout_feedforward_scale = 1.0f;
            element_roundabout_dead = 0U;
            element_roundabout_confirm = 0U;
            if(element_roundabout_count >= APP_ELEMENT_ROUNDABOUT_STOP_COUNT)
            {
                /* 第5环岛：停车+关负压在中断内即时执行（非阻塞寄存器写） */
                element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
                app_speedout_request_stop_all();
                element_roundabout_event_flags |= ELEMENT_RB_EVENT_FINISH;
            }
            else
            {
                /* 出环反向偏置: 持续40ms, 由到期机制自动清除;
                   方向与环内偏置相反(回正): 环内负偏置(1/3/5环)->正向, 环内正偏置(2/4环)->负向 */
                float reverse_bias_dps = APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_DPS_DEFAULT;
                if(app_element_roundabout_get_bias_dps() > 0.0f)
                {
                    reverse_bias_dps = tfpu_sub(0.0f, reverse_bias_dps);
                }
                app_element_roundabout_bias_yaw_radps = tfpu_mul(reverse_bias_dps, APP_ELEMENT_DEG_TO_RAD);
                app_element_roundabout_bias_active = 1U;
                element_roundabout_bias_start_tick = now;
                element_roundabout_bias_duration_tick = APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_TICK;

                element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT;
                element_roundabout_distance_m = 0.0f;
                element_roundabout_exit_wait_start_tick = now;
                element_roundabout_event_flags |= ELEMENT_RB_EVENT_EXIT;
                ea_backup = EA;
                EA = 0;
                element_data.type = APP_ELEMENT_TYPE_NONE;
                element_data.state = APP_ELEMENT_STATE_IDLE;
                element_data.dir = APP_ELEMENT_DIR_NONE;
                element_data.active = 0.0f;
                EA = ea_backup;
            }
        }
    }

    /* 环岛退出距离积分：累计前进1m后恢复空闲态; 1.5s 超时兜底防卡死 */
    if(APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT == element_roundabout_fsm)
    {
        service_speed_data_t speed;
        float avg_mps;

        if((uint32)(now - element_roundabout_exit_wait_start_tick) >=
                APP_ELEMENT_ROUNDABOUT_EXIT_WAIT_TIMEOUT_TICK)
        {
            /* 超时强制恢复: 距离积分异常(如长时间停车)也不能卡死状态机 */
            element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
            element_roundabout_distance_m = 0.0f;
            element_roundabout_event_flags |= ELEMENT_RB_EVENT_READY;
        }
        else
        {
            service_speed_get(&speed);
            /* 平均前进速度 = (左+右)/2, 用差速(左-右)会导致直行时距离永远积不满 */
            avg_mps = tfpu_mul(tfpu_add(speed.left_mps, speed.right_mps), 0.5f);
            element_roundabout_distance_m = tfpu_add(element_roundabout_distance_m,
                    tfpu_mul(avg_mps, tfpu_mul(tfpu_int2float((long)control_delta_tick), APP_ELEMENT_TICK_TO_SECOND)));
            if(element_roundabout_distance_m >= APP_ELEMENT_ROUNDABOUT_EXIT_DISTANCE_M)
            {
                element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
                element_roundabout_distance_m = 0.0f;
                element_roundabout_event_flags |= ELEMENT_RB_EVENT_READY;
            }
        }
    }

    /* 环岛角速度偏置到期清除 */
    if((0U != app_element_roundabout_bias_active) &&
            ((uint32)(now - element_roundabout_bias_start_tick) >= element_roundabout_bias_duration_tick))
    {
        app_element_roundabout_bias_active = 0U;
        app_element_roundabout_bias_yaw_radps = 0.0f;
    }
}

static void app_element_roundabout_clear_count(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    element_roundabout_count = 0U;
    element_roundabout_count_float = 0.0f;
    app_element_roundabout_bias_active = 0U;
    app_element_roundabout_bias_yaw_radps = 0.0f;
    app_element_roundabout_feedforward_scale = 1.0f;
    EA = ea_backup;

    wprint("roundabout_count,0.000\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     环岛事件泵 - 主循环调用，把中断中置位的事件转为 wprint/蜂鸣器
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_pump_events();
// 备注信息     由 app_motion_postprocess_gyro_task 周期调用
//-------------------------------------------------------------------------------------------------------------------
void app_element_pump_events(void)
{
    uint8 flags;
    uint8 realtime_flags;
    uint16 count_snapshot;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    flags = element_roundabout_event_flags;
    element_roundabout_event_flags = 0U;
    realtime_flags = element_realtime_event_flags;
    element_realtime_event_flags = 0U;
    count_snapshot = element_roundabout_event_count;
    EA = ea_backup;

    if(0U == flags)
    {
        if(0U == realtime_flags)
        {
            return;
        }
    }

    if(0U != (flags & ELEMENT_RB_EVENT_FOUND))
    {
        wprint("roundabout,1.000,%u\r\n", (uint16)count_snapshot);
    }
    if(0U != (flags & ELEMENT_RB_EVENT_EXIT))
    {
        wprint("roundabout_exit,1.000\r\n");
    }
    if(0U != (flags & ELEMENT_RB_EVENT_READY))
    {
        wprint("roundabout_ready,1.000\r\n");
    }
    if(0U != (flags & ELEMENT_RB_EVENT_FINISH))
    {
        wprint("roundabout_finish,1.000\r\n");
    }
    if(0U != (flags & ELEMENT_RB_EVENT_END))
    {
        wprint("roundabout_end,1.000\r\n");
    }
    if(0U != (flags & ELEMENT_RB_EVENT_TIMEOUT))
    {
        wprint("roundabout_timeout,1.000\r\n");
    }
    if(0U != (realtime_flags & ELEMENT_EVENT_CYLINDER_FOUND))
    {
        wprint("cylinder_count,%u\r\n", (uint16)element_cylinder_count);
        wprint("cylinder,1.000\r\n");
    }
    if(0U != (realtime_flags & ELEMENT_EVENT_CYLINDER_SLOWDOWN))
    {
        wprint("cylinder_slowdown,1.000\r\n");
    }
    if(0U != (realtime_flags & ELEMENT_EVENT_SEESAW_FOUND))
    {
        wprint("seesaw,1.000\r\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跷跷板降速是否激活
// 参数说明     void
// 返回参数     uint8           1：降速激活中 0：已到期或未触发
// 使用示例     if(app_element_seesaw_is_slowdown_active()) ...
//-------------------------------------------------------------------------------------------------------------------
uint8 app_element_seesaw_is_slowdown_active(void)
{
    return element_seesaw_slowdown_active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跷跷板降速目标速度
// 参数说明     void
// 返回参数     float           降速目标速度 m/s
// 使用示例     speed = app_element_seesaw_slowdown_speed_mps();
//-------------------------------------------------------------------------------------------------------------------
float app_element_seesaw_slowdown_speed_mps(void)
{
    return APP_ELEMENT_SEESAW_SLOWDOWN_SPEED_MPS;
}

static void app_element_cylinder_clear_count(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    element_cylinder_count = 0U;
    element_cylinder_count_float = 0.0f;
    EA = ea_backup;

    wprint("cylinder_count,0.000\r\n");
}

#if 0
static void app_element_seesaw_found(uint32 now)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    element_data.type = APP_ELEMENT_TYPE_SEESAW;
    element_data.state = APP_ELEMENT_STATE_DONE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 1.0f;
    EA = ea_backup;

    element_seesaw_dead = 1U;
    element_seesaw_dead_start_tick = now;
    element_seesaw_confirm = 0U;
    element_cylinder_dead = 1U;
    element_cylinder_dead_start_tick = now;
    element_seesaw_active = 1U;
    element_seesaw_active_start_tick = now;
    element_seesaw_event = 1.0f;
    element_seesaw_slowdown_active = 1U;
    element_seesaw_slowdown_start_tick = now;

    element_realtime_event_flags |= ELEMENT_EVENT_SEESAW_FOUND;
}

static void app_element_seesaw_update(uint32 now, uint8 attitude_fresh)
{
    app_inductor_preprocess_data_t inductor;
    app_attitude_data_t attitude;
    int32 score;

    /* 死区检查 */
    if(0U != element_seesaw_dead)
    {
        if((uint32)(now - element_seesaw_dead_start_tick) >= APP_ELEMENT_SEESAW_DEAD_TICK)
        {
            element_seesaw_dead = 0U;
            element_seesaw_confirm = 0U;
        }
        return;
    }

    /* 连续帧判定只消费本次新到的解算姿态，禁止把同一帧重复计数。 */
    if(0U == attitude_fresh)
    {
        return;
    }

    app_attitude_get_data(&attitude);
    if((0U == attitude.valid) || (attitude.sequence != element_imu_last_sequence))
    {
        element_seesaw_confirm = 0U;
        return;
    }

    app_inductor_preprocess_get_data(&inductor);

    score = (int32)(-81 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1]
                    -56 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2]
                    -204 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3]
                    -56 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4]
                    +159 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M])
            - 2 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M] * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M];

    if((score >= APP_ELEMENT_SEESAW_SCORE_THRESHOLD) &&
            (attitude.pitch_deg >= APP_ELEMENT_SEESAW_PITCH_THRESHOLD_DEG))
    {
        element_seesaw_confirm++;
        if(element_seesaw_confirm >= APP_ELEMENT_SEESAW_CONFIRM_COUNT)
        {
            app_element_seesaw_found(now);
        }
    }
    else
    {
        element_seesaw_confirm = 0U;
    }
}

#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素识别初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_init();
// 备注信息     当前仅启用圆筒识别，初始化后默认无元素
//-------------------------------------------------------------------------------------------------------------------
void app_element_init(void)
{
    app_element_reset();
    (void)service_packet_add_action("cylinder_state", app_element_cylinder_state_reply, 0UL);
    (void)service_packet_add_variable("cylinder_event",
            &element_cylinder_event, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("cylinder_count",
            &element_cylinder_count_float, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_action("reset_cylinder", app_element_cylinder_clear_count, 0UL);
    (void)service_packet_add_variable("seesaw_event",
            &element_seesaw_event, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_action("reset_round", app_element_roundabout_clear_count, 0UL);
    (void)service_packet_add_variable("roundabout_count",
            &element_roundabout_count_float, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_ff_scale1",
            &element_roundabout_ff_scale_1, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_ff_scale2",
            &element_roundabout_ff_scale_2, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_ff_scale3",
            &element_roundabout_ff_scale_3, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_ff_scale4",
            &element_roundabout_ff_scale_4, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_ff_scale5",
            &element_roundabout_ff_scale_5, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias1_dps",
            &element_roundabout_bias_dps_1, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias2_dps",
            &element_roundabout_bias_dps_2, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias3_dps",
            &element_roundabout_bias_dps_3, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias4_dps",
            &element_roundabout_bias_dps_4, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias5_dps",
            &element_roundabout_bias_dps_5, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle1_deg",
            &element_roundabout_angle_deg_1, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle2_deg",
            &element_roundabout_angle_deg_2, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle3_deg",
            &element_roundabout_angle_deg_3, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle4_deg",
            &element_roundabout_angle_deg_4, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle5_deg",
            &element_roundabout_angle_deg_5, APP_ELEMENT_PACKET_SINGLE_COUNT);
    #if __DBGFLAG__
    printf(">>[app_element_init]\r\n");
    wprint(">>[app_element_init]\r\n");
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取元素识别数据
// 参数说明     out_data        数据输出地址
// 返回参数     void
// 使用示例     app_element_get_data(&element_data);
//-------------------------------------------------------------------------------------------------------------------
void app_element_get_data(app_element_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    *out_data = element_data;
    EA = ea_backup;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素识别IMU输入任务
// 参数说明     gyro             IMU三轴陀螺仪数据，单位 deg/s
// 返回参数     void
// 使用示例     app_element_imu_task(&gyro);
// 备注信息     当前只识别gyro_x滑动窗口内单方向累计转动超过180度的圆筒元素
//-------------------------------------------------------------------------------------------------------------------
void app_element_imu_task(const service_imu_sample_t *imu)
{
    uint8 ea_backup;
    uint8 sample_fresh;
    uint32 now;
    uint32 raw_delta_tick = 0U;
    float gyro_x;

    if(NULL == imu)
    {
        return;
    }

    /* 提供 gyro 快照供 TIM7 环岛中断读取（volatile 写） */
    sample_fresh = ((0U != imu->valid) && (0U != imu->sequence) &&
            (imu->sequence != element_imu_last_sequence)) ? 1U : 0U;
    if(0U != sample_fresh)
    {
        element_imu_last_sequence = imu->sequence;
        ea_backup = EA;
        EA = 0;
        element_gyro_snapshot.gyro_x = imu->gyro_x;
        element_gyro_snapshot.gyro_y = imu->gyro_y;
        element_gyro_snapshot.gyro_z = imu->gyro_z;
        element_gyro_snapshot.sequence = imu->sequence;
        EA = ea_backup;

        if(0U == element_raw_timestamp_valid)
        {
            element_last_raw_timestamp_tick = imu->timestamp_tick;
            element_raw_timestamp_valid = 1U;
        }
        else
        {
            raw_delta_tick = imu->timestamp_tick - element_last_raw_timestamp_tick;
            element_last_raw_timestamp_tick = imu->timestamp_tick;
        }
    }

    now = service_timetick_what();
    app_element_process_control_events(now);

    gyro_x = element_cylinder_gyro_x;
    if(0U != sample_fresh)
    {
        if(0U == element_gyro_x_lpf_ready)
        {
            shared_lpf_reset(&element_gyro_x_lpf, imu->gyro_x);
            element_gyro_x_lpf_ready = 1U;
        }
        gyro_x = shared_lpf_update(&element_gyro_x_lpf, imu->gyro_x);

        ea_backup = EA;
        EA = 0;
        element_data.gyro_x = gyro_x;
        element_data.gyro_y = imu->gyro_y;
        element_data.gyro_z = imu->gyro_z;
        EA = ea_backup;
    }

    /* 注: 环岛 gz 积分/退出/EXIT_WAIT 距离积分/偏置到期清除仍禁用;
       gx 横滚抑制(环岛阻挡)已恢复在本函数内实时执行 */

    if((0U != sample_fresh) &&
       (raw_delta_tick > APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_TICK))
    {
        app_element_cylinder_clear();
    }
    else if((0U != sample_fresh) && (0U != raw_delta_tick))
    {
        app_element_cylinder_update(gyro_x, raw_delta_tick, now);
    }

    /* 跷跷板检测(当前禁用, 仅保留圆筒识别) */
    // app_element_seesaw_update(now, sample_fresh);

    /* 跷跷板动作到期清除（100ms后释放控制，死区保留到500ms自动到期） */
    if((0U != element_seesaw_active) &&
            ((uint32)(now - element_seesaw_active_start_tick) >= APP_ELEMENT_SEESAW_ACTIVE_TICK))
    {
        element_seesaw_active = 0U;
        element_seesaw_event = 0.0f;
        ea_backup = EA;
        EA = 0;
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
        EA = ea_backup;
    }

    /* 跷跷板降速到期清除（160ms后恢复正常速度规划） */
    if((0U != element_seesaw_slowdown_active) &&
            ((uint32)(now - element_seesaw_slowdown_start_tick) >= APP_ELEMENT_SEESAW_SLOWDOWN_TICK))
    {
        element_seesaw_slowdown_active = 0U;
    }

    /* 圆筒转向限幅持续时间到期后清除元素状态（跷跷板活跃时跳过） */
    if((0U != element_cylinder_yaw_limit_active) &&
            (0U == element_seesaw_active) &&
            (0U == element_seesaw_dead) &&
            ((uint32)(now - element_cylinder_yaw_limit_start_tick) >= APP_ELEMENT_CYLINDER_YAW_LIMIT_TICK))
    {
        element_cylinder_yaw_limit_active = 0U;
        ea_backup = EA;
        EA = 0;
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
        EA = ea_backup;
    }

    /* 环岛 gz 积分状态机(横滚抑制 gx_high 在其内部执行, 与 car3 布局一致) */
    app_element_roundabout_imu_step(sample_fresh, raw_delta_tick);
}
