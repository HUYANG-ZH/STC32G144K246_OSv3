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
#include "service_tof.h"
#include "app_attitude.h"
#include "roundabout_priority_tree.h"
#include "app_element.h"

#define APP_ELEMENT_TICK_PER_MS                 (10UL)
/* 圆筒识别: TOF<200mm 连续2帧 且 pitch∈(120,270)° 连续5帧 触发(0-360 pitch 语义, 单点触发) */
#define APP_ELEMENT_CYLINDER_TOF_NEAR_MM         (200U)
#define APP_ELEMENT_CYLINDER_TOF_NEAR_FRAMES     (2U)
#define APP_ELEMENT_CYLINDER_TOF_EVAL_TICK       (400UL)   /* ≈TOF 采样周期(30Hz)+裕量, TOF 计数按采样节拍 */
#define APP_ELEMENT_CYLINDER_PITCH_TRIGGER_DEG   (120.0f)
#define APP_ELEMENT_CYLINDER_PITCH_TRIGGER_MAX_DEG (270.0f)
#define APP_ELEMENT_CYLINDER_PITCH_TRIGGER_FRAMES (5U)
/* 圆筒结束: pitch 回到 0°±15° 连续5帧 且 TOF>200mm */
#define APP_ELEMENT_CYLINDER_PITCH_END_DEG       (15.0f)
#define APP_ELEMENT_CYLINDER_PITCH_END_FRAMES    (5U)
/* 超时: 触发后 2s 内未结束则强制退出 */
#define APP_ELEMENT_CYLINDER_TIMEOUT_MS          (2000UL)
#define APP_ELEMENT_CYLINDER_TIMEOUT_TICK        (APP_ELEMENT_CYLINDER_TIMEOUT_MS * APP_ELEMENT_TICK_PER_MS)
/* 结束/超时后冷却期, 防止立即重复触发 */
#define APP_ELEMENT_CYLINDER_COOLDOWN_MS         (1000UL)
#define APP_ELEMENT_CYLINDER_COOLDOWN_TICK       (APP_ELEMENT_CYLINDER_COOLDOWN_MS * APP_ELEMENT_TICK_PER_MS)
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

#define APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT    (1U)
#define APP_ELEMENT_ROUNDABOUT_TOF_TRIGGER_DISTANCE_MM (200U)
#define APP_ELEMENT_ROUNDABOUT_DEAD_MS          (200UL)
#define APP_ELEMENT_ROUNDABOUT_DEAD_TICK        (APP_ELEMENT_ROUNDABOUT_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_ROUNDABOUT_TASK_ID          (4U)
#define APP_ELEMENT_ROUNDABOUT_TASK_PRIORITY    (9U)
#define APP_ELEMENT_ROUNDABOUT_PERIOD_MS        (5U)
#define APP_ELEMENT_ROUNDABOUT_FSM_IDLE          (0U)
#define APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE        (1U)
#define APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT     (2U)
#define APP_ELEMENT_ROUNDABOUT_EXIT_DISTANCE_M   (1.0f)
#define APP_ELEMENT_ROUNDABOUT_1_FF_SCALE_DEFAULT (2.0f)
#define APP_ELEMENT_ROUNDABOUT_2_FF_SCALE_DEFAULT (2.0f)
#define APP_ELEMENT_ROUNDABOUT_3_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_1_BIAS_DPS_DEFAULT (-1700.0f)
#define APP_ELEMENT_ROUNDABOUT_2_BIAS_DPS_DEFAULT (-1700.0f)
#define APP_ELEMENT_ROUNDABOUT_3_BIAS_DPS_DEFAULT (-1800.0f)
#define APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT (275.0f)
#define APP_ELEMENT_ROUNDABOUT_2_ANGLE_DEG_DEFAULT (275.0f)
#define APP_ELEMENT_ROUNDABOUT_3_ANGLE_DEG_DEFAULT (170.0f)
#define APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_MS           (40UL)
#define APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_TICK         (APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_DPS_DEFAULT  (1800.0f)

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

static uint32 element_last_raw_timestamp_tick = 0U;
static uint8 element_raw_timestamp_valid = 0U;
static uint32 element_imu_last_sequence = 0U;
/* 圆筒状态机状态 */
static volatile uint8 element_cylinder_tof_near_count = 0U;
static volatile uint8 element_cylinder_pitch_high_count = 0U;
static volatile uint8 element_cylinder_pitch_ret_count = 0U;
static volatile uint8 element_cylinder_active = 0U;
static uint32 element_cylinder_start_tick = 0U;
static uint32 element_cylinder_tof_eval_tick = 0U;
static volatile uint8 element_cylinder_cooldown = 0U;
static uint32 element_cylinder_cooldown_start_tick = 0U;
static volatile uint8 element_cylinder_count = 0U;
static volatile float element_cylinder_count_float = 0.0f;
static volatile uint8 element_seesaw_confirm = 0U;
static volatile uint8 element_seesaw_dead = 0U;
static uint32 element_seesaw_dead_start_tick = 0U;
static volatile uint8 element_seesaw_active = 0U;
static uint32 element_seesaw_active_start_tick = 0U;
static volatile float element_seesaw_event = 0.0f;
static volatile uint8 element_seesaw_slowdown_active = 0U;
static volatile uint32 element_seesaw_slowdown_start_tick = 0U;
static volatile uint8 element_roundabout_confirm = 0U;
static volatile uint32 element_roundabout_dead_start_tick = 0U;
static volatile uint8 element_roundabout_dead = 0U;
static volatile uint8 element_roundabout_count = 0U;
static volatile float element_roundabout_count_float = 0.0f;
static volatile uint8 element_roundabout_gz_high = 0U;
static volatile uint32 element_roundabout_gz_high_tick = 0U;
static volatile uint8 element_roundabout_gz_integrate = 0U;
static volatile float element_roundabout_gz_angle_deg = 0.0f;
static volatile uint8 element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
static float element_roundabout_distance_m = 0.0f;

volatile float app_element_roundabout_bias_yaw_radps = 0.0f;
volatile uint8 app_element_roundabout_bias_active = 0U;
volatile float app_element_roundabout_feedforward_scale = 1.0f;
static float element_roundabout_ff_scale_1 = APP_ELEMENT_ROUNDABOUT_1_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_2 = APP_ELEMENT_ROUNDABOUT_2_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_3 = APP_ELEMENT_ROUNDABOUT_3_FF_SCALE_DEFAULT;
static float element_roundabout_bias_dps_1 = APP_ELEMENT_ROUNDABOUT_1_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_2 = APP_ELEMENT_ROUNDABOUT_2_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_3 = APP_ELEMENT_ROUNDABOUT_3_BIAS_DPS_DEFAULT;
static float element_roundabout_angle_deg_1 = APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_2 = APP_ELEMENT_ROUNDABOUT_2_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_3 = APP_ELEMENT_ROUNDABOUT_3_ANGLE_DEG_DEFAULT;
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
#define ELEMENT_EVENT_CYLINDER_FOUND       (0x01U)
#define ELEMENT_EVENT_SEESAW_FOUND         (0x04U)
#define ELEMENT_EVENT_CYLINDER_END         (0x08U)
#define ELEMENT_EVENT_CYLINDER_TIMEOUT     (0x10U)

static void app_element_cylinder_state_reply(void);
/* static void app_element_roundabout_imu_step(uint8 sample_fresh, uint32 raw_delta_tick); 环岛禁用 */

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
        default:
            return 1.0f;
    }
}

#if 0
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
        default:
            return APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT;
    }
}
#endif

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
    element_cylinder_tof_near_count = 0U;
    element_cylinder_pitch_high_count = 0U;
    element_cylinder_pitch_ret_count = 0U;
    element_cylinder_active = 0U;
    element_cylinder_start_tick = 0U;
    element_cylinder_tof_eval_tick = 0U;
    element_cylinder_cooldown = 0U;
    element_cylinder_cooldown_start_tick = 0U;
    element_cylinder_count = 0U;
    element_cylinder_count_float = 0.0f;
    element_seesaw_confirm = 0U;
    element_seesaw_dead = 0U;
    element_seesaw_active = 0U;
    element_seesaw_event = 0.0f;
    element_seesaw_slowdown_active = 0U;
    element_seesaw_slowdown_start_tick = 0U;
    element_roundabout_confirm = 0U;
    element_roundabout_dead = 0U;
    element_roundabout_count = 0U;
    element_roundabout_count_float = 0.0f;
    element_roundabout_gz_high = 0U;
    element_roundabout_gz_integrate = 0U;
    element_roundabout_gz_angle_deg = 0.0f;
    element_roundabout_last_tick = 0U;
    element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
    element_roundabout_distance_m = 0.0f;
    app_element_roundabout_bias_yaw_radps = 0.0f;
    app_element_roundabout_bias_active = 0U;
    app_element_roundabout_feedforward_scale = 1.0f;
    element_roundabout_event_flags = 0U;
    element_roundabout_event_count = 0U;
    element_realtime_event_flags = 0U;
}

#if 0
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
#endif

static void app_element_cylinder_found(uint32 now)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    element_data.type = APP_ELEMENT_TYPE_CYLINDER;
    element_data.state = APP_ELEMENT_STATE_DONE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 1.0f;
    EA = ea_backup;

    element_cylinder_active = 1U;
    element_cylinder_start_tick = now;
    element_cylinder_tof_eval_tick = now;
    element_cylinder_tof_near_count = 0U;
    element_cylinder_pitch_high_count = 0U;
    element_cylinder_pitch_ret_count = 0U;
    element_cylinder_count++;
    element_cylinder_count_float = (float)element_cylinder_count;

    element_realtime_event_flags |= ELEMENT_EVENT_CYLINDER_FOUND;
}

/* 圆筒结束: 正常结束(timeout=0)或超时强制退出(timeout=1), 清元素状态并进入冷却 */
static void app_element_cylinder_end(uint32 now, uint8 timeout)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    element_data.type = APP_ELEMENT_TYPE_NONE;
    element_data.state = APP_ELEMENT_STATE_IDLE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 0.0f;
    EA = ea_backup;

    element_cylinder_active = 0U;
    element_cylinder_pitch_ret_count = 0U;
    element_cylinder_tof_near_count = 0U;
    element_cylinder_pitch_high_count = 0U;
    element_cylinder_cooldown = 1U;
    element_cylinder_cooldown_start_tick = now;
    element_realtime_event_flags |= (0U != timeout) ?
            ELEMENT_EVENT_CYLINDER_TIMEOUT : ELEMENT_EVENT_CYLINDER_END;
}

/* 圆筒状态机: 每 IMU 帧(新鲜采样)执行一次。
   触发: TOF<200mm 连续2帧(按 TOF 30Hz 采样节拍) 且 pitch∈(120,270)° 连续5帧(0-360 语义, 单点触发);
   结束: pitch 回到 0°±15° 连续5帧 且 TOF>200mm; 触发后 2s 超时强制退出 */
static void app_element_cylinder_step(uint32 now)
{
    app_attitude_data_t attitude;
    uint16 tof_mm;
    uint8 pitch_trigger;
    uint8 pitch_end;
    uint8 ea_backup;

    if(0U != element_cylinder_cooldown)
    {
        if((uint32)(now - element_cylinder_cooldown_start_tick) >= APP_ELEMENT_CYLINDER_COOLDOWN_TICK)
        {
            element_cylinder_cooldown = 0U;
        }
        else
        {
            return;
        }
    }

    app_attitude_get_data(&attitude);
    /* TOF 值由主循环 30Hz 写入, EA 关断防撕裂读 */
    ea_backup = EA;
    EA = 0;
    tof_mm = service_tof_get_distance_mm();
    EA = ea_backup;

    if(0U == element_cylinder_active)
    {
        pitch_trigger = ((0U != attitude.valid) &&
                (attitude.pitch_deg > APP_ELEMENT_CYLINDER_PITCH_TRIGGER_DEG) &&
                (attitude.pitch_deg < APP_ELEMENT_CYLINDER_PITCH_TRIGGER_MAX_DEG)) ? 1U : 0U;
        /* TOF 计数按 TOF 采样节拍推进(30Hz), 使"连续2帧"对应两次真实 TOF 采样 */
        if((uint32)(now - element_cylinder_tof_eval_tick) >= APP_ELEMENT_CYLINDER_TOF_EVAL_TICK)
        {
            element_cylinder_tof_eval_tick = now;
            if(tof_mm < APP_ELEMENT_CYLINDER_TOF_NEAR_MM)
            {
                element_cylinder_tof_near_count++;
            }
            else
            {
                element_cylinder_tof_near_count = 0U;
            }
        }
        if(0U != pitch_trigger)
        {
            element_cylinder_pitch_high_count++;
        }
        else
        {
            element_cylinder_pitch_high_count = 0U;
        }
        if((element_cylinder_tof_near_count >= APP_ELEMENT_CYLINDER_TOF_NEAR_FRAMES) &&
           (element_cylinder_pitch_high_count >= APP_ELEMENT_CYLINDER_PITCH_TRIGGER_FRAMES))
        {
            app_element_cylinder_found(now);
        }
    }
    else
    {
        pitch_end = ((0U != attitude.valid) &&
                ((attitude.pitch_deg <= APP_ELEMENT_CYLINDER_PITCH_END_DEG) ||
                 (attitude.pitch_deg >= (360.0f - APP_ELEMENT_CYLINDER_PITCH_END_DEG)))) ? 1U : 0U;
        if(0U != pitch_end)
        {
            element_cylinder_pitch_ret_count++;
        }
        else
        {
            element_cylinder_pitch_ret_count = 0U;
        }
        if((element_cylinder_pitch_ret_count >= APP_ELEMENT_CYLINDER_PITCH_END_FRAMES) &&
           (tof_mm > APP_ELEMENT_CYLINDER_TOF_NEAR_MM))
        {
            app_element_cylinder_end(now, 0U);
        }
        else if((uint32)(now - element_cylinder_start_tick) >= APP_ELEMENT_CYLINDER_TIMEOUT_TICK)
        {
            app_element_cylinder_end(now, 1U);
        }
    }
}

#if 0
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
#endif

static void app_element_cylinder_state_reply(void)
{
    app_attitude_data_t attitude;

    app_attitude_get_data(&attitude);
    wprint("cylinder_state,%.1f,%u,%u,%u,%u\r\n",
            (double)attitude.pitch_deg,
            (uint16)service_tof_get_distance_mm(),
            (uint16)element_cylinder_active,
            (uint16)element_cylinder_tof_near_count,
            (uint16)element_cylinder_pitch_high_count);
}

static void app_element_roundabout_found(uint32 now)
{
    uint8 ea_backup;

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

    app_element_roundabout_apply_runtime_config();
    element_roundabout_bias_start_tick = now;
    element_roundabout_bias_duration_tick = 0xFFFFFFFFU;
    element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE;
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

    /* FSM非空闲态不检测 */
    if(APP_ELEMENT_ROUNDABOUT_FSM_IDLE != element_roundabout_fsm)
    {
        return;
    }

    /* 环岛判据 = TOF 距离超出阈值 且 电感评分函数判定为环岛 (两者同时满足才计入确认) */
    tof_distance_mm = service_tof_get_distance_mm();
    app_inductor_preprocess_get_data(&inductor);
    roundabout_detected = ((tof_distance_mm > APP_ELEMENT_ROUNDABOUT_TOF_TRIGGER_DISTANCE_MM) &&
            (0U != roundabout_priority_tree_predict(
                    inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1],
                    inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2],
                    inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3],
                    inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4],
                    inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M]))) ? 1U : 0U;
    if(0U != roundabout_detected)
    {
        if((0U == element_seesaw_active) && (0U == element_roundabout_gz_high))
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
// 函数简介     环岛状态机 IMU 步进 - 在 TIM7 中断内硬实时执行
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_roundabout_imu_step();
// 备注信息     从主循环 imu_task 移植而来，使用 element_gyro_snapshot 共享数据，时基独立
//-------------------------------------------------------------------------------------------------------------------
#if 0
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

    /* 追踪gx是否在20ms内超过200°/s（环岛检测阻挡条件） */
    if((0U != sample_fresh) && ((gyro.gyro_x > 200.0f) || (gyro.gyro_x < -200.0f)))
    {
        element_roundabout_gz_high = 1U;
        element_roundabout_gz_high_tick = now;
    }
    else if((0U != element_roundabout_gz_high) &&
            ((uint32)(now - element_roundabout_gz_high_tick) > 20U * APP_ELEMENT_TICK_PER_MS))
    {
        element_roundabout_gz_high = 0U;
    }

    /* 环岛gz积分：达到当前环岛配置角度后退出，第三次环岛直接停车 */
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
            if(3U == element_roundabout_count)
            {
                /* 第3环岛：停车+关负压在中断内即时执行（非阻塞寄存器写） */
                app_speedout_request_stop_all();
                element_roundabout_event_flags |= ELEMENT_RB_EVENT_FINISH;
            }
            else
            {
                /* 出环反向偏置：持续40ms，由到期机制自动清除 */
                app_element_roundabout_bias_yaw_radps = tfpu_mul(APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_DPS_DEFAULT, APP_ELEMENT_DEG_TO_RAD);
                app_element_roundabout_bias_active = 1U;
                element_roundabout_bias_start_tick = now;
                element_roundabout_bias_duration_tick = APP_ELEMENT_ROUNDABOUT_REVERSE_BIAS_TICK;

                element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT;
                element_roundabout_distance_m = 0.0f;
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

    /* 环岛退出距离积分：累计前进1m后恢复空闲态 */
    if(APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT == element_roundabout_fsm)
    {
        service_speed_data_t speed;
        float avg_mps;

        service_speed_get(&speed);
        avg_mps = tfpu_mul(tfpu_sub(speed.left_mps, speed.right_mps), 0.5f);
        element_roundabout_distance_m = tfpu_add(element_roundabout_distance_m,
                tfpu_mul(avg_mps, tfpu_mul(tfpu_int2float((long)control_delta_tick), APP_ELEMENT_TICK_TO_SECOND)));
        if(element_roundabout_distance_m >= APP_ELEMENT_ROUNDABOUT_EXIT_DISTANCE_M)
        {
            element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
            element_roundabout_distance_m = 0.0f;
            element_roundabout_event_flags |= ELEMENT_RB_EVENT_READY;
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

#endif

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
    if(0U != (realtime_flags & ELEMENT_EVENT_CYLINDER_FOUND))
    {
        wprint("cylinder_count,%u\r\n", (uint16)element_cylinder_count);
        wprint("cylinder,1.000\r\n");
    }
    if(0U != (realtime_flags & ELEMENT_EVENT_CYLINDER_END))
    {
        wprint("cylinder_end,1.000\r\n");
    }
    if(0U != (realtime_flags & ELEMENT_EVENT_CYLINDER_TIMEOUT))
    {
        wprint("cylinder_timeout,1.000\r\n");
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
    (void)service_packet_add_variable("round_bias1_dps",
            &element_roundabout_bias_dps_1, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias2_dps",
            &element_roundabout_bias_dps_2, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_bias3_dps",
            &element_roundabout_bias_dps_3, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle1_deg",
            &element_roundabout_angle_deg_1, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle2_deg",
            &element_roundabout_angle_deg_2, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)service_packet_add_variable("round_angle3_deg",
            &element_roundabout_angle_deg_3, APP_ELEMENT_PACKET_SINGLE_COUNT);
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
// 备注信息     圆筒识别: TOF<200mm 连续2帧 且 pitch>60° 连续5帧 触发(0-360 pitch 语义)
//-------------------------------------------------------------------------------------------------------------------
void app_element_imu_task(const service_imu_sample_t *imu)
{
    uint8 ea_backup;
    uint8 sample_fresh;
    uint32 now;

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
    }

    now = service_timetick_what();

    if(0U != sample_fresh)
    {
        ea_backup = EA;
        EA = 0;
        element_data.gyro_x = imu->gyro_x;
        element_data.gyro_y = imu->gyro_y;
        element_data.gyro_z = imu->gyro_z;
        EA = ea_backup;

        /* 圆筒状态机: TOF+pitch 触发/结束/超时(每 IMU 帧一帧) */
        app_element_cylinder_step(now);
    }

    /* 跷跷板检测(当前禁用) */
    // app_element_seesaw_update(now, sample_fresh);

    /* 环岛 IMU 步进(当前禁用) */
    // app_element_roundabout_imu_step(sample_fresh, raw_delta_tick);
}
