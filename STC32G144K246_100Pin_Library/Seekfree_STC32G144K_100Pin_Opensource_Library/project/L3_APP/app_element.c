#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "service_speed.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "service_buzzer.h"
#include "app_inductor_preprocess.h"
#include "app_speedout.h"
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
#define APP_ELEMENT_TICK_TO_SECOND              (0.0001f)
#define APP_ELEMENT_DEG_TO_RAD                  (0.0174532925f)
#define APP_ELEMENT_PACKET_SINGLE_COUNT         (1U)

#define APP_ELEMENT_SEESAW_CONFIRM_COUNT        (3U)
#define APP_ELEMENT_SEESAW_DEAD_MS              (500UL)
#define APP_ELEMENT_SEESAW_DEAD_TICK            (APP_ELEMENT_SEESAW_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_SEESAW_ACTIVE_MS            (100UL)
#define APP_ELEMENT_SEESAW_ACTIVE_TICK          (APP_ELEMENT_SEESAW_ACTIVE_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_SEESAW_SCORE_THRESHOLD      (1020)

#define APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT    (1U)
#define APP_ELEMENT_ROUNDABOUT_DEAD_MS          (200UL)
#define APP_ELEMENT_ROUNDABOUT_DEAD_TICK        (APP_ELEMENT_ROUNDABOUT_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_ROUNDABOUT_SCORE_THRESHOLD  (0.0f)
#define APP_ELEMENT_ROUNDABOUT_TASK_ID          (4U)
#define APP_ELEMENT_ROUNDABOUT_TASK_PRIORITY    (9U)
#define APP_ELEMENT_ROUNDABOUT_PERIOD_MS        (1U)
#define APP_ELEMENT_ROUNDABOUT_FSM_IDLE          (0U)
#define APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE        (1U)
#define APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT     (2U)
#define APP_ELEMENT_ROUNDABOUT_EXIT_DISTANCE_M   (1.0f)
#define APP_ELEMENT_ROUNDABOUT_1_FF_SCALE_DEFAULT (2.0f)
#define APP_ELEMENT_ROUNDABOUT_2_FF_SCALE_DEFAULT (2.0f)
#define APP_ELEMENT_ROUNDABOUT_3_FF_SCALE_DEFAULT (1.0f)
#define APP_ELEMENT_ROUNDABOUT_1_BIAS_DPS_DEFAULT (-2250.0f)
#define APP_ELEMENT_ROUNDABOUT_2_BIAS_DPS_DEFAULT (-2250.0f)
#define APP_ELEMENT_ROUNDABOUT_3_BIAS_DPS_DEFAULT (0.0f)
#define APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT (330.0f)
#define APP_ELEMENT_ROUNDABOUT_2_ANGLE_DEG_DEFAULT (330.0f)
#define APP_ELEMENT_ROUNDABOUT_3_ANGLE_DEG_DEFAULT (180.0f)

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
static uint32 element_last_tick = 0U;
static uint32 element_cylinder_dead_start_tick = 0U;
static uint32 element_cylinder_yaw_limit_start_tick = 0U;
static uint8 element_cylinder_yaw_limit_active = 0U;
static float element_cylinder_angle_pos_deg = 0.0f;
static float element_cylinder_angle_neg_deg = 0.0f;
static float element_cylinder_gyro_x = 0.0f;
static float element_cylinder_event = 0.0f;
static uint8 element_cylinder_dead = 0U;
static uint8 element_gyro_x_lpf_ready = 0U;
static uint8 element_cylinder_bucket_head = 0U;
static uint8 element_cylinder_bucket_count = 0U;
static uint8 element_seesaw_confirm = 0U;
static uint8 element_seesaw_dead = 0U;
static uint32 element_seesaw_dead_start_tick = 0U;
static uint8 element_seesaw_gz_high = 0U;
static uint32 element_seesaw_gz_high_tick = 0U;
static uint8 element_seesaw_active = 0U;
static uint32 element_seesaw_active_start_tick = 0U;
static float element_seesaw_event = 0.0f;
static uint8 element_roundabout_confirm = 0U;
static uint32 element_roundabout_dead_start_tick = 0U;
static uint8 element_roundabout_dead = 0U;
static uint8 element_roundabout_count = 0U;
static float element_roundabout_count_float = 0.0f;
static uint8 element_roundabout_gz_high = 0U;
static uint32 element_roundabout_gz_high_tick = 0U;
static uint8 element_roundabout_gz_integrate = 0U;
static float element_roundabout_gz_angle_deg = 0.0f;
static uint8 element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
static float element_roundabout_distance_m = 0.0f;

float app_element_roundabout_bias_yaw_radps = 0.0f;
uint8 app_element_roundabout_bias_active = 0U;
float app_element_roundabout_feedforward_scale = 1.0f;
static float element_roundabout_ff_scale_1 = APP_ELEMENT_ROUNDABOUT_1_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_2 = APP_ELEMENT_ROUNDABOUT_2_FF_SCALE_DEFAULT;
static float element_roundabout_ff_scale_3 = APP_ELEMENT_ROUNDABOUT_3_FF_SCALE_DEFAULT;
static float element_roundabout_bias_dps_1 = APP_ELEMENT_ROUNDABOUT_1_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_2 = APP_ELEMENT_ROUNDABOUT_2_BIAS_DPS_DEFAULT;
static float element_roundabout_bias_dps_3 = APP_ELEMENT_ROUNDABOUT_3_BIAS_DPS_DEFAULT;
static float element_roundabout_angle_deg_1 = APP_ELEMENT_ROUNDABOUT_1_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_2 = APP_ELEMENT_ROUNDABOUT_2_ANGLE_DEG_DEFAULT;
static float element_roundabout_angle_deg_3 = APP_ELEMENT_ROUNDABOUT_3_ANGLE_DEG_DEFAULT;
static uint32 element_roundabout_bias_start_tick = 0U;
static uint32 element_roundabout_bias_duration_tick = 0U;

static void app_element_cylinder_state_reply(void);

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
    uint8 i;

    element_data.type = APP_ELEMENT_TYPE_NONE;
    element_data.state = APP_ELEMENT_STATE_IDLE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 0.0f;
    element_data.gyro_x = 0.0f;
    element_data.gyro_y = 0.0f;
    element_data.gyro_z = 0.0f;
    element_last_tick = service_timetick_what();
    element_cylinder_dead_start_tick = 0U;
    element_cylinder_angle_pos_deg = 0.0f;
    element_cylinder_angle_neg_deg = 0.0f;
    element_cylinder_gyro_x = 0.0f;
    element_cylinder_event = 0.0f;
    element_cylinder_dead = 0U;
    element_gyro_x_lpf_ready = 0U;
    element_cylinder_bucket_head = 0U;
    element_cylinder_bucket_count = 0U;
    element_seesaw_confirm = 0U;
    element_seesaw_dead = 0U;
    element_seesaw_gz_high = 0U;
    element_seesaw_active = 0U;
    element_seesaw_event = 0.0f;
    element_roundabout_confirm = 0U;
    element_roundabout_dead = 0U;
    element_roundabout_count = 0U;
    element_roundabout_count_float = 0.0f;
    element_roundabout_gz_high = 0U;
    element_roundabout_gz_integrate = 0U;
    element_roundabout_gz_angle_deg = 0.0f;
    element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
    element_roundabout_distance_m = 0.0f;
    app_element_roundabout_bias_yaw_radps = 0.0f;
    app_element_roundabout_bias_active = 0U;
    app_element_roundabout_feedforward_scale = 1.0f;
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
    app_element_cylinder_clear();

    wprint("cylinder,1.000\r\n");
    service_buzzer_beep_ms(300U);
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

static float app_element_roundabout_score(const app_inductor_preprocess_data_t *inductor)
{
    float y1 = inductor->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1];
    float x1 = inductor->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2];
    float x2 = inductor->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3];
    float y2 = inductor->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4];
    float m  = inductor->normalized[APP_INDUCTOR_PREPROCESS_INDEX_M];
    float score_a;
    float score_b;
    float score_c;
    float sr;
    float t;

    /* 分支 A：原有环岛特征 */
    score_a = tfpu_add(tfpu_mul(103.613f, y1), tfpu_mul(23.045f, x1));
    score_a = tfpu_add(score_a, tfpu_mul(373.933f, m));

    t = tfpu_mul(0.135f, tfpu_mul(y1, y1));
    score_a = tfpu_sub(score_a, t);

    t = tfpu_mul(0.689f, tfpu_mul(x2, x2));
    score_a = tfpu_add(score_a, t);

    t = tfpu_mul(0.703f, tfpu_mul(x2, y2));
    score_a = tfpu_sub(score_a, t);

    t = tfpu_mul(0.937f, tfpu_mul(x2, m));
    score_a = tfpu_sub(score_a, t);

    t = tfpu_mul(1.116f, tfpu_mul(y2, y2));
    score_a = tfpu_add(score_a, t);

    t = tfpu_mul(2.881f, tfpu_mul(m, m));
    score_a = tfpu_sub(score_a, t);

    /* 分支 B：新环岛 5 特征 */
    score_b = tfpu_sub(tfpu_mul(-12.226f, y1), tfpu_mul(17.723f, x1));
    score_b = tfpu_sub(score_b, tfpu_mul(119.330f, x2));
    score_b = tfpu_add(score_b, tfpu_mul(328.974f, y2));

    t = tfpu_mul(0.954f, tfpu_mul(y1, y1));
    score_b = tfpu_sub(score_b, t);

    t = tfpu_mul(0.871f, tfpu_mul(y1, x1));
    score_b = tfpu_add(score_b, t);

    t = tfpu_mul(2.299f, tfpu_mul(y1, y2));
    score_b = tfpu_add(score_b, t);

    t = tfpu_mul(0.363f, tfpu_mul(x1, x1));
    score_b = tfpu_add(score_b, t);

    t = tfpu_mul(0.835f, tfpu_mul(x1, x2));
    score_b = tfpu_sub(score_b, t);

    t = tfpu_mul(0.216f, tfpu_mul(x1, m));
    score_b = tfpu_add(score_b, t);

    t = tfpu_mul(0.819f, tfpu_mul(x2, x2));
    score_b = tfpu_add(score_b, t);

    t = tfpu_mul(1.978f, tfpu_mul(y2, y2));
    score_b = tfpu_sub(score_b, t);

    t = tfpu_mul(1.837f, tfpu_mul(m, m));
    score_b = tfpu_sub(score_b, t);

    /* 分支 C：新环岛 6、7 特征 */
    score_c = tfpu_add(tfpu_mul(138.507f, y1), tfpu_mul(262.993f, m));

    t = tfpu_mul(0.204f, tfpu_mul(y1, y1));
    score_c = tfpu_sub(score_c, t);

    t = tfpu_mul(0.547f, tfpu_mul(y1, x2));
    score_c = tfpu_sub(score_c, t);

    t = tfpu_mul(0.055f, tfpu_mul(x1, x1));
    score_c = tfpu_sub(score_c, t);

    t = tfpu_mul(0.281f, tfpu_mul(x1, x2));
    score_c = tfpu_sub(score_c, t);

    t = tfpu_mul(0.750f, tfpu_mul(x1, y2));
    score_c = tfpu_add(score_c, t);

    t = tfpu_mul(0.541f, tfpu_mul(x2, x2));
    score_c = tfpu_add(score_c, t);

    t = tfpu_mul(0.261f, tfpu_mul(y2, m));
    score_c = tfpu_sub(score_c, t);

    t = tfpu_mul(2.282f, tfpu_mul(m, m));
    score_c = tfpu_sub(score_c, t);

    sr = tfpu_sub(score_a, 21736.0f);
    t = tfpu_sub(score_b, 8425.0f);
    if(t > sr)
    {
        sr = t;
    }

    t = tfpu_sub(score_c, 17502.0f);
    if(t > sr)
    {
        sr = t;
    }

    return sr;
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
    wprint("roundabout,1.000,%u\r\n", (uint16)element_roundabout_count);
    service_buzzer_beep_ms(300U);

    app_element_roundabout_apply_runtime_config();
    element_roundabout_bias_start_tick = now;
    element_roundabout_bias_duration_tick = 0xFFFFFFFFU;
    element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_ACTIVE;
    element_roundabout_gz_integrate = 1U;
    element_roundabout_gz_angle_deg = 0.0f;
}

static void app_element_roundabout_task(void)
{
    app_inductor_preprocess_data_t inductor;
    uint32 now;
    float score;

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

    app_inductor_preprocess_get_data(&inductor);

    score = app_element_roundabout_score(&inductor);
    if(score >= APP_ELEMENT_ROUNDABOUT_SCORE_THRESHOLD)
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

static void app_element_roundabout_clear_count(void)
{
    element_roundabout_count = 0U;
    element_roundabout_count_float = 0.0f;
    app_element_roundabout_bias_active = 0U;
    app_element_roundabout_bias_yaw_radps = 0.0f;
    app_element_roundabout_feedforward_scale = 1.0f;
    wprint("roundabout_count,0.000\r\n");
}

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

    wprint("seesaw,1.000\r\n");
    service_buzzer_beep_ms(300U);
}

static void app_element_seesaw_update(uint32 now)
{
    app_inductor_preprocess_data_t inductor;
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

    app_inductor_preprocess_get_data(&inductor);

    score = (int32)(-81 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1]
                    -56 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2]
                    -204 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3]
                    -56 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4]
                    +159 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M])
            - 2 * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M] * (int16)inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M];

    if((score >= APP_ELEMENT_SEESAW_SCORE_THRESHOLD) && (0U != element_seesaw_gz_high))
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
    (void)service_packet_add_variable("seesaw_event",
            &element_seesaw_event, APP_ELEMENT_PACKET_SINGLE_COUNT);
    pit_ms_init(APP_ELEMENT_ROUNDABOUT_PIT, APP_ELEMENT_ROUNDABOUT_PERIOD_MS, app_element_roundabout_task);
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
void app_element_imu_task(const service_imu_gyro_t *gyro)
{
    uint8 ea_backup;
    uint32 now;
    uint32 delta_tick;
    float gyro_x;

    if(NULL == gyro)
    {
        return;
    }

    now = service_timetick_what();
    delta_tick = now - element_last_tick;
    element_last_tick = now;

    if(0U == element_gyro_x_lpf_ready)
    {
        shared_lpf_reset(&element_gyro_x_lpf, gyro->gyro_x);
        element_gyro_x_lpf_ready = 1U;
    }
    gyro_x = shared_lpf_update(&element_gyro_x_lpf, gyro->gyro_x);

    ea_backup = EA;
    EA = 0;
    element_data.gyro_x = gyro_x;
    element_data.gyro_y = gyro->gyro_y;
    element_data.gyro_z = gyro->gyro_z;
    EA = ea_backup;

    /* 追踪gz是否在50ms内出现过>50°/s（供跷跷板检测） */
    if((gyro->gyro_z > 120.0f) || (gyro->gyro_z < -120.0f))
    {
        element_seesaw_gz_high = 1U;
        element_seesaw_gz_high_tick = now;
    }
    else if((0U != element_seesaw_gz_high) &&
            ((uint32)(now - element_seesaw_gz_high_tick) > 50U * APP_ELEMENT_TICK_PER_MS))
    {
        element_seesaw_gz_high = 0U;
    }

    /* 追踪gz是否在20ms内超过200°/s（环岛检测阻挡条件） */
    if((gyro->gyro_z > 200.0f) || (gyro->gyro_z < -200.0f))
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
    if(0U != element_roundabout_gz_integrate)
    {
        float target_angle_deg;
        float delta_angle = tfpu_mul(gyro->gyro_z,
                tfpu_mul(tfpu_int2float((long)delta_tick), APP_ELEMENT_TICK_TO_SECOND));

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
                app_speedout_stop();
            }
            else
            {
                element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_EXIT_WAIT;
                element_roundabout_distance_m = 0.0f;
                wprint("roundabout_exit,1.000\r\n");
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
                tfpu_mul(avg_mps, tfpu_mul(tfpu_int2float((long)delta_tick), APP_ELEMENT_TICK_TO_SECOND)));
        if(element_roundabout_distance_m >= APP_ELEMENT_ROUNDABOUT_EXIT_DISTANCE_M)
        {
            element_roundabout_fsm = APP_ELEMENT_ROUNDABOUT_FSM_IDLE;
            element_roundabout_distance_m = 0.0f;
            wprint("roundabout_ready,1.000\r\n");
        }
    }

    if(delta_tick > APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_TICK)
    {
        app_element_cylinder_clear();
    }
    else if(0U != delta_tick)
    {
        app_element_cylinder_update(gyro_x, delta_tick, now);
    }

    /* 跷跷板检测 */
    app_element_seesaw_update(now);

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

    /* 环岛角速度偏置到期清除 */
    if((0U != app_element_roundabout_bias_active) &&
            ((uint32)(now - element_roundabout_bias_start_tick) >= element_roundabout_bias_duration_tick))
    {
        app_element_roundabout_bias_active = 0U;
        app_element_roundabout_bias_yaw_radps = 0.0f;
    }
}
