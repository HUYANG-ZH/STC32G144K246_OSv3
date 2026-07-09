#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "service_buzzer.h"
#include "app_inductor_preprocess.h"
#include "app_scheduler.h"
#include "app_element.h"

#define APP_ELEMENT_TICK_PER_MS                 (10UL)
#define APP_ELEMENT_CYLINDER_DEAD_MS            (1000UL)
#define APP_ELEMENT_CYLINDER_DEAD_TICK          (APP_ELEMENT_CYLINDER_DEAD_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_YAW_LIMIT_MS       (500UL)        // 圆筒触发后转向限幅持续时间
#define APP_ELEMENT_CYLINDER_YAW_LIMIT_TICK     (APP_ELEMENT_CYLINDER_YAW_LIMIT_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_WINDOW_MS          (1000UL)
#define APP_ELEMENT_CYLINDER_WINDOW_TICK        (APP_ELEMENT_CYLINDER_WINDOW_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_BUCKET_MS          (10UL)
#define APP_ELEMENT_CYLINDER_BUCKET_TICK        (APP_ELEMENT_CYLINDER_BUCKET_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_BUCKET_COUNT       ((APP_ELEMENT_CYLINDER_WINDOW_MS / APP_ELEMENT_CYLINDER_BUCKET_MS) + 2U)
#define APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_MS  (50UL)
#define APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_TICK \
    (APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_MS * APP_ELEMENT_TICK_PER_MS)
#define APP_ELEMENT_CYLINDER_TRIGGER_DEG        (100.0f)
#define APP_ELEMENT_CYLINDER_GYRO_DEADBAND      (8.0f)
#define APP_ELEMENT_CYLINDER_GYRO_LPF_ALPHA     (0.1f)
#define APP_ELEMENT_TICK_TO_SECOND              (0.0001f)
#define APP_ELEMENT_PACKET_SINGLE_COUNT         (1U)

/* 环岛元素检测参数 */
#define APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT    (5U)          // 连续确认帧数
#define APP_ELEMENT_ROUNDABOUT_DEAD_MS          (1000UL)      // 触发后死区时间
#define APP_ELEMENT_ROUNDABOUT_MID_MIN          (70.0f)       // 中电感归一化下限
#define APP_ELEMENT_ROUNDABOUT_MID_MAX          (90.0f)       // 中电感归一化上限
#define APP_ELEMENT_ROUNDABOUT_Y_SUM_MIN        (190.0f)      // y1+y2下限(外侧电感)
#define APP_ELEMENT_ROUNDABOUT_X_SUM_MAX        (150.0f)      // x1+x2上限(内侧电感)
#define APP_ELEMENT_ROUNDABOUT_TASK_ID          (4U)          // scheduler任务ID
#define APP_ELEMENT_ROUNDABOUT_TASK_PRIORITY    (9U)          // 任务优先级
#define APP_ELEMENT_ROUNDABOUT_PERIOD_MS        (2U)          // 检测周期2ms

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
static uint8 element_roundabout_confirm_count = 0U;
static uint32 element_roundabout_dead_start_tick = 0U;
static uint8 element_roundabout_dead = 0U;

static void app_element_cylinder_state_reply(void);

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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     环岛元素检测任务，2ms周期
// 参数说明     void
// 返回参数     void
// 备注信息     中电感70~90、y1+y2>190、x1+x2<150连续5帧确认
//-------------------------------------------------------------------------------------------------------------------
static void app_element_roundabout_task(void)
{
    app_inductor_preprocess_data_t inductor;
    uint32 now;
    uint8 ea_backup;

    now = service_timetick_what();

    /* 死区到期自动清除 */
    if((0U != element_roundabout_dead) &&
            ((uint32)(now - element_roundabout_dead_start_tick) >=
             (APP_ELEMENT_ROUNDABOUT_DEAD_MS * APP_ELEMENT_TICK_PER_MS)))
    {
        element_roundabout_dead = 0U;
    }

    /* 死区内跳过检测 */
    if(0U != element_roundabout_dead)
    {
        return;
    }

    app_inductor_preprocess_get_data(&inductor);

    if((inductor.normalized[4] >= APP_ELEMENT_ROUNDABOUT_MID_MIN) &&
            (inductor.normalized[4] <= APP_ELEMENT_ROUNDABOUT_MID_MAX) &&
            ((inductor.normalized[0] + inductor.normalized[3]) > APP_ELEMENT_ROUNDABOUT_Y_SUM_MIN) &&
            ((inductor.normalized[1] + inductor.normalized[2]) < APP_ELEMENT_ROUNDABOUT_X_SUM_MAX))
    {
        element_roundabout_confirm_count++;
        if(element_roundabout_confirm_count >= APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT)
        {
            element_roundabout_confirm_count = 0U;
            /* 圆筒优先：圆筒活跃时不发布环岛 */
            if((0U == element_cylinder_dead) && (0U == element_cylinder_yaw_limit_active))
            {
                element_roundabout_dead = 1U;
                element_roundabout_dead_start_tick = now;
                ea_backup = EA;
                EA = 0;
                element_data.type = APP_ELEMENT_TYPE_ROUNDABOUT;
                element_data.state = APP_ELEMENT_STATE_DONE;
                element_data.dir = APP_ELEMENT_DIR_NONE;
                element_data.active = 1.0f;
                EA = ea_backup;
                wprint("roundabout,1.000\r\n");
                service_buzzer_beep_ms(300U);
            }
        }
    }
    else
    {
        element_roundabout_confirm_count = 0U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素识别初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_init();
// 备注信息     启用圆筒识别+环岛识别，初始化后默认无元素
//-------------------------------------------------------------------------------------------------------------------
void app_element_init(void)
{
    app_element_reset();
    (void)service_packet_add_action("cylinder_state", app_element_cylinder_state_reply, 0UL);
    (void)service_packet_add_variable("cylinder_event",
            &element_cylinder_event, APP_ELEMENT_PACKET_SINGLE_COUNT);
    (void)app_scheduler_add(APP_ELEMENT_ROUNDABOUT_TASK_ID, app_element_roundabout_task,
            APP_ELEMENT_ROUNDABOUT_TASK_PRIORITY, APP_ELEMENT_ROUNDABOUT_PERIOD_MS);
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

    if(delta_tick > APP_ELEMENT_CYLINDER_SAMPLE_GAP_MAX_TICK)
    {
        app_element_cylinder_clear();
    }
    else if(0U != delta_tick)
    {
        app_element_cylinder_update(gyro_x, delta_tick, now);
    }

    /* 圆筒转向限幅持续时间到期后清除元素状态 */
    if((0U != element_cylinder_yaw_limit_active) &&
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
}
