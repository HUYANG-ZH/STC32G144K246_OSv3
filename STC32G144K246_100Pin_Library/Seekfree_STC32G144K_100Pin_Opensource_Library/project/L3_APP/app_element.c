#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_imu.h"
#include "app_inductor_preprocess.h"
#include "app_motion_preprocess.h"
#include "app_element.h"

#define APP_ELEMENT_ENABLE_THRESHOLD                    (0.5f)      // 元素识别使能阈值
#define APP_ELEMENT_ROUNDABOUT_ENABLE_DEFAULT           (1.0f)      // 环岛识别默认使能
#define APP_ELEMENT_ROUNDABOUT_ENTER_ERROR_DEFAULT      (0.45f)     // 环岛进入误差阈值
#define APP_ELEMENT_ROUNDABOUT_EXIT_ERROR_DEFAULT       (0.15f)     // 环岛退出误差阈值
#define APP_ELEMENT_ROUNDABOUT_SIGNAL_MIN_DEFAULT       (10.0f)     // 环岛识别最小电感归一化值
#define APP_ELEMENT_ROUNDABOUT_ENTER_MS_DEFAULT         (30U)       // 环岛进入确认时间，单位 ms
#define APP_ELEMENT_ROUNDABOUT_INSIDE_MS_DEFAULT        (120U)      // 环岛入内保持时间，单位 ms
#define APP_ELEMENT_ROUNDABOUT_EXIT_MS_DEFAULT          (50U)       // 环岛退出确认时间，单位 ms
#define APP_ELEMENT_ROUNDABOUT_DONE_MS_DEFAULT          (200U)      // 环岛结束冷却时间，单位 ms
#define APP_ELEMENT_CROSSROAD_ENABLE_DEFAULT            (1.0f)      // 十字识别默认使能
#define APP_ELEMENT_CROSSROAD_CENTER_ERROR_DEFAULT      (0.18f)     // 十字中心误差阈值
#define APP_ELEMENT_CROSSROAD_SIGNAL_MIN_DEFAULT        (18.0f)     // 十字单路最小电感归一化值
#define APP_ELEMENT_CROSSROAD_SIGNAL_SUM_MIN_DEFAULT    (120.0f)    // 十字四路电感归一化和值阈值
#define APP_ELEMENT_CROSSROAD_ENTER_MS_DEFAULT          (20U)       // 十字进入确认时间，单位 ms
#define APP_ELEMENT_CROSSROAD_INSIDE_MS_DEFAULT         (80U)       // 十字入内保持时间，单位 ms
#define APP_ELEMENT_CROSSROAD_EXIT_MS_DEFAULT           (30U)       // 十字退出确认时间，单位 ms
#define APP_ELEMENT_CROSSROAD_DONE_MS_DEFAULT           (150U)      // 十字结束冷却时间，单位 ms
#define APP_ELEMENT_LOST_LINE_SIGNAL_SUM_MAX_DEFAULT    (45.0f)
#define APP_ELEMENT_LOST_LINE_CONFIRM_MS_DEFAULT        (20U)

#define APP_ELEMENT_ROUNDABOUT_MX_STRONG                (62.15f)
#define APP_ELEMENT_ROUNDABOUT_YSUM_SPLIT               (169.03f)
#define APP_ELEMENT_ROUNDABOUT_XBIAS_LOW                (6.59f)
#define APP_ELEMENT_ROUNDABOUT_XBIAS_HIGH               (11.64f)
#define APP_ELEMENT_ROUNDABOUT_M_WEAK                   (93.85f)
#define APP_ELEMENT_ROUNDABOUT_MY_MIN                   (0.56f)
#define APP_ELEMENT_ROUNDABOUT_MY_MAX                   (0.63f)
#define APP_ELEMENT_ROUNDABOUT_EPS                      (0.001f)
#define APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT            (3U)
#define APP_ELEMENT_CYLINDER_ALLSUM_MIN                 (170.0f)
#define APP_ELEMENT_CYLINDER_MY_MAX_LOW                 (0.555f)
#define APP_ELEMENT_CYLINDER_LRSUM_SPLIT                (-2.18f)
#define APP_ELEMENT_CYLINDER_XBIAS_MIN                  (-22.87f)
#define APP_ELEMENT_CYLINDER_LY_MAX_LOW                 (78.70f)
#define APP_ELEMENT_CYLINDER_MEDGE_MAX                  (22.50f)
#define APP_ELEMENT_CYLINDER_LY_MAX_MEDGE               (64.97f)
#define APP_ELEMENT_CYLINDER_EPS                        (0.001f)
#define APP_ELEMENT_CYLINDER_CONFIRM_COUNT              (3U)
#define APP_ELEMENT_UPHILL_GX_THRESHOLD                 (10.0f)
#define APP_ELEMENT_UPHILL_CONFIRM_COUNT                (2U)
#define APP_ELEMENT_UPHILL_GX_LPF_ALPHA                 (0.5f)

typedef struct
{
    app_element_type_t type;
    app_element_state_t state;
    app_element_dir_t dir;
    float active;
} app_element_candidate_t;

typedef struct
{
    app_element_state_t state;
    app_element_dir_t dir;
    uint16 state_ms;
    uint16 confirm_ms;
} app_element_roundabout_t;

typedef struct
{
    app_element_state_t state;
    uint16 state_ms;
    uint16 confirm_ms;
} app_element_crossroad_t;

app_element_config_t app_element_config =
{
    APP_ELEMENT_ROUNDABOUT_ENABLE_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_ENTER_ERROR_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_EXIT_ERROR_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_SIGNAL_MIN_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_ENTER_MS_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_INSIDE_MS_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_EXIT_MS_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_DONE_MS_DEFAULT,
    APP_ELEMENT_CROSSROAD_ENABLE_DEFAULT,
    APP_ELEMENT_CROSSROAD_CENTER_ERROR_DEFAULT,
    APP_ELEMENT_CROSSROAD_SIGNAL_MIN_DEFAULT,
    APP_ELEMENT_CROSSROAD_SIGNAL_SUM_MIN_DEFAULT,
    APP_ELEMENT_CROSSROAD_ENTER_MS_DEFAULT,
    APP_ELEMENT_CROSSROAD_INSIDE_MS_DEFAULT,
    APP_ELEMENT_CROSSROAD_EXIT_MS_DEFAULT,
    APP_ELEMENT_CROSSROAD_DONE_MS_DEFAULT,
    APP_ELEMENT_LOST_LINE_SIGNAL_SUM_MAX_DEFAULT,
    APP_ELEMENT_LOST_LINE_CONFIRM_MS_DEFAULT
};

#if 0
static app_element_roundabout_t element_roundabout;
static app_element_crossroad_t element_crossroad;
#endif
static uint16 element_lost_line_confirm_ms = 0U;
static volatile app_element_data_t element_data =
{
    APP_ELEMENT_TYPE_NONE,
    APP_ELEMENT_STATE_IDLE,
    APP_ELEMENT_DIR_NONE,
    0.0f,
    0.0f, 0.0f, 0.0f
};
static app_element_dir_t element_roundabout_candidate_dir = APP_ELEMENT_DIR_NONE;
static uint8 element_roundabout_confirm_count = 0U;
static uint8 element_cylinder_confirm_count = 0U;
static uint8 element_uphill_confirm_count = 0U;
static uint8 element_gyro_x_lpf_ready = 0U;
static float element_gyro_x_filtered = 0.0f;
static volatile service_imu_gyro_t element_gyro;

static void app_element_tick(void);
void app_element_imu_task(const service_imu_gyro_t *gyro);
static void app_element_update_gyro(void);
static uint8 app_element_uphill_detect_update(void);
static uint8 app_element_cylinder_judge(const app_inductor_preprocess_data_t *inductor_data);
static uint8 app_element_cylinder_detect_update(const app_inductor_preprocess_data_t *inductor_data);
static uint8 app_element_roundabout_judge(const app_inductor_preprocess_data_t *inductor_data,
        app_element_dir_t *out_dir);
static void app_element_roundabout_detect_update(const app_inductor_preprocess_data_t *inductor_data);
#if 0
static void app_element_roundabout_update(app_element_candidate_t *candidate,
        const app_inductor_preprocess_data_t *inductor_data, const app_motion_preprocess_data_t *motion_data);
static void app_element_crossroad_update(app_element_candidate_t *candidate,
        const app_inductor_preprocess_data_t *inductor_data, const app_motion_preprocess_data_t *motion_data);
static void app_element_arbitrate(const app_element_candidate_t *roundabout_candidate,
        const app_element_candidate_t *crossroad_candidate);
#endif
static uint8 app_element_lost_line_update(const app_inductor_preprocess_data_t *inductor_data);

#if 0
static float app_element_abs(float value)
{
    if(0.0f > value)
    {
        return tfpu_sub(0.0f, value);
    }

    return value;
}
#endif

static uint16 app_element_add_period(uint16 value)
{
    if((uint16)(0xFFFFU - APP_ELEMENT_PERIOD_MS) <= value)
    {
        return 0xFFFFU;
    }

    return (uint16)(value + APP_ELEMENT_PERIOD_MS);
}

static uint8 app_element_cylinder_judge(const app_inductor_preprocess_data_t *inductor_data)
{
    float Ly;
    float Lx;
    float Rx;
    float Ry;
    float M;
    float Ysum;
    float Xsum;
    float AllSum;
    float EdgeMean;
    float MY;
    float LRsum;
    float Xbias;
    float MEdge;

    if(NULL == inductor_data)
    {
        return 0U;
    }

    Ly = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1];
    Lx = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2];
    Rx = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3];
    Ry = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4];
    M = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_M];

    Ysum = tfpu_add(Ly, Ry);
    Xsum = tfpu_add(Lx, Rx);
    AllSum = tfpu_add(Ysum, Xsum);
    EdgeMean = tfpu_mul(AllSum, 0.25f);
    MY = tfpu_div(M, tfpu_add(Ysum, APP_ELEMENT_CYLINDER_EPS));
    LRsum = tfpu_sub(tfpu_add(Ly, Lx), tfpu_add(Ry, Rx));
    Xbias = tfpu_sub(Lx, Rx);
    MEdge = tfpu_sub(M, EdgeMean);

    if(AllSum < APP_ELEMENT_CYLINDER_ALLSUM_MIN)
    {
        return 0U;
    }

    if(MY <= APP_ELEMENT_CYLINDER_MY_MAX_LOW)
    {
        if(LRsum <= APP_ELEMENT_CYLINDER_LRSUM_SPLIT)
        {
            if(Xbias > APP_ELEMENT_CYLINDER_XBIAS_MIN)
            {
                return 1U;
            }
            return 0U;
        }

        if(Ly <= APP_ELEMENT_CYLINDER_LY_MAX_LOW)
        {
            return 1U;
        }
        return 0U;
    }

    if((MEdge <= APP_ELEMENT_CYLINDER_MEDGE_MAX) && (Ly <= APP_ELEMENT_CYLINDER_LY_MAX_MEDGE))
    {
        return 1U;
    }

    return 0U;
}

static uint8 app_element_roundabout_judge(const app_inductor_preprocess_data_t *inductor_data,
        app_element_dir_t *out_dir)
{
    float Ly;
    float Lx;
    float Rx;
    float Ry;
    float M;
    float Xsum;
    float Ysum;
    float Mx;
    float Xbias;
    float Ybias;
    float LRsum;
    float MY;
    float LxRx;

    if((NULL == inductor_data) || (NULL == out_dir))
    {
        return 0U;
    }

    Ly = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1];
    Lx = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2];
    Rx = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3];
    Ry = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4];
    M = inductor_data->normalized[APP_INDUCTOR_PREPROCESS_INDEX_M];

    Xsum = tfpu_add(Lx, Rx);
    Ysum = tfpu_add(Ly, Ry);
    Mx = tfpu_sub(M, Xsum);
    Xbias = tfpu_sub(Lx, Rx);
    Ybias = tfpu_sub(Ly, Ry);
    LRsum = tfpu_sub(tfpu_add(Ly, Lx), tfpu_add(Ry, Rx));
    MY = tfpu_div(M, tfpu_add(Ysum, APP_ELEMENT_ROUNDABOUT_EPS));
    LxRx = tfpu_div(tfpu_add(Lx, APP_ELEMENT_ROUNDABOUT_EPS),
            tfpu_add(Rx, APP_ELEMENT_ROUNDABOUT_EPS));

    if(Mx > APP_ELEMENT_ROUNDABOUT_MX_STRONG)
    {
        if(Ysum <= APP_ELEMENT_ROUNDABOUT_YSUM_SPLIT)
        {
            if(Xbias <= APP_ELEMENT_ROUNDABOUT_XBIAS_HIGH)
            {
                if((Ybias <= -30.47f) && (MY <= 0.58f))
                {
                    return 0U;
                }
                *out_dir = APP_ELEMENT_DIR_RIGHT;
                return 2U;
            }
            *out_dir = APP_ELEMENT_DIR_LEFT;
            return 1U;
        }

        if(Xbias > APP_ELEMENT_ROUNDABOUT_XBIAS_LOW)
        {
            *out_dir = APP_ELEMENT_DIR_LEFT;
            return 1U;
        }

        if(MY <= APP_ELEMENT_ROUNDABOUT_MY_MIN)
        {
            if(Ybias > -11.79f)
            {
                *out_dir = APP_ELEMENT_DIR_RIGHT;
                return 2U;
            }
            return 0U;
        }

        if(LRsum <= -28.52f)
        {
            *out_dir = APP_ELEMENT_DIR_RIGHT;
            return 2U;
        }
        *out_dir = APP_ELEMENT_DIR_LEFT;
        return 1U;
    }

    if(LRsum <= 7.62f)
    {
        if(M <= APP_ELEMENT_ROUNDABOUT_M_WEAK)
        {
            if(LRsum <= -57.94f)
            {
                *out_dir = APP_ELEMENT_DIR_RIGHT;
                return 2U;
            }

            if((Ybias > 13.56f) && (M > 85.64f))
            {
                *out_dir = APP_ELEMENT_DIR_RIGHT;
                return 2U;
            }

            if((Ybias <= 13.56f) && (Xbias <= -31.34f) && (LxRx > 0.15f))
            {
                *out_dir = APP_ELEMENT_DIR_LEFT;
                return 1U;
            }

            return 0U;
        }

        if((MY <= APP_ELEMENT_ROUNDABOUT_MY_MIN) || (MY > APP_ELEMENT_ROUNDABOUT_MY_MAX))
        {
            return 0U;
        }

        if(Xbias <= -9.49f)
        {
            if(Xbias <= -29.21f)
            {
                *out_dir = APP_ELEMENT_DIR_LEFT;
                return 1U;
            }

            if(Ysum <= 166.39f)
            {
                *out_dir = APP_ELEMENT_DIR_RIGHT;
                return 2U;
            }

            *out_dir = APP_ELEMENT_DIR_LEFT;
            return 1U;
        }

        *out_dir = APP_ELEMENT_DIR_LEFT;
        return 1U;
    }

    if(Xbias <= 2.88f)
    {
        if(Ysum > 137.66f)
        {
            *out_dir = APP_ELEMENT_DIR_RIGHT;
            return 2U;
        }

        return 0U;
    }

    *out_dir = APP_ELEMENT_DIR_LEFT;
    return 1U;
}

#if 0
static void app_element_roundabout_set_state(app_element_state_t state, app_element_dir_t dir)
{
    element_roundabout.state = state;
    element_roundabout.dir = dir;
    element_roundabout.state_ms = 0U;
    element_roundabout.confirm_ms = 0U;
}

static void app_element_crossroad_set_state(app_element_state_t state)
{
    element_crossroad.state = state;
    element_crossroad.state_ms = 0U;
    element_crossroad.confirm_ms = 0U;
}

static float app_element_roundabout_signal_max(const app_inductor_preprocess_data_t *inductor_data)
{
    uint8 i;
    float max_value;

    max_value = inductor_data->normalized[0];
    for(i = 1U; i < 4U; i++)
    {
        if(max_value < inductor_data->normalized[i])
        {
            max_value = inductor_data->normalized[i];
        }
    }

    return max_value;
}

static uint8 app_element_roundabout_enter_check(const app_motion_preprocess_data_t *motion_data,
        const app_inductor_preprocess_data_t *inductor_data, app_element_dir_t *out_dir)
{
    float error_abs;
    float signal_max;

    if(app_element_config.roundabout_enable < APP_ELEMENT_ENABLE_THRESHOLD)
    {
        return 0U;
    }

    error_abs = app_element_abs(motion_data->line_error);
    if(error_abs < app_element_config.roundabout_enter_error)
    {
        return 0U;
    }

    signal_max = app_element_roundabout_signal_max(inductor_data);
    if(signal_max < app_element_config.roundabout_signal_min)
    {
        return 0U;
    }

    *out_dir = (0.0f < motion_data->line_error) ? APP_ELEMENT_DIR_LEFT : APP_ELEMENT_DIR_RIGHT;
    return 1U;
}

static uint8 app_element_roundabout_exit_check(const app_motion_preprocess_data_t *motion_data)
{
    return (app_element_abs(motion_data->line_error) < app_element_config.roundabout_exit_error) ? 1U : 0U;
}

static uint8 app_element_crossroad_enter_check(const app_motion_preprocess_data_t *motion_data,
        const app_inductor_preprocess_data_t *inductor_data)
{
    uint8 i;
    float signal_sum;

    if(app_element_config.crossroad_enable < APP_ELEMENT_ENABLE_THRESHOLD)
    {
        return 0U;
    }

    if(app_element_abs(motion_data->line_error) > app_element_config.crossroad_center_error)
    {
        return 0U;
    }

    signal_sum = 0.0f;
    for(i = 0U; i < 4U; i++)
    {
        if(inductor_data->normalized[i] < app_element_config.crossroad_signal_min)
        {
            return 0U;
        }
        signal_sum = tfpu_add(signal_sum, inductor_data->normalized[i]);
    }

    return (signal_sum >= app_element_config.crossroad_signal_sum_min) ? 1U : 0U;
}

static uint8 app_element_crossroad_exit_check(const app_inductor_preprocess_data_t *inductor_data)
{
    uint8 i;

    for(i = 0U; i < 4U; i++)
    {
        if(inductor_data->normalized[i] < app_element_config.crossroad_signal_min)
        {
            return 1U;
        }
    }

    return 0U;
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素识别初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_init();
// 备注信息     使用 APP_ELEMENT_PIT 每 APP_ELEMENT_PERIOD_MS ms 更新一次
//-------------------------------------------------------------------------------------------------------------------
void app_element_init(void)
{
    element_lost_line_confirm_ms = 0U;
    element_uphill_confirm_count = 0U;
    element_gyro_x_lpf_ready = 0U;
    element_gyro_x_filtered = 0.0f;
    element_gyro.gyro_x = 0.0f;
    element_gyro.gyro_y = 0.0f;
    element_gyro.gyro_z = 0.0f;
    app_element_tick();
    pit_ms_init(APP_ELEMENT_PIT, APP_ELEMENT_PERIOD_MS, app_element_tick);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取元素识别数据
// 参数说明     out_data        数据输出地址
// 返回参数     void
// 使用示例     app_element_get_data(&element_data);
//-------------------------------------------------------------------------------------------------------------------
void app_element_get_data(app_element_data_t *out_data)
{
    if(NULL == out_data)
    {
        return;
    }

    *out_data = element_data;
}

static void app_element_tick(void)
{
    app_inductor_preprocess_data_t inductor_data;

    app_element_update_gyro();
    app_inductor_preprocess_get_data(&inductor_data);

    if(0U != app_element_lost_line_update(&inductor_data))
    {
        return;
    }

    if(0U != app_element_uphill_detect_update())
    {
        return;
    }

    if(0U != app_element_cylinder_detect_update(&inductor_data))
    {
        return;
    }

    app_element_roundabout_detect_update(&inductor_data);
}

void app_element_imu_task(const service_imu_gyro_t *gyro)
{
    float delta;
    float filtered;
    uint8 lpf_ready;
    uint8 ea_backup;

    if(NULL == gyro)
    {
        return;
    }

    lpf_ready = element_gyro_x_lpf_ready;
    filtered = element_gyro_x_filtered;
    if(0U == lpf_ready)
    {
        filtered = gyro->gyro_x;
    }
    else
    {
        delta = tfpu_sub(gyro->gyro_x, filtered);
        filtered = tfpu_add(filtered,
                tfpu_mul(APP_ELEMENT_UPHILL_GX_LPF_ALPHA, delta));
    }

    ea_backup = EA;
    EA = 0;
    element_gyro_x_filtered = filtered;
    element_gyro_x_lpf_ready = 1U;
    element_gyro.gyro_x = gyro->gyro_x;
    element_gyro.gyro_y = gyro->gyro_y;
    element_gyro.gyro_z = gyro->gyro_z;
    EA = ea_backup;
}

static void app_element_update_gyro(void)
{
    element_data.gyro_x = element_gyro.gyro_x;
    element_data.gyro_y = element_gyro.gyro_y;
    element_data.gyro_z = element_gyro.gyro_z;
}

static uint8 app_element_uphill_detect_update(void)
{
    if(element_gyro_x_filtered > APP_ELEMENT_UPHILL_GX_THRESHOLD)
    {
        if(element_uphill_confirm_count < APP_ELEMENT_UPHILL_CONFIRM_COUNT)
        {
            element_uphill_confirm_count++;
        }
    }
    else
    {
        element_uphill_confirm_count = 0U;
        return 0U;
    }

    element_roundabout_candidate_dir = APP_ELEMENT_DIR_NONE;
    element_roundabout_confirm_count = 0U;
    element_cylinder_confirm_count = 0U;
    if(element_uphill_confirm_count < APP_ELEMENT_UPHILL_CONFIRM_COUNT)
    {
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
        return 1U;
    }

    element_lost_line_confirm_ms = 0U;
    element_data.type = APP_ELEMENT_TYPE_UPHILL;
    element_data.state = APP_ELEMENT_STATE_INSIDE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 1.0f;
    return 1U;
}

static uint8 app_element_cylinder_detect_update(const app_inductor_preprocess_data_t *inductor_data)
{
    if(0U == app_element_cylinder_judge(inductor_data))
    {
        element_cylinder_confirm_count = 0U;
        element_uphill_confirm_count = 0U;
        return 0U;
    }

    if(element_cylinder_confirm_count < APP_ELEMENT_CYLINDER_CONFIRM_COUNT)
    {
        element_cylinder_confirm_count++;
    }

    element_roundabout_candidate_dir = APP_ELEMENT_DIR_NONE;
    element_roundabout_confirm_count = 0U;
    element_uphill_confirm_count = 0U;
    if(element_cylinder_confirm_count < APP_ELEMENT_CYLINDER_CONFIRM_COUNT)
    {
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
        return 1U;
    }

    element_lost_line_confirm_ms = 0U;
    element_data.type = APP_ELEMENT_TYPE_CYLINDER;
    element_data.state = APP_ELEMENT_STATE_INSIDE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 1.0f;
    return 1U;
}

static void app_element_roundabout_detect_update(const app_inductor_preprocess_data_t *inductor_data)
{
    uint8 roundabout_type;
    app_element_dir_t dir;

    dir = APP_ELEMENT_DIR_NONE;
    roundabout_type = app_element_roundabout_judge(inductor_data, &dir);
    if(0U == roundabout_type)
    {
        element_roundabout_candidate_dir = APP_ELEMENT_DIR_NONE;
        element_roundabout_confirm_count = 0U;
        element_cylinder_confirm_count = 0U;
        element_uphill_confirm_count = 0U;
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
        return;
    }

    if(dir == element_roundabout_candidate_dir)
    {
        if(element_roundabout_confirm_count < APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT)
        {
            element_roundabout_confirm_count++;
        }
    }
    else
    {
        element_roundabout_candidate_dir = dir;
        element_roundabout_confirm_count = 1U;
    }

    if(element_roundabout_confirm_count < APP_ELEMENT_ROUNDABOUT_CONFIRM_COUNT)
    {
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
        return;
    }

    element_lost_line_confirm_ms = 0U;
    element_cylinder_confirm_count = 0U;
    element_uphill_confirm_count = 0U;
    element_data.type = APP_ELEMENT_TYPE_ROUNDABOUT;
    element_data.state = APP_ELEMENT_STATE_INSIDE;
    element_data.dir = dir;
    element_data.active = 1.0f;
}

static uint8 app_element_lost_line_update(const app_inductor_preprocess_data_t *inductor_data)
{
    uint8 i;
    float signal_sum;

    signal_sum = 0.0f;
    for(i = 0U; i < 4U; i++)
    {
        signal_sum = tfpu_add(signal_sum, inductor_data->normalized[i]);
    }

    if(signal_sum < app_element_config.lost_line_signal_sum_max)
    {
        element_lost_line_confirm_ms = app_element_add_period(element_lost_line_confirm_ms);
        if(element_lost_line_confirm_ms >= app_element_config.lost_line_confirm_ms)
        {
            element_roundabout_candidate_dir = APP_ELEMENT_DIR_NONE;
            element_roundabout_confirm_count = 0U;
            element_cylinder_confirm_count = 0U;
            element_uphill_confirm_count = 0U;
            element_data.type = APP_ELEMENT_TYPE_LOST_LINE;
            element_data.state = APP_ELEMENT_STATE_INSIDE;
            element_data.dir = APP_ELEMENT_DIR_NONE;
            element_data.active = 1.0f;
            return 1U;
        }
    }
    else
    {
        element_lost_line_confirm_ms = 0U;
    }

    element_data.type = APP_ELEMENT_TYPE_NONE;
    element_data.state = APP_ELEMENT_STATE_IDLE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 0.0f;
    return 0U;
}

#if 0
static void app_element_roundabout_update(app_element_candidate_t *candidate,
        const app_inductor_preprocess_data_t *inductor_data, const app_motion_preprocess_data_t *motion_data)
{
    app_element_dir_t enter_dir;

    candidate->type = APP_ELEMENT_TYPE_NONE;
    candidate->state = APP_ELEMENT_STATE_IDLE;
    candidate->dir = APP_ELEMENT_DIR_NONE;
    candidate->active = 0.0f;

    element_roundabout.state_ms = app_element_add_period(element_roundabout.state_ms);

    switch(element_roundabout.state)
    {
        case APP_ELEMENT_STATE_IDLE:
        {
            if(APP_ELEMENT_STATE_IDLE != element_crossroad.state)
            {
                element_roundabout.confirm_ms = 0U;
                element_roundabout.dir = APP_ELEMENT_DIR_NONE;
            }
            else if(0U != app_element_roundabout_enter_check(motion_data, inductor_data, &enter_dir))
            {
                element_roundabout.confirm_ms = app_element_add_period(element_roundabout.confirm_ms);
                element_roundabout.dir = enter_dir;
                if(element_roundabout.confirm_ms >= app_element_config.roundabout_enter_ms)
                {
                    app_element_roundabout_set_state(APP_ELEMENT_STATE_ENTER, enter_dir);
                }
            }
            else
            {
                element_roundabout.confirm_ms = 0U;
                element_roundabout.dir = APP_ELEMENT_DIR_NONE;
            }
            break;
        }

        case APP_ELEMENT_STATE_ENTER:
        {
            if(element_roundabout.state_ms >= app_element_config.roundabout_inside_ms)
            {
                app_element_roundabout_set_state(APP_ELEMENT_STATE_INSIDE, element_roundabout.dir);
            }
            break;
        }

        case APP_ELEMENT_STATE_INSIDE:
        {
            if(0U != app_element_roundabout_exit_check(motion_data))
            {
                element_roundabout.confirm_ms = app_element_add_period(element_roundabout.confirm_ms);
                if(element_roundabout.confirm_ms >= app_element_config.roundabout_exit_ms)
                {
                    app_element_roundabout_set_state(APP_ELEMENT_STATE_EXIT, element_roundabout.dir);
                }
            }
            else
            {
                element_roundabout.confirm_ms = 0U;
            }
            break;
        }

        case APP_ELEMENT_STATE_EXIT:
        {
            app_element_roundabout_set_state(APP_ELEMENT_STATE_DONE, element_roundabout.dir);
            break;
        }

        case APP_ELEMENT_STATE_DONE:
        {
            if(element_roundabout.state_ms >= app_element_config.roundabout_done_ms)
            {
                app_element_roundabout_set_state(APP_ELEMENT_STATE_IDLE, APP_ELEMENT_DIR_NONE);
            }
            break;
        }

        default:
        {
            app_element_roundabout_set_state(APP_ELEMENT_STATE_IDLE, APP_ELEMENT_DIR_NONE);
            break;
        }
    }

    if(APP_ELEMENT_STATE_IDLE != element_roundabout.state)
    {
        candidate->type = APP_ELEMENT_TYPE_ROUNDABOUT;
        candidate->state = element_roundabout.state;
        candidate->dir = element_roundabout.dir;
        candidate->active = 1.0f;
    }
}

static void app_element_crossroad_update(app_element_candidate_t *candidate,
        const app_inductor_preprocess_data_t *inductor_data, const app_motion_preprocess_data_t *motion_data)
{
    candidate->type = APP_ELEMENT_TYPE_NONE;
    candidate->state = APP_ELEMENT_STATE_IDLE;
    candidate->dir = APP_ELEMENT_DIR_NONE;
    candidate->active = 0.0f;

    element_crossroad.state_ms = app_element_add_period(element_crossroad.state_ms);

    switch(element_crossroad.state)
    {
        case APP_ELEMENT_STATE_IDLE:
        {
            if(APP_ELEMENT_STATE_IDLE != element_roundabout.state)
            {
                element_crossroad.confirm_ms = 0U;
            }
            else if(0U != app_element_crossroad_enter_check(motion_data, inductor_data))
            {
                element_crossroad.confirm_ms = app_element_add_period(element_crossroad.confirm_ms);
                if(element_crossroad.confirm_ms >= app_element_config.crossroad_enter_ms)
                {
                    app_element_crossroad_set_state(APP_ELEMENT_STATE_ENTER);
                }
            }
            else
            {
                element_crossroad.confirm_ms = 0U;
            }
            break;
        }

        case APP_ELEMENT_STATE_ENTER:
        {
            if(element_crossroad.state_ms >= app_element_config.crossroad_inside_ms)
            {
                app_element_crossroad_set_state(APP_ELEMENT_STATE_INSIDE);
            }
            break;
        }

        case APP_ELEMENT_STATE_INSIDE:
        {
            if(0U != app_element_crossroad_exit_check(inductor_data))
            {
                element_crossroad.confirm_ms = app_element_add_period(element_crossroad.confirm_ms);
                if(element_crossroad.confirm_ms >= app_element_config.crossroad_exit_ms)
                {
                    app_element_crossroad_set_state(APP_ELEMENT_STATE_EXIT);
                }
            }
            else
            {
                element_crossroad.confirm_ms = 0U;
            }
            break;
        }

        case APP_ELEMENT_STATE_EXIT:
        {
            app_element_crossroad_set_state(APP_ELEMENT_STATE_DONE);
            break;
        }

        case APP_ELEMENT_STATE_DONE:
        {
            if(element_crossroad.state_ms >= app_element_config.crossroad_done_ms)
            {
                app_element_crossroad_set_state(APP_ELEMENT_STATE_IDLE);
            }
            break;
        }

        default:
        {
            app_element_crossroad_set_state(APP_ELEMENT_STATE_IDLE);
            break;
        }
    }

    if(APP_ELEMENT_STATE_IDLE != element_crossroad.state)
    {
        candidate->type = APP_ELEMENT_TYPE_CROSSROAD;
        candidate->state = element_crossroad.state;
        candidate->dir = APP_ELEMENT_DIR_NONE;
        candidate->active = 1.0f;
    }
}

static void app_element_arbitrate(const app_element_candidate_t *roundabout_candidate,
        const app_element_candidate_t *crossroad_candidate)
{
    if(0.0f < roundabout_candidate->active)
    {
        element_data.type = roundabout_candidate->type;
        element_data.state = roundabout_candidate->state;
        element_data.dir = roundabout_candidate->dir;
        element_data.active = 1.0f;
    }
    else if(0.0f < crossroad_candidate->active)
    {
        element_data.type = crossroad_candidate->type;
        element_data.state = crossroad_candidate->state;
        element_data.dir = crossroad_candidate->dir;
        element_data.active = 1.0f;
    }
    else
    {
        element_data.type = APP_ELEMENT_TYPE_NONE;
        element_data.state = APP_ELEMENT_STATE_IDLE;
        element_data.dir = APP_ELEMENT_DIR_NONE;
        element_data.active = 0.0f;
    }
}
#endif
