#include "zf_common_headfile.h"
#include "sys_tfpu.h"
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

app_element_config_t app_element_config =
{
    APP_ELEMENT_ROUNDABOUT_ENABLE_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_ENTER_ERROR_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_EXIT_ERROR_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_SIGNAL_MIN_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_ENTER_MS_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_INSIDE_MS_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_EXIT_MS_DEFAULT,
    APP_ELEMENT_ROUNDABOUT_DONE_MS_DEFAULT
};

static app_element_roundabout_t element_roundabout;
static volatile app_element_data_t element_data =
{
    APP_ELEMENT_TYPE_NONE,
    APP_ELEMENT_STATE_IDLE,
    APP_ELEMENT_DIR_NONE,
    0.0f
};

static void app_element_tick(void);
static void app_element_roundabout_update(app_element_candidate_t *candidate);
static void app_element_arbitrate(const app_element_candidate_t *roundabout_candidate);

static float app_element_abs(float value)
{
    if(0.0f > value)
    {
        return tfpu_sub(0.0f, value);
    }

    return value;
}

static uint16 app_element_add_period(uint16 value)
{
    if((uint16)(0xFFFFU - APP_ELEMENT_PERIOD_MS) <= value)
    {
        return 0xFFFFU;
    }

    return (uint16)(value + APP_ELEMENT_PERIOD_MS);
}

static void app_element_roundabout_set_state(app_element_state_t state, app_element_dir_t dir)
{
    element_roundabout.state = state;
    element_roundabout.dir = dir;
    element_roundabout.state_ms = 0U;
    element_roundabout.confirm_ms = 0U;
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素识别初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_init();
// 备注信息     使用 APP_ELEMENT_PIT 每 APP_ELEMENT_PERIOD_MS ms 更新一次
//-------------------------------------------------------------------------------------------------------------------
void app_element_init(void)
{
    app_element_roundabout_set_state(APP_ELEMENT_STATE_IDLE, APP_ELEMENT_DIR_NONE);
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
    app_element_candidate_t roundabout_candidate;

    app_element_roundabout_update(&roundabout_candidate);
    app_element_arbitrate(&roundabout_candidate);
}

static void app_element_roundabout_update(app_element_candidate_t *candidate)
{
    app_inductor_preprocess_data_t inductor_data;
    app_motion_preprocess_data_t motion_data;
    app_element_dir_t enter_dir;

    candidate->type = APP_ELEMENT_TYPE_NONE;
    candidate->state = APP_ELEMENT_STATE_IDLE;
    candidate->dir = APP_ELEMENT_DIR_NONE;
    candidate->active = 0.0f;

    app_inductor_preprocess_get_data(&inductor_data);
    app_motion_preprocess_get_data(&motion_data);

    element_roundabout.state_ms = app_element_add_period(element_roundabout.state_ms);

    switch(element_roundabout.state)
    {
        case APP_ELEMENT_STATE_IDLE:
        {
            if(0U != app_element_roundabout_enter_check(&motion_data, &inductor_data, &enter_dir))
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
            if(0U != app_element_roundabout_exit_check(&motion_data))
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

static void app_element_arbitrate(const app_element_candidate_t *roundabout_candidate)
{
    if(0.0f < roundabout_candidate->active)
    {
        element_data.type = roundabout_candidate->type;
        element_data.state = roundabout_candidate->state;
        element_data.dir = roundabout_candidate->dir;
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
