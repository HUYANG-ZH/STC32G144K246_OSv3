#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "app_fuzzy_pid.h"

#define APP_FUZZY_PID_TABLE_SIZE                 (7U)        // 模糊表节点数量
#define APP_FUZZY_PID_DEFAULT_ERROR_SCALE        (1.0f)      // 误差输入缩放
#define APP_FUZZY_PID_DEFAULT_ERROR_DELTA_SCALE  (1.0f)      // 误差变化输入缩放
#define APP_FUZZY_PID_DEFAULT_KP_SCALE           (1.0f)      // Kp变化量输出缩放
#define APP_FUZZY_PID_DEFAULT_KD_SCALE           (1.0f)      // Kd变化量输出缩放

static const float fuzzy_pid_axis[APP_FUZZY_PID_TABLE_SIZE] =
{
    -1.0f, -0.66f, -0.33f, 0.0f, 0.33f, 0.66f, 1.0f
};

static const float fuzzy_pid_kp_table[APP_FUZZY_PID_TABLE_SIZE][APP_FUZZY_PID_TABLE_SIZE] =
{
    { 0.45f,  0.40f,  0.35f,  0.30f,  0.35f,  0.40f,  0.45f},
    { 0.40f,  0.35f,  0.30f,  0.25f,  0.30f,  0.35f,  0.40f},
    { 0.30f,  0.25f,  0.20f,  0.15f,  0.20f,  0.25f,  0.30f},
    { 0.15f,  0.10f,  0.05f,  0.00f,  0.05f,  0.10f,  0.15f},
    { 0.30f,  0.25f,  0.20f,  0.15f,  0.20f,  0.25f,  0.30f},
    { 0.40f,  0.35f,  0.30f,  0.25f,  0.30f,  0.35f,  0.40f},
    { 0.45f,  0.40f,  0.35f,  0.30f,  0.35f,  0.40f,  0.45f},
};

static const float fuzzy_pid_kd_table[APP_FUZZY_PID_TABLE_SIZE][APP_FUZZY_PID_TABLE_SIZE] =
{
    { 0.30f,  0.25f,  0.20f,  0.15f,  0.20f,  0.25f,  0.30f},
    { 0.28f,  0.23f,  0.18f,  0.12f,  0.18f,  0.23f,  0.28f},
    { 0.25f,  0.20f,  0.15f,  0.08f,  0.15f,  0.20f,  0.25f},
    { 0.22f,  0.16f,  0.10f,  0.00f,  0.10f,  0.16f,  0.22f},
    { 0.25f,  0.20f,  0.15f,  0.08f,  0.15f,  0.20f,  0.25f},
    { 0.28f,  0.23f,  0.18f,  0.12f,  0.18f,  0.23f,  0.28f},
    { 0.30f,  0.25f,  0.20f,  0.15f,  0.20f,  0.25f,  0.30f},
};

app_fuzzy_pid_config_t app_fuzzy_pid_config =
{
    APP_FUZZY_PID_DEFAULT_ERROR_SCALE,
    APP_FUZZY_PID_DEFAULT_ERROR_DELTA_SCALE,
    APP_FUZZY_PID_DEFAULT_KP_SCALE,
    APP_FUZZY_PID_DEFAULT_KD_SCALE
};

static float app_fuzzy_pid_clamp(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(max_value < value)
    {
        return max_value;
    }

    return value;
}

static uint8 app_fuzzy_pid_find_index(float value)
{
    uint8 i;

    for(i = 0U; i < (APP_FUZZY_PID_TABLE_SIZE - 1U); i++)
    {
        if(value <= fuzzy_pid_axis[i + 1U])
        {
            return i;
        }
    }

    return (uint8)(APP_FUZZY_PID_TABLE_SIZE - 2U);
}

static float app_fuzzy_pid_axis_rate(float value, uint8 index)
{
    float range;

    range = tfpu_sub(fuzzy_pid_axis[index + 1U], fuzzy_pid_axis[index]);
    if(0.0f == range)
    {
        return 0.0f;
    }

    return tfpu_div(tfpu_sub(value, fuzzy_pid_axis[index]), range);
}

static float app_fuzzy_pid_bilinear(const float table[APP_FUZZY_PID_TABLE_SIZE][APP_FUZZY_PID_TABLE_SIZE],
        float error, float error_delta)
{
    uint8 error_index;
    uint8 error_delta_index;
    float error_rate;
    float error_delta_rate;
    float value_00;
    float value_01;
    float value_10;
    float value_11;
    float value_0;
    float value_1;

    error_index = app_fuzzy_pid_find_index(error);
    error_delta_index = app_fuzzy_pid_find_index(error_delta);
    error_rate = app_fuzzy_pid_axis_rate(error, error_index);
    error_delta_rate = app_fuzzy_pid_axis_rate(error_delta, error_delta_index);

    value_00 = table[error_index][error_delta_index];
    value_01 = table[error_index][error_delta_index + 1U];
    value_10 = table[error_index + 1U][error_delta_index];
    value_11 = table[error_index + 1U][error_delta_index + 1U];

    value_0 = tfpu_add(value_00, tfpu_mul(tfpu_sub(value_01, value_00), error_delta_rate));
    value_1 = tfpu_add(value_10, tfpu_mul(tfpu_sub(value_11, value_10), error_delta_rate));

    return tfpu_add(value_0, tfpu_mul(tfpu_sub(value_1, value_0), error_rate));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     模糊 PID 参数修正初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_fuzzy_pid_init();
//-------------------------------------------------------------------------------------------------------------------
void app_fuzzy_pid_init(void)
{
    app_fuzzy_pid_config.error_scale = APP_FUZZY_PID_DEFAULT_ERROR_SCALE;
    app_fuzzy_pid_config.error_delta_scale = APP_FUZZY_PID_DEFAULT_ERROR_DELTA_SCALE;
    app_fuzzy_pid_config.kp_scale = APP_FUZZY_PID_DEFAULT_KP_SCALE;
    app_fuzzy_pid_config.kd_scale = APP_FUZZY_PID_DEFAULT_KD_SCALE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新模糊 PID 参数变化量
// 参数说明     error           误差输入
// 参数说明     error_delta     误差变化输入
// 参数说明     out_data        参数变化量输出地址
// 返回参数     void
// 使用示例     app_fuzzy_pid_update(error, error_delta, &fuzzy_data);
//-------------------------------------------------------------------------------------------------------------------
void app_fuzzy_pid_update(float error, float error_delta, app_fuzzy_pid_data_t *out_data)
{
    float use_error;
    float use_error_delta;
    float delta_kp;
    float delta_kd;

    if(NULL == out_data)
    {
        return;
    }

    use_error = tfpu_mul(error, app_fuzzy_pid_config.error_scale);
    use_error_delta = tfpu_mul(error_delta, app_fuzzy_pid_config.error_delta_scale);
    use_error = app_fuzzy_pid_clamp(use_error, fuzzy_pid_axis[0], fuzzy_pid_axis[APP_FUZZY_PID_TABLE_SIZE - 1U]);
    use_error_delta = app_fuzzy_pid_clamp(use_error_delta,
            fuzzy_pid_axis[0], fuzzy_pid_axis[APP_FUZZY_PID_TABLE_SIZE - 1U]);

    delta_kp = app_fuzzy_pid_bilinear(fuzzy_pid_kp_table, use_error, use_error_delta);
    delta_kd = app_fuzzy_pid_bilinear(fuzzy_pid_kd_table, use_error, use_error_delta);

    out_data->delta_kp = tfpu_mul(delta_kp, app_fuzzy_pid_config.kp_scale);
    out_data->delta_kd = tfpu_mul(delta_kd, app_fuzzy_pid_config.kd_scale);
}
