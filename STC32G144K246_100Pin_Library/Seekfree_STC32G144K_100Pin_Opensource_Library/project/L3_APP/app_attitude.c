#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "shared_kalman.h"
#include "service_imu.h"
#include "app_attitude.h"

static shared_lpf_t attitude_gyro_z_lpf;
static shared_lpf_t attitude_pitch_lpf;
static shared_kalman_t attitude_roll_kalman;
static volatile app_attitude_data_t attitude_data;
static uint32 attitude_last_sequence = 0UL;
static uint32 attitude_last_timestamp_tick = 0UL;
static uint8 attitude_kalman_ready = 0U;

#define APP_ATTITUDE_PI                  (3.1415926f)
#define APP_ATTITUDE_HALF_PI             (1.5707963f)
#define APP_ATTITUDE_RAD_TO_DEG          (57.2957795f)
#define APP_ATTITUDE_TICK_TO_SECOND      (0.0001f)

static float app_attitude_atan2(float y, float x)
{
    float value;

    if(x > 0.0f)
    {
        value = tfpu_atan(tfpu_div(y, x));
    }
    else if(x < 0.0f)
    {
        value = tfpu_atan(tfpu_div(y, x));
        value = (y >= 0.0f) ? tfpu_add(value, APP_ATTITUDE_PI) :
                tfpu_sub(value, APP_ATTITUDE_PI);
    }
    else if(y > 0.0f)
    {
        value = APP_ATTITUDE_HALF_PI;
    }
    else if(y < 0.0f)
    {
        value = tfpu_sub(0.0f, APP_ATTITUDE_HALF_PI);
    }
    else
    {
        value = 0.0f;
    }

    return value;
}

static void app_attitude_get_accel_angles(const service_imu_sample_t *imu,
        float *roll_deg,
        float *pitch_deg)
{
    float roll_rad;
    float pitch_rad;

    if((NULL == imu) || (NULL == roll_deg) || (NULL == pitch_deg))
    {
        return;
    }

    roll_rad = app_attitude_atan2(imu->acc_y_g, imu->acc_z_g);
    /* pitch 0-360 全象限: 分母用 az 保留重力方向(而非恒正模长),
       倒置(az<0)时 pitch=180°, 与正放(0°)区分; 绕 pitch 轴翻转可全周观测 */
    pitch_rad = app_attitude_atan2(tfpu_sub(0.0f, imu->acc_x_g), imu->acc_z_g);
    *roll_deg = tfpu_mul(roll_rad, APP_ATTITUDE_RAD_TO_DEG);
    *pitch_deg = tfpu_mul(pitch_rad, APP_ATTITUDE_RAD_TO_DEG);
    if(*pitch_deg < 0.0f)
    {
        *pitch_deg = tfpu_add(*pitch_deg, 360.0f);
    }
}

static float app_attitude_get_dt(const service_imu_sample_t *imu)
{
    uint32 delta_tick;
    float dt;

    if(NULL == imu)
    {
        return 0.0001f;
    }
    delta_tick = imu->timestamp_tick - attitude_last_timestamp_tick;
    if(0UL == delta_tick)
    {
        return 0.0001f;
    }
    dt = tfpu_mul(tfpu_int2float((long)delta_tick),
            APP_ATTITUDE_TICK_TO_SECOND);
    if(APP_ATTITUDE_DT_MAX_SECOND < dt)
    {
        dt = APP_ATTITUDE_DT_MAX_SECOND;
    }
    return dt;
}

void app_attitude_init(void)
{
    shared_lpf_init(&attitude_gyro_z_lpf,
            APP_ATTITUDE_GYRO_Z_LPF_ALPHA_DEFAULT, 0.0f);
    shared_lpf_init(&attitude_pitch_lpf,
            APP_ATTITUDE_PITCH_LPF_ALPHA_DEFAULT, 0.0f);
    attitude_last_sequence = 0UL;
    attitude_last_timestamp_tick = 0UL;
    attitude_kalman_ready = 0U;
    attitude_data.roll_deg = 0.0f;
    attitude_data.pitch_deg = 0.0f;
    attitude_data.yaw_deg = 0.0f;
    attitude_data.gyro_z = 0.0f;
    attitude_data.sequence = 0UL;
    attitude_data.timestamp_tick = 0UL;
    attitude_data.valid = 0U;
}

static float app_attitude_pitch_lpf_update(float input)
{
    float delta;
    float output;

    /* 回绕感知一阶低通: 0/360 接缝处(恰为水平姿态)避免线性混叠,
       359→0 按 -1° 处理而非 -359°, 防止平路振动虚假触发 (30,330) */
    output = attitude_pitch_lpf.output;
    delta = tfpu_sub(input, output);
    if(delta > 180.0f)
    {
        delta = tfpu_sub(delta, 360.0f);
    }
    else if(delta < -180.0f)
    {
        delta = tfpu_add(delta, 360.0f);
    }
    output = tfpu_add(output, tfpu_mul(attitude_pitch_lpf.alpha, delta));
    if(output >= 360.0f)
    {
        output = tfpu_sub(output, 360.0f);
    }
    else if(output < 0.0f)
    {
        output = tfpu_add(output, 360.0f);
    }
    attitude_pitch_lpf.output = output;
    return output;
}

void app_attitude_update(const service_imu_sample_t *imu)
{
    uint8 ea_backup;
    float accel_roll_deg;
    float accel_pitch_deg;
    float dt;
    float gyro_z;

    if((NULL == imu) || (0U == imu->valid) || (0UL == imu->sequence) ||
       (imu->sequence == attitude_last_sequence))
    {
        return;
    }

    app_attitude_get_accel_angles(imu, &accel_roll_deg, &accel_pitch_deg);
    gyro_z = shared_lpf_update(&attitude_gyro_z_lpf, imu->gyro_z);
    if(0U == attitude_kalman_ready)
    {
        shared_kalman_init(&attitude_roll_kalman, accel_roll_deg,
                APP_ATTITUDE_KALMAN_Q_ANGLE_DEFAULT,
                APP_ATTITUDE_KALMAN_Q_BIAS_DEFAULT,
                APP_ATTITUDE_KALMAN_R_MEASURE_DEFAULT);
        shared_lpf_reset(&attitude_pitch_lpf, accel_pitch_deg);
        attitude_data.roll_deg = accel_roll_deg;
        attitude_data.pitch_deg = accel_pitch_deg;
        attitude_data.yaw_deg = 0.0f;
        attitude_kalman_ready = 1U;
    }
    else
    {
        dt = app_attitude_get_dt(imu);
        attitude_data.roll_deg = shared_kalman_update(&attitude_roll_kalman,
                accel_roll_deg, imu->gyro_x, dt);
        /* pitch 0-360 无法用连续状态 Kalman(359→0 回绕会发散), 采用加速度计全象限 + 回绕感知低通 */
        attitude_data.pitch_deg = app_attitude_pitch_lpf_update(accel_pitch_deg);
        attitude_data.yaw_deg = tfpu_add(attitude_data.yaw_deg,
                tfpu_mul(gyro_z, dt));
        if(180.0f < attitude_data.yaw_deg)
        {
            attitude_data.yaw_deg = tfpu_sub(attitude_data.yaw_deg, 360.0f);
        }
        else if(-180.0f > attitude_data.yaw_deg)
        {
            attitude_data.yaw_deg = tfpu_add(attitude_data.yaw_deg, 360.0f);
        }
    }
    attitude_last_sequence = imu->sequence;
    attitude_last_timestamp_tick = imu->timestamp_tick;

    ea_backup = EA;
    EA = 0;
    attitude_data.gyro_z = gyro_z;
    attitude_data.sequence = imu->sequence;
    attitude_data.timestamp_tick = imu->timestamp_tick;
    attitude_data.valid = 1U;
    EA = ea_backup;
}

void app_attitude_task(void)
{
    service_imu_sample_t imu;

    /* Compatibility entry point; update() rejects duplicate frame sequences. */
    if(0U != service_imu_get_latest_sample(&imu))
    {
        app_attitude_update(&imu);
    }
}

void app_attitude_get_data(app_attitude_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    out_data->roll_deg = attitude_data.roll_deg;
    out_data->pitch_deg = attitude_data.pitch_deg;
    out_data->yaw_deg = attitude_data.yaw_deg;
    out_data->gyro_z = attitude_data.gyro_z;
    out_data->sequence = attitude_data.sequence;
    out_data->timestamp_tick = attitude_data.timestamp_tick;
    out_data->valid = attitude_data.valid;
    EA = ea_backup;
}
