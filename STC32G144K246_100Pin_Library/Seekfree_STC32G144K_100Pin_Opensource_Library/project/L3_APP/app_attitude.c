#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "shared_lpf.h"
#include "service_imu.h"
#include "app_attitude.h"

/*
 * 卡尔曼滤波器，每轴独立 2 状态（角度 + 陀螺零偏）：
 *   预测：angle += (gyro - bias) * dt
 *         P = F * P * F^T + Q
 *   观测：融合加速度计角度，修正 angle 和 bias
 *
 * roll/pitch：预测来自陀螺仪，观测来自加速度计
 * yaw：仅预测无观测（纯陀螺积分，会漂移）
 */

#define KALMAN_Q_ANGLE      (0.01f)    // 角度过程噪声 — 越大越信任陀螺预测
#define KALMAN_Q_GYRO       (0.003f)   // 陀螺零偏过程噪声 — 越大零偏收敛越快
#define KALMAN_R_ANGLE      (0.005f)   // 加速度计观测噪声 — 越大越不信任加速度计
#define KALMAN_TICK_PER_S   (10000.0f) // 时间戳 tick 换算为秒 (0.1ms/tick)
#define KALMAN_DT_MIN       (0.0005f)  // 最小 dt 保护
#define KALMAN_DT_MAX       (0.0500f)  // 最大 dt 保护（防断流积分发散）

typedef struct
{
    float angle; // 估计角度 (°)
    float bias;  // 陀螺零偏估计 (°/s)
    float P00;   // 协方差 [0][0]
    float P01;   // 协方差 [0][1]
    float P10;   // 协方差 [1][0]
    float P11;   // 协方差 [1][1]
} kalman_axis_t;

static kalman_axis_t kalman_roll;
static kalman_axis_t kalman_pitch;
static kalman_axis_t kalman_yaw;
static shared_lpf_t attitude_gyro_z_lpf;
static volatile app_attitude_data_t attitude_data;
static uint32 attitude_last_sequence = 0UL;
static uint32 attitude_last_tick = 0UL;
static uint8 attitude_first_frame = 1U;

static void kalman_init(kalman_axis_t *k, float init_angle)
{
    k->angle = init_angle;
    k->bias = 0.0f;
    k->P00 = 1.0f;
    k->P01 = 0.0f;
    k->P10 = 0.0f;
    k->P11 = 1.0f;
}

static void kalman_predict(kalman_axis_t *k, float gyro_dps, float dt)
{
    float dP;
    float dPdt;

    k->angle = tfpu_add(k->angle, tfpu_mul(tfpu_sub(gyro_dps, k->bias), dt));
    dP = tfpu_sub(tfpu_sub(tfpu_mul(dt, k->P11), k->P01), k->P10);
    k->P00 = tfpu_add(k->P00, tfpu_mul(dt, tfpu_add(dP, KALMAN_Q_ANGLE)));
    dPdt = tfpu_mul(dt, k->P11);
    k->P01 = tfpu_sub(k->P01, dPdt);
    k->P10 = tfpu_sub(k->P10, dPdt);
    k->P11 = tfpu_add(k->P11, tfpu_mul(KALMAN_Q_GYRO, dt));
}

static void kalman_update(kalman_axis_t *k, float measurement)
{
    float y = tfpu_sub(measurement, k->angle);
    float s = tfpu_add(k->P00, KALMAN_R_ANGLE);
    float k0 = tfpu_div(k->P00, s);
    float k1 = tfpu_div(k->P10, s);

    k->angle = tfpu_add(k->angle, tfpu_mul(k0, y));
    k->bias = tfpu_add(k->bias, tfpu_mul(k1, y));
    k->P00 = tfpu_sub(k->P00, tfpu_mul(k0, k->P00));
    k->P01 = tfpu_sub(k->P01, tfpu_mul(k0, k->P01));
    k->P10 = tfpu_sub(k->P10, tfpu_mul(k1, k->P00));
    k->P11 = tfpu_sub(k->P11, tfpu_mul(k1, k->P01));
}

void app_attitude_init(void)
{
    service_imu_sample_t imu;
    float gyro_z = 0.0f;
    float roll_init = 0.0f;
    float pitch_init = 0.0f;

    if(0U != service_imu_get_latest_sample(&imu))
    {
        gyro_z = imu.gyro_z;
        roll_init = imu.roll_deg;
        pitch_init = imu.pitch_deg;
    }
    shared_lpf_init(&attitude_gyro_z_lpf, APP_ATTITUDE_GYRO_LPF_ALPHA_DEFAULT, gyro_z);
    kalman_init(&kalman_roll, roll_init);
    kalman_init(&kalman_pitch, pitch_init);
    kalman_init(&kalman_yaw, 0.0f);
    attitude_data.gyro_z = gyro_z;
    attitude_data.roll_deg = roll_init;
    attitude_data.pitch_deg = pitch_init;
    attitude_data.yaw_deg = 0.0f;
    attitude_data.sequence = 0UL;
    attitude_data.valid = 0U;
    attitude_last_sequence = 0UL;
    attitude_last_tick = 0UL;
    attitude_first_frame = 1U;

    if(0U != service_imu_get_latest_sample(&imu))
    {
        app_attitude_update(&imu);
    }
}

void app_attitude_update(const service_imu_sample_t *imu)
{
    uint8 ea_backup;
    float gyro_z;
    float dt;

    if((NULL == imu) || (0U == imu->valid) ||
       (imu->sequence == attitude_last_sequence))
    {
        return;
    }

    if(0U != attitude_first_frame)
    {
        /* 首帧先用加速度计初始化卡尔曼 */
        attitude_first_frame = 0U;
        kalman_init(&kalman_roll, imu->roll_deg);
        kalman_init(&kalman_pitch, imu->pitch_deg);
        kalman_init(&kalman_yaw, 0.0f);
        attitude_last_tick = imu->timestamp_tick;
    }
    else
    {
        dt = tfpu_div(tfpu_int2float((long)(imu->timestamp_tick - attitude_last_tick)), KALMAN_TICK_PER_S);
        if(dt < KALMAN_DT_MIN)
        {
            dt = KALMAN_DT_MIN;
        }
        if(dt > KALMAN_DT_MAX)
        {
            dt = KALMAN_DT_MAX;
        }
        kalman_predict(&kalman_roll, imu->gyro_x, dt);
        kalman_predict(&kalman_pitch, imu->gyro_y, dt);
        kalman_predict(&kalman_yaw, imu->gyro_z, dt);
        /* roll/pitch 用加速度计观测修正，yaw 无观测 */
        kalman_update(&kalman_roll, imu->roll_deg);
        kalman_update(&kalman_pitch, imu->pitch_deg);
        attitude_last_tick = imu->timestamp_tick;
    }

    gyro_z = shared_lpf_update(&attitude_gyro_z_lpf, imu->gyro_z);
    ea_backup = EA;
    EA = 0;
    attitude_data.gyro_z = gyro_z;
    attitude_data.roll_deg = kalman_roll.angle;
    attitude_data.pitch_deg = kalman_pitch.angle;
    attitude_data.yaw_deg = kalman_yaw.angle;
    attitude_data.sequence = imu->sequence;
    attitude_data.valid = imu->valid;
    attitude_last_sequence = imu->sequence;
    EA = ea_backup;
}

void app_attitude_task(void)
{
    service_imu_sample_t imu;

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
    *out_data = attitude_data;
    EA = ea_backup;
}
