#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "bsp_include.h"
#include "service_delay.h"
#include "service_speed.h"

#define SERVICE_SPEED_PI                     (3.1415926f)
#define SERVICE_SPEED_ENCODER_PULSE_PER_REV (1024.0f)
#define SERVICE_SPEED_ENCODER_TO_WHEEL_RATIO (4.4f)
#define SERVICE_SPEED_WHEEL_DIAMETER_M       (0.0252f)
#define SERVICE_SPEED_SAMPLE_PERIOD_MS       (1U)
#define SERVICE_SPEED_SAMPLE_PERIOD_SECOND   (0.001f)

#define SERVICE_SPEED_DISTANCE_PER_PULSE_M \
    ((SERVICE_SPEED_PI * SERVICE_SPEED_WHEEL_DIAMETER_M) / \
    (SERVICE_SPEED_ENCODER_PULSE_PER_REV * SERVICE_SPEED_ENCODER_TO_WHEEL_RATIO))

#define SERVICE_SPEED_MPS_PER_PULSE \
    (SERVICE_SPEED_DISTANCE_PER_PULSE_M / SERVICE_SPEED_SAMPLE_PERIOD_SECOND)

#define SERVICE_SPEED_AVG_WINDOW             (20U)

static volatile service_speed_data_t speed_data = {0.0f, 0.0f};

static float speed_avg_buf_left[SERVICE_SPEED_AVG_WINDOW];
static float speed_avg_buf_right[SERVICE_SPEED_AVG_WINDOW];
static uint8 speed_avg_idx = 0U;
static uint8 speed_avg_cnt = 0U;
static float speed_avg_sum_left = 0.0f;
static float speed_avg_sum_right = 0.0f;

static float service_speed_avg_update(float *buf, float *sum, float raw)
{
    *sum = tfpu_sub(tfpu_add(*sum, raw), buf[speed_avg_idx]);
    buf[speed_avg_idx] = raw;
    return tfpu_div(*sum, (float)speed_avg_cnt);
}

void service_speed_init(void)
{
    uint8 i;

    bsp_encoder_init();
    speed_data.left_mps = 0.0f;
    speed_data.right_mps = 0.0f;
    speed_avg_idx = 0U;
    speed_avg_cnt = 1U;
    speed_avg_sum_left = 0.0f;
    speed_avg_sum_right = 0.0f;
    for(i = 0U; i < SERVICE_SPEED_AVG_WINDOW; i++)
    {
        speed_avg_buf_left[i] = 0.0f;
        speed_avg_buf_right[i] = 0.0f;
    }
    pit_ms_init(TIM3_PIT, SERVICE_SPEED_SAMPLE_PERIOD_MS, service_speed_update);
}

void service_speed_debug(void)
{
    service_speed_data_t testdata;
    service_delay_ms(1000U);
    service_speed_get(&testdata);
    printf("[speed:] %f,%f\r\n",testdata.left_mps,testdata.right_mps);
}

void service_speed_update(void)
{
    bsp_encoder_count_t encoder_delta;
    float raw_left;
    float raw_right;

    bsp_encoder_get_delta(&encoder_delta);
    raw_left = tfpu_mul(tfpu_int2float(encoder_delta.left), SERVICE_SPEED_MPS_PER_PULSE);
    raw_right = tfpu_mul(tfpu_int2float(encoder_delta.right), SERVICE_SPEED_MPS_PER_PULSE);

    speed_data.left_mps = service_speed_avg_update(speed_avg_buf_left, &speed_avg_sum_left, raw_left);
    speed_data.right_mps = service_speed_avg_update(speed_avg_buf_right, &speed_avg_sum_right, raw_right);

    speed_avg_idx++;
    if(speed_avg_idx >= SERVICE_SPEED_AVG_WINDOW)
    {
        speed_avg_idx = 0U;
    }
    if(speed_avg_cnt < SERVICE_SPEED_AVG_WINDOW)
    {
        speed_avg_cnt++;
    }
}

void service_speed_get(service_speed_data_t *out_speed)
{
    if(NULL == out_speed)
    {
        return;
    }

    out_speed->left_mps = speed_data.left_mps;
    out_speed->right_mps = speed_data.right_mps;
}
