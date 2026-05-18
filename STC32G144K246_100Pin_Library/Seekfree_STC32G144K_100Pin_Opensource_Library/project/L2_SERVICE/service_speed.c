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

static volatile service_speed_data_t speed_data = {0.0f, 0.0f};

void service_speed_init(void)
{
    bsp_encoder_init();
    speed_data.left_mps = 0.0f;
    speed_data.right_mps = 0.0f;
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

    bsp_encoder_get_delta(&encoder_delta);
    speed_data.left_mps = tfpu_mul(tfpu_int2float(encoder_delta.left), SERVICE_SPEED_MPS_PER_PULSE);
    speed_data.right_mps = tfpu_mul(tfpu_int2float(encoder_delta.right), SERVICE_SPEED_MPS_PER_PULSE);
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
