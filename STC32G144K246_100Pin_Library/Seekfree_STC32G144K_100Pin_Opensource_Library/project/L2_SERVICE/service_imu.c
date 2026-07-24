#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "bsp_include.h"
#include "service_imu.h"

#define SERVICE_IMU_CALIBRATE_COUNT     (200U)
#define SERVICE_IMU_CALIBRATE_INV       (0.005f)

typedef enum
{
    SERVICE_IMU_CALIBRATION_IDLE = 0,
    SERVICE_IMU_CALIBRATION_GYRO_Z,
    SERVICE_IMU_CALIBRATION_GYRO_X,
    SERVICE_IMU_CALIBRATION_COMPLETE,
} service_imu_calibration_state_enum;

static volatile service_imu_sample_t service_imu_sample;
static uint32 service_imu_last_sequence = 0U;
static float gyro_x_offset = 0.0f;
static float gyro_z_offset = 0.0f;
/* Reciprocals are calculated once outside TIM7.  Each complete IMU frame
 * therefore needs TFPU int-to-float + multiply, not six TFPU divisions. */
static float service_imu_acc_scale = 0.0f;
static float service_imu_gyro_scale = 0.0f;
static float service_imu_calibration_sum = 0.0f;
static uint16 service_imu_calibration_count = 0U;
static volatile service_imu_calibration_state_enum service_imu_calibration_state = SERVICE_IMU_CALIBRATION_IDLE;

static void service_imu_begin_calibration(service_imu_calibration_state_enum state)
{
    service_imu_calibration_sum = 0.0f;
    service_imu_calibration_count = 0U;
    service_imu_calibration_state = state;
}

static void service_imu_update_calibration(const service_imu_sample_t *sample)
{
    if(NULL == sample)
    {
        return;
    }

    if(SERVICE_IMU_CALIBRATION_GYRO_Z == service_imu_calibration_state)
    {
        service_imu_calibration_sum = tfpu_add(service_imu_calibration_sum, sample->gyro_z);
        service_imu_calibration_count++;
        if(SERVICE_IMU_CALIBRATE_COUNT <= service_imu_calibration_count)
        {
            gyro_z_offset = tfpu_mul(service_imu_calibration_sum, SERVICE_IMU_CALIBRATE_INV);
            service_imu_begin_calibration(SERVICE_IMU_CALIBRATION_GYRO_X);
        }
    }
    else if(SERVICE_IMU_CALIBRATION_GYRO_X == service_imu_calibration_state)
    {
        service_imu_calibration_sum = tfpu_add(service_imu_calibration_sum, sample->gyro_x);
        service_imu_calibration_count++;
        if(SERVICE_IMU_CALIBRATE_COUNT <= service_imu_calibration_count)
        {
            gyro_x_offset = tfpu_mul(service_imu_calibration_sum, SERVICE_IMU_CALIBRATE_INV);
            service_imu_calibration_state = SERVICE_IMU_CALIBRATION_COMPLETE;
        }
    }
}

void service_imu_init(void)
{
    service_imu_sample.valid = 0U;
    service_imu_sample.sequence = 0U;
    service_imu_last_sequence = 0U;
    gyro_x_offset = 0.0f;
    gyro_z_offset = 0.0f;
    bsp_imu_init();
    service_imu_acc_scale = tfpu_div(1.0f, imu660rc_transition_factor[0]);
    service_imu_gyro_scale = tfpu_div(1.0f, imu660rc_transition_factor[1]);
    service_imu_start_calibration();
}

void service_imu_task(void)
{
    bsp_imu_bootstrap_process();
}

void service_imu_update(void)
{
    bsp_imu_sample_t raw;
    service_imu_sample_t sample;
    uint8 ea_backup;

    if((0U != bsp_imu_get_latest_sample(&raw)) && (raw.sequence != service_imu_last_sequence))
    {
        sample.temperature_raw = raw.temperature_raw;
        sample.gyro_x_raw = raw.gyro_x_raw;
        sample.gyro_y_raw = raw.gyro_y_raw;
        sample.gyro_z_raw = raw.gyro_z_raw;
        sample.acc_x_raw = raw.acc_x_raw;
        sample.acc_y_raw = raw.acc_y_raw;
        sample.acc_z_raw = raw.acc_z_raw;
        sample.gyro_x = tfpu_mul(tfpu_int2float((long)raw.gyro_x_raw), service_imu_gyro_scale);
        sample.gyro_y = tfpu_mul(tfpu_int2float((long)raw.gyro_y_raw), service_imu_gyro_scale);
        sample.gyro_z = tfpu_mul(tfpu_int2float((long)raw.gyro_z_raw), service_imu_gyro_scale);
        sample.acc_x_g = tfpu_mul(tfpu_int2float((long)raw.acc_x_raw), service_imu_acc_scale);
        sample.acc_y_g = tfpu_mul(tfpu_int2float((long)raw.acc_y_raw), service_imu_acc_scale);
        sample.acc_z_g = tfpu_mul(tfpu_int2float((long)raw.acc_z_raw), service_imu_acc_scale);
        sample.sequence = raw.sequence;
        sample.valid = raw.valid;

        service_imu_update_calibration(&sample);
        sample.gyro_x = tfpu_sub(sample.gyro_x, gyro_x_offset);
        sample.gyro_z = tfpu_sub(sample.gyro_z, gyro_z_offset);

        ea_backup = EA;
        EA = 0;
        service_imu_sample = sample;
        EA = ea_backup;
        service_imu_last_sequence = raw.sequence;
    }

    // Request the next full frame after consuming the previous one.  A busy
    // SPI DMA channel simply keeps the current snapshot; it never stalls TIM.
    (void)bsp_imu_request_sample();
}

void service_imu_start_calibration(void)
{
    gyro_x_offset = 0.0f;
    gyro_z_offset = 0.0f;
    service_imu_begin_calibration(SERVICE_IMU_CALIBRATION_GYRO_Z);
}

uint8 service_imu_calibration_is_complete(void)
{
    return (SERVICE_IMU_CALIBRATION_COMPLETE == service_imu_calibration_state) ? 1U : 0U;
}

void service_imu_calibrate_gyro_z(void)
{
    gyro_z_offset = 0.0f;
    service_imu_begin_calibration(SERVICE_IMU_CALIBRATION_GYRO_Z);
}

void service_imu_calibrate_gyro_x(void)
{
    gyro_x_offset = 0.0f;
    service_imu_begin_calibration(SERVICE_IMU_CALIBRATION_GYRO_X);
}

void service_imu_read_gyro(service_imu_gyro_t *out_data)
{
    service_imu_sample_t sample;

    if(NULL == out_data)
    {
        return;
    }

    if(0U == service_imu_get_latest_sample(&sample))
    {
        out_data->gyro_x = 0.0f;
        out_data->gyro_y = 0.0f;
        out_data->gyro_z = 0.0f;
        out_data->sequence = 0U;
        return;
    }

    out_data->gyro_x = sample.gyro_x;
    out_data->gyro_y = sample.gyro_y;
    out_data->gyro_z = sample.gyro_z;
    out_data->sequence = sample.sequence;
}

float service_imu_read_gyro_z(void)
{
    service_imu_sample_t sample;

    if(0U == service_imu_get_latest_sample(&sample))
    {
        return 0.0f;
    }

    return sample.gyro_z;
}

uint8 service_imu_get_latest_sample(service_imu_sample_t *out_data)
{
    uint8 result;
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return 0U;
    }

    ea_backup = EA;
    EA = 0;
    *out_data = service_imu_sample;
    result = service_imu_sample.valid;
    EA = ea_backup;
    return result;
}
