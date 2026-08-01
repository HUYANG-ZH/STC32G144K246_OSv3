#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "bsp_include.h"
#include "service_imu.h"
#include "service_buzzer.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"

#define SERVICE_IMU_CALIBRATE_COUNT     (200U)
#define SERVICE_IMU_CALIBRATE_INV       (0.005f)

/* 开机校准监督状态机 (蜂鸣提示 + 2s 延时后启动校准) */
#define SERVICE_IMU_CAL_BOOT_DELAY_TICK (20000UL)    /* 开机后等待 2s (0.1ms/tick) */
#define SERVICE_IMU_CAL_BEEP_MS         (120UL)      /* 单声鸣响时长 ms */
#define SERVICE_IMU_CAL_BEEP_ON_TICK    (1200UL)     /* 鸣响持续 120ms */
#define SERVICE_IMU_CAL_BEEP_GAP_TICK   (1500UL)     /* 声间间隔 150ms */

typedef enum
{
    SERVICE_IMU_CAL_SUPERVISOR_WAIT_BOOT = 0,
    SERVICE_IMU_CAL_SUPERVISOR_BEEPING,
    SERVICE_IMU_CAL_SUPERVISOR_CALIBRATING,
    SERVICE_IMU_CAL_SUPERVISOR_DONE,
} service_imu_cal_supervisor_state_enum;

typedef enum
{
    SERVICE_IMU_CALIBRATION_IDLE = 0,
    SERVICE_IMU_CALIBRATION_GYRO_Z,
    SERVICE_IMU_CALIBRATION_GYRO_X,
    SERVICE_IMU_CALIBRATION_COMPLETE,
} service_imu_calibration_state_enum;

static volatile service_imu_sample_t service_imu_sample;
static uint32 service_imu_last_sequence = 0UL;
static float gyro_x_offset = 0.0f;
static float gyro_z_offset = 0.0f;
static float service_imu_acc_scale = 0.0f;
static float service_imu_gyro_scale = 0.0f;
static float service_imu_calibration_sum = 0.0f;
static uint16 service_imu_calibration_count = 0U;
static volatile service_imu_calibration_state_enum service_imu_calibration_state = SERVICE_IMU_CALIBRATION_IDLE;
static service_imu_cal_supervisor_state_enum cal_supervisor_state = SERVICE_IMU_CAL_SUPERVISOR_WAIT_BOOT;
static service_imu_cal_supervisor_state_enum cal_supervisor_beep_next = SERVICE_IMU_CAL_SUPERVISOR_WAIT_BOOT;
static uint8 cal_supervisor_beep_left = 0U;
static uint8 cal_supervisor_beep_on = 0U;
static uint32 cal_supervisor_beep_deadline = 0UL;
static uint32 cal_supervisor_boot_tick = 0UL;

static void service_imu_begin_calibration(service_imu_calibration_state_enum state);
static void service_imu_update_calibration(const service_imu_sample_t *sample);
static void service_imu_make_sample(service_imu_sample_t *out_data, const bsp_imu_sample_t *raw);
static void service_imu_publish_sample(const service_imu_sample_t *sample);
static void service_imu_cal_supervisor_task(void);
static void service_imu_cal_supervisor_start_beeps(uint8 beep_count,
        service_imu_cal_supervisor_state_enum next_state);
static void service_imu_cal_supervisor_beep_step(uint32 now);

static void service_imu_cal_supervisor_start_beeps(uint8 beep_count,
        service_imu_cal_supervisor_state_enum next_state)
{
    cal_supervisor_beep_left = beep_count;
    cal_supervisor_beep_next = next_state;
    cal_supervisor_beep_on = 0U;
    cal_supervisor_beep_deadline = 0UL;
    cal_supervisor_state = SERVICE_IMU_CAL_SUPERVISOR_BEEPING;
}

static void service_imu_cal_supervisor_beep_step(uint32 now)
{
    if(0U == cal_supervisor_beep_left)
    {
        if(SERVICE_IMU_CAL_SUPERVISOR_CALIBRATING == cal_supervisor_beep_next)
        {
            service_imu_start_calibration();
        }
        cal_supervisor_state = cal_supervisor_beep_next;
        return;
    }

    if((uint32)(now - cal_supervisor_beep_deadline) < 0x80000000UL)
    {
        if(0U != cal_supervisor_beep_on)
        {
            service_buzzer_stop();
            cal_supervisor_beep_on = 0U;
            cal_supervisor_beep_left--;
            cal_supervisor_beep_deadline = now + SERVICE_IMU_CAL_BEEP_GAP_TICK;
        }
        else
        {
            service_buzzer_beep_ms(SERVICE_IMU_CAL_BEEP_MS);
            cal_supervisor_beep_on = 1U;
            cal_supervisor_beep_deadline = now + SERVICE_IMU_CAL_BEEP_ON_TICK;
        }
    }
}

static void service_imu_cal_supervisor_task(void)
{
    uint32 now;

    now = service_timetick_what();
    switch(cal_supervisor_state)
    {
        case SERVICE_IMU_CAL_SUPERVISOR_WAIT_BOOT:
            if((uint32)(now - cal_supervisor_boot_tick) >= SERVICE_IMU_CAL_BOOT_DELAY_TICK)
            {
                /* 开机 2s 后: 短鸣两声提示, 随后开始校准 */
                service_imu_cal_supervisor_start_beeps(2U, SERVICE_IMU_CAL_SUPERVISOR_CALIBRATING);
            }
            break;

        case SERVICE_IMU_CAL_SUPERVISOR_BEEPING:
            service_imu_cal_supervisor_beep_step(now);
            break;

        case SERVICE_IMU_CAL_SUPERVISOR_CALIBRATING:
            if(0U != service_imu_calibration_is_complete())
            {
                /* 校准完成: 短鸣三声提示 */
                service_imu_cal_supervisor_start_beeps(3U, SERVICE_IMU_CAL_SUPERVISOR_DONE);
            }
            break;

        case SERVICE_IMU_CAL_SUPERVISOR_DONE:
        default:
            break;
    }
}

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

static void service_imu_make_sample(service_imu_sample_t *out_data, const bsp_imu_sample_t *raw)
{
    if((NULL == out_data) || (NULL == raw))
    {
        return;
    }
    out_data->gyro_x_raw = raw->gyro_x_raw;
    out_data->gyro_y_raw = raw->gyro_y_raw;
    out_data->gyro_z_raw = raw->gyro_z_raw;
    out_data->acc_x_raw = raw->acc_x_raw;
    out_data->acc_y_raw = raw->acc_y_raw;
    out_data->acc_z_raw = raw->acc_z_raw;
    out_data->gyro_x = tfpu_mul(tfpu_int2float((long)raw->gyro_x_raw), service_imu_gyro_scale);
    out_data->gyro_y = tfpu_mul(tfpu_int2float((long)raw->gyro_y_raw), service_imu_gyro_scale);
    out_data->gyro_z = tfpu_mul(tfpu_int2float((long)raw->gyro_z_raw), service_imu_gyro_scale);
    out_data->acc_x_g = tfpu_mul(tfpu_int2float((long)raw->acc_x_raw), service_imu_acc_scale);
    out_data->acc_y_g = tfpu_mul(tfpu_int2float((long)raw->acc_y_raw), service_imu_acc_scale);
    out_data->acc_z_g = tfpu_mul(tfpu_int2float((long)raw->acc_z_raw), service_imu_acc_scale);
    out_data->roll_deg = raw->roll_deg;
    out_data->pitch_deg = raw->pitch_deg;
    out_data->yaw_deg = raw->yaw_deg;
    out_data->sequence = raw->sequence;
    out_data->drdy_tick = raw->drdy_tick;
    out_data->timestamp_tick = raw->timestamp_tick;
    out_data->valid = raw->valid;
}

static void service_imu_publish_sample(const service_imu_sample_t *sample)
{
    uint8 ea_backup;

    if(NULL == sample)
    {
        return;
    }
    ea_backup = EA;
    EA = 0;
    service_imu_sample = *sample;
    EA = ea_backup;
}

void service_imu_init(void)
{
    service_imu_sample.valid = 0U;
    service_imu_sample.sequence = 0UL;
    service_imu_last_sequence = 0UL;
    gyro_x_offset = 0.0f;
    gyro_z_offset = 0.0f;
    service_imu_acc_scale = 0.0f;
    service_imu_gyro_scale = 0.0f;
    bsp_imu_init();
    if((0U != bsp_imu_is_ready()) &&
       (0.0f < imu660rc_transition_factor[0]) &&
       (0.0f < imu660rc_transition_factor[1]))
    {
        service_imu_acc_scale = tfpu_div(1.0f, imu660rc_transition_factor[0]);
        service_imu_gyro_scale = tfpu_div(1.0f, imu660rc_transition_factor[1]);
    }
    /* 校准不再开机立即启动: 由监督状态机在开机 2s 后蜂鸣两声再启动,
       校准完成后蜂鸣三声提示。 */
    cal_supervisor_state = SERVICE_IMU_CAL_SUPERVISOR_WAIT_BOOT;
    cal_supervisor_boot_tick = service_timetick_what();
    #if __DBGFLAG__
    printf(">>[service_imu_init]\r\n");
    wprint(">>[service_imu_init]\r\n");
    #endif
}

void service_imu_task(void)
{
    bsp_imu_task();
    service_imu_cal_supervisor_task();
}

void service_imu_update(void)
{
    bsp_imu_sample_t raw;
    service_imu_sample_t sample;

    if((0U == bsp_imu_get_latest_sample(&raw)) ||
       (raw.sequence == service_imu_last_sequence))
    {
        return;
    }

    service_imu_make_sample(&sample, &raw);
    service_imu_update_calibration(&sample);
    sample.gyro_x = tfpu_sub(sample.gyro_x, gyro_x_offset);
    sample.gyro_z = tfpu_sub(sample.gyro_z, gyro_z_offset);
    service_imu_publish_sample(&sample);
    service_imu_last_sequence = raw.sequence;
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
        out_data->sequence = 0UL;
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
