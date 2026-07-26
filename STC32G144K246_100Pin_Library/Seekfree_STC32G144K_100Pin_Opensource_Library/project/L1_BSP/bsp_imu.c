#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_timetick.h"
#include "bsp_imu.h"

#ifndef BSP_IMU_QUARTERNION_RATE
#define BSP_IMU_QUARTERNION_RATE        (IMU660RC_QUARTERNION_480HZ)
#endif

/* FIFO frame: gyro word + accelerometer word + SFLP game-vector word. */
#define BSP_IMU_FIFO_WORD_COUNT         (3U)
#define BSP_IMU_FIFO_WORD_LENGTH        (7U)
#define BSP_IMU_FIFO_DMA_LENGTH         (1U + BSP_IMU_FIFO_WORD_COUNT * BSP_IMU_FIFO_WORD_LENGTH)
#define BSP_IMU_FIFO_REGISTER           (0x78U | IMU660RC_SPI_R)
#define BSP_IMU_SPI_TIMEOUT_TICK        (50UL)
#define BSP_IMU_HALF_PI                  (1.5707963f)
#define BSP_IMU_PI                       (3.1415926f)
#define BSP_IMU_RAD_TO_DEG               (57.2957795f)

#define BSP_IMU_FIFO_TAG_GYRO            (0x01U)
#define BSP_IMU_FIFO_TAG_ACC             (0x02U)
#define BSP_IMU_FIFO_TAG_SFLP_GAME       (0x13U)

/* P84 is dedicated IMU CS.  Atomic SET/CLR avoids a P8OUT read-modify-write
 * collision with the motor direction pin on the same port. */
#define BSP_IMU_CS(level)                do { if(0U != (level)) { P8SETB = 0x10U; } else { P8CLRB = 0x10U; } } while(0)

static volatile uint8 s_bsp_imu_ready = 0U;
static volatile uint8 s_bsp_imu_transfer_active = 0U;
static volatile uint8 s_bsp_imu_frame_pending = 0U;
static volatile uint8 s_bsp_imu_frame_ready = 0U;
static volatile uint8 s_bsp_imu_published_index = 0U;
static volatile uint32 s_bsp_imu_sequence = 0UL;
static volatile uint32 s_bsp_imu_dma_error_count = 0UL;
static volatile uint32 s_bsp_imu_transfer_deadline_tick = 0UL;
static volatile uint32 s_bsp_imu_pending_drdy_tick = 0UL;
static uint32 s_bsp_imu_active_drdy_tick = 0UL;
static bsp_imu_sample_t xdata s_bsp_imu_sample[2];
static uint8 xdata s_bsp_imu_fifo_tx[BSP_IMU_FIFO_DMA_LENGTH];
static uint8 xdata s_bsp_imu_fifo_rx[BSP_IMU_FIFO_DMA_LENGTH];

static int16 bsp_imu_make_i16(uint8 low, uint8 high);
static uint8 bsp_imu_deadline_reached(uint32 now, uint32 deadline);
static void bsp_imu_record_dma_error(void) reentrant;
static void bsp_imu_fifo_dma_complete(spi_index_enum spi_n, uint8 success);
static void bsp_imu_fifo_drdy_isr(void);
static uint8 bsp_imu_start_fifo_frame(void);
static void bsp_imu_check_timeout(uint32 now);
static uint32 bsp_imu_fp16_to_float_bits(uint16 h);
static float bsp_imu_tfpu_atan2(float y, float x);
static float bsp_imu_tfpu_asin(float value);
static uint8 bsp_imu_decode_fifo_frame(bsp_imu_sample_t *sample);
static void bsp_imu_publish_fifo_frame(void);

static int16 bsp_imu_make_i16(uint8 low, uint8 high)
{
    return (int16)(((uint16)high << 8) | low);
}

static uint8 bsp_imu_deadline_reached(uint32 now, uint32 deadline)
{
    return ((uint32)(now - deadline) < 0x80000000UL) ? 1U : 0U;
}

static void bsp_imu_record_dma_error(void) reentrant
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    s_bsp_imu_dma_error_count++;
    EA = ea_backup;
}

/* DMA completion is intentionally integer-only.  Floating-point conversion
 * and snapshot publication stay in the foreground task. */
static void bsp_imu_fifo_dma_complete(spi_index_enum spi_n, uint8 success)
{
    BSP_IMU_CS(1U);
    s_bsp_imu_transfer_active = 0U;
    s_bsp_imu_transfer_deadline_tick = 0UL;

    if((SPI_3 != spi_n) || (0U == success))
    {
        /* Keep the watermark event pending so a transient bus error retries. */
        s_bsp_imu_frame_pending = 1U;
        bsp_imu_record_dma_error();
        return;
    }

    s_bsp_imu_frame_ready = 1U;
}

/* INT2 is FIFO watermark, configured for exactly one gyro+acc+SFLP frame. */
static void bsp_imu_fifo_drdy_isr(void)
{
    s_bsp_imu_pending_drdy_tick = service_timetick_what();
    s_bsp_imu_frame_pending = 1U;
}

static uint8 bsp_imu_start_fifo_frame(void)
{
    uint8 ea_backup;
    uint8 result;

    if((0U == s_bsp_imu_ready) || (0U == s_bsp_imu_frame_pending) ||
       (0U != s_bsp_imu_transfer_active) || (0U != s_bsp_imu_frame_ready) ||
       (0U != spi_dma_async_is_busy(IMU660RC_SPI)))
    {
        return 0U;
    }

    ea_backup = EA;
    EA = 0;
    if((0U == s_bsp_imu_frame_pending) || (0U != s_bsp_imu_transfer_active) ||
       (0U != s_bsp_imu_frame_ready))
    {
        EA = ea_backup;
        return 0U;
    }
    s_bsp_imu_active_drdy_tick = s_bsp_imu_pending_drdy_tick;
    s_bsp_imu_frame_pending = 0U;
    s_bsp_imu_transfer_active = 1U;
    s_bsp_imu_transfer_deadline_tick = service_timetick_what() + BSP_IMU_SPI_TIMEOUT_TICK;
    EA = ea_backup;

    BSP_IMU_CS(0U);
    result = spi_dma_async_transfer(IMU660RC_SPI, s_bsp_imu_fifo_tx, s_bsp_imu_fifo_rx,
            BSP_IMU_FIFO_DMA_LENGTH, bsp_imu_fifo_dma_complete);
    if(0U == result)
    {
        BSP_IMU_CS(1U);
        ea_backup = EA;
        EA = 0;
        s_bsp_imu_transfer_active = 0U;
        s_bsp_imu_transfer_deadline_tick = 0UL;
        s_bsp_imu_frame_pending = 1U;
        EA = ea_backup;
        bsp_imu_record_dma_error();
    }
    return result;
}

static void bsp_imu_check_timeout(uint32 now)
{
    uint8 ea_backup;

    if((0U == s_bsp_imu_transfer_active) ||
       (0U == bsp_imu_deadline_reached(now, s_bsp_imu_transfer_deadline_tick)))
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    if((0U != s_bsp_imu_transfer_active) &&
       (0U != bsp_imu_deadline_reached(now, s_bsp_imu_transfer_deadline_tick)))
    {
        (void)spi_dma_async_abort(IMU660RC_SPI);
        BSP_IMU_CS(1U);
        s_bsp_imu_transfer_active = 0U;
        s_bsp_imu_transfer_deadline_tick = 0UL;
        s_bsp_imu_frame_pending = 1U;
        bsp_imu_record_dma_error();
    }
    EA = ea_backup;
}

static uint32 bsp_imu_fp16_to_float_bits(uint16 h)
{
    uint16 h_exp;
    uint16 h_sig;
    uint32 f_sgn;
    uint32 f_exp;
    uint32 f_sig;

    h_exp = (uint16)(h & 0x7c00U);
    h_sig = (uint16)(h & 0x03ffU);
    f_sgn = ((uint32)(h & 0x8000U)) << 16;
    switch(h_exp)
    {
        case 0x0000U:
            if(0U == h_sig)
            {
                return f_sgn;
            }
            h_sig <<= 1;
            h_exp = 0U;
            while(0U == (h_sig & 0x0400U))
            {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32)(h_sig & 0x03ffU)) << 13;
            return f_sgn + f_exp + f_sig;
        case 0x7c00U:
            return f_sgn + 0x7f800000UL + (((uint32)(h & 0x03ffU)) << 13);
        default:
            return f_sgn + (((uint32)(h & 0x7fffU) + 0x1c000UL) << 13);
    }
}

static float bsp_imu_tfpu_atan2(float y, float x)
{
    float value;

    if(x > 0.0f)
    {
        value = tfpu_atan(tfpu_div(y, x));
    }
    else if(x < 0.0f)
    {
        value = tfpu_atan(tfpu_div(y, x));
        value = (y >= 0.0f) ? tfpu_add(value, BSP_IMU_PI) : tfpu_sub(value, BSP_IMU_PI);
    }
    else if(y > 0.0f)
    {
        value = BSP_IMU_HALF_PI;
    }
    else if(y < 0.0f)
    {
        value = tfpu_sub(0.0f, BSP_IMU_HALF_PI);
    }
    else
    {
        value = 0.0f;
    }
    return value;
}

static float bsp_imu_tfpu_asin(float value)
{
    float denominator;

    if(value >= 1.0f)
    {
        return BSP_IMU_HALF_PI;
    }
    if(value <= -1.0f)
    {
        return tfpu_sub(0.0f, BSP_IMU_HALF_PI);
    }
    denominator = tfpu_sqrt(tfpu_sub(1.0f, tfpu_mul(value, value)));
    return tfpu_atan(tfpu_div(value, denominator));
}

static uint8 bsp_imu_decode_fifo_frame(bsp_imu_sample_t *sample)
{
    uint8 word;
    uint8 offset;
    uint8 tag;
    uint8 gyro_seen = 0U;
    uint8 acc_seen = 0U;
    uint8 quaternion_seen = 0U;
    uint16 qx_half = 0U;
    uint16 qy_half = 0U;
    uint16 qz_half = 0U;
    float qx;
    float qy;
    float qz;
    float qw;
    float norm_squared;
    float norm;
    float temp_a;
    float temp_b;

    if(NULL == sample)
    {
        return 0U;
    }

    for(word = 0U; word < BSP_IMU_FIFO_WORD_COUNT; word++)
    {
        offset = (uint8)(1U + word * BSP_IMU_FIFO_WORD_LENGTH);
        tag = (uint8)(s_bsp_imu_fifo_rx[offset] >> 3);
        if(BSP_IMU_FIFO_TAG_GYRO == tag)
        {
            if(0U != gyro_seen)
            {
                return 0U;
            }
            sample->gyro_x_raw = bsp_imu_make_i16(s_bsp_imu_fifo_rx[offset + 1U], s_bsp_imu_fifo_rx[offset + 2U]);
            sample->gyro_y_raw = bsp_imu_make_i16(s_bsp_imu_fifo_rx[offset + 3U], s_bsp_imu_fifo_rx[offset + 4U]);
            sample->gyro_z_raw = bsp_imu_make_i16(s_bsp_imu_fifo_rx[offset + 5U], s_bsp_imu_fifo_rx[offset + 6U]);
            gyro_seen = 1U;
        }
        else if(BSP_IMU_FIFO_TAG_ACC == tag)
        {
            if(0U != acc_seen)
            {
                return 0U;
            }
            sample->acc_x_raw = bsp_imu_make_i16(s_bsp_imu_fifo_rx[offset + 1U], s_bsp_imu_fifo_rx[offset + 2U]);
            sample->acc_y_raw = bsp_imu_make_i16(s_bsp_imu_fifo_rx[offset + 3U], s_bsp_imu_fifo_rx[offset + 4U]);
            sample->acc_z_raw = bsp_imu_make_i16(s_bsp_imu_fifo_rx[offset + 5U], s_bsp_imu_fifo_rx[offset + 6U]);
            acc_seen = 1U;
        }
        else if(BSP_IMU_FIFO_TAG_SFLP_GAME == tag)
        {
            if(0U != quaternion_seen)
            {
                return 0U;
            }
            qx_half = (uint16)(((uint16)s_bsp_imu_fifo_rx[offset + 2U] << 8) | s_bsp_imu_fifo_rx[offset + 1U]);
            qy_half = (uint16)(((uint16)s_bsp_imu_fifo_rx[offset + 4U] << 8) | s_bsp_imu_fifo_rx[offset + 3U]);
            qz_half = (uint16)(((uint16)s_bsp_imu_fifo_rx[offset + 6U] << 8) | s_bsp_imu_fifo_rx[offset + 5U]);
            quaternion_seen = 1U;
        }
        else
        {
            return 0U;
        }
    }

    if((0U == gyro_seen) || (0U == acc_seen) || (0U == quaternion_seen) ||
       (0x7c00U == (qx_half & 0x7c00U)) || (0x7c00U == (qy_half & 0x7c00U)) ||
       (0x7c00U == (qz_half & 0x7c00U)))
    {
        return 0U;
    }

    *(uint32 *)(&qx) = bsp_imu_fp16_to_float_bits(qx_half);
    *(uint32 *)(&qy) = bsp_imu_fp16_to_float_bits(qy_half);
    *(uint32 *)(&qz) = bsp_imu_fp16_to_float_bits(qz_half);
    norm_squared = tfpu_add(tfpu_add(tfpu_mul(qx, qx), tfpu_mul(qy, qy)), tfpu_mul(qz, qz));
    if(norm_squared > 1.01f)
    {
        return 0U;
    }
    qw = (norm_squared >= 1.0f) ? 0.0f : tfpu_sqrt(tfpu_sub(1.0f, norm_squared));
    norm = tfpu_sqrt(tfpu_add(norm_squared, tfpu_mul(qw, qw)));
    if(0U == (norm > 0.001f))
    {
        return 0U;
    }

    /* The old page path read [qw,qx,qy,qz] and published [qx,qy,qw,qz].
     * Preserve that project-facing convention while FIFO supplies [qx,qy,qz]. */
    sample->quaternion_0 = tfpu_div(qx, norm);
    sample->quaternion_1 = tfpu_div(qy, norm);
    sample->quaternion_2 = tfpu_div(qw, norm);
    sample->quaternion_3 = tfpu_div(qz, norm);

    temp_a = tfpu_mul(2.0f, tfpu_add(tfpu_mul(sample->quaternion_1, sample->quaternion_3),
            tfpu_mul(sample->quaternion_0, sample->quaternion_2)));
    temp_b = tfpu_sub(1.0f, tfpu_mul(2.0f, tfpu_add(tfpu_mul(sample->quaternion_1, sample->quaternion_1),
            tfpu_mul(sample->quaternion_0, sample->quaternion_0))));
    sample->roll_deg = tfpu_mul(bsp_imu_tfpu_atan2(temp_a, temp_b), BSP_IMU_RAD_TO_DEG);
    temp_a = tfpu_mul(2.0f, tfpu_sub(tfpu_mul(sample->quaternion_0, sample->quaternion_3),
            tfpu_mul(sample->quaternion_1, sample->quaternion_2)));
    sample->pitch_deg = tfpu_mul(tfpu_sub(0.0f, bsp_imu_tfpu_asin(temp_a)), BSP_IMU_RAD_TO_DEG);
    temp_a = tfpu_mul(2.0f, tfpu_add(tfpu_mul(sample->quaternion_0, sample->quaternion_1),
            tfpu_mul(sample->quaternion_2, sample->quaternion_3)));
    temp_b = tfpu_sub(1.0f, tfpu_mul(2.0f, tfpu_add(tfpu_mul(sample->quaternion_0, sample->quaternion_0),
            tfpu_mul(sample->quaternion_2, sample->quaternion_2))));
    sample->yaw_deg = tfpu_mul(bsp_imu_tfpu_atan2(temp_a, temp_b), BSP_IMU_RAD_TO_DEG);
    if(sample->yaw_deg < 0.0f)
    {
        sample->yaw_deg = tfpu_add(sample->yaw_deg, 360.0f);
    }
    return 1U;
}

static void bsp_imu_publish_fifo_frame(void)
{
    bsp_imu_sample_t xdata *sample;
    uint8 current_index;
    uint8 next_index;
    uint8 ea_backup;
    uint32 next_sequence;

    current_index = s_bsp_imu_published_index;
    next_index = (uint8)(current_index ^ 1U);
    s_bsp_imu_sample[next_index] = s_bsp_imu_sample[current_index];
    sample = &s_bsp_imu_sample[next_index];
    if(0U == bsp_imu_decode_fifo_frame(sample))
    {
        bsp_imu_record_dma_error();
        return;
    }

    next_sequence = s_bsp_imu_sequence + 1UL;
    sample->sequence = next_sequence;
    sample->drdy_tick = s_bsp_imu_active_drdy_tick;
    sample->timestamp_tick = service_timetick_what();
    sample->valid = 1U;

    ea_backup = EA;
    EA = 0;
    s_bsp_imu_sequence = next_sequence;
    s_bsp_imu_published_index = next_index;
    EA = ea_backup;
}

void bsp_imu_init(void)
{
    uint8 i;
    uint8 ea_backup;

    s_bsp_imu_ready = 0U;
    s_bsp_imu_transfer_active = 0U;
    s_bsp_imu_frame_pending = 0U;
    s_bsp_imu_frame_ready = 0U;
    s_bsp_imu_published_index = 0U;
    s_bsp_imu_sequence = 0UL;
    s_bsp_imu_dma_error_count = 0UL;
    s_bsp_imu_transfer_deadline_tick = 0UL;
    s_bsp_imu_pending_drdy_tick = 0UL;
    s_bsp_imu_active_drdy_tick = 0UL;
    for(i = 0U; i < 2U; i++)
    {
        s_bsp_imu_sample[i].valid = 0U;
        s_bsp_imu_sample[i].sequence = 0UL;
        s_bsp_imu_sample[i].drdy_tick = 0UL;
        s_bsp_imu_sample[i].timestamp_tick = 0UL;
    }
    for(i = 0U; i < BSP_IMU_FIFO_DMA_LENGTH; i++)
    {
        s_bsp_imu_fifo_tx[i] = 0x00U;
    }
    s_bsp_imu_fifo_tx[0] = BSP_IMU_FIFO_REGISTER;

    if((0U != imu660rc_init(BSP_IMU_QUARTERNION_RATE)) ||
       (0U != imu660rc_enable_unified_fifo()))
    {
        bsp_imu_record_dma_error();
        return;
    }

    spi_dma_init(IMU660RC_SPI, SPI_MODE0, IMU660RC_SPI_SPEED, IMU660RC_SPC_PIN,
            IMU660RC_SDI_PIN, IMU660RC_SDO_PIN, SPI_CS_NULL);

    ea_backup = EA;
    EA = 0;
    gpio_int_irq_handlers[((IMU660RC_INT2_PIN & 0x0f00U) >> 8)]
            [IMU660RC_INT2_PIN & 0x000fU] = bsp_imu_fifo_drdy_isr;
    /* imu660rc_init temporarily owned INT2.  Do not lose a watermark that
     * became active in the short hand-off window before our callback arrived. */
    if(0U != P37)
    {
        s_bsp_imu_pending_drdy_tick = service_timetick_what();
        s_bsp_imu_frame_pending = 1U;
    }
    s_bsp_imu_ready = 1U;
    EA = ea_backup;
}

void bsp_imu_task(void)
{
    uint8 ea_backup;
    uint8 frame_ready;

    if(0U == s_bsp_imu_ready)
    {
        return;
    }

    bsp_imu_check_timeout(service_timetick_what());
    ea_backup = EA;
    EA = 0;
    frame_ready = s_bsp_imu_frame_ready;
    s_bsp_imu_frame_ready = 0U;
    EA = ea_backup;
    if(0U != frame_ready)
    {
        bsp_imu_publish_fifo_frame();
    }
    (void)bsp_imu_start_fifo_frame();
}

void bsp_imu_bootstrap_process(void)
{
    bsp_imu_task();
}

uint8 bsp_imu_request_sample(void)
{
    /* FIFO watermark is the sole sampling clock; never issue a duplicate read. */
    return 0U;
}

uint8 bsp_imu_get_latest_sample(bsp_imu_sample_t *out_data)
{
    uint8 index;
    uint8 result;
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return 0U;
    }

    ea_backup = EA;
    EA = 0;
    index = s_bsp_imu_published_index;
    result = s_bsp_imu_sample[index].valid;
    *out_data = s_bsp_imu_sample[index];
    EA = ea_backup;
    return result;
}

uint32 bsp_imu_get_dma_error_count(void)
{
    uint32 result;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    result = s_bsp_imu_dma_error_count;
    EA = ea_backup;
    return result;
}

void bsp_imu_read_gyro(bsp_imu_gyro_t *out_data)
{
    bsp_imu_sample_t sample;

    if(NULL == out_data)
    {
        return;
    }
    if(0U == bsp_imu_get_latest_sample(&sample))
    {
        out_data->gyro_x = 0.0f;
        out_data->gyro_y = 0.0f;
        out_data->gyro_z = 0.0f;
        return;
    }
    out_data->gyro_x = imu660rc_gyro_transition(sample.gyro_x_raw);
    out_data->gyro_y = imu660rc_gyro_transition(sample.gyro_y_raw);
    out_data->gyro_z = imu660rc_gyro_transition(sample.gyro_z_raw);
}

float bsp_imu_read_gyro_z(void)
{
    bsp_imu_sample_t sample;

    if(0U == bsp_imu_get_latest_sample(&sample))
    {
        return 0.0f;
    }
    return imu660rc_gyro_transition(sample.gyro_z_raw);
}

uint8 bsp_imu_is_ready(void)
{
    return s_bsp_imu_ready;
}
