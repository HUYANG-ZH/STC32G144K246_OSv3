#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_timetick.h"
#include "bsp_imu.h"

#ifndef BSP_IMU_QUARTERNION_RATE
#define BSP_IMU_QUARTERNION_RATE        (IMU660RC_QUARTERNION_DISABLE)
#endif

#define BSP_IMU_FRAME_REGISTER           (IMU660RC_OUT_TEMP_L | IMU660RC_SPI_R)
#define BSP_IMU_FRAME_DATA_LENGTH        (14U)
#define BSP_IMU_DMA_LENGTH               (BSP_IMU_FRAME_DATA_LENGTH + 1U)
#define BSP_IMU_BOOT_DMA_LENGTH          (2U)
#define BSP_IMU_BOOT_CONFIG_MAX          (20U)
#define BSP_IMU_BOOT_POWER_WAIT_TICK     (100UL)
#define BSP_IMU_BOOT_RESET_WAIT_TICK     (300UL)
#define BSP_IMU_SPI_TIMEOUT_TICK          (50UL)

typedef struct
{
    uint8 reg;
    uint8 value;
} bsp_imu_boot_write_t;

typedef enum
{
    BSP_IMU_BOOT_IDLE = 0,
    BSP_IMU_BOOT_POWER_WAIT,
    BSP_IMU_BOOT_ID_SUBMIT,
    BSP_IMU_BOOT_ID_WAIT,
    BSP_IMU_BOOT_RESET_SUBMIT,
    BSP_IMU_BOOT_RESET_WAIT,
    BSP_IMU_BOOT_RESET_DELAY,
    BSP_IMU_BOOT_CONFIG_SUBMIT,
    BSP_IMU_BOOT_CONFIG_WAIT,
    BSP_IMU_BOOT_READY,
    BSP_IMU_BOOT_ERROR,
} bsp_imu_boot_state_enum;

static volatile uint8 s_bsp_imu_ready = 0U;
static volatile uint8 s_bsp_imu_transfer_active = 0U;
static volatile uint8 s_bsp_imu_published_index = 0U;
static volatile uint32 s_bsp_imu_sequence = 0U;
static volatile uint32 s_bsp_imu_dma_error_count = 0U;
static bsp_imu_sample_t xdata s_bsp_imu_sample[2];
static uint8 xdata s_bsp_imu_tx[BSP_IMU_DMA_LENGTH];
static uint8 xdata s_bsp_imu_rx[BSP_IMU_DMA_LENGTH];
static uint8 xdata s_bsp_imu_boot_tx[BSP_IMU_BOOT_DMA_LENGTH];
static uint8 xdata s_bsp_imu_boot_rx[BSP_IMU_BOOT_DMA_LENGTH];
static bsp_imu_boot_write_t xdata s_bsp_imu_boot_config[BSP_IMU_BOOT_CONFIG_MAX];
static uint8 s_bsp_imu_boot_config_count = 0U;
static uint8 s_bsp_imu_boot_config_index = 0U;
static uint32 s_bsp_imu_boot_deadline_tick = 0UL;
static uint32 s_bsp_imu_transfer_deadline_tick = 0UL;
static uint32 s_bsp_imu_boot_transfer_deadline_tick = 0UL;
static volatile uint8 s_bsp_imu_boot_transfer_active = 0U;
static volatile uint8 s_bsp_imu_boot_transfer_done = 0U;
static volatile uint8 s_bsp_imu_boot_transfer_success = 0U;
static bsp_imu_boot_state_enum s_bsp_imu_boot_state = BSP_IMU_BOOT_IDLE;

static int16 bsp_imu_make_i16(uint8 low, uint8 high)
{
    return (int16)(((uint16)high << 8) | low);
}

static uint8 bsp_imu_deadline_reached(uint32 now, uint32 deadline)
{
    return ((uint32)(now - deadline) < 0x80000000UL) ? 1U : 0U;
}

static uint8 bsp_imu_boot_deadline_reached(uint32 now)
{
    return bsp_imu_deadline_reached(now, s_bsp_imu_boot_deadline_tick);
}

static uint8 bsp_imu_boot_append_write(uint8 reg, uint8 value)
{
    if(BSP_IMU_BOOT_CONFIG_MAX <= s_bsp_imu_boot_config_count)
    {
        return 0U;
    }

    s_bsp_imu_boot_config[s_bsp_imu_boot_config_count].reg = reg;
    s_bsp_imu_boot_config[s_bsp_imu_boot_config_count].value = value;
    s_bsp_imu_boot_config_count++;
    return 1U;
}

static uint8 bsp_imu_build_boot_config(void)
{
    uint8 acc_config;
    uint8 gyro_config;

    switch(IMU660RC_ACC_SAMPLE_DEFAULT)
    {
        case IMU660RC_ACC_SAMPLE_SGN_2G:
            acc_config = 0x00U;
            imu660rc_transition_factor[0] = 16393.44f;
            break;
        case IMU660RC_ACC_SAMPLE_SGN_4G:
            acc_config = 0x01U;
            imu660rc_transition_factor[0] = 8196.72f;
            break;
        case IMU660RC_ACC_SAMPLE_SGN_8G:
            acc_config = 0x02U;
            imu660rc_transition_factor[0] = 4098.36f;
            break;
        case IMU660RC_ACC_SAMPLE_SGN_16G:
            acc_config = 0x03U;
            imu660rc_transition_factor[0] = 2049.18f;
            break;
        default:
            return 0U;
    }

    switch(IMU660RC_GYRO_SAMPLE_DEFAULT)
    {
        case IMU660RC_GYRO_SAMPLE_SGN_125DPS:
            gyro_config = 0x00U;
            imu660rc_transition_factor[1] = 228.5714f;
            break;
        case IMU660RC_GYRO_SAMPLE_SGN_250DPS:
            gyro_config = 0x01U;
            imu660rc_transition_factor[1] = 114.2857f;
            break;
        case IMU660RC_GYRO_SAMPLE_SGN_500DPS:
            gyro_config = 0x02U;
            imu660rc_transition_factor[1] = 57.1428f;
            break;
        case IMU660RC_GYRO_SAMPLE_SGN_1000DPS:
            gyro_config = 0x03U;
            imu660rc_transition_factor[1] = 28.5714f;
            break;
        case IMU660RC_GYRO_SAMPLE_SGN_2000DPS:
            gyro_config = 0x04U;
            imu660rc_transition_factor[1] = 14.2857f;
            break;
        case IMU660RC_GYRO_SAMPLE_SGN_4000DPS:
            gyro_config = 0x0CU;
            imu660rc_transition_factor[1] = 7.14285f;
            break;
        default:
            return 0U;
    }

    s_bsp_imu_boot_config_count = 0U;
    // Use simple sequential checks.  C251 has fragile diagnostics for long
    // short-circuit expressions, while this form is equivalent and clearer.
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL3, 0x44U))
    {
        return 0U;
    }
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL8, acc_config))
    {
        return 0U;
    }
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL6, gyro_config))
    {
        return 0U;
    }
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL1, 0x15U))
    {
        return 0U;
    }
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL2, 0x18U))
    {
        return 0U;
    }
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL7, 0x01U))
    {
        return 0U;
    }
    if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL9, 0x08U))
    {
        return 0U;
    }

    if(IMU660RC_QUARTERNION_DISABLE != BSP_IMU_QUARTERNION_RATE)
    {
        // Keep each append separate: C251's old expression parser is much
        // less robust with a long OR-chain containing enum casts.
        if(0U == bsp_imu_boot_append_write(IMU660RC_INT2_CTRL, 0x80U))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL4, 0x08U))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_EMB_FUNC_CFG, 0x30U))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL1,
                (uint8)(0x10U | (BSP_IMU_QUARTERNION_RATE + 3U))))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_CTRL2,
                (uint8)(0x10U | (BSP_IMU_QUARTERNION_RATE + 3U))))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_FUNC_CFG_ACCESS, IMU660RC_EMBED_MEM_BANK))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_SFLP_ODR,
                (uint8)(0x43U | (BSP_IMU_QUARTERNION_RATE << 3))))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_EMB_FUNC_EN_A, 0x02U))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_PAGE_RW, 0x00U))
        {
            return 0U;
        }
        if(0U == bsp_imu_boot_append_write(IMU660RC_FUNC_CFG_ACCESS, IMU660RC_MAIN_MEM_BANK))
        {
            return 0U;
        }
    }

    return 1U;
}

static void bsp_imu_boot_dma_complete(spi_index_enum spi_n, uint8 success)
{
    if(SPI_3 != spi_n)
    {
        s_bsp_imu_dma_error_count++;
        return;
    }

    IMU660RC_CS(1);
    s_bsp_imu_boot_transfer_active = 0U;
    s_bsp_imu_boot_transfer_deadline_tick = 0UL;
    s_bsp_imu_boot_transfer_success = success;
    s_bsp_imu_boot_transfer_done = 1U;
}

static uint8 bsp_imu_boot_submit(uint8 command, uint8 value)
{
    uint8 ea_backup;
    uint8 result;
    uint32 now;

    now = service_timetick_what();

    ea_backup = EA;
    EA = 0;
    if((0U != s_bsp_imu_boot_transfer_active) || (0U != spi_dma_async_is_busy(IMU660RC_SPI)))
    {
        if(0U != spi_dma_async_is_busy(IMU660RC_SPI))
        {
            (void)spi_dma_async_abort(IMU660RC_SPI);
            IMU660RC_CS(1);
            s_bsp_imu_dma_error_count++;
        }
        EA = ea_backup;
        return 0U;
    }

    s_bsp_imu_boot_tx[0] = command;
    s_bsp_imu_boot_tx[1] = value;
    s_bsp_imu_boot_transfer_done = 0U;
    s_bsp_imu_boot_transfer_active = 1U;
    IMU660RC_CS(0);
    result = spi_dma_async_transfer(IMU660RC_SPI, s_bsp_imu_boot_tx, s_bsp_imu_boot_rx,
            BSP_IMU_BOOT_DMA_LENGTH, bsp_imu_boot_dma_complete);
    if(0U != result)
    {
        s_bsp_imu_boot_transfer_deadline_tick = now + BSP_IMU_SPI_TIMEOUT_TICK;
    }
    else
    {
        IMU660RC_CS(1);
        s_bsp_imu_boot_transfer_active = 0U;
        s_bsp_imu_boot_transfer_deadline_tick = 0UL;
        s_bsp_imu_dma_error_count++;
    }
    EA = ea_backup;
    return result;
}

/* 0: transfer in flight, 1: success consumed, 2: failed and boot state is terminal. */
static uint8 bsp_imu_boot_take_result(void)
{
    if(0U == s_bsp_imu_boot_transfer_done)
    {
        return 0U;
    }

    s_bsp_imu_boot_transfer_done = 0U;
    if(0U == s_bsp_imu_boot_transfer_success)
    {
        s_bsp_imu_dma_error_count++;
        s_bsp_imu_boot_state = BSP_IMU_BOOT_ERROR;
        return 2U;
    }
    return 1U;
}

// DMA ISR callback: only publish a raw frame.  Floating-point conversion and
// control filtering run in the timer task, never in the transport ISR.
static void bsp_imu_dma_complete(spi_index_enum spi_n, uint8 success)
{
    bsp_imu_sample_t xdata *sample;
    uint8 next_index;
    uint8 ea_backup;
    uint32 next_sequence;

    if(SPI_3 != spi_n)
    {
        s_bsp_imu_dma_error_count++;
        return;
    }

    IMU660RC_CS(1);

    if(0U == success)
    {
        s_bsp_imu_dma_error_count++;
        s_bsp_imu_transfer_active = 0U;
        s_bsp_imu_transfer_deadline_tick = 0UL;
        return;
    }

    next_index = (uint8)(s_bsp_imu_published_index ^ 1U);
    sample = &s_bsp_imu_sample[next_index];
    // RX[0] belongs to the command byte; the 14 payload bytes start at RX[1].
    sample->temperature_raw = bsp_imu_make_i16(s_bsp_imu_rx[1], s_bsp_imu_rx[2]);
    sample->gyro_x_raw = bsp_imu_make_i16(s_bsp_imu_rx[3], s_bsp_imu_rx[4]);
    sample->gyro_y_raw = bsp_imu_make_i16(s_bsp_imu_rx[5], s_bsp_imu_rx[6]);
    sample->gyro_z_raw = bsp_imu_make_i16(s_bsp_imu_rx[7], s_bsp_imu_rx[8]);
    sample->acc_x_raw = bsp_imu_make_i16(s_bsp_imu_rx[9], s_bsp_imu_rx[10]);
    sample->acc_y_raw = bsp_imu_make_i16(s_bsp_imu_rx[11], s_bsp_imu_rx[12]);
    sample->acc_z_raw = bsp_imu_make_i16(s_bsp_imu_rx[13], s_bsp_imu_rx[14]);
    next_sequence = s_bsp_imu_sequence + 1UL;
    sample->sequence = next_sequence;
    sample->valid = 1U;

    /* Keep the BSP transfer lock asserted until RX parsing and the complete
       double-buffer publication are finished.  TIM7 may preempt this DMA ISR,
       but it cannot start another DMA transfer over s_bsp_imu_rx mid-parse. */
    ea_backup = EA;
    EA = 0;
    s_bsp_imu_sequence = next_sequence;
    s_bsp_imu_published_index = next_index;
    s_bsp_imu_transfer_active = 0U;
    s_bsp_imu_transfer_deadline_tick = 0UL;
    EA = ea_backup;
}

void bsp_imu_init(void)
{
    uint8 i;

    s_bsp_imu_ready = 0U;
    s_bsp_imu_transfer_active = 0U;
    s_bsp_imu_published_index = 0U;
    s_bsp_imu_sequence = 0U;
    s_bsp_imu_dma_error_count = 0U;
    s_bsp_imu_transfer_deadline_tick = 0UL;
    for(i = 0U; i < 2U; i++)
    {
        s_bsp_imu_sample[i].valid = 0U;
        s_bsp_imu_sample[i].sequence = 0U;
    }

    (void)imu660rc_init(BSP_IMU_QUARTERNION_RATE);

    s_bsp_imu_ready = 1U;
}

void bsp_imu_bootstrap_process(void)
{
}

uint8 bsp_imu_request_sample(void)
{
    uint8 i;

    if(0U == s_bsp_imu_ready)
    {
        return 0U;
    }

    /* Synchronous read of 14 bytes from IMU660RC (0x20: temp, 0x22: gyro, 0x28: accel).
       Write starting at s_bsp_imu_rx[1] so the DMA completion handler's RX[1..14] layout matches. */
    IMU660RC_CS(0);
    spi_read_8bit_registers(IMU660RC_SPI, BSP_IMU_FRAME_REGISTER, &s_bsp_imu_rx[1], BSP_IMU_FRAME_DATA_LENGTH);
    IMU660RC_CS(1);

    /* Process via the same DMA completion handler so data format is identical. */
    bsp_imu_dma_complete(IMU660RC_SPI, 1U);

    return 1U;
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
