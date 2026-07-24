#include "zf_common_headfile.h"
#include "bsp_inductor.h"

#define ADC_CHANNEL_1 ADC2_CH3_P03
#define ADC_CHANNEL_2 ADC2_CH4_P04
#define ADC_CHANNEL_M ADC2_CH5_P05
#define ADC_CHANNEL_3 ADC2_CH6_P06
#define ADC_CHANNEL_4 ADC2_CH7_P07

/*
 * ADC2 的 DMA 格式：每通道占用 2 * samples + 4 字节，最后两个字节为硬件平均值。
 * 0x0A 对应每通道 8 次转换；5 个通道一次扫描由 DMA 完成，CPU 不轮询转换完成位。
 */
#define BSP_INDUCTOR_DMA_CHANNEL_COUNT       (5U)
#define BSP_INDUCTOR_DMA_SAMPLES              (8U)
#define BSP_INDUCTOR_DMA_CFG2                 (0x0AU)
#define BSP_INDUCTOR_DMA_DATA_BYTES           ((BSP_INDUCTOR_DMA_SAMPLES * 2U) + 4U)
#define BSP_INDUCTOR_DMA_CHSW0                (0xF8U)     /* ADC2 CH3..CH7 */

enum
{
    BSP_INDUCTOR_DMA_CH1 = 0,
    BSP_INDUCTOR_DMA_CH2,
    BSP_INDUCTOR_DMA_CHM,
    BSP_INDUCTOR_DMA_CH3,
    BSP_INDUCTOR_DMA_CH4,
};

typedef struct
{
    inductor_rawdata_t raw;
    uint32 sequence;
    uint8 valid;
} bsp_inductor_snapshot_t;

static uint8 xdata inductor_dma_buffer[BSP_INDUCTOR_DMA_CHANNEL_COUNT][BSP_INDUCTOR_DMA_DATA_BYTES];
static bsp_inductor_snapshot_t xdata inductor_snapshot[2];
static volatile uint8 inductor_published_index = 0U;
static volatile uint32 inductor_sequence = 0UL;
static volatile uint8 inductor_dma_active = 0U;

static void bsp_inductor_dma_configure(void)
{
    EAXFR = 1;
    DMA_ADC2_STA = 0x00U;
    /* Fresh samples matter to TIM4, but control timers at level 3 still preempt this IRQ. */
    DMA_ADC2_CFG = 0x88U;      /* Completion interrupt, level 2. */
    DMA_ADC2_RXAH = (uint8)((uint16)&inductor_dma_buffer[0][0] >> 8);
    DMA_ADC2_RXAL = (uint8)((uint16)&inductor_dma_buffer[0][0]);
    DMA_ADC2_CFG2 = BSP_INDUCTOR_DMA_CFG2;
    DMA_ADC2_CHSW0 = BSP_INDUCTOR_DMA_CHSW0;
    DMA_ADC2_CHSW1 = 0x00U;
    DMA_ADC2_CR = 0x80U;
}

void bsp_inductor_init(void)
{
    uint8 channel;
    uint8 byte_index;
    uint8 slot;

    adc_init(ADC_CHANNEL_1, ADC_12BIT);
    adc_init(ADC_CHANNEL_2, ADC_12BIT);
    adc_init(ADC_CHANNEL_M, ADC_12BIT);
    adc_init(ADC_CHANNEL_3, ADC_12BIT);
    adc_init(ADC_CHANNEL_4, ADC_12BIT);

    for(slot = 0U; slot < 2U; slot++)
    {
        inductor_snapshot[slot].raw.Channel_1 = 0U;
        inductor_snapshot[slot].raw.Channel_2 = 0U;
        inductor_snapshot[slot].raw.Channel_M = 0U;
        inductor_snapshot[slot].raw.Channel_3 = 0U;
        inductor_snapshot[slot].raw.Channel_4 = 0U;
        inductor_snapshot[slot].sequence = 0UL;
        inductor_snapshot[slot].valid = 0U;
    }

    for(channel = 0U; channel < BSP_INDUCTOR_DMA_CHANNEL_COUNT; channel++)
    {
        for(byte_index = 0U; byte_index < BSP_INDUCTOR_DMA_DATA_BYTES; byte_index++)
        {
            inductor_dma_buffer[channel][byte_index] = 0U;
        }
    }

    inductor_dma_active = 0U;
    inductor_published_index = 0U;
    inductor_sequence = 0UL;
    bsp_inductor_dma_configure();
    (void)bsp_inductor_request_sample();
}

void bsp_inductor_debug(void)
{
}

void bsp_inductor_get(inductor_rawdata_t* out_data)
{
    (void)bsp_inductor_get_snapshot(out_data, NULL);
}

uint8 bsp_inductor_get_snapshot(inductor_rawdata_t *out_data, uint32 *sequence)
{
    uint8 ea_backup;
    uint8 index;
    uint8 valid;

    if(NULL == out_data)
    {
        return 0U;
    }

    ea_backup = EA;
    EA = 0;
    index = inductor_published_index;
    *out_data = inductor_snapshot[index].raw;
    valid = inductor_snapshot[index].valid;
    if(NULL != sequence)
    {
        *sequence = inductor_snapshot[index].sequence;
    }
    EA = ea_backup;
    return valid;
}

uint8 bsp_inductor_request_sample(void)
{
    uint8 ea_backup;
    uint8 started = 0U;

    ea_backup = EA;
    EA = 0;
    if(0U == inductor_dma_active)
    {
        EAXFR = 1;
        DMA_ADC2_STA = 0x00U;
        DMA_ADC2_CR = 0xC0U;  /* Enable + trigger one DMA scan. */
        inductor_dma_active = 1U;
        started = 1U;
    }
    EA = ea_backup;

    return started;
}

void bsp_inductor_dma_irq_handler(void)
{
    uint8 next_index;
    uint8 ea_backup;
    uint32 next_sequence;

    EAXFR = 1;
    if(0U == (DMA_ADC2_STA & 0x01U))
    {
        return;
    }

    DMA_ADC2_STA = 0x00U;
    /* Write only the inactive slot.  A high-priority TIM4 can preempt this
       low-priority DMA ISR, but it continues to read the old published slot. */
    next_index = (uint8)(inductor_published_index ^ 1U);
    inductor_snapshot[next_index].raw.Channel_1 =
            (uint16)(((uint16)inductor_dma_buffer[BSP_INDUCTOR_DMA_CH1][BSP_INDUCTOR_DMA_DATA_BYTES - 2U] << 8) |
            inductor_dma_buffer[BSP_INDUCTOR_DMA_CH1][BSP_INDUCTOR_DMA_DATA_BYTES - 1U]);
    inductor_snapshot[next_index].raw.Channel_2 =
            (uint16)(((uint16)inductor_dma_buffer[BSP_INDUCTOR_DMA_CH2][BSP_INDUCTOR_DMA_DATA_BYTES - 2U] << 8) |
            inductor_dma_buffer[BSP_INDUCTOR_DMA_CH2][BSP_INDUCTOR_DMA_DATA_BYTES - 1U]);
    inductor_snapshot[next_index].raw.Channel_M =
            (uint16)(((uint16)inductor_dma_buffer[BSP_INDUCTOR_DMA_CHM][BSP_INDUCTOR_DMA_DATA_BYTES - 2U] << 8) |
            inductor_dma_buffer[BSP_INDUCTOR_DMA_CHM][BSP_INDUCTOR_DMA_DATA_BYTES - 1U]);
    inductor_snapshot[next_index].raw.Channel_3 =
            (uint16)(((uint16)inductor_dma_buffer[BSP_INDUCTOR_DMA_CH3][BSP_INDUCTOR_DMA_DATA_BYTES - 2U] << 8) |
            inductor_dma_buffer[BSP_INDUCTOR_DMA_CH3][BSP_INDUCTOR_DMA_DATA_BYTES - 1U]);
    inductor_snapshot[next_index].raw.Channel_4 =
            (uint16)(((uint16)inductor_dma_buffer[BSP_INDUCTOR_DMA_CH4][BSP_INDUCTOR_DMA_DATA_BYTES - 2U] << 8) |
            inductor_dma_buffer[BSP_INDUCTOR_DMA_CH4][BSP_INDUCTOR_DMA_DATA_BYTES - 1U]);
    next_sequence = inductor_sequence + 1UL;
    inductor_snapshot[next_index].sequence = next_sequence;
    inductor_snapshot[next_index].valid = 1U;

    /* Publish index and sequence as one short unpreemptable commit. */
    ea_backup = EA;
    EA = 0;
    inductor_sequence = next_sequence;
    inductor_published_index = next_index;
    EA = ea_backup;
    inductor_dma_active = 0U;
}

uint8 bsp_inductor_sample_is_valid(void)
{
    uint8 ea_backup;
    uint8 valid;

    ea_backup = EA;
    EA = 0;
    valid = inductor_snapshot[inductor_published_index].valid;
    EA = ea_backup;
    return valid;
}

uint8 bsp_inductor_is_busy(void)
{
    return inductor_dma_active;
}

void bsp_inductor_recover(void)
{
    uint8 ea_backup;

    /* Recovery runs from background after a timeout.  It performs only
       register writes; it never waits for ADC/DMA completion. */
    ea_backup = EA;
    EA = 0;
    EAXFR = 1;
    DMA_ADC2_CR = 0x00U;
    DMA_ADC2_STA = 0x00U;
    inductor_dma_active = 0U;
    bsp_inductor_dma_configure();
    EA = ea_backup;
}
