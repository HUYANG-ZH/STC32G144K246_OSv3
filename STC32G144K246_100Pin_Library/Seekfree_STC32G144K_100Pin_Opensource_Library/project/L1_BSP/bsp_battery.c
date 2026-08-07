#include "zf_common_headfile.h"
#include "bsp_battery.h"

#define BSP_BATTERY_ADC_CH              ADC1_CH0_P10
#define BSP_BATTERY_ADC_FULL_SCALE      (4096.0f)
#define BSP_BATTERY_VOLTAGE_SCALE       (36.4737f)

/* DMA ADC stores N 16-bit raw samples plus a final 16-bit hardware average. */
#define BSP_BATTERY_DMA_SAMPLES          (8U)
#define BSP_BATTERY_DMA_CFG2             (0x0AU)
#define BSP_BATTERY_DMA_DATA_BYTES       ((BSP_BATTERY_DMA_SAMPLES * 2U) + 4U)
#define BSP_BATTERY_DMA_CHSW0            (0x01U)

typedef struct
{
    uint16 raw;
    uint32 sequence;
    uint8 valid;
} bsp_battery_snapshot_t;

static uint8 xdata battery_dma_buffer[BSP_BATTERY_DMA_DATA_BYTES];
static bsp_battery_snapshot_t xdata battery_snapshot[2];
static volatile uint8 battery_published_index = 0U;
static volatile uint32 battery_sequence = 0UL;
static volatile uint8 battery_dma_active = 0U;

static void bsp_battery_dma_configure(void)
{
    EAXFR = 1;
    DMA_ADC_STA = 0x00U;
    DMA_ADC_CFG = 0x80U;       /* Completion interrupt, lowest priority. */
    DMA_ADC_RXAH = (uint8)((uint16)&battery_dma_buffer[0] >> 8);
    DMA_ADC_RXAL = (uint8)((uint16)&battery_dma_buffer[0]);
    DMA_ADC_CFG2 = BSP_BATTERY_DMA_CFG2;
    DMA_ADC_CHSW0 = BSP_BATTERY_DMA_CHSW0;
    DMA_ADC_CHSW1 = 0x00U;
    DMA_ADC_CR = 0x80U;
}

void bsp_battery_init(void)
{
    uint8 index;

    adc_init(BSP_BATTERY_ADC_CH, ADC_12BIT);
    for(index = 0U; index < BSP_BATTERY_DMA_DATA_BYTES; index++)
    {
        battery_dma_buffer[index] = 0U;
    }

    for(index = 0U; index < 2U; index++)
    {
        battery_snapshot[index].raw = 0U;
        battery_snapshot[index].sequence = 0UL;
        battery_snapshot[index].valid = 0U;
    }

    battery_published_index = 0U;
    battery_sequence = 0UL;
    battery_dma_active = 0U;
    bsp_battery_dma_configure();
    (void)bsp_battery_request_sample();
}

void bsp_battery_debug(void)
{
}

uint8 bsp_battery_request_sample(void)
{
    uint8 ea_backup;
    uint8 started = 0U;

    ea_backup = EA;
    EA = 0;
    if(0U == battery_dma_active)
    {
        EAXFR = 1;
        DMA_ADC_STA = 0x00U;
        DMA_ADC_CR = 0xC0U;     /* Enable + trigger one ADC1 DMA scan. */
        battery_dma_active = 1U;
        started = 1U;
    }
    EA = ea_backup;

    return started;
}

void bsp_battery_dma_irq_handler(void)
{
    uint8 next_index;
    uint8 ea_backup;
    uint32 next_sequence;

    EAXFR = 1;
    if(0U == (DMA_ADC_STA & 0x01U))
    {
        return;
    }

    DMA_ADC_STA = 0x00U;
    /* The active slot is immutable.  Publish the completed ADC frame only
       after the inactive slot has all data and metadata. */
    next_index = (uint8)(battery_published_index ^ 1U);
    battery_snapshot[next_index].raw =
            (uint16)(((uint16)battery_dma_buffer[BSP_BATTERY_DMA_DATA_BYTES - 2U] << 8) |
            battery_dma_buffer[BSP_BATTERY_DMA_DATA_BYTES - 1U]);
    next_sequence = battery_sequence + 1UL;
    battery_snapshot[next_index].sequence = next_sequence;
    battery_snapshot[next_index].valid = 1U;

    ea_backup = EA;
    EA = 0;
    battery_sequence = next_sequence;
    battery_published_index = next_index;
    EA = ea_backup;
    battery_dma_active = 0U;
}

uint8 bsp_battery_sample_is_valid(void)
{
    uint8 ea_backup;
    uint8 valid;

    ea_backup = EA;
    EA = 0;
    valid = battery_snapshot[battery_published_index].valid;
    EA = ea_backup;
    return valid;
}

uint32 bsp_battery_sample_sequence(void)
{
    uint32 sequence;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    sequence = battery_snapshot[battery_published_index].sequence;
    EA = ea_backup;
    return sequence;
}

void bsp_battery_get_raw(uint16 *rawdata)
{
    if(NULL == rawdata)
    {
        return;
    }

    *rawdata = 0U;
    (void)bsp_battery_get_snapshot(rawdata, NULL);
}

uint8 bsp_battery_get_snapshot(uint16 *rawdata, uint32 *sequence)
{
    uint8 ea_backup;
    uint8 index;
    uint8 valid;

    if(NULL == rawdata)
    {
        return 0U;
    }

    ea_backup = EA;
    EA = 0;
    index = battery_published_index;
    *rawdata = battery_snapshot[index].raw;
    valid = battery_snapshot[index].valid;
    if(NULL != sequence)
    {
        *sequence = battery_snapshot[index].sequence;
    }
    EA = ea_backup;
    return valid;
}

uint8 bsp_battery_is_busy(void)
{
    return battery_dma_active;
}

void bsp_battery_recover(void)
{
    uint8 ea_backup;

    /* Background recovery only resets/re-arms peripheral registers. */
    ea_backup = EA;
    EA = 0;
    EAXFR = 1;
    DMA_ADC_CR = 0x00U;
    DMA_ADC_STA = 0x00U;
    battery_dma_active = 0U;
    bsp_battery_dma_configure();
    EA = ea_backup;
}

void bsp_battery_vol(float *vol)
{
    uint16 rawdata = 0U;

    if(NULL == vol)
    {
        return;
    }

    bsp_battery_get_raw(&rawdata);
    *vol = ((float)rawdata / BSP_BATTERY_ADC_FULL_SCALE) * BSP_BATTERY_VOLTAGE_SCALE;
}

float bsp_battery_vol_from_raw(uint16 rawdata)
{
    return ((float)rawdata / BSP_BATTERY_ADC_FULL_SCALE) * BSP_BATTERY_VOLTAGE_SCALE;
}
