#include "zf_common_headfile.h"
#include "bsp_battery.h"

#define BSP_BATTERY_ADC_CH              ADC1_CH0_P10
#define BSP_BATTERY_ADC_FULL_SCALE      (4096.0f)
#define BSP_BATTERY_VOLTAGE_SCALE       (36.4737f)

void bsp_battery_init(void)
{
    adc_init(BSP_BATTERY_ADC_CH, ADC_12BIT);
}

void bsp_battery_debug(void)
{
}

void bsp_battery_vol(float *vol)
{
    uint16 rawdatas;

    if(NULL == vol)
    {
        return;
    }

    rawdatas = adc_mean_filter_convert(BSP_BATTERY_ADC_CH, 20U);
    *vol = ((float)rawdatas / BSP_BATTERY_ADC_FULL_SCALE) * BSP_BATTERY_VOLTAGE_SCALE;
}
