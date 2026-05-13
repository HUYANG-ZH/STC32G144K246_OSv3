#include "zf_common_headfile.h"
#include "bsp_battery.h"

void bsp_battery_init(void)
{
    adc_init(ADC1_CH0_P10,ADC_12BIT);
}

void bsp_battery_vol(float *vol)
{
    uint16 rawdatas;
    rawdatas = adc_mean_filter_convert(ADC1_CH0_P10,20U);
    *vol = ((float)rawdatas / 4096.0f) * 33.0f;
}
