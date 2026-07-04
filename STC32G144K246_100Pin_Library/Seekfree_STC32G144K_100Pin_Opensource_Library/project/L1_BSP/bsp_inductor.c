#include "zf_common_headfile.h"
#include "bsp_inductor.h"

#define ADC_CHANNEL_1 ADC2_CH3_P03
#define ADC_CHANNEL_2 ADC2_CH4_P04
#define ADC_CHANNEL_M ADC2_CH5_P05
#define ADC_CHANNEL_3 ADC2_CH6_P06
#define ADC_CHANNEL_4 ADC2_CH7_P07

void bsp_inductor_init(void)
{
    adc_init(ADC_CHANNEL_1, ADC_12BIT);
    adc_init(ADC_CHANNEL_2, ADC_12BIT);
    adc_init(ADC_CHANNEL_M, ADC_12BIT);
    adc_init(ADC_CHANNEL_3, ADC_12BIT);
    adc_init(ADC_CHANNEL_4, ADC_12BIT);
}

void bsp_inductor_debug(void)
{
}

void bsp_inductor_get(inductor_rawdata_t* out_data)
{
    out_data->Channel_1 = adc_convert(ADC_CHANNEL_1);
    out_data->Channel_2 = adc_convert(ADC_CHANNEL_2);
    out_data->Channel_M = adc_convert(ADC_CHANNEL_M);
    out_data->Channel_3 = adc_convert(ADC_CHANNEL_3);
    out_data->Channel_4 = adc_convert(ADC_CHANNEL_4);
}
