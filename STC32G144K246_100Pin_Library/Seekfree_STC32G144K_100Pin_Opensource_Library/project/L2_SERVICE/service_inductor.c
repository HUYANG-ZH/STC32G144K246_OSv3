#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_inductor.h"

void service_inductor_init(void)
{
    bsp_inductor_init();
}

void service_inductor_get_data(service_inductor_data_t *out_data)
{
    inductor_rawdata_t raw_data;

    if(NULL == out_data)
    {
        return;
    }

    bsp_inductor_get(&raw_data);

    out_data->channel_1 = raw_data.Channel_1;
    out_data->channel_2 = raw_data.Channel_2;
    out_data->channel_3 = raw_data.Channel_3;
    out_data->channel_4 = raw_data.Channel_4;
}
