#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_timetick.h"
#include "service_inductor.h"
#include "service_wireless_uart.h"

#define SERVICE_INDUCTOR_DMA_TIMEOUT_TICK      (10000UL)
#define SERVICE_INDUCTOR_DEBUG_ENABLE          (0U)
#define SERVICE_INDUCTOR_DEBUG_PERIOD_TICK     (1000UL)  /* 10 Hz，0.1 ms/tick */

static uint32 service_inductor_last_sequence = 0UL;
static uint32 service_inductor_last_progress_tick = 0UL;
static uint8 service_inductor_seen_snapshot = 0U;

void service_inductor_init(void)
{
    bsp_inductor_init();
    service_inductor_last_sequence = 0UL;
    service_inductor_last_progress_tick = service_timetick_what();
    service_inductor_seen_snapshot = 0U;
}

void service_inductor_debug(void)
{
    static uint32 last_debug_tick = 0UL;
    uint32 now;
    service_inductor_data_t snapshot;

    now = service_timetick_what();
    if((uint32)(now - last_debug_tick) < SERVICE_INDUCTOR_DEBUG_PERIOD_TICK)
    {
        return;
    }
    last_debug_tick = now;

    if(0U != service_inductor_get_snapshot(&snapshot, NULL))
    {
        /* 电感调试模式：输出 CH1、CH2、CH3、CH4、M 五路原始 ADC 值。 */
        wprint("%u,%u,%u,%u,%u\r\n",
                snapshot.channel_1,
                snapshot.channel_2,
                snapshot.channel_3,
                snapshot.channel_4,
                snapshot.channel_m);
    }
}

void service_inductor_get_data(service_inductor_data_t *out_data)
{
    inductor_rawdata_t raw_data;

    if(NULL == out_data)
    {
        return;
    }

    raw_data.Channel_1 = 0U;
    raw_data.Channel_2 = 0U;
    raw_data.Channel_M = 0U;
    raw_data.Channel_3 = 0U;
    raw_data.Channel_4 = 0U;

    (void)bsp_inductor_get_snapshot(&raw_data, NULL);

    out_data->channel_1 = raw_data.Channel_1;
    out_data->channel_2 = raw_data.Channel_2;
    out_data->channel_m = raw_data.Channel_M;
    out_data->channel_3 = raw_data.Channel_3;
    out_data->channel_4 = raw_data.Channel_4;
}

uint8 service_inductor_get_snapshot(service_inductor_data_t *out_data, uint32 *sequence)
{
    inductor_rawdata_t raw_data;
    uint8 valid;

    if(NULL == out_data)
    {
        return 0U;
    }

    valid = bsp_inductor_get_snapshot(&raw_data, sequence);
    out_data->channel_1 = raw_data.Channel_1;
    out_data->channel_2 = raw_data.Channel_2;
    out_data->channel_m = raw_data.Channel_M;
    out_data->channel_3 = raw_data.Channel_3;
    out_data->channel_4 = raw_data.Channel_4;
    return valid;
}

uint8 service_inductor_request_sample(void)
{
    return bsp_inductor_request_sample();
}

uint8 service_inductor_sample_is_valid(void)
{
    return bsp_inductor_sample_is_valid();
}

uint8 service_inductor_is_busy(void)
{
    return bsp_inductor_is_busy();
}

void service_inductor_task(void)
{
    service_inductor_data_t snapshot;
    uint32 sequence;
    uint32 now;

    now = service_timetick_what();
    if(0U != service_inductor_get_snapshot(&snapshot, &sequence))
    {
        if((0U == service_inductor_seen_snapshot) ||
                (sequence != service_inductor_last_sequence))
        {
            service_inductor_last_sequence = sequence;
            service_inductor_last_progress_tick = now;
            service_inductor_seen_snapshot = 1U;
        }
    }

    /* A stalled DMA must not remain busy forever.  Recovery is background
       register work only; TIM4 independently latches the control safety fault. */
    if((0U != service_inductor_is_busy()) &&
            ((uint32)(now - service_inductor_last_progress_tick) >= SERVICE_INDUCTOR_DMA_TIMEOUT_TICK))
    {
        bsp_inductor_recover();
        service_inductor_last_progress_tick = now;
    }

#if (SERVICE_INDUCTOR_DEBUG_ENABLE != 0U)
    service_inductor_debug();
#endif
}
