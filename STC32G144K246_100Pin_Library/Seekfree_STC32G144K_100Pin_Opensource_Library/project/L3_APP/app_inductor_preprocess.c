#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_inductor.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "service_packet.h"
#include "app_inductor_preprocess.h"

#define APP_INDUCTOR_CHANNEL_COUNT             APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT
#define APP_INDUCTOR_MEDIAN_SAMPLE_COUNT       (3U)
#define APP_INDUCTOR_HISTORY_COUNT             (15U)
#define APP_INDUCTOR_AVERAGE_COUNT             (13U)
// [4]is middle inductor
uint16 app_inductor_preprocess_min_value[APP_INDUCTOR_CHANNEL_COUNT] = {850U, 1000U, 1000U, 750U, 100U};
uint16 app_inductor_preprocess_max_value[APP_INDUCTOR_CHANNEL_COUNT] = {4095U, 3000U, 3100U, 4095U, 4095U};

static float inductor_cal_min[APP_INDUCTOR_CHANNEL_COUNT];
static float inductor_cal_max[APP_INDUCTOR_CHANNEL_COUNT];

static void app_inductor_update_precomputed(void);

static void app_inductor_preprocess_sync_calibration(void)
{
    uint8 i;

    for(i = 0U; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        app_inductor_preprocess_min_value[i] = (uint16)inductor_cal_min[i];
        app_inductor_preprocess_max_value[i] = (uint16)inductor_cal_max[i];
    }
    app_inductor_update_precomputed();
}

static void app_inductor_preprocess_print_calibration(void)
{
    wprint("min=%d,%d,%d,%d,%d max=%d,%d,%d,%d,%d\r\n",
            app_inductor_preprocess_min_value[0],
            app_inductor_preprocess_min_value[1],
            app_inductor_preprocess_min_value[2],
            app_inductor_preprocess_min_value[3],
            app_inductor_preprocess_min_value[4],
            app_inductor_preprocess_max_value[0],
            app_inductor_preprocess_max_value[1],
            app_inductor_preprocess_max_value[2],
            app_inductor_preprocess_max_value[3],
            app_inductor_preprocess_max_value[4]);
}

static float inductor_min_float[APP_INDUCTOR_CHANNEL_COUNT];
static float inductor_max_float[APP_INDUCTOR_CHANNEL_COUNT];
static float inductor_range_inv[APP_INDUCTOR_CHANNEL_COUNT];

static uint16 inductor_history[APP_INDUCTOR_CHANNEL_COUNT][APP_INDUCTOR_HISTORY_COUNT];
static uint8 inductor_history_index = 0U;
static volatile app_inductor_preprocess_data_t inductor_data;

static void app_inductor_preprocess_tick(void);

static uint16 app_inductor_median3(uint16 a, uint16 b, uint16 c)
{
    uint16 temp;

    if(a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }
    if(b > c)
    {
        temp = b;
        b = c;
        c = temp;
    }
    if(a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }

    return b;
}

static void app_inductor_sort_ascending(uint16 *values)
{
    uint8 i;
    uint8 j;
    uint16 temp;

    for(i = 0U; i < (APP_INDUCTOR_HISTORY_COUNT - 1U); i++)
    {
        for(j = (uint8)(i + 1U); j < APP_INDUCTOR_HISTORY_COUNT; j++)
        {
            if(values[i] > values[j])
            {
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
}

static void app_inductor_sample_median(uint16 median[APP_INDUCTOR_CHANNEL_COUNT])
{
    uint8 i;
    service_inductor_data_t raw;
    uint16 sample[APP_INDUCTOR_MEDIAN_SAMPLE_COUNT][APP_INDUCTOR_CHANNEL_COUNT];

    for(i = 0; i < APP_INDUCTOR_MEDIAN_SAMPLE_COUNT; i++)
    {
        service_inductor_get_data(&raw);
        sample[i][APP_INDUCTOR_PREPROCESS_INDEX_CH1] = raw.channel_1;
        sample[i][APP_INDUCTOR_PREPROCESS_INDEX_CH2] = raw.channel_2;
        sample[i][APP_INDUCTOR_PREPROCESS_INDEX_CH3] = raw.channel_3;
        sample[i][APP_INDUCTOR_PREPROCESS_INDEX_CH4] = raw.channel_4;
        sample[i][APP_INDUCTOR_PREPROCESS_INDEX_M] = raw.channel_m;
    }

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        median[i] = app_inductor_median3(sample[0][i], sample[1][i], sample[2][i]);
    }
}

static void app_inductor_update_precomputed(void)
{
    uint8 i;
    uint16 range;

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_min_float[i] = tfpu_int2float((long)app_inductor_preprocess_min_value[i]);
        inductor_max_float[i] = tfpu_int2float((long)app_inductor_preprocess_max_value[i]);
        range = app_inductor_preprocess_max_value[i] - app_inductor_preprocess_min_value[i];
        if(0U < range)
        {
            inductor_range_inv[i] = tfpu_div(100.0f, tfpu_int2float((long)range));
        }
        else
        {
            inductor_range_inv[i] = 0.0f;
        }
    }
}

static void app_inductor_update_output(void)
{
    uint8 i;
    uint8 j;
    uint8 ea_backup;
    uint32 sum;
    uint16 sorted[APP_INDUCTOR_HISTORY_COUNT];
    float filtered[APP_INDUCTOR_CHANNEL_COUNT];
    float normalized[APP_INDUCTOR_CHANNEL_COUNT];

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        for(j = 0; j < APP_INDUCTOR_HISTORY_COUNT; j++)
        {
            sorted[j] = inductor_history[i][j];
        }

        app_inductor_sort_ascending(sorted);

        sum = 0U;
        for(j = 1U; j < (APP_INDUCTOR_HISTORY_COUNT - 1U); j++)
        {
            sum += sorted[j];
        }

        filtered[i] = tfpu_div(tfpu_int2float((long)sum), tfpu_int2float((long)APP_INDUCTOR_AVERAGE_COUNT));

        if(0.0f >= inductor_range_inv[i])
        {
            normalized[i] = 0.0f;
        }
        else
        {
            normalized[i] = tfpu_mul(tfpu_sub(filtered[i], inductor_min_float[i]), inductor_range_inv[i]);
            if(0.0f > normalized[i])
            {
                normalized[i] = 0.0f;
            }
            else if(100.0f < normalized[i])
            {
                normalized[i] = 100.0f;
            }
        }
    }

    ea_backup = EA;
    EA = 0;
    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_data.filtered[i] = filtered[i];
        inductor_data.normalized[i] = normalized[i];
    }
    EA = ea_backup;
}

static void app_inductor_preprocess_tick(void)
{
    uint8 i;
    uint16 median[APP_INDUCTOR_CHANNEL_COUNT];

    app_inductor_sample_median(median);

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_history[i][inductor_history_index] = median[i];
    }

    inductor_history_index++;
    if(APP_INDUCTOR_HISTORY_COUNT <= inductor_history_index)
    {
        inductor_history_index = 0U;
    }

    app_inductor_update_output();
}

void app_inductor_preprocess_update_calibration(void)
{
    app_inductor_update_precomputed();
}

void app_inductor_preprocess_init(void)
{
    uint8 i;
    uint8 j;
    uint16 median[APP_INDUCTOR_CHANNEL_COUNT];

    service_inductor_init();

    app_inductor_sample_median(median);

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        for(j = 0; j < APP_INDUCTOR_HISTORY_COUNT; j++)
        {
            inductor_history[i][j] = median[i];
        }
    }

    inductor_history_index = 0U;
    app_inductor_update_precomputed();
    app_inductor_update_output();

    /* 无线校准变量：float副本，修改后自动同步到uint16并重算 */
    for(i = 0U; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_cal_min[i] = (float)app_inductor_preprocess_min_value[i];
        inductor_cal_max[i] = (float)app_inductor_preprocess_max_value[i];
    }
    (void)service_packet_add_variable_with_callback("ind_min0", &inductor_cal_min[0], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_min1", &inductor_cal_min[1], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_min2", &inductor_cal_min[2], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_min3", &inductor_cal_min[3], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_minm", &inductor_cal_min[4], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_max0", &inductor_cal_max[0], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_max1", &inductor_cal_max[1], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_max2", &inductor_cal_max[2], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_max3", &inductor_cal_max[3], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_variable_with_callback("ind_maxm", &inductor_cal_max[4], 1U, app_inductor_preprocess_sync_calibration);
    (void)service_packet_add_action("ind_read", app_inductor_preprocess_print_calibration, 0UL);

    pit_us_init(APP_INDUCTOR_PREPROCESS_PIT, APP_INDUCTOR_PREPROCESS_PERIOD_US, app_inductor_preprocess_tick);
}

void app_inductor_preprocess_debug(void)
{
    static uint32 last_tick = 0U;
    app_inductor_preprocess_data_t inductor;

    if((service_timetick_what() - last_tick) >= 33U)  // 30Hz
    {
        last_tick = service_timetick_what();
        app_inductor_preprocess_get_data(&inductor);
        wprint("%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_M]);
    }
}

void app_inductor_preprocess_get_data(app_inductor_preprocess_data_t *out_data)
{
    uint8 i;
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        out_data->filtered[i] = inductor_data.filtered[i];
        out_data->normalized[i] = inductor_data.normalized[i];
    }
    EA = ea_backup;
}
