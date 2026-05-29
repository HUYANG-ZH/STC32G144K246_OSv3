#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_inductor.h"
#include "app_inductor_preprocess.h"

#define APP_INDUCTOR_CHANNEL_COUNT             (4U)
#define APP_INDUCTOR_MEDIAN_SAMPLE_COUNT       (3U)
#define APP_INDUCTOR_HISTORY_COUNT             (15U)
#define APP_INDUCTOR_AVERAGE_COUNT             (13U)

uint16 app_inductor_preprocess_min_value[APP_INDUCTOR_CHANNEL_COUNT] = {0U, 5U, 5U, 0U};
uint16 app_inductor_preprocess_max_value[APP_INDUCTOR_CHANNEL_COUNT] = {3586U, 1896U, 1851U, 3500U};

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
        sample[i][0] = raw.channel_1;
        sample[i][1] = raw.channel_2;
        sample[i][2] = raw.channel_3;
        sample[i][3] = raw.channel_4;
    }

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        median[i] = app_inductor_median3(sample[0][i], sample[1][i], sample[2][i]);
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
    float min_value;
    float max_value;

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

        if(app_inductor_preprocess_max_value[i] <= app_inductor_preprocess_min_value[i])
        {
            normalized[i] = 0.0f;
        }
        else
        {
            min_value = tfpu_int2float((long)app_inductor_preprocess_min_value[i]);
            max_value = tfpu_int2float((long)app_inductor_preprocess_max_value[i]);
            normalized[i] = tfpu_div(tfpu_mul(tfpu_sub(filtered[i], min_value), 100.0f),
                    tfpu_sub(max_value, min_value));
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
    app_inductor_update_output();

    pit_us_init(APP_INDUCTOR_PREPROCESS_PIT, APP_INDUCTOR_PREPROCESS_PERIOD_US, app_inductor_preprocess_tick);
}

void app_inductor_preprocess_debug(void)
{
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
