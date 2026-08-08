#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_inductor.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "app_speedout.h"
#include "app_inductor_preprocess.h"

#define APP_INDUCTOR_CHANNEL_COUNT             APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT
#define APP_INDUCTOR_HISTORY_COUNT             (7U)
#define APP_INDUCTOR_AVERAGE_COUNT             (5U)
#define APP_INDUCTOR_AVERAGE_INV               (0.2f)
#define APP_INDUCTOR_STARTUP_GRACE_TICK         (1000UL)
#define APP_INDUCTOR_MAX_SAMPLE_AGE_TICK        (500UL)
// [4]is middle inductor
uint16 app_inductor_preprocess_min_value[APP_INDUCTOR_CHANNEL_COUNT] = {25U, 50U, 50U, 25U, 100U};
uint16 app_inductor_preprocess_max_value[APP_INDUCTOR_CHANNEL_COUNT] = {2560U, 2100U, 2100U, 2560U, 4095U};

static void app_inductor_update_precomputed(void);

static float inductor_min_float[APP_INDUCTOR_CHANNEL_COUNT];
static float inductor_max_float[APP_INDUCTOR_CHANNEL_COUNT];
static float inductor_range_inv[APP_INDUCTOR_CHANNEL_COUNT];

static uint16 inductor_history[APP_INDUCTOR_CHANNEL_COUNT][APP_INDUCTOR_HISTORY_COUNT];
static uint8 inductor_history_index = 0U;
static uint8 inductor_history_valid = 0U;
static volatile app_inductor_preprocess_data_t inductor_data;
static uint32 inductor_last_sequence = 0UL;
static uint32 inductor_last_fresh_tick = 0UL;
static uint32 inductor_start_tick = 0UL;
static uint8 inductor_sensor_seen = 0U;
static uint8 inductor_sensor_fault = 0U;

static void app_inductor_preprocess_tick(void);

static uint8 app_inductor_sample(uint16 sample[APP_INDUCTOR_CHANNEL_COUNT], uint32 *sequence)
{
    service_inductor_data_t raw;

    if(0U == service_inductor_get_snapshot(&raw, sequence))
    {
        return 0U;
    }

    sample[APP_INDUCTOR_PREPROCESS_INDEX_CH1] = raw.channel_1;
    sample[APP_INDUCTOR_PREPROCESS_INDEX_CH2] = raw.channel_2;
    sample[APP_INDUCTOR_PREPROCESS_INDEX_CH3] = raw.channel_3;
    sample[APP_INDUCTOR_PREPROCESS_INDEX_CH4] = raw.channel_4;
    sample[APP_INDUCTOR_PREPROCESS_INDEX_M] = raw.channel_m;
    return 1U;
}

static void app_inductor_update_precomputed(void)
{
    uint8 i;
    uint8 ea_backup;
    uint16 range;
    float next_min[APP_INDUCTOR_CHANNEL_COUNT];
    float next_max[APP_INDUCTOR_CHANNEL_COUNT];
    float next_range_inv[APP_INDUCTOR_CHANNEL_COUNT];

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        next_min[i] = tfpu_int2float((long)app_inductor_preprocess_min_value[i]);
        next_max[i] = tfpu_int2float((long)app_inductor_preprocess_max_value[i]);
        if(app_inductor_preprocess_max_value[i] > app_inductor_preprocess_min_value[i])
        {
            range = app_inductor_preprocess_max_value[i] - app_inductor_preprocess_min_value[i];
            next_range_inv[i] = tfpu_div(100.0f, tfpu_int2float((long)range));
        }
        else
        {
            next_range_inv[i] = 0.0f;
        }
    }

    /* TIM4 only observes fully published calibration triples. */
    ea_backup = EA;
    EA = 0;
    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_min_float[i] = next_min[i];
        inductor_max_float[i] = next_max[i];
        inductor_range_inv[i] = next_range_inv[i];
    }
    EA = ea_backup;
}

static void app_inductor_update_output(void)
{
    uint8 i;
    uint8 j;
    uint8 ea_backup;
    uint32 sum;
    uint16 min_val;
    uint16 max_val;
    uint16 values[APP_INDUCTOR_HISTORY_COUNT];
    float filtered[APP_INDUCTOR_CHANNEL_COUNT];
    float normalized[APP_INDUCTOR_CHANNEL_COUNT];

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        for(j = 0; j < APP_INDUCTOR_HISTORY_COUNT; j++)
        {
            values[j] = inductor_history[i][j];
        }

        min_val = values[0];
        max_val = values[0];
        sum = values[0];
        for(j = 1U; j < APP_INDUCTOR_HISTORY_COUNT; j++)
        {
            sum += values[j];
            if(values[j] < min_val)
            {
                min_val = values[j];
            }
            if(values[j] > max_val)
            {
                max_val = values[j];
            }
        }

        filtered[i] = tfpu_mul(tfpu_int2float((long)(sum - min_val - max_val)),
                APP_INDUCTOR_AVERAGE_INV);

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
    uint8 j;
    uint16 sample[APP_INDUCTOR_CHANNEL_COUNT];
    uint32 sequence;
    uint32 now;

    now = service_timetick_what();
    if(0U == app_inductor_sample(sample, &sequence))
    {
        if((uint32)(now - inductor_start_tick) >= APP_INDUCTOR_STARTUP_GRACE_TICK)
        {
            if(0U == inductor_sensor_fault)
            {
                inductor_sensor_fault = 1U;
                app_speedout_set_safety_inhibit(APP_SPEEDOUT_SAFETY_INDUCTOR);
            }
        }
        (void)service_inductor_request_sample();
        return;
    }

    if((0U != inductor_sensor_seen) && (sequence == inductor_last_sequence))
    {
        if((uint32)(now - inductor_last_fresh_tick) >= APP_INDUCTOR_MAX_SAMPLE_AGE_TICK)
        {
            if(0U == inductor_sensor_fault)
            {
                inductor_sensor_fault = 1U;
                app_speedout_set_safety_inhibit(APP_SPEEDOUT_SAFETY_INDUCTOR);
            }
        }
        (void)service_inductor_request_sample();
        return;
    }

    inductor_last_sequence = sequence;
    inductor_last_fresh_tick = now;
    inductor_sensor_seen = 1U;
    if(0U != inductor_sensor_fault)
    {
        inductor_sensor_fault = 0U;
        app_speedout_clear_safety_inhibit(APP_SPEEDOUT_SAFETY_INDUCTOR);
    }

    if(0U == inductor_history_valid)
    {
        for(i = 0U; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
        {
            for(j = 0U; j < APP_INDUCTOR_HISTORY_COUNT; j++)
            {
                inductor_history[i][j] = sample[i];
            }
        }
        inductor_history_index = 0U;
        inductor_history_valid = 1U;
    }
    else
    {
        for(i = 0U; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
        {
            inductor_history[i][inductor_history_index] = sample[i];
        }

        inductor_history_index++;
        if(APP_INDUCTOR_HISTORY_COUNT <= inductor_history_index)
        {
            inductor_history_index = 0U;
        }
    }

    app_inductor_update_output();
    (void)service_inductor_request_sample();
}

void app_inductor_preprocess_init(void)
{
    uint8 i;
    uint8 j;
    uint16 sample[APP_INDUCTOR_CHANNEL_COUNT];

    service_inductor_init();

    for(i = 0U; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        sample[i] = 0U;
    }
    (void)app_inductor_sample(sample, NULL);

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        for(j = 0; j < APP_INDUCTOR_HISTORY_COUNT; j++)
        {
            inductor_history[i][j] = sample[i];
        }
    }

    inductor_history_index = 0U;
    inductor_history_valid = 0U;
    inductor_last_sequence = 0UL;
    inductor_last_fresh_tick = service_timetick_what();
    inductor_start_tick = inductor_last_fresh_tick;
    inductor_sensor_seen = 0U;
    inductor_sensor_fault = 0U;
    app_inductor_update_precomputed();
    app_inductor_update_output();

    pit_us_init(APP_INDUCTOR_PREPROCESS_PIT, APP_INDUCTOR_PREPROCESS_PERIOD_US, app_inductor_preprocess_tick);
    interrupt_set_priority(TIM4_IRQn, 3U);
    #if __DBGFLAG__
    printf(">>[app_inductor_preprocess_init]\r\n");
    wprint(">>[app_inductor_preprocess_init]\r\n");
    #endif
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
