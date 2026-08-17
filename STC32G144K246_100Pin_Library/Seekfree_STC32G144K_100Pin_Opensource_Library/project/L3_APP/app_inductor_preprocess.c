#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_inductor.h"
#include "service_timetick.h"
#include "service_packet.h"
#include "service_wireless_uart.h"
#include "app_speedout.h"
#include "app_inductor_preprocess.h"

#define APP_INDUCTOR_CHANNEL_COUNT             APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT
#define APP_INDUCTOR_HISTORY_COUNT             (7U)
#define APP_INDUCTOR_STARTUP_GRACE_TICK         (1000UL)
#define APP_INDUCTOR_MAX_SAMPLE_AGE_TICK        (500UL)
/* 电感归一化(Q16 定点, 零浮点), 按通道分两种模式:
   锚点模式(CH1/CH4 左右横电感): 单锚点三点分段 min->0, mid->X, max->100
     锚点输出 X 为全局无线可调变量 inductor_anchor_out(默认 50), 两通道共用同一 X,
     保证 CH1/CH4 在锚点(mid)处归一化值严格相等(车居中输出恒等, 左右绝对对称)
   单段模式(CH2/CH3 前后纵电感): 保持原 min-max 线性归一化 (v-min)*100/(max-min)
   斜率 k = ceil(Δout<<16/区间宽), uint32 存; 区间内 Δv<=w => 乘积<=100<<16<2^32 无溢出 */
#define APP_INDUCTOR_SLOPE_Q16_SHIFT          (16U)
#define APP_INDUCTOR_NORM_UPPER_LIMIT_U16      (300U)
#define APP_INDUCTOR_ANCHOR_OUT_DEFAULT        (68.0f)
/* 锚点模式通道判定: 仅 CH1/CH4 */
#define APP_INDUCTOR_IS_ANCHOR_CHANNEL(i)      ((APP_INDUCTOR_PREPROCESS_INDEX_CH1 == (i)) || \
        (APP_INDUCTOR_PREPROCESS_INDEX_CH4 == (i)))
// car3 已彻底弃用 M 通道电感, 仅标定 CH1~CH4 四路
// 锚点模式(CH1/CH4)实测标定: min/mid(车严格居中实测)/max
//   CH1: min=714 mid=1831 max=2265; CH4: min=853 mid=1720 max=2100
// 单段模式(CH2/CH3)仅用 min/max: CH2 1007/3125, CH3 1175/3190; mid 占位值不参与计算
uint16 app_inductor_preprocess_min_value[APP_INDUCTOR_CHANNEL_COUNT] = {714U, 1007U, 1175U, 853U};
uint16 app_inductor_preprocess_mid_value[APP_INDUCTOR_CHANNEL_COUNT] = {1831U, 1935U, 2070U, 1720U};
uint16 app_inductor_preprocess_max_value[APP_INDUCTOR_CHANNEL_COUNT] = {2265U, 3125U, 3190U, 2100U};

static void app_inductor_update_precomputed(void);

/* 锚点输出 X(全局共用, 无线可调): 锚点模式通道在 mid 处的归一化值, 默认 68
   依据: car3 标定(CH1 714/1831/2265, CH4 853/1720/2100), 两段斜率均衡 X=69.5~72 折中取 68:
   CH1 下段0.061/上段0.074, CH4 下段0.078/上段0.084 均较均衡; 实车可无线微调 66~72 */
static volatile float inductor_anchor_out = APP_INDUCTOR_ANCHOR_OUT_DEFAULT;
static float inductor_anchor_out_last = APP_INDUCTOR_ANCHOR_OUT_DEFAULT;
static uint16 inductor_mid_value[APP_INDUCTOR_CHANNEL_COUNT];
static uint32 inductor_slope_low_q16[APP_INDUCTOR_CHANNEL_COUNT];
static uint32 inductor_slope_high_q16[APP_INDUCTOR_CHANNEL_COUNT];
static uint32 inductor_slope_full_q16[APP_INDUCTOR_CHANNEL_COUNT];   /* 单段模式斜率 */
static uint16 inductor_anchor_u16;              /* 与斜率同批发布的锚点输出 X */

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
    return 1U;
}

static void app_inductor_update_precomputed(void)
{
    uint8 i;
    uint8 ea_backup;
    uint16 low_width;
    uint16 high_width;
    uint16 full_width;
    uint16 anchor_out_u16;
    uint16 next_mid[APP_INDUCTOR_CHANNEL_COUNT];
    uint32 next_slope_low[APP_INDUCTOR_CHANNEL_COUNT];
    uint32 next_slope_high[APP_INDUCTOR_CHANNEL_COUNT];
    uint32 next_slope_full[APP_INDUCTOR_CHANNEL_COUNT];

    /* 锚点输出 X 截断到 [0,100] */
    if(0.0f >= inductor_anchor_out)
    {
        anchor_out_u16 = 0U;
    }
    else if(100.0f <= inductor_anchor_out)
    {
        anchor_out_u16 = 100U;
    }
    else
    {
        anchor_out_u16 = (uint16)inductor_anchor_out;
    }

    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        next_mid[i] = app_inductor_preprocess_mid_value[i];
        if(0U != APP_INDUCTOR_IS_ANCHOR_CHANNEL(i))
        {
            /* 锚点模式(CH1/CH4): 下段 [min, mid] -> [0, X] */
            if(app_inductor_preprocess_mid_value[i] > app_inductor_preprocess_min_value[i])
            {
                low_width = (uint16)(app_inductor_preprocess_mid_value[i] -
                        app_inductor_preprocess_min_value[i]);
                next_slope_low[i] = (((uint32)anchor_out_u16 << APP_INDUCTOR_SLOPE_Q16_SHIFT) +
                        low_width - 1UL) / low_width;
            }
            else
            {
                next_slope_low[i] = 0UL;
            }
            /* 锚点模式: 上段 [mid, max] -> [X, 100] */
            if(app_inductor_preprocess_max_value[i] > app_inductor_preprocess_mid_value[i])
            {
                high_width = (uint16)(app_inductor_preprocess_max_value[i] -
                        app_inductor_preprocess_mid_value[i]);
                next_slope_high[i] = (((uint32)(100U - anchor_out_u16) << APP_INDUCTOR_SLOPE_Q16_SHIFT) +
                        high_width - 1UL) / high_width;
            }
            else
            {
                next_slope_high[i] = 0UL;
            }
            next_slope_full[i] = 0UL;
        }
        else
        {
            /* 单段模式(CH2/CH3): [min, max] -> [0, 100], 与原 min-max 归一化一致 */
            next_slope_low[i] = 0UL;
            next_slope_high[i] = 0UL;
            if(app_inductor_preprocess_max_value[i] > app_inductor_preprocess_min_value[i])
            {
                full_width = (uint16)(app_inductor_preprocess_max_value[i] -
                        app_inductor_preprocess_min_value[i]);
                next_slope_full[i] = ((100UL << APP_INDUCTOR_SLOPE_Q16_SHIFT) +
                        full_width - 1UL) / full_width;
            }
            else
            {
                next_slope_full[i] = 0UL;
            }
        }
    }

    /* TIM4 only observes fully published calibration triples. */
    ea_backup = EA;
    EA = 0;
    inductor_anchor_u16 = anchor_out_u16;
    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_mid_value[i] = next_mid[i];
        inductor_slope_low_q16[i] = next_slope_low[i];
        inductor_slope_high_q16[i] = next_slope_high[i];
        inductor_slope_full_q16[i] = next_slope_full[i];
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
    uint16 filtered[APP_INDUCTOR_CHANNEL_COUNT];
    uint16 normalized[APP_INDUCTOR_CHANNEL_COUNT];

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

        /* 整数中值滤波: 去极值后 5 帧平均(四舍五入), 零浮点 */
        filtered[i] = (uint16)((sum - min_val - max_val + 2UL) / 5UL);

        if(0U != APP_INDUCTOR_IS_ANCHOR_CHANNEL(i))
        {
            /* 锚点模式(CH1/CH4): 分段映射(零浮点定点 Q16) min->0, mid->X, max->100, 上段保留 300 外推
               X=inductor_anchor_u16 为全局共用锚点输出, 两通道 mid 处输出严格相等 */
            if(filtered[i] <= inductor_mid_value[i])
            {
                if((0UL == inductor_slope_low_q16[i]) ||
                        (filtered[i] <= app_inductor_preprocess_min_value[i]))
                {
                    normalized[i] = 0U;
                }
                else
                {
                    normalized[i] = (uint16)(((uint32)(filtered[i] - app_inductor_preprocess_min_value[i]) *
                            inductor_slope_low_q16[i]) >> APP_INDUCTOR_SLOPE_Q16_SHIFT);
                }
            }
            else
            {
                if(0UL == inductor_slope_high_q16[i])
                {
                    normalized[i] = inductor_anchor_u16;
                }
                else
                {
                    normalized[i] = (uint16)(inductor_anchor_u16 +
                            (((uint32)(filtered[i] - inductor_mid_value[i]) *
                            inductor_slope_high_q16[i]) >> APP_INDUCTOR_SLOPE_Q16_SHIFT));
                    if(APP_INDUCTOR_NORM_UPPER_LIMIT_U16 < normalized[i])
                    {
                        normalized[i] = APP_INDUCTOR_NORM_UPPER_LIMIT_U16;
                    }
                }
            }
        }
        else
        {
            /* 单段模式(CH2/CH3): 原 min-max 线性归一化 (v-min)*100/(max-min), 钳 0/300 */
            if((0UL == inductor_slope_full_q16[i]) ||
                    (filtered[i] <= app_inductor_preprocess_min_value[i]))
            {
                normalized[i] = 0U;
            }
            else
            {
                normalized[i] = (uint16)(((uint32)(filtered[i] - app_inductor_preprocess_min_value[i]) *
                        inductor_slope_full_q16[i]) >> APP_INDUCTOR_SLOPE_Q16_SHIFT);
                if(APP_INDUCTOR_NORM_UPPER_LIMIT_U16 < normalized[i])
                {
                    normalized[i] = APP_INDUCTOR_NORM_UPPER_LIMIT_U16;
                }
            }
        }
    }

    ea_backup = EA;
    EA = 0;
    for(i = 0; i < APP_INDUCTOR_CHANNEL_COUNT; i++)
    {
        inductor_data.filtered[i] = tfpu_int2float((long)filtered[i]);
        inductor_data.normalized[i] = tfpu_int2float((long)normalized[i]);
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

    /* 无线锚点输出 X 变化时重算两段斜率(仅变化瞬间执行, 8次32位除法约数十us) */
    if(inductor_anchor_out != inductor_anchor_out_last)
    {
        inductor_anchor_out_last = inductor_anchor_out;
        app_inductor_update_precomputed();
    }

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
    (void)service_packet_add_variable("anchor_out",
            (float *)&inductor_anchor_out, 1U);

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
        wprint("%.1f,%.1f,%.1f,%.1f\r\n",
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3],
                inductor.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4]);
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
