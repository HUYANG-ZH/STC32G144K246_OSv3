#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_batterycheck.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "shared_lpf.h"

#define BATTERYCHECK_PERIOD_TICK        (1000UL)
#define BATTERYCHECK_DMA_TIMEOUT_TICK   (100UL)
/* 电压发布数据一阶低通系数: 0.1, 平滑采样噪声 */
#define BATTERYCHECK_LPF_ALPHA          (0.1f)

static volatile float batterycheck_voltage = 0.0f;
static uint32 batterycheck_last_request_tick = 0UL;
static uint32 batterycheck_last_sequence = 0UL;
static uint8 batterycheck_valid = 0U;
static shared_lpf_t batterycheck_voltage_lpf;
static volatile uint8 batterycheck_lpf_ready = 0U;

static void service_batterycheck_refresh_snapshot(void);
static void service_batterycheck_request_next(void);
static void service_batterycheck_voltage_reply(void);

static void service_batterycheck_refresh_snapshot(void)
{
    uint16 rawdata;
    uint32 sequence;
    float raw_voltage;
    float filtered;
    uint8 ea_backup;

    if(0U == bsp_battery_get_snapshot(&rawdata, &sequence))
    {
        return;
    }

    if((0U == batterycheck_valid) || (sequence != batterycheck_last_sequence))
    {
        /* 直接用本次取回的原始值换算, 避免二次读快照导致 LPF 双连击 */
        raw_voltage = bsp_battery_vol_from_raw(rawdata);
        batterycheck_last_sequence = sequence;
        batterycheck_valid = 1U;
        /* 发布数据 = 低通后的电压: 首帧直接落位, 之后按系数 0.1 平滑 */
        if(0U == batterycheck_lpf_ready)
        {
            shared_lpf_reset(&batterycheck_voltage_lpf, raw_voltage);
            batterycheck_lpf_ready = 1U;
            filtered = raw_voltage;
        }
        else
        {
            filtered = shared_lpf_update(&batterycheck_voltage_lpf, raw_voltage);
        }
        /* float 多字节存储必须对中断原子: TIM9 欠压守卫 10ms 会读该值,
           撕裂读会把半个旧值+半个新值拼成垃圾(曾导致开机恒定 -0.16V 误报警) */
        ea_backup = EA;
        EA = 0;
        batterycheck_voltage = filtered;
        EA = ea_backup;
    }
}

static void service_batterycheck_request_next(void)
{
    if(0U != bsp_battery_request_sample())
    {
        batterycheck_last_request_tick = service_timetick_what();
    }
}

static void service_batterycheck_voltage_reply(void)
{
    service_batterycheck_refresh_snapshot();
    service_batterycheck_request_next();
    if(0U != batterycheck_valid)
    {
        wprint("battery_voltage,%.3f\r\n", batterycheck_voltage);
    }
    else
    {
        wprint("battery_voltage,pending\r\n");
    }
}

void service_batterycheck_init(void)
{
    bsp_battery_init();
    batterycheck_voltage = 0.0f;
    batterycheck_last_request_tick = service_timetick_what();
    batterycheck_last_sequence = 0UL;
    batterycheck_valid = 0U;
    shared_lpf_init(&batterycheck_voltage_lpf, BATTERYCHECK_LPF_ALPHA, 0.0f);
    batterycheck_lpf_ready = 0U;
    (void)service_packet_add_action("battery_voltage", service_batterycheck_voltage_reply, 0UL);
    #if __DBGFLAG__
    printf(">>[service_batterycheck_init]\r\n");
    wprint(">>[service_batterycheck_init]\r\n");
    #endif
}

void service_batterycheck_debug(void)
{
    service_batterycheck_refresh_snapshot();
    if(0U != batterycheck_valid)
    {
        printf("[batterycheck:voltage=%.4f]\r\n", batterycheck_voltage);
    }
    else
    {
        printf("[batterycheck:pending]\r\n");
    }
}

void service_batterycheck_task(void)
{
    uint32 now;

    service_batterycheck_refresh_snapshot();
    now = service_timetick_what();
    if(0U != bsp_battery_is_busy())
    {
        if((uint32)(now - batterycheck_last_request_tick) >= BATTERYCHECK_DMA_TIMEOUT_TICK)
        {
            bsp_battery_recover();
            service_batterycheck_request_next();
        }
        return;
    }

    if((uint32)(now - batterycheck_last_request_tick) >= BATTERYCHECK_PERIOD_TICK)
    {
        service_batterycheck_request_next();
    }
}

uint8 service_batterycheck_is_valid(void)
{
    service_batterycheck_refresh_snapshot();
    return batterycheck_valid;
}

uint8 service_batterycheck_raw_is_valid(void)
{
    return bsp_battery_sample_is_valid();
}

void service_batterycheck_get_raw(uint16 *rawdata)
{
    bsp_battery_get_raw(rawdata);
}

uint8 service_batterycheck_get_raw_snapshot(uint16 *rawdata, uint32 *sequence)
{
    return bsp_battery_get_snapshot(rawdata, sequence);
}

void service_batterycheck_get_voltage(float *voltage)
{
    service_batterycheck_refresh_snapshot();
    if(NULL != voltage)
    {
        *voltage = batterycheck_voltage;
    }
}

/* ISR 安全读取: 只返回已发布的滤波后电压, 不触发刷新/低通计算, 撕裂读保护 */
float service_batterycheck_get_filtered_voltage(void)
{
    float voltage;

    voltage = batterycheck_voltage;
    while(voltage != batterycheck_voltage)
    {
        voltage = batterycheck_voltage;
    }
    return voltage;
}

/* 滤波已就绪标记: 1 = 主循环至少发布过一次滤波电压(首帧), 0 = 尚未(初值 0.0 不可用于判定) */
uint8 service_batterycheck_filter_ready(void)
{
    return batterycheck_lpf_ready;
}
