#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "service_inductor.h"
#include "service_batterycheck.h"
#include "app_attitude.h"
#include "debugw.h"

/* 调试模式(无线 debuginfo):
   0 = 关闭
   1 = IMU 三轴姿态 roll/pitch/yaw (deg), 50Hz
   2 = 4路电感原始值 CH1~CH4, 10Hz
   3 = 电池电压 (V), 10Hz */
#define DEBUGW_IMU_PERIOD_TICK          (200UL)    /* 50Hz, 0.1ms/tick */
#define DEBUGW_INDUCTOR_PERIOD_TICK     (1000UL)   /* 10Hz */
#define DEBUGW_VOLTAGE_PERIOD_TICK      (1000UL)   /* 10Hz */

static volatile float debugw_debug_info = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无线调试模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     debugw_init();
// 备注信息     注册 debuginfo 无线变量(0/1/2/3 模式选择), 由主循环调用 debugw_task() 周期输出
//-------------------------------------------------------------------------------------------------------------------
void debugw_init(void)
{
    debugw_debug_info = 0.0f;
    (void)service_packet_add_variable("debuginfo", (float *)&debugw_debug_info, 1U);
    #if __DBGFLAG__
    printf(">>[debugw_init]\r\n");
    wprint(">>[debugw_init]\r\n");
    #endif
}

static uint8 debugw_mode(void)
{
    uint8 mode;

    mode = (uint8)(debugw_debug_info + 0.5f);
    if(mode > 3U)
    {
        mode = 0U;   /* 未知模式视为关闭 */
    }
    return mode;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无线调试任务
// 参数说明     void
// 返回参数     void
// 使用示例     debugw_task();
// 备注信息     由主循环调用; 按 debuginfo 模式输出对应数据, vofa+ FireWater 最简格式
//-------------------------------------------------------------------------------------------------------------------
void debugw_task(void)
{
    static uint32 last_tick = 0UL;
    uint8 mode;
    uint32 period_tick;
    uint32 now;
    app_attitude_data_t attitude;
    service_inductor_data_t inductor;
    float voltage;

    mode = debugw_mode();
    if(0U == mode)
    {
        return;
    }

    period_tick = (1U == mode) ? DEBUGW_IMU_PERIOD_TICK : DEBUGW_INDUCTOR_PERIOD_TICK;

    now = service_timetick_what();
    if((uint32)(now - last_tick) < period_tick)
    {
        return;
    }
    last_tick = now;

    switch(mode)
    {
        case 1U:
            /* 50Hz IMU 三轴姿态: roll, pitch, yaw (deg); pitch 为 0-360 全象限(倒置=180°) */
            app_attitude_get_data(&attitude);
            if(0U != attitude.valid)
            {
                wprint("%.1f,%.1f,%.1f\r\n",
                        (double)attitude.roll_deg,
                        (double)attitude.pitch_deg,
                        (double)attitude.yaw_deg);
            }
            break;

        case 2U:
            /* 10Hz 4路电感原始值: CH1, CH2, CH3, CH4 */
            (void)service_inductor_get_snapshot(&inductor, NULL);
            wprint("%u,%u,%u,%u\r\n",
                    inductor.channel_1,
                    inductor.channel_2,
                    inductor.channel_3,
                    inductor.channel_4);
            break;

        case 3U:
            /* 10Hz 电池电压(V) */
            service_batterycheck_get_voltage(&voltage);
            wprint("%.2f\r\n", (double)voltage);
            break;

        default:
            break;
    }
}
