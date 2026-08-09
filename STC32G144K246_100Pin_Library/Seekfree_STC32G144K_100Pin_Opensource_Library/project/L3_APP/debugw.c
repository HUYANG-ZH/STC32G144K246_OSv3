#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "service_inductor.h"
#include "debugw.h"

/* 无线调试输出周期: 50Hz, 0.1ms/tick */
#define DEBUGW_PERIOD_TICK          (200UL)

/* 无线开关: debuginfo >= 0.5 开启调试输出, < 0.5 关闭 */
static volatile float debugw_debug_info = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无线调试模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     debugw_init();
// 备注信息     注册 debuginfo 无线变量, 由主循环调用 debugw_task() 周期输出
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无线调试任务
// 参数说明     void
// 返回参数     void
// 使用示例     debugw_task();
// 备注信息     由主循环调用; 开关开启后按 50Hz 输出 4 路电感原始 ADC 值,
//             vofa+ FireWater 最简格式, 后续复杂调试输出在此追加
//-------------------------------------------------------------------------------------------------------------------
void debugw_task(void)
{
    static uint32 last_tick = 0UL;
    uint32 now;
    service_inductor_data_t inductor;

    if(debugw_debug_info < 0.5f)
    {
        return;
    }

    now = service_timetick_what();
    if((uint32)(now - last_tick) < DEBUGW_PERIOD_TICK)
    {
        return;
    }
    last_tick = now;

    /* 50Hz 4 路电感原始值最简输出: CH1, CH2, CH3, CH4 */
    (void)service_inductor_get_snapshot(&inductor, NULL);
    wprint("%u,%u,%u,%u\r\n",
            inductor.channel_1,
            inductor.channel_2,
            inductor.channel_3,
            inductor.channel_4);
}
