#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "app_attitude.h"
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
// 备注信息     由主循环调用; 开关开启后按 50Hz 输出 IMU 三轴姿态(roll/pitch/yaw),
//             vofa+ FireWater 最简格式, 后续复杂调试输出在此追加
//-------------------------------------------------------------------------------------------------------------------
void debugw_task(void)
{
    static uint32 last_tick = 0UL;
    uint32 now;
    app_attitude_data_t attitude;

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

    /* 50Hz IMU 三轴姿态最简输出: roll, pitch, yaw (deg); pitch 为 0-360 全象限(倒置=180°) */
    app_attitude_get_data(&attitude);
    if(0U != attitude.valid)
    {
        wprint("%.1f,%.1f,%.1f\r\n",
                (double)attitude.roll_deg,
                (double)attitude.pitch_deg,
                (double)attitude.yaw_deg);
    }
}
