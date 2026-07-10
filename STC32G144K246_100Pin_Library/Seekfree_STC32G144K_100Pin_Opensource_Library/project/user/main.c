/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可证英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "sys_include.h"
#include "service_timetick.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"
#include "service_packet.h"
#include "service_batterycheck.h"
#include "service_buzzer.h"
#include "service_imu.h"
#include "service_motor.h"
#include "service_negative_pressure.h"
#include "service_speed.h"
#include "service_delay.h"
#include "shared_lpf.h"
// #include "app_battery_guard.h"
#include "app_element.h"
#include "app_feedforward.h"
#include "app_inductor_preprocess.h"
// #include "app_log.h"
#include "app_motion_preprocess.h"
#include "app_motion_postprocess.h"
#include "app_scheduler.h"
#include "app_speed_plan.h"
#include "app_speedout.h"

void main(void)
{
    SystemStart();

    service_timetick_init();
    service_function_queue_init();
    app_scheduler_init();
    service_wireless_uart_init();
    service_packet_init();
    // app_log_init();
    service_batterycheck_init();
    service_buzzer_init();
    service_buzzer_stop();
    service_imu_init();
    service_delay_ms(2000U);
    service_imu_calibrate_gyro_z();
    service_imu_calibrate_gyro_x();
    service_motor_init();
    service_negative_pressure_init();
    service_speed_init();
    app_inductor_preprocess_init();
    app_motion_preprocess_init();
    app_feedforward_init();
    app_speed_plan_init();
    app_element_init();
    app_speedout_init();
    app_motion_postprocess_init();
    // app_battery_guard_init();
    service_negative_pressure_set_percent(25U);

    while(1)
    {
        service_function_queue_update();
        service_packet_update();
        app_scheduler_run();
        service_negative_pressure_task();
        service_buzzer_task();
    }
}
