#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_timetick.h"
#include "service_wireless_uart.h"
#include "service_inductor.h"
#include "service_tof.h"
#include "service_speed.h"
#include "service_imu.h"
#include "app_inductor_preprocess.h"
#include "app_motion_postprocess.h"
#include "debugw.h"

/* 调试模式(无线 debuginfo):
   0 = 关闭
   1 = 4路电感归一化值(CH1~CH4) + TOF测距(mm), 20Hz
   2 = 4路电感原始值 CH1~CH4, 10Hz
   3 = 电机速度 left,right (m/s), 100Hz
   4 = 角速度环目标/实际 target,actual (rad/s), 50Hz
   5 = 轮速 left,right (m/s) + IMU三轴角速度 gx,gy,gz (deg/s), 20Hz */
#define DEBUGW_IMU_PERIOD_TICK          (500UL)    /* 20Hz, 0.1ms/tick */
#define DEBUGW_INDUCTOR_PERIOD_TICK     (200UL)    /* 50Hz */
#define DEBUGW_MOTOR_PERIOD_TICK        (100UL)    /* 100Hz */

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
    if(mode > 5U)
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
    app_inductor_preprocess_data_t preprocessed;
    service_inductor_data_t inductor;
    service_speed_data_t speed;

    mode = debugw_mode();
    if(0U == mode)
    {
        return;
    }

    period_tick = ((1U == mode) || (5U == mode)) ? DEBUGW_IMU_PERIOD_TICK :
            ((3U == mode) ? DEBUGW_MOTOR_PERIOD_TICK : DEBUGW_INDUCTOR_PERIOD_TICK);

    now = service_timetick_what();
    if((uint32)(now - last_tick) < period_tick)
    {
        return;
    }
    last_tick = now;

    switch(mode)
    {
        case 1U:
            /* 20Hz 4路电感归一化值 + TOF测距: norm(CH1),norm(CH2),norm(CH3),norm(CH4),tof(mm) */
            app_inductor_preprocess_get_data(&preprocessed);
            wprint("%.1f,%.1f,%.1f,%.1f,%u\r\n",
                    (double)preprocessed.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH1],
                    (double)preprocessed.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH2],
                    (double)preprocessed.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH3],
                    (double)preprocessed.normalized[APP_INDUCTOR_PREPROCESS_INDEX_CH4],
                    (uint16)service_tof_get_distance_mm());
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
            /* 100Hz 电机速度: left,right (m/s) */
            service_speed_get(&speed);
            wprint("%.2f,%.2f\r\n",
                    (double)speed.left_mps,
                    (double)speed.right_mps);
            break;

        case 4U:
            /* 50Hz 角速度环监控: target_yaw_rate, actual_yaw_rate (rad/s) */
            {
                app_motion_postprocess_data_t motion;

                app_motion_postprocess_get_data(&motion);
                wprint("%.3f,%.3f\r\n",
                        (double)motion.target_yaw_rate_radps,
                        (double)motion.actual_yaw_rate_radps);
            }
            break;

        case 5U:
            /* 20Hz 轮速+IMU角速度: left,right (m/s), gx,gy,gz (deg/s) */
            {
                service_imu_gyro_t gyro;

                service_speed_get(&speed);
                service_imu_read_gyro(&gyro);
                wprint("%.2f,%.2f,%.1f,%.1f,%.1f\r\n",
                        (double)speed.left_mps,
                        (double)speed.right_mps,
                        (double)gyro.gyro_x,
                        (double)gyro.gyro_y,
                        (double)gyro.gyro_z);
            }
            break;

        default:
            break;
    }
}
