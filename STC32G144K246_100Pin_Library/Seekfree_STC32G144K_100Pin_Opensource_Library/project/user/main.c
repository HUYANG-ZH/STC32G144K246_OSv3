#include "zf_common_headfile.h"
#include "sys_include.h"
#include "service_timetick.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"
#include "service_packet.h"
#include "service_batterycheck.h"
#include "service_led.h"
#include "service_imu.h"
#include "service_inductor.h"
#include "service_motor.h"
#include "service_negative_pressure.h"
#include "service_speed.h"
#include "service_tof.h"
#include "app_element.h"
#include "app_feedforward.h"
#include "app_inductor_preprocess.h"
#include "app_motion_preprocess.h"
#include "app_motion_postprocess.h"
#include "app_scheduler.h"
#include "app_speed_plan.h"
#include "app_speedout.h"
#include "app_boot_sequence.h"
#include "app_battery_guard.h"
#include "debugw.h"
#include "service_boot_request.h"
#include "service_button.h"

void main(void)
{
    SystemStart();

    service_timetick_init();
    service_function_queue_init();
    app_scheduler_init();
    service_wireless_uart_init();
    service_packet_init();
    service_batterycheck_init();
    service_led_init();
    service_imu_init();
    service_tof_init();
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
    app_boot_sequence_init();
    service_boot_request_init();
    service_button_init();
    debugw_init();

    wdt_init();

    /* 欠压守卫放在阻塞性开机流程之后启动, 避免阻塞期内采样停滞被误判失效 */
    app_battery_guard_init();
    while(1)
    {
        wdt_feed();
        iic_async_process_all_timed(service_timetick_what());
        service_imu_task();
        service_tof_task();
        service_batterycheck_task();
        service_inductor_task();
        service_boot_request_process();
        service_function_queue_update();
        service_packet_update();
        app_scheduler_run();
        app_element_pump_events();
        app_battery_guard_pump_events();
        service_negative_pressure_task();
        service_led_task();
        service_button_task();
        debugw_task();
    }
}
