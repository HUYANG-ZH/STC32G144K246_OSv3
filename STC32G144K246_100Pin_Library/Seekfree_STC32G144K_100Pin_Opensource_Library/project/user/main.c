#include "zf_common_headfile.h"
#include "sys_include.h"
#include "bsp_buzzer.h"
#include "service_timetick.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"
#include "service_packet.h"
#include "service_batterycheck.h"
#include "service_buzzer.h"
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
#include "service_boot_request.h"
#include "service_button.h"

#define BOOT_BUZZ_MS            (500UL)
#define MAIN_TOF_REPORT_PERIOD_TICK (333UL)   // 约30Hz, 0.1ms/tick

void main(void)
{
    uint32 tof_report_last_tick;
    uint32 now;
    uint16 tof_distance_mm;
    uint8 tof_range_status;

    SystemStart();

    service_timetick_init();
    tof_report_last_tick = service_timetick_what();
    service_function_queue_init();
    app_scheduler_init();
    service_wireless_uart_init();
    service_packet_init();
    service_batterycheck_init();
    service_buzzer_init();
    service_buzzer_stop();
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

    wdt_init();

    bsp_buzzer_init();
    bsp_buzzer_on();
    system_delay_ms(BOOT_BUZZ_MS);
    bsp_buzzer_off();

    while(1)
    {
        wdt_feed();
        iic_async_process_all_timed(service_timetick_what());
        service_imu_task();
        service_tof_task();
        now = service_timetick_what();
        if((uint32)(now - tof_report_last_tick) >= MAIN_TOF_REPORT_PERIOD_TICK)
        {
            tof_report_last_tick = now;
            tof_distance_mm = service_tof_get_distance_mm();
            tof_range_status = service_tof_get_range_status();
            wprint("tof_distance_mm,%u,%u\r\n", tof_distance_mm, (uint16)tof_range_status);
        }
        service_batterycheck_task();
        service_inductor_task();
        service_boot_request_process();
        service_function_queue_update();
        service_packet_update();
        app_scheduler_run();
        app_element_pump_events();
        service_negative_pressure_task();
        service_buzzer_task();
        service_button_task();
    }
}
