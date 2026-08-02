#include "zf_common_headfile.h"
#include "sys_include.h"
#include "bsp_buzzer.h"
#include "service_timetick.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"
#include "service_packet.h"
#include "service_batterycheck.h"
#include "service_buzzer.h"
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
#include "app_attitude.h"
#include "app_scheduler.h"
#include "app_speed_plan.h"
#include "app_speedout.h"
#include "app_boot_sequence.h"
#include "service_boot_request.h"
#include "service_button.h"

#define BOOT_BUZZ_MS            (500UL)
#define ATTITUDE_VOFA_PERIOD_TICK (100UL)  /* 10 ms, 100 Hz; 0.1 ms/tick */

static void main_attitude_vofa_output(void)
{
    static uint32 last_output_tick = 0UL;
    uint32 now;
    app_attitude_data_t attitude;

    now = service_timetick_what();
    if((uint32)(now - last_output_tick) < ATTITUDE_VOFA_PERIOD_TICK)
    {
        return;
    }
    last_output_tick = now;

    app_attitude_get_data(&attitude);
    if(0U != attitude.valid)
    {
        /* VOFA+ RawData text: roll, pitch, yaw, CRLF.  Wired printf UART. */
        printf("%.3f,%.3f,%.3f\r\n",
                attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg);
    }
}

void main(void)
{
    SystemStart();

    service_timetick_init();
    service_function_queue_init();
    app_scheduler_init();
    service_wireless_uart_init();
    service_packet_init();
    service_batterycheck_init();
    service_buzzer_init();
    service_buzzer_stop();
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
        main_attitude_vofa_output();
        service_tof_task();
        service_batterycheck_task();
        service_inductor_task();
        service_boot_request_process();
        service_function_queue_update();
        service_packet_update();
        app_scheduler_run();
        app_element_pump_events();
        service_negative_pressure_task();
        service_buzzer_task();
        service_led_task();
        service_button_task();
    }
}
