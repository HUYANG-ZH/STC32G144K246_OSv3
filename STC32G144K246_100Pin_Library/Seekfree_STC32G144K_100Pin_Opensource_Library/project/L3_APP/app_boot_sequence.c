#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_timetick.h"
#include "service_packet.h"
#include "service_wireless_uart.h"
#include "service_buzzer.h"
#include "service_negative_pressure.h"
#include "app_scheduler.h"
#include "app_speedout.h"
#include "app_motion_preprocess.h"
#include "app_boot_sequence.h"

#define APP_BOOT_TASK_ID                    (5U)
#define APP_BOOT_TASK_PRIORITY              (1U)
#define APP_BOOT_TASK_PERIOD_MS             (10U)
#define APP_BOOT_TICKS_PER_MS               (10UL)

#define APP_BOOT_BEEP_MS                    (1000UL)
#define APP_BOOT_WAIT_MS                    (5000UL)
#define APP_BOOT_PRESSURE_PERCENT           (60U)
#define APP_BOOT_PRESSURE_DELAY_MS          (2000UL)
#define APP_BOOT_DEFAULT_SPEED_MPS          (2.1f)

#define APP_BOOT_BEEP_TICKS                 (APP_BOOT_BEEP_MS * APP_BOOT_TICKS_PER_MS)
#define APP_BOOT_WAIT_TICKS                 (APP_BOOT_WAIT_MS * APP_BOOT_TICKS_PER_MS)
#define APP_BOOT_PRESSURE_DELAY_TICKS       (APP_BOOT_PRESSURE_DELAY_MS * APP_BOOT_TICKS_PER_MS)

#define APP_BOOT_STATE_IDLE                 (0U)
#define APP_BOOT_STATE_BEEP                 (1U)
#define APP_BOOT_STATE_WAIT                 (2U)
#define APP_BOOT_STATE_PRESSURE             (3U)
#define APP_BOOT_STATE_DONE                 (4U)
#define APP_BOOT_STATE_ABORTED              (5U)

static uint8 boot_state = APP_BOOT_STATE_IDLE;
static uint32 boot_state_tick = 0U;
static volatile uint8 boot_stop_received = 0U;

static void app_boot_stop_handler(void)
{
    app_speedout_stop();
    service_negative_pressure_set_percent(0U);
    boot_stop_received = 1U;
    wprint("stop,0.000\r\n");
}

static void app_boot_task(void)
{
    uint32 now;

    if((APP_BOOT_STATE_DONE == boot_state) || (APP_BOOT_STATE_ABORTED == boot_state))
    {
        return;
    }

    if(0U != boot_stop_received)
    {
        boot_state = APP_BOOT_STATE_ABORTED;
        wprint("boot_abort,1.000\r\n");
        return;
    }

    now = service_timetick_what();

    switch(boot_state)
    {
        case APP_BOOT_STATE_BEEP:
            if((uint32)(now - boot_state_tick) >= APP_BOOT_BEEP_TICKS)
            {
                boot_state = APP_BOOT_STATE_WAIT;
                boot_state_tick = now;
                wprint("boot_wait,1.000\r\n");
            }
            break;

        case APP_BOOT_STATE_WAIT:
            if((uint32)(now - boot_state_tick) >= APP_BOOT_WAIT_TICKS)
            {
                service_negative_pressure_set_percent(APP_BOOT_PRESSURE_PERCENT);
                boot_state = APP_BOOT_STATE_PRESSURE;
                boot_state_tick = now;
                wprint("boot_pressure,1.000\r\n");
            }
            break;

        case APP_BOOT_STATE_PRESSURE:
            if((uint32)(now - boot_state_tick) >= APP_BOOT_PRESSURE_DELAY_TICKS)
            {
                app_motion_preprocess_config.linear_mps = APP_BOOT_DEFAULT_SPEED_MPS;
                app_speedout_start();
                boot_state = APP_BOOT_STATE_DONE;
                wprint("boot_drive,1.000\r\n");
            }
            break;

        default:
            break;
    }
}

void app_boot_sequence_init(void)
{
    service_negative_pressure_set_percent(0U);
    service_buzzer_beep_ms(APP_BOOT_BEEP_MS);

    boot_stop_received = 0U;
    boot_state = APP_BOOT_STATE_BEEP;
    boot_state_tick = service_timetick_what();

    (void)service_packet_add_action("stop", app_boot_stop_handler, 0UL);

    wprint("boot_beep,1.000\r\n");

    (void)app_scheduler_add(APP_BOOT_TASK_ID, app_boot_task,
            APP_BOOT_TASK_PRIORITY, APP_BOOT_TASK_PERIOD_MS);
}
