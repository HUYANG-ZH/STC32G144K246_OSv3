#include "zf_common_headfile.h"
#include "sys_include.h"
#include "service_timetick.h"
#include "service_imu.h"
#include "app_attitude.h"
#include "service_boot_request.h"

#define ATTITUDE_PRINT_INTERVAL_TICK    (20UL)     // 2ms @ 0.1ms/tick = 500Hz 打印

void main(void)
{
    uint32 print_last_tick;

    SystemStart();

    service_timetick_init();
    service_imu_init();
    app_attitude_init();
    service_boot_request_init();

    wdt_init();

    print_last_tick = service_timetick_what();

    while(1)
    {
        wdt_feed();
        service_imu_task();
        service_boot_request_process();

        service_imu_update();
        app_attitude_task();

        {
            uint32 now = service_timetick_what();
            if((uint32)(now - print_last_tick) >= ATTITUDE_PRINT_INTERVAL_TICK)
            {
                app_attitude_data_t att;
                print_last_tick = now;
                app_attitude_get_data(&att);
                if(0U != att.valid)
                {
                    printf("%.4f,%.4f,%.4f,%u\r\n",
                        att.roll_deg, att.pitch_deg, att.yaw_deg, att.sequence);
                }
            }
        }
    }
}
