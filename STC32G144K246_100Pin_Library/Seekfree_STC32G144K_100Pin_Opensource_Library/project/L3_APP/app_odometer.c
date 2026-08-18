#include "zf_common_headfile.h"
#include "sys_tfpu.h"
#include "service_speed.h"
#include "service_packet.h"
#include "service_wireless_uart.h"
#include "app_odometer.h"

/* 无线展示缓存: [0]=总里程  [1]=左轮里程  [2]=右轮里程 (单位 米) */
#define APP_ODOMETER_VIEW_COUNT    (3U)

static volatile float app_odometer_view[APP_ODOMETER_VIEW_COUNT] = {0.0f, 0.0f, 0.0f};

void app_odometer_reset(void)
{
    uint8 i;
    uint8 ea_backup;

    service_speed_odometer_reset();
    ea_backup = EA;
    EA = 0;
    for(i = 0; i < APP_ODOMETER_VIEW_COUNT; i++)
    {
        app_odometer_view[i] = 0.0f;
    }
    EA = ea_backup;
}

float app_odometer_get_total_m(void)
{
    uint8 ea_backup;
    float total;

    ea_backup = EA;
    EA = 0;
    total = app_odometer_view[0];
    EA = ea_backup;
    return total;
}

/* 主循环周期调用: 从 service_speed 拷贝最新里程到无线展示缓存 */
void app_odometer_task(void)
{
    service_speed_data_t speed;
    uint8 ea_backup;

    service_speed_get(&speed);
    ea_backup = EA;
    EA = 0;
    app_odometer_view[0] = speed.odo_total_m;
    app_odometer_view[1] = speed.odo_left_m;
    app_odometer_view[2] = speed.odo_right_m;
    EA = ea_backup;
}

void app_odometer_init(void)
{
    (void)service_packet_add_variable("ODO",
            (float *)app_odometer_view, APP_ODOMETER_VIEW_COUNT);
    (void)service_packet_add_action("odo_reset", app_odometer_reset, 0UL);
    #if __DBGFLAG__
    printf(">>[app_odometer_init]\r\n");
    wprint(">>[app_odometer_init]\r\n");
    #endif
}
