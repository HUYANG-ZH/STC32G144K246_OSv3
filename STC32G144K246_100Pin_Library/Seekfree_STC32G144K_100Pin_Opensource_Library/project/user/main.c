#include "zf_common_headfile.h"
#include "sys_include.h"
#include "bsp_buzzer.h"
#include "service_boot_request.h"

#define BOOT_BUZZ_MS            (500UL)
#define PERIOD_BUZZ_MS          (500UL)
#define PERIOD_ON_MS            (20UL)
#define PERIOD_OFF_MS           (PERIOD_BUZZ_MS - PERIOD_ON_MS)

void main(void)
{
    SystemStart();

    bsp_buzzer_init();
    service_boot_request_init();

    wdt_init();

    bsp_buzzer_on();
    system_delay_ms(BOOT_BUZZ_MS);
    bsp_buzzer_off();

    while(1)
    {
        wdt_feed();
        service_boot_request_process();

        bsp_buzzer_on();
        system_delay_ms(PERIOD_ON_MS);
        bsp_buzzer_off();
        system_delay_ms(PERIOD_OFF_MS);
    }
}
