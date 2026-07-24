#include "stc.h"
#include "uart.h"
#include "iap.h"
#include "dfu.h"

static void sys_init(void);

void main(void)
{
    /* Must run before changing WTST or clearing the retained DfuFlag. */
    dfu_check();

    sys_init();
    uart_init();

    while (1)
    {
        WDT_CONTR |= 0x10U;                     // feed watchdog
        uart_isr();
        dfu_events();
    }
}

static void sys_init(void)
{
    /* 124 MHz hot entry needs the APP's wait-state setting retained. */
    WTST = bDfuHotEntry ? 4U : 0U;
    CKCON = 0x00U;
    EAXFR = 1;

    P3M0 &= (BYTE)~0x03U;
    P3M1 &= (BYTE)~0x03U;

    WDT_CONTR = 0x37U;                          // EN_WDT=1, CLR_WDT=1, prescaler=7(max)
}
