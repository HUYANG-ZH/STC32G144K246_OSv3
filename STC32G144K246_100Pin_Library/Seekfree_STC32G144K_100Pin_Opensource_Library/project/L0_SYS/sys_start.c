#include "zf_common_headfile.h"
#include "sys_start.h"
#include "sys_tfpu.h"
static void tfpu_start(void);

void SystemStart(void)
{
    clock_init(SYSTEM_CLOCK_124M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    tfpu_start();
}

static void tfpu_start(void)
{
    P_SW2 |= 0x80;
    TFPU_CLKDIV = 0x02;
    tfpu_init();
}
