#include "zf_common_headfile.h"
#include "bsp_buzzer.h"

#define BSP_BUZZER_INACTIVE_LEVEL   ((GPIO_HIGH == BSP_BUZZER_ACTIVE_LEVEL) ? GPIO_LOW : GPIO_HIGH)

void bsp_buzzer_init(void)
{
    gpio_init(BSP_BUZZER_PIN, GPO, BSP_BUZZER_ACTIVE_LEVEL, GPO_PUSH_PULL);
}

void bsp_buzzer_on(void)
{
    gpio_set_level(BSP_BUZZER_PIN, BSP_BUZZER_ACTIVE_LEVEL);
}

void bsp_buzzer_off(void)
{
    gpio_set_level(BSP_BUZZER_PIN, BSP_BUZZER_INACTIVE_LEVEL);
}

void bsp_buzzer_set(uint8 enable)
{
    if(0U != enable)
    {
        bsp_buzzer_on();
    }
    else
    {
        bsp_buzzer_off();
    }
}
