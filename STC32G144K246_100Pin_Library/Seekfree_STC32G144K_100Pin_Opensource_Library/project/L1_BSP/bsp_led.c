#include "zf_common_headfile.h"
#include "bsp_led.h"

#define BSP_LED_INACTIVE_LEVEL    ((GPIO_HIGH == BSP_LED_ACTIVE_LEVEL) ? GPIO_LOW : GPIO_HIGH)

void bsp_led_init(void)
{
    /* P23 is active low: initialize it high so the LED is off. */
    gpio_init(BSP_LED_PIN, GPO, BSP_LED_INACTIVE_LEVEL, GPO_PUSH_PULL);
}

void bsp_led_debug(void)
{
}

void bsp_led_on(void)
{
    gpio_set_level(BSP_LED_PIN, BSP_LED_ACTIVE_LEVEL);
}

void bsp_led_off(void)
{
    gpio_set_level(BSP_LED_PIN, BSP_LED_INACTIVE_LEVEL);
}

void bsp_led_set(uint8 enable)
{
    if(0U != enable)
    {
        bsp_led_on();
    }
    else
    {
        bsp_led_off();
    }
}
