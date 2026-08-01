#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#ifndef BSP_LED_PIN
#define BSP_LED_PIN                 IO_P23
#endif

#ifndef BSP_LED_ACTIVE_LEVEL
#define BSP_LED_ACTIVE_LEVEL        GPIO_LOW
#endif

void bsp_led_init(void);
void bsp_led_debug(void);
void bsp_led_on(void);
void bsp_led_off(void);
void bsp_led_set(uint8 enable);

#endif
