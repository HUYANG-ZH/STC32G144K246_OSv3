#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#ifndef BSP_BUZZER_PIN
#define BSP_BUZZER_PIN              IO_P22
#endif

#ifndef BSP_BUZZER_ACTIVE_LEVEL
#define BSP_BUZZER_ACTIVE_LEVEL     GPIO_HIGH
#endif

void bsp_buzzer_init(void);
void bsp_buzzer_debug(void);
void bsp_buzzer_on(void);
void bsp_buzzer_off(void);
void bsp_buzzer_set(uint8 enable);

#endif
