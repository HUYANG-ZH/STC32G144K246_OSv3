#ifndef SERVICE_NEGATIVE_PRESSURE_H
#define SERVICE_NEGATIVE_PRESSURE_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

#ifndef SERVICE_NEGATIVE_PRESSURE_PWM_CH
#define SERVICE_NEGATIVE_PRESSURE_PWM_CH        PWMD_CH1_PB4
#endif

#ifndef SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ
#define SERVICE_NEGATIVE_PRESSURE_PWM_FREQ_HZ   (50U)
#endif

void service_negative_pressure_init(void);
void service_negative_pressure_debug(void);
void service_negative_pressure_set_percent(uint8 percent);
uint8 service_negative_pressure_get_percent(void);

#endif
