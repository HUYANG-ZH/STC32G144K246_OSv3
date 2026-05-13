#ifndef SERVICE_BUZZER_H
#define SERVICE_BUZZER_H

#include "zf_common_typedef.h"

void service_buzzer_init(void);
void service_buzzer_beep_ms(uint32 duration_ms);
void service_buzzer_task(void);
void service_buzzer_stop(void);
uint8 service_buzzer_is_on(void);

#endif
