#ifndef SERVICE_TIMETICK_H
#define SERVICE_TIMETICK_H

#include "zf_common_typedef.h"

void service_timetick_init(void);
void service_timetick_debug(void);
/* Called by foreground, control timers and IMU DMA/EXTI ISRs. */
uint32 service_timetick_what(void) reentrant;

#endif
