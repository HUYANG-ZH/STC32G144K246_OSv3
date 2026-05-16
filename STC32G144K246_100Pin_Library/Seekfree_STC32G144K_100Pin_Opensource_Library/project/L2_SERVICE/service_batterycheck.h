#ifndef SERVICE_BATTERYCHECK_H
#define SERVICE_BATTERYCHECK_H

#include "zf_common_typedef.h"

void service_batterycheck_init(void);
void service_batterycheck_debug(void);
void service_batterycheck_task(void);
void service_batterycheck_get_voltage(float *voltage);

#endif
