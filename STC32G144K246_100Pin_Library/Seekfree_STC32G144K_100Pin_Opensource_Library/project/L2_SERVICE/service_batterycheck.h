#ifndef SERVICE_BATTERYCHECK_H
#define SERVICE_BATTERYCHECK_H

#include "zf_common_typedef.h"

void service_batterycheck_init(void);
void service_batterycheck_debug(void);
void service_batterycheck_task(void);
uint8 service_batterycheck_is_valid(void);
uint8 service_batterycheck_raw_is_valid(void);
void service_batterycheck_get_raw(uint16 *rawdata);
uint8 service_batterycheck_get_raw_snapshot(uint16 *rawdata, uint32 *sequence);
void service_batterycheck_get_voltage(float *voltage);

#endif
