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
/* ISR 安全读取滤波后电压(不触发刷新) */
float service_batterycheck_get_filtered_voltage(void);
/* 滤波已就绪标记: 1 = 已发布至少一次滤波电压 */
uint8 service_batterycheck_filter_ready(void);

#endif
