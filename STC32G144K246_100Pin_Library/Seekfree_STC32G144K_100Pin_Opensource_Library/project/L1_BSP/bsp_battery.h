#ifndef BSP_BATTERY_H
#define BSP_BATTERY_H

#include "zf_common_typedef.h"

void bsp_battery_init(void);
void bsp_battery_debug(void);
uint8 bsp_battery_request_sample(void);
void bsp_battery_dma_irq_handler(void);
uint8 bsp_battery_sample_is_valid(void);
uint32 bsp_battery_sample_sequence(void);
void bsp_battery_get_raw(uint16 *rawdata);
uint8 bsp_battery_get_snapshot(uint16 *rawdata, uint32 *sequence);
uint8 bsp_battery_is_busy(void);
void bsp_battery_recover(void);
void bsp_battery_vol(float *vol);
/* 由已取回的原始采样值直接换算电压, 避免二次读快照 */
float bsp_battery_vol_from_raw(uint16 rawdata);

#endif
