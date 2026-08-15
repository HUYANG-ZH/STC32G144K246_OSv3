#ifndef __BSP_INDUCTOR_H
#define __BSP_INDUCTOR_H
#include "zf_common_typedef.h"
typedef struct {
    uint16 Channel_1;
    uint16 Channel_2;
    uint16 Channel_3;
    uint16 Channel_4;
} inductor_rawdata_t;

void bsp_inductor_init(void);
void bsp_inductor_debug(void);
void bsp_inductor_get(inductor_rawdata_t* out_data);
uint8 bsp_inductor_get_snapshot(inductor_rawdata_t *out_data, uint32 *sequence);
/* 启动一次 ADC2 DMA 全通道扫描；返回 0 表示上一帧尚未结束，不会等待。 */
uint8 bsp_inductor_request_sample(void);
/* 由 DMA ADC2 中断入口调用。 */
void bsp_inductor_dma_irq_handler(void);
uint8 bsp_inductor_sample_is_valid(void);
uint8 bsp_inductor_is_busy(void);
void bsp_inductor_recover(void);

#endif
