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

#endif
