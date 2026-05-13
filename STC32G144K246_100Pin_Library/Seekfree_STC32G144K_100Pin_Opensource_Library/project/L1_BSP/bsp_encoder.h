#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "zf_common_typedef.h"

typedef struct
{
    int16 left;
    int16 right;
} bsp_encoder_count_t;

void bsp_encoder_init(void);
void bsp_encoder_clear(void);
void bsp_encoder_get_delta(bsp_encoder_count_t *out_count);

#endif
