#ifndef SERVICE_INDUCTOR_H
#define SERVICE_INDUCTOR_H

#include "zf_common_typedef.h"

typedef struct
{
    uint16 channel_1;
    uint16 channel_2;
    uint16 channel_3;
    uint16 channel_4;
} service_inductor_data_t;

void service_inductor_init(void);
void service_inductor_get_data(service_inductor_data_t *out_data);

#endif
