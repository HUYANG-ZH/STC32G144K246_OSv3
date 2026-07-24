#ifndef SERVICE_INDUCTOR_H
#define SERVICE_INDUCTOR_H

#include "zf_common_typedef.h"

typedef struct
{
    uint16 channel_1;
    uint16 channel_2;
    uint16 channel_m;
    uint16 channel_3;
    uint16 channel_4;
} service_inductor_data_t;

void service_inductor_init(void);
void service_inductor_debug(void);
void service_inductor_get_data(service_inductor_data_t *out_data);
uint8 service_inductor_get_snapshot(service_inductor_data_t *out_data, uint32 *sequence);
uint8 service_inductor_request_sample(void);
uint8 service_inductor_sample_is_valid(void);
uint8 service_inductor_is_busy(void);
void service_inductor_task(void);

#endif
