#ifndef SERVICE_SPEED_H
#define SERVICE_SPEED_H

#include "zf_common_typedef.h"

typedef struct
{
    float left_mps;
    float right_mps;
} service_speed_data_t;

void service_speed_init(void);
void service_speed_update(void);
void service_speed_get(service_speed_data_t *out_speed);

#endif
