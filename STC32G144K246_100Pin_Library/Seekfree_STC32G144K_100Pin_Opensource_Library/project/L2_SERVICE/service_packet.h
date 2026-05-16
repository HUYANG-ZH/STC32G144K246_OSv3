#ifndef SERVICE_PACKET_H
#define SERVICE_PACKET_H

#include "zf_common_typedef.h"

#ifndef SERVICE_PACKET_VARIABLE_MAX
#define SERVICE_PACKET_VARIABLE_MAX        (8U)
#endif

#ifndef SERVICE_PACKET_ACTION_MAX
#define SERVICE_PACKET_ACTION_MAX          (8U)
#endif

#ifndef SERVICE_PACKET_VALUE_MAX
#define SERVICE_PACKET_VALUE_MAX           (8U)
#endif

typedef void (*service_packet_action_func_t)(void);

void service_packet_init(void);
void service_packet_debug(void);
uint8 service_packet_add_variable(const char *name, float *value, uint8 value_count);
uint8 service_packet_add_action(const char *name, service_packet_action_func_t func, uint32 delay_ms);
void service_packet_update(void);

#endif
