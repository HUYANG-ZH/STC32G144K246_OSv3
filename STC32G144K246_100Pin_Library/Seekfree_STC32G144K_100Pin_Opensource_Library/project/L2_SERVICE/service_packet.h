#ifndef SERVICE_PACKET_H
#define SERVICE_PACKET_H

#include "zf_common_typedef.h"

#ifndef SERVICE_PACKET_VARIABLE_MAX
#define SERVICE_PACKET_VARIABLE_MAX        (48U)
#endif

#ifndef SERVICE_PACKET_ACTION_MAX
#define SERVICE_PACKET_ACTION_MAX          (10U)
#endif

#ifndef SERVICE_PACKET_VALUE_MAX
#define SERVICE_PACKET_VALUE_MAX           (8U)
#endif

/* 单次主循环最多处理的无线接收分块数，避免持续流量独占后台 CPU。 */
#ifndef SERVICE_PACKET_MAX_CHUNKS_PER_UPDATE
#define SERVICE_PACKET_MAX_CHUNKS_PER_UPDATE (2U)
#endif

typedef void (*service_packet_action_func_t)(void);
typedef void (*service_packet_write_callback_t)(void);

void service_packet_init(void);
void service_packet_debug(void);
uint8 service_packet_add_variable(const char *name, float *value, uint8 value_count);
uint8 service_packet_add_variable_with_callback(const char *name, float *value, uint8 value_count,
        service_packet_write_callback_t callback);
uint8 service_packet_add_action(const char *name, service_packet_action_func_t func, uint32 delay_ms);
void service_packet_update(void);

#endif
