#ifndef SERVICE_FUNCTION_QUEUE_H
#define SERVICE_FUNCTION_QUEUE_H

#include "zf_common_typedef.h"

#ifndef SERVICE_FUNCTION_QUEUE_MAX
#define SERVICE_FUNCTION_QUEUE_MAX      (8U)
#endif

typedef void (*service_function_queue_func_t)(void);

void service_function_queue_init(void);
uint8 service_function_queue_add(service_function_queue_func_t func, uint32 delay_ms, uint8 priority);
void service_function_queue_update(void);

#endif
