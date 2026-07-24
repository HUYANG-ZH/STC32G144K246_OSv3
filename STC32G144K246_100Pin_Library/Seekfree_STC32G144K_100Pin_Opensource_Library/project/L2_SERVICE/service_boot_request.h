#ifndef SERVICE_BOOT_REQUEST_H
#define SERVICE_BOOT_REQUEST_H

#include "zf_common_typedef.h"

#define BOOT_REQUEST_USER_SYSTEM   (0x28U)
#define BOOT_REQUEST_FACTORY_ISP   (0x60U)

#ifndef SERVICE_BOOT_REQUEST_ENABLE
#define SERVICE_BOOT_REQUEST_ENABLE    (1U)
#endif

#if SERVICE_BOOT_REQUEST_ENABLE
void service_boot_request_init(void);
void service_boot_request_feed_byte(uint8 rx_byte);
void service_boot_request_process(void);
uint8 service_boot_request_is_pending(void);
#endif

#endif
