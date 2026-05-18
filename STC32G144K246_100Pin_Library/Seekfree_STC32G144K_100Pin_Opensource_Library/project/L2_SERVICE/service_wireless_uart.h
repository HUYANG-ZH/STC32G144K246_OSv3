#ifndef SERVICE_WIRELESS_UART_H
#define SERVICE_WIRELESS_UART_H

#include "zf_common_typedef.h"

void service_wireless_uart_init(void);
void service_wireless_uart_debug(void);
uint32 service_wireless_uart_read_buffer(uint8 *buff, uint32 len);
uint32 service_wireless_uart_send_buffer(const uint8 *buff, uint32 len);
uint32 wprint(const char *format, ...);

#endif
