#ifndef _zf_device_wireless_uart_h_
#define _zf_device_wireless_uart_h_

#include "zf_common_typedef.h"

#define WIRELESS_UART_INDEX              ( UART_8 )
#define WIRELESS_UART_BUAD_RATE          ( 115200 )
#define WIRELESS_UART_TX_PIN             ( UART8_RX_P92 )
#define WIRELESS_UART_RX_PIN             ( UART8_TX_P93 )
#define WIRELESS_UART_RTS_PIN            ( IO_P91 )

#define WIRELESS_UART_TRANSFER_MODE_RTS      ( 1 )
#define WIRELESS_UART_TRANSFER_MODE_4WIRE    ( 0 )
#define WIRELESS_UART_TRANSFER_MODE          ( WIRELESS_UART_TRANSFER_MODE_RTS )

#define WIRELESS_UART_AUTO_BAUD_RATE     ( 0 )

#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
typedef enum
{
    WIRELESS_UART_AUTO_BAUD_RATE_SUCCESS,
    WIRELESS_UART_AUTO_BAUD_RATE_INIT,
    WIRELESS_UART_AUTO_BAUD_RATE_START,
    WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK,
} wireless_uart_auto_baudrate_state_enum;
#endif

#if ((WIRELESS_UART_TRANSFER_MODE != WIRELESS_UART_TRANSFER_MODE_RTS) && \
     (WIRELESS_UART_TRANSFER_MODE != WIRELESS_UART_TRANSFER_MODE_4WIRE))
    #error "WIRELESS_UART_TRANSFER_MODE setting is invalid."
#endif

#if ((WIRELESS_UART_TRANSFER_MODE == WIRELESS_UART_TRANSFER_MODE_4WIRE) && \
     (1 == WIRELESS_UART_AUTO_BAUD_RATE))
    #error "4-wire compatibility mode requires WIRELESS_UART_AUTO_BAUD_RATE to be 0."
#endif

#define WIRELESS_UART_BUFFER_SIZE        ( 128 )
#define WIRELESS_UART_TIMEOUT_COUNT      ( 0x64 )

uint32      wireless_uart_send_byte      (const uint8 dat);
uint32      wireless_uart_send_buffer    (const uint8 *buff, uint32 len);
uint32      wireless_uart_send_string    (const char *str);
uint32      wireless_uart_read_buffer    (uint8 *buff, uint32 len);
void        wireless_uart_callback       (uint8 uart_dat);
uint8       wireless_uart_init           (void);

#endif
