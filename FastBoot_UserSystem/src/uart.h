#ifndef __UART_H__
#define __UART_H__

void uart_init(void);
void uart_isr(void);

extern BOOL bUartRxReady;
extern BYTE edata UartTxBuffer[256];
extern BYTE edata UartRxBuffer[256];

void uart_send(BYTE status, BYTE size);
void uart_recv_done(void);

#endif
