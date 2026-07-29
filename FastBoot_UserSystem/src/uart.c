#include "stc.h"
#include "uart.h"
#include "dfu.h"


BOOL bUartRxReady;

BYTE UartrecvIndex;
BYTE UartRecvStep;
BYTE UartRecvSum;

BYTE edata UartTxBuffer[256];
BYTE edata UartRxBuffer[256];

void uart_init(void)
{
    WORD reload;

    /*
     * Match the clock retained across the software reset.  Cold and hot entry
     * therefore expose the same nominal 1,000,000 baud to the host.
     */
    reload = bDfuHotEntry ? (WORD)UART_T1_RELOAD_HOT
                          : (WORD)UART_T1_RELOAD_COLD;

    S1BRT = 0;
    SCON = 0x52;

    TMOD &= ~0xf0;
    T1x12 = 1;
    TL1 = (BYTE)reload;
    TH1 = (BYTE)(reload >> 8);
    TR1 = 1;

    uart_recv_done();
}


void uart_isr(void)
{
    BYTE dat;
    
    if (RI)
    {
        RI = 0;
        
        UartRecvSum += (dat = SBUF);
        switch (UartRecvStep)
        {
        case 0:
L_CheckHead:
            UartRecvStep = ((UartRecvSum = dat) == '#');
            break;
        case 1:
            UartRxBuffer[0] = dat;
            UartrecvIndex = 0;
            UartRecvStep++;
            break;
        case 2:
            UartRxBuffer[1 + UartrecvIndex++] = dat;
            if (UartrecvIndex >= UartRxBuffer[0])
                UartRecvStep++;
            break;
        case 3:
            if (dat != '$') goto L_CheckHead;
            UartRecvStep++;
            break;
        case 4:
            if (UartRecvSum != 0) goto L_CheckHead;
            bUartRxReady = 1;
            UartRecvStep++;
            break;
        default:
            break;
        }
    }
}


static BYTE send(BYTE dat)
{
    while (!TI);
    TI = 0;
    SBUF = dat;
    
    return dat;
}

void uart_send(BYTE status, BYTE size)
{
    BYTE sum;
    BYTE i;
    
    sum = send('@');
    sum += send(status);
    sum += send(size);
    if (size)
    {
        for (i = 0; i < size; i++)
        {
            sum += send(UartTxBuffer[i]);
        }
    }
    sum += send('$');
    send(-sum);
    
    while (!TI);
}

void uart_recv_done(void)
{
    bUartRxReady = 0;
    UartrecvIndex = 0;
    UartRecvStep = 0;
    UartRecvSum = 0;
}


