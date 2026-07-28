#include "zf_common_headfile.h"
#include "service_boot_request.h"
#include "service_motor.h"
#include "sys_start.h"
#include "service_wireless_uart.h"

#if SERVICE_BOOT_REQUEST_ENABLE

#define BOOT_REQUEST_FRAME_SIZE     (12U)
#define BOOT_REQUEST_PREFIX_SIZE    (10U)
#define BOOT_REQUEST_DFU_TAG        (0x12ABCD34UL)
#define BOOT_REQUEST_ACK_SIZE       (10U)
#define BOOT_REQUEST_TX_TIMEOUT     (5000UL)

static const uint8 boot_request_prefix[4] = {0x53U, 0x42U, 0x4CU, 0x52U};
static volatile uint8 service_boot_request_pending = 0U;
static volatile uint8 boot_request_frame[BOOT_REQUEST_FRAME_SIZE];
static volatile uint8 boot_request_index = 0U;

volatile unsigned long xdata boot_request_dfu_flag _at_ 0xfffc;

static uint16 service_boot_request_crc16(const uint8 *buffer, uint8 length)
{
    uint8 i;
    uint8 bit_index;
    uint16 crc = 0xFFFFU;

    for(i = 0U; i < length; i++)
    {
        crc ^= (uint16)buffer[i] << 8;
        for(bit_index = 0U; bit_index < 8U; bit_index++)
        {
            if(0U != (crc & 0x8000U))
            {
                crc = (uint16)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void service_boot_request_reset_parser(uint8 rx_byte)
{
    boot_request_index = 0U;
    if(rx_byte == boot_request_prefix[0])
    {
        boot_request_frame[0] = rx_byte;
        boot_request_index = 1U;
    }
}

void service_boot_request_feed_byte(uint8 rx_byte)
{
    uint8 command;
    uint16 expected_crc;
    uint16 actual_crc;

    if(0U != service_boot_request_pending)
    {
        return;
    }

    if(boot_request_index < 4U)
    {
        if(rx_byte != boot_request_prefix[boot_request_index])
        {
            service_boot_request_reset_parser(rx_byte);
            return;
        }
    }

    boot_request_frame[boot_request_index++] = rx_byte;
    if(boot_request_index < BOOT_REQUEST_FRAME_SIZE)
    {
        return;
    }

    command = boot_request_frame[4];
    expected_crc = (uint16)boot_request_frame[10]
            | ((uint16)boot_request_frame[11] << 8);
    actual_crc = service_boot_request_crc16(boot_request_frame, BOOT_REQUEST_PREFIX_SIZE);

    if((boot_request_frame[5] == (uint8)(~command))
            && (0x32U == boot_request_frame[6])
            && (0x47U == boot_request_frame[7])
            && (0x14U == boot_request_frame[8])
            && (0x4BU == boot_request_frame[9])
            && (expected_crc == actual_crc)
            && ((BOOT_REQUEST_USER_SYSTEM == command)
                || (BOOT_REQUEST_FACTORY_ISP == command)))
    {
        service_boot_request_pending = command;
    }

    boot_request_index = 0U;
}

uint8 service_boot_request_is_pending(void)
{
    return service_boot_request_pending;
}

void service_boot_request_process(void)
{
    uint8 command = service_boot_request_pending;
    uint8 ack_frame[BOOT_REQUEST_ACK_SIZE];

    if(0U == command)
    {
        return;
    }

    service_boot_request_pending = 0U;

    ack_frame[0] = 0xA5U;
    ack_frame[1] = 0x5AU;
    ack_frame[2] = 0x53U;
    ack_frame[3] = 0x42U;
    ack_frame[4] = 0x4CU;
    ack_frame[5] = 0x52U;
    ack_frame[6] = command;
    ack_frame[7] = (uint8)(~command);
    ack_frame[8] = 0x0DU;
    ack_frame[9] = 0x0AU;

    uart_write_buffer(DEBUG_UART_INDEX, ack_frame, BOOT_REQUEST_ACK_SIZE);

    {
        uint32 tx_wait = BOOT_REQUEST_TX_TIMEOUT;
        while (0U != uart_tx_is_busy(DEBUG_UART_INDEX) && (0UL != tx_wait))
        {
            tx_wait--;
            wdt_feed();
        }
    }

    EAXFR = 1;
    EA = 0;

    DMA_UR1R_CR = 0x00;
    service_motor_stop();

    if(BOOT_REQUEST_USER_SYSTEM == command)
    {
        boot_request_dfu_flag = BOOT_REQUEST_DFU_TAG;
        IAP_CONTR = 0x28;
    }
    else
    {
        IAP_CONTR = 0x60;
    }

    #pragma SAVE         // 保存当前的编译器优化状态
    #pragma OPTIMIZE(0)  // 局部将优化级别降为 0（关闭优化，OT(0) 也可以）

    while(1)
    {
    }

    #pragma RESTORE      // 恢复为之前全局设置的高优化级别（如 O8）
}

void service_boot_request_init(void)
{
    boot_request_dfu_flag = 0UL;
    uart_rx_handlers[UART_1] = service_boot_request_feed_byte;
    #if __DBGFLAG__
    printf(">>[service_boot_request_init]\r\n");
    wprint(">>[service_boot_request_init]\r\n");
    #endif
}

#endif
