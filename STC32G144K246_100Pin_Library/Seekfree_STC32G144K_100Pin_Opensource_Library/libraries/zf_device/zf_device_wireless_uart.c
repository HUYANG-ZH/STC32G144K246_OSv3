#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_type.h"

#pragma warning disable = 183
#pragma warning disable = 177

#if ((WIRELESS_UART_BUFFER_SIZE & (WIRELESS_UART_BUFFER_SIZE - 1U)) != 0U)
    #error "WIRELESS_UART_BUFFER_SIZE must be a power of two."
#endif

#define WIRELESS_UART_BUFFER_MASK     (WIRELESS_UART_BUFFER_SIZE - 1U)

// UART DMA RX is a single-byte producer.  These uint8 indices form a
// lock-free single-producer/single-consumer queue: ISR writes head only and
// foreground reads tail only.  This removes the FIFO read/write race.
static uint8 xdata wireless_uart_buffer[WIRELESS_UART_BUFFER_SIZE];
static volatile uint8 wireless_uart_rx_head = 0U;
static volatile uint8 wireless_uart_rx_tail = 0U;
static volatile uint16 wireless_uart_rx_dropped = 0U;

#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
static fifo_struct wireless_uart_auto_baud_fifo;
static uint8 wireless_uart_auto_baud_buffer[WIRELESS_UART_BUFFER_SIZE];
#endif

#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
static volatile wireless_uart_auto_baudrate_state_enum wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_INIT;
static volatile uint8 wireless_auto_baud_data[3] = {0x00, 0x01, 0x03};
#endif

static uint32 wireless_uart_send_direct(const uint8 *buff, uint32 len)
{
    uint16 request_len;
    uint16 accepted_len;

    if(NULL == buff)
    {
        return len;
    }

    // Copy only into the UART driver's xdata DMA queue.  The loop chunks a
    // wide API request; it never waits for the physical transmission.
    while(0U != len)
    {
        request_len = (len > 0xFFFFUL) ? 0xFFFFU : (uint16)len;
        accepted_len = uart_write_buffer_async(WIRELESS_UART_INDEX, buff, request_len);
        buff += accepted_len;
        len -= accepted_len;
        if(accepted_len != request_len)
        {
            break;
        }
    }

    return len;
}

uint32 wireless_uart_send_byte(const uint8 dat)
{
#if (WIRELESS_UART_TRANSFER_MODE == WIRELESS_UART_TRANSFER_MODE_RTS)
    if(gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return 1U;
    }
#else
#endif

    return (1U == uart_write_buffer_async(WIRELESS_UART_INDEX, &dat, 1U)) ? 0U : 1U;
}

uint32 wireless_uart_send_buffer(const uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);

#if (WIRELESS_UART_TRANSFER_MODE == WIRELESS_UART_TRANSFER_MODE_RTS)
    if(gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return len;
    }
#else
#endif

    return wireless_uart_send_direct(buff, len);
}

uint32 wireless_uart_send_string(const char *str)
{
    uint32 len;

    zf_assert(NULL != str);
    len = strlen(str);

    return wireless_uart_send_buffer((const uint8 *)str, len);
}

uint32 wireless_uart_read_buffer(uint8 *buff, uint32 len)
{
    uint32 data_len = 0U;
    uint8 tail;

    zf_assert(NULL != buff);

    while(data_len < len)
    {
        tail = wireless_uart_rx_tail;
        if(tail == wireless_uart_rx_head)
        {
            break;
        }

        buff[data_len] = wireless_uart_buffer[tail];
        wireless_uart_rx_tail = (uint8)((tail + 1U) & WIRELESS_UART_BUFFER_MASK);
        data_len++;
    }

    return data_len;
}

void wireless_uart_callback(uint8 uart_dat)
{
#if WIRELESS_UART_AUTO_BAUD_RATE
    if(WIRELESS_UART_AUTO_BAUD_RATE_START == wireless_auto_baud_flag)
    {
        fifo_write_buffer(&wireless_uart_auto_baud_fifo, &uart_dat, 1U);
        if(3U == fifo_used(&wireless_uart_auto_baud_fifo))
        {
            uint32 wireless_auto_baud_count = 3U;

            wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK;
            fifo_read_buffer(&wireless_uart_auto_baud_fifo, (uint8 *)wireless_auto_baud_data,
                    &wireless_auto_baud_count, FIFO_READ_AND_CLEAN);
        }
        return;
    }
#endif

    {
        uint8 next_head = (uint8)((wireless_uart_rx_head + 1U) & WIRELESS_UART_BUFFER_MASK);
        if(next_head == wireless_uart_rx_tail)
        {
            wireless_uart_rx_dropped++;
            return;
        }

        wireless_uart_buffer[wireless_uart_rx_head] = uart_dat;
        wireless_uart_rx_head = next_head;
    }
}

uint8 wireless_uart_init(void)
{
    uint8 return_state = 0U;

    wireless_uart_rx_head = 0U;
    wireless_uart_rx_tail = 0U;
    wireless_uart_rx_dropped = 0U;

#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
    fifo_init(&wireless_uart_auto_baud_fifo, FIFO_DATA_8BIT,
            wireless_uart_auto_baud_buffer, WIRELESS_UART_BUFFER_SIZE);
#endif

#if ((1 == WIRELESS_UART_AUTO_BAUD_RATE) || (WIRELESS_UART_TRANSFER_MODE == WIRELESS_UART_TRANSFER_MODE_RTS))
    gpio_init(WIRELESS_UART_RTS_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
#endif

#if (0 == WIRELESS_UART_AUTO_BAUD_RATE)
    uart_init(WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);
    uart_rx_interrupt(WIRELESS_UART_INDEX, 1, wireless_uart_callback);
#elif (1 == WIRELESS_UART_AUTO_BAUD_RATE)
    uint8 rts_init_status;
    uint16 time_count = 0U;

    wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_INIT;
    wireless_auto_baud_data[0] = 0U;
    wireless_auto_baud_data[1] = 1U;
    wireless_auto_baud_data[2] = 3U;

    rts_init_status = gpio_get_level(WIRELESS_UART_RTS_PIN);
    gpio_init(WIRELESS_UART_RTS_PIN, GPO, rts_init_status, GPO_PUSH_PULL);

    uart_init(WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);
    uart_rx_interrupt(WIRELESS_UART_INDEX, 1, wireless_uart_callback);

    system_delay_ms(5);
    gpio_set_level(WIRELESS_UART_RTS_PIN, !rts_init_status);
    system_delay_ms(100);
    gpio_toggle_level(WIRELESS_UART_RTS_PIN);

    do
    {
        wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_START;
        uart_write_byte(WIRELESS_UART_INDEX, wireless_auto_baud_data[0]);
        uart_write_byte(WIRELESS_UART_INDEX, wireless_auto_baud_data[1]);
        uart_write_byte(WIRELESS_UART_INDEX, wireless_auto_baud_data[2]);
        system_delay_ms(20);

        if(WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK != wireless_auto_baud_flag)
        {
            return_state = 1U;
            break;
        }

        time_count = 0U;

        if((0xa5 != wireless_auto_baud_data[0]) &&
                (0xff != wireless_auto_baud_data[1]) &&
                (0xff != wireless_auto_baud_data[2]))
        {
            return_state = 1U;
            break;
        }

        wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_SUCCESS;
        gpio_init(WIRELESS_UART_RTS_PIN, GPI, 0, GPI_PULL_UP);
        system_delay_ms(10);
    }
    while(0);

    (void)time_count;
#endif

    return return_state;
}
