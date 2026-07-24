#include "stc.h"
#include "uart.h"
#include "iap.h"
#include "update.h"
#include "dfu.h"

long xdata DfuFlag _at_ 0xfffc;
bit bDfuHotEntry;

/*
 * P3.3 force-entry (low = DFU). Safe during cold boot because FastBoot
 * runs with EA=0 (interrupts globally disabled), so the camera VSYNC
 * (INT1 on P3.3) never fires. The pin is only sampled once before the
 * APP ever executes.
 */
static void dfu_delay(void)
{
    volatile int i;
    for (i = 0; i < 30000; i++);
}

static DWORD get_dword(BYTE near *ptr)
{
    DWORD value;
    value = (DWORD)ptr[0] << 24;
    value |= (DWORD)ptr[1] << 16;
    value |= (DWORD)ptr[2] << 8;
    value |= (DWORD)ptr[3];
    return value;
}

static void put_dword(BYTE near *ptr, DWORD value)
{
    ptr[0] = BYTE3(value);
    ptr[1] = BYTE2(value);
    ptr[2] = BYTE1(value);
    ptr[3] = BYTE0(value);
}

void dfu_check(void)
{
    long retained_flag;

    EAXFR = 1;
    retained_flag = DfuFlag;
    bDfuHotEntry = (retained_flag == DFU_TAG);

    P3M1 &= (BYTE)~0x08U;
    P3PU |= 0x08U;
    dfu_delay();

    /* Needed here so a persistent interrupted-update marker can be read. */
    iap_init();

    /*
     * Cold-boot force entry: hold P3.3 low at power-on.
     * (EA=0 here, so the camera VSYNC on INT1/P3.3 never fires.)
     * Hot entry: APP wrote DfuTag then IAP_CONTR=0x28.
     * Recovery: interrupted update with IN_PROGRESS/VERIFIED state.
     */
    if ((DFU_FORCEPIN != 0) && !bDfuHotEntry && !update_recovery_pending())
    {
        P3M1 |= 0x08U;
        P3PU = 0x00U;
        IAP_CONTR = 0x20U;
        while (1);
    }

    P3M1 |= 0x08U;
    P3PU = 0x00U;
    DfuFlag = 0;
}

void dfu_events(void)
{
    BYTE cmd;
    DWORD addr;
    DWORD value32;
    DWORD image_size;
    DWORD image_crc;
    BYTE size;
    BYTE ret;
    BYTE near *ptr;
    BYTE status;
    BYTE update_state;

    if (!bUartRxReady)
        return;

    cmd = UartRxBuffer[1];
    addr = get_dword(&UartRxBuffer[2]);
    size = UartRxBuffer[6];
    ptr = &UartRxBuffer[7];
    status = STATUS_OK;
    ret = 0;

    switch (cmd)
    {
    case DFU_CMD_CONNECT:
        if (UartRxBuffer[0] != 1U) { status = STATUS_ERRORWRAP; break; }
        UartTxBuffer[0] = LDR_VERSION >> 8;
        UartTxBuffer[1] = LDR_VERSION;
        ret = 2;
        break;

    case DFU_CMD_READ:
#ifdef DEBUG
        ret = size;
        ptr = &UartInBuffer[0];
        while (size--)
            *ptr++ = iap_read_byte(addr++);
#else
        status = STATUS_ERRORCMD;
#endif
        break;

    case DFU_CMD_PROGRAM:
        if (!update_write_allowed()) { status = STATUS_STATEERR; break; }
        if (UartRxBuffer[0] != (BYTE)(6U + size))
        {
            status = STATUS_ERRORWRAP;
            break;
        }
        /* Fast dense block programming; final whole-image CRC32 is authoritative. */
        if (!iap_program_block(addr, ptr, size))
            status = STATUS_PROGRAMERR;
        break;

    case DFU_CMD_ERASE:
        if (!update_write_allowed()) { status = STATUS_STATEERR; break; }
        if (UartRxBuffer[0] != 1U) { status = STATUS_ERRORWRAP; break; }
        addr = APP_LOGICAL_BEGIN;
        while (addr < AP_SIZE)
        {
            if (!iap_erase_page(addr))
            {
                status = STATUS_ERASEERR;
                break;
            }
            addr += FLASH_PAGE_SIZE;
        }
        break;

    case DFU_CMD_ERASE_PAGE:
        if (!update_write_allowed()) { status = STATUS_STATEERR; break; }
        if (UartRxBuffer[0] != 6U) { status = STATUS_ERRORWRAP; break; }
        if (!iap_erase_page(addr))
            status = STATUS_ERASEERR;
        break;

    case DFU_CMD_CRC32:
        if (UartRxBuffer[0] != 9U) { status = STATUS_ERRORWRAP; break; }
        value32 = get_dword(&UartRxBuffer[6]);
        if (!iap_crc32(addr, value32, &image_crc))
        {
            status = STATUS_OUTOFRANGE;
            break;
        }
        put_dword(&UartTxBuffer[0], image_crc);
        ret = 4;
        break;

    case DFU_CMD_PAGE_CRC_TABLE:
        if (UartRxBuffer[0] != 6U) { status = STATUS_ERRORWRAP; break; }
        if ((size == 0U) || (size > 60U)) { status = STATUS_OUTOFRANGE; break; }
        if (!iap_check_addr(addr)) { status = STATUS_OUTOFRANGE; break; }
        if ((addr & (FLASH_PAGE_SIZE - 1UL)) != 0UL)
        {
            status = STATUS_OUTOFRANGE;
            break;
        }
        if ((DWORD)size > ((AP_SIZE - addr) / FLASH_PAGE_SIZE))
        {
            status = STATUS_OUTOFRANGE;
            break;
        }
        ret = 0U;
        while (size--)
        {
            if (!iap_crc32(addr, FLASH_PAGE_SIZE, &value32))
            {
                status = STATUS_OUTOFRANGE;
                ret = 0U;
                break;
            }
            put_dword(&UartTxBuffer[ret], value32);
            ret = (BYTE)(ret + 4U);
            addr += FLASH_PAGE_SIZE;
        }
        break;

    case DFU_CMD_STATUS:
        if (UartRxBuffer[0] != 1U) { status = STATUS_ERRORWRAP; break; }
        update_state = update_get_status(&image_size, &image_crc);
        UartTxBuffer[0] = 1U;
        UartTxBuffer[1] = update_state;
        put_dword(&UartTxBuffer[2], image_size);
        put_dword(&UartTxBuffer[6], image_crc);
        ret = 10;
        break;

    case DFU_CMD_UPDATE_BEGIN:
        if (UartRxBuffer[0] != 9U) { status = STATUS_ERRORWRAP; break; }
        image_size = get_dword(&UartRxBuffer[2]);
        image_crc = get_dword(&UartRxBuffer[6]);
        if (!update_begin(image_size, image_crc))
            status = STATUS_METAERR;
        break;

    case DFU_CMD_UPDATE_VERIFY:
        if (UartRxBuffer[0] != 1U) { status = STATUS_ERRORWRAP; break; }
        image_crc = 0UL;
        if (!update_verify(&image_crc))
            status = STATUS_CRCERR;
        put_dword(&UartTxBuffer[0], image_crc);
        ret = 4;
        break;

    case DFU_CMD_UPDATE_COMMIT:
        if (UartRxBuffer[0] != 1U) { status = STATUS_ERRORWRAP; break; }
        if (!update_commit())
            status = STATUS_STATEERR;
        break;

    case DFU_CMD_REBOOT:
        if (UartRxBuffer[0] != 1U) { status = STATUS_ERRORWRAP; break; }
        IAP_CONTR = 0x20U;
        while (1);
        break;

    default:
        status = STATUS_ERRORCMD;
        break;
    }

    uart_send(status, ret);
    uart_recv_done();
}
