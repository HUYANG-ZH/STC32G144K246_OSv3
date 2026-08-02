#include "stc.h"
#include "iap.h"
#include "dfu.h"

/* Reflected CRC-32/ISO-HDLC nibble table, polynomial 0xEDB88320. */
static const DWORD crc32_nibble_table[16] =
{
    0x00000000UL, 0x1db71064UL, 0x3b6e20c8UL, 0x26d930acUL,
    0x76dc4190UL, 0x6b6b51f4UL, 0x4db26158UL, 0x5005713cUL,
    0xedb88320UL, 0xf00f9344UL, 0xd6d6a3e8UL, 0xcb61b38cUL,
    0x9b64c2b0UL, 0x86d3d2d4UL, 0xa00ae278UL, 0xbdbdf21cUL
};

static void iap_program_physical(DWORD physical, BYTE value)
{
    IAP_CMD = 2U;
    IAP_ADDRE = BYTE2(physical);
    IAP_ADDRH = BYTE1(physical);
    IAP_ADDRL = BYTE0(physical);
    IAP_DATA = value;
    IAP_TRIG = 0x5aU;
    IAP_TRIG = 0xa5U;
    _nop_();
    _nop_();
    _nop_();
    _nop_();
}

static BOOL iap_write_physical_checked(DWORD physical, BYTE value)
{
    iap_program_physical(physical, value);
    return (iap_read_byte(physical) == value);
}

static void iap_erase_physical(DWORD physical)
{
    IAP_CMD = 3U;
    IAP_ADDRE = BYTE2(physical);
    IAP_ADDRH = BYTE1(physical);
    IAP_ADDRL = BYTE0(physical);
    IAP_TRIG = 0x5aU;
    IAP_TRIG = 0xa5U;
    _nop_();
    _nop_();
    _nop_();
    _nop_();
}

static BOOL iap_erase_physical_checked(DWORD physical)
{
    WORD offset;

    iap_erase_physical(physical);
    for (offset = 0U; offset < (WORD)FLASH_PAGE_SIZE; offset++)
    {
        if (iap_read_byte(physical + (DWORD)offset) != 0xffU)
            return 0;
    }
    return 1;
}

void iap_init(void)
{
    IAP_CONTR = 0x80U;
    IAP_TPS = bDfuHotEntry ? IAP_TPS_HOT : IAP_TPS_COLD;
}

BOOL iap_check_addr(DWORD addr)
{
    /* Protect FC:3800-FC:47FF, including the dedicated metadata page. */
    return ((addr >= APP_LOGICAL_BEGIN) && (addr < AP_SIZE));
}

BYTE iap_read_byte(DWORD physical_addr)
{
    IAP_CMD = 1U;
    IAP_ADDRE = BYTE2(physical_addr);
    IAP_ADDRH = BYTE1(physical_addr);
    IAP_ADDRL = BYTE0(physical_addr);
    IAP_TRIG = 0x5aU;
    IAP_TRIG = 0xa5U;
    _nop_();
    _nop_();
    _nop_();
    _nop_();

    return IAP_DATA;
}

BOOL iap_program_block(DWORD addr, BYTE near *src_ptr, BYTE size)
{
    DWORD physical;

    if ((size == 0U) || !iap_check_addr(addr))
        return 0;
    if ((DWORD)size > (AP_SIZE - addr))
        return 0;

    physical = addr + IAP_BASE;
    while (size--)
    {
        /* The page is already erased. Sending dense chunks is faster; FF is a no-op. */
        if (*src_ptr != 0xffU)
            iap_program_physical(physical, *src_ptr);
        physical++;
        src_ptr++;

        /* Defensive WDT feed every 64 bytes: a 249-byte PROGRAM frame already
           fits the ~35ms window, but this keeps the margin large. */
        if ((size & 0x3fU) == 0U)
        {
            WDT_CONTR |= 0x10;
        }
    }
    return 1;
}

BOOL iap_erase_page(DWORD addr)
{
    if (!iap_check_addr(addr))
        return 0;
    if ((addr & (FLASH_PAGE_SIZE - 1UL)) != 0UL)
        return 0;
    if (addr > (AP_SIZE - FLASH_PAGE_SIZE))
        return 0;

    /* Final full-image CRC32 is the authoritative verification. */
    iap_erase_physical(addr + IAP_BASE);
    return 1;
}

BOOL iap_crc32(DWORD addr, DWORD length, DWORD *result)
{
    DWORD crc;
    BYTE value;

    if ((length == 0UL) || !iap_check_addr(addr))
        return 0;
    if (length > (AP_SIZE - addr))
        return 0;

    crc = 0xffffffffUL;
    addr += IAP_BASE;
    while (length--)
    {
        value = iap_read_byte(addr++);
        crc ^= (DWORD)value;
        crc = (crc >> 4) ^ crc32_nibble_table[(BYTE)(crc & 0x0fUL)];
        crc = (crc >> 4) ^ crc32_nibble_table[(BYTE)(crc & 0x0fUL)];

        /* Feed the WDT every 512 bytes (~0.5-2ms per page, well inside the
           ~35ms window at 120MHz).  This covers CRC32(0xa6),
           UPDATE_VERIFY(0xa9) and PAGE_CRC_TABLE(0xab), all of which call
           iap_crc32 and would otherwise reset the chip mid-calculation. */
        if ((length & 0x1ffUL) == 0UL)
        {
            WDT_CONTR |= 0x10;
        }
    }
    *result = crc ^ 0xffffffffUL;
    return 1;
}

BOOL iap_meta_erase(void)
{
    /* Metadata is tiny and safety-critical, so retain full erase verification. */
    return iap_erase_physical_checked(UPDATE_META_PHYSICAL);
}

BOOL iap_meta_write(BYTE offset, BYTE value)
{
    return iap_write_physical_checked(UPDATE_META_PHYSICAL + (DWORD)offset, value);
}

BYTE iap_meta_read(BYTE offset)
{
    return iap_read_byte(UPDATE_META_PHYSICAL + (DWORD)offset);
}
