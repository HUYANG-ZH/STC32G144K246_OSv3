#include "zf_common_headfile.h"
#include "service_packet.h"
#include "service_wireless_uart.h"
#include "app_log.h"
#include <stdarg.h>

#define APP_LOG_REGION_START_ADDR           (0xFC2800UL)
#define APP_LOG_REGION_END_ADDR             (0xFC47FFUL)
#define APP_LOG_REGION_LIMIT_ADDR           (APP_LOG_REGION_END_ADDR + 1UL)
#define APP_LOG_PAGE_SIZE                   (512UL)
#define APP_LOG_PAGE_SIZE_U16               (512U)
#define APP_LOG_HEADER_ADDR                 (APP_LOG_REGION_START_ADDR)
#define APP_LOG_DATA_START_ADDR             (APP_LOG_REGION_START_ADDR + APP_LOG_PAGE_SIZE)
#define APP_LOG_DATA_SIZE                   ((uint16)(APP_LOG_REGION_END_ADDR - APP_LOG_DATA_START_ADDR + 1UL))

#define APP_LOG_MAGIC                       (0x574C4F47UL)
#define APP_LOG_VERSION                     (1U)
#define APP_LOG_EMPTY_BYTE                  (0xFFU)
#define APP_LOG_BUFFER_SIZE                 (128U)
#define APP_LOG_READ_BUFFER_SIZE            (64U)
#define APP_LOG_FLOAT_DEFAULT_PRECISION     (3U)
#define APP_LOG_FLOAT_MAX_PRECISION         (6U)

#define APP_LOG_HEADER_MAGIC_OFFSET         (0UL)
#define APP_LOG_HEADER_VERSION_OFFSET       (4UL)
#define APP_LOG_HEADER_WRITE_OFFSET         (6UL)
#define APP_LOG_HEADER_USED_OFFSET          (8UL)
#define APP_LOG_HEADER_SEQUENCE_OFFSET      (10UL)
#define APP_LOG_HEADER_CHECKSUM_OFFSET      (12UL)

static char app_log_buffer[APP_LOG_BUFFER_SIZE];
static uint8 app_log_read_buffer[APP_LOG_READ_BUFFER_SIZE];
static uint16 app_log_write_offset = 0U;
static uint16 app_log_used_size = 0U;
static uint16 app_log_sequence = 0U;
static uint8 app_log_ready = 0U;

static uint32 app_log_data_addr(uint16 offset)
{
    return APP_LOG_DATA_START_ADDR + (uint32)offset;
}

static uint16 app_log_read_u16(uint32 addr)
{
    uint16 value;

    value = (uint16)iap_read_byte(addr);
    value |= (uint16)((uint16)iap_read_byte(addr + 1UL) << 8);

    return value;
}

static uint32 app_log_read_u32(uint32 addr)
{
    uint32 value;

    value = (uint32)iap_read_byte(addr);
    value |= (uint32)iap_read_byte(addr + 1UL) << 8;
    value |= (uint32)iap_read_byte(addr + 2UL) << 16;
    value |= (uint32)iap_read_byte(addr + 3UL) << 24;

    return value;
}

static void app_log_write_u16(uint32 addr, uint16 value)
{
    iap_write_byte(addr, (uint8)value);
    iap_write_byte(addr + 1UL, (uint8)(value >> 8));
}

static void app_log_write_u32(uint32 addr, uint32 value)
{
    iap_write_byte(addr, (uint8)value);
    iap_write_byte(addr + 1UL, (uint8)(value >> 8));
    iap_write_byte(addr + 2UL, (uint8)(value >> 16));
    iap_write_byte(addr + 3UL, (uint8)(value >> 24));
}

static uint16 app_log_calc_checksum(uint32 magic, uint16 version, uint16 write_offset,
        uint16 used_size, uint16 sequence)
{
    uint16 checksum;

    checksum = 0xA55AU;
    checksum += (uint16)(magic & 0xFFUL);
    checksum += (uint16)((magic >> 8) & 0xFFUL);
    checksum += (uint16)((magic >> 16) & 0xFFUL);
    checksum += (uint16)((magic >> 24) & 0xFFUL);
    checksum += version;
    checksum += write_offset;
    checksum += used_size;
    checksum += sequence;

    return (uint16)(checksum ^ 0x5A5AU);
}

static void app_log_reset_state(void)
{
    app_log_write_offset = 0U;
    app_log_used_size = 0U;
    app_log_sequence = 0U;
}

static void app_log_load_header(void)
{
    uint32 magic;
    uint16 version;
    uint16 write_offset;
    uint16 used_size;
    uint16 sequence;
    uint16 checksum;
    uint16 real_checksum;

    magic = app_log_read_u32(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_MAGIC_OFFSET);
    version = app_log_read_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_VERSION_OFFSET);
    write_offset = app_log_read_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_WRITE_OFFSET);
    used_size = app_log_read_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_USED_OFFSET);
    sequence = app_log_read_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_SEQUENCE_OFFSET);
    checksum = app_log_read_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_CHECKSUM_OFFSET);
    real_checksum = app_log_calc_checksum(magic, version, write_offset, used_size, sequence);

    if((APP_LOG_MAGIC != magic) || (APP_LOG_VERSION != version)
            || (write_offset >= APP_LOG_DATA_SIZE) || (used_size > APP_LOG_DATA_SIZE)
            || (checksum != real_checksum))
    {
        app_log_reset_state();
        return;
    }

    app_log_write_offset = write_offset;
    app_log_used_size = used_size;
    app_log_sequence = sequence;
}

static void app_log_save_header(void)
{
    uint16 checksum;

    app_log_sequence++;
    checksum = app_log_calc_checksum(APP_LOG_MAGIC, APP_LOG_VERSION,
            app_log_write_offset, app_log_used_size, app_log_sequence);

    iap_erase_page(APP_LOG_HEADER_ADDR);
    app_log_write_u32(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_MAGIC_OFFSET, APP_LOG_MAGIC);
    app_log_write_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_VERSION_OFFSET, APP_LOG_VERSION);
    app_log_write_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_WRITE_OFFSET, app_log_write_offset);
    app_log_write_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_USED_OFFSET, app_log_used_size);
    app_log_write_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_SEQUENCE_OFFSET, app_log_sequence);
    app_log_write_u16(APP_LOG_HEADER_ADDR + APP_LOG_HEADER_CHECKSUM_OFFSET, checksum);
}

static void app_log_drop_reused_page(void)
{
    if(app_log_used_size > APP_LOG_PAGE_SIZE_U16)
    {
        app_log_used_size = (uint16)(app_log_used_size - APP_LOG_PAGE_SIZE_U16);
    }
    else
    {
        app_log_used_size = 0U;
    }
}

static void app_log_prepare_write_page(void)
{
    uint32 addr;

    if(0U != (app_log_write_offset % APP_LOG_PAGE_SIZE_U16))
    {
        return;
    }

    addr = app_log_data_addr(app_log_write_offset);
    if(APP_LOG_EMPTY_BYTE != iap_read_byte(addr))
    {
        iap_erase_page(addr);
        app_log_drop_reused_page();
    }
}

static void app_log_write_data_byte(uint8 value)
{
    app_log_prepare_write_page();
    iap_write_byte(app_log_data_addr(app_log_write_offset), value);

    app_log_write_offset++;
    if(app_log_write_offset >= APP_LOG_DATA_SIZE)
    {
        app_log_write_offset = 0U;
    }

    if(app_log_used_size < APP_LOG_DATA_SIZE)
    {
        app_log_used_size++;
    }
}

static void app_log_put_char(char *buffer, uint16 *index, char value)
{
    if(*index < (APP_LOG_BUFFER_SIZE - 1U))
    {
        buffer[*index] = value;
        (*index)++;
    }
}

static void app_log_put_string(char *buffer, uint16 *index, const char *text)
{
    if(NULL == text)
    {
        text = "(null)";
    }

    while('\0' != *text)
    {
        app_log_put_char(buffer, index, *text);
        text++;
    }
}

static void app_log_put_unsigned(char *buffer, uint16 *index, uint32 value, uint8 base, uint8 uppercase)
{
    char temp[11];
    uint8 temp_index = 0U;
    const char *digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    if(0UL == value)
    {
        app_log_put_char(buffer, index, '0');
        return;
    }

    while(0UL != value)
    {
        temp[temp_index] = digits[value % base];
        temp_index++;
        value /= base;
    }

    while(0U != temp_index)
    {
        temp_index--;
        app_log_put_char(buffer, index, temp[temp_index]);
    }
}

static void app_log_put_signed(char *buffer, uint16 *index, int32 value)
{
    uint32 abs_value;

    if(0 > value)
    {
        app_log_put_char(buffer, index, '-');
        abs_value = (uint32)(0 - value);
    }
    else
    {
        abs_value = (uint32)value;
    }

    app_log_put_unsigned(buffer, index, abs_value, 10U, 0U);
}

static uint32 app_log_get_scale(uint8 precision)
{
    uint32 scale = 1UL;

    while(0U != precision)
    {
        scale *= 10UL;
        precision--;
    }

    return scale;
}

static void app_log_put_float(char *buffer, uint16 *index, double value, uint8 precision)
{
    uint8 i;
    uint32 scale;
    uint32 integer_part;
    uint32 fraction_part;
    double fraction_value;
    uint32 divisor;

    if(precision > APP_LOG_FLOAT_MAX_PRECISION)
    {
        precision = APP_LOG_FLOAT_MAX_PRECISION;
    }

    if(0.0 > value)
    {
        app_log_put_char(buffer, index, '-');
        value = 0.0 - value;
    }

    integer_part = (uint32)value;
    scale = app_log_get_scale(precision);
    fraction_value = value - (double)integer_part;
    fraction_part = (uint32)((fraction_value * (double)scale) + 0.5);

    if(fraction_part >= scale)
    {
        integer_part++;
        fraction_part -= scale;
    }

    app_log_put_unsigned(buffer, index, integer_part, 10U, 0U);

    if(0U == precision)
    {
        return;
    }

    app_log_put_char(buffer, index, '.');
    divisor = scale / 10UL;

    for(i = 0U; i < precision; i++)
    {
        app_log_put_char(buffer, index, (char)('0' + (fraction_part / divisor)));
        fraction_part %= divisor;
        divisor /= 10UL;
    }
}

static uint16 app_log_format(char *buffer, const char *format, va_list args)
{
    char specifier;
    uint8 precision;
    uint16 index = 0U;

    while('\0' != *format)
    {
        if('%' != *format)
        {
            app_log_put_char(buffer, &index, *format);
            format++;
            continue;
        }

        format++;
        if('\0' == *format)
        {
            app_log_put_char(buffer, &index, '%');
            break;
        }

        precision = APP_LOG_FLOAT_DEFAULT_PRECISION;

        if('%' == *format)
        {
            app_log_put_char(buffer, &index, '%');
            format++;
            continue;
        }

        if('.' == *format)
        {
            format++;
            if(('0' <= *format) && ('9' >= *format))
            {
                precision = (uint8)(*format - '0');
                if(precision > APP_LOG_FLOAT_MAX_PRECISION)
                {
                    precision = APP_LOG_FLOAT_MAX_PRECISION;
                }
                format++;
            }
            else
            {
                app_log_put_char(buffer, &index, '%');
                app_log_put_char(buffer, &index, '.');
                continue;
            }
        }

        if('\0' == *format)
        {
            app_log_put_char(buffer, &index, '%');
            break;
        }

        specifier = *format;

        switch(specifier)
        {
            case 'c':
                app_log_put_char(buffer, &index, (char)va_arg(args, int));
                break;

            case 's':
                app_log_put_string(buffer, &index, va_arg(args, char *));
                break;

            case 'd':
            case 'i':
                app_log_put_signed(buffer, &index, (int32)va_arg(args, int));
                break;

            case 'u':
                app_log_put_unsigned(buffer, &index, (uint32)va_arg(args, unsigned int), 10U, 0U);
                break;

            case 'x':
                app_log_put_unsigned(buffer, &index, (uint32)va_arg(args, unsigned int), 16U, 0U);
                break;

            case 'X':
                app_log_put_unsigned(buffer, &index, (uint32)va_arg(args, unsigned int), 16U, 1U);
                break;

            case 'f':
                app_log_put_float(buffer, &index, va_arg(args, double), precision);
                break;

            default:
                app_log_put_char(buffer, &index, '%');
                app_log_put_char(buffer, &index, specifier);
                break;
        }

        if('\0' != *format)
        {
            format++;
        }
    }

    buffer[index] = '\0';
    return index;
}

void app_log_init(void)
{
    iap_init();
    app_log_load_header();
    app_log_ready = 1U;

    (void)service_packet_add_action("Rlog", Rlog, 0UL);
    (void)service_packet_add_action("Clog", Clog, 0UL);
}

uint32 Wlog(const char *format, ...)
{
    uint16 length;
    uint16 i;
    va_list args;

    if((NULL == format) || (0U == app_log_ready))
    {
        return 0UL;
    }

    va_start(args, format);
    length = app_log_format(app_log_buffer, format, args);
    va_end(args);

    for(i = 0U; i < length; i++)
    {
        app_log_write_data_byte((uint8)app_log_buffer[i]);
    }

    if(0U != length)
    {
        app_log_save_header();
    }

    return (uint32)length;
}

void Rlog(void)
{
    uint16 remaining;
    uint16 read_offset;
    uint16 chunk_len;
    uint16 linear_left;
    uint16 i;

    if((0U == app_log_ready) || (0U == app_log_used_size))
    {
        return;
    }

    remaining = app_log_used_size;
    read_offset = (uint16)((app_log_write_offset + APP_LOG_DATA_SIZE - app_log_used_size) % APP_LOG_DATA_SIZE);

    while(0U != remaining)
    {
        chunk_len = APP_LOG_READ_BUFFER_SIZE;
        if(chunk_len > remaining)
        {
            chunk_len = remaining;
        }

        linear_left = (uint16)(APP_LOG_DATA_SIZE - read_offset);
        if(chunk_len > linear_left)
        {
            chunk_len = linear_left;
        }

        for(i = 0U; i < chunk_len; i++)
        {
            app_log_read_buffer[i] = iap_read_byte(app_log_data_addr(read_offset));
            read_offset++;
        }

        if(read_offset >= APP_LOG_DATA_SIZE)
        {
            read_offset = 0U;
        }

        (void)service_wireless_uart_send_buffer(app_log_read_buffer, (uint32)chunk_len);
        remaining = (uint16)(remaining - chunk_len);
    }
}

void Clog(void)
{
    uint32 addr;

    if(0U == app_log_ready)
    {
        iap_init();
        app_log_ready = 1U;
    }

    for(addr = APP_LOG_REGION_START_ADDR; addr < APP_LOG_REGION_LIMIT_ADDR; addr += APP_LOG_PAGE_SIZE)
    {
        iap_erase_page(addr);
    }

    app_log_reset_state();
}
