#include "zf_common_headfile.h"
#include "service_wireless_uart.h"
#include <stdarg.h>

#define WPRINT_BUFFER_SIZE              (128U)
#define WPRINT_FLOAT_DEFAULT_PRECISION  (3U)
#define WPRINT_FLOAT_MAX_PRECISION      (6U)

static char wprint_buffer[WPRINT_BUFFER_SIZE];

static void wprint_put_char(char *buffer, uint16 *index, char value)
{
    if(*index < (WPRINT_BUFFER_SIZE - 1U))
    {
        buffer[*index] = value;
        (*index)++;
    }
}

static void wprint_put_string(char *buffer, uint16 *index, const char *text)
{
    if(NULL == text)
    {
        text = "(null)";
    }

    while('\0' != *text)
    {
        wprint_put_char(buffer, index, *text);
        text++;
    }
}

static void wprint_put_unsigned(char *buffer, uint16 *index, uint32 value, uint8 base, uint8 uppercase)
{
    char temp[11];
    uint8 temp_index = 0;
    const char *digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    if(0U == value)
    {
        wprint_put_char(buffer, index, '0');
        return;
    }

    while(0U != value)
    {
        temp[temp_index] = digits[value % base];
        temp_index++;
        value /= base;
    }

    while(0U != temp_index)
    {
        temp_index--;
        wprint_put_char(buffer, index, temp[temp_index]);
    }
}

static void wprint_put_signed(char *buffer, uint16 *index, int32 value)
{
    uint32 abs_value;

    if(0 > value)
    {
        wprint_put_char(buffer, index, '-');
        abs_value = (uint32)(0 - value);
    }
    else
    {
        abs_value = (uint32)value;
    }

    wprint_put_unsigned(buffer, index, abs_value, 10U, 0U);
}

static uint32 wprint_get_scale(uint8 precision)
{
    uint32 scale = 1U;

    while(0U != precision)
    {
        scale *= 10U;
        precision--;
    }

    return scale;
}

static void wprint_put_float(char *buffer, uint16 *index, double value, uint8 precision)
{
    uint8 i;
    uint32 scale;
    uint32 integer_part;
    uint32 fraction_part;
    double fraction_value;
    uint32 divisor;

    if(precision > WPRINT_FLOAT_MAX_PRECISION)
    {
        precision = WPRINT_FLOAT_MAX_PRECISION;
    }

    if(0.0 > value)
    {
        wprint_put_char(buffer, index, '-');
        value = 0.0 - value;
    }

    integer_part = (uint32)value;
    scale = wprint_get_scale(precision);
    fraction_value = value - (double)integer_part;
    fraction_part = (uint32)((fraction_value * (double)scale) + 0.5);

    if(fraction_part >= scale)
    {
        integer_part++;
        fraction_part -= scale;
    }

    wprint_put_unsigned(buffer, index, integer_part, 10U, 0U);

    if(0U == precision)
    {
        return;
    }

    wprint_put_char(buffer, index, '.');
    divisor = scale / 10U;

    for(i = 0; i < precision; i++)
    {
        wprint_put_char(buffer, index, (char)('0' + (fraction_part / divisor)));
        fraction_part %= divisor;
        divisor /= 10U;
    }
}

static uint16 wprint_format(char *buffer, const char *format, va_list args)
{
    char specifier;
    uint8 precision;
    uint16 index = 0;

    while('\0' != *format)
    {
        if('%' != *format)
        {
            wprint_put_char(buffer, &index, *format);
            format++;
            continue;
        }

        format++;
        if('\0' == *format)
        {
            wprint_put_char(buffer, &index, '%');
            break;
        }

        precision = WPRINT_FLOAT_DEFAULT_PRECISION;

        if('%' == *format)
        {
            wprint_put_char(buffer, &index, '%');
            format++;
            continue;
        }

        if('.' == *format)
        {
            format++;
            if(('0' <= *format) && ('9' >= *format))
            {
                precision = (uint8)(*format - '0');
                if(precision > WPRINT_FLOAT_MAX_PRECISION)
                {
                    precision = WPRINT_FLOAT_MAX_PRECISION;
                }
                format++;
            }
            else
            {
                wprint_put_char(buffer, &index, '%');
                wprint_put_char(buffer, &index, '.');
                continue;
            }
        }

        if('\0' == *format)
        {
            wprint_put_char(buffer, &index, '%');
            break;
        }

        specifier = *format;

        switch(specifier)
        {
            case 'c':
                wprint_put_char(buffer, &index, (char)va_arg(args, int));
                break;

            case 's':
                wprint_put_string(buffer, &index, va_arg(args, char *));
                break;

            case 'd':
            case 'i':
                wprint_put_signed(buffer, &index, (int32)va_arg(args, int));
                break;

            case 'u':
                wprint_put_unsigned(buffer, &index, (uint32)va_arg(args, unsigned int), 10U, 0U);
                break;

            case 'x':
                wprint_put_unsigned(buffer, &index, (uint32)va_arg(args, unsigned int), 16U, 0U);
                break;

            case 'X':
                wprint_put_unsigned(buffer, &index, (uint32)va_arg(args, unsigned int), 16U, 1U);
                break;

            case 'f':
                wprint_put_float(buffer, &index, va_arg(args, double), precision);
                break;

            default:
                wprint_put_char(buffer, &index, '%');
                wprint_put_char(buffer, &index, specifier);
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

void service_wireless_uart_init(void)
{
    wireless_uart_init();
}

void service_wireless_uart_debug(void)
{
    printf("[wireless_uart:send start.]\r\n");
    wprint("[wireless_uart:] success.\r\n");
    printf("[wireless_uart:send end.]\r\n");
}

uint32 service_wireless_uart_read_buffer(uint8 *buff, uint32 len)
{
    if(NULL == buff)
    {
        return 0U;
    }

    return wireless_uart_read_buffer(buff, len);
}

uint32 wprint(const char *format, ...)
{
    uint16 length;
    va_list args;

    if(NULL == format)
    {
        return 0U;
    }

    va_start(args, format);
    length = wprint_format(wprint_buffer, format, args);
    va_end(args);

    return wireless_uart_send_buffer((const uint8 *)wprint_buffer, (uint32)length);
}
