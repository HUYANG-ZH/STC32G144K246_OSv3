#include "zf_common_headfile.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"
#include "service_packet.h"

#define SERVICE_PACKET_RX_READ_SIZE       (32U)
#define SERVICE_PACKET_FRAME_MAX          (96U)
#define SERVICE_PACKET_TRIGGER_PRIORITY   (1U)

typedef struct
{
    const char *name;
    float *value;
    uint8 value_count;
} service_packet_variable_t;

typedef struct
{
    const char *name;
    service_packet_action_func_t func;
    uint32 delay_ms;
} service_packet_action_t;

typedef enum
{
    SERVICE_PACKET_RX_WAIT_HEAD_1 = 0,
    SERVICE_PACKET_RX_WAIT_HEAD_2,
    SERVICE_PACKET_RX_BODY,
    SERVICE_PACKET_RX_TAIL_2,
} service_packet_rx_state_t;

static service_packet_variable_t packet_variables[SERVICE_PACKET_VARIABLE_MAX];
static service_packet_action_t packet_actions[SERVICE_PACKET_ACTION_MAX];
static uint8 packet_variable_count = 0U;
static uint8 packet_action_count = 0U;

static service_packet_rx_state_t packet_rx_state = SERVICE_PACKET_RX_WAIT_HEAD_1;
static char packet_frame[SERVICE_PACKET_FRAME_MAX];
static uint8 packet_frame_index = 0U;

static uint8 service_packet_is_field_separator(char value)
{
    return ((',' == value) || (':' == value) || ('=' == value) ||
            (' ' == value) || ('\t' == value));
}

static uint8 service_packet_is_value_separator(char value)
{
    return ((',' == value) || (';' == value) || ('|' == value) ||
            (' ' == value) || ('\t' == value));
}

static char *service_packet_skip_field_separator(char *text)
{
    while(('\0' != *text) && (0U != service_packet_is_field_separator(*text)))
    {
        text++;
    }

    return text;
}

static char *service_packet_skip_value_separator(char *text)
{
    while(('\0' != *text) && (0U != service_packet_is_value_separator(*text)))
    {
        text++;
    }

    return text;
}

static uint8 service_packet_is_number_char(char value)
{
    return ((('0' <= value) && ('9' >= value)) || ('+' == value) ||
            ('-' == value) || ('.' == value));
}

static service_packet_variable_t *service_packet_find_variable(const char *name)
{
    uint8 i;

    for(i = 0; i < packet_variable_count; i++)
    {
        if(0 == strcmp(packet_variables[i].name, name))
        {
            return &packet_variables[i];
        }
    }

    return NULL;
}

static service_packet_action_t *service_packet_find_action(const char *name)
{
    uint8 i;

    for(i = 0; i < packet_action_count; i++)
    {
        if(0 == strcmp(packet_actions[i].name, name))
        {
            return &packet_actions[i];
        }
    }

    return NULL;
}

static uint8 service_packet_parse_float_list(char *text, float *out_value, uint8 value_count)
{
    uint8 i;
    uint8 digit_found;
    uint8 point_found;
    char *token_start;
    char delimiter;

    for(i = 0; i < value_count; i++)
    {
        text = service_packet_skip_value_separator(text);
        if('\0' == *text)
        {
            return 0U;
        }

        token_start = text;
        digit_found = 0U;
        point_found = 0U;

        if(('+' == *text) || ('-' == *text))
        {
            text++;
        }

        while(0U != service_packet_is_number_char(*text))
        {
            if(('.' == *text) && (0U != point_found))
            {
                return 0U;
            }
            else if('.' == *text)
            {
                point_found = 1U;
            }
            else if(('0' <= *text) && ('9' >= *text))
            {
                digit_found = 1U;
            }

            text++;
        }

        if(0U == digit_found)
        {
            return 0U;
        }

        if(('\0' != *text) && (0U == service_packet_is_value_separator(*text)))
        {
            return 0U;
        }

        delimiter = *text;
        *text = '\0';
        out_value[i] = func_str_to_float(token_start);

        if('\0' == delimiter)
        {
            return (i == (value_count - 1U));
        }

        text++;
    }

    text = service_packet_skip_value_separator(text);

    return ('\0' == *text);
}

static void service_packet_print_variable(service_packet_variable_t *variable)
{
    uint8 i;
    uint8 ea_backup;
    float value_copy[SERVICE_PACKET_VALUE_MAX];

    ea_backup = EA;
    EA = 0;
    for(i = 0; i < variable->value_count; i++)
    {
        value_copy[i] = variable->value[i];
    }
    EA = ea_backup;

    wprint("%s,", (char *)variable->name);

    for(i = 0; i < variable->value_count; i++)
    {
        if(0U != i)
        {
            wprint(",");
        }
        wprint("%.3f", (double)value_copy[i]);
    }

    wprint("\r\n");
}

static void service_packet_handle_read(char *name)
{
    service_packet_variable_t *variable;

    variable = service_packet_find_variable(name);
    if(NULL == variable)
    {
        return;
    }

    service_packet_print_variable(variable);
}

static void service_packet_handle_write(char *name, char *value_text)
{
    uint8 i;
    uint8 ea_backup;
    float value_copy[SERVICE_PACKET_VALUE_MAX];
    service_packet_variable_t *variable;

    variable = service_packet_find_variable(name);
    if((NULL == variable) || (NULL == value_text))
    {
        return;
    }

    if(0U == service_packet_parse_float_list(value_text, value_copy, variable->value_count))
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    for(i = 0; i < variable->value_count; i++)
    {
        variable->value[i] = value_copy[i];
    }
    EA = ea_backup;

    service_packet_print_variable(variable);
}

static void service_packet_handle_trigger(char *name)
{
    service_packet_action_t *action;

    action = service_packet_find_action(name);
    if(NULL == action)
    {
        return;
    }

    (void)service_function_queue_add(action->func, action->delay_ms, SERVICE_PACKET_TRIGGER_PRIORITY);
}

static void service_packet_handle_frame(char *frame)
{
    char command;
    char *name;
    char *value_text;

    if('\0' == frame[0])
    {
        return;
    }

    command = frame[0];
    name = service_packet_skip_field_separator(&frame[1]);
    value_text = name;

    while(('\0' != *value_text) && (0U == service_packet_is_field_separator(*value_text)))
    {
        value_text++;
    }

    if('\0' != *value_text)
    {
        *value_text = '\0';
        value_text++;
        value_text = service_packet_skip_field_separator(value_text);
    }
    else
    {
        value_text = NULL;
    }

    if('\0' == *name)
    {
        return;
    }

    if('R' == command)
    {
        service_packet_handle_read(name);
    }
    else if('W' == command)
    {
        service_packet_handle_write(name, value_text);
    }
    else if('T' == command)
    {
        service_packet_handle_trigger(name);
    }
}

static void service_packet_reset_rx(void)
{
    packet_rx_state = SERVICE_PACKET_RX_WAIT_HEAD_1;
    packet_frame_index = 0U;
}

static void service_packet_push_body_char(char value)
{
    if(packet_frame_index < (SERVICE_PACKET_FRAME_MAX - 1U))
    {
        packet_frame[packet_frame_index] = value;
        packet_frame_index++;
    }
    else
    {
        service_packet_reset_rx();
    }
}

static void service_packet_input_char(char value)
{
    if(SERVICE_PACKET_RX_WAIT_HEAD_1 == packet_rx_state)
    {
        if('$' == value)
        {
            packet_rx_state = SERVICE_PACKET_RX_WAIT_HEAD_2;
        }
        return;
    }

    if(SERVICE_PACKET_RX_WAIT_HEAD_2 == packet_rx_state)
    {
        if('$' == value)
        {
            packet_frame_index = 0U;
            packet_rx_state = SERVICE_PACKET_RX_BODY;
        }
        else
        {
            packet_rx_state = SERVICE_PACKET_RX_WAIT_HEAD_1;
        }
        return;
    }

    if(SERVICE_PACKET_RX_BODY == packet_rx_state)
    {
        if('@' == value)
        {
            packet_rx_state = SERVICE_PACKET_RX_TAIL_2;
        }
        else
        {
            service_packet_push_body_char(value);
        }
        return;
    }

    if(SERVICE_PACKET_RX_TAIL_2 == packet_rx_state)
    {
        if('@' == value)
        {
            packet_frame[packet_frame_index] = '\0';
            service_packet_handle_frame(packet_frame);
            service_packet_reset_rx();
        }
        else
        {
            service_packet_push_body_char('@');
            service_packet_push_body_char(value);
            packet_rx_state = SERVICE_PACKET_RX_BODY;
        }
    }
}

void service_packet_init(void)
{
    uint8 i;

    packet_variable_count = 0U;
    packet_action_count = 0U;
    service_packet_reset_rx();

    for(i = 0; i < SERVICE_PACKET_VARIABLE_MAX; i++)
    {
        packet_variables[i].name = NULL;
        packet_variables[i].value = NULL;
        packet_variables[i].value_count = 0U;
    }

    for(i = 0; i < SERVICE_PACKET_ACTION_MAX; i++)
    {
        packet_actions[i].name = NULL;
        packet_actions[i].func = NULL;
        packet_actions[i].delay_ms = 0U;
    }
}

uint8 service_packet_add_variable(const char *name, float *value, uint8 value_count)
{
    service_packet_variable_t *variable;

    if((NULL == name) || (NULL == value) || (0U == value_count) ||
            (SERVICE_PACKET_VALUE_MAX < value_count))
    {
        return 0U;
    }

    variable = service_packet_find_variable(name);
    if(NULL != variable)
    {
        variable->value = value;
        variable->value_count = value_count;
        return 1U;
    }

    if(SERVICE_PACKET_VARIABLE_MAX <= packet_variable_count)
    {
        return 0U;
    }

    packet_variables[packet_variable_count].name = name;
    packet_variables[packet_variable_count].value = value;
    packet_variables[packet_variable_count].value_count = value_count;
    packet_variable_count++;

    return 1U;
}

uint8 service_packet_add_action(const char *name, service_packet_action_func_t func, uint32 delay_ms)
{
    service_packet_action_t *action;

    if((NULL == name) || (NULL == func))
    {
        return 0U;
    }

    action = service_packet_find_action(name);
    if(NULL != action)
    {
        action->func = func;
        action->delay_ms = delay_ms;
        return 1U;
    }

    if(SERVICE_PACKET_ACTION_MAX <= packet_action_count)
    {
        return 0U;
    }

    packet_actions[packet_action_count].name = name;
    packet_actions[packet_action_count].func = func;
    packet_actions[packet_action_count].delay_ms = delay_ms;
    packet_action_count++;

    return 1U;
}

void service_packet_update(void)
{
    uint8 i;
    uint8 read_buffer[SERVICE_PACKET_RX_READ_SIZE];
    uint8 read_length;

    read_length = (uint8)service_wireless_uart_read_buffer(read_buffer, SERVICE_PACKET_RX_READ_SIZE);

    for(i = 0; i < read_length; i++)
    {
        service_packet_input_char((char)read_buffer[i]);
    }
}
