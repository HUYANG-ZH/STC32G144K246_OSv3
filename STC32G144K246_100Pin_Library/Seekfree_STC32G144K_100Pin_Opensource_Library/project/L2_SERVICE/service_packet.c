#include "zf_common_headfile.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"
#include "service_packet.h"

#define SERVICE_PACKET_RX_READ_SIZE       (64U)
#define SERVICE_PACKET_FRAME_MAX          (96U)
#define SERVICE_PACKET_TX_FRAME_MAX       (128U)
#define SERVICE_PACKET_NAME_SNAPSHOT_MAX  (32U)
#define SERVICE_PACKET_TRIGGER_PRIORITY   (1U)
#define SERVICE_PACKET_REPLY_FLOAT_SCALE  (1000.0f)
#define SERVICE_PACKET_REPLY_FLOAT_BASE   (1000UL)

typedef struct
{
    const char *name;
    float *value;
    uint8 value_count;
    service_packet_write_callback_t callback;
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
static char packet_reply[SERVICE_PACKET_TX_FRAME_MAX];
static uint8 packet_frame_index = 0U;
static volatile uint32 packet_rx_byte_total = 0U;
static volatile uint32 packet_frame_total = 0U;
static volatile uint32 packet_read_total = 0U;
static volatile uint32 packet_read_miss_total = 0U;
static volatile uint32 packet_write_total = 0U;
static volatile uint32 packet_write_ok_total = 0U;
static volatile uint32 packet_write_miss_total = 0U;
static volatile uint32 packet_write_parse_fail_total = 0U;
static char packet_last_miss_name[SERVICE_PACKET_NAME_SNAPSHOT_MAX];

static void service_packet_save_miss_name(const char *name);
static void service_packet_append_char(char *buffer, uint8 *index, char value);
static void service_packet_append_string(char *buffer, uint8 *index, const char *text);
static void service_packet_append_uint32(char *buffer, uint8 *index, uint32 value);
static void service_packet_append_float3(char *buffer, uint8 *index, float value);
static uint8 service_packet_value_changed(service_packet_variable_t *variable, const float *value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断字符是否为帧字段分隔符
// 参数说明     value           待判断字符
// 返回参数     uint8           1：是字段分隔符 0：不是字段分隔符
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static uint8 service_packet_is_field_separator(char value)
{
    return ((',' == value) || (':' == value) || ('=' == value) ||
            (' ' == value) || ('\t' == value));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断字符是否为数值列表分隔符
// 参数说明     value           待判断字符
// 返回参数     uint8           1：是数值分隔符 0：不是数值分隔符
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static uint8 service_packet_is_value_separator(char value)
{
    return ((',' == value) || (';' == value) || ('|' == value) ||
            (' ' == value) || ('\t' == value));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跳过字段分隔符
// 参数说明     text            字符串当前位置
// 返回参数     char *          跳过分隔符后的字符串位置
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static char *service_packet_skip_field_separator(char *text)
{
    while(('\0' != *text) && (0U != service_packet_is_field_separator(*text)))
    {
        text++;
    }

    return text;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跳过数值列表分隔符
// 参数说明     text            字符串当前位置
// 返回参数     char *          跳过分隔符后的字符串位置
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static char *service_packet_skip_value_separator(char *text)
{
    while(('\0' != *text) && (0U != service_packet_is_value_separator(*text)))
    {
        text++;
    }

    return text;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断字符是否为浮点数字符
// 参数说明     value           待判断字符
// 返回参数     uint8           1：是浮点数字符 0：不是浮点数字符
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static uint8 service_packet_is_number_char(char value)
{
    return ((('0' <= value) && ('9' >= value)) || ('+' == value) ||
            ('-' == value) || ('.' == value));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据变量名查找变量映射
// 参数说明     name            变量名称
// 返回参数     service_packet_variable_t * 变量映射指针 NULL：未找到
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据操作名查找触发动作映射
// 参数说明     name            操作名称
// 返回参数     service_packet_action_t *   动作映射指针 NULL：未找到
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
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

static void service_packet_save_miss_name(const char *name)
{
    uint8 i = 0U;

    if(NULL == name)
    {
        packet_last_miss_name[0] = '\0';
        return;
    }

    while(('\0' != name[i]) && (i < (SERVICE_PACKET_NAME_SNAPSHOT_MAX - 1U)))
    {
        packet_last_miss_name[i] = name[i];
        i++;
    }

    packet_last_miss_name[i] = '\0';
}

static void service_packet_append_char(char *buffer, uint8 *index, char value)
{
    if(*index < (SERVICE_PACKET_TX_FRAME_MAX - 1U))
    {
        buffer[*index] = value;
        (*index)++;
    }
}

static void service_packet_append_string(char *buffer, uint8 *index, const char *text)
{
    if(NULL == text)
    {
        return;
    }

    while('\0' != *text)
    {
        service_packet_append_char(buffer, index, *text);
        text++;
    }
}

static void service_packet_append_uint32(char *buffer, uint8 *index, uint32 value)
{
    char temp[10];
    uint8 temp_index = 0U;

    if(0UL == value)
    {
        service_packet_append_char(buffer, index, '0');
        return;
    }

    while((0UL != value) && (temp_index < sizeof(temp)))
    {
        temp[temp_index] = (char)('0' + (value % 10UL));
        value /= 10UL;
        temp_index++;
    }

    while(0U != temp_index)
    {
        temp_index--;
        service_packet_append_char(buffer, index, temp[temp_index]);
    }
}

static void service_packet_append_float3(char *buffer, uint8 *index, float value)
{
    uint32 scaled;
    uint32 integer_part;
    uint32 fraction_part;

    if(0.0f > value)
    {
        service_packet_append_char(buffer, index, '-');
        value = 0.0f - value;
    }

    scaled = (uint32)((value * SERVICE_PACKET_REPLY_FLOAT_SCALE) + 0.5f);
    integer_part = scaled / SERVICE_PACKET_REPLY_FLOAT_BASE;
    fraction_part = scaled % SERVICE_PACKET_REPLY_FLOAT_BASE;

    service_packet_append_uint32(buffer, index, integer_part);
    service_packet_append_char(buffer, index, '.');
    service_packet_append_char(buffer, index, (char)('0' + (fraction_part / 100UL)));
    fraction_part %= 100UL;
    service_packet_append_char(buffer, index, (char)('0' + (fraction_part / 10UL)));
    service_packet_append_char(buffer, index, (char)('0' + (fraction_part % 10UL)));
}

static uint8 service_packet_value_changed(service_packet_variable_t *variable, const float *value)
{
    uint8 i;

    if((NULL == variable) || (NULL == value))
    {
        return 0U;
    }

    for(i = 0U; i < variable->value_count; i++)
    {
        if(variable->value[i] != value[i])
        {
            return 1U;
        }
    }

    return 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析浮点数列表
// 参数说明     text            数值字符串
// 参数说明     out_value       解析结果保存地址
// 参数说明     value_count     需要解析的数值个数
// 返回参数     uint8           1：解析成功 0：解析失败
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过无线串口输出变量名称和当前数值
// 参数说明     variable        变量映射指针
// 返回参数     void
// 使用示例     内部调用，用户无需关心
// 备注信息     输出格式为 name,value0,value1...\r\n
//-------------------------------------------------------------------------------------------------------------------
static void service_packet_print_variable(service_packet_variable_t *variable)
{
    uint8 i;
    uint8 index = 0U;
    float value_copy[SERVICE_PACKET_VALUE_MAX];

    for(i = 0; i < variable->value_count; i++)
    {
        value_copy[i] = variable->value[i];
    }

    service_packet_append_string(packet_reply, &index, variable->name);
    service_packet_append_char(packet_reply, &index, ',');
    for(i = 0; i < variable->value_count; i++)
    {
        if(0U != i)
        {
            service_packet_append_char(packet_reply, &index, ',');
        }
        service_packet_append_float3(packet_reply, &index, value_copy[i]);
    }

    service_packet_append_string(packet_reply, &index, "\r\n");
    service_wireless_uart_send_buffer((const uint8 *)packet_reply, index);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理读取指令
// 参数说明     name            变量名称
// 返回参数     void
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void service_packet_handle_read(char *name)
{
    service_packet_variable_t *variable;

    packet_read_total++;
    variable = service_packet_find_variable(name);
    if(NULL == variable)
    {
        packet_read_miss_total++;
        service_packet_save_miss_name(name);
        return;
    }

    service_packet_print_variable(variable);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理写入指令
// 参数说明     name            变量名称
// 参数说明     value_text      待写入的数值字符串
// 返回参数     void
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void service_packet_handle_write(char *name, char *value_text)
{
    uint8 i;
    uint8 changed;
    float value_copy[SERVICE_PACKET_VALUE_MAX];
    service_packet_variable_t *variable;

    packet_write_total++;
    variable = service_packet_find_variable(name);
    if((NULL == variable) || (NULL == value_text))
    {
        packet_write_miss_total++;
        service_packet_save_miss_name(name);
        return;
    }

    if(0U == service_packet_parse_float_list(value_text, value_copy, variable->value_count))
    {
        packet_write_parse_fail_total++;
        return;
    }

    changed = service_packet_value_changed(variable, value_copy);
    for(i = 0; i < variable->value_count; i++)
    {
        variable->value[i] = value_copy[i];
    }

    packet_write_ok_total++;
    if((0U != changed) && (NULL != variable->callback))
    {
        variable->callback();
    }
    service_packet_print_variable(variable);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理触发指令
// 参数说明     name            操作名称
// 返回参数     void
// 使用示例     内部调用，用户无需关心
// 备注信息     匹配成功后将动作加入 service_function_queue，优先级为 1
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并处理一帧完整数据
// 参数说明     frame           去除帧头帧尾后的帧内容
// 返回参数     void
// 使用示例     内部调用，用户无需关心
// 备注信息     帧格式为 $$命令,名称,数值@@，命令支持 W/R/T
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位接收状态机
// 参数说明     void
// 返回参数     void
// 使用示例     内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void service_packet_reset_rx(void)
{
    packet_rx_state = SERVICE_PACKET_RX_WAIT_HEAD_1;
    packet_frame_index = 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向帧缓存写入一个字符
// 参数说明     value           待写入字符
// 返回参数     void
// 使用示例     内部调用，用户无需关心
// 备注信息     缓存溢出时自动复位接收状态机
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     输入一个接收字符并推进帧状态机
// 参数说明     value           接收到的字符
// 返回参数     void
// 使用示例     内部调用，用户无需关心
// 备注信息     检测到完整 $$...@@ 帧后自动解析执行
//-------------------------------------------------------------------------------------------------------------------
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
            packet_frame[packet_frame_index] = '\0';
            packet_frame_total++;
            service_packet_handle_frame(packet_frame);
            service_packet_reset_rx();
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
            packet_frame_total++;
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通讯包服务初始化
// 参数说明     void
// 返回参数     void
// 使用示例     service_packet_init();
// 备注信息     初始化后需要通过 service_packet_add_variable/action 注册变量和动作
//-------------------------------------------------------------------------------------------------------------------
void service_packet_init(void)
{
    uint8 i;

    packet_variable_count = 0U;
    packet_action_count = 0U;
    packet_rx_byte_total = 0U;
    packet_frame_total = 0U;
    packet_read_total = 0U;
    packet_read_miss_total = 0U;
    packet_write_total = 0U;
    packet_write_ok_total = 0U;
    packet_write_miss_total = 0U;
    packet_write_parse_fail_total = 0U;
    packet_last_miss_name[0] = '\0';
    service_packet_reset_rx();

    for(i = 0; i < SERVICE_PACKET_VARIABLE_MAX; i++)
    {
        packet_variables[i].name = NULL;
        packet_variables[i].value = NULL;
        packet_variables[i].value_count = 0U;
        packet_variables[i].callback = (service_packet_write_callback_t)0;
    }

    for(i = 0; i < SERVICE_PACKET_ACTION_MAX; i++)
    {
        packet_actions[i].name = NULL;
        packet_actions[i].func = (service_packet_action_func_t)0;
        packet_actions[i].delay_ms = 0U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通讯包服务调试接口
// 参数说明     void
// 返回参数     void
// 使用示例     service_packet_debug();
//-------------------------------------------------------------------------------------------------------------------
void service_packet_debug(void)
{
    printf("[packet:byte=%ld,frame=%ld]\r\n", packet_rx_byte_total, packet_frame_total);
    printf("[packet:R=%ld/%ld]\r\n", packet_read_total, packet_read_miss_total);
    printf("[packet:W=%ld/%ld]\r\n", packet_write_total, packet_write_ok_total);
    printf("[packet:miss=%ld,parse=%ld]\r\n", packet_write_miss_total, packet_write_parse_fail_total);
    printf("[packet:var=%ld,state=%ld]\r\n", (uint32)packet_variable_count, (uint32)packet_rx_state);
    printf("[packet:index=%ld,last=%s]\r\n", (uint32)packet_frame_index, packet_last_miss_name);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     注册可读写变量
// 参数说明     name            变量名称
// 参数说明     value           变量数据地址
// 参数说明     value_count     变量包含的 float 数量
// 返回参数     uint8           1：注册成功 0：注册失败
// 使用示例     service_packet_add_variable("speed", speed_value, 2);
// 备注信息     相同名称重复注册时会更新变量地址和数量
//-------------------------------------------------------------------------------------------------------------------
uint8 service_packet_add_variable(const char *name, float *value, uint8 value_count)
{
    return service_packet_add_variable_with_callback(name, value, value_count, (service_packet_write_callback_t)0);
}

uint8 service_packet_add_variable_with_callback(const char *name, float *value, uint8 value_count,
        service_packet_write_callback_t callback)
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
        variable->callback = callback;
        return 1U;
    }

    if(SERVICE_PACKET_VARIABLE_MAX <= packet_variable_count)
    {
        return 0U;
    }

    packet_variables[packet_variable_count].name = name;
    packet_variables[packet_variable_count].value = value;
    packet_variables[packet_variable_count].value_count = value_count;
    packet_variables[packet_variable_count].callback = callback;
    packet_variable_count++;

    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     注册触发动作
// 参数说明     name            操作名称
// 参数说明     func            触发动作函数
// 参数说明     delay_ms        触发后延迟执行时间 单位 ms
// 返回参数     uint8           1：注册成功 0：注册失败
// 使用示例     service_packet_add_action("reset", service_motor_reset, 0);
// 备注信息     相同名称重复注册时会更新函数和延迟时间
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通讯包服务周期更新
// 参数说明     void
// 返回参数     void
// 使用示例     service_packet_update();
// 备注信息     从无线串口读取数据并送入帧状态机，建议在主循环中周期调用
//-------------------------------------------------------------------------------------------------------------------
void service_packet_update(void)
{
    uint8 i;
    uint8 read_buffer[SERVICE_PACKET_RX_READ_SIZE];
    uint8 read_length;

    do
    {
        read_length = (uint8)service_wireless_uart_read_buffer(read_buffer, SERVICE_PACKET_RX_READ_SIZE);
        packet_rx_byte_total += read_length;

        for(i = 0; i < read_length; i++)
        {
            service_packet_input_char((char)read_buffer[i]);
        }
    }
    while(SERVICE_PACKET_RX_READ_SIZE == read_length);
}
