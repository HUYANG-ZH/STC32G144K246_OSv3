#include "zf_common_headfile.h"
#include "service_timetick.h"
#include "service_function_queue.h"
#include "service_wireless_uart.h"

#pragma warning disable = 150

#define SERVICE_FUNCTION_QUEUE_TICK_PER_MS     (10UL)
#define SERVICE_FUNCTION_QUEUE_PRIORITY_MIN    (1U)
#define SERVICE_FUNCTION_QUEUE_PRIORITY_MAX    (4U)

typedef struct
{
    service_function_queue_func_t func;
    uint32 start_tick;
    uint32 delay_tick;
    uint32 order;
    uint8 priority;
    uint8 used;
} service_function_queue_item_t;

static service_function_queue_item_t function_queue[SERVICE_FUNCTION_QUEUE_MAX];
static uint32 function_queue_order = 0U;

static uint8 service_function_queue_time_ready(uint32 now, service_function_queue_item_t *item)
{
    return ((uint32)(now - item->start_tick) >= item->delay_tick);
}

void service_function_queue_init(void)
{
    uint8 i;
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;

    for(i = 0; i < SERVICE_FUNCTION_QUEUE_MAX; i++)
    {
        function_queue[i].func = NULL;
        function_queue[i].start_tick = 0U;
        function_queue[i].delay_tick = 0U;
        function_queue[i].order = 0U;
        function_queue[i].priority = 0U;
        function_queue[i].used = 0U;
    }

    function_queue_order = 0U;

    EA = ea_backup;
    #if __DBGFLAG__
    printf(">>[service_function_queue_init]\r\n");
    wprint(">>[service_function_queue_init]\r\n");
    #endif
}

void service_function_queue_debug(void)
{
}

uint8 service_function_queue_add(service_function_queue_func_t func, uint32 delay_ms, uint8 priority)
{
    uint8 i;
    uint8 ea_backup;
    uint32 now;

    if((NULL == func) || (SERVICE_FUNCTION_QUEUE_PRIORITY_MIN > priority) ||
            (SERVICE_FUNCTION_QUEUE_PRIORITY_MAX < priority))
    {
        return 0U;
    }

    now = service_timetick_what();

    ea_backup = EA;
    EA = 0;

    for(i = 0; i < SERVICE_FUNCTION_QUEUE_MAX; i++)
    {
        if(0U == function_queue[i].used)
        {
            function_queue[i].func = func;
            function_queue[i].start_tick = now;
            function_queue[i].delay_tick = delay_ms * SERVICE_FUNCTION_QUEUE_TICK_PER_MS;
            function_queue[i].order = function_queue_order++;
            function_queue[i].priority = priority;
            function_queue[i].used = 1U;

            EA = ea_backup;
            return 1U;
        }
    }

    EA = ea_backup;
    return 0U;
}

void service_function_queue_update(void)
{
    uint8 priority;
    uint8 i;
    uint8 selected;
    uint8 ea_backup;
    uint32 now;
    uint32 update_order_limit;
    uint32 selected_order;
    service_function_queue_func_t selected_func;
    uint8 executed = 0U;

    now = service_timetick_what();

    ea_backup = EA;
    EA = 0;
    update_order_limit = function_queue_order;
    EA = ea_backup;

    for(priority = SERVICE_FUNCTION_QUEUE_PRIORITY_MAX; priority >= SERVICE_FUNCTION_QUEUE_PRIORITY_MIN; priority--)
    {
        while(1)
        {
            selected = SERVICE_FUNCTION_QUEUE_MAX;
            selected_order = update_order_limit;
            selected_func = NULL;

            ea_backup = EA;
            EA = 0;

            for(i = 0; i < SERVICE_FUNCTION_QUEUE_MAX; i++)
            {
                if((0U != function_queue[i].used) &&
                        (priority == function_queue[i].priority) &&
                        (function_queue[i].order < update_order_limit) &&
                        service_function_queue_time_ready(now, &function_queue[i]) &&
                        (function_queue[i].order < selected_order))
                {
                    selected = i;
                    selected_order = function_queue[i].order;
                    selected_func = function_queue[i].func;
                }
            }

            if(SERVICE_FUNCTION_QUEUE_MAX != selected)
            {
                function_queue[selected].func = NULL;
                function_queue[selected].used = 0U;
            }

            EA = ea_backup;

            if(NULL == selected_func)
            {
                break;
            }

            selected_func();
            executed++;
            if(executed >= SERVICE_FUNCTION_QUEUE_MAX_EXECUTE_PER_UPDATE)
            {
                return;
            }
        }
    }
}
