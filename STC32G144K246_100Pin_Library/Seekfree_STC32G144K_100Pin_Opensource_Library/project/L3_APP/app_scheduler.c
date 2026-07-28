#include "zf_common_headfile.h"
#include "app_scheduler.h"
#include "service_wireless_uart.h"

typedef struct
{
    void (*task_func)(void);
    uint16 period_ms;
    uint16 tick_count;
    uint8 priority;
    volatile uint8 ready;
    uint8 used;
} app_scheduler_task_t;

static app_scheduler_task_t app_scheduler_tasks[APP_SCHEDULER_TASK_MAX];

static void app_scheduler_tick(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     app 层任务调度器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_scheduler_init();
// 备注信息     使用 TIM2_PIT 作为 1ms 调度心跳，中断回调只更新任务状态，不执行任务函数
//-------------------------------------------------------------------------------------------------------------------
void app_scheduler_init(void)
{
    uint8 i;

    for(i = 0; i < APP_SCHEDULER_TASK_MAX; i++)
    {
        app_scheduler_tasks[i].task_func = NULL;
        app_scheduler_tasks[i].period_ms = 0;
        app_scheduler_tasks[i].tick_count = 0;
        app_scheduler_tasks[i].priority = 0;
        app_scheduler_tasks[i].ready = 0;
        app_scheduler_tasks[i].used = 0;
    }

    pit_ms_init(TIM2_PIT, 1, app_scheduler_tick);
    #if __DBGFLAG__
    printf(">>[app_scheduler_init]\r\n");
    wprint(">>[app_scheduler_init]\r\n");
    #endif
}

void app_scheduler_debug(void)
{
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     app 层任务注册
// 参数说明     id              任务编号，范围 0 - APP_SCHEDULER_TASK_MAX-1
// 参数说明     task_func       任务函数指针，任务内部不允许死延时
// 参数说明     priority        任务优先级，数值越大优先级越高
// 参数说明     period_ms       任务执行周期，单位 ms
// 返回参数     uint8           1：注册成功 0：注册失败
// 使用示例     app_scheduler_add(0, task_fast, 10, 10);
// 备注信息     任务重复注册会覆盖原任务配置
//-------------------------------------------------------------------------------------------------------------------
uint8 app_scheduler_add(uint8 id, void (*task_func)(void), uint8 priority, uint16 period_ms)
{
    uint8 ea_backup;

    if((APP_SCHEDULER_TASK_MAX <= id) || (NULL == task_func) || (0U == period_ms))
    {
        return 0;
    }

    ea_backup = EA;
    EA = 0;

    app_scheduler_tasks[id].task_func = task_func;
    app_scheduler_tasks[id].period_ms = period_ms;
    app_scheduler_tasks[id].tick_count = 0;
    app_scheduler_tasks[id].priority = priority;
    app_scheduler_tasks[id].ready = 0;
    app_scheduler_tasks[id].used = 1;

    EA = ea_backup;

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     app 层任务调度运行
// 参数说明     void
// 返回参数     void
// 使用示例     while(1) { app_scheduler_run(); }
// 备注信息     每次最多执行一个就绪任务，同优先级时任务编号小的先执行
//-------------------------------------------------------------------------------------------------------------------
void app_scheduler_run(void)
{
    uint8 i;
    uint8 selected = APP_SCHEDULER_TASK_MAX;
    uint8 selected_priority = 0;
    uint8 ea_backup;
    void (*task_func)(void);

    for(i = 0; i < APP_SCHEDULER_TASK_MAX; i++)
    {
        if((0U != app_scheduler_tasks[i].used) && (0U != app_scheduler_tasks[i].ready))
        {
            if((APP_SCHEDULER_TASK_MAX == selected) || (app_scheduler_tasks[i].priority > selected_priority))
            {
                selected = i;
                selected_priority = app_scheduler_tasks[i].priority;
            }
        }
    }

    if(APP_SCHEDULER_TASK_MAX == selected)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    app_scheduler_tasks[selected].ready = 0;
    task_func = app_scheduler_tasks[selected].task_func;
    EA = ea_backup;

    if(NULL != task_func)
    {
        task_func();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     app 层任务调度器 1ms 心跳
// 参数说明     void
// 返回参数     void
// 使用示例     pit_ms_init(TIM2_PIT, 1, app_scheduler_tick);
// 备注信息     此函数运行在 TIM2 中断中，只更新 tick_count 和 ready 标志
//-------------------------------------------------------------------------------------------------------------------
static void app_scheduler_tick(void)
{
    uint8 i;

    for(i = 0; i < APP_SCHEDULER_TASK_MAX; i++)
    {
        if((0U != app_scheduler_tasks[i].used) && (NULL != app_scheduler_tasks[i].task_func))
        {
            app_scheduler_tasks[i].tick_count++;

            if(app_scheduler_tasks[i].tick_count >= app_scheduler_tasks[i].period_ms)
            {
                app_scheduler_tasks[i].tick_count = 0;
                app_scheduler_tasks[i].ready = 1;
            }
        }
    }
}
