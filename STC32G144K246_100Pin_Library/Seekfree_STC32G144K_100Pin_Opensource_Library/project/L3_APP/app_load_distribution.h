#ifndef APP_LOAD_DISTRIBUTION_H
#define APP_LOAD_DISTRIBUTION_H
#include "zf_common_typedef.h"
typedef struct
{
    // 输出数据
    float load_left;     // 左轮负载最终输出
    float load_right;    // 右轮负载最终输出
    
    // 输入数据
    float straight_current;   // 当前直线速度
    float turn_current;      // 当前转向速度
    float Target_differential_speed; // 目标差速
    float Target_Anglespeed;  // 目标角速度

    // 中间数据
    float ay; // 横向加速度
    float LTC; // 荷载转移比例系数
} app_load_distribution_data_t;

void app_load_distribution_init(void);
void app_load_distribution_process(app_load_distribution_data_t *datas);

#endif
