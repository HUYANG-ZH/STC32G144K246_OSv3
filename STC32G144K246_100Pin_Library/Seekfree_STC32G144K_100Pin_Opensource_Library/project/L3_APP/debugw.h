#ifndef DEBUGW_H
#define DEBUGW_H

#include "zf_common_typedef.h"

/* 无线调试模块: 由无线变量 debuginfo(0/1) 控制是否输出调试数据。
   复杂/连续的调试输出统一放在本模块, 按需在此扩展。 */
void debugw_init(void);
void debugw_task(void);

#endif
