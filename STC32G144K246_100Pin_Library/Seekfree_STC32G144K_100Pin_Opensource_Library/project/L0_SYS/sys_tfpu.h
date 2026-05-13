#ifndef __TFPU_MATH_H__
#define __TFPU_MATH_H__

// 声明外部汇编函数
// 必须使用 extern，并且无需指定寄存器，因为ASM中已经处理了寄存器位置
// 确保编译器没有禁用浮点参数传递优化

void  tfpu_init(void);

float tfpu_add(float a, float b);
float tfpu_sub(float a, float b);
float tfpu_mul(float a, float b);
float tfpu_div(float a, float b);

float tfpu_sqrt(float a);

// 三角函数输入必须是弧度制
float tfpu_sin(float radian);
float tfpu_cos(float radian);
float tfpu_tan(float radian);
float tfpu_atan(float val);

// 类型转换
float tfpu_int2float(long val);
long  tfpu_float2int(float val);

#endif