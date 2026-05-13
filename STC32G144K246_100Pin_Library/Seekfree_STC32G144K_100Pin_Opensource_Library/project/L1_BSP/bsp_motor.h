#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "zf_common_typedef.h"

void bsp_motor_init(void);
void bsp_motor_set_left_pwm(uint32 duty);
void bsp_motor_set_right_pwm(uint32 duty);
void bsp_motor_set_pwm(uint32 left_duty, uint32 right_duty);
void bsp_motor_set_left_dir(uint8 level);
void bsp_motor_set_right_dir(uint8 level);
void bsp_motor_set_dir(uint8 left_level, uint8 right_level);
void bsp_motor_set_nsleep(uint8 enable);

#endif
