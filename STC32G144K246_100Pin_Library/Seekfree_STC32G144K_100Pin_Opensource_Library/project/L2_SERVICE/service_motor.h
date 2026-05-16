#ifndef SERVICE_MOTOR_H
#define SERVICE_MOTOR_H

#include "zf_common_typedef.h"

void service_motor_init(void);
void service_motor_debug(void);
void service_motor_reset(void);
void service_motor_stop(void);
void service_motor_set_left_pwm(int32 pwm);
void service_motor_set_right_pwm(int32 pwm);
void service_motor_set_pwm(int32 left_pwm, int32 right_pwm);
uint8 service_motor_is_ready(void);

#endif
