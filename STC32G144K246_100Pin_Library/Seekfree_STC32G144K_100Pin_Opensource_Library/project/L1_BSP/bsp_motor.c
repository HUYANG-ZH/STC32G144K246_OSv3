#include "zf_common_headfile.h"
#include "bsp_motor.h"

#define BSP_MOTOR_PWM_FREQ_HZ       (10000U)
#define BSP_MOTOR_INIT_DUTY         (0U)

#define BSP_MOTOR_LEFT_PWM_CH       PWMC_CH1P_P80
#define BSP_MOTOR_LEFT_DIR_PIN      IO_P62

#define BSP_MOTOR_RIGHT_PWM_CH      PWMC_CH2P_P82
#define BSP_MOTOR_RIGHT_DIR_PIN     IO_P83

#define BSP_MOTOR_NSLEEP_PIN        IO_P61
#define BSP_MOTOR_INIT_DIR_LEVEL    GPIO_LOW
#define BSP_MOTOR_NSLEEP_ACTIVE     GPIO_HIGH
#define BSP_MOTOR_NSLEEP_INACTIVE   GPIO_LOW

static uint32 bsp_motor_limit_duty(uint32 duty)
{
    if(PWM_DUTY_MAX < duty)
    {
        return PWM_DUTY_MAX;
    }

    return duty;
}

void bsp_motor_init(void)
{
    gpio_init(BSP_MOTOR_NSLEEP_PIN, GPO, BSP_MOTOR_NSLEEP_INACTIVE, GPO_PUSH_PULL);
    gpio_init(BSP_MOTOR_LEFT_DIR_PIN, GPO, BSP_MOTOR_INIT_DIR_LEVEL, GPO_PUSH_PULL);
    gpio_init(BSP_MOTOR_RIGHT_DIR_PIN, GPO, BSP_MOTOR_INIT_DIR_LEVEL, GPO_PUSH_PULL);

    pwm_init(BSP_MOTOR_LEFT_PWM_CH, BSP_MOTOR_PWM_FREQ_HZ, BSP_MOTOR_INIT_DUTY);
    pwm_init(BSP_MOTOR_RIGHT_PWM_CH, BSP_MOTOR_PWM_FREQ_HZ, BSP_MOTOR_INIT_DUTY);
}

void bsp_motor_debug(void)
{
}

void bsp_motor_set_left_pwm(uint32 duty)
{
    pwm_set_duty(BSP_MOTOR_LEFT_PWM_CH, bsp_motor_limit_duty(duty));
}

void bsp_motor_set_right_pwm(uint32 duty)
{
    pwm_set_duty(BSP_MOTOR_RIGHT_PWM_CH, bsp_motor_limit_duty(duty));
}

void bsp_motor_set_pwm(uint32 left_duty, uint32 right_duty)
{
    bsp_motor_set_left_pwm(left_duty);
    bsp_motor_set_right_pwm(right_duty);
}

void bsp_motor_set_left_dir(uint8 level)
{
    gpio_set_level(BSP_MOTOR_LEFT_DIR_PIN, level);
}

void bsp_motor_set_right_dir(uint8 level)
{
    gpio_set_level(BSP_MOTOR_RIGHT_DIR_PIN, level);
}

void bsp_motor_set_dir(uint8 left_level, uint8 right_level)
{
    bsp_motor_set_left_dir(left_level);
    bsp_motor_set_right_dir(right_level);
}

void bsp_motor_set_nsleep(uint8 enable)
{
    gpio_set_level(BSP_MOTOR_NSLEEP_PIN, (0U != enable) ? BSP_MOTOR_NSLEEP_ACTIVE : BSP_MOTOR_NSLEEP_INACTIVE);
}
