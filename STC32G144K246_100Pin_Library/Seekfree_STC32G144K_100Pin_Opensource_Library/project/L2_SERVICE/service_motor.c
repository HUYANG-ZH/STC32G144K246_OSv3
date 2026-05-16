#include "zf_common_headfile.h"
#include "bsp_include.h"
#include "service_motor.h"

#define SERVICE_MOTOR_NSLEEP_PRE_HIGH_MS       (1U)
#define SERVICE_MOTOR_NSLEEP_RESET_LOW_US      (30U)
#define SERVICE_MOTOR_NSLEEP_POST_HIGH_MS      (1U)

#define SERVICE_MOTOR_LEFT_FORWARD_LEVEL       GPIO_HIGH
#define SERVICE_MOTOR_RIGHT_FORWARD_LEVEL      GPIO_HIGH

static uint8 motor_initialized = 0U;
static uint8 motor_ready = 0U;

static void service_motor_init_bsp_once(void)
{
    if(0U == motor_initialized)
    {
        bsp_motor_init();
        motor_initialized = 1U;
    }
}

static uint8 service_motor_reverse_level(uint8 level)
{
    return (GPIO_HIGH == level) ? GPIO_LOW : GPIO_HIGH;
}

static uint32 service_motor_abs_duty(int32 pwm)
{
    if((int32)PWM_DUTY_MAX <= pwm)
    {
        return PWM_DUTY_MAX;
    }

    if(-((int32)PWM_DUTY_MAX) >= pwm)
    {
        return PWM_DUTY_MAX;
    }

    if(0 > pwm)
    {
        return (uint32)(0 - pwm);
    }

    return (uint32)pwm;
}

static uint8 service_motor_dir_level(int32 pwm, uint8 forward_level)
{
    if(0 <= pwm)
    {
        return forward_level;
    }

    return service_motor_reverse_level(forward_level);
}

void service_motor_init(void)
{
    service_motor_init_bsp_once();
    service_motor_reset();
}

void service_motor_debug(void)
{
    printf("[motor:30/100]\r\n");
    service_motor_set_pwm(3000,3000);
}

void service_motor_reset(void)
{
    service_motor_init_bsp_once();

    motor_ready = 0U;
    bsp_motor_set_pwm(0U, 0U);

    bsp_motor_set_nsleep(1U);
    system_delay_ms(SERVICE_MOTOR_NSLEEP_PRE_HIGH_MS);
    bsp_motor_set_nsleep(0U);
    system_delay_us(SERVICE_MOTOR_NSLEEP_RESET_LOW_US);
    bsp_motor_set_nsleep(1U);
    system_delay_ms(SERVICE_MOTOR_NSLEEP_POST_HIGH_MS);

    motor_ready = 1U;
}

void service_motor_stop(void)
{
    if(0U == motor_initialized)
    {
        return;
    }

    bsp_motor_set_pwm(0U, 0U);
}

void service_motor_set_left_pwm(int32 pwm)
{
    if(0U == motor_ready)
    {
        return;
    }

    bsp_motor_set_left_dir(service_motor_dir_level(pwm, SERVICE_MOTOR_LEFT_FORWARD_LEVEL));
    bsp_motor_set_left_pwm(service_motor_abs_duty(pwm));
}

void service_motor_set_right_pwm(int32 pwm)
{
    if(0U == motor_ready)
    {
        return;
    }

    bsp_motor_set_right_dir(service_motor_dir_level(pwm, SERVICE_MOTOR_RIGHT_FORWARD_LEVEL));
    bsp_motor_set_right_pwm(service_motor_abs_duty(pwm));
}

void service_motor_set_pwm(int32 left_pwm, int32 right_pwm)
{
    if(0U == motor_ready)
    {
        return;
    }

    bsp_motor_set_dir(service_motor_dir_level(left_pwm, SERVICE_MOTOR_LEFT_FORWARD_LEVEL),
            service_motor_dir_level(right_pwm, SERVICE_MOTOR_RIGHT_FORWARD_LEVEL));
    bsp_motor_set_pwm(service_motor_abs_duty(left_pwm), service_motor_abs_duty(right_pwm));
}

uint8 service_motor_is_ready(void)
{
    return motor_ready;
}
