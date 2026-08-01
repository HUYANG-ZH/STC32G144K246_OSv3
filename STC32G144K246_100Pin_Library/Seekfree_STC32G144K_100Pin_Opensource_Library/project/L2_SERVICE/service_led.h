#ifndef SERVICE_LED_H
#define SERVICE_LED_H

#include "zf_common_typedef.h"

void service_led_init(void);
void service_led_debug(void);
void service_led_on(void);
void service_led_off(void);
void service_led_set(uint8 enable);
/* Turn on for duration_ms, then turn off automatically. */
void service_led_on_ms(uint32 duration_ms);
/* Compatibility-style name for the timed-on operation. */
void service_led_delay_ms(uint32 duration_ms);
/*
 * Blink with the specified on/off durations.  repeat_count is the number
 * of complete blink cycles; zero means blink continuously until stopped.
 */
void service_led_blink_ms(uint32 on_time_ms, uint32 off_time_ms, uint32 repeat_count);
/* Read the application state, update the LED mode, and run the timing state machine. */
void service_led_task(void);
void service_led_stop(void);
uint8 service_led_is_on(void);

#endif
