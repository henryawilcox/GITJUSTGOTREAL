/*
* dio.h
* header for the digital i/o module that manages leds and buttons
* handles led state management - stage c
*/

#ifndef __DIO_H
#define __DIO_H

#include <stdint.h>
#include "stm32f303xc.h"

// initialise and set up button callback
void dio_init(void (*callback)(void));

// return which leds are currently on
uint8_t dio_get_led_state(void);

// changes which leds are on.
// not blocking per se, but only updates led if time delay passed
// if called too soon, we just ignore the request
void dio_set_led_state(uint8_t state);

// switches between two led states
void dio_toggle_led_halves(void);

// change how fast leds can update
void dio_set_led_rate(uint32_t rate_ms);

#endif /* __DIO_H */
