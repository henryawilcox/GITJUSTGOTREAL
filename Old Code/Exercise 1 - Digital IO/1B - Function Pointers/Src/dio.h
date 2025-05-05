/*
* dio.h
* header for the digital i/o module that manages leds and buttons
* handles led state management - stage b
*/

#ifndef __DIO_H
#define __DIO_H

#include <stdint.h>
#include "stm32f303xc.h"

// initialise and set up button callback
void dio_init(void (*callback)(void));

//toggle between both led halves
void dio_toggle_led_halves(void);

//interrupt handler for button pressed
void EXTI0_IRQHandler(void);

#endif /* DIO_H_ */
