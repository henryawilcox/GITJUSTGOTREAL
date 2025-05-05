/*
* main.c
* main program that controls leds with button input
* uses the dio module for handling leds
* stage b
*/

#include <stdint.h>
#include "stm32f303xc.h"
#include "dio.h"



// handles what happens when button pressed
void button_pressed(void)
{
    dio_toggle_led_halves();
}


int main(void)
{
    // initialize the digital I/O module with callback
    dio_init(&button_pressed);

    //loop 4ever
    for(;;) {
    }
}
