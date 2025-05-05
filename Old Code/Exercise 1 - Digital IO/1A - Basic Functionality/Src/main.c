/*
* main.c
* main program that controls leds with button input
* stage a
*/

#include <stdint.h>
#include "stm32f303xc.h"
#include "dio.h"


//main function - loop
int main(void)
{
    // initialise digital i/o
    dio_init();

    // loop 4ever
    for(;;) {
    }
}
