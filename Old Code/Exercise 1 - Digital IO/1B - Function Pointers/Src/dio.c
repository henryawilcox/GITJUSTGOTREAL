/*
* dio.c
* this is a digital i/o module that handles leds and buttons
*/

#include "dio.h"

// function pointer for button callback
static void (*button_callback)(void) = 0x00;


// handles button press interrupts
void EXTI0_IRQHandler(void)
{
    // call the button handler if we have one
    if (button_callback != 0x00) {
        button_callback();
    }

    // clear the interrupt flag
    EXTI->PR |= EXTI_PR_PR0;
}


// enable clocks of perifs
static void enable_clocks(void)
{
    // enable gpio clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;

}

//initialise board
static void initialise_board(void)
{
    //config led pins (PE8-PE15) as outputs
    uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
    *led_output_registers = 0x5555;
}


//enable button interrupt
static void enable_interrupt(void)
{
    __disable_irq();

    // enable system conifgs
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // configure PA0 as EXTI0 source
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;

    // configure EXTI0 to trigger on rising edge
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // enable EXTI0 interrupt
    EXTI->IMR |= EXTI_IMR_MR0;

    // configure NVIC for EXTI0 interrupt
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);

    // re-enable interrupts
    __enable_irq();
}


//initialise digital io with callback function
void dio_init(void (*callback)(void))
{
    //set the button callback function
    button_callback = callback;

    //initialise hardware
    enable_clocks();
    initialise_board();
    enable_interrupt();
}

//toggle between two led halves
void dio_toggle_led_halves(void)
{
    uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
    uint8_t current_state = *led_register;

    // check which half is currently on and toggle
    if ((current_state & 0xF0) == 0xF0) {
        // upper half is on, switch to lower half
        *led_register = 0x0F;
    } else {
        // lower half is on or other state, switch to upper half
        *led_register = 0xF0;
    }
}
