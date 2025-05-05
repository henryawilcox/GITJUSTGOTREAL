#include "stm32f303xc.h"
#include "timer_module.h"

// Enable clocks for GPIOE and the timers (TIM2 for blinking, TIM3 for one-shot)
void enable_clocks() {
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;                             // Enable clock for GPIOE (for LED output)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;       // Enable clocks for TIM2 and TIM3
}

// Configure PE8–PE15 as outputs (LEDs)
void initialise_board() {
    GPIOE->MODER &= ~(0xFFFF << 16);       // Clear mode bits for PE8–PE15
    GPIOE->MODER |=  (0x5555 << 16);       // Set PE8–PE15 as general purpose outputs (binary: 01 for each pin)
}

// State variable for LED pattern
uint8_t led_state = 0;

// Periodic LED blinking pattern (toggles PE8–PE15 every call)
void blink_leds() {
    led_state ^= 0xFF;     // Invert bits: 0xFF → 0x00 → 0xFF ...
    GPIOE->ODR = (GPIOE->ODR & 0x00FF) | (led_state << 8);  // Write new state to PE8–PE15 only
}


int main(void) {
    enable_clocks();          // Enable all required peripheral clocks
    initialise_board();       // Set LED pins (PE8–PE15) to output mode

    __enable_irq();           // Globally enable interrupts

    timer_init(500, blink_leds);              // Set up TIM2 with 1ms tick (via prescaler) and blink_leds callback


    while (1) {
        // All logic is interrupt-driven — nothing required in main loop
    }
}
