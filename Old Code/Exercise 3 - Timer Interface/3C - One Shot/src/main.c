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

// Completely stop blinking and turn off LEDs
void all_leds_off() {
    TIM2->DIER &= ~TIM_DIER_UIE;   // Disable TIM2 update interrupt (stops calling blink_leds)
    TIM2->CR1 &= ~TIM_CR1_CEN;     // Disable TIM2 counter (stops the timer itself)
    GPIOE->ODR &= 0x00FF;          // Clear upper byte: PE8–PE15 = OFF
}

// Temporarily pause blinking for a specified duration (in milliseconds)
void temp_leds_off(uint32_t delay_ms) {
    pause_timer(TIM2);                     // Pause TIM2 blinking
    GPIOE->ODR &= 0x00FF;                  // Turn off LEDs immediately
    pause_timer_for(TIM2, delay_ms);      // Resume blinking after `delay_ms` using TIM4 as one-shot
}

// This callback is passed to the one-shot timer (TIM3) — executes after 4 seconds
void temp_leds_off_callback(void) {
    temp_leds_off(2000);   // Pause blinking for 2 seconds
}

int main(void) {
    enable_clocks();          // Enable all required peripheral clocks
    initialise_board();       // Set LED pins (PE8–PE15) to output mode

    __enable_irq();           // Globally enable interrupts

    timer_init(100, blink_leds);              // Set up TIM2 with 1ms tick (via prescaler) and blink_leds callback
    set_period(500);                          // Set period to 500ms (blinking interval)

    start_oneshot(4000, temp_leds_off_callback);  // After 4 seconds, turn off LEDs for 2 seconds, then resume blinking

    while (1) {
        // All logic is interrupt-driven — nothing required in main loop
    }
}
