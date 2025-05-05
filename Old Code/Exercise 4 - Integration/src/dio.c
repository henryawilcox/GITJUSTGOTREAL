// dio.c
// this is a digital i/o module that handles leds and buttons
// manages led state with rate limiting to prevent rapid changes

#include "dio.h"
#include "timer_module.h"
#include "stm32f303xc.h"
#include <stdbool.h>

// internal led stuff
static uint8_t led_state = 0;          // current led state
static uint8_t blink_mask = 0;
static uint32_t led_rate_ms = 100;     // default cooldown time (100ms)
static bool blinking_active = false;
static bool toggle_flag = false;

// interrupt handler for button press (pa0)
// disables further interrupts, clears pending flag, and starts timer
void EXTI0_IRQHandler(void)
{
    EXTI->IMR &= ~EXTI_IMR_MR0;
    EXTI->PR |= EXTI_PR_PR0;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

// timer2 interrupt handler
// re-enables exti interrupt and stops the timer
void TIM2_IRQHandler(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR &= ~TIM_SR_UIF;
    EXTI->IMR |= EXTI_IMR_MR0;
}

// enables peripheral clocks for gpio and timer2
static void enable_clocks(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

static void initialise_board(void)
{
    uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
    *led_output_registers = 0x5555;
}


// configures timer2 for delay-based rate limiting
static void configure_timer(void)
{
    __disable_irq();
    TIM2->PSC = 7999; // 1ms tick with 8mhz / 8000
    TIM2->ARR = led_rate_ms;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 &= ~TIM_CR1_CEN;
    __enable_irq();
}


// configures exti0 (pa0) for button press with rising edge trigger
static void enable_interrupt(void)
{
    __disable_irq();
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->IMR |= EXTI_IMR_MR0;
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);
    __enable_irq();
}


// initializes dio module: clocks, gpio, timer, interrupt, and sets leds off
void dio_init(void)
{
    enable_clocks();
    initialise_board();
    configure_timer();
    enable_interrupt();
    led_state = 0x00;
    uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
    *led_register = led_state;
}


// returns the current led state (8-bit mask)
uint8_t dio_get_led_state(void)
{
    return led_state;
}


// sets the led state and resets the timer for rate limiting
void dio_set_led_state(uint8_t state)
{
    led_state = state;

    if (blinking_active) {
        blink_mask = state;
    }

    uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
    *led_register = led_state;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}


// sets the minimum time between led changes (rate limit) in milliseconds
void dio_set_led_rate(uint32_t rate_ms)
{
    led_rate_ms = (rate_ms < 1) ? 1 : rate_ms;
    TIM2->ARR = led_rate_ms - 1;
}


// starts blinking mode — actual toggle happens in dio_toggle_leds()
void dio_start_blinking(void)
{
    blinking_active = true;
    toggle_flag = false;  // Start from OFF
}

// stops blinking mode
void dio_stop_blinking(void)
{
    blinking_active = false;
}


// toggles led state based on blinking flag
// alternates between off and stored blink_mask
void dio_toggle_leds(void)
{
    if (!blinking_active) return;

    if (toggle_flag) {
        dio_set_led_state(blink_mask);
    } else {
        uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
        *led_register = 0x00;
    }
    toggle_flag = !toggle_flag;
}


// inverts current led state
// if blinking is active, inverts blink mask instead
void dio_invert_leds(void)
{
    if (blinking_active) {
        blink_mask = ~blink_mask;
        toggle_flag = false;
    } else {
        uint8_t new_state = ~dio_get_led_state();
        dio_set_led_state(new_state);
    }
}

// updates the blink mask based on current led state
void dio_update_blink_mask_if_active(void)
{
    if (blinking_active) {
        led_state = dio_get_led_state();
    }
}
