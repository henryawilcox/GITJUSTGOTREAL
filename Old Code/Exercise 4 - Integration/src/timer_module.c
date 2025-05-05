#include "timer_module.h"
#include "stm32f303xc.h"
#include <stdbool.h>

/* header
 * This module sets two timers using TIM3 and TIM4 on the STM32F303:
 *
 * - TIM3 does the repetitive stuff
 * - TIM4 handles oneshot
 *
 * functionality:
 *   - Start a periodic timer with a callback
 *   - change the period anytime
 *   - set a one-shot delay callback
 *   - Check what period you're running at
 *
 * Internally:
 *   - TIM3 and TIM4 both run off 1ms resolution (using 8kHz prescaler @ 8MHz)
 *
*/


// --- Internal module state ---
static void (*timer_callback)(void) = 0;       // Callback for periodic timer (TIM3)
static void (*oneshot_callback)(void) = 0;     // Callback for one-shot timer (TIM4)
static uint32_t timer_period = 1000;           // Period in ms for periodic timer (kept private)

// --- Utility: Force prescaler to load using a short fake overflow sequence ---
static void trigger_prescaler(TIM_TypeDef *TIMx, uint32_t real_arr) {
    TIMx->ARR = 0x01;
    TIMx->CNT = 0x00;
    __asm__("NOP");
    __asm__("NOP");
    __asm__("NOP");
    TIMx->ARR = real_arr;
}

// --- TIM3: Periodic Timer Setup ---
void timer_init(uint32_t period_ms, void (*callback)(void)) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CR1 = 0;
    TIM3->CNT = 0;
    TIM3->PSC = 8000 - 1;
    //trigger_prescaler(TIM3, period_ms);
    TIM3->ARR = period_ms;
    TIM3->EGR = TIM_EGR_UG;

    timer_callback = callback;
    timer_period = period_ms;

    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

// --- Change the period of TIM3 while running ---
void set_period(uint32_t period_ms) {
    timer_period = period_ms;

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->CNT = 0;
    TIM3->ARR = period_ms;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

// --- Read the currently configured period (read-only access) ---
uint32_t get_period(void) {
    return timer_period;
}

// --- TIM4: One-shot Timer for Delayed Callback ---
void start_oneshot(uint32_t delay_ms, void (*callback)(void)) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->CR1 = 0;
    TIM4->CNT = 0;
    TIM4->PSC = 8000 - 1;
    TIM4->ARR = delay_ms;
    TIM4->EGR = TIM_EGR_UG;

    TIM4->SR = 0;

    oneshot_callback = callback;

    TIM4->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 |= TIM_CR1_CEN;
}

// --- TIM3 Interrupt Handler: Periodic Timer ---
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;

        // OPTIONAL: Debug blink still useful
        GPIOE->ODR ^= 0x0100;

        // ✅ Actually trigger the blinking logic
        if (timer_callback) timer_callback();
    }
}

// --- TIM4 Interrupt Handler: One-shot Timer ---
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;

        if (oneshot_callback) oneshot_callback();

        TIM4->DIER &= ~TIM_DIER_UIE;
        TIM4->CR1 &= ~TIM_CR1_CEN;
    }
}

void enable_timer_interrupts(void)
{
    __disable_irq();

    // TIM3: blinking (periodic)
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);

    // TIM4: one-shot
    NVIC_SetPriority(TIM4_IRQn, 2);
    NVIC_EnableIRQ(TIM4_IRQn);

    __enable_irq();
}
