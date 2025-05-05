#include "timer_module.h"
#include "stm32f303xc.h"
#include <stdbool.h>

// --- Internal module state ---

static void (*timer_callback)(void) = 0;       // Callback for periodic timer (TIM2)
static void (*oneshot_callback)(void) = 0;     // Callback for one-shot timer (TIM3)
static uint32_t timer_period = 1000;           // Period in ms for periodic timer (kept private)
static TIM_TypeDef *paused_timer = 0;          // Pointer to any timer currently paused by pause_timer_for()

// --- Utility: Force prescaler to load using a short fake overflow sequence ---
static void trigger_prescaler(TIM_TypeDef *TIMx, uint32_t real_arr) {
    TIMx->ARR = 0x01;        // Set a very short period
    TIMx->CNT = 0x00;        // Reset counter
    __asm__("NOP");          // Wait a few cycles (hardware sync delay)
    __asm__("NOP");
    __asm__("NOP");
    TIMx->ARR = real_arr;    // Restore real period after prescaler loaded
}

// --- TIM2: Periodic Timer Setup ---
void timer_init(uint32_t period_ms, void (*callback)(void)) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // Enable TIM2 clock

    TIM2->CR1 = 0;        // Disable timer before configuration
    TIM2->CNT = 0;        // Reset counter

    TIM2->PSC = 8000 - 1;         // Prescaler: 8 MHz / 8000 = 1 kHz (1ms tick)
    trigger_prescaler(TIM2, period_ms);   // Force prescaler to latch
    TIM2->ARR = period_ms;        // Set the auto-reload (match) value

    timer_callback = callback;    // Save the function to call on interrupt
    timer_period = period_ms;     // Save period internally for access via get/set

    TIM2->DIER |= TIM_DIER_UIE;   // Enable update interrupt
    NVIC_EnableIRQ(TIM2_IRQn);    // Enable interrupt in NVIC
    TIM2->CR1 |= TIM_CR1_CEN;     // Start TIM2
}

// --- Change the period of TIM2 while running ---
void set_period(uint32_t period_ms) {
    timer_period = period_ms;

    TIM2->CR1 &= ~TIM_CR1_CEN;    // Stop timer
    TIM2->CNT = 0;                // Reset count
    TIM2->ARR = period_ms;        // Update auto-reload register
    TIM2->EGR = TIM_EGR_UG;       // Latch updated registers
    TIM2->CR1 |= TIM_CR1_CEN;     // Restart timer
}

// --- Read the currently configured period (read-only access) ---
uint32_t get_period(void) {
    return timer_period;
}

// --- TIM2 Interrupt Handler: handles periodic blinking ---
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;           // Clear interrupt flag

        GPIOE->ODR ^= (1 << 15);           // Toggle PE15 for debug pulse

        if (timer_callback) timer_callback();  // Call the user’s callback
    }
}
