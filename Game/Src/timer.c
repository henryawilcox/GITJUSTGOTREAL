#include "timer.h"
#include "stm32f303xc.h"

static TimerCallback periodicCallback = 0;
static TimerCallback oneShotCallback = 0;

static uint32_t timerPeriod_ms = 0;
static uint8_t isOneShotMode = 0;

void Timer_Init(uint32_t period_ms, TimerCallback cb) {
    timerPeriod_ms = period_ms;
    periodicCallback = cb;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 8000 - 1;  // 1 ms tick
    TIM2->ARR = timerPeriod_ms;
    TIM2->CNT = 0;

    TIM2->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;

    NVIC_EnableIRQ(TIM2_IRQn);
}

void Timer_Start(void) {
    isOneShotMode = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Timer_StartOneShot(uint32_t delay_ms, TimerCallback cb) {

    isOneShotMode = 1;
    oneShotCallback = cb;

    TIM2->CR1 &= ~TIM_CR1_CEN;      // Stop timer before configuring
    TIM2->PSC = 8000 - 1;           // 1 ms tick
    TIM2->ARR = delay_ms;
    TIM2->CNT = 0;

    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;

    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

void setTimerPeriod(uint32_t new_period_ms) {
    timerPeriod_ms = new_period_ms;
    TIM2->ARR = timerPeriod_ms;
    TIM2->EGR |= TIM_EGR_UG;
}

uint32_t getTimerPeriod(void) {
    return timerPeriod_ms;
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        if (isOneShotMode) {
            if (oneShotCallback) oneShotCallback();
            oneShotCallback = 0;
            isOneShotMode = 0;
            TIM2->CR1 &= ~TIM_CR1_CEN; // Stop timer
        } else {
            if (periodicCallback) periodicCallback();
        }
    }
}

// Example LED blink callback
void periodicBlink(void) {
    static uint8_t led_state = 0; //start with LEDs off
    uint8_t *led_output = ((uint8_t *)&(GPIOE->ODR)) + 1;
    *led_output = led_state ? 0xAA : 0x00;  // 0xAA = 10101010
    led_state = !led_state; //switch the LED state from 10101010 to 00000000 or vice versa dependent on current
}

void oneShotBlink(void) {
    uint8_t *led_output = ((uint8_t *)&(GPIOE->ODR)) + 1;
    *led_output = 0xFF;  // turn all LED's briefly
    for (volatile int i = 0; i < 10000; i++); //gunshot flash for some arbitrary amount of time, small relative to wait time
    *led_output = 0x00; //turn off LEDs
}

void enable_clocks(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // For EXTI
}

void initialise_board(void) {
    // Configure PE8-15 as output

    // Configure PC13 as input (default state)
    GPIOA->MODER &= ~(3 << (0 * 2)); // PA0 input

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;      // Clear EXTI0 bits
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PA;   // Set EXTI0 to PA0

    EXTI->IMR  |= EXTI_IMR_MR0;     // Unmask EXTI0
    EXTI->FTSR |= EXTI_FTSR_TR0;    // Trigger on falling edge
    NVIC_EnableIRQ(EXTI0_IRQn);     // Enable EXTI0 interrupt in NVIC

}

void oneshot_action(uint32_t operand){
	Timer_StartOneShot(operand, oneShotBlink); //call function to trigger a oneshot using the specified operand from the serial comm.
}

//THIS CODE SHOULD RUN AFTER YOU HAVE CHECKED IF THE OPERATOR IS ONESHOT AND THE OPERAND IS ALL DIGITS
//IT SHOULD BE DEFINED IN INTEGRATION.c OR IN A RELEVANT HEADER (INTEGRATION.h)
void periodic_action(uint32_t operand){
	setTimerPeriod(operand); //set the hardware timer ARR period as the operand given in the serial comm.
	Timer_Init(operand, periodicBlink); // initialise the timer for periodic blinking with the operand as the delay_time
	Timer_Start();
	periodicBlink(); // Immediate first toggle
}



