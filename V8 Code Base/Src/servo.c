#include "servo.h"
#include "stm32f303xc.h"

#define SERVO_TIMER       TIM3
#define SERVO_CHANNEL     1
#define SERVO_GPIO_PORT   GPIOC
#define SERVO_GPIO_PIN    6
#define SERVO_GPIO_AF     2  // AF2 for TIM3_CH1 on PC6
#define SERVO_OPEN_PULSE  2400  // in microseconds
#define SERVO_CLOSE_PULSE 500  // in microseconds

void Servo_Init(void) {
    // Enable clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Set PA6 to AF mode
    SERVO_GPIO_PORT->MODER &= ~(3 << (SERVO_GPIO_PIN * 2));
    SERVO_GPIO_PORT->MODER |= (2 << (SERVO_GPIO_PIN * 2));  // AF mode
    SERVO_GPIO_PORT->AFR[0] |= (SERVO_GPIO_AF << (SERVO_GPIO_PIN * 4));

    // Configure timer for 50 Hz PWM (20 ms period)
    SERVO_TIMER->PSC = 8 - 1;      // 72 MHz / 72 = 1 MHz (1 µs tick)
    SERVO_TIMER->ARR = 20000 - 1;   // 20,000 µs period = 50 Hz

    // PWM mode 1 on CH1
    SERVO_TIMER->CCMR1 |= (6 << 4);  // OC1M = 110
    SERVO_TIMER->CCMR1 |= (1 << 3);  // OC1PE enable
    SERVO_TIMER->CCER |= (1 << 0);   // Enable CH1 output
    SERVO_TIMER->CR1 |= (1 << 7);    // ARPE
    SERVO_TIMER->EGR |= (1 << 0);    // UG - force update
    SERVO_TIMER->CR1 |= (1 << 0);    // Enable counter

    // Start with neutral pulse
    Servo_SetPulse(1500);
}

void Servo_SetPulse(uint16_t pulse_us) {
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    SERVO_TIMER->CCR1 = pulse_us;
}

void Servo_Open(void) {
    Servo_SetPulse(SERVO_OPEN_PULSE);
}

void Servo_Close(void) {
    Servo_SetPulse(SERVO_CLOSE_PULSE);
}
