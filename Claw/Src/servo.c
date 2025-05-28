/**
 * @file servo.c
 * @brief PWM-based servo control using TIM3 on STM32F303.
 *
 * This module configures TIM3 CH1 on PC6 to generate a 50Hz PWM signal
 * for standard servo control. Functions allow setting pulse width directly,
 * or using high-level open/close commands.
 *
 * Designed for USYD MTRX2700 robotic systems.
 */

#include "servo.h"
#include "stm32f303xc.h"
#include "config.h"  // Suggested: move SERVO_GPIO_*, TIM, and pulse values here

// Local default values in case config.h is not used
#define SERVO_TIMER       TIM3
#define SERVO_CHANNEL     1
#define SERVO_GPIO_PORT   GPIOC
#define SERVO_GPIO_PIN    6
#define SERVO_GPIO_AF     2  // AF2 = TIM3_CH1 on PC6
#define SERVO_OPEN_PULSE  1600  ///< in microseconds
#define SERVO_CLOSE_PULSE 1470  ///< in microseconds

/**
 * @brief Initialises TIM3 CH1 for PWM output to control a servo on PC6.
 */
void Servo_Init(void) {
    // Enable GPIO and timer clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Set PC6 to Alternate Function mode
    SERVO_GPIO_PORT->MODER &= ~(3 << (SERVO_GPIO_PIN * 2));
    SERVO_GPIO_PORT->MODER |=  (2 << (SERVO_GPIO_PIN * 2));

    // Select alternate function AF2 for TIM3_CH1
    SERVO_GPIO_PORT->AFR[0] &= ~(0xF << (SERVO_GPIO_PIN * 4));
    SERVO_GPIO_PORT->AFR[0] |=  (SERVO_GPIO_AF << (SERVO_GPIO_PIN * 4));

    // Configure TIM3 for 50Hz PWM: 1us ticks, 20ms period
    SERVO_TIMER->PSC = 8 - 1;       // 8-1 = 1 MHz = 1us tick
    SERVO_TIMER->ARR = 20000 - 1;    // 20,000us = 20ms = 50Hz

    // PWM Mode 1 on CH1, preload enable
    SERVO_TIMER->CCMR1 |= (6 << 4);  // OC1M = 110
    SERVO_TIMER->CCMR1 |= (1 << 3);  // OC1PE = 1
    SERVO_TIMER->CCER  |= (1 << 0);  // Enable CH1 output
    SERVO_TIMER->CR1   |= (1 << 7);  // ARPE = 1
    SERVO_TIMER->EGR   |= (1 << 0);  // UG = update registers
    SERVO_TIMER->CR1   |= (1 << 0);  // CEN = enable counter

    // Start with neutral position
    Servo_SetPulse(1500);
}

/**
 * @brief Sets the servo pulse width in microseconds (1000-2000us typical).
 * @param pulse_us Desired pulse width
 */
void Servo_SetPulse(uint16_t pulse_us) {
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    SERVO_TIMER->CCR1 = pulse_us;
}

/**
 * @brief Sends the 'open' position pulse to the servo.
 */
void Servo_Open(void) {
    Servo_SetPulse(SERVO_OPEN_PULSE);
}

/**
 * @brief Sends the 'close' position pulse to the servo.
 */
void Servo_Close(void) {
    Servo_SetPulse(SERVO_CLOSE_PULSE);
}



