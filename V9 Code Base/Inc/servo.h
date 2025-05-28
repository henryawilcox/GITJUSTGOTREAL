/**
 * @file servo.h
 * @brief Interface for PWM-based servo control.
 *
 * Provides function declarations to control a standard RC servo using
 * STM32 timer-based PWM. Designed for use in embedded robotic systems
 * such as those built in MTRX2700.
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

/**
 * @brief Initialise the timer and GPIO for servo PWM control.
 *
 * Configures TIM3 CH1 on PC6 (by default) for 50Hz output.
 */
void Servo_Init(void);

/**
 * @brief Set the servo PWM pulse width manually.
 * @param pulse_us Pulse width in microseconds (typically 1000–2000us)
 */
void Servo_SetPulse(uint16_t pulse_us);

/**
 * @brief Move the servo to the 'open' position.
 */
void Servo_Open(void);

/**
 * @brief Move the servo to the 'closed' position.
 */
void Servo_Close(void);

#endif /* SERVO_H_ */
