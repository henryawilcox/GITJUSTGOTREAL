/**
 * @file stepper_timer.h
 * @brief Stepper motor control interface for STM32 timer-based motion.
 *
 * Provides functions for initializing stepper control GPIO, timers, and
 * velocity planning for CoreXY and Z-axis motion systems. Designed to be
 * used with a fixed-interval timer ISR and external velocity ramping.
 */

#ifndef STEPPER_TIMER_H
#define STEPPER_TIMER_H

#include "stm32f303xc.h"
#include <stdint.h>
#include "serial.h"
#include <stdbool.h>

/**
 * @brief Initialize GPIO pins for Axis 0 (X/Y stepper output).
 *
 * Sets direction and step pins to output mode. Should be called before
 * any axis motion is commanded.
 */
void init_axis0_gpio(void);

/**
 * @brief Initialize GPIO pins for Axis 1 (X/Y stepper output).
 */
void init_axis1_gpio(void);

/**
 * @brief Initialize GPIO pins for Axis 2 (Z stepper output).
 */
void init_axis2_gpio(void);

/**
 * @brief Initialize the stepper timer.
 *
 * Configures a base timer to trigger a fixed-frequency interrupt
 * (e.g. every 40 µs) used to drive stepper pulses.
 */
void stepper_timer_init(void);

/**
 * @brief Initialize all stepper control systems.
 *
 * Includes timer init, GPIO setup, and resets control state.
 */
void stepper_control_init(void);

/**
 * @brief Plan CoreXY velocity based on input vector and speed index.
 *
 * Converts an X/Y velocity vector into A/B motor steps using CoreXY kinematics.
 * @param vx X-axis velocity (-100 to +100)
 * @param vy Y-axis velocity (-100 to +100)
 * @param index Index into velocity table (0–99)
 */
void plan_velocity(int8_t vx, int8_t vy, uint8_t index);

/**
 * @brief Set the absolute target for Z-axis position.
 *
 * Used to control vertical stepper motion.
 * @param z_target Target Z position in steps (signed)
 */
void set_z_target(int16_t z_target);

#endif // STEPPER_TIMER_H
