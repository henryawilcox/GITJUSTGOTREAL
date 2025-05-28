/**
 * @file    lidar.c
 * @brief   Lidar pulse input capture and height processing module for STM32.
 *
 * This module interfaces with a PWM-based LiDAR sensor using input capture on TIM1.
 * It includes initialization routines, a HAL interrupt callback for measuring pulse
 * width, and logic to compute height as a percentage from captured periods.
 *
 * Captured pulse widths are smoothed in a fixed-size buffer, and a mapping is provided
 * from period values to percentage height readings for simplified display or control.
 *
 * Author: [Your Name]
 * Date: [Today's Date]
 */

#include "lidar.h"
#include "ptu_definitions.h"
#include "stm32f3xx_hal_gpio.h"

// --- Global state ---
uint16_t last_period = 0;
uint16_t rise_time = 0;
uint16_t last_capture = 0;
uint16_t diff = 0;

#define LIDAR_SMOOTHING_WINDOW 5

static uint16_t period_buffer[LIDAR_SMOOTHING_WINDOW] = {0};
static uint8_t period_index = 0;
static uint8_t buffer_filled = 0;

/**
 * @brief Enables the GPIO clocks for ports A, C, and E.
 *
 * Required before using GPIO pins on these ports.
 */
void enable_clocks() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}

/**
 * @brief Configures GPIOE pins PE8–PE15 as output for LED display.
 *
 * Also enables GPIOA clock for LiDAR input pin. Assumes LEDs are connected to GPIOE.
 */
void initialise_board() {
	// Pointer to MODER register's upper half for configuring PE8–PE15 as outputs
	uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	*led_output_registers = 0x5555;  // Sets mode to general purpose output

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->PUPDR &= ~(3 << (9 * 2));   // Clear
	GPIOA->PUPDR |=  (1 << (9 * 2));   // Set pull-up

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();   // Needed for PA8 input capture
}

/**
 * @brief Input Capture interrupt handler for TIM1 Channel 1.
 *
 * Measures the duration between rising and falling edges of a PWM pulse.
 * Assumes rising and falling edge events are toggled using GPIO read state.
 *
 * @param htim Pointer to the TIM handle. Must be TIM1.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		uint16_t IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1)
			rise_time = IC_Val1;  // Rising edge detected
		else
			last_period = IC_Val1 - rise_time;  // Falling edge: compute high time

		diff = IC_Val1 - last_capture;  // Time since last event (for debug or filtering)
		last_capture = IC_Val1;
	}
}

/**
 * @brief Initializes the LiDAR sensor via I2C by writing a reset value.
 *
 * Sends a software reset command and delays 100 ms to allow startup.
 *
 * @param hi2c Pointer to I2C handle used for communication with LiDAR.
 */
void Lidar_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t reset_value = 0x00;
	HAL_I2C_Mem_Write(hi2c, LIDAR_WR, 0x00, 1, &reset_value, 1, 10);
	HAL_Delay(100);  // Give sensor time to reboot
}

/**
 * @brief Converts a LiDAR pulse period to a percentage height [0–100].
 *
 * Maps the period value (in timer ticks) to a scale defined by min and max height thresholds.
 * Values below min are clamped to 0, and above max to 100.
 *
 * @param period Pulse width in timer ticks.
 * @return uint8_t Height as a percentage.
 */
uint8_t Lidar_GetHeightPercent(uint16_t period)
{
	const uint16_t max_height = 4000;
	const uint16_t min_height = 500;

	if (period <= min_height)
		return 0;
	else if (period >= max_height)
		return 100;
	else
		return ((period - min_height) * 100) / (max_height - min_height);
}

/**
 * @brief Returns the last measured LiDAR pulse period.
 *
 * This value is set on each falling edge capture event.
 *
 * @return uint16_t Most recent pulse period in timer ticks.
 */
uint16_t Lidar_GetLastPeriod(void) {
    return last_period;
}
