/**
 * @file    gyro.c
 * @brief   Gyroscope and accelerometer integration with angle filtering.
 *
 * This module initializes and reads data from the onboard accelerometer and gyroscope
 * on the STM32F3 Discovery board. It uses a complementary filter to combine accelerometer
 * and gyroscope data for estimating the pitch and roll angles.
 *
 * Includes basic calibration logic and optional deadzone angle mapping for user-defined output ranges.
 *
 * Author: Connor Stock
 * Date: 25/05/2025
 */

#include "gyro.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>

// --- Internal state variables ---
static float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
static float i = 0.0f;
static float accel_x_bias = 0.0f, accel_y_bias = 0.0f, accel_z_bias = 0.0f;
static float gyroAngleX = 0.0f, gyroAngleY = 0.0f;
static const float alpha = 0.1f;  // Complementary filter weighting
static const float dt = 0.01f;    // Time delta for integration (assumes 100 Hz sample rate)

/**
 * @brief Initializes the accelerometer and gyroscope hardware.
 *
 * This must be called before reading angle data.
 */
void Gyro_Init(void)
{
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
}

/**
 * @brief Computes filtered pitch and roll angles from accelerometer and gyro data.
 *
 * Uses a complementary filter to combine short-term gyroscope data with long-term
 * stable accelerometer data. The function includes an initial calibration phase
 * where biases are averaged over 100 samples.
 *
 * @param[out] angleX Pointer to store the filtered pitch angle (in degrees).
 * @param[out] angleY Pointer to store the filtered roll angle (in degrees).
 */
void Gyro_GetFilteredAngles(float *angleX, float *angleY)
{
	int16_t acc_raw[3];
	float gyro_raw[3];

	float ax, ay, az;
	float pitch, roll;

	static float gyro_bias_x = 0.0f;
	static float gyro_bias_y = 0.0f;
	static float gyro_sum_x = 0.0f;
	static float gyro_sum_y = 0.0f;

	BSP_ACCELERO_GetXYZ(acc_raw);
	BSP_GYRO_GetXYZ(gyro_raw);

	ax = (float)acc_raw[0] / 1500.0f;
	ay = (float)acc_raw[1] / 1500.0f;
	az = (float)acc_raw[2] / 1500.0f;

	float raw_gyro_x = gyro_raw[0];
	float raw_gyro_y = gyro_raw[1];

	// === Calibration Phase ===
	if (i < 101.0f) {
		sum_x += ax;
		sum_y += ay;
		sum_z += az - 10.0f;

		gyro_sum_x += raw_gyro_x;
		gyro_sum_y += raw_gyro_y;

		i++;
		return;
	}

	// === Post-Calibration Setup ===
	if (i == 101.0f) {
		accel_x_bias = 0.0f; // sum_x / 101.0f;
		accel_y_bias = 0.0f;
		accel_z_bias = 0.0f;

		gyro_bias_x = gyro_sum_x / 101.0f;
		gyro_bias_y = gyro_sum_y / 101.0f;

		i++;
	}

	// === Angle Estimation ===
	pitch = atan2f(ax - accel_x_bias, sqrtf((ay - accel_y_bias) * (ay - accel_y_bias) + (az - accel_z_bias) * (az - accel_z_bias))) * 180.0f / 3.14f;
	roll  = atan2f(ay - accel_y_bias, az - accel_z_bias) * 180.0f / 3.14f;

	float gyroRateX = (raw_gyro_x - gyro_bias_x) / 131.0f;
	float gyroRateY = (raw_gyro_y - gyro_bias_y) / 131.0f;

	gyroAngleX += gyroRateX * dt;
	gyroAngleY += gyroRateY * dt;

	*angleX = alpha * gyroAngleX + (1.0f - alpha) * pitch;
	*angleY = alpha * gyroAngleY + (1.0f - alpha) * roll;
}

/**
 * @brief Maps a signed angle to a range of -100 to 100 with a deadzone.
 *
 * Converts angles in degrees to an output value with a central deadzone.
 * Useful for joystick-style inputs or percentage-based actuation.
 *
 * @param[in] angle_deg     Input angle in degrees.
 * @param[in] deadzone_deg  Deadzone range near 0° where output is forced to 0.
 * @param[in] max_deg       Maximum angle used for scaling (clipped if exceeded).
 * @return int              Mapped value between -100 and 100 (excluding deadzone returns 0).
 */
int MapSignedAngleWithDeadzoneInt(int angle_deg, int deadzone_deg, int max_deg)
{
	if (max_deg <= deadzone_deg) return 0;  // prevent divide-by-zero or invalid range

	int abs_angle = (angle_deg < 0) ? -angle_deg : angle_deg;

	if (abs_angle < deadzone_deg)
		return 0;

	if (abs_angle > max_deg)
		abs_angle = max_deg;

	int usable_range = max_deg - deadzone_deg;
	int scaled = ((abs_angle - deadzone_deg) * 99) / usable_range + 1;

	return (angle_deg < 0) ? -scaled : scaled;
}

void UpdateHeightLEDs(uint8_t height_percent)
{
    if (height_percent > 100) height_percent = 100; // Clamp just in case

    // Map 0–100% to 0–8 LEDs
    uint8_t num_leds_on = (height_percent * 8) / 100;

    // Each LED is GPIOE Pin 8 to 15 (LD3 to LD10)
    uint16_t led_mask = 0xFF << 8;  // All 8 LED pins
    uint16_t new_led_state = (0xFF << (8 + (8 - num_leds_on))) & led_mask;

    // Apply new LED state to GPIOE
    GPIOE->ODR = (GPIOE->ODR & ~led_mask) | new_led_state;
}

uint8_t IsLimitSwitchPressed(void)
{
    return (GPIOA->IDR & GPIO_IDR_9) == 0 ? 1 : 0;
}
