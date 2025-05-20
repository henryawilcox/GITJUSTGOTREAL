#include "gyro.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#include <math.h>
#include <stdio.h>

static float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
static float i = 0.0f;
static float accel_x_bias = 0.0f, accel_y_bias = 0.0f, accel_z_bias = 0.0f;
static float gyroAngleX = 0.0f, gyroAngleY = 0.0f;
static const float alpha = 0.1f;
static const float dt = 0.01f;

void Gyro_Init(void)
{
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
}

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


