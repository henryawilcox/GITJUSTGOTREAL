/**
 * @file    input_smoother.c
 * @brief   Provides smoothing and rate-limiting for sensor inputs (X, Y tilt and Z height).
 *
 * This module applies noise reduction and response limiting to raw input values
 * such as tilt angles and distance/height measurements. It includes:
 *  - A rate limiter with min/max delta bounds for X and Y smoothing.
 *  - A moving average filter and deadband trigger for Z smoothing.
 *
 * The logic is useful for applications like gesture control or user input where
 * responsiveness must be balanced with stability.
 *
 * Author: Connor Stock
 * Date: 25/05/2025
 */

#include "input_smoother.h"

#define HEIGHT_AVG_WINDOW 10
#define Z_DEADBAND 25

// Clamp ranges for smoothed X/Y values
#define X_MIN -100
#define X_MAX  100
#define Y_MIN -100
#define Y_MAX  100

// Internal state
static int prevX = 0;
static int prevY = 0;

static int prevZ = 0;
static int last_sent_z = 0;
static int z_buffer[HEIGHT_AVG_WINDOW] = {0};
static int z_index = 0;
static int z_filled = 0;

/**
 * @brief Limits the rate of change of a value within a min and max delta.
 *
 * This function is used to prevent abrupt jumps in input readings.
 *
 * @param current    The current input value.
 * @param previous   The previously smoothed value.
 * @param minDelta   Minimum change required to update the value.
 * @param maxDelta   Maximum allowed change between consecutive values.
 * @return int       The new smoothed value after applying rate limiting.
 */
static int limit_change_min_max(int current, int previous, int minDelta, int maxDelta)
{
    int delta = current - previous;

    if (abs(delta) < minDelta)
        return previous;

    if (delta > maxDelta)
        return previous + maxDelta;

    if (delta < -maxDelta)
        return previous - maxDelta;

    return current;
}

/**
 * @brief Initializes internal buffers and state for smoothing logic.
 *
 * This should be called once before any smoothing functions are used.
 */
void Smoother_Init(void)
{
    prevX = 0;
    prevY = 0;
    prevZ = 0;
    last_sent_z = 0;
    z_index = 0;
    z_filled = 0;

    for (int i = 0; i < HEIGHT_AVG_WINDOW; ++i)
        z_buffer[i] = 0;
}

/**
 * @brief Smooths the X tilt input using delta bounds and clamping.
 *
 * Applies rate limiting and clamps the result to [-100, 100].
 *
 * @param rawX       Raw input X value.
 * @param minDelta   Minimum change required to update.
 * @param maxDelta   Maximum allowed change per update.
 * @return int       Smoothed X value.
 */
int SmoothAngleX(int rawX, int minDelta, int maxDelta)
{
    prevX = limit_change_min_max(rawX, prevX, minDelta, maxDelta);

    if (prevX > X_MAX) prevX = X_MAX;
    if (prevX < X_MIN) prevX = X_MIN;

    return prevX;
}

/**
 * @brief Smooths the Y tilt input using delta bounds and clamping.
 *
 * Applies rate limiting and clamps the result to [-100, 100].
 *
 * @param rawY       Raw input Y value.
 * @param minDelta   Minimum change required to update.
 * @param maxDelta   Maximum allowed change per update.
 * @return int       Smoothed Y value.
 */
int SmoothAngleY(int rawY, int minDelta, int maxDelta)
{
    prevY = limit_change_min_max(rawY, prevY, minDelta, maxDelta);

    if (prevY > Y_MAX) prevY = Y_MAX;
    if (prevY < Y_MIN) prevY = Y_MIN;

    return prevY;
}

/**
 * @brief Smooths the Z (height/distance) input using a moving average and deadband.
 *
 * Averages the last N readings, applies max rate limiting, and only updates the
 * output if the new value exceeds a deadband threshold from the last sent value.
 *
 * @param rawZ       Raw Z input value (e.g. from a distance sensor).
 * @param maxDelta   Maximum allowed change from previous value.
 * @return int       Smoothed and deadband-filtered Z value.
 */
int SmoothHeight(int rawZ, int maxDelta)
{
    // Add to moving average buffer
    z_buffer[z_index] = rawZ;
    z_index = (z_index + 1) % HEIGHT_AVG_WINDOW;
    if (z_index == 0) z_filled = 1;

    int count = z_filled ? HEIGHT_AVG_WINDOW : z_index;
    int sum = 0;
    for (int i = 0; i < count; ++i)
        sum += z_buffer[i];
    int avgZ = sum / count;

    // Apply max delta rate limiting
    int limitedZ = limit_change_min_max(avgZ, prevZ, 0, maxDelta);  // No minDelta for Z
    prevZ = limitedZ;

    // Only update if change exceeds deadband
    if (abs(limitedZ - last_sent_z) > Z_DEADBAND)
        last_sent_z = limitedZ;

    return last_sent_z;
}
