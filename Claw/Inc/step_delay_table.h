/**
 * @file step_delay_table.h
 * @brief Delay lookup table for stepper motion acceleration profiles.
 *
 * Provides a fixed 100-entry table mapping velocity index to timer delay.
 * This table is used by the stepper controller to determine pulse intervals
 * for smooth velocity ramping. Intended for 40 µs timer resolution.
 */

#ifndef STEP_DELAY_TABLE_H
#define STEP_DELAY_TABLE_H

#include <stdint.h>

// Delay values in timer ticks (for 40 µs resolution)
extern const uint16_t delay_table[100];

#endif // STEP_DELAY_TABLE_H
