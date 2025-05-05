#ifndef DIO_H
#define DIO_H

#include <stdint.h>

// Initialise digital I/O (LEDs and button interrupt)
void dio_init(void);

// Set the LED state (bitmask)
void dio_set_led_state(uint8_t state);

// Get the current LED state (bitmask)
uint8_t dio_get_led_state(void);

// Set the minimum time (in ms) between LED updates
void dio_set_led_rate(uint32_t rate_ms);

// Start blinking the current LED pattern
void dio_start_blinking(void);

// Stop any blinking behaviour
void dio_stop_blinking(void);

// Toggle LED visibility on/off (called by timer)
void dio_toggle_leds(void);

// Invert the current LED pattern
void dio_invert_leds(void);

// Update blink pattern if blinking is active
void dio_update_blink_mask_if_active(void);

#endif // DIO_H
