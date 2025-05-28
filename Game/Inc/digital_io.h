#ifndef DIGITAL_H_
#define DIGITAL_H_

#include <stdint.h>

/**
 * Initialize digital I/O module with a callback for button press
 * on_button_press Function to call when button is pressed
 */
void DigitalInitialise(void (*callback)());

//Get current state of bitmask and return it
uint8_t GetLEDBitmask();

// Set the LED state using a bitmask
void SetLEDBitmask(uint8_t mask);


#endif /* DIGITAL_H_ */
