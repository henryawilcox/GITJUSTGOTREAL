

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "serial.h"              // UART setup and transmit functions
#include "stm32f303xc.h"         // Device-specific header for STM32F3 series
#include "serial_interrupt.h"    // Interrupt-based RX handler


// This function is called when a complete string is received over UART
void callback1(uint8_t *string_buffer, uint8_t length) {
    // Display the received message
    SerialSendString((uint8_t*)"You entered: ", &USART1_PORT);
    SerialSendString(string_buffer, &USART1_PORT);
    SerialSendString((uint8_t*)"\r\n\r\n", &USART1_PORT);

    SerialSendString((uint8_t*)"Sending to secondary board \r\n", &USART1_PORT);
    SerialSendString(string_buffer, &USART2_PORT);

    // Prompt the user to enter new input
    SerialSendString((uint8_t*)"Enter text (RETURN to terminate):\r\n", &USART1_PORT);
}

void callback2(uint8_t *string_buffer, uint8_t length) {
	 SerialSendString((uint8_t*)"Data received:", &USART1_PORT);
	 SerialSendString(string_buffer, &USART1_PORT);
	 SerialSendString((uint8_t*)"\r\n\r\n", &USART1_PORT);
}



//main for transmit
int main(void) {
    // Initialize the serial port with baud rate 115200 and a pointer to the callback
    SerialInitialise(BAUD_115200, &USART1_PORT, &callback1);
    SerialInitialise(BAUD_115200, &USART2_PORT, 0);

    // Send welcome messages over UART
    SerialSendString((uint8_t*)"UART ACTIVATED\r\n", &USART1_PORT);
    SerialSendString((uint8_t*)"Enter text (RETURN to terminate):\r\n", &USART1_PORT);

    // Enable RX interrupt-based input
    EnableSerialInterrupts(&USART1_PORT);
    EnableSerialInterrupts(&USART2_PORT);

    // Infinite loop: main code does not need to do anything here
    // since everything is handled through interrupts
    for(;;) {

    }
}

//main for receive
/*int main(void) {
	// Initialize the serial port with baud rate 115200 and a pointer to the callback
	    SerialInitialise(BAUD_115200, &USART1_PORT, 0);
	    SerialInitialise(BAUD_115200, &USART2_PORT, &callback2);

	    // Send welcome messages over UART
	    SerialSendString((uint8_t*)"UART ACTIVATED\r\n", &USART1_PORT);
	    SerialSendString((uint8_t*)"WAITING TO RECIEVE\r\n", &USART1_PORT);

	    // Enable RX interrupt-based input
	    EnableSerialInterrupts(&USART2_PORT);

	    // Infinite loop: main code does not need to do anything here
	    // since everything is handled through interrupts
	    for(;;) {

	    }
}
*/

