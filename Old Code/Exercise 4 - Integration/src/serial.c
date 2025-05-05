#include "serial.h"
#include "dio.h"
#include "timer_module.h"
#include "string.h"
#include "stm32f303xc.h"
#include "bin2int.h"


/*
* this module uses USART1 on
 * It supports interrupt-driven transmission 
 * and double-buffered reception, allowing non-blocking data handling. 
 * The received commands trigger LED operations or timer-based tasks.
 
 * Key Features:
 * - double buffer implementation for USART reception
 * - interrupt-driven data transmission
 * - dynamic command parsing and execution ("led", "timer", "oneshot")
 * - optional completion callback after data transmission

*/


//transmitting buffer
volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_index = 0;
volatile uint8_t tx_length = 0;

//Receiving buffers for double buffer interrupt function
volatile uint8_t buffer1[RX_BUFFER_SIZE];
volatile uint8_t buffer2[RX_BUFFER_SIZE];

// Variables to switch buffers in double buffer implementation
volatile uint8_t *active_buffer = buffer1;
volatile uint8_t *processing_buffer = buffer2;

volatile uint8_t processing_flag = 1;
volatile uint8_t transmitting = 0;



// We store the pointers to the GPIO and USART that are used
//  for a specific serial port. To add another serial port
//  you need to select the appropriate values.
struct _SerialPort {
	USART_TypeDef *UART;
	GPIO_TypeDef *GPIO;
	volatile uint32_t MaskAPB2ENR;	// mask to enable RCC APB2 bus registers
	volatile uint32_t MaskAPB1ENR;	// mask to enable RCC APB1 bus registers
	volatile uint32_t MaskAHBENR;	// mask to enable RCC AHB bus registers
	volatile uint32_t SerialPinModeValue;
	volatile uint32_t SerialPinSpeedValue;
	volatile uint32_t SerialPinAlternatePinValueLow;
	volatile uint32_t SerialPinAlternatePinValueHigh;
	void (*completion_function)(uint32_t, char *);
};


// instantiate the serial port parameters
//   note: the complexity is hidden in the c file
SerialPort USART1_PORT = {USART1,
	GPIOC,
	RCC_APB2ENR_USART1EN, // bit to enable for APB2 bus
	0x00,	// bit to enable for APB1 bus
	RCC_AHBENR_GPIOCEN, // bit to enable for AHB bus
	0xA00,
	0xF00,
	0x770000,  // for USART1 PC10 and 11, this is in the AFR low register
	0x00, // no change to the high alternate function register
	0x00 // default function pointer is NULL
	};


// InitialiseSerial - Initialise the serial port
// Input: baudRate is from an enumerated set
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t, char *)) {

	serial_port->completion_function = completion_function;

	// enable clock power, system configuration clock and GPIOC
	// common to all UARTs
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// enable the GPIO which is on the AHB bus
	RCC->AHBENR |= serial_port->MaskAHBENR;

	// set pin mode to alternate function for the specific GPIO pins
	serial_port->GPIO->MODER = serial_port->SerialPinModeValue;

	// enable high speed clock for specific GPIO pins
	serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;

	// set alternate function to enable USART to external pins
	serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;
	serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;

	// enable the device based on the bits defined in the serial port definition
	RCC->APB1ENR |= serial_port->MaskAPB1ENR;
	RCC->APB2ENR |= serial_port->MaskAPB2ENR;

	// Get a pointer to the 16 bits of the BRR register that we want to change
	uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR; // only 16 bits used!

	// Baud rate calculation from datasheet
	switch(baudRate){
	case BAUD_9600:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x341;  // 115200 at 8MHz
		break;
	case BAUD_19200:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x1A1;  // 115200 at 8MHz
		break;
	case BAUD_38400:
		// NEED TO FIX THIS !
		*baud_rate_config = 0xD0;  // 115200 at 8MHz
		break;
	case BAUD_57600:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x8B;  // 115200 at 8MHz
		break;
	case BAUD_115200:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	}


	// enable serial port for tx and rx
	serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void finished_transmission(uint32_t bytes_sent, char *sent_string) {
	// This function will be called after a transmission is complete

	volatile uint32_t test = 0;
	// make a very simple delay
	for (volatile uint32_t i = 0; i < 0x8ffff; i++) {
		// waste time !
	}
}

void start_interrupt_transmission(SerialPort *serial_port, uint8_t *data, uint8_t size){

	if (transmitting || (size + 4) == 0) return;

	transmitting = 1;

	memset((void *)tx_buffer, 0, TX_BUFFER_SIZE);
	tx_buffer[0] = '\r';
	tx_buffer[1] = '\n';

	memcpy((void *)(tx_buffer+2), data, size);

	tx_buffer[size + 1] = '\r';
	tx_buffer[size + 2] = '\n';
	//tx_buffer[size] = '\0';
	tx_length = size + 4;

	tx_index = 0;
	serial_port->UART->TDR = tx_buffer[tx_index];
	USART1->CR1 |= USART_CR1_TXEIE;

}

void finished_receiving(uint8_t num_characters, char *received_string){
    processing_flag = 0;

    uint8_t *finished_op = "Finished task. Send new prompt!";

    if (strncmp(received_string, "serial ", 7) == 0) {
        received_string += 7;
        start_interrupt_transmission(&USART1_PORT, received_string, strlen(received_string));
    }
    else if (strncmp(received_string, "led ", 4) == 0) {
        received_string += 4;

        // Remove trailing newline characters
        char *newline = strchr(received_string, '\r');
        if (newline) *newline = '\0';

        int mask = binary_to_int(received_string);
        dio_set_led_state(mask);

        start_interrupt_transmission(&USART1_PORT, finished_op, strlen(finished_op));
    }
    else if (strncmp(received_string, "oneshot ", 8) == 0) {
        received_string += 8;
        uint32_t delay = atoi(received_string);
        start_oneshot(delay, dio_invert_leds);

        start_interrupt_transmission(&USART1_PORT, finished_op, strlen(finished_op));
    }
    else if (strncmp(received_string, "timer ", 6) == 0) {
        received_string += 6;
        uint32_t period = atoi(received_string);
        dio_start_blinking();
        timer_init(period, dio_toggle_leds);

        start_interrupt_transmission(&USART1_PORT, finished_op, strlen(finished_op));
    }
    else {
        uint8_t *error_message = "Invalid string entered. Please try again!\r\n";
        start_interrupt_transmission(&USART1_PORT, (uint8_t *)error_message, strlen(error_message));
    }

    processing_flag = 1;
}




void SerialOutputChar(uint8_t data, SerialPort *serial_port) {

	while((serial_port->UART->ISR & USART_ISR_TXE) == 0){
	}

	serial_port->UART->TDR = data;
}

void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {

	uint32_t counter = 0;
	char start_string = pt;
	while(*pt) {
		SerialOutputChar(*pt, serial_port);
		counter++;
		pt++;
	}

	serial_port->completion_function(counter, (char *)start_string);
}

void USART1_EXTI25_IRQHandler(void){

	if (USART1_PORT.UART->ISR & USART_ISR_RXNE){

		static uint8_t rx_index = 0;
		uint8_t *overflow_error = "Message too long! Reset the STM and send a shorter string!";
		char received_byte = USART1_PORT.UART->RDR;

		// Store byte into active buffer if not filled

		if(rx_index < RX_BUFFER_SIZE -2){
			active_buffer[rx_index] = received_byte;
			rx_index++;
		}

		// Check if buffer swap ready
		if (received_byte == '\r'){

			active_buffer[rx_index] = '\0';  			// Needs \0 for strlen function
			if(processing_flag == 1 && transmitting == 0){		// Check both buffers

				//Swap buffers
				volatile uint8_t * temp = active_buffer;
				active_buffer = processing_buffer;
				processing_buffer = temp;

				//Store length of received string
				uint8_t length = rx_index;
				rx_index = 0;

				//reset flags and send buffer to process
				processing_flag = 0;
				USART1_PORT.completion_function(length, processing_buffer);
			}
			//Discard received string if still processing other buffer
			else{
				rx_index = 0;
			}
		}
		//Handle buffer overflow - send error message, reset buffer
		if(rx_index >= RX_BUFFER_SIZE -2){
			start_interrupt_transmission(&USART1_PORT, overflow_error, strlen(overflow_error));
			rx_index = 0;
			memset((void*)active_buffer, 0, RX_BUFFER_SIZE);
		}

	}

	if (USART1_PORT.UART->ISR & USART_ISR_TXE){

		if(tx_index < tx_length){
			USART1_PORT.UART->TDR = tx_buffer[tx_index + 1];
			tx_index++;
		}
		else {
			//reset flags and disable transmission
			tx_index = 0;
			tx_length = 0;
			transmitting = 0;
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}

}



void enable_USART_interrupt() {
	// Disable the interrupts while messing around with the settings
	//  otherwise can lead to strange behaviour
	__disable_irq();

	//enable the USART 1 clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	//RXNE interrupt enable
	USART1->CR1 |= USART_CR1_RXNEIE;
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE; //Enable receiver + transmitter
	USART1->CR1 |= USART_CR1_UE; //enable USART


	// Tell the NVIC module that USART1 interrupts should be handled
	NVIC_SetPriority(USART1_IRQn, 1);  // Set Priority
	NVIC_EnableIRQ(USART1_IRQn);

	// Re-enable all interrupts (now that we are finished)
	__enable_irq();
}



void SerialInputChar(uint8_t *data, SerialPort *serial_port){

	while((serial_port->UART->ISR & USART_ISR_RXNE) == 0){
	}
	*data = (uint8_t)(serial_port->UART->RDR);
}

void SerialInputString(uint8_t buffer_size, SerialPort *serial_port, char *buffer){

	uint8_t buffer_counter = 0;
	uint8_t character;
	while (1){
		SerialInputChar(&character, serial_port);
		if (character == '!'){
			break;
		}
		if (buffer_counter < buffer_size-1){
			buffer[buffer_counter] = character;
			buffer_counter++;
		}
	}
	buffer[buffer_counter] = '!';
	serial_port->completion_function(buffer_counter, buffer);
}
