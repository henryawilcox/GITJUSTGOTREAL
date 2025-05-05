#include "serial.h"
#include "stm32f303xc.h"

// Serial Port and USART struct type to enable different ports
// Note only one completion function per UART can be selected
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

// Instantiate USART1 serial port parameters and struct
SerialPort USART1_PORT = {USART1,
	GPIOC,
	RCC_APB2ENR_USART1EN, 	// bit to enable for APB2 bus
	0x00,			// bit to enable for APB1 bus
	RCC_AHBENR_GPIOCEN, 	// bit to enable for AHB bus
	0xA00,			// Pin mod value for alternate function
	0xF00,			// High speed setting 
	0x770000,  		// for USART1 PC10 and 11, this is in the AFR low register
	0x00, 			// no change to the high alternate function register
	0x00 			// default function pointer is NULL
	};


// InitialiseSerial - function to initialise serial port
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

	// Baud rates to select
	switch(baudRate){
	case BAUD_9600:
		*baud_rate_config = 0x341;  // 9600 at 8MHz
		break;
	case BAUD_19200:
		*baud_rate_config = 0x1A1;  // 19200 at 8MHz
		break;
	case BAUD_38400:
		*baud_rate_config = 0xD0;  // 38400 at 8MHz
		break;
	case BAUD_57600:
		*baud_rate_config = 0x8B;  // 57600 at 8MHz
		break;
	case BAUD_115200:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	}
	// enable serial port for tx and rx
	serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}



//Transmission completion function. Short delay to prevent other operations happening too quickly afterwards
void finished_transmission(uint32_t bytes_sent, char *sent_string) {
	
	volatile uint32_t test = 0;
	for (volatile uint32_t i = 0; i < 0x8ffff; i++) {
		//delay
	}
}

//Function to transmit a byte via polling method, called in SerialOutputString
void SerialOutputChar(uint8_t data, SerialPort *serial_port) {

	while((serial_port->UART->ISR & USART_ISR_TXE) == 0){
	}

	serial_port->UART->TDR = data;
}

//Function to transmit a string via polling method
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

//Receiving completion function to process data
void finished_receiving(uint8_t num_characters, char *received_string){

	//do tasks here!

	//Transmit completion message after task has finished
	uint8_t *finished_op = "Finished task. Send new prompt!";
	SerialOutputString(finished_op, &USART1_PORT);

}

//Function to receive a byte from the RDR (Receive data register)
void SerialInputChar(uint8_t *data, SerialPort *serial_port){

	while((serial_port->UART->ISR & USART_ISR_RXNE) == 0){
	}
	*data = (uint8_t)(serial_port->UART->RDR);
}

//Function to Receive String over selected UART Port
//Receives data until carriage return is detected
void SerialInputString(uint8_t buffer_size, SerialPort *serial_port, char *buffer){

	uint8_t buffer_counter = 0;
	uint8_t character;
	while (1){
		SerialInputChar(&character, serial_port);
		if (character == '\r'){
			break;
		}
		if (buffer_counter < buffer_size-1){
			buffer[buffer_counter] = character;
			buffer_counter++;
		}
	}
	buffer[buffer_counter] = '\0';
	serial_port->completion_function(buffer_counter, buffer);
}

