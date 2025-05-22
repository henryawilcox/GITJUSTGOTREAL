#include "serial.h"
#include "stepper_timer.h"
#include "stm32f303xc.h"
#define BUFFER_SIZE 64
#include "servo.h"
// We store the pointers to the GPIO and USART that are used
//  for a specific serial port. To add another serial port
//  you need to select the appropriate values.

// instantiate the serial port parameters
//   note: the complexity is hidden in the c file
SerialPort USART1_PORT = {USART1,
		GPIOC,
		RCC_APB2ENR_USART1EN, // bit to enable for APB2 bus
		0x00,	// bit to enable for APB1 bus
		RCC_AHBENR_GPIOCEN, // bit to enable for AHB bus
		0x0A00,  // value for pins to change to 'alternate function mode'
		0x0F00,  // value for pins to enable 'high speed'
		0x770000,  // for USART1 PC10 and 11, this is in the AFR low register
		0x00, // no change to the high alternate function register
		{}, // RX string
		0x00, //RX index
		{}, // TX string
		0x00, // TX index
		1, // default is ready

		0x00 // default function pointer is NULL
};

//SerialPort USART1_PORT = {
//    USART1,
//    GPIOC,
//    RCC_APB2ENR_USART1EN,     // USART1 on APB2
//    0x00,                      // not used
//    RCC_AHBENR_GPIOCEN,       // Enable GPIOC
//    0x00A000,                 // MODER: PC4 and PC5 to AF mode (10b at bits 8-11)
//    0x00F000,                 // OSPEEDR: PC4 and PC5 to high speed
//    0x00007700,               // AFRL: AF7 for PC4 (bits 19:16) and PC5 (bits 23:20)
//    0x00,                     // AFRH unused
//    {},                       // RX buffer
//    0,
//    {},                       // TX buffer
//    0,
//    1,                        // string ready
//    0x00                      // callback function
//};


SerialPort USART2_PORT = {
    USART2,
    GPIOA,
    RCC_APB1ENR_USART2EN,
    0x00,  // not needed for USART2
    RCC_AHBENR_GPIOAEN,
    0x00A0,  // PA2 (TX), PA3 (RX) alternate function mode (bits 5:4 and 7:6 = 10)
    0x00F0,  // PA2 and PA3 high speed
    0x07700000,  // AFRL: AF7 for PA2 and PA3 (bits 11:8 and 15:12 = 0111)
    0x00,       // AFRH
    {},
    0,
    {},
    0,
    1,
    0x00
};



// InitialiseSerial - Initialise the serial port
// Input: baudRate is from an enumerated set

void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {

	serial_port->completion_function = completion_function;

	// enable clock power, system configuration clock and GPIOC
	// common to all UARTs
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// enable the GPIO which is on the AHB bus
	RCC->AHBENR |= serial_port->MaskAHBENR;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR |= (0xF << 4);       // High speed on PA2 and PA3
	// Clear bits 4–7 (PA2 and PA3), then set to alternate function (10)
	GPIOA->MODER &= ~(0xF << 4);        // Clear PA2 and PA3 mode
	GPIOA->MODER |=  (0xA << 4);        // Set PA2, PA3 to alternate function (10)


	// set pin mode to alternate function for the specific GPIO pins
	serial_port->GPIO->MODER = serial_port->SerialPinModeValue;

	// enable high speed clock for specific GPIO pins
	serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;

	// set alternate function to enable USART to external pins
	serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;
	serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;

	GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));  // Clear bits for PA2, PA3
	GPIOA->AFR[0] |=  ((0x7 << (4 * 2)) | (0x7 << (4 * 3)));  // Set AF7 for PA2, PA3


	// enable the device based on the bits defined in the serial port definition
	RCC->APB1ENR |= serial_port->MaskAPB1ENR;
	RCC->APB2ENR |= serial_port->MaskAPB2ENR;

	// Get a pointer to the 16 bits of the BRR register that we want to change
	uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR; // only 16 bits used!

	switch(baudRate){
		case BAUD_9600:
			*baud_rate_config = 0x341;
			break;
		case BAUD_19200:
			*baud_rate_config = 0x1A1;
			break;
		case BAUD_38400:
			*baud_rate_config = 0xD1;
			break;
		case BAUD_57600:
			*baud_rate_config = 0x8B;
			break;
		case BAUD_115200:
			*baud_rate_config = 0x46;
			break;
	}


	// enable serial port for tx and rx
	serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}


void enable_interrupt_USART1_PC11() {
    __disable_irq();

    // Enable NVIC interrupt for USART1
    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);

    // Enable RXNE (receive register not empty) interrupt
    USART1->CR1 |= USART_CR1_RXNEIE;

    __enable_irq();
}

void enable_interrupt_USART2_PA3() {
    __disable_irq();
    NVIC_SetPriority(USART2_IRQn, 5);
    NVIC_EnableIRQ(USART2_IRQn);
    USART2->CR1 |= USART_CR1_RXNEIE;
    __enable_irq();
}


#define PACKET_SIZE 8
volatile uint8_t rx_packet[PACKET_SIZE];
volatile uint8_t rx_index = 0;

void USART1_EXTI25_IRQHandler(void) {
    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;

        if (rx_index == 0 && c != 0xAA) {
            return;  // Wait for start byte
        }

        rx_packet[rx_index++] = c;

        if (rx_index >= PACKET_SIZE) {
            rx_index = 0;

            // Dispatch based on command
            switch (rx_packet[1]) {
                case 0x01: {
                    int8_t vx = rx_packet[2];
                    int8_t vy = rx_packet[3];
                    int8_t i = rx_packet[4];
                    uint16_t z_target = (rx_packet[6] << 8) | rx_packet[5];  // Little endian
                    plan_velocity(vx, vy, i);  // Use your existing logic
                    set_z_target(z_target);
                    break;
                }
                case 0x02:
                	Servo_Close();
                    // future: home command
                    break;
                case 0x03:
                	Servo_Open();
                    // future: return current position
                    break;
            }
        }
    }
}


void USART2_EXTI26_IRQHandler(void) {
	SerialOutputString((uint8_t *)"recieved\r\n", &USART2_PORT);

    if (USART2->ISR & USART_ISR_RXNE) {
        char c = USART2->RDR;

        if (USART2_PORT.RXIndex < BUFFER_SIZE - 2) {  // -2 to fit \n and \0
            if (c == '\r') {
                USART2_PORT.RXBuffer[USART2_PORT.RXIndex++] = '\0';

                if (USART2_PORT.completion_function) {
					USART2_PORT.completion_function(USART2_PORT.RXIndex);
                }
                //  Copy to TX buffer


                //  Start TX
                // Enable TX interrupt
                USART2_PORT.string_ready = 1;

                // Reset RX buffer
                USART2_PORT.RXIndex = 0;


            }else{
                USART2_PORT.RXBuffer[USART2_PORT.RXIndex++] = c;
            }
        } else {
            USART2_PORT.RXIndex = 0; // Overflow safety
        }
    }
}

//void USART2_EXTI25_IRQHandler(void) {
//    if (USART2->ISR & USART_ISR_RXNE) {
//        char c = USART2->RDR;
//
//        if (rx_index == 0 && c != 0xAA) {
//            return;
//        }
//
//        rx_packet[rx_index++] = c;
//
//        if (rx_index >= PACKET_SIZE) {
//            rx_index = 0;
//
//            switch (rx_packet[1]) {
//                case 0x01: {
//                    int8_t vx = rx_packet[2];
//                    int8_t vy = rx_packet[3];
//                    int8_t i = rx_packet[4];
//                    uint16_t z_target = (rx_packet[6] << 8) | rx_packet[5];
//                    plan_velocity(vx, vy, i);
//                    set_z_target(z_target);
//                    break;
//                }
//                case 0x02:
//                    Servo_Close();
//                    break;
//                case 0x03:
//                    Servo_Open();
//                    break;
//            }
//        }
//    }
//}



// Clear framing and overrun errors to allow fresh reception
void Clear_error(SerialPort *serial_port){
    serial_port->UART->ICR |= USART_ICR_ORECF | USART_ICR_FECF;
}


// Waits for one character to be received via polling
void SerialInputChar(uint8_t *pt, SerialPort *serial_port) {

    // Check for framing or overrun errors before reading
    if ((serial_port->UART->ISR & (USART_ISR_FE | USART_ISR_ORE)) != 0) {
        Clear_error(serial_port);
    }

    // Wait until a character is received
    while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0) {
    }

    // Read received character
    *pt = serial_port->UART->RDR;

    // Clear the RXNE flag manually (optional but safe practice)
    serial_port->UART->RQR |= USART_RQR_RXFRQ;
}


// Polls for an entire string, terminated by '\r'
void SerialInputString(uint8_t *pt, SerialPort *serial_port) {
    uint32_t counter = 0;
    while (1) {

        SerialInputChar(pt, serial_port);

        if (*pt == '\r') {     // Stop when carriage return is received
            *pt = '\0';        // Null-terminate the string
            break;
        }

        counter++;
        pt++;
    }

    SerialOutputString((uint8_t*)"\r\n", serial_port);

//    SerialOutputString("\r\n", serial_port);           // Echo newline after input
    serial_port->completion_function(counter);
}



void SerialOutputChar(uint8_t data, SerialPort *serial_port) {

	while((serial_port->UART->ISR & USART_ISR_TXE) == 0){
	}

	serial_port->UART->TDR = data;
}

void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {

	uint32_t counter = 0;
	while(*pt) {
		SerialOutputChar(*pt, serial_port);
		counter++;
		pt++;
	}

	SerialOutputChar('\n', serial_port);
	SerialOutputChar('\r', serial_port);
	//serial_port->completion_function(counter);
}
