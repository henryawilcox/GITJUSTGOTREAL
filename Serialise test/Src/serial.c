#include "serial.h"

#include "stm32f303xc.h"


// NOTE: these are stored as pointers because they
//       are const values so we can't store them directly
//       in the struct

// The different serial ports require different GPIO ports
enum {
	SERIAL_GPIO_A,
	SERIAL_GPIO_B,
	SERIAL_GPIO_C,
	SERIAL_GPIO_D,
	SERIAL_GPIO_E
};



// instantiate the serial port parameters
//   note: the complexity is hidden in the c file
SerialPort USART1_PORT = {
	&(USART1->BRR),
	&(USART1->CR1),
	&(USART1->ICR),
	&(USART1->ISR),
	&(USART1->TDR),
	&(USART1->RDR),
	&(RCC->APB2ENR),
	RCC_APB2ENR_USART1EN,
	SERIAL_GPIO_C,

	// MODER for PC4,5
	&(GPIOC->MODER),
	(2 << (4 * 2)) | (2 << (5 * 2)),           // AF mode
	(3 << (4 * 2)) | (3 << (5 * 2)),           // MODER mask

	// OSPEEDR
	&(GPIOC->OSPEEDR),
	(3 << (4 * 2)) | (3 << (5 * 2)),           // High speed
	(3 << (4 * 2)) | (3 << (5 * 2)),           // OSPEEDR mask

	// AFRH for PC4/5 is AFR[1]
	&(GPIOC->AFR[0]),
	(7 << (4 * 4)) | (7 << (4 * 5)),// AF7
	(0xF << (4 * 4)) | (0xF << (4 * 5)) // Mask
};

SerialPort USART2_PORT = {
	&(USART2->BRR),
	&(USART2->CR1),
	&(USART2->ICR),
	&(USART2->ISR),
	&(USART2->TDR),
	&(USART2->RDR),
	&(RCC->APB1ENR),
	RCC_APB1ENR_USART2EN,
	SERIAL_GPIO_A,

	// MODER for PA2, PA3
	&(GPIOA->MODER),
	(2 << (2 * 2)) | (2 << (3 * 2)),             // AF mode
	(3 << (2 * 2)) | (3 << (3 * 2)),             // MODER mask

	// OSPEEDR
	&(GPIOA->OSPEEDR),
	(3 << (2 * 2)) | (3 << (3 * 2)),             // High speed
	(3 << (2 * 2)) | (3 << (3 * 2)),             // OSPEEDR mask

	// AFRL for PA2/PA3 is AFR[0]
	&(GPIOA->AFR[0]),
	(7 << (4 * 2)) | (7 << (4 * 3)),             // AF7
	(0xF << (4 * 2)) | (0xF << (4 * 3))          // Mask
};


// InitialiseSerial - Initialise the serial port
// Input: baudRate is from an enumerated set
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {
	serial_port->completion_function = completion_function;

	// Enable clocks
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	switch (serial_port->SerialPortGPIO) {
		case SERIAL_GPIO_A:
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
			break;
		case SERIAL_GPIO_C:
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
			break;
	}

	// Set MODER (alternate function)
	uint32_t moder = *(serial_port->SerialPinModeRegister);
	moder &= ~serial_port->SerialPinModeMask;
	moder |= serial_port->SerialPinModeValue;
	*(serial_port->SerialPinModeRegister) = moder;

	// Set OSPEEDR (high speed)
	uint32_t ospeed = *(serial_port->SerialPinSpeedRegister);
	ospeed &= ~serial_port->SerialPinSpeedMask;
	ospeed |= serial_port->SerialPinSpeedValue;
	*(serial_port->SerialPinSpeedRegister) = ospeed;

	// Set AFR (alternate function 7 for USART)
	uint32_t afr = *(serial_port->SerialPinAlternateRegister);
	afr &= ~serial_port->SerialPinAlternateMask;
	afr |= serial_port->SerialPinAlternateValue;
	*(serial_port->SerialPinAlternateRegister) = afr;

	// Enable USART peripheral
	*(serial_port->TimerEnableRegister) |= serial_port->TimerEnableMask;

	// Set baud rate (assuming 8 MHz clock)
	uint16_t *baud_rate_config = (uint16_t *)serial_port->BaudRate;

	switch (baudRate) {
		case BAUD_9600:   *baud_rate_config = 0x341; break;
		case BAUD_19200:  *baud_rate_config = 0x1A1; break;
		case BAUD_38400:  *baud_rate_config = 0xD1;  break;
		case BAUD_57600:  *baud_rate_config = 0x8B;  break;
		case BAUD_115200: *baud_rate_config = 0x46;  break;
	}

	// Enable TX, RX and USART
	*(serial_port->ControlRegister1) |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

	//make sure start not busy
	serial_port->TXbusy=0;
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

    NVIC_SetPriority(USART2_IRQn, 4);
    NVIC_EnableIRQ(USART2_IRQn);

    USART2->CR1 |= USART_CR1_RXNEIE;

    __enable_irq();
}


void USART1_EXTI25_IRQHandler(void) {

    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;

        if (USART1_PORT.RXIndex < BUFFER_SIZE - 2) {  // -2 to fit \n and \0
            if (c == '\r') {
                USART1_PORT.RXBuffer[USART1_PORT.RXIndex++] = '\0';

                if (USART1_PORT.completion_function) {
					USART1_PORT.completion_function(USART1_PORT.RXIndex);
                }

                SerialOutputStringInterrupt(USART1_PORT.RXBuffer, &USART2_PORT);
//                SerialOutputString(USART1_PORT.RXBuffer, &USART1_PORT);

                //not ideal but makeser sure it waits before sending to usart port 1
                while (USART2_PORT.TXbusy);
                SerialOutputStringInterrupt(USART1_PORT.RXBuffer, &USART1_PORT);
                //  Copy to TX buffer


                //  Start TX
                // Enable TX interrupt
//                USART1_PORT.string_ready = 1;

                // Reset RX buffer
                USART1_PORT.RXIndex = 0;


            }else{
                USART1_PORT.RXBuffer[USART1_PORT.RXIndex++] = c;
            }
        } else {
            USART1_PORT.RXIndex = 0; // Overflow safety
        }
    }


	if (USART1->ISR & USART_ISR_TXE) {
		if (USART1_PORT.TXIndex < USART1_PORT.TXLength) {
			USART1->TDR = USART1_PORT.TXBuffer[USART1_PORT.TXIndex++];
		} else {
			// Transmission complete
			USART1->CR1 &= ~USART_CR1_TXEIE; // Disable TXE interrupt
			USART1_PORT.TXIndex = 0;
			USART1_PORT.TXLength = 0;
			USART1_PORT.TXbusy = 0;
		}
	}
}

void USART2_EXTI26_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        char c = USART2->RDR;

        if (USART2_PORT.RXIndex < BUFFER_SIZE - 2) {
            if (c == '\r') {
                USART2_PORT.RXBuffer[USART2_PORT.RXIndex++] = '\0';

                if (USART2_PORT.completion_function) {
                    USART2_PORT.completion_function(USART2_PORT.RXIndex);
                }

                //relay message
                SerialOutputStringInterrupt(USART2_PORT.RXBuffer, &USART1_PORT);
                USART2_PORT.RXIndex = 0;
            } else {
                USART2_PORT.RXBuffer[USART2_PORT.RXIndex++] = c;
            }
        } else {
            USART2_PORT.RXIndex = 0; // overflow safety
        }
    }

    if (USART2->ISR & USART_ISR_TXE) {
    		if (USART2_PORT.TXIndex < USART2_PORT.TXLength) {
    			USART2->TDR = USART1_PORT.TXBuffer[USART2_PORT.TXIndex++];
    		} else {
    			// Transmission complete
    			USART2->CR1 &= ~USART_CR1_TXEIE; // Disable TXE interrupt
    			USART2_PORT.TXIndex = 0;
    			USART2_PORT.TXLength = 0;
    			USART2_PORT.TXbusy = 0;
    		}
    	}
}






void SerialOutputStringInterrupt(uint8_t *pt, SerialPort *serial_port) {
	if (serial_port->TXbusy) {
			return;  // TX already in progress, drop this transmission
		}


	uint8_t i = 0;
	while (pt[i] != '\0' && i < BUFFER_SIZE - 1) {
		serial_port->TXBuffer[i] = pt[i];
		i++;
	}
	serial_port->TXBuffer[i] = '\0'; // null-terminate
	serial_port->TXLength = i;
	serial_port->TXIndex = 0;
	serial_port->TXbusy = 1;
	// Enable TXE interrupt
	*(serial_port->ControlRegister1) |= USART_CR1_TXEIE;

}

// Interrupt-based buffer output for binary data
void SerialOutputBufferInterrupt(uint8_t *buffer, uint16_t buffer_length, SerialPort *serial_port) {
    if (serial_port->TXbusy) {
        return;  // TX already in progress, drop this transmission
    }

    // Copy buffer data (not null-terminated)
    uint16_t copy_length = (buffer_length < BUFFER_SIZE) ? buffer_length : BUFFER_SIZE;

    for (uint16_t i = 0; i < copy_length; i++) {
        serial_port->TXBuffer[i] = buffer[i];
    }

    serial_port->TXLength = copy_length;
    serial_port->TXIndex = 0;
    serial_port->TXbusy = 1;

    // Enable TXE interrupt
    *(serial_port->ControlRegister1) |= USART_CR1_TXEIE;
}



