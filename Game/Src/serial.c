#include "serial.h"
#include "serialise.h"
#include "stm32f303xc.h"

// Packet reception states
#define PACKET_STATE_WAITING_SENTINEL1  0
#define PACKET_STATE_WAITING_SENTINEL2  1
#define PACKET_STATE_READING_HEADER     2
#define PACKET_STATE_READING_DATA       3

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



// InitialiseSerial - Initialise the serial port
// Input: baudRate is from an enumerated set
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {
    serial_port->completion_function = completion_function;

    // Initialize packet reception state
    serial_port->packet_state = PACKET_STATE_WAITING_SENTINEL1;
    serial_port->packet_index = 0;
    serial_port->expected_length = 0;

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

    // Make sure start not busy
    serial_port->TXbusy = 0;
    serial_port->RXIndex = 0;
    serial_port->TXIndex = 0;
    serial_port->TXLength = 0;
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



// Fixed USART1 interrupt handler for binary packet reception
void USART1_EXTI25_IRQHandler(void) {
    if (USART1->ISR & USART_ISR_RXNE) {
        uint8_t received_byte = USART1->RDR;

        switch (USART1_PORT.packet_state) {
            case PACKET_STATE_WAITING_SENTINEL1:
                if (received_byte == SENTINEL_1) {
                    USART1_PORT.packet_buffer[0] = received_byte;
                    USART1_PORT.packet_index = 1;
                    USART1_PORT.packet_state = PACKET_STATE_WAITING_SENTINEL2;
                }
                break;

            case PACKET_STATE_WAITING_SENTINEL2:
                if (received_byte == SENTINEL_2) {
                    USART1_PORT.packet_buffer[1] = received_byte;
                    USART1_PORT.packet_index = 2;
                    USART1_PORT.packet_state = PACKET_STATE_READING_HEADER;
                } else {
                    // Reset if second sentinel doesn't match
                    USART1_PORT.packet_state = PACKET_STATE_WAITING_SENTINEL1;
                    USART1_PORT.packet_index = 0;
                }
                break;

            case PACKET_STATE_READING_HEADER:
                USART1_PORT.packet_buffer[USART1_PORT.packet_index++] = received_byte;

                // Header is 6 bytes total (2 sentinel + 2 message_type + 2 data_length)
                if (USART1_PORT.packet_index >= 6) {
                    // Extract data length from header (little-endian)
                    USART1_PORT.expected_length =
                        (USART1_PORT.packet_buffer[5] << 8) | USART1_PORT.packet_buffer[4];

                    if (USART1_PORT.expected_length == 0) {
                        // No data, process packet immediately
                        if (USART1_PORT.completion_function) {
                            USART1_PORT.completion_function(USART1_PORT.packet_index);
                        }
                        USART1_PORT.packet_state = PACKET_STATE_WAITING_SENTINEL1;
                        USART1_PORT.packet_index = 0;
                    } else if (USART1_PORT.expected_length <= (256 - 6)) {
                        // Valid data length, continue to read data
                        USART1_PORT.packet_state = PACKET_STATE_READING_DATA;
                    } else {
                        // Invalid data length, reset
                        USART1_PORT.packet_state = PACKET_STATE_WAITING_SENTINEL1;
                        USART1_PORT.packet_index = 0;
                    }
                }
                break;

            case PACKET_STATE_READING_DATA:
                USART1_PORT.packet_buffer[USART1_PORT.packet_index++] = received_byte;

                // Check if we've received all expected data
                if (USART1_PORT.packet_index >= (6 + USART1_PORT.expected_length)) {
                    // Complete packet received, process it
                    if (USART1_PORT.completion_function) {
                        USART1_PORT.completion_function(USART1_PORT.packet_index);
                    }

                    // Reset for next packet
                    USART1_PORT.packet_state = PACKET_STATE_WAITING_SENTINEL1;
                    USART1_PORT.packet_index = 0;
                }
                break;
        }
    }

    // TX interrupt handling
    if (USART1->ISR & USART_ISR_TXE) {
        if (USART1_PORT.TXIndex < USART1_PORT.TXLength) {
            USART1->TDR = USART1_PORT.TXBuffer[USART1_PORT.TXIndex++];
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;
            USART1_PORT.TXIndex = 0;
            USART1_PORT.TXLength = 0;
            USART1_PORT.TXbusy = 0;
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



