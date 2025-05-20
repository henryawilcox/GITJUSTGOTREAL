#include "serial.h"
#include "stm32f303xc.h"

// Define the USART1 serial port configuration and initialize its properties
// Each field represents specific peripheral setup for GPIO and USART1

SerialPort USART1_PORT = {
    USART1,                    // Pointer to USART1 peripheral
    GPIOC,                    // GPIO port C used for TX/RX
    RCC_APB2ENR_USART1EN,     // Clock enable bit for USART1 on APB2
    0x00,                     // No APB1 usage for USART1
    RCC_AHBENR_IOPAEN,       // Clock enable bit for GPIOC on AHB
    0xA00,                    // MODER config for PC10/PC11 (alternate function mode)
    0xF00,                    // OSPEEDR config for PC10/PC11 (high speed)
    0x770000,                 // AFR[0] setting for PC10/PC11 to alternate function 7 (USART)
    0x00,                     // No configuration for AFR[1]
    0x00,                     // TX buffer initialized to 0 (legacy/placeholder)
	.tx_buffer = {0},
	.rx_buffer1 = {0},
	.rx_buffer2 = {0},			//rx_buffer2 0
	0x00,						//rx_index 0
	0x00,						//string_recived flag 0
	USART1_IRQn
};


// Define the USART2 serial port configuration
SerialPort USART2_PORT = {
    USART2,                    // Pointer to USART2 peripheral
    GPIOA,                     // GPIO port A used for TX/RX
    RCC_APB1ENR_USART2EN,      // Clock enable bit for USART2 on APB1
    0x00,                      // No APB2 usage for USART2
    RCC_AHBENR_GPIOAEN,        // Clock enable bit for GPIOA on AHB
    0xA0,                      // MODER config for PA2/PA3 (alternate function mode)
    0xF0,                      // OSPEEDR config for PA2/PA3 (high speed)
    0x7700,                    // AFR[0] setting for PA2/PA3 to alternate function 7 (USART)
    0x00,                      // No configuration for AFR[1]
    0x00,                      // TX buffer initialized to 0
	.tx_buffer = {0},
	.rx_buffer1 = {0},
	.rx_buffer2 = {0},
	0x00,						//rx_index 0
	0x00,						//string_recived flag 0
	USART2_IRQn
};



// Initialize the serial port with specified baud rate and a receive completion callback
void SerialInitialise(BaudRate baudRate, SerialPort *serial_port, void (*callback)(uint8_t *data, uint8_t length)) {

    // Set the function to call when a full string is received
    serial_port->completion_function = callback;

    // Enable peripheral clocks required by the system and GPIO
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;      // Power interface clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   // System config clock
    RCC->AHBENR |= serial_port->MaskAHBENR; // Enable GPIO port clock

    // Set the GPIO pins to alternate function mode (for USART)
    serial_port->GPIO->MODER = serial_port->SerialPinModeValue;

    // Set GPIO pins to high speed mode
    serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;

    // Select alternate function for each pin
    serial_port->GPIO->AFR[0] &= ~serial_port->SerialPinAlternatePinValueLow;
    serial_port->GPIO->AFR[0] |=  serial_port->SerialPinAlternatePinValueLow;

    serial_port->GPIO->AFR[1] &= ~serial_port->SerialPinAlternatePinValueHigh;
    serial_port->GPIO->AFR[1] |=  serial_port->SerialPinAlternatePinValueHigh;

    // Enable USART peripheral clock
    RCC->APB1ENR |= serial_port->MaskAPB1ENR;
    RCC->APB2ENR |= serial_port->MaskAPB2ENR;

    // Set baud rate register (BRR) based on selected baud rate
    uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR;

    switch(baudRate) {
        case BAUD_9600:
            *baud_rate_config = 0x341;  // BRR value for 9600 baud at 8 MHz
            break;
        case BAUD_19200:
            *baud_rate_config = 0x1A0;
            break;
        case BAUD_38400:
            *baud_rate_config = 0xD0;
            break;
        case BAUD_57600:
            *baud_rate_config = 0x8B;
            break;
        case BAUD_115200:
            *baud_rate_config = 0x46;
            break;
    }

    // Enable USART transmitter and receiver, and the USART itself
    serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

// Transmit a single character via USART using non-blocking (interrupt-based) method
void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
    // Wait (stall) if buffer is full (circular buffer wraparound)
    while (((serial_port->tx_head + 1) % TX_BUFFER_SIZE) == serial_port->tx_tail);

    // Store character in TX buffer
    serial_port->tx_buffer[serial_port->tx_head] = data;

    // Advance head index, wrapping around if needed
    serial_port->tx_head = (serial_port->tx_head + 1) % TX_BUFFER_SIZE;

    // Mark transmitter as active and enable TXE interrupt to begin sending
    serial_port->tx_busy = 1;
    serial_port->UART->CR1 |= USART_CR1_TXEIE;
}

// Transmit a full null-terminated string via USART using interrupt-based method
void SerialSendString(uint8_t *pt, SerialPort *serial_port) {
    uint32_t counter = 0;

    // Send each character until null-terminator is reached
    while (*pt) {
        SerialOutputChar(*pt, serial_port);
        counter++;
        pt++;
    }
}
