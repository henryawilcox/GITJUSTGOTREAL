/**
 * @file serial.c
 * @brief Serial communication module for STM32F3 with packet-based RX support.
 *
 * Supports full-duplex UART communication on USART1 and USART2.
 * Automatically dispatches received packets of configurable size to a user-defined handler.
 * Designed for robotics and embedded systems applications in MTRX2700.
 */

#include "serial.h"
#include "stm32f303xc.h"
#include <stdio.h>


// Enum used to identify GPIO ports
enum {
    SERIAL_GPIO_A,
    SERIAL_GPIO_B,
    SERIAL_GPIO_C,
    SERIAL_GPIO_D,
    SERIAL_GPIO_E
};

// Instantiate serial port configuration for USART1 (PC4/PC5)
SerialPort USART1_PORT = {
    &(USART1->BRR), &(USART1->CR1), &(USART1->ICR), &(USART1->ISR),
    &(USART1->TDR), &(USART1->RDR),
    &(RCC->APB2ENR), RCC_APB2ENR_USART1EN, SERIAL_GPIO_C,
    &(GPIOC->MODER), (2 << (4 * 2)) | (2 << (5 * 2)), (3 << (4 * 2)) | (3 << (5 * 2)),
    &(GPIOC->OSPEEDR), (3 << (4 * 2)) | (3 << (5 * 2)), (3 << (4 * 2)) | (3 << (5 * 2)),
    &(GPIOC->AFR[0]), (7 << (4 * 4)) | (7 << (4 * 5)), (0xF << (4 * 4)) | (0xF << (4 * 5))
};

// Instantiate serial port configuration for USART2 (PA2/PA3)
SerialPort USART2_PORT = {
    &(USART2->BRR), &(USART2->CR1), &(USART2->ICR), &(USART2->ISR),
    &(USART2->TDR), &(USART2->RDR),
    &(RCC->APB1ENR), RCC_APB1ENR_USART2EN, SERIAL_GPIO_A,
    &(GPIOA->MODER), (2 << (2 * 2)) | (2 << (3 * 2)), (3 << (2 * 2)) | (3 << (3 * 2)),
    &(GPIOA->OSPEEDR), (3 << (2 * 2)) | (3 << (3 * 2)), (3 << (2 * 2)) | (3 << (3 * 2)),
    &(GPIOA->AFR[0]), (7 << (4 * 2)) | (7 << (4 * 3)), (0xF << (4 * 2)) | (0xF << (4 * 3))
};

void SerialInitialise(uint32_t baudrate, SerialPort *port, uint8_t packet_size,
                      void (*on_packet_received)(uint8_t *packet, SerialPort *port)) {
    port->completion_function = on_packet_received;
    port->expected_packet_size = packet_size;
    port->TXbusy = 0;

    // Enable required peripheral clocks
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    switch (port->SerialPortGPIO) {
        case SERIAL_GPIO_A: RCC->AHBENR |= RCC_AHBENR_GPIOAEN; break;
        case SERIAL_GPIO_C: RCC->AHBENR |= RCC_AHBENR_GPIOCEN; break;
    }

    // Configure GPIO for alternate function
    *(port->SerialPinModeRegister) = (*(port->SerialPinModeRegister) & ~port->SerialPinModeMask) | port->SerialPinModeValue;
    *(port->SerialPinSpeedRegister) = (*(port->SerialPinSpeedRegister) & ~port->SerialPinSpeedMask) | port->SerialPinSpeedValue;
    *(port->SerialPinAlternateRegister) = (*(port->SerialPinAlternateRegister) & ~port->SerialPinAlternateMask) | port->SerialPinAlternateValue;

    // Enable USART peripheral
    *(port->TimerEnableRegister) |= port->TimerEnableMask;

    // Set baud rate (assumes 8 MHz clock)
    uint16_t *baud = (uint16_t *)port->BaudRate;
    switch (baudrate) {
        case BAUD_9600: *baud = 0x341; break;
        case BAUD_19200: *baud = 0x1A1; break;
        case BAUD_38400: *baud = 0xD1; break;
        case BAUD_57600: *baud = 0x8B; break;
        case BAUD_115200: *baud = 0x46; break;
    }

    *(port->ControlRegister1) |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // Enable TX, RX, USART
}

void enable_interrupt_USART1_PC11() {
    __disable_irq();
    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 |= USART_CR1_RXNEIE;  // Enable RX interrupt
    __enable_irq();
}

void enable_interrupt_USART2_PA3() {
    __disable_irq();
    NVIC_SetPriority(USART2_IRQn, 4);
    NVIC_EnableIRQ(USART2_IRQn);
    USART2->CR1 |= USART_CR1_RXNEIE;
    __enable_irq();
}

// Interrupt handler for USART1
void USART1_EXTI25_IRQHandler(void) {
    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;
        if (USART1_PORT.RXIndex == 0 && c != 0xAA) return;  // Wait for start byte
        USART1_PORT.RXBuffer[USART1_PORT.RXIndex++] = c;
        if (USART1_PORT.RXIndex >= USART1_PORT.expected_packet_size) {
            USART1_PORT.RXIndex = 0;
            if (USART1_PORT.completion_function) {
                USART1_PORT.completion_function((uint8_t *)USART1_PORT.RXBuffer, &USART1_PORT);
            }
        }
    }

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

// Interrupt handler for USART2
void USART2_EXTI26_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        char c = USART2->RDR;
        if (USART2_PORT.RXIndex == 0 && c != 0xAA) return;
        USART2_PORT.RXBuffer[USART2_PORT.RXIndex++] = c;
        if (USART2_PORT.RXIndex >= USART2_PORT.expected_packet_size) {
            USART2_PORT.RXIndex = 0;
            if (USART2_PORT.completion_function) {
                USART2_PORT.completion_function((uint8_t *)USART2_PORT.RXBuffer, &USART2_PORT);
            }
        }
    }

    if (USART2->ISR & USART_ISR_TXE) {
        if (USART2_PORT.TXIndex < USART2_PORT.TXLength) {
            USART2->TDR = USART2_PORT.TXBuffer[USART2_PORT.TXIndex++];
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
            USART2_PORT.TXIndex = 0;
            USART2_PORT.TXLength = 0;
            USART2_PORT.TXbusy = 0;
        }
    }
}

// Sends a null-terminated string using interrupt-driven TX
void SerialOutputStringInterrupt(uint8_t *pt, SerialPort *port) {
    if (port->TXbusy) return;  // Drop if TX in progress

    uint8_t i = 0;
    while (pt[i] != '\0' && i < BUFFER_SIZE - 1) {
        port->TXBuffer[i] = pt[i];
        i++;
    }
    port->TXBuffer[i] = '\0';
    port->TXLength = i;
    port->TXIndex = 0;
    port->TXbusy = 1;

    *(port->ControlRegister1) |= USART_CR1_TXEIE;  // Enable TXE interrupt
}
