/*
 * serial_interrupt.c
 *
 *  Created on: Apr 13, 2025
 *      Author: henrywilcox
 */

#include "serial.h"
#include "serial_interrupt.h"
#include "stm32f303xc.h"


void EnableSerialInterrupts(SerialPort *serial_port) {
    // Disable global interrupts to prevent issues during setup
    __disable_irq();

    // Reset flags and index
    serial_port->rx_index = 0;
    serial_port->string_received = 0;

    // Clear both RX buffers
    for (int i = 0; i < BUFFER_SIZE; i++) {
        serial_port->rx_buffer1[i] = 0;
        serial_port->rx_buffer2[i] = 0;
    }

    // Enable Receive Not Empty interrupt (triggers when data arrives)
    serial_port->UART->CR1 |= USART_CR1_RXNEIE;

    // Enable UART interrupt in the NVIC
    NVIC_EnableIRQ(serial_port->IRQn);
    NVIC_SetPriority(serial_port->IRQn, 0);  // Highest priority

    // Re-enable global interrupts
    __enable_irq();
}

void HandleUSARTInterrupt(SerialPort *serial_port) {
	 // ---------------------------
	    // RX SECTION
	    // ---------------------------
	    // Check if RXNE (Receive Data Register Not Empty) is set
	    if (serial_port->UART->ISR & USART_ISR_RXNE) {
	        // Read the incoming character
	        uint8_t received_char = serial_port->UART->RDR;

	        // Echo the character back for user feedback
	        //SerialOutputChar(received_char, &USART1_PORT);

	        // Only process if there's space in the buffer
	        if (serial_port->rx_index < BUFFER_SIZE - 1) {
	            // Store received character in live RX buffer
	            serial_port->rx_buffer1[serial_port->rx_index++] = received_char;

	            // Check for end-of-input character (ENTER/RETURN key)
	            if (received_char == '\r' || received_char == '\n') {
	                // Null-terminate the string
	                serial_port->rx_buffer1[serial_port->rx_index] = '\0';
	                serial_port->string_received = 1;

	                // Copy buffer1 to buffer2 (safe for processing outside ISR)
	                for (int i = 0; i < BUFFER_SIZE; i++) {
	                	serial_port->rx_buffer2[i] = serial_port->rx_buffer1[i];
	                	serial_port->rx_buffer1[i] = 0; // Clear buffer1 for next message
	                }

	                // Trigger the completion callback with the full message
	                serial_port->completion_function(serial_port->rx_buffer2, serial_port->rx_index);

	                // Reset index for next message
	                serial_port->rx_index = 0;
	            }
	        } else {
	            // If buffer overflows, clear and notify
	        	serial_port->rx_index = 0;
	            uint8_t BUFFEROVERFLOW[BUFFER_SIZE] = "ERROR: BUFFER OVERFLOW";
	            serial_port->completion_function(BUFFEROVERFLOW, serial_port->rx_index);
	        }
	    }

	    // ---------------------------
	    // TX SECTION
	    // ---------------------------
	    // Check if TXE (Transmit Data Register Empty) interrupt is enabled and pending
	    if ((serial_port->UART->CR1 & USART_CR1_TXEIE) && (serial_port->UART->ISR & USART_ISR_TXE)) {
	        // Check if there is data left to transmit in the circular buffer
	        if (serial_port->tx_tail != serial_port->tx_head) {
	            // Send next character from the TX buffer
	        	serial_port->UART->TDR = serial_port->tx_buffer[serial_port->tx_tail];
	        	serial_port->tx_tail = (serial_port->tx_tail + 1) % TX_BUFFER_SIZE;
	        } else {
	            // Transmission is complete — disable TXE interrupt
	        	serial_port->UART->CR1 &= ~USART_CR1_TXEIE;
	        	serial_port->tx_busy = 0;
	        }
	    }
}

// Interrupt Service Routine for USART1
void USART1_EXTI25_IRQHandler(void) {
	HandleUSARTInterrupt(&USART1_PORT);
}

// Interrupt Service Routine for USART2
void USART2_EXTI26_IRQHandler(void) {
	HandleUSARTInterrupt(&USART2_PORT);
}

/*
// Interrupt Service Routine for UART4
void UART4_EXTI34_IRQHandler(void) {
	HandleUSARTInterrupt(&USART4_PORT);
}*/
