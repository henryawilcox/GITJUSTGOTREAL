/*
 * serial.h
 *
 *  Created on: May 21, 2025
 *      Author: Angus
 */

#ifndef SERIAL_H_
#define SERIAL_H_


#include <stdint.h>

// Defining the serial port struct, the definition is hidden in the
// c file as no one really needs to know this.
#define BUFFER_SIZE 64

typedef struct _SerialPort {
    volatile uint32_t *BaudRate;
    volatile uint32_t *ControlRegister1;
    volatile uint32_t *FlagClearRegister;
    volatile uint32_t *StatusRegister;
    volatile uint16_t *DataOutputRegister;
    volatile uint16_t *DataInputRegister;
    volatile uint32_t *TimerEnableRegister;
    volatile uint32_t TimerEnableMask;
    volatile uint32_t SerialPortGPIO;

    // GPIO MODER
    volatile uint32_t *SerialPinModeRegister;
    volatile uint32_t SerialPinModeValue;
    volatile uint32_t SerialPinModeMask;

    // GPIO OSPEEDR
    volatile uint32_t *SerialPinSpeedRegister;
    volatile uint32_t SerialPinSpeedValue;
    volatile uint32_t SerialPinSpeedMask;

    // GPIO Alternate Function
    volatile uint32_t *SerialPinAlternateRegister;
    volatile uint32_t SerialPinAlternateValue;
    volatile uint32_t SerialPinAlternateMask;

    void (*completion_function)(uint32_t);

    // RX/TX Buffers
    volatile uint8_t RXBuffer[BUFFER_SIZE];
    volatile uint8_t RXIndex;

    volatile uint8_t TXBuffer[BUFFER_SIZE];
    volatile uint8_t TXIndex;
    volatile uint8_t TXLength;
    volatile uint8_t TXbusy;

    // Binary packet reception fields
     uint8_t packet_buffer[256];     // Buffer for incoming packet
     uint16_t packet_index;          // Current position in packet buffer
     uint8_t packet_state;           // Reception state machine
     uint16_t expected_length;       // Expected packet length
     uint8_t sentinel_count;         // Count of sentinel bytes received
} SerialPort;


// make any number of instances of the serial port (they are extern because
//   they are fixed, unique values)
extern SerialPort USART1_PORT;
extern SerialPort USART2_PORT;

// The user might want to select the baud rate
enum {
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200
};


//// SerialInitialise - initialise the serial port
//// Input: baud rate as defined in the enum
//void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t) );
//
//
// SerialReceiveChar - read a char from the serial port
//  note: this version uses polling
// Input: char to be received
// Returns 1 for success, 0 for timeout/failure
//uint8_t SerialReceiveChar(SerialPort *serial_port, uint8_t *received_char);
//
//
//// SerialOutputString - output a NULL TERMINATED string to the serial port
//// Input: pointer to a NULL-TERMINATED string (if not null terminated, there will be problems)
//void SerialOutputString(uint8_t *pt, SerialPort *serial_port);
//
//
//// SerialOutputBuffer - output a buffer with defined length to the serial port
//// Input: pointer to a buffer, length of the buffer
//void SerialOutputBuffer(uint8_t *buffer, uint16_t buffer_length, SerialPort *serial_port);
//
//
//// Read a packet header - includes waiting until a valid sentinel is read
//// Input: char to be received
//// Returns 1 for success, 0 for timeout/failure
//uint16_t SerialInputPacketHeader(uint8_t *buffer, SerialPort *serial_port);
//
//// Read the buffer of the size indicated by length
//// Input: char to be received
//// Returns 1 for success, 0 for timeout/failure
//uint16_t SerialInputDataPacket(uint8_t *buffer, uint16_t length, SerialPort *serial_port);

void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t));
void SerialOutputChar(uint8_t, SerialPort *serial_port);
void SerialOutputString(uint8_t *pt, SerialPort *serial_port);
void enable_interrupt_USART1_PC11(void);
void enable_interrupt_USART2_PA3(void);

void SerialOutputStringInterrupt(uint8_t *pt, SerialPort *serial_port);
void SerialOutputBufferInterrupt(uint8_t *buffer, uint16_t buffer_length, SerialPort *serial_port);


#endif /* SERIAL_H_ */
