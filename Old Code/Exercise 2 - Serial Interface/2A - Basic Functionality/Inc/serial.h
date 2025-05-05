#ifndef SERIAL_PORT_HEADER
#define SERIAL_PORT_HEADER


#include <stdint.h>

// Defining the serial port struct
struct _SerialPort;
typedef struct _SerialPort SerialPort;


// USART1 serial port instance
// Must be external for main to access it
extern SerialPort USART1_PORT;

// Baud Rate Selection
enum {
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200
};

// SerialInitialise - initialise the serial port
// Input: baud rate as defined in the enum
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t, char *) );

// finished_transmission - callback function to access length of string and buffer of stored string
void finished_transmission(uint32_t bytes_sent, char *sent_string);

// finished_receiving - callback function to access stored buffer and its length in memory
void finished_receiving(uint8_t num_characters, char *received_string);

// SerialOutputChar - output a char to the serial port
//  note: this version polls (not using interrupts)
// Input: char to be transferred
void SerialOutputChar(uint8_t, SerialPort *serial_port);

// SerialOutputString - output a NULL TERMINATED string to the serial port
// Input: pointer to a NULL-TERMINATED string (if not null terminated, there will be problems)
void SerialOutputString(uint8_t *pt, SerialPort *serial_port);

// SerialInputChar - input a character from serial port RDR
void SerialInputChar(uint8_t *data, SerialPort *serial_port);

// SerialInputString - inputs a string until '/r' character is sent from serial port
// Polling function, not interrupt
void SerialInputString(uint8_t buffer_size, SerialPort *serial_port, char *buffer);
 
 
#endif

