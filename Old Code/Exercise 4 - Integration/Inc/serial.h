#ifndef SERIAL_PORT_HEADER
#define SERIAL_PORT_HEADER


#include <stdint.h>

// Defining the serial port struct, the definition is hidden in the
// c file as no one really needs to know this.
struct _SerialPort;
typedef struct _SerialPort SerialPort;


// make any number of instances of the serial port (they are extern because
//   they are fixed, unique values)
extern SerialPort USART1_PORT;

//Ability of user to change buffer size
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

//Ability of user to modify buffers in main
extern volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
extern volatile uint8_t buffer1[RX_BUFFER_SIZE];
extern volatile uint8_t buffer2[RX_BUFFER_SIZE];




// The user might want to select the baud rate
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


//USART1_EXTI25_IRQHandler - function called in receiving interrupt
//Stores received byte into buffer
void USART1_EXTI25_IRQHandler(void);

// Enable_USART_interrupt - configures interrupt over USART1
void enable_USART_interrupt();

//start_interrupt_transmission - sends first byte over to load data into TDR
//Triggers transmission interrupt by raising TDR flag
//Takes in string to transmit and its size as well as UART channel
void start_interrupt_tranmission(SerialPort *serial_port, uint8_t *data, uint8_t size);

// finished_transmission - callback function to access length of string and buffer of stored string
void finished_transmission(uint32_t bytes_sent, char *sent_string);

// finished_receiving - callback function to access stored buffer and its length in memory
void finished_receiving(uint8_t num_characters, char *received_string);

// SerialOutputChar - output a char to the serial port
//  note: this version waits until the port is ready (not using interrupts)
// Input: char to be transferred
void SerialOutputChar(uint8_t, SerialPort *serial_port);
 

// SerialOutputString - output a NULL TERMINATED string to the serial port
// Input: pointer to a NULL-TERMINATED string (if not null terminated, there will be problems)
void SerialOutputString(uint8_t *pt, SerialPort *serial_port);

// SerialInputChar - input a character from serial port RDR
void SerialInputChar(uint8_t *data, SerialPort *serial_port);

// SerialInputString - inputs a string until '!' character is sent from serial port
void SerialInputString(uint8_t buffer_size, SerialPort *serial_port, char *buffer);

extern uint8_t current_led_mask;
 
 
#endif
