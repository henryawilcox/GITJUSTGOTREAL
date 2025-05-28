/**
 * @file serial.h
 * @brief Header file for serial communication using STM32 USART peripherals.
 *
 * Supports UART RX/TX with configurable GPIO and baud rate settings.
 * Uses interrupt-driven transmission and packet-based reception with callbacks.
 * Designed for MTRX2700 mechatronics projects.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

#define BUFFER_SIZE 64  ///< Size of TX and RX buffers (shared across all ports)

/**
 * @brief SerialPort structure containing all configuration and buffer information
 *        for each USART peripheral.
 */
typedef struct _SerialPort {
    volatile uint32_t *BaudRate;                  ///< Pointer to BRR register
    volatile uint32_t *ControlRegister1;          ///< Pointer to CR1 register
    volatile uint32_t *FlagClearRegister;         ///< Pointer to ICR register
    volatile uint32_t *StatusRegister;            ///< Pointer to ISR register
    volatile uint16_t *DataOutputRegister;        ///< Pointer to TDR register
    volatile uint16_t *DataInputRegister;         ///< Pointer to RDR register
    volatile uint32_t *TimerEnableRegister;       ///< Pointer to RCC enable register
    volatile uint32_t TimerEnableMask;            ///< Bitmask to enable USART clock
    volatile uint32_t SerialPortGPIO;             ///< Enum tag for GPIO port

    // GPIO Mode
    volatile uint32_t *SerialPinModeRegister;     ///< MODER register
    volatile uint32_t SerialPinModeValue;         ///< Alternate function mode value
    volatile uint32_t SerialPinModeMask;          ///< Mask for clearing mode bits

    // GPIO Speed
    volatile uint32_t *SerialPinSpeedRegister;    ///< OSPEEDR register
    volatile uint32_t SerialPinSpeedValue;        ///< Speed setting (e.g., high)
    volatile uint32_t SerialPinSpeedMask;         ///< Mask for speed bits

    // GPIO Alternate Function
    volatile uint32_t *SerialPinAlternateRegister;///< AFR register
    volatile uint32_t SerialPinAlternateValue;    ///< Value to set AF7
    volatile uint32_t SerialPinAlternateMask;     ///< Mask for alternate function bits

    // Callback triggered when a full packet is received
    void (*completion_function)(uint8_t *packet, struct _SerialPort *port);
    uint8_t expected_packet_size;                 ///< Packet length in bytes

    // Buffers
    volatile uint8_t RXBuffer[BUFFER_SIZE];
    volatile uint8_t RXIndex;
    volatile uint8_t TXBuffer[BUFFER_SIZE];
    volatile uint8_t TXIndex;
    volatile uint8_t TXLength;
    volatile uint8_t TXbusy;
} SerialPort;

// External declarations for USART1 and USART2
extern SerialPort USART1_PORT;
extern SerialPort USART2_PORT;

// Supported baud rates
enum {
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200
};

/**
 * @brief Initialise the given USART port with required settings.
 * @param baudrate One of the enum values above
 * @param serial_port Pointer to the port struct (e.g., &USART1_PORT)
 * @param packet_size Number of bytes expected per RX packet
 * @param on_packet_received Callback function triggered when packet is received
 */
void SerialInitialise(uint32_t baudrate, SerialPort *serial_port, uint8_t packet_size,
                      void (*on_packet_received)(uint8_t *packet, struct _SerialPort *port));

/** @brief Enable interrupt handling for USART1 on PC11 (RX) */
void enable_interrupt_USART1_PC11(void);

/** @brief Enable interrupt handling for USART2 on PA3 (RX) */
void enable_interrupt_USART2_PA3(void);

/**
 * @brief Sends a null-terminated string using interrupt-driven TX
 * @param pt Pointer to the string
 * @param serial_port Port to send from
 */
void SerialOutputStringInterrupt(uint8_t *pt, SerialPort *serial_port);

#endif /* SERIAL_H_ */
