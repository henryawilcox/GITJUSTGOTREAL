#ifndef SERIALISE_HEADER
#define SERIALISE_HEADER

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// Constants
#define SENTINEL_1 0xAA
#define SENTINEL_2 0x55

// Enum for message types
typedef enum {
    SENSOR_DATA = 0,
    SERVO_STATE = 1,
} MessageType;

// Sensor data struct
typedef struct {
	int8_t vx;
	int8_t vy;
	int8_t speed;
	uint16_t z_target;
} SensorData;

// servo state
typedef struct {
    int8_t servo_state : 1;
} ServoState;


// Union of data types
typedef union {
    SensorData sensor_data;
    ServoState servo_state;
} Data;

// Header structure
typedef struct {
    uint8_t sentinel1;
    uint8_t sentinel2;
    uint16_t message_type;
    uint16_t data_length;
} Header;

// Sensor data struct
typedef struct {
	int8_t vx;
	int8_t vy;
	int8_t speed;
	uint16_t z_target;
	int8_t servo_state;
} ReceivedSensorData;

//received data, global
extern ReceivedSensorData received_sensor_data;

// Function to pack data into a buffer for transmission
uint16_t pack_buffer(uint8_t *buffer, MessageType message_type, Data *data);
// Function to unpack the buffer and check for sentinel bytes
bool unpack_buffer(const uint8_t *buffer, Data *output_data, MessageType *output_message_type, uint16_t *output_data_length);

void process_received_packet(uint8_t *buffer, uint16_t length);

#endif
