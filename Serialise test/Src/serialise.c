#include "serialise.h"
#include "serial.h"

// Function to pack data into a buffer for transmission
uint16_t pack_buffer(uint8_t *buffer, MessageType message_type, Data *data) {
    Header header = {SENTINEL_1, SENTINEL_2, message_type, 0};
    uint16_t buffer_idx = 0;
    uint16_t data_length = 0;

    switch (message_type) {
        case SENSOR_DATA:
            data_length = sizeof(SensorData);
            break;
        case SERVO_STATE:
            data_length = sizeof(ServoState);
            break;
    }

    header.data_length = data_length;

    // Copy header to buffer
    memcpy(buffer, &header, sizeof(Header));
    buffer_idx += sizeof(Header);

    // Copy data to buffer
    memcpy(buffer + buffer_idx, data, data_length);
    buffer_idx += data_length;

    return buffer_idx;
}


// Function to unpack the buffer and check for sentinel bytes
bool unpack_buffer(const uint8_t *buffer, Data *output_data, MessageType *output_message_type, uint16_t *output_data_length) {
    Header header;
    uint16_t buffer_idx = sizeof(Header);

    // Copy header from buffer
    memcpy(&header, buffer, sizeof(Header));

    // Check sentinel bytes
    if (header.sentinel1 != SENTINEL_1 || header.sentinel2 != SENTINEL_2) {
        return false;
    }

    *output_message_type = header.message_type;
    *output_data_length = header.data_length;

    // Copy data from buffer
    switch (*output_message_type) {
        case SENSOR_DATA:
            memcpy(&output_data->sensor_data, buffer + buffer_idx, sizeof(SensorData));
            break;
        case SERVO_STATE:
            memcpy(&output_data->servo_state, buffer + buffer_idx, sizeof(ServoState));
            break;
    }

    return true;

}

// Function to send sensor data packet over serial
void send_sensor_data_packet(int8_t vx, int8_t vy, int8_t speed, uint16_t z_target, int8_t servo) {
    Data sensor_data;
    uint8_t sensor_data_packet_buffer[sizeof(Header) + sizeof(SensorData)] = {0};

    // Fill sensor_data with provided values
    sensor_data.sensor_data.vx = vx;
    sensor_data.sensor_data.vy = vy;
    sensor_data.sensor_data.speed = speed;
    sensor_data.sensor_data.z_target = z_target;

    uint16_t sensor_data_buffer_length = pack_buffer(sensor_data_packet_buffer, SENSOR_DATA, &sensor_data);

    SerialOutputBufferInterrupt(sensor_data_packet_buffer, sensor_data_buffer_length, &USART1_PORT);
}

void send_servo_state_packet(int8_t servo_state) {
    Data servo_data;
    uint8_t servo_data_packet_buffer[sizeof(Header) + sizeof(ServoState)] = {0};

    servo_data.servo_state.servo_state = servo_state;

    uint16_t servo_data_buffer_length = pack_buffer(servo_data_packet_buffer, SERVO_STATE, &servo_data);

    SerialOutputBufferInterrupt(servo_data_packet_buffer, servo_data_buffer_length, &USART1_PORT);
}



// Simple packet processing function to call from interrupt handler
void process_received_packet( uint8_t *buffer, uint16_t length) {
    Data received_data;
    MessageType msg_type;
    uint16_t data_length;

    // Try to unpack the buffer
    if (unpack_buffer(buffer, &received_data, &msg_type, &data_length)) {
        if (msg_type == SENSOR_DATA) {
            // Update only sensor data fields
            received_sensor_data.vx = received_data.sensor_data.vx;
            received_sensor_data.vy = received_data.sensor_data.vy;
            received_sensor_data.speed = received_data.sensor_data.speed;
            received_sensor_data.z_target = received_data.sensor_data.z_target;
        }
        else if (msg_type == SERVO_STATE) {
            // Update only servo field
            received_sensor_data.servo_state = received_data.servo_state.servo_state;
        }
    }
}
