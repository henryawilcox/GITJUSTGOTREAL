#include "serialise.h"
#include "serial.h"
#include "game.h"

ReceivedData received_data = {0};

// Function to pack data into a buffer for transmission
uint16_t pack_buffer(uint8_t *buffer, MessageType message_type, Data *data) {
    Header header = {SENTINEL_1, SENTINEL_2, message_type, 0};
    uint16_t buffer_idx = 0;
    uint16_t data_length = 0;

    switch (message_type) {
        case GAME_DATA:
            data_length = sizeof(GameData);
            break;
        case TIME_DATA:
            data_length = sizeof(TimeData);
            break;
        case IR_DATA:
        	data_length = sizeof(IRData);
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
        case GAME_DATA:
            memcpy(&output_data->game_data, buffer + buffer_idx, sizeof(GameData));
            break;
        case TIME_DATA:
            memcpy(&output_data->time_data, buffer + buffer_idx, sizeof(TimeData));
            break;
        case IR_DATA:
        	memcpy(&output_data->ir_data, buffer + buffer_idx, sizeof(IRData));
        	break;
    }

    return true;

}

void send_game_data_packet(int8_t START, int8_t ACTIVE, int8_t LEVEL) {
    Data game_data;
    uint8_t game_data_packet_buffer[sizeof(Header) + sizeof(GameData)] = {0};

    // Fill IR_data with provided values
    game_data.game_data.START = START;  // Fixed: use ir_data instead of IR_data
    game_data.game_data.ACTIVE = ACTIVE;  // Fixed: use ir_data instead of IR_data
    game_data.game_data.LEVEL = LEVEL;  // Fixed: use ir_data instead of IR_data

    uint16_t game_data_buffer_length = pack_buffer(game_data_packet_buffer, GAME_DATA, &game_data);

    SerialOutputBufferInterrupt(game_data_packet_buffer, game_data_buffer_length, &USART1_PORT);
}

//send time data
void send_time_data_packet(int16_t TIME) {
    Data time_data;
    uint8_t time_data_packet_buffer[sizeof(Header) + sizeof(TimeData)] = {0};

    // Fill time_data with provided values
    time_data.time_data.TIME = TIME;

    uint16_t time_data_buffer_length = pack_buffer(time_data_packet_buffer, TIME_DATA, &time_data);

    // Fixed: Use time_data_packet_buffer instead of TIME_data_packet_buffer
    SerialOutputBufferInterrupt(time_data_packet_buffer, time_data_buffer_length, &USART1_PORT);
}

// send_IR_data_packet function
void send_IR_data_packet(int8_t IR1, int8_t IR2, int8_t IR3) {
    Data IR_data;
    uint8_t IR_data_packet_buffer[sizeof(Header) + sizeof(IRData)] = {0};

    // Fill IR_data with provided values
    IR_data.ir_data.IR1 = IR1;  // Fixed: use ir_data instead of IR_data
    IR_data.ir_data.IR2 = IR2;  // Fixed: use ir_data instead of IR_data
    IR_data.ir_data.IR3 = IR3;  // Fixed: use ir_data instead of IR_data

    uint16_t IR_data_buffer_length = pack_buffer(IR_data_packet_buffer, IR_DATA, &IR_data);

    SerialOutputBufferInterrupt(IR_data_packet_buffer, IR_data_buffer_length, &USART1_PORT);
}

//process_received_packet function
void process_received_packet(uint8_t *buffer, uint16_t length) {
    Data received;
    MessageType msg_type;
    uint16_t data_length;

    // Try to unpack the buffer
    if (unpack_buffer(buffer, &received, &msg_type, &data_length)) {
    	if (msg_type == GAME_DATA) {
    	    received_data.START = received.game_data.START;
    	    received_data.ACTIVE = received.game_data.ACTIVE;
    	    received_data.LEVEL = received.game_data.LEVEL;

    	    if (received_data.START == 1) {
    	        current_level = received_data.LEVEL;

    	        switch (current_level) {
    	            case 1:
    	                start_countdown(60); // Level 1: 60
    	                break;
    	            case 2:
    	                start_countdown(40); // Level 2: 40s
    	                break;
    	            case 3:
    	                start_countdown(20); // Level 3: 20s
    	                break;
    	            default:
    	                break;
    	        }
    	    }
    	}
        else if (msg_type == TIME_DATA) {
            // Fixed: Update time field correctly
            received_data.TIME = received.time_data.TIME;
        }
        else if (msg_type == IR_DATA) {
            // Fixed: Access IR data correctly
            received_data.IR1 = received.ir_data.IR1;
            received_data.IR2 = received.ir_data.IR2;
            received_data.IR3 = received.ir_data.IR3;
        }
    }
}
