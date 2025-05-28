#ifndef SERIALISE_HEADER
#define SERIALISE_HEADER

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "game.h"

// === Constants ===
#define SENTINEL_1 0xAA
#define SENTINEL_2 0x55

// === Enum for message types ===
typedef enum {
    GAME_DATA = 0,
    TIME_DATA = 1,
    IR_DATA   = 2,
} MessageType;

// === Data Structures ===

// Game data packet
typedef struct  {
    int8_t START;
    int8_t ACTIVE;
    int8_t LEVEL;
} GameData;

// Time data packet
typedef struct {
    int16_t TIME;
} TimeData;

// IR data packet
typedef struct  {
    int8_t IR1;
    int8_t IR2;
    int8_t IR3;
} IRData;

// Unified data structure
typedef union {
    GameData game_data;
    TimeData time_data;
    IRData   ir_data;
} Data;

// Packet header
typedef struct {
    uint8_t  sentinel1;
    uint8_t  sentinel2;
    uint16_t message_type;
    uint16_t data_length;
} Header;

// Parsed received data (global holder)
typedef struct {
    int8_t  START;
    int8_t	ACTIVE;
    int8_t  LEVEL;
    int16_t TIME;
    int8_t  IR1;
    int8_t  IR2;
    int8_t  IR3;
} ReceivedData;

// === Global Received Data ===
extern ReceivedData received_data;

// === Function Prototypes ===
uint16_t pack_buffer(uint8_t *buffer, MessageType message_type, Data *data);
bool unpack_buffer(const uint8_t *buffer, Data *output_data, MessageType *output_message_type, uint16_t *output_data_length);
void process_received_packet(uint8_t *buffer, uint16_t length);

void send_game_data_packet(int8_t START, int8_t ACTIVE, int8_t LEVEL);
void send_time_data_packet(int16_t TIME);
void send_IR_data_packet(int8_t IR1, int8_t IR2, int8_t IR3);

extern int8_t current_level;

#endif // SERIALISE_HEADER
