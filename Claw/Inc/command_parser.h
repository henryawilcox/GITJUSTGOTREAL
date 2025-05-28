#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>
#include "serial.h"

void command_parser_execute(uint8_t *packet, SerialPort *port);

#endif
