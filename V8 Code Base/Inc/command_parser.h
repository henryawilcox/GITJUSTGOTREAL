#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>
#include "serial.h"

void parse_command(uint8_t *input, SerialPort *port);

#endif
