#include "stepper_timer.h"

#include "command_parser.h"

#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


void parse_command(uint8_t *input, SerialPort *port) {
	SerialOutputString((uint8_t *)"received command\r\n", &USART1_PORT);


	return;
//    int32_t x_val = 0, y_val = 0, z_val = 0;
//    int32_t vx = 0, vy = 0, vz = 0;
//    bool is_velocity_command = false;
//
////    if (strcmp((char*)input, "HOME") == 0) {
//////        HomeAllAxes();
////        return;
////    }
//
//    char *token = strtok((char*)input, " ");
//    while (token != NULL) {
//        if (strncmp(token, "VX", 2) == 0 || strncmp(token, "vx", 2) == 0) {
//            vx = strtol(token + 2, NULL, 10);
//            is_velocity_command = true;
//        } else if (strncmp(token, "VY", 2) == 0 || strncmp(token, "vy", 2) == 0) {
//            vy = strtol(token + 2, NULL, 10);
//            is_velocity_command = true;
//        } else if (strncmp(token, "VZ", 2) == 0 || strncmp(token, "vz", 2) == 0) {
//            vz = strtol(token + 2, NULL, 10);
//            is_velocity_command = true;
//        } else if (token[0] == 'X' || token[0] == 'x') {
//            x_val = strtol(token + 1, NULL, 10);
//        } else if (token[0] == 'Y' || token[0] == 'y') {
//            y_val = strtol(token + 1, NULL, 10);
//        } else if (token[0] == 'Z' || token[0] == 'z') {
//            z_val = strtol(token + 1, NULL, 10);
//        }
//        token = strtok(NULL, " ");
//    }
//
//    char msg[64];
//    if (is_velocity_command) {
////        snprintf(msg, sizeof(msg), "Velocity command VX:%ld VY:%ld VZ:%ld\r\n", vx, vy, vz);
////        SerialOutputString((uint8_t*)msg, port);
////        move_axes_with_velocity(vx, vy);
//        plan_velocity(vx, vy);
//    } else {
//        snprintf(msg, sizeof(msg), "Parsed X:%ld Y:%ld Z:%ld\r\n", x_val, y_val, z_val);
//        SerialOutputString((uint8_t*)msg, port);
//        if (x_val >= 0 || y_val >= 0 || z_val >= 0)
////            move_axes_to_position(x_val, y_val, z_val);
//        	SerialOutputString((uint8_t*)"No valid coordinates\r\n", port);
//        else
//            SerialOutputString((uint8_t*)"No valid coordinates\r\n", port);
//    }
}



