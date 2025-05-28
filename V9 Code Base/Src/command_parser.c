#include <stepper.h>
#include "command_parser.h"
#include "servo.h"
#include "serial.h"

/**
 * @brief Execute parsed packet
 * @param packet Pointer to received 8-byte buffer
 * @param port Pointer to the SerialPort that received it
 */
void command_parser_execute(uint8_t *packet, SerialPort *port) {
    uint8_t message_type = packet[1];

    switch (message_type) {
        case 0x01: {  // velocity + Z move
            int8_t vx = packet[2];
            int8_t vy = packet[3];
            uint8_t velocity_index = packet[4];
            uint16_t z_target = (packet[6] << 8) | packet[5];
            plan_velocity(vx, vy, velocity_index);
            set_z_target(z_target);
            break;
        }

        case 0x02:  // Close servo
            Servo_Close();
            break;

        case 0x03:  // Open servo
            Servo_Open();
            break;

        default:
//            SerialOutputString((uint8_t *)"Unknown command\r\n", port);
            break;
    }
}
