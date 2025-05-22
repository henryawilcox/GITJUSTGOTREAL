/*
 * servo.h
 *
 *  Created on: May 20, 2025
 *      Author: Angus
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

void Servo_Init(void);
void Servo_SetPulse(uint16_t pulse_us);
void Servo_Open(void);
void Servo_Close(void);

#endif /* SERVO_H_ */
