/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "command_parser.h"
#include "config.h"
#include <stdlib.h>
#include <stepper.h>
#include "servo.h"




#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif



int main(void)
{

	SerialInitialise(BAUD_115200, &USART1_PORT, 8, command_parser_execute);
	SerialInitialise(BAUD_115200, &USART2_PORT, 8, command_parser_execute);

	enable_interrupt_USART1_PC11();
	enable_interrupt_USART2_PA3();

	Servo_Init();
	stepper_control_init();

	SerialOutputStringInterrupt((uint8_t *)"Steppers Initialised\r\n", &USART1_PORT);


//    /* Loop forever */
	for(;;){
	};
}
