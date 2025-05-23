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

#include <serialise.h>
#include <stdint.h>
#include "serial.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void callback1(uint32_t length){
	process_received_packet((uint8_t *)USART1_PORT.RXBuffer, (uint16_t)length);


}

void uart1_test(void) {
    Data test;
    // fill in some arbitrary values
    test.sensor_data.vx       =  10;
    test.sensor_data.vy       = -20;
    test.sensor_data.speed    =   5;
    test.sensor_data.z_target = 1234;

    // build the packet
    uint8_t buf[ sizeof(Header) + sizeof(SensorData) ];
    uint16_t pkt_len = pack_buffer(buf, SENSOR_DATA, &test);

    // fire it off
    SerialOutputBufferInterrupt(buf, pkt_len, &USART1_PORT);
}

int main(void)
{
	for (volatile int i = 0; i < 800000; i++); // 100ms-ish delay


	SerialInitialise(BAUD_115200, &USART1_PORT, &callback1);
	SerialInitialise(BAUD_115200, &USART2_PORT, 0x00);

	enable_interrupt_USART1_PC11();
	enable_interrupt_USART2_PA3();

	while (1) {
		for (volatile int i = 0; i < 800000; i++); // 100ms-ish delay
		uart1_test();
	}
    /* Loop forever */
	for(;;);
}
