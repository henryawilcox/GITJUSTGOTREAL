#ifndef LIDAR_H
#define LIDAR_H

#include "stm32f3xx_hal.h"

void Lidar_Init(I2C_HandleTypeDef *hi2c);
uint8_t Lidar_GetHeightPercent(uint16_t period);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void enable_clocks();

void initialise_board();

uint16_t Lidar_GetLastPeriod(void);



#endif // LIDAR_H

