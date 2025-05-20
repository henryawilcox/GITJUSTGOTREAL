#include "lidar.h"
#include "ptu_definitions.h"
#include "stm32f3xx_hal_gpio.h"

uint16_t last_period = 0;
uint16_t rise_time = 0;
uint16_t last_capture = 0;
uint16_t diff = 0;

// enable the clocks for desired peripherals (GPIOA, C and E)
void enable_clocks() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}


// initialise the discovery board I/O (just outputs: inputs are selected by default)
void initialise_board() {
	// get a pointer to the second half word of the MODER register (for outputs pe8-15)
	uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	*led_output_registers = 0x5555;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		uint16_t IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1)
			rise_time = IC_Val1;
		else
			last_period = IC_Val1 - rise_time;

		diff = IC_Val1 - last_capture;
		last_capture = IC_Val1;
	}
}

void Lidar_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t reset_value = 0x00;
	HAL_I2C_Mem_Write(hi2c, LIDAR_WR, 0x00, 1, &reset_value, 1, 10);
	HAL_Delay(100);
}

uint8_t Lidar_GetHeightPercent(uint16_t period)
{
	const uint16_t max_height = 1000;
	const uint16_t min_height = 100;

	if (period <= min_height)
		return 0;
	else if (period >= max_height)
		return 100;
	else
		return ((period - min_height) * 100) / (max_height - min_height);
}

uint16_t Lidar_GetLastPeriod(void) {
    return last_period;
}



