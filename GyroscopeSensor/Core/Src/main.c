/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP/stm32f3_discovery.h"
#include "BSP/stm32f3_discovery_gyroscope.h"
#include "BSP/stm32f3_discovery_accelerometer.h"

#include "serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TIM6_Init(void) {
    // Enable TIM6 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Set TIM6 prescaler and ARR for 1ms time base
    // 72 MHz / 7200 = 10 kHz -> 0.1ms tick
    TIM6->PSC = 7200 - 1;   // Prescaler
    TIM6->ARR = 10 - 1;     // Auto-reload for 1ms period

    TIM6->EGR |= TIM_EGR_UG;      // Force update event to load prescaler
    TIM6->SR &= ~TIM_SR_UIF;      // Clear update interrupt flag
    TIM6->CR1 |= TIM_CR1_CEN;     // Start the timer
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        TIM6->CNT = 0;
        TIM6->SR &= ~TIM_SR_UIF;           // Clear update interrupt flag
        while (!(TIM6->SR & TIM_SR_UIF));  // Wait for update event
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  TIM6_Init();


  SerialInitialise(BAUD_115200, &USART1_PORT, 0x00);
  uint8_t string_to_send[64] = "This is a string !\r\n";

  int16_t acc_values[3];
  float float_acc_values[3];
  float gyro_values[3];

  float sum_x = 0x0;
  float sum_y = 0x0;
  float sum_z = 0x0;
  float i = 0;
  float accel_x_bias = 0;
  float accel_y_bias = 0;
  float accel_z_bias = 0;
  float pitch;
  float roll;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyroAngleX = 0;
  float gyroAngleY = 0;
  float angleX = 0;
  float angleY = 0;
  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float alpha = 0.1;
  float dt = 0.01;
  while (1)
  {
	  BSP_ACCELERO_GetXYZ(&acc_values[0]);
	  BSP_GYRO_GetXYZ(&gyro_values[0]);

	  accel_x = (float)acc_values[0] / 1500.;
	  accel_y = (float)acc_values[1] / 1500.;
	  accel_z = (float)acc_values[2] / 1500.;



	  //sprintf(string_to_send, "%0.3f,%0.3f,%0.3f\r\n",accel_x,accel_y,accel_z);
	  //SerialOutputString(string_to_send, &USART1_PORT);

	  if (i<101){
		  sum_x += accel_x;
		  sum_y += accel_y;
		  sum_z += accel_z - 10.0;
		  i++;

	  }

	  	  if (i == 101){
		  accel_x_bias = 0;
		  accel_y_bias = 0;
		  accel_z_bias = 0;
		  //sprintf(string_to_send, "Bias Values: %0.3f,%0.3f,%0.3f", accel_x_bias,accel_y_bias,accel_z_bias);
		  //SerialOutputString(string_to_send, &USART1_PORT);
		  pitch = atan2(accel_x - accel_x_bias, sqrt((accel_y - accel_y_bias)*(accel_y - accel_y_bias) + (accel_z - accel_z_bias)*(accel_z - accel_z_bias))) * 180.0 / 3.14;
		  roll = atan2(accel_y-accel_y_bias,accel_z - accel_z_bias) * 180.0 / 3.14;
		  float gyroRateX = gyro_values[0]/ 131.0;
		  float gyroRateY = gyro_values[1]/ 131.0;
		  gyroAngleX += gyroRateX * dt;
		  gyroAngleY += gyroRateY * dt;

		  angleX = alpha*gyroAngleX + (1-alpha)* pitch;
		  angleY = alpha*gyroAngleY + (1-alpha)* roll;
		  sprintf(string_to_send,"%0.3f,%0.3f\r\n",angleX,angleY);
		  SerialOutputString(string_to_send, &USART1_PORT);

	  }




	  delay_ms(75);






    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    // 1. Enable HSE with BYPASS
    RCC->CR |= RCC_CR_HSEBYP;      // External clock instead of crystal
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait for HSE ready

    // 2. Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // 3. Configure PLL: source = HSE, multiplier = x6
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;       // HSE selected as PLL source
    RCC->CFGR |= RCC_CFGR_PLLMUL6;                 // PLL multiplier = x6

    // 4. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL ready

    // 5. Set FLASH latency = 1 wait state (for >24 MHz)
    FLASH->ACR |= FLASH_ACR_LATENCY_1;

    // 6. Select PLL as SYSCLK
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 7. Set AHB, APB1, APB2 dividers
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 = HCLK / 2
    // AHB = /1 and APB2 = /1 are already default (0b000)


    // 8. Configure I2C1 clock source: HSI
    RCC->CFGR3 &= ~RCC_CFGR3_I2C1SW;
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW_HSI;

    // 9. Enable peripheral clocks if not already done (example)
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	// Enable GPIOE and GPIOA clocks
	    RCC->AHBENR |= RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOAEN;

	    // Define GPIOE output pins
	    uint32_t gpioe_output_pins = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin |
	                                  LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;

	    // Reset GPIOE output pins (set to 0)
	    GPIOE->BRR = gpioe_output_pins;

	    // Configure output pins (push-pull, no pull-up/down, low speed)
	    for (int pin = 0; pin < 16; pin++) {
	        if (gpioe_output_pins & (1 << pin)) {
	            // Set mode to output (01)
	            GPIOE->MODER &= ~(3U << (pin * 2));
	            GPIOE->MODER |=  (1U << (pin * 2));

	            // Output type = push-pull (0)
	            GPIOE->OTYPER &= ~(1U << pin);

	            // Speed = low (00)
	            GPIOE->OSPEEDR &= ~(3U << (pin * 2));

	            // No pull-up/down (00)
	            GPIOE->PUPDR &= ~(3U << (pin * 2));
	        }
	    }

	    // Configure GPIOE input event pins (rising edge trigger, no pull-up/down)
	    uint32_t gpioe_input_pins = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin |
	                                 MEMS_INT1_Pin | MEMS_INT2_Pin;

	    for (int pin = 0; pin < 16; pin++) {
	        if (gpioe_input_pins & (1 << pin)) {
	            // Set mode to input (00) — default
	            GPIOE->MODER &= ~(3U << (pin * 2));

	            // No pull-up/down
	            GPIOE->PUPDR &= ~(3U << (pin * 2));
	        }
	    }

	    // NOTE: Enabling EXTI (event interrupt trigger) is separate and optional.
	    // You would use SYSCFG and EXTI if needed.

	    // Configure B1_Pin as input on GPIOA
	    for (int pin = 0; pin < 16; pin++) {
	        if (B1_Pin & (1 << pin)) {
	            GPIOA->MODER &= ~(3U << (pin * 2)); // Input mode
	            GPIOA->PUPDR &= ~(3U << (pin * 2)); // No pull
	        }
	    }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
