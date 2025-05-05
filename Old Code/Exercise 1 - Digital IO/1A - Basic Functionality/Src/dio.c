/**
******************************************************************************
* @file    : dio.c
* @brief   : Basic Digital I/O module implementation - Stage A
******************************************************************************
*/

#include "dio.h"

/**
 * @brief EXTI0 IRQ Handler for button press
 */
void EXTI0_IRQHandler(void)
{
    // Toggle LED halves
    dio_toggle_led_halves();

    // Reset the interrupt
    EXTI->PR |= EXTI_PR_PR0;
}

/**
 * @brief Enable clocks for needed peripherals
 */
static void enable_clocks(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}

/**
 * @brief Initialize board I/O
 */
static void initialise_board(void)
{
    // Configure LED pins (PE8-PE15) as outputs
    uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
    *led_output_registers = 0x5555;

    // Initialize LEDs - turn on first half
    uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
    *led_register = 0x0F;
}

/**
 * @brief Configure and enable button interrupt
 */
static void enable_interrupt(void)
{
    // Disable interrupts during configuration
    __disable_irq();

    // Enable system configuration controller
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PA0 as EXTI0 source
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;

    // Configure EXTI0 to trigger on rising edge
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // Enable EXTI0 interrupt
    EXTI->IMR |= EXTI_IMR_MR0;

    // Configure NVIC for EXTI0 interrupt
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);

    // Re-enable interrupts
    __enable_irq();
}

/**
 * @brief Initialize the Digital I/O module
 */
void dio_init(void)
{
    // Initialize hardware
    enable_clocks();
    initialise_board();
    enable_interrupt();
}

/**
 * @brief Toggle between first half and second half LEDs
 */
void dio_toggle_led_halves(void)
{
    uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
    uint8_t current_state = *led_register;

    // Check which half is currently on and toggle
    if ((current_state & 0xF0) == 0xF0) {
        // Upper half is on, switch to lower half
        *led_register = 0x0F;
    } else {
        // Lower half is on or other state, switch to upper half
        *led_register = 0xF0;
    }
}
