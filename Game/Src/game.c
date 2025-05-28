#include "serialise.h"
#include "serial.h"
#include "timer.h"
#include "stm32f303xc.h"
#include "ADC.h"

static int16_t countdown_time = 0;

int8_t current_level = 1;
extern int8_t presses;
 extern int8_t button_IR1;
 extern int8_t button_IR2;
 extern int8_t button_IR3;

void System_Init(void) {
    // Enable all required clocks once
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_ADC12EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure GPIO modes once
    // PE8-15 as outputs for LEDs
    uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
    *led_output_registers = 0x5555;

    // PA1, PA2, PA3 as analog for ADC
    GPIOA->MODER |= (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2));

    // PA0 as input (default, but explicit)
    GPIOA->MODER &= ~(3 << (0 * 2));
}

// Countdown callback function that decrements and sends time via serial
void countdownCallback(void) {
    if (countdown_time >= 0) {

    	// Send current countdown time via serial
        send_time_data_packet(countdown_time);

        // Check if countdown has reached zero
        if (countdown_time == 0) {
        	stop_countdown();
        } else {
        	countdown_time--;
        }
    }
}

// Function to start countdown from specified time (in seconds)
void start_countdown(int16_t start_time) {
    if (start_time <= 0) {
        return; // Invalid countdown time
    }

    countdown_time = start_time;
    send_time_data_packet(countdown_time);

    presses = 0; //variable for button test
    button_IR1= 0;
    button_IR2=0;
    button_IR3=0;

    // Initialize timer for 1 second intervals (1000ms)
    Timer_Init(1000, countdownCallback);
    Timer_Start();
}

// Function to stop countdown
void stop_countdown(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN; // Stop timer

}


void check_IR_win_condition(int8_t IR1, int8_t IR2, int8_t IR3) {
    // Check if all IR sensors are covered
    if (IR1 == 1 && IR2 == 1 && IR3 == 1) {
        // Stop countdown
        stop_countdown();
    }
}

void IR_monitoring(int8_t IR1, int8_t IR2, int8_t IR3) {

	// Send IR data packet (called from main loop, so safe)
    send_IR_data_packet(IR1, IR2, IR3);

    // Check for win condition
    check_IR_win_condition(IR1, IR2, IR3);
}
