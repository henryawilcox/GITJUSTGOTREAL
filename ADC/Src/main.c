#include <stdint.h>
#include "stm32f303xc.h"
#include <stdio.h>
#include "serial.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// Simple delay
void delay(volatile uint32_t t) {
    while (t--) __asm("nop");
}

void init_adc1_pa1(void) {
    // 1. Enable GPIOA and ADC12 clocks
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR  |= RCC_AHBENR_ADC12EN;

    // 2. Set PA1 to analog mode
    GPIOA->MODER |= (3U << (1 * 2));  // Analog mode for PA1

    // 3. Ensure ADC is disabled
    if (ADC1->CR & ADC_CR_ADEN) {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN);
    }

    // 4. Enable ADC internal voltage regulator
    ADC1->CR &= ~ADC_CR_ADVREGEN;           // Clear both bits first
    ADC1->CR |= ADC_CR_ADVREGEN_0;          // Enable regulator in "intermediate mode"

    // 5. Wait at least 10 us for regulator startup (exact timing may vary)
    for (volatile int i = 0; i < 1000; i++) __asm("nop");

    // 6. Start ADC calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);  // Wait for calibration to finish

    // 7. Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));  // Wait for ADC to be ready LET KNOW IF STUCK ON THIS

    // 8. Configure ADC sequence: Channel 2 (PA1)
    ADC1->SQR1 = (2 << 6);             // 1st conversion: channel 2
    ADC1->SMPR1 |= (4 << (3 * 2));     // Sample time for channel 2
}

uint16_t read_adc(void) {
    ADC1->CR |= ADC_CR_ADSTART;            // Start the conversion
    while (!(ADC1->ISR & ADC_ISR_EOC));   // Wait for end of conversion
    return ADC1->DR;                       // Read the converted data from the data register
}

int main(void) {
	 uint8_t string_to_send[64] = "This is a string !\r\n";

    SerialInitialise(BAUD_115200, &USART1_PORT, &finished_transmission);
    init_adc1_pa1();

    while (1) {
        uint16_t raw = read_adc();
        float voltage = (raw * 3.3f) / 4095.0f;  // Convert ADC value to voltage

        Format the output string
        sprintf(string_to_send,"Voltage: %0.3f\r\n",voltage);

        SerialOutputString(string_to_send, &USART1_PORT);

        //delay(800000);  // Delay (adjust as needed)
    }
}
