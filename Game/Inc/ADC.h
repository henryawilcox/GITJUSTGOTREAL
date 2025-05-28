/**
 ******************************************************************************
 * @file           : adc_module.h
 * @author         : Auto-generated header for ADC module
 * @brief          : Header file for ADC functions and data types
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
#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdio.h>

// Public structure for IR sensor states
typedef struct {
    int8_t IR1;
    int8_t IR2;
    int8_t IR3;
} IR_values;

extern IR_values current_ir_values;
extern IR_values previous_ir_values;

void ADC_EnableSimulationMode(void);
void ADC_DisableSimulationMode(void);
uint8_t ADC_IsSimulationMode(void);

// Public function declarations
void delay(volatile uint32_t ms);
void ADC_Initialize(void);
IR_values ADC_ReadIRSensors(void);
int8_t ADC_CheckIRChanges(void);
void ADC_ResetIRSensors(void);
IR_values ADC_GetCurrentIRValues(void);

#endif // ADC_H
