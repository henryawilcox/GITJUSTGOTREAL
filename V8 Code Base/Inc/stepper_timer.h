#ifndef STEPPER_TIMER_H
#define STEPPER_TIMER_H

#include "stm32f303xc.h"
#include <stdint.h>
#include "serial.h"
#include <stdbool.h>


#include <stdbool.h>

void init_axis0_gpio(void);
void init_axis1_gpio(void);
void init_axis2_gpio(void);

void stepper_timer_init(void);

void stepper_control_init(void);

void plan_velocity(int8_t vx, int8_t vy, uint8_t index);

void set_z_target(int16_t z_target);

#endif
