#ifndef STEPPER_TIMER_H
#define STEPPER_TIMER_H

#include "stm32f303xc.h"
#include <stdint.h>

typedef enum { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 } Axis;

void stepper_timer_init(void);
void stepper_timer_set_interval_us(Axis axis, uint32_t interval_us);  // 0 = disable
void stepper_timer_enable_axis(Axis axis);
void stepper_timer_disable_axis(Axis axis);

#endif
