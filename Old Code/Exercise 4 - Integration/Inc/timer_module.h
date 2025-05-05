#ifndef TIMER_MODULE_H
#define TIMER_MODULE_H

#include <stdint.h>

void timer_init(uint32_t period_ms, void (*callback)(void));
void set_period(uint32_t period_ms);
uint32_t get_period(void);
void start_oneshot(uint32_t delay_ms, void (*callback)(void));
void enable_timer_interrupts(void);

#endif
