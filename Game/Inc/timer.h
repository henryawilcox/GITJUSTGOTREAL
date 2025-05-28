#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

typedef void (*TimerCallback)(void);

void Timer_Init(uint32_t period_ms, TimerCallback cb);
void Timer_Start(void);

void Timer_StartOneShot(uint32_t delay_ms, TimerCallback cb);

void setTimerPeriod(uint32_t new_period_ms);
uint32_t getTimerPeriod(void);


void enable_clocks(void);
void initialise_board(void);


void oneshot_action(uint32_t operand);
void periodic_action(uint32_t operand);


#endif // TIMER_H
