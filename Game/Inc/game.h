/*
 * game.h
 *
 *  Created on: May 25, 2025
 *      Author: henrywilcox
 */

#ifndef GAME_H_
#define GAME_H_

#include <stdint.h>
#include "serialise.h"



// === Countdown control ===
void countdownCallback(void);
void start_countdown(int16_t start_time);
void stop_countdown(void);
int16_t get_countdown_time(void);
uint8_t is_countdown_active(void);

// === IR monitoring ===
void check_IR_win_condition(int8_t IR1, int8_t IR2, int8_t IR3);
void IR_monitoring(int8_t IR1, int8_t IR2, int8_t IR3);

// === Simulated IR ===
void simulate_IR(int8_t *IR1, int8_t *IR2, int8_t *IR3);

// === Button input ===
void Button_Init(void);
uint8_t read_button(void);

extern int8_t presses;

void System_Init(void);


#endif /* GAME_H_ */
