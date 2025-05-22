#include "stm32f303xc.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "serial.h"

#include "stepper_timer.h"

#define TIMER_FREQ 1000000  // 1 MHz tick = 1 us
#define FIXED_DELAY_US 1  // Placeholder delay for testing

const uint16_t delay_table[100] = {
    20,  // index 0
    20,  // index 1
    20,  // index 2
    20,  // index 3
    20,  // index 4
    19,  // index 5
    19,  // index 6
    19,  // index 7
    19,  // index 8
    19,  // index 9
    18,  // index 10
    18,  // index 11
    18,  // index 12
    18,  // index 13
    18,  // index 14
    17,  // index 15
    17,  // index 16
    17,  // index 17
    17,  // index 18
    17,  // index 19
    16,  // index 20
    16,  // index 21
    16,  // index 22
    16,  // index 23
    16,  // index 24
    15,  // index 25
    15,  // index 26
    15,  // index 27
    15,  // index 28
    15,  // index 29
    14,  // index 30
    14,  // index 31
    14,  // index 32
    14,  // index 33
    14,  // index 34
    13,  // index 35
    13,  // index 36
    13,  // index 37
    13,  // index 38
    13,  // index 39
    12,  // index 40
    12,  // index 41
    12,  // index 42
    12,  // index 43
    12,  // index 44
    11,  // index 45
    11,  // index 46
    11,  // index 47
    11,  // index 48
    11,  // index 49
    10,  // index 50
    10,  // index 51
    10,  // index 52
    10,  // index 53
    10,  // index 54
     9,  // index 55
     9,  // index 56
     9,  // index 57
     9,  // index 58
     9,  // index 59
     8,  // index 60
     8,  // index 61
     8,  // index 62
     8,  // index 63
     8,  // index 64
     7,  // index 65
     7,  // index 66
     7,  // index 67
     7,  // index 68
     7,  // index 69
     6,  // index 70
     6,  // index 71
     6,  // index 72
     6,  // index 73
     6,  // index 74
     5,  // index 75
     5,  // index 76
     5,  // index 77
     5,  // index 78
     5,  // index 79
     4,  // index 80
     4,  // index 81
     4,  // index 82
     4,  // index 83
     4,  // index 84
     3,  // index 85
     3,  // index 86
     3,  // index 87
     3,  // index 88
     3,  // index 89
     2,  // index 90
     2,  // index 91
     2,  // index 92
     2,  // index 93
     2,  // index 94
//     1,  // index 95
//     1,  // index 96
//     1,  // index 97
//     1,  // index 98
//     1  // index 99
	 1,
	 1,
	 1,
	 1,
	 1
};
typedef struct {
    int32_t a_error;
    int32_t b_error;
    int32_t a_steps;
    int32_t b_steps;
    int8_t signA;
    int8_t signB;
    int32_t steps_remaining;
    int8_t tick_counter;
    int32_t x;
    int32_t y;
    int32_t index;
} CoreXYMotion;

typedef struct {
    int32_t position_steps;
    int32_t step_count_remaining;
    int32_t step_delay_ticks;
    int8_t direction;
    int8_t tick_counter;
    uint8_t enabled;
} ZAxis;

volatile CoreXYMotion corexy = {0};
volatile ZAxis axis2 = {0};

static inline void step_motor_A(void);
static inline void step_motor_B(void);

#define CLAMP(val, min, max) ((val < min) ? min : ((val > max) ? max : val))

// === TIMER SETUP ===
void stepper_timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

//    TIM2->PSC = 79999;          // 1 µs resolution at 8 MHz
    TIM2->PSC = 7;
    TIM2->ARR = 50; //60


    TIM2->DIER |= TIM_DIER_UIE;       // Enable Update Interrupt
    TIM2->EGR |= TIM_EGR_UG;          // Force update event to load settings
    TIM2->CR1 |= TIM_CR1_CEN;         // Start timer

    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;

    __disable_irq();
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
    __enable_irq();
}

void stepper_control_init(void) {
	init_axis0_gpio();
	init_axis1_gpio();
	init_axis2_gpio();
    stepper_timer_init();


    corexy.x = 0;
    corexy.y = 0;
    corexy.signA = 0;
    corexy.signB = 0;
    corexy.a_steps = 0;
    corexy.b_steps = 0;
    corexy.tick_counter = 0;

    axis2.position_steps = 0;
    axis2.step_count_remaining = 0;
    axis2.step_delay_ticks = 6;
    axis2.direction = 0;
    axis2.tick_counter = 0;
    axis2.enabled = 0;

}


// === ISR ===
void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;


	corexy.tick_counter--;

	if(corexy.tick_counter <= 0){
		if (corexy.a_steps >= corexy.b_steps) {
			step_motor_A();
			corexy.a_error += corexy.b_steps;
			if (corexy.a_error >= corexy.a_steps) {
				step_motor_B();
				corexy.a_error -= corexy.a_steps;
			}
		} else {
			step_motor_B();
			corexy.b_error += corexy.a_steps;
			if (corexy.b_error >= corexy.b_steps) {
				step_motor_A();
				corexy.b_error -= corexy.b_steps;
			}
		}

		corexy.tick_counter = delay_table[corexy.index];
	}

//	 ----- Z Axis -----
	if (axis2.tick_counter > 0) axis2.tick_counter--;

	if (axis2.tick_counter == 0 && axis2.enabled) {
		if (axis2.step_count_remaining == 0) {
			axis2.enabled = 0;
		} else {
			if (axis2.direction > 0)
				AXIS2_GPIO_PORT->BSRR = (1 << AXIS2_DIR_PIN);
			else
				AXIS2_GPIO_PORT->BSRR = (1 << (AXIS2_DIR_PIN + 16));

			AXIS2_GPIO_PORT->BSRR = (1 << AXIS2_STEP_PIN);
			AXIS2_GPIO_PORT->BRR  = (1 << AXIS2_STEP_PIN);

			axis2.position_steps += axis2.direction;
			axis2.step_count_remaining--;
			axis2.tick_counter = axis2.step_delay_ticks;
		}
	}

}


// === Velocity Planner ===
// vx, vy range is arbitrary — just needs to be signed
//void plan_velocity(int16_t vx, int16_t vy) {
//	char msg[64];
//	snprintf(msg, sizeof(msg), "%d, %d\r\n", corexy.x, corexy.y);
//	SerialOutputString((uint8_t*)msg, &USART1_PORT);
//
//	if (vx == 0 && vy == 0) {
//	        corexy.a_steps = 0;
//	        corexy.b_steps = 0;
//	        corexy.signA = 0;
//	        corexy.signB = 0;
//	        return;  // ← don't set direction or error values
//	    }
//
//
//    int vA = vx + vy;
//    int vB = vx - vy;
//
//    corexy.signA = (vA >= 0) ? 1 : -1;
//    corexy.signB = (vB >= 0) ? 1 : -1;
//    corexy.a_steps = abs(vA);
//    corexy.b_steps = abs(vB);
//    corexy.steps_remaining = (corexy.a_steps > corexy.b_steps) ? corexy.a_steps : corexy.b_steps;
//
//    corexy.a_error = 0;
//    corexy.b_error = 0;
//}
void plan_velocity(int8_t vx, int8_t vy, uint8_t index) {
//	char msg[64];
//	snprintf(msg, sizeof(msg), "%d, %d\r\n", vx, vy);
//	SerialOutputString((uint8_t*)msg, &USART1_PORT);

	if (vx == 0 && vy == 0) {
//		char msg[64];
//		snprintf(msg, sizeof(msg), "s");
//		SerialOutputString((uint8_t*)msg, &USART1_PORT);

        corexy.a_steps = 0;
        corexy.b_steps = 0;
        corexy.signA = 0;
        corexy.signB = 0;
        return;
    }

    int vA = vx + vy;
    int vB = vx - vy;

    corexy.signA = (vA >= 0) ? 1 : -1;
    corexy.signB = (vB >= 0) ? 1 : -1;
    corexy.a_steps = abs(vA);
    corexy.b_steps = abs(vB);
    corexy.steps_remaining = (corexy.a_steps > corexy.b_steps) ? corexy.a_steps : corexy.b_steps;

    corexy.a_error = 0;
    corexy.b_error = 0;


	if (corexy.signA > 0)
		AXIS0_GPIO_PORT->BSRR = (1 << AXIS0_DIR_PIN);
	else
		AXIS0_GPIO_PORT->BSRR = (1 << (AXIS0_DIR_PIN + 16)); // Reset bit


	if (corexy.signB > 0)
		AXIS1_GPIO_PORT->BSRR = (1 << AXIS1_DIR_PIN);
	else
		AXIS1_GPIO_PORT->BSRR = (1 << (AXIS1_DIR_PIN + 16));

    // === Vector magnitude normalization ===
//    float fx = (float)vx;
//    float fy = (float)vy;
//    int index = (int)(sqrtf((float)(vx * vx + vy * vy)) + 0.5f);
    if (index < 0) index = 0;
    if (index > 100) index = 99;

    corexy.index = index;

//	char buf[64];
//	snprintf(buf, sizeof(buf), "%d\r\n", corexy.index);
//	SerialOutputString((uint8_t*)buf, &USART1_PORT);

//    corexy.tick_counter = delay_table[index];
}


// === Motor Step Functions (replace with actual GPIO later) ===
//static void step_motor_A() {
//	if (corexy.signA == 0) return;
//
//    int dx = corexy.signA;
//    int dy = corexy.signA;
//
//    int32_t x_next = corexy.x + dx;
//    int32_t y_next = corexy.y + dy;
//
//    if (x_next < X_MIN || x_next > X_MAX || y_next < Y_MIN || y_next > Y_MAX) return;
//
//    // Set direction pin
//    if (corexy.signA > 0)
//        AXIS0_GPIO_PORT->BSRR = (1 << AXIS0_DIR_PIN);
//    else
//        AXIS0_GPIO_PORT->BSRR = (1 << (AXIS0_DIR_PIN + 16)); // Reset bit
//
//    // Pulse step pin
//    AXIS0_GPIO_PORT->BSRR = (1 << AXIS0_STEP_PIN);              // STEP HIGH
//    AXIS0_GPIO_PORT->BSRR = (1 << (AXIS0_STEP_PIN + 16));       // STEP LOW
//
//    corexy.x = x_next;
//    corexy.y = y_next;
//}
//
//static void step_motor_B() {
//	if (corexy.signB == 0) return;
//
//    int dx = corexy.signB;
//    int dy = -corexy.signB;
//
//    int32_t x_next = corexy.x + dx;
//    int32_t y_next = corexy.y + dy;
//
//    if (x_next < X_MIN || x_next > X_MAX || y_next < Y_MIN || y_next > Y_MAX) return;
//
//    // Set direction pin
//    if (corexy.signB > 0)
//        AXIS1_GPIO_PORT->BSRR = (1 << AXIS1_DIR_PIN);
//    else
//        AXIS1_GPIO_PORT->BSRR = (1 << (AXIS1_DIR_PIN + 16));
//
//    // Pulse step pin
//    AXIS1_GPIO_PORT->BSRR = (1 << AXIS1_STEP_PIN);
//    AXIS1_GPIO_PORT->BSRR = (1 << (AXIS1_STEP_PIN + 16));
//
//    corexy.x = x_next;
//    corexy.y = y_next;
//}

// === Z-Axis Command Handler ===
void set_z_target(int16_t z_target) {
    if (!axis2.enabled && z_target >= 0) {
        z_target = CLAMP(z_target, AXIS2_LIMIT_MIN, AXIS2_LIMIT_MAX);
        int delta = z_target - axis2.position_steps;
        if (delta != 0) {
            axis2.direction = (delta > 0) ? 1 : -1;
            axis2.step_count_remaining = abs(delta);
            axis2.step_delay_ticks = 2;
            axis2.enabled = 1;
        }
    }
}


// === INLINE STEPPING ===
static inline void step_motor_A(void) {
    if (corexy.signA == 0) return;

    int32_t x_next = corexy.x + corexy.signA;
    int32_t y_next = corexy.y + corexy.signA;
    if (x_next < X_MIN || x_next > X_MAX || y_next < Y_MIN || y_next > Y_MAX) return;

//    AXIS0_GPIO_PORT->BSRR = (corexy.signA > 0) ? (1 << AXIS0_DIR_PIN) : (1 << (AXIS0_DIR_PIN + 16));
    AXIS0_GPIO_PORT->BSRR = (1 << AXIS0_STEP_PIN);
    AXIS0_GPIO_PORT->BSRR = (1 << (AXIS0_STEP_PIN + 16));

	corexy.x = x_next;
	corexy.y = y_next;
}

static inline void step_motor_B(void) {
    if (corexy.signB == 0) return;

    //next steps
    int32_t x_next = corexy.x + corexy.signB;
    int32_t y_next = corexy.y - corexy.signB;
    if (x_next < X_MIN || x_next > X_MAX || y_next < Y_MIN || y_next > Y_MAX) return;

//    AXIS1_GPIO_PORT->BSRR = (corexy.signB > 0) ? (1 << AXIS1_DIR_PIN) : (1 << (AXIS1_DIR_PIN + 16));
    AXIS1_GPIO_PORT->BSRR = (1 << AXIS1_STEP_PIN);
    AXIS1_GPIO_PORT->BSRR = (1 << (AXIS1_STEP_PIN + 16));

	corexy.x = x_next;
	corexy.y = y_next;

}




void init_axis0_gpio(void) {
    // --- Enable clock for GPIO port used by AXIS0 ---
    if (AXIS0_GPIO_PORT == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if (AXIS0_GPIO_PORT == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    else if (AXIS0_GPIO_PORT == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    else if (AXIS0_GPIO_PORT == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    else if (AXIS0_GPIO_PORT == GPIOE) RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    else if (AXIS0_GPIO_PORT == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

    // --- STEP Pin Setup ---
    AXIS0_GPIO_PORT->MODER   &= ~(3 << (AXIS0_STEP_PIN * 2));         // Clear mode bits (input mode)
    AXIS0_GPIO_PORT->MODER   |=  (1 << (AXIS0_STEP_PIN * 2));         // Set as general-purpose output mode
    AXIS0_GPIO_PORT->OTYPER  &= ~(1 << AXIS0_STEP_PIN);               // Output push-pull
    AXIS0_GPIO_PORT->OSPEEDR |=  (3 << (AXIS0_STEP_PIN * 2));         // High speed output
    AXIS0_GPIO_PORT->PUPDR   &= ~(3 << (AXIS0_STEP_PIN * 2));         // No pull-up/pull-down

    // --- DIR Pin Setup ---
    AXIS0_GPIO_PORT->MODER   &= ~(3 << (AXIS0_DIR_PIN * 2));          // 1. Clear MODER bits (sets to input)
    AXIS0_GPIO_PORT->MODER   |=  (1 << (AXIS0_DIR_PIN * 2));          // 2. Set MODER to 01 (output mode)
    AXIS0_GPIO_PORT->OTYPER  &= ~(1 << AXIS0_DIR_PIN);                // 3. Set output type to push-pull
    AXIS0_GPIO_PORT->OSPEEDR |=  (3 << (AXIS0_DIR_PIN * 2));          // 4. Set speed to high (11)
    AXIS0_GPIO_PORT->PUPDR   &= ~(3 << (AXIS0_DIR_PIN * 2));          // 5. No pull-up / pull-down

    // --- LIMIT Pin Setup ---
    AXIS0_GPIO_PORT->MODER   &= ~(3 << (AXIS0_LIMIT_PIN * 2));        // Set as input (00)
    AXIS0_GPIO_PORT->PUPDR   &= ~(3 << (AXIS0_LIMIT_PIN * 2));        // No pull-up/pull-down


    // Optional: enable pull-up if limit switch is normally open
    // AXIS0_GPIO_PORT->PUPDR |= (1 << (AXIS0_LIMIT_PIN * 2));        // Pull-up
}

void init_axis1_gpio(void) {
    // --- Enable clock for GPIO port used by AXIS0 ---
    if (AXIS1_GPIO_PORT == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if (AXIS1_GPIO_PORT == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    else if (AXIS1_GPIO_PORT == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    else if (AXIS1_GPIO_PORT == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    else if (AXIS1_GPIO_PORT == GPIOE) RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    else if (AXIS1_GPIO_PORT == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

    // --- STEP Pin Setup ---
    AXIS1_GPIO_PORT->MODER   &= ~(3 << (AXIS1_STEP_PIN * 2));         // Clear mode bits (input mode)
    AXIS1_GPIO_PORT->MODER   |=  (1 << (AXIS1_STEP_PIN * 2));         // Set as general-purpose output mode
    AXIS1_GPIO_PORT->OTYPER  &= ~(1 << AXIS1_STEP_PIN);               // Output push-pull
    AXIS1_GPIO_PORT->OSPEEDR |=  (3 << (AXIS1_STEP_PIN * 2));         // High speed output
    AXIS1_GPIO_PORT->PUPDR   &= ~(3 << (AXIS1_STEP_PIN * 2));         // No pull-up/pull-down

    // --- DIR Pin Setup ---
    AXIS1_GPIO_PORT->MODER   &= ~(3 << (AXIS1_DIR_PIN * 2));          // 1. Clear MODER bits (sets to input)
    AXIS1_GPIO_PORT->MODER   |=  (1 << (AXIS1_DIR_PIN * 2));          // 2. Set MODER to 01 (output mode)
    AXIS1_GPIO_PORT->OTYPER  &= ~(1 << AXIS1_DIR_PIN);                // 3. Set output type to push-pull
    AXIS1_GPIO_PORT->OSPEEDR |=  (3 << (AXIS1_DIR_PIN * 2));          // 4. Set speed to high (11)
    AXIS1_GPIO_PORT->PUPDR   &= ~(3 << (AXIS1_DIR_PIN * 2));          // 5. No pull-up / pull-down

    // --- LIMIT Pin Setup ---
    AXIS1_GPIO_PORT->MODER   &= ~(3 << (AXIS1_LIMIT_PIN * 2));        // Set as input (00)
    AXIS1_GPIO_PORT->PUPDR   &= ~(3 << (AXIS1_LIMIT_PIN * 2));        // No pull-up/pull-down
    // Optional: enable pull-up if limit switch is normally open
    // AXIS0_GPIO_PORT->PUPDR |= (1 << (AXIS0_LIMIT_PIN * 2));        // Pull-up
}

void init_axis2_gpio(void) {
    if (AXIS2_GPIO_PORT == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if (AXIS2_GPIO_PORT == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    else if (AXIS2_GPIO_PORT == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    else if (AXIS2_GPIO_PORT == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    else if (AXIS2_GPIO_PORT == GPIOE) RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    else if (AXIS2_GPIO_PORT == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

    AXIS2_GPIO_PORT->MODER   &= ~(3 << (AXIS2_STEP_PIN * 2));
    AXIS2_GPIO_PORT->MODER   |=  (1 << (AXIS2_STEP_PIN * 2));
    AXIS2_GPIO_PORT->OTYPER  &= ~(1 << AXIS2_STEP_PIN);
    AXIS2_GPIO_PORT->OSPEEDR |=  (3 << (AXIS2_STEP_PIN * 2));
    AXIS2_GPIO_PORT->PUPDR   &= ~(3 << (AXIS2_STEP_PIN * 2));

    AXIS2_GPIO_PORT->MODER   &= ~(3 << (AXIS2_DIR_PIN * 2));
    AXIS2_GPIO_PORT->MODER   |=  (1 << (AXIS2_DIR_PIN * 2));
    AXIS2_GPIO_PORT->OTYPER  &= ~(1 << AXIS2_DIR_PIN);
    AXIS2_GPIO_PORT->OSPEEDR |=  (3 << (AXIS2_DIR_PIN * 2));
    AXIS2_GPIO_PORT->PUPDR   &= ~(3 << (AXIS2_DIR_PIN * 2));

    AXIS2_GPIO_PORT->MODER   &= ~(3 << (AXIS2_LIMIT_PIN * 2));
    AXIS2_GPIO_PORT->PUPDR   &= ~(3 << (AXIS2_LIMIT_PIN * 2));
}
