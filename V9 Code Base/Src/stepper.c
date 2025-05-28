/**
 * @file stepper_timer.c
 * @brief CoreXY + Z axis stepper motor control using TIM2 on STM32F303.
 *
 * Provides timer ISR-based step pulse control for CoreXY configuration on X/Y,
 * plus a linear Z-axis with independent step timing. Velocity control, GPIO
 * initialization, and motion planning are included.
 *
 * Delay profiles for acceleration are stored in an external delay table.
 */

#include "stm32f303xc.h"
#include "config.h"
#include <stdint.h>
#include <stdlib.h>
#include "stepper.h"
#include "step_delay_table.h"

// === Constants ===
#define CLAMP(val, min, max) ((val < min) ? min : ((val > max) ? max : val))


// === Motion Structures ===
typedef struct {
    int32_t a_error, b_error;
    int32_t a_steps, b_steps;
    int8_t signA, signB;
    int32_t steps_remaining;
    int8_t tick_counter;
    int32_t x, y;
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

// === Global State ===
volatile CoreXYMotion corexy = {0};
volatile ZAxis axis2 = {0};

// === Internal Helpers ===
static inline void step_motor_A(void);
static inline void step_motor_B(void);

// === Stepper Timer Setup ===
void stepper_timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 7;             // Prescaler for 1 MHz (assuming 8 MHz input clock)
    TIM2->ARR = 50;            // Auto-reload for interrupt rate
    TIM2->DIER |= TIM_DIER_UIE;
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

    corexy = (CoreXYMotion){0};
    axis2 = (ZAxis){.step_delay_ticks = 6};
}

// === Timer Interrupt Handler ===
void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;
    corexy.tick_counter--;

    if (corexy.tick_counter <= 0) {
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
                corexy.b_error -= corexy.a_steps;
            }
        }
        corexy.tick_counter = delay_table[corexy.index];
    }

    if (axis2.tick_counter > 0) axis2.tick_counter--;

    if (axis2.tick_counter == 0 && axis2.enabled) {
        if (axis2.step_count_remaining == 0) {
            axis2.enabled = 0;
        } else {
            AXIS2_GPIO_PORT->BSRR = (axis2.direction > 0) ? (1 << AXIS2_DIR_PIN)
                                                          : (1 << (AXIS2_DIR_PIN + 16));
            AXIS2_GPIO_PORT->BSRR = (1 << AXIS2_STEP_PIN);
            AXIS2_GPIO_PORT->BRR  = (1 << AXIS2_STEP_PIN);
            axis2.position_steps += axis2.direction;
            axis2.step_count_remaining--;
            axis2.tick_counter = axis2.step_delay_ticks;
        }
    }
}

// === Motion Planning ===
void plan_velocity(int8_t vx, int8_t vy, uint8_t index) {
    if (vx == 0 && vy == 0) {
        corexy.a_steps = corexy.b_steps = 0;
        corexy.signA = corexy.signB = 0;
        return;
    }

    int vA = vx + vy;
    int vB = vx - vy;
    corexy.signA = (vA >= 0) ? 1 : -1;
    corexy.signB = (vB >= 0) ? 1 : -1;
    corexy.a_steps = abs(vA);
    corexy.b_steps = abs(vB);
    corexy.steps_remaining = (corexy.a_steps > corexy.b_steps) ? corexy.a_steps : corexy.b_steps;
    corexy.a_error = corexy.b_error = 0;

    corexy.index = CLAMP(index, 0, 99);

    AXIS0_GPIO_PORT->BSRR = (corexy.signA > 0) ? (1 << AXIS0_DIR_PIN) : (1 << (AXIS0_DIR_PIN + 16));
    AXIS1_GPIO_PORT->BSRR = (corexy.signB > 0) ? (1 << AXIS1_DIR_PIN) : (1 << (AXIS1_DIR_PIN + 16));
}

// === Z-Axis Motion ===
void set_z_target(int16_t z_target) {
    if (!axis2.enabled && z_target >= 0) {
        z_target = CLAMP(z_target, Z_MIN, Z_MAX);
        int delta = z_target - axis2.position_steps;
        if (delta != 0) {
            axis2.direction = (delta > 0) ? 1 : -1;
            axis2.step_count_remaining = abs(delta);
            axis2.step_delay_ticks = 3;
            axis2.enabled = 1;
        }
    }
}

// === Step Functions ===
static inline void step_motor_A(void) {
    if (corexy.signA == 0) return;
    int32_t x_next = corexy.x + corexy.signA;
    int32_t y_next = corexy.y + corexy.signA;
    if (x_next < X_MIN || x_next > X_MAX || y_next < Y_MIN || y_next > Y_MAX) return;
    AXIS0_GPIO_PORT->BSRR = (1 << AXIS0_STEP_PIN);
    AXIS0_GPIO_PORT->BSRR = (1 << (AXIS0_STEP_PIN + 16));
    corexy.x = x_next;
    corexy.y = y_next;
}

static inline void step_motor_B(void) {
    if (corexy.signB == 0) return;
    int32_t x_next = corexy.x + corexy.signB;
    int32_t y_next = corexy.y - corexy.signB;
    if (x_next < X_MIN || x_next > X_MAX || y_next < Y_MIN || y_next > Y_MAX) return;
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
