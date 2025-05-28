#ifndef CONFIG_H
#define CONFIG_H


#define VELOCITY_COMMAND_RANGE 128 //range of inputted velocites



// Axis 0 (X)
#define AXIS0_GPIO_PORT     GPIOD
#define AXIS0_STEP_PIN      11
#define AXIS0_DIR_PIN       13
#define AXIS0_LIMIT_PIN		0


// Axis 1 (Y)
#define AXIS1_GPIO_PORT     GPIOD
#define AXIS1_STEP_PIN      2
#define AXIS1_DIR_PIN       4
#define AXIS1_LIMIT_PIN		6


// Axis 2 (Z)
#define AXIS2_GPIO_PORT     GPIOD
#define AXIS2_STEP_PIN      1
#define AXIS2_DIR_PIN       3
#define AXIS2_LIMIT_PIN     5


#define X_MIN 0
//#define X_MAX 30
#define X_MAX 40000
#define Y_MIN 0
//#define Y_MAX 30
#define Y_MAX 56000
#define Z_MIN 0
#define Z_MAX 11300


#endif
