#ifndef CONFIG_H
#define CONFIG_H


#define VELOCITY_COMMAND_RANGE 128 //range of inputted velocites


#define DEFAULT_MAX_VELOCITY 27000
#define DEFAULT_MAX_ACCEL 	 100000

#define HOMING_SPEED 		 50 //40us increments


// Axis 0 (X)
#define AXIS0_GPIO_PORT     GPIOD
#define AXIS0_STEP_PIN      11
#define AXIS0_DIR_PIN       13
#define AXIS0_LIMIT_PIN		0
#define AXIS0_VEL_MAX       27000 // steps/sec
#define AXIS0_ACCEL_MAX     1000000 //steps/sec2
#define AXIS0_LIMIT_MIN     0 //steps
#define AXIS0_LIMIT_MAX     18900 //steps
#define AXIS0_DIRECTION  	-1  // -1 to reverse axis direction

// Axis 1 (Y)
#define AXIS1_GPIO_PORT     GPIOD
#define AXIS1_STEP_PIN      2
#define AXIS1_DIR_PIN       4
#define AXIS1_LIMIT_PIN		6
#define AXIS1_VEL_MAX       27000 // steps/sec
#define AXIS1_ACCEL_MAX     100000 //steps/sec2
#define AXIS1_LIMIT_MIN     0 //steps
#define AXIS1_LIMIT_MAX     25500 //steps
#define AXIS1_DIRECTION  	-1  // -1 to reverse axis direction

// Axis 2 (Z)
#define AXIS2_GPIO_PORT     GPIOD
#define AXIS2_STEP_PIN      1
#define AXIS2_DIR_PIN       3
#define AXIS2_LIMIT_PIN     5
#define AXIS2_VEL_MAX       27000
#define AXIS2_ACCEL_MAX     100000
#define AXIS2_LIMIT_MIN     0
#define AXIS2_LIMIT_MAX     8940
#define AXIS2_DIRECTION     1


//adjust for faster pulses (smaller values may be unstable)
#define PULSE_WIDTH_US 10

#endif

#define X_MIN 0
//#define X_MAX 30
#define X_MAX 40000
#define Y_MIN 0
//#define Y_MAX 30
#define Y_MAX 56000



//todo
//add guard for low acceleration and high velocity to make sure can malloc
//fi bug where reverse too close to end and it stops without restarting direction
//fix step pin low in intetteupt for tim2 and tim3
//reverse directions, make sure to fix homing direction code at same time
//fix interrupt code so not hard coded for X axis
//add stopping to generate lookup not command parser
//FIX INPUT IF ONLY ONE ACIS GIVEN
//loook up table too big

