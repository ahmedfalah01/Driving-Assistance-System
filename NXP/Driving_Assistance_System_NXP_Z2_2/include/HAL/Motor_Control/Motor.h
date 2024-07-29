/*
 * Motor.h
 *
 *  Created on: Jul 9, 2024
 *      Author: ahmed
 */

#ifndef HAL_MOTOR_CONTROL_MOTOR_H_
#define HAL_MOTOR_CONTROL_MOTOR_H_


#include "pin_mux.h"
#include "pwm_pal1.h"
#include "pwm_pal2.h"

/**************************************************************/
/**************		Motor Configurations	**************/
/************	FRONT RIGHT MOTOR	************/
// PORT CONFIG
#define FR_MOTOR_IN1_PORT			PTJ
#define FR_MOTOR_IN2_PORT			PTJ
#define FR_MOTOR_SPEED_EN_PORT		PTJ
// PIN CONFIG
#define FR_MOTOR_IN1_PIN			2
#define FR_MOTOR_IN2_PIN			1
#define FR_MOTOR_SPEED_EN_PIN		0
/************	FRONT LEFT MOTOR	************/
// PORT CONFIG
#define FL_MOTOR_IN1_PORT			PTD
#define FL_MOTOR_IN2_PORT           PTD
#define FL_MOTOR_SPEED_EN_PORT      PTD
// PIN CONFIG
#define FL_MOTOR_IN1_PIN			15
#define FL_MOTOR_IN2_PIN			14
#define FL_MOTOR_SPEED_EN_PIN		13
/************	BACK RIGHT MOTOR	************/
// PORT CONFIG
#define BR_MOTOR_IN1_PORT			PTI
#define BR_MOTOR_IN2_PORT           PTI
#define BR_MOTOR_SPEED_EN_PORT      PTI
// PIN CONFIG
#define BR_MOTOR_IN1_PIN			12
#define BR_MOTOR_IN2_PIN			13
#define BR_MOTOR_SPEED_EN_PIN		11
/************	BACK LEFT MOTOR	************/
// PORT CONFIG
#define BL_MOTOR_IN1_PORT			PTH
#define BL_MOTOR_IN2_PORT           PTH
#define BL_MOTOR_SPEED_EN_PORT      PTI
// PIN CONFIG
#define BL_MOTOR_IN1_PIN			3
#define BL_MOTOR_IN2_PIN			4
#define BL_MOTOR_SPEED_EN_PIN		3
/**************************************************************/
/**************		Motors Enumeration		**************/
/**************		PWM Channels		**************/
enum Motor {
	Front_Right = 19, Front_Left = 25, Back_Right = 14, Back_Left = 31,
};
enum Direction {
	Clockwise = 0, CounterClockwise = 1,
};


// Function prototypes
void MotorInit(void);
void set_motor_direction(uint8_t motor, uint8_t direction);
void set_motor_speed(uint8_t motor, uint32_t speed);

#endif /* HAL_MOTOR_CONTROL_MOTOR_H_ */
