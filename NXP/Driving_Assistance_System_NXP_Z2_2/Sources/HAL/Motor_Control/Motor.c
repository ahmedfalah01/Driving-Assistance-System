/*
 * Motor.c
 *
 *  Created on: Jul 9, 2024
 *      Author: ahmed
 */

#include "Motor.h"
void MotorInit(void){
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	PWM_Init(&pwm_pal1Instance, &pwm_pal1Configs);
	PWM_Init(&pwm_pal2Instance, &pwm_pal2Configs);
}
void set_motor_direction(uint8_t motor, uint8_t direction) {
	switch (motor) {
	case Front_Right: // Front Right Motor
		if (direction == 0) { // Forward
			PINS_DRV_SetPins(FR_MOTOR_IN1_PORT, 1 << FR_MOTOR_IN1_PIN);
			PINS_DRV_ClearPins(FR_MOTOR_IN2_PORT, 1 << FR_MOTOR_IN2_PIN);
		} else { // Backward
			PINS_DRV_ClearPins(FR_MOTOR_IN1_PORT, 1 << FR_MOTOR_IN1_PIN);
			PINS_DRV_SetPins(FR_MOTOR_IN2_PORT, 1 << FR_MOTOR_IN2_PIN);
		}
		break;
	case Front_Left: // Front Left Motor
		if (direction == 0) { // Forward
			PINS_DRV_SetPins(FL_MOTOR_IN1_PORT, 1 << FL_MOTOR_IN1_PIN);
			PINS_DRV_ClearPins(FL_MOTOR_IN2_PORT, 1 << FL_MOTOR_IN2_PIN);
		} else { // Backward
			PINS_DRV_ClearPins(FL_MOTOR_IN1_PORT, 1 << FL_MOTOR_IN1_PIN);
			PINS_DRV_SetPins(FL_MOTOR_IN2_PORT, 1 << FL_MOTOR_IN2_PIN);
		}
		break;
	case Back_Right: // Back Right Motor
		if (direction == 0) { // Forward
			PINS_DRV_SetPins(BR_MOTOR_IN1_PORT, 1 << BR_MOTOR_IN1_PIN);
			PINS_DRV_ClearPins(BR_MOTOR_IN2_PORT, 1 << BR_MOTOR_IN2_PIN);
		} else { // Backward
			PINS_DRV_ClearPins(BR_MOTOR_IN1_PORT, 1 << BR_MOTOR_IN1_PIN);
			PINS_DRV_SetPins(BR_MOTOR_IN2_PORT, 1 << BR_MOTOR_IN2_PIN);
		}
		break;
	case Back_Left: // Back Left Motor
		if (direction == 0) { // Forward
			PINS_DRV_SetPins(BL_MOTOR_IN1_PORT, 1 << BL_MOTOR_IN1_PIN);
			PINS_DRV_ClearPins(BL_MOTOR_IN2_PORT, 1 << BL_MOTOR_IN2_PIN);
		} else { // Backward
			PINS_DRV_ClearPins(BL_MOTOR_IN1_PORT, 1 << BL_MOTOR_IN1_PIN);
			PINS_DRV_SetPins(BL_MOTOR_IN2_PORT, 1 << BL_MOTOR_IN2_PIN);
		}
		break;
	default:
		break;
	}
}
void set_motor_speed(uint8_t motor, uint32_t speed) {
	/* Update PWM duty cycle for the specified motor */
	if (motor == Front_Right || motor == Back_Right) {
		PWM_UpdateDuty(&pwm_pal2Instance, motor, speed);
	} else if (motor == Front_Left || motor == Back_Left) {
		PWM_UpdateDuty(&pwm_pal1Instance, motor, speed);
	}
}
