/*
 * PID.c
 *
 *  Created on: Jul 10, 2024
 *      Author: ahmed
 */

#include "PID.h"



void PID_Init(void){

	/* Initialize selected pins and configure external interrupts
	 *  -   See PinSettings component for more info
	 */
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	/* Initialize STM */
	STM_DRV_Init(INST_STM1, &stm1_InitConfig0);
	/* Compute the number of ticks from microseconds */
	STM_DRV_ComputeTicksByUs(INST_STM1, 100000U, &ticks);
	/* Initialize channel 0 */
	stm1_ChnConfig0.compareValue = ticks;
	STM_DRV_InitChannel(INST_STM1, &stm1_ChnConfig0);
	/* Enable Interrupt for STM2 channel 0 */
	INT_SYS_EnableIRQ(STM2_Ch0_IRQn);/* Start running the common timer counter */
	STM_DRV_StartTimer(INST_STM1);
}


void STM2_Ch0_IRQHandler(void) {
    /* Clear channel interrupt flag */
    STM_DRV_ClearStatusFlags(INST_STM1, stm1_ChnConfig0.channel);
	/*Timer routine every 10ms for PID */
	pid_outputs[0] = Kp			* (motor_velocities[0]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[0] - prev_encoder_counts[0]))
							/ ENCODER_CPR) / (PID_INTERVAL)));
	pid_outputs[1] = Kp
			* (motor_velocities[1]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[1] - prev_encoder_counts[1]))
							* ENCODER_CPR) / (PID_INTERVAL)));
	pid_outputs[2] = Kp
			* (motor_velocities[2]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[2] - prev_encoder_counts[2]))
							* ENCODER_CPR) / (PID_INTERVAL)));
	pid_outputs[3] = Kp
			* (motor_velocities[3]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[3] - prev_encoder_counts[3]))
							* ENCODER_CPR) / (PID_INTERVAL)));
	prev_encoder_counts[0] = encoder_counts[0];
	prev_encoder_counts[1] = encoder_counts[1];
	prev_encoder_counts[2] = encoder_counts[2];
	prev_encoder_counts[3] = encoder_counts[3];

	//set motor direction based on the pid sign
	Motor_direction[0] =
			(pid_outputs[0] > 0) ? Clockwise : CounterClockwise;
	Motor_direction[1] =
			(pid_outputs[1] > 0) ? Clockwise : CounterClockwise;
	Motor_direction[2] =
			(pid_outputs[2] > 0) ? Clockwise : CounterClockwise;
	Motor_direction[3] =
			(pid_outputs[3] > 0) ? Clockwise : CounterClockwise;
	set_motor_direction(Front_Right, Motor_direction[0] );
	set_motor_direction(Front_Left, Motor_direction[1]);
	set_motor_direction(Back_Right, Motor_direction[2]);
	set_motor_direction(Back_Left, Motor_direction[3]);

	set_motor_speed(Front_Right, pid_outputs[0] > 0? pid_outputs[0]:pid_outputs[0] * -1);
	set_motor_speed(Front_Left, pid_outputs[1] > 0? pid_outputs[1]:pid_outputs[1] * -1);
	set_motor_speed(Back_Right, pid_outputs[2] > 0? pid_outputs[2]:pid_outputs[2] * -1);
	set_motor_speed(Back_Left, pid_outputs[3] > 0? pid_outputs[3]:pid_outputs[3] * -1);
    /* Increase the number of ticks in compare register to create a periodic for next event */
    STM_DRV_IncrementTicks(INST_STM1, stm1_ChnConfig0.channel, ticks);

}
