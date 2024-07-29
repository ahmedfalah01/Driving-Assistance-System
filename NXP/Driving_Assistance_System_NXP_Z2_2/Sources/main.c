/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ###################################################################
 **     Filename    : main.c
 **     Processor   : MPC574xG
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 01.00
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "smpu_driver.h"
#include "stm1.h"
#include "interrupt_manager.h"
#include "pwm_pal1.h"
#include "pwm_pal2.h"
#include <math.h>
#include <string.h>
volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */
///* SMPU 1 - RAM protection */
//#define INST_SMPU 1U
///* Status variable */
//status_t status;
///* Region count */
//#define REGION_CNT (1U)
#define PI 3.14159265358979323846
#define WHEEL_DIAMETER 6.5 // 65mm in CM
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define ENCODER_CPR 492
#define PID_INTERVAL 0.01 // 10ms
// Encoder counts for each motor
//FR FL BR BL
//volatile int32_t *encoder_counts = (int32_t *) 0x40080000;
volatile int32_t encoder_counts[4] = { 0 };
volatile int32_t prev_encoder_counts[4] = { 0 };
volatile float motor_velocities[4] = { 0 };
volatile float pid_outputs[4] = { 0 };
volatile int32_t Motor_direction[4] = { 0 };

struct STM_Readings{
	float straight;
	float diagonal;
	float ir;
};
struct STM_Readings sensor_readings[4];
// PID constants
const float Kp = 5000 / 88;
const float Ki = 0.1;
float integral_errors[4] = { 0 };
char last_command = 's';
uint32_t ticks; /* The number of ticks */

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
///**************************************************************/
///**************		Encoders Configurations		**************/
///************	FRONT RIGHT MOTOR	************/
//// PORT CONFIG
//#define FrontRight_ENC_A_PORT	PTE
//#define FrontRight_ENC_B_PORT	PTE
//// PIN CONFIG
////PE4 PE10
//#define FrontRight_ENC_A_PIN	4
//#define FrontRight_ENC_B_PIN	10
///************	FRONT LEFT MOTOR	************/
//// PORT CONFIG
//#define FrontLeft_ENC_A_PORT	PTA
//#define FrontLeft_ENC_B_PORT	PTE
//// PIN CONFIG
////PTA 16u PTE 22u
//#define FrontLeft_ENC_A_PIN		12u
//#define FrontLeft_ENC_B_PIN		6u
///************	BACK RIGHT MOTOR	************/
//// PORT CONFIG
//#define BackRight_ENC_A_PORT	PTE
//#define BackRight_ENC_B_PORT	PTG
//// PIN CONFIG
////PTE 23u PTG 15u
//#define BackRight_ENC_A_PIN		7u
//#define BackRight_ENC_B_PIN		8u
///************	BACK LEFT MOTOR	************/
//// PORT CONFIG
//#define BackLeft_ENC_A_PORT		PTA
//#define BackLeft_ENC_B_PORT		PTA
//// PIN CONFIG
////PTA 17u PTA 4u
//#define BackLeft_ENC_A_PIN		6u
//#define BackLeft_ENC_B_PIN		14u
///**************************************************************/
///**************		EIRQ Configurations	**************/
///************	FRONT RIGHT MOTOR	************/
//#define FrontRight_ENC_A_EIRQ	9u
//#define FrontRight_ENC_B_EIRQ	10u
///************	FRONT LEFT MOTOR	************/
//#define FrontLeft_ENC_A_EIRQ	17u
//#define FrontLeft_ENC_B_EIRQ	22u
///************	BACK RIGHT MOTOR	************/
//#define BackRight_ENC_A_EIRQ	23u
//#define BackRight_ENC_B_EIRQ	15u
///************	BACK LEFT MOTOR	************/
//#define BackLeft_ENC_A_EIRQ		1u
//#define BackLeft_ENC_B_EIRQ		4u
/**************************************************************/
/**************		BUZZER Config		**************/
// PORT CONFIG
#define BUZZER_PORT	PTF
// PIN CONFIG
#define BUZZER_PIN	11
//
//#define welcomeMsg "Established Connection Successfully!\r\n\
//Now you can begin typing:\r\n"
///* Error message displayed at the console, in case data is received erroneously */
//#define errorMsg "An error occurred! The application will stop!\r\n"
/* Motor speed */
uint32_t prev_speed = 30;
///* Timeout in ms for blocking operations */
//#define TIMEOUT         200UL
//#define BUFFER_SIZE     256UL
///* Buffer used to receive data from the console */
//uint8_t buffer[BUFFER_SIZE];
//uint8_t bufferIdx;
//#define TX_MAILBOX  (1UL)
//#define TX_MSG_ID   (1UL)
#define RX_MAILBOX  (0UL)
// Function prototypes
void set_motor_direction(uint8_t motor, uint8_t direction);
void set_motor_speed(uint8_t motor, uint32_t speed);
void process_command(char command);
//void EncoderINT(void);
void CAN_INT(void);
// Interrupt handler for shared EIRQ vector
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
void process_command(char command) {
	switch (command) {
	case 'f': // Move Forward
		motor_velocities[0] = prev_speed;
		motor_velocities[1] = prev_speed;
		motor_velocities[2] = prev_speed;
		motor_velocities[3] = prev_speed;
		break;
	case 'b': // Move Backward
		motor_velocities[0] = prev_speed * -1;
		motor_velocities[1] = prev_speed * -1;
		motor_velocities[2] = prev_speed * -1;
		motor_velocities[3] = prev_speed * -1;
		break;
	case 'l': // Move Left
		motor_velocities[0] = prev_speed * -1;
		motor_velocities[1] = prev_speed;
		motor_velocities[2] = prev_speed * -1;
		motor_velocities[3] = prev_speed;
		break;
	case 'r': // Move Right
		motor_velocities[0] = prev_speed;
		motor_velocities[1] = prev_speed * -1;
		motor_velocities[2] = prev_speed;
		motor_velocities[3] = prev_speed * -1;
		break;
	case 'w': // Move Forward Right
		motor_velocities[0] = prev_speed * 0.707;
		motor_velocities[1] = prev_speed;
		motor_velocities[2] = prev_speed * 0.707;
		motor_velocities[3] = prev_speed;
		break;
	case 'x': // Forward Left
		motor_velocities[0] = prev_speed * 0.707;
		motor_velocities[1] = prev_speed;
		motor_velocities[2] = prev_speed * 0.707;
		motor_velocities[3] = prev_speed;
		break;
	case 'y': // Backward Right
		motor_velocities[0] = prev_speed * -0.707;
		motor_velocities[1] = prev_speed * -1;
		motor_velocities[2] = prev_speed * -0.707;
		motor_velocities[3] = prev_speed * -1;
		break;
	case 'z': // Backward Left
		motor_velocities[0] = prev_speed * -1;
		motor_velocities[1] = prev_speed * -0.707;
		motor_velocities[2] = prev_speed * -1;
		motor_velocities[3] = prev_speed * -0.707;
	case 'i': // increment speed
		prev_speed = prev_speed > 39 ? 44 : prev_speed + 5;
		motor_velocities[0] =motor_velocities[0] > 0 ? prev_speed : prev_speed * -1;
		motor_velocities[1] =motor_velocities[1] > 0 ? prev_speed : prev_speed * -1;
		motor_velocities[2] =motor_velocities[2] > 0 ? prev_speed : prev_speed * -1;
		motor_velocities[3] =motor_velocities[3] > 0 ? prev_speed : prev_speed * -1;
	case 'd': // decrement speed
		prev_speed = prev_speed < 5 ? 0 : prev_speed - 5;
		motor_velocities[0] =motor_velocities[0] > 0 ? prev_speed : prev_speed * -1;
		motor_velocities[1] =motor_velocities[1] > 0 ? prev_speed : prev_speed * -1;
		motor_velocities[2] =motor_velocities[2] > 0 ? prev_speed : prev_speed * -1;
		motor_velocities[3] =motor_velocities[3] > 0 ? prev_speed : prev_speed * -1;
		break;
	case 's': // stop Motors
		motor_velocities[0] = 0;
		motor_velocities[1] = 0;
		motor_velocities[2] = 0;
		motor_velocities[3] = 0;
		break;
	default:
		break;
	}
}
//void EncoderINT(void) {
//	uint16_t encoder_a_state, encoder_b_state;
//
//	// Process FrontRight encoder
//	if (PINS_DRV_GetPinExIntFlag(FrontRight_ENC_A_EIRQ)
//			|| PINS_DRV_GetPinExIntFlag(FrontRight_ENC_B_EIRQ)) {
//		encoder_a_state = ((uint16_t) (1 << FrontRight_ENC_A_PIN)
//				& PINS_DRV_ReadPins(FrontRight_ENC_A_PORT))
//				>> (FrontRight_ENC_A_PIN);
//		encoder_b_state = ((uint16_t) (1 << FrontRight_ENC_B_PIN)
//				& PINS_DRV_ReadPins(FrontRight_ENC_B_PORT))
//				>> (FrontRight_ENC_B_PIN);
//		encoder_counts[0] += (encoder_a_state == encoder_b_state) ? 1 : -1;
//		PINS_DRV_ClearPinExIntFlag(FrontRight_ENC_A_EIRQ);
//		PINS_DRV_ClearPinExIntFlag(FrontRight_ENC_B_EIRQ);
//	}
//
//	// Process FrontLeft encoder
//	if (PINS_DRV_GetPinExIntFlag(FrontLeft_ENC_A_EIRQ)
//			|| PINS_DRV_GetPinExIntFlag(FrontLeft_ENC_B_EIRQ)) {
//		encoder_a_state = ((uint16_t) (1 << FrontLeft_ENC_A_PIN)
//				& PINS_DRV_ReadPins(FrontLeft_ENC_A_PORT))
//				>> (FrontLeft_ENC_A_PIN);
//		encoder_b_state = ((uint16_t) (1 << FrontLeft_ENC_B_PIN)
//				& PINS_DRV_ReadPins(FrontLeft_ENC_B_PORT))
//				>> (FrontLeft_ENC_B_PIN);
//		encoder_counts[1] += (encoder_a_state == encoder_b_state) ? 1 : -1;
//		PINS_DRV_ClearPinExIntFlag(FrontLeft_ENC_A_EIRQ);
//		PINS_DRV_ClearPinExIntFlag(FrontLeft_ENC_B_EIRQ);
//	}
//
//	// Process RearRight encoder
//	if (PINS_DRV_GetPinExIntFlag(BackRight_ENC_A_EIRQ)
//			|| PINS_DRV_GetPinExIntFlag(BackRight_ENC_B_EIRQ)) {
//		encoder_a_state = ((uint16_t) (1 << BackRight_ENC_A_PIN)
//				& PINS_DRV_ReadPins(BackRight_ENC_A_PORT))
//				>> (BackRight_ENC_A_PIN);
//		encoder_b_state = ((uint16_t) (1 << BackRight_ENC_B_PIN)
//				& PINS_DRV_ReadPins(BackRight_ENC_B_PORT))
//				>> (BackRight_ENC_B_PIN);
//		encoder_counts[2] += (encoder_a_state == encoder_b_state) ? 1 : -1;
//		PINS_DRV_ClearPinExIntFlag(BackRight_ENC_A_EIRQ);
//		PINS_DRV_ClearPinExIntFlag(BackRight_ENC_B_EIRQ);
//	}
//
//	// Process RearLeft encoder
//	if (PINS_DRV_GetPinExIntFlag(BackLeft_ENC_A_EIRQ)
//			|| PINS_DRV_GetPinExIntFlag(BackLeft_ENC_B_EIRQ)) {
//		encoder_a_state = ((uint16_t) (1 << BackLeft_ENC_A_PIN)
//				& PINS_DRV_ReadPins(BackLeft_ENC_A_PORT))
//				>> (BackLeft_ENC_A_PIN);
//		encoder_b_state = ((uint16_t) (1 << BackLeft_ENC_B_PIN)
//				& PINS_DRV_ReadPins(BackLeft_ENC_B_PORT))
//				>> (BackLeft_ENC_B_PIN);
//		encoder_counts[3] += (encoder_a_state == encoder_b_state) ? 1 : -1;
//		PINS_DRV_ClearPinExIntFlag(BackLeft_ENC_A_EIRQ);
//		PINS_DRV_ClearPinExIntFlag(BackLeft_ENC_B_EIRQ);
//	}
//}
void CAN_INT(void) {
		/* Define receive buffer */
		can_message_t recvMsg;
		/* Start receiving data in RX_MAILBOX. */
		CAN_Receive(&can_pal1_instance, RX_MAILBOX, &recvMsg);
		/* Wait until the previous FlexCAN receive is completed */
		while(CAN_GetTransferStatus(&can_pal1_instance, RX_MAILBOX) == STATUS_BUSY);
		if(recvMsg.length > 0){
			float straight = (((uint16_t)recvMsg.data[0] << 8) | recvMsg.data[1]) / 10.0;
			float diagonal = (((uint16_t)recvMsg.data[2] << 8) | recvMsg.data[3]) / 10.0;
			sensor_readings[recvMsg.id-1].straight = straight ;
			sensor_readings[recvMsg.id-1].diagonal = diagonal ;
			sensor_readings[recvMsg.id-1].ir = recvMsg.id < 3? recvMsg.data[4]:0;
		}
}
void STM2_Ch0_IRQHandler(void) {
    /* Clear channel interrupt flag */
    STM_DRV_ClearStatusFlags(INST_STM1, stm1_ChnConfig0.channel);
	/*Timer routine every 10ms for PID */
	pid_outputs[0] = Kp			* (motor_velocities[0]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[0] - prev_encoder_counts[0]))
							/ ENCODER_CPR) / (0.01)));
	pid_outputs[1] = Kp
			* (motor_velocities[1]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[1] - prev_encoder_counts[1]))
							* ENCODER_CPR) / (0.01)));
	pid_outputs[2] = Kp
			* (motor_velocities[2]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[2] - prev_encoder_counts[2]))
							* ENCODER_CPR) / (0.01)));
	pid_outputs[3] = Kp
			* (motor_velocities[3]
					- ((WHEEL_CIRCUMFERENCE
							* (abs(encoder_counts[3] - prev_encoder_counts[3]))
							* ENCODER_CPR) / (0.01)));
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

//		set_motor_speed(Front_Right, 4000);
//		set_motor_speed(Front_Left, 4000);
//		set_motor_speed(Back_Right, 4000);
//		set_motor_speed(Back_Left, 4000);
}

/*! 
 \brief The main function for the project.
 \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
 */
int main(void) {

	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
#ifdef PEX_RTOS_INIT
	PEX_RTOS_INIT(); /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
#endif
	/*** End of Processor Expert internal initialization.                    ***/

	/* Write your code here */
	/* For example: for(;;) { } */

	/* Initialize and configure clocks
	 *  -   Setup system clocks, dividers
	 *  -   see clock manager component for more details
	 */
	CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
			g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
	CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

	/* Initialize selected pins and configure external interrupts
	 *  -   See PinSettings component for more info
	 */
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	PWM_Init(&pwm_pal1Instance, &pwm_pal1Configs);
	PWM_Init(&pwm_pal2Instance, &pwm_pal2Configs);
	UART_Init(&uart_pal1_instance, &uart_pal1_Config0);
	/* Send a welcome message */
	UART_SendDataBlocking(&uart_pal1_instance, (uint8_t *) welcomeMsg,
			strlen(welcomeMsg), TIMEOUT);

	char received_char;
	status_t status;
	uint32_t bytesRemaining;

//	/* Enable Global Interrupt Request*/
//	INT_SYS_EnableIRQGlobal();
//
//	/* Install Encoder ISR */
//	INT_SYS_InstallHandler(SIUL_EIRQ_00_07_IRQn, &EncoderINT, NULL);
//	INT_SYS_InstallHandler(SIUL_EIRQ_08_15_IRQn, &EncoderINT, NULL);
//	INT_SYS_InstallHandler(SIUL_EIRQ_16_23_IRQn, &EncoderINT, NULL);
//	//INT_SYS_SetPriority(SIUL_EIRQ_00_07_IRQn,0);
//	/* Enable Encoder interrupt */
//	INT_SYS_EnableIRQ(SIUL_EIRQ_00_07_IRQn);
//	INT_SYS_EnableIRQ(SIUL_EIRQ_08_15_IRQn);
//	INT_SYS_EnableIRQ(SIUL_EIRQ_16_23_IRQn);
	/* Initialize STM */
	STM_DRV_Init(INST_STM1, &stm1_InitConfig0);
	/* Compute the number of ticks from microseconds */
	STM_DRV_ComputeTicksByUs(INST_STM1, 100000U, &ticks);
	/* Initialize channel 0 */
	stm1_ChnConfig0.compareValue = ticks;
	STM_DRV_InitChannel(INST_STM1, &stm1_ChnConfig0);
//	INT_SYS_InstallHandler(STM2_Ch0_IRQn, &STM2_Ch0_IRQHandler, NULL);
	/* Enable Interrupt for STM2 channel 0 */
	INT_SYS_EnableIRQ(STM2_Ch0_IRQn);/* Start running the common timer counter */
	STM_DRV_StartTimer(INST_STM1);

	/* Initialize CAN module */
	CAN_Init(&can_pal1_instance, &can_pal1_Config0);
	/* Set information about the data to be sent
	 *  - Standard message ID
	 *  - Bit rate switch disabled
	 *  - Flexible data rate disabled
	 *  - Use zeros for FD padding
	 */
	CAN_InstallEventCallback(&can_pal1_instance, &CAN_INT, NULL);
	can_buff_config_t buffCfg = {
			.enableFD = false,
			.enableBRS = false,
			.fdPadding = 0U,
			.idType = CAN_MSG_ID_STD,
			.isRemote = false };
//	CAN_SetRxFilter(&can_pal1_instance, CAN_MSG_ID_STD, 0, 0);
	CAN_ConfigRxBuff(&can_pal1_instance, RX_MAILBOX, &buffCfg, 0);
	while (1) {
		/* Check for received UART data */
		/* Define receive buffer */
		can_message_t recvMsg;
		if (UART_ReceiveData(&uart_pal1_instance, (uint8_t*) &received_char, 1)
				== STATUS_SUCCESS) {
			while (UART_GetReceiveStatus(&uart_pal1_instance, &bytesRemaining)
					!= STATUS_SUCCESS)
				;
			status = UART_GetReceiveStatus(&uart_pal1_instance,
					&bytesRemaining);
			if (status != STATUS_SUCCESS) {
				/* If an error occurred, send the error message and exit the loop */
				UART_SendDataBlocking(&uart_pal1_instance, (uint8_t *) errorMsg,
						strlen(errorMsg), TIMEOUT);
				break;
			}
			/* Process end of line in Doc/Window(CRLF) or Unix(LF) */
			if (buffer[bufferIdx - 1] == '\r') {
				bufferIdx = bufferIdx - 1;
				/* Replace end of line character with null */
				buffer[bufferIdx] = 0U;
			} else {
				/* Replace end of line character with null */
				buffer[bufferIdx] = 0U;
			}
			/* Process the received command */
			process_command(received_char);
		}

	}
	/*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/
/* END main */
/*!
 ** @}
 */
/*
 ** ###################################################################
 **
 **     This file was created by Processor Expert 10.1 [05.21]
 **     for the NXP C55 series of microcontrollers.
 **
 ** ###################################################################
 */
