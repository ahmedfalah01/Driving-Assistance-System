/*
 * EXTI_Encoder.h
 *
 *  Created on: May 21, 2024
 *      Author: Ahmed Falah
 */

#ifndef EXTI_ENCODER_H_
#define EXTI_ENCODER_H_

#include "pin_mux.h"
#include "interrupt_manager.h"


/**************************************************************/
/**************		Encoders Configurations		**************/
/************	FRONT RIGHT MOTOR	************/
// PORT CONFIG
#define FrontRight_ENC_A_PORT	PTE
#define FrontRight_ENC_B_PORT	PTE
// PIN CONFIG
//PE4 PE10
#define FrontRight_ENC_A_PIN	4
#define FrontRight_ENC_B_PIN	10
/************	FRONT LEFT MOTOR	************/
// PORT CONFIG
#define FrontLeft_ENC_A_PORT	PTA
#define FrontLeft_ENC_B_PORT	PTE
// PIN CONFIG
//PTA 16u PTE 22u
#define FrontLeft_ENC_A_PIN		12u
#define FrontLeft_ENC_B_PIN		6u
/************	BACK RIGHT MOTOR	************/
// PORT CONFIG
#define BackRight_ENC_A_PORT	PTE
#define BackRight_ENC_B_PORT	PTG
// PIN CONFIG
//PTE 23u PTG 15u
#define BackRight_ENC_A_PIN		7u
#define BackRight_ENC_B_PIN		8u
/************	BACK LEFT MOTOR	************/
// PORT CONFIG
#define BackLeft_ENC_A_PORT		PTA
#define BackLeft_ENC_B_PORT		PTA
// PIN CONFIG
//PTA 17u PTA 4u
#define BackLeft_ENC_A_PIN		6u
#define BackLeft_ENC_B_PIN		14u
/**************************************************************/
/**************		EIRQ Configurations	**************/
/************	FRONT RIGHT MOTOR	************/
#define FrontRight_ENC_A_EIRQ	9u
#define FrontRight_ENC_B_EIRQ	10u
/************	FRONT LEFT MOTOR	************/
#define FrontLeft_ENC_A_EIRQ	17u
#define FrontLeft_ENC_B_EIRQ	22u
/************	BACK RIGHT MOTOR	************/
#define BackRight_ENC_A_EIRQ	23u
#define BackRight_ENC_B_EIRQ	15u
/************	BACK LEFT MOTOR	************/
#define BackLeft_ENC_A_EIRQ		1u
#define BackLeft_ENC_B_EIRQ		4u


// Encoder counts for each motor
//FR FL BR BL
volatile int32_t *encoder_counts = (int32_t *) 0x40080000;
//volatile int32_t encoder_counts[4] = { 0 };


/*	Function Prototype	*/
void EncoderInit(void);
void EncoderINT(void);


#endif /* EXTI_ENCODER_H_ */
