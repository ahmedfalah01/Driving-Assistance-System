/*
 * EXTI_Encoder.c
 *
 *  Created on: June 9, 2024
 *      Author: Ahmed Falah
 */
#include "EXTI_Encoder.h"

void EncoderInit(void){

	/* Initialize selected pins and configure external interrupts
	 *  -   See PinSettings component for more info
	 */
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

	/* Enable Global Interrupt Request*/
	INT_SYS_EnableIRQGlobal();

	/* Install Encoder ISR */
	INT_SYS_InstallHandler(SIUL_EIRQ_00_07_IRQn, &EncoderINT, NULL);
	INT_SYS_InstallHandler(SIUL_EIRQ_08_15_IRQn, &EncoderINT, NULL);
	INT_SYS_InstallHandler(SIUL_EIRQ_16_23_IRQn, &EncoderINT, NULL);
	//INT_SYS_SetPriority(SIUL_EIRQ_00_07_IRQn,0);
	/* Enable Encoder interrupt */
	INT_SYS_EnableIRQ(SIUL_EIRQ_00_07_IRQn);
	INT_SYS_EnableIRQ(SIUL_EIRQ_08_15_IRQn);
	INT_SYS_EnableIRQ(SIUL_EIRQ_16_23_IRQn);
}



void EncoderINT(void) {
	uint16_t encoder_a_state, encoder_b_state;

	// Process FrontRight encoder
	if (PINS_DRV_GetPinExIntFlag(FrontRight_ENC_A_EIRQ)
			|| PINS_DRV_GetPinExIntFlag(FrontRight_ENC_B_EIRQ)) {
		encoder_a_state = ((uint16_t) (1 << FrontRight_ENC_A_PIN)
				& PINS_DRV_ReadPins(FrontRight_ENC_A_PORT))
				>> (FrontRight_ENC_A_PIN);
		encoder_b_state = ((uint16_t) (1 << FrontRight_ENC_B_PIN)
				& PINS_DRV_ReadPins(FrontRight_ENC_B_PORT))
				>> (FrontRight_ENC_B_PIN);
		encoder_counts[0] += (encoder_a_state == encoder_b_state) ? 1 : -1;
		PINS_DRV_ClearPinExIntFlag(FrontRight_ENC_A_EIRQ);
		PINS_DRV_ClearPinExIntFlag(FrontRight_ENC_B_EIRQ);
	}

	// Process FrontLeft encoder
	if (PINS_DRV_GetPinExIntFlag(FrontLeft_ENC_A_EIRQ)
			|| PINS_DRV_GetPinExIntFlag(FrontLeft_ENC_B_EIRQ)) {
		encoder_a_state = ((uint16_t) (1 << FrontLeft_ENC_A_PIN)
				& PINS_DRV_ReadPins(FrontLeft_ENC_A_PORT))
				>> (FrontLeft_ENC_A_PIN);
		encoder_b_state = ((uint16_t) (1 << FrontLeft_ENC_B_PIN)
				& PINS_DRV_ReadPins(FrontLeft_ENC_B_PORT))
				>> (FrontLeft_ENC_B_PIN);
		encoder_counts[1] += (encoder_a_state == encoder_b_state) ? 1 : -1;
		PINS_DRV_ClearPinExIntFlag(FrontLeft_ENC_A_EIRQ);
		PINS_DRV_ClearPinExIntFlag(FrontLeft_ENC_B_EIRQ);
	}

	// Process RearRight encoder
	if (PINS_DRV_GetPinExIntFlag(BackRight_ENC_A_EIRQ)
			|| PINS_DRV_GetPinExIntFlag(BackRight_ENC_B_EIRQ)) {
		encoder_a_state = ((uint16_t) (1 << BackRight_ENC_A_PIN)
				& PINS_DRV_ReadPins(BackRight_ENC_A_PORT))
				>> (BackRight_ENC_A_PIN);
		encoder_b_state = ((uint16_t) (1 << BackRight_ENC_B_PIN)
				& PINS_DRV_ReadPins(BackRight_ENC_B_PORT))
				>> (BackRight_ENC_B_PIN);
		encoder_counts[2] += (encoder_a_state == encoder_b_state) ? 1 : -1;
		PINS_DRV_ClearPinExIntFlag(BackRight_ENC_A_EIRQ);
		PINS_DRV_ClearPinExIntFlag(BackRight_ENC_B_EIRQ);
	}

	// Process RearLeft encoder
	if (PINS_DRV_GetPinExIntFlag(BackLeft_ENC_A_EIRQ)
			|| PINS_DRV_GetPinExIntFlag(BackLeft_ENC_B_EIRQ)) {
		encoder_a_state = ((uint16_t) (1 << BackLeft_ENC_A_PIN)
				& PINS_DRV_ReadPins(BackLeft_ENC_A_PORT))
				>> (BackLeft_ENC_A_PIN);
		encoder_b_state = ((uint16_t) (1 << BackLeft_ENC_B_PIN)
				& PINS_DRV_ReadPins(BackLeft_ENC_B_PORT))
				>> (BackLeft_ENC_B_PIN);
		encoder_counts[3] += (encoder_a_state == encoder_b_state) ? 1 : -1;
		PINS_DRV_ClearPinExIntFlag(BackLeft_ENC_A_EIRQ);
		PINS_DRV_ClearPinExIntFlag(BackLeft_ENC_B_EIRQ);
	}
}
