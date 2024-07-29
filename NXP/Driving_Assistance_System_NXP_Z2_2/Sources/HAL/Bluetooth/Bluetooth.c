/*
 * Bluetooth.c
 *
 *  Created on: June 9, 2024
 *      Author: Ahmed Falah
 */

#include "Bluetooth.h"

void BluetoothInit(void){
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	UART_Init(&uart_pal1_instance, &uart_pal1_Config0);
	/* Send a welcome message */
	UART_SendDataBlocking(&uart_pal1_instance, (uint8_t *) welcomeMsg,
			strlen(welcomeMsg), TIMEOUT);
}

char BluetoothReceiveData(void){
	/* Check for received UART data */
	char received_char;
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
		return received_char;
	}
}
