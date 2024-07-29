/*
 * Bluetooth.h
 *
 *  Created on: June 9, 2024
 *      Author: Ahmed Falah
 */

#ifndef HAL_BLUETOOTH_BLUETOOTH_H_
#define HAL_BLUETOOTH_BLUETOOTH_H_

#include "clockMan1.h"
#include "pin_mux.h"
#include "uart_pal.h"
#include <string.h>


#define welcomeMsg "Established Connection Successfully!\r\n\
Now you can begin typing:\r\n"
/* Error message displayed at the console, in case data is received erroneously */
#define errorMsg "An error occurred! The application will stop!\r\n"


/* Timeout in ms for blocking operations */
#define TIMEOUT         200UL
#define BUFFER_SIZE     256UL
/* Buffer used to receive data from the console */
uint8_t buffer[BUFFER_SIZE];
uint8_t bufferIdx;

status_t status;
uint32_t bytesRemaining;

void BluetoothInit(void);
void BluetoothReceiveData(void);


#endif /* HAL_BLUETOOTH_BLUETOOTH_H_ */
