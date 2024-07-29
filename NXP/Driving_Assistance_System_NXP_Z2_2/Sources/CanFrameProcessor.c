/*
 * CanFrameProcessor.c
 *
 *  Created on: Jul 10, 2024
 *      Author: ahmed
 */

#include "CanFrameProcessor.h"

void CanInit(void){
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
	CAN_ConfigRxBuff(&can_pal1_instance, RX_STM_FR_MAILBOX, &buffCfg, RX_STM_FR_ID);
	CAN_ConfigRxBuff(&can_pal1_instance, RX_STM_FL_MAILBOX, &buffCfg, RX_STM_FL_ID);
	CAN_ConfigRxBuff(&can_pal1_instance, RX_STM_BR_MAILBOX, &buffCfg, RX_STM_BR_ID);
	CAN_ConfigRxBuff(&can_pal1_instance, RX_STM_BL_MAILBOX, &buffCfg, RX_STM_BL_ID);
}

void CAN_INT(uint8_t instance,
        flexcan_event_type_t eventType,
        uint32_t buffIdx,
        struct FlexCANState *driverState) {

		/* Define receive buffer */
		can_message_t recvMsg;
		/* Start receiving data in RX_MAILBOX. */
		CAN_Receive(&can_pal1_instance, buffIdx, &recvMsg);
		/* Wait until the previous FlexCAN receive is completed */
		while(CAN_GetTransferStatus(&can_pal1_instance, buffIdx) == STATUS_BUSY);
		if(recvMsg.length > 0){
			float straight = (((uint16_t)recvMsg.data[0] << 8) | recvMsg.data[1]) / 10.0;
			float diagonal = (((uint16_t)recvMsg.data[2] << 8) | recvMsg.data[3]) / 10.0;
			sensor_readings[recvMsg.id-1].straight = straight ;
			sensor_readings[recvMsg.id-1].diagonal = diagonal ;
			sensor_readings[recvMsg.id-1].ir = recvMsg.id < 3? recvMsg.data[4]:0;
		}
}
