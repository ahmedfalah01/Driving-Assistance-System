/*
 * CanFrameProcessor.h
 *
 *  Created on: Jul 10, 2024
 *      Author: ahmed
 */

#ifndef CANFRAMEPROCESSOR_H_
#define CANFRAMEPROCESSOR_H_


struct STM_Readings{
	float straight;
	float diagonal;
	float ir;
};
struct STM_Readings sensor_readings[4];

#define RX_STM_FR_MAILBOX  (0UL)
#define RX_STM_FL_MAILBOX  (1UL)
#define RX_STM_BR_MAILBOX  (2UL)
#define RX_STM_BL_MAILBOX  (3UL)

#define RX_STM_FR_ID  (1UL)
#define RX_STM_FL_ID  (2UL)
#define RX_STM_BR_ID  (3UL)
#define RX_STM_BL_ID  (4UL)


void CanInit(void);


#endif /* CANFRAMEPROCESSOR_H_ */
