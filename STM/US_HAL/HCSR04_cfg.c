/*
 * File: HCSR04_cfg.c
 * Driver Name: [[ HC-SR04 Ultrasonic Sensor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "../US_HAL/HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		GPIOB,
		GPIO_PIN_11,
		TIM2,
		TIM_CHANNEL_3,
		72
	},
	// HC-SR04 Sensor Unit 2 Configurations
	{
		GPIOB,
		GPIO_PIN_5,
		TIM3,
		TIM_CHANNEL_1,
		72
	}
};
