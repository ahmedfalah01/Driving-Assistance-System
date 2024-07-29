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
