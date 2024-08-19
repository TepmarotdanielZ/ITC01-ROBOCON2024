/*
 * File: JOYSTICK_cfg.c
 * Driver Name: [[ JoyStick ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */
#include "JOYSTICK.h"

const JoyStick_CfgType JoyStick_CfgParam[JOYSTICK_UNITS] =
{
	// JoyStick unit 1 Configurations
    {
	    GPIOA,
		GPIOA,
		GPIO_PIN_0,
		GPIO_PIN_1,
		ADC2,
		ADC_CHANNEL_0,
		ADC_CHANNEL_1
	},
	{
		GPIOA,
		GPIOA,
		GPIO_PIN_2,
		GPIO_PIN_3,
		ADC1,
		ADC_CHANNEL_2,
		ADC_CHANNEL_3
	}
};

