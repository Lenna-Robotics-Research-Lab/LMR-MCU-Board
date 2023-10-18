/*
 * ultrasonic.c
 *
 *  Created on: Oct 18, 2023
 *      Author: Lenna Robotics
 */

#include "tim.h"
#include "mcu_layout.h"
#include "ultrasonic.h"

uint8_t Capture_Flag = 0;

void LMR_Delayus (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (__HAL_TIM_GET_COUNTER (&htim6) < time);
}

void LMR_US1_Init(void)
{
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
}

void LMR_US1_Trig(void)
{
	HAL_GPIO_WritePin(US1_TRIG_PORT, US1_TRIG_PIN, GPIO_PIN_SET);
	LMR_Delayus(10);  // wait for 10 us
	HAL_GPIO_WritePin(US1_TRIG_PORT, US1_TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC3);
}

uint8_t LMR_US1_Read(TIM_HandleTypeDef *htim){
	uint32_t IC_Val1 = 0;
	uint32_t IC_Val2 = 0;
	uint32_t Difference = 0;
	uint8_t Distance  = 0;


	if(Capture_Flag == 0)
	{
		IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		Capture_Flag = 1;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
	}
	else if (Capture_Flag ==1)
	{
		IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COUNTER(htim,0);
		if(IC_Val2 > IC_Val1)
		{
			Difference = IC_Val2 - IC_Val1;
		}
		else
		{
			Difference = (0xffff - IC_Val1) + IC_Val2;
		}
		Distance = Difference * .034/2;
		Capture_Flag = 0;

		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC3);
	}
	return Distance;
}
