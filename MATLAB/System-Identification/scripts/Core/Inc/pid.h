/*
 * pid.h
 *
 *  Created on: Mar 24, 2024
 *      Author: arian
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx_hal.h"

typedef struct
	{
	float Kp;
	float Ki;
	float Kd;
	float Ts;
	float Lower_Limit_Saturation;
	float Upper_Limit_Saturation;
	float Integrator_Amount;
	float Prev_Measurement;
	float Prev_Error;
	int8_t Control_Signal;
	uint8_t Anti_windup_EN;

	}pid_cfgType;

void LRL_PID_Init(pid_cfgType *pid_cfg,uint8_t AntiWindup);
void LRL_PID_Update(pid_cfgType *pid_cfg,float measurement,float set_point);

#endif /* INC_PID_H_ */
