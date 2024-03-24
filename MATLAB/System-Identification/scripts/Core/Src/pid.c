#include "pid.h"
#include "main.h"
#include "mcu_layout.h"


void LRL_PID_Init(pid_cfgType *pid_cfg,uint8_t AntiWindup)
	{
	// Resetting the PID parameters
	pid_cfg->Anti_windup_EN = AntiWindup;
	pid_cfg->Prev_Measurement = 0.0f;
	pid_cfg->Integrator_Amount = 0.0f;
	pid_cfg->Prev_Error = 0.0f;// initial error you can change it if by default you have an error
	pid_cfg->Control_Signal = 0;
	}

void LRL_PID_Update(pid_cfgType *pid_cfg,float measurement,float set_point)
	{
	float error = set_point - measurement;
	// Setting Values
	float P = pid_cfg->Kp * error;
	pid_cfg->Integrator_Amount = pid_cfg->Integrator_Amount + (pid_cfg->Ts*(pid_cfg->Ki * (error + pid_cfg->Prev_Error)));
	if(pid_cfg->Anti_windup_EN == 1)
		{
		if(pid_cfg->Integrator_Amount <= pid_cfg->Upper_Limit_Saturation)
			{
			pid_cfg->Integrator_Amount = pid_cfg->Integrator_Amount + (pid_cfg->Ts*(pid_cfg->Ki * (error + pid_cfg->Prev_Error)));
			HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
			}
		else
			{
			pid_cfg->Integrator_Amount = 0;
			HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 0);
			}
		}
	else
		{
		pid_cfg->Integrator_Amount = pid_cfg->Integrator_Amount + (pid_cfg->Ts*(pid_cfg->Ki * (error + pid_cfg->Prev_Error)));
		//HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
		}

	float I = pid_cfg->Integrator_Amount;

	float D = (pid_cfg->Kd * (measurement - pid_cfg->Prev_Measurement))/(pid_cfg->Ts);
    /*
    // another way to use derivative term used by phils lab channel
   pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);
     */

	pid_cfg->Control_Signal = P + I + D;

	if(pid_cfg->Control_Signal > pid_cfg->Upper_Limit_Saturation)
	  {
		pid_cfg->Control_Signal = pid_cfg->Upper_Limit_Saturation;
	  }
	else if(pid_cfg->Control_Signal < pid_cfg->Lower_Limit_Saturation)
	  {
		pid_cfg->Control_Signal = pid_cfg->Lower_Limit_Saturation;
	  }


	pid_cfg->Prev_Measurement = measurement;
	pid_cfg->Prev_Error = error;


	}


/*
 * this is another approach for pid control

float LRL_PID_Calculation(float input,float set_point)
	{
	error=(set_point-input);
	float out;
	if(Anti_windup_EN)
		{

		if(awu_error<abs(error))
			{
			out=Kp*(error)+Kd*(input-prev_input)/Ts;
			}
		else
			{
			out=(Kp*(error)) +( Ki*(Ki_sum)*Ts) -( Kd*(input-prev_input)/Ts);
			}

		}

	else
		{
		out=Kp*(error) + Ki*(Ki_sum)*Ts - Kd*(input-prev_input)/Ts;
		}
	Ki_sum=Ki_sum+(Ki_sum);
	if(out>Upper_Limit_Saturation)
	  {
		out=Upper_Limit_Saturation;
	  }
	if(out<Lower_Limit_Saturation)
	  {
		out=Lower_Limit_Saturation;
	  }
	prev_input=input;
	return out;
	}

*/