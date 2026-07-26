/**
 * @file pid.c
 * @brief This module provides a PID (Proportional-Integral-Derivative) controller implementation.
 *
 * It includes functions for initializing the controller, updating the control
 * signal based on sensor feedback, and handling anti-windup and saturation.
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch, Iran University of Science and Technology
 * @date October 11, 2025
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

#include <mcu_config.h>
#include "pid.h"
#include "odometry.h"
#include "main.h"
#include "stdlib.h"


/**
 * @brief Initializes the PID controller parameters.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param AntiWindup Enables or disables the anti-windup feature (1 for enabled, 0 for disabled).
 *
 * This function resets all PID state variables and sets the initial configuration,
 * including the anti-windup flag.
 */
void LRL_PID_Init(pid_cfgType *pid, uint8_t AntiWindup)
{
    pid->Kp_r = Proportional_Gain_RIGHT_MOTOR;
    pid->Ki_r = Integral_Gain_RIGHT_MOTOR;
    pid->Kd_r = Derivative_Gain_RIGHT_MOTOR;

    pid->Kp_l = Proportional_Gain_LEFT_MOTOR;
    pid->Ki_l = Integral_Gain_LEFT_MOTOR;
    pid->Kd_l = Derivative_Gain_LEFT_MOTOR;

    pid->Ts = Sampling_Time;

    pid->Lower_Limit_Saturation = Lower_Saturation_Limit;
    pid->Upper_Limit_Saturation = Upper_Saturation_Limit;

    pid->Integrator_Amount_r = 0.0f;
    pid->Integrator_Amount_l = 0.0f;
    pid->Differentiator_Amount_r = 0.0f;
    pid->Differentiator_Amount_l = 0.0f;

    pid->Prev_Measurement_r = 0;
    pid->Prev_Measurement_l = 0;
    pid->Prev_Error_r = 0.0f;
    pid->Prev_Error_l = 0.0f;

    pid->Error_r = 0.0f;
    pid->Error_l = 0.0f;
    pid->Wind_Up_Amount_r = 0.0f;
    pid->Wind_Up_Amount_l = 0.0f;

    pid->Control_Signal_r = 0;
    pid->Control_Signal_l = 0;

    pid->Ref_Vel_r = 0;
    pid->Ref_Vel_l = 0;
    pid->dir = 0;

    pid->Anti_windup_EN = AntiWindup;
}

//void LRL_PID_Init(pid_cfgType2 *pid_cfg,uint8_t AntiWindup)
//{
//	// Reset PID state variables.
//	pid_cfg->Anti_windup_EN = AntiWindup;
//	pid_cfg->Prev_Measurement = 0.0f;
//	pid_cfg->Integrator_Amount = 0;
//	pid_cfg->Prev_Error = 0.0f;
//	pid_cfg->Control_Signal = 0;
//}

/**
 * @brief Updates the PID control signal.
 * @param pid_cfg Pointer to the PID configuration structure.
 * @param measurement The current measured value (e.g., motor speed).
 * @param set_point The desired set point.
 *
 * This function calculates the error and then computes the proportional, integral,
 * and derivative terms to generate a new control signal. It also handles
 * direction, anti-windup, and output saturation.
 */

//void LRL_PID_Update(pid_cfgType2 *pid_cfg, int16_t measurement, int16_t set_point)
//{
//	int8_t _dir;
//
//	// Determine the direction of the control signal.
//	_dir = set_point / abs(set_point);
//
//	// Use absolute values for error calculation to handle direction separately.
//	measurement = abs(measurement);
//	set_point = abs(set_point);
//
//	// Calculate the current error and scale it.
//	pid_cfg->Error = set_point - measurement;
//	pid_cfg->Error = pid_cfg->Error * Speed2PWM_Rate;
//
//	// Calculate the integral term using the trapezoidal rule.
//	pid_cfg->Integrator_Amount += (pid_cfg->Ts * (pid_cfg->Ki * (pid_cfg->Error + pid_cfg->Prev_Error)));
//
//	// Derivative term is currently set to zero.
//	pid_cfg->Differentiator_Amount = 0;
//
//	// Calculate the control signal as the sum of PID terms.
//	pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Integrator_Amount + pid_cfg->Differentiator_Amount;
//
//	// Anti-windup implementation.
//	if(pid_cfg->Anti_windup_EN == 1)
//	{
//		// If the control signal is within limits, update the anti-windup amount.
//		if(pid_cfg->Control_Signal <= Upper_Saturation_Limit)
//		{
//			pid_cfg->Wind_Up_Amount = pid_cfg->Integrator_Amount;
//		}
//		else
//		{
//			// If saturated, re-calculate the control signal using the last non-saturated integral value.
//			pid_cfg->Control_Signal = (pid_cfg->Kp * pid_cfg->Error) + pid_cfg->Wind_Up_Amount + pid_cfg->Differentiator_Amount;
//		}
//	}
//
//	// Apply output saturation limits.
//	if(pid_cfg->Control_Signal > pid_cfg->Upper_Limit_Saturation)
//	{
//		pid_cfg->Control_Signal = pid_cfg->Upper_Limit_Saturation;
//	}
//	else if(pid_cfg->Control_Signal < pid_cfg->Lower_Limit_Saturation)
//	{
//		pid_cfg->Control_Signal = pid_cfg->Lower_Limit_Saturation;
//	}
//
//	// Apply the determined direction to the final control signal.
//	pid_cfg->Control_Signal = pid_cfg->Control_Signal * _dir;
//
//	// Update previous values for the next iteration.
//	pid_cfg->Prev_Measurement = measurement;
//	pid_cfg->Prev_Error = pid_cfg->Error;
//}
void LRL_PID_Update(pid_cfgType *pid, odom_cfgType *odom)
{

	int16_t _vel_r, _vel_l;

    int16_t _left_setpoint  = pid->Ref_Vel_l;
    int16_t _right_setpoint = pid->Ref_Vel_r;

	LRL_Odometry_ReadAngularSpeed(odom);

	_vel_l = odom->vel.left;
	_vel_r = odom->vel.right;

	// Determine the direction of the control signal.
	int8_t _dir_r = (_right_setpoint > 0) - (_right_setpoint < 0);
	int8_t _dir_l = (_left_setpoint  > 0) - (_left_setpoint < 0);

	// Use absolute values for error calculation to handle direction separately.

//	_vel_r 			= abs(_vel_r);
//	pid->Ref_Vel_r 	= abs(pid->Ref_Vel_r);
//
//	_vel_l 			= abs(_vel_l);
//	pid->Ref_Vel_r 	= abs(pid->Ref_Vel_r);

	// Calculate the current error and scale it.
	pid->Error_r = (_right_setpoint - _vel_r) * Speed2PWM_Rate;
	pid->Error_l = (_left_setpoint - _vel_l) * Speed2PWM_Rate;

	// Calculate the integral term using the trapezoidal rule.
	pid->Integrator_Amount_r += (pid->Ts * (pid->Ki_r * (pid->Error_r + pid->Prev_Error_r)));
	pid->Integrator_Amount_l += (pid->Ts * (pid->Ki_l * (pid->Error_l + pid->Prev_Error_l)));

	// Derivative term is currently set to zero.
	pid->Differentiator_Amount_r = 0;
	pid->Differentiator_Amount_l = 0;

	// Calculate the control signal as the sum of PID terms.
	pid->Control_Signal_r = (pid->Kp_r * pid->Error_r) + pid->Integrator_Amount_r + pid->Differentiator_Amount_r;
	pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Integrator_Amount_l + pid->Differentiator_Amount_l;

	// Anti-windup implementation.
	if(pid->Anti_windup_EN == 1)
	{
		// If the control signal is within limits, update the anti-windup amount.
		if(pid->Control_Signal_r <= Upper_Saturation_Limit)
		{
			pid->Wind_Up_Amount_r = pid->Integrator_Amount_r;
		}
		else
		{
			// If saturated, re-calculate the control signal using the last non-saturated integral value.
			pid->Control_Signal_r = (pid->Kp_r * pid->Error_r) + pid->Wind_Up_Amount_r + pid->Differentiator_Amount_r;
		}

		if(pid->Control_Signal_l <= Upper_Saturation_Limit)
		{
			pid->Wind_Up_Amount_l = pid->Integrator_Amount_l;
		}
		else
		{
			pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Wind_Up_Amount_l + pid->Differentiator_Amount_l;
		}
	}

//	if(pid->Anti_windup_EN == 1)
//	{
//		// If the control signal is within limits, update the anti-windup amount.
//		if(pid->Control_Signal_l <= Upper_Saturation_Limit)
//		{
//			pid->Wind_Up_Amount_l = pid->Integrator_Amount_l;
//		}
//		else
//		{
//			// If saturated, re-calculate the control signal using the last non-saturated integral value.
//			pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Wind_Up_Amount_l + pid->Differentiator_Amount_l;
//		}
//	}

	// Apply output saturation limits.
	if(pid->Control_Signal_r > pid->Upper_Limit_Saturation)
	{
		pid->Control_Signal_r = pid->Upper_Limit_Saturation;
	}
	else if(pid->Control_Signal_r < pid->Lower_Limit_Saturation)
	{
		pid->Control_Signal_r = pid->Lower_Limit_Saturation;
	}

	if(pid->Control_Signal_l > pid->Upper_Limit_Saturation)
	{
		pid->Control_Signal_l = pid->Upper_Limit_Saturation;
	}
	else if(pid->Control_Signal_l < pid->Lower_Limit_Saturation)
	{
		pid->Control_Signal_l = pid->Lower_Limit_Saturation;
	}

	// Apply the determined direction to the final control signal.
	pid->Control_Signal_r = pid->Control_Signal_r * _dir_r;
	pid->Control_Signal_l = pid->Control_Signal_l * _dir_l;

	// Update previous values for the next iteration.
	pid->Prev_Measurement_r = _vel_r;
	pid->Prev_Error_r = pid->Error_r;

	pid->Prev_Measurement_l = _vel_l;
	pid->Prev_Error_l = pid->Error_l;
}
