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
    pid->Prev_Direction_r = 0;
    pid->Prev_Direction_l = 0;

    pid->Anti_windup_EN = AntiWindup;
}


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

static float LRL_PID_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum)
        return maximum;

    if (value < minimum)
        return minimum;

    return value;
}


static int8_t LRL_PID_Sign(int16_t value)
{
    return (value > 0) - (value < 0);
}


static int8_t LRL_PID_UpdateWheel(
    float kp,
    float ki,
    float sample_time,
    int16_t reference_rpm,
    int16_t measured_rpm,
    float *integrator,
    float *previous_error,
    float *current_error,
    int8_t *previous_direction,
    uint8_t anti_windup_enabled)
{
    int8_t direction = LRL_PID_Sign(reference_rpm);

    /*
     * A zero reference means zero PWM. Clear the stored controller
     * state so that the next command cannot inherit old integral action.
     */
    if (direction == 0)
    {
        *integrator = 0.0f;
        *previous_error = 0.0f;
        *current_error = 0.0f;
        *previous_direction = 0;
        return 0;
    }

    /*
     * Reset the controller when the commanded wheel direction changes.
     * This prevents a positive-command integrator from kicking the motor
     * after receiving a negative command, or vice versa.
     */
    if (
        *previous_direction != 0 &&
        direction != *previous_direction)
    {
        *integrator = 0.0f;
        *previous_error = 0.0f;
    }

    /*
     * Preserve reference_rpm's sign, but run the controller on speed
     * magnitudes, matching the working Lenna-Mobile-Robot-ONE firmware.
     */
    float reference_magnitude =
        (float)abs((int)reference_rpm);

    float measurement_magnitude =
        (float)abs((int)measured_rpm);

    float error =
        (reference_magnitude - measurement_magnitude) *
        Speed2PWM_Rate;

    /*
     * Preserve the original firmware's effective integral behavior.
     * Do not add a 0.5 factor yet because doing so changes the effective
     * Ki and would require retuning.
     */
    float candidate_integrator =
        *integrator +
        sample_time * ki * (error + *previous_error);

    float candidate_output =
        kp * error + candidate_integrator;

    /*
     * Conditional integration:
     * - Integrate normally inside the permitted PWM range.
     * - When saturated, integrate only if the error moves the controller
     *   back toward the valid range.
     */
    uint8_t allow_integration =
        !anti_windup_enabled ||
        (
            candidate_output >= 0.0f &&
            candidate_output <= 100.0f
        ) ||
        (
            candidate_output > 100.0f &&
            error < 0.0f
        ) ||
        (
            candidate_output < 0.0f &&
            error > 0.0f
        );

    if (allow_integration)
    {
        *integrator = candidate_integrator;
    }

    float output_magnitude =
        kp * error + *integrator;

    output_magnitude = LRL_PID_ClampFloat(
        output_magnitude,
        0.0f,
        100.0f
    );

    *current_error = error;
    *previous_error = error;
    *previous_direction = direction;

    /*
     * Convert only after saturation. Adding 0.5 performs positive
     * rounding instead of truncation.
     */
    int8_t pwm_magnitude =
        (int8_t)(output_magnitude + 0.5f);

    return (int8_t)(direction * pwm_magnitude);
}

void LRL_PID_Update(
    pid_cfgType *pid,
    odom_cfgType *odom)
{
    /*
     * Encoder acquisition remains at the fixed 10 ms controller rate.
     */
    LRL_Odometry_ReadAngularSpeed(odom);
    odom->is_pid = 1;

    int16_t measured_right_rpm = odom->vel.right;
    int16_t measured_left_rpm = odom->vel.left;

    int16_t reference_right_rpm = pid->Ref_Vel_r;
    int16_t reference_left_rpm = pid->Ref_Vel_l;

    pid->Prev_Measurement_r = measured_right_rpm;
    pid->Prev_Measurement_l = measured_left_rpm;

    /*
     * The derivative term is intentionally disabled, matching the
     * existing controller.
     */
    pid->Differentiator_Amount_r = 0.0f;
    pid->Differentiator_Amount_l = 0.0f;

    pid->Control_Signal_r = LRL_PID_UpdateWheel(
        pid->Kp_r,
        pid->Ki_r,
        pid->Ts,
        reference_right_rpm,
        measured_right_rpm,
        &pid->Integrator_Amount_r,
        &pid->Prev_Error_r,
        &pid->Error_r,
        &pid->Prev_Direction_r,
        pid->Anti_windup_EN
    );

    pid->Control_Signal_l = LRL_PID_UpdateWheel(
        pid->Kp_l,
        pid->Ki_l,
        pid->Ts,
        reference_left_rpm,
        measured_left_rpm,
        &pid->Integrator_Amount_l,
        &pid->Prev_Error_l,
        &pid->Error_l,
        &pid->Prev_Direction_l,
        pid->Anti_windup_EN
    );
}

//void LRL_PID_Update(pid_cfgType *pid, odom_cfgType *odom)
//{
//	int16_t _vel_r, _vel_l;
//
//    int16_t _left_setpoint  = pid->Ref_Vel_l;
//    int16_t _right_setpoint = pid->Ref_Vel_r;
//
//	LRL_Odometry_ReadAngularSpeed(odom);
//	odom->is_pid = 1;
//
//	_vel_l = odom->vel.left;
//	_vel_r = odom->vel.right;
//
//	// Determine the direction of the control signal.
//	int8_t _dir_r = (_right_setpoint > 0) - (_right_setpoint < 0);
//	int8_t _dir_l = (_left_setpoint  > 0) - (_left_setpoint < 0);
//
//	// Use absolute values for error calculation to handle direction separately.
//
////	_vel_r 			= abs(_vel_r);
////	pid->Ref_Vel_r 	= abs(pid->Ref_Vel_r);
////
////	_vel_l 			= abs(_vel_l);
////	pid->Ref_Vel_r 	= abs(pid->Ref_Vel_r);
//
//	// Calculate the current error and scale it.
//	pid->Error_r = (_right_setpoint - _vel_r) * Speed2PWM_Rate;
//	pid->Error_l = (_left_setpoint - _vel_l) * Speed2PWM_Rate;
//
//	// Calculate the integral term using the trapezoidal rule.
//	pid->Integrator_Amount_r += (pid->Ts * (pid->Ki_r * (pid->Error_r + pid->Prev_Error_r)));
//	pid->Integrator_Amount_l += (pid->Ts * (pid->Ki_l * (pid->Error_l + pid->Prev_Error_l)));
//
//	// Derivative term is currently set to zero.
//	pid->Differentiator_Amount_r = 0;
//	pid->Differentiator_Amount_l = 0;
//
//	// Calculate the control signal as the sum of PID terms.
//	pid->Control_Signal_r = (pid->Kp_r * pid->Error_r) + pid->Integrator_Amount_r + pid->Differentiator_Amount_r;
//	pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Integrator_Amount_l + pid->Differentiator_Amount_l;
//
//	// Anti-windup implementation.
//	if(pid->Anti_windup_EN == 1)
//	{
//		// If the control signal is within limits, update the anti-windup amount.
//		if(pid->Control_Signal_r <= Upper_Saturation_Limit)
//		{
//			pid->Wind_Up_Amount_r = pid->Integrator_Amount_r;
//		}
//		else
//		{
//			// If saturated, re-calculate the control signal using the last non-saturated integral value.
//			pid->Control_Signal_r = (pid->Kp_r * pid->Error_r) + pid->Wind_Up_Amount_r + pid->Differentiator_Amount_r;
//		}
//
//		if(pid->Control_Signal_l <= Upper_Saturation_Limit)
//		{
//			pid->Wind_Up_Amount_l = pid->Integrator_Amount_l;
//		}
//		else
//		{
//			pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Wind_Up_Amount_l + pid->Differentiator_Amount_l;
//		}
//	}
//
////	if(pid->Anti_windup_EN == 1)
////	{
////		// If the control signal is within limits, update the anti-windup amount.
////		if(pid->Control_Signal_l <= Upper_Saturation_Limit)
////		{
////			pid->Wind_Up_Amount_l = pid->Integrator_Amount_l;
////		}
////		else
////		{
////			// If saturated, re-calculate the control signal using the last non-saturated integral value.
////			pid->Control_Signal_l = (pid->Kp_l * pid->Error_l) + pid->Wind_Up_Amount_l + pid->Differentiator_Amount_l;
////		}
////	}
//
//	// Apply output saturation limits.
//	if(pid->Control_Signal_r > pid->Upper_Limit_Saturation)
//	{
//		pid->Control_Signal_r = pid->Upper_Limit_Saturation;
//	}
//	else if(pid->Control_Signal_r < pid->Lower_Limit_Saturation)
//	{
//		pid->Control_Signal_r = pid->Lower_Limit_Saturation;
//	}
//
//	if(pid->Control_Signal_l > pid->Upper_Limit_Saturation)
//	{
//		pid->Control_Signal_l = pid->Upper_Limit_Saturation;
//	}
//	else if(pid->Control_Signal_l < pid->Lower_Limit_Saturation)
//	{
//		pid->Control_Signal_l = pid->Lower_Limit_Saturation;
//	}
//
//	// Apply the determined direction to the final control signal.
//	pid->Control_Signal_r = pid->Control_Signal_r * _dir_r;
//	pid->Control_Signal_l = pid->Control_Signal_l * _dir_l;
//
//	// Update previous values for the next iteration.
//	pid->Prev_Measurement_r = _vel_r;
//	pid->Prev_Error_r = pid->Error_r;
//
//	pid->Prev_Measurement_l = _vel_l;
//	pid->Prev_Error_l = pid->Error_l;
//}
