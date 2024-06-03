/*
 * odometry.h
 *
 *  Created on: Jun 1, 2024
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "i2c.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

// #################################################################
// ####################  MAGNETOMETER HMC5883L  ####################
// #################################################################
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_ADDRESS            HMC5883L_DEFAULT_ADDRESS << 1
#define HMC5883L_ADDRESS_WRITE      0x3C
#define HMC5883L_ADDRESS_READ       0x3D

// Register Address List
#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define MAGNETIC_DECLINATION		0.0881

// #######################################################
// ####################  IMU MPU6050  ####################
// #######################################################
#define WHO_AM_I 		0x75
#define INT_PIN_CFG 	0x37
#define USER_CTRL 		0x6A
#define PWR_MGMT_1 		0x6B
#define SMPLRT_DIV 		0x19
#define ACCEL_CONFIG 	0x1C
#define ACCEL_XOUT_H 	0x3B
#define TEMP_OUT_H 		0x41
#define GYRO_CONFIG 	0x1B
#define GYRO_XOUT_H 	0x43

#define MPU_ADDR 		0xD0

#define ACCEL_X_CORRECTOR 16384
#define ACCEL_Y_CORRECTOR 16384
#define ACCEL_Z_CORRECTOR 14418

#define GYRO_CORRECTOR 	131.0

// Complementary filter constants
#define ALPHA 			0.9f

#define DELAY_TIMEOUT	10
#define FLOAT_SCALING	1000

// ############################################################
// ####################  ODOMETRY STRUCTS  ####################
// ############################################################
typedef struct
{
	float	x;
	float	y;
	float	z;
} linear_position;

typedef struct
{
	float	x;
	float	y;
	float	z;
} angular_position;

typedef struct
{
	int16_t	x;
	int16_t	y;
	int16_t	z;
} accelerometer;

typedef struct
{
	int16_t	x;
	int16_t	y;
	int16_t	z;
} gyroscope;

typedef struct
{
	float	x;
	float	y;
	float	z;
	uint16_t	heading;
} magnetometer;

typedef struct
{
	uint16_t	right;
	uint16_t	left;
} motor_velocity;

typedef struct
{
	TIM_HandleTypeDef * htim;

	uint16_t 			MAX_ARR;
	float 				TICK2RPM;

	uint16_t tick;
	uint16_t tick_prev;
} encoder_cfgType;

typedef struct
{
	I2C_HandleTypeDef * hi2c;	// I2C Sensors e.g.  IMU, Magnetometer, etc.
	float 	offset_calibration_x;
	float 	offset_calibration_y;
	float 	offset_calibration_z;
	float 	roll_temp;
	float 	pitch_temp;
	float	yaw_temp;
} imu_cfgType;

typedef struct
{
	imu_cfgType 		imu;
	encoder_cfgType		enc_right;
	encoder_cfgType		enc_left;

	linear_position 	pose;
	angular_position 	angle;
	accelerometer 		accel;
	gyroscope 			gyro;
	magnetometer 		mag;
	motor_velocity 		vel;
} odom_cfgType;

// ##############################################################
// ####################  FUNCTION PROTOTYPE  ####################
// ##############################################################
float LRL_HMC5883L_SetDeclination(int16_t declination_degs , int16_t declination_mins, char declination_dir);
void LRL_HMC5883L_Init(odom_cfgType * odom);
void LRL_HMC5883L_ReadHeading(odom_cfgType * odom);

void LRL_MPU6050_Init(odom_cfgType * odom);
void LRL_MPU6050_ReadAccel(odom_cfgType * odom);
void _LRL_MPU6050_EnableBypass(odom_cfgType * odom, uint8_t enable);
void LRL_MPU6050_ReadGyro(odom_cfgType *odom);
void LRL_MPU6050_ReadAll(odom_cfgType *odom);
void LRL_MPU6050_ComplementaryFilter(odom_cfgType *odom);

void LRL_Encoder_Init(odom_cfgType * odom);
void LRL_Encoder_ReadAngularSpeed(odom_cfgType * odom);

#endif /* INC_ODOMETRY_H_ */
