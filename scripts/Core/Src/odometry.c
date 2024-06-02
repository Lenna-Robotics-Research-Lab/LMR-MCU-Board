/*
 * odometry.c
 *
 *  Created on: Jun 1, 2024
 *      Author: Lenna Robotics Research Laboratory
 *      		Autonomous Systems Research Branch
 *				Iran University of Science and Technology
 *		GitHub:	github.com/Lenna-Robotics-Research-Lab
 */

#include "odometry.h"
#include "math.h"

uint8_t _i2c_reg_data;
uint8_t _mag_buffer[6];
float _mag_heading_temp;

uint8_t _imu_addr_check;
uint8_t _imu_buffer[6];
float tmp_x,tmp_y,tmp_z;

// ############################################################
// ####################  HMC MAGNETOMETER  ####################
// ############################################################
float LRL_HMC5883L_SetDeclination(int16_t declination_degs , int16_t declination_mins, char declination_dir)
{
	int8_t _dir = 0;

	if (declination_dir == 'E' || declination_dir == 'e' || declination_dir == 1)
	{
		_dir = 1;
	}
	else if (declination_dir == 'W' || declination_dir == 'w' || declination_dir == -1)
	{
		_dir = -1;
	}

	return ((_dir)* ( declination_degs + (1/60 * declination_mins)) * (M_PI / 180));
}

void LRL_HMC5883L_Init(odom_cfgType * odom)
{
    // write CONFIG_A register
	HAL_I2C_Master_Transmit(odom->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_WRITE, 1, DELAY_TIMEOUT);
	_i2c_reg_data = 0x10;
	HAL_I2C_Mem_Write(odom->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);

	// write CONFIG_B register
	HAL_I2C_Master_Transmit(odom->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_WRITE, 1, DELAY_TIMEOUT);
	_i2c_reg_data = 0xE0;
	HAL_I2C_Mem_Write(odom->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);

	// write MODE register
	HAL_I2C_Master_Transmit(odom->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_WRITE, 1, DELAY_TIMEOUT);
	HAL_I2C_Mem_Write(odom->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_MODE, 1, (uint8_t *)HMC5883L_MODE_SINGLE, 1, DELAY_TIMEOUT);

	HAL_Delay(10);
}

void LRL_HMC5883L_ReadHeading(odom_cfgType * odom)
{
	HAL_I2C_Master_Transmit(odom->imu.hi2c, HMC5883L_ADDRESS, (uint8_t *)HMC5883L_ADDRESS_READ, 1, DELAY_TIMEOUT);
	HAL_I2C_Mem_Read(odom->imu.hi2c, HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 1, (uint8_t *)&_mag_buffer, 6, DELAY_TIMEOUT);

	odom->mag.x = (int16_t)((_mag_buffer[0] << 8) | _mag_buffer[1]);
	odom->mag.y = (int16_t)((_mag_buffer[4] << 8) | _mag_buffer[5]);
	odom->mag.z = (int16_t)((_mag_buffer[2] << 8) | _mag_buffer[3]);

	// Evaluate Heading and Correcting Declination (IRAN Coordinates)
	_mag_heading_temp = atan2(odom->mag.x, odom->mag.y) + MAGNETIC_DECLINATION;

	// Correct for when signs are reversed.
    if(_mag_heading_temp < 0)
    	_mag_heading_temp += 2*M_PI;

    // Check for wrap due to addition of declination.
    if(_mag_heading_temp > 2*M_PI)
    	_mag_heading_temp -= 2*M_PI;

    // Convert radians to degrees for readability.
    odom->mag.heading = _mag_heading_temp * 180/M_PI;
}

// #######################################################
// ####################  IMU MPU6050  ####################
// #######################################################
void LRL_MPU6050_Init(odom_cfgType * odom)
{
    HAL_I2C_Mem_Read(odom->imu.hi2c, MPU_ADDR, WHO_AM_I, 1, &_imu_addr_check, 1, DELAY_TIMEOUT);

    if (_imu_addr_check == 0x68) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        _i2c_reg_data = 0x00;
        HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, PWR_MGMT_1, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        _i2c_reg_data = 0x07;
        HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, SMPLRT_DIV, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        _i2c_reg_data = 0x00;
        HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->  250 �/s
        _i2c_reg_data = 0x00;
        HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, GYRO_CONFIG, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);
    }
}

void LRL_MPU6050_ReadAccel(odom_cfgType * odom)
{
    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(odom->imu.hi2c, MPU_ADDR, ACCEL_XOUT_H, 1, (uint8_t *)&_imu_buffer, 6, DELAY_TIMEOUT);

    odom->accel.x = (int16_t)(_imu_buffer[0] << 8 | _imu_buffer[1]);
    odom->accel.y = (int16_t)(_imu_buffer[2] << 8 | _imu_buffer[3]);
    odom->accel.z = (int16_t)(_imu_buffer[4] << 8 | _imu_buffer[5]);

    /***
     * convert the RAW values into acceleration in 'g'
     * we have to divide according to the Full scale value set in FS_SEL
     * I have configured FS_SEL = 0. So I am dividing by 16384.0
     * for more details check ACCEL_CONFIG Register
    ****/

    odom->accel.x /= (ACCEL_X_CORRECTOR / FLOAT_SCALING);
    odom->accel.y /= (ACCEL_Y_CORRECTOR / FLOAT_SCALING);
    odom->accel.z /= (ACCEL_Z_CORRECTOR / FLOAT_SCALING);
}

void LRL_MPU6050_ReadGyro(odom_cfgType *odom)
{
	HAL_I2C_Mem_Read(odom->imu.hi2c, MPU_ADDR, GYRO_XOUT_H, 1, (uint8_t *)_imu_buffer, 6, DELAY_TIMEOUT);

	odom->gyro.x = (int16_t)(_imu_buffer[0] << 8 | _imu_buffer[1]);
	odom->gyro.y = (int16_t)(_imu_buffer[2] << 8 | _imu_buffer[3]);
	odom->gyro.z = (int16_t)(_imu_buffer[4] << 8 | _imu_buffer[5]);

	odom->gyro.x /= (GYRO_CORRECTOR / FLOAT_SCALING);
	odom->gyro.y /= (GYRO_CORRECTOR / FLOAT_SCALING);
	odom->gyro.z /= (GYRO_CORRECTOR / FLOAT_SCALING);
}

void LRL_MPU6050_EnableBypass(odom_cfgType * odom, uint8_t enable)
{
	_i2c_reg_data = 0x00;
	HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, USER_CTRL, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);
	_i2c_reg_data = 0x02 * enable;
	HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, INT_PIN_CFG, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);
	_i2c_reg_data = 0x00;
	HAL_I2C_Mem_Write(odom->imu.hi2c, MPU_ADDR, PWR_MGMT_1, 1, &_i2c_reg_data, 1, DELAY_TIMEOUT);
}

// #########################################################
// ####################  MOTOR ENCODER  ####################
// #########################################################
void LRL_Encoder_Init(odom_cfgType * odom)
{
	__HAL_TIM_SET_COUNTER(odom->enc_right.htim, 0);
	__HAL_TIM_SET_COUNTER(odom->enc_left.htim, 0);

	odom->enc_right.tick = 0;
	odom->enc_left.tick = 0;
	odom->enc_right.tick_prev = 0;
	odom->enc_left.tick_prev = 0;
}

void LRL_Encoder_ReadAngularSpeed(odom_cfgType * odom)
{
	odom->enc_right.tick = __HAL_TIM_GET_COUNTER(odom->enc_right.htim);
	odom->enc_left.tick = __HAL_TIM_GET_COUNTER(odom->enc_left.htim);

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_right.htim) == 0)
	{
	  if(odom->enc_right.tick - odom->enc_right.tick_prev >= 0)
	  {
		  odom->vel.right = odom->enc_right.tick - odom->enc_right.tick_prev;
	  }
	  else
	  {
		  odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick_prev) + odom->enc_right.tick;
	  }
	}
	else
	{
	  if(odom->enc_right.tick_prev - odom->enc_right.tick >= 0)
	  {
		  odom->vel.right = -(odom->enc_right.tick - odom->enc_right.tick_prev);
	  }
	  else
	  {
		  odom->vel.right = (odom->enc_right.MAX_ARR - odom->enc_right.tick) + odom->enc_right.tick_prev;
	  }
	}

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(odom->enc_left.htim) == 0)
	{
	  if(odom->enc_left.tick - odom->enc_left.tick_prev >= 0)
	  {
		  odom->vel.left = odom->enc_left.tick - odom->enc_left.tick_prev;
	  }
	  else
	  {
		  odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick_prev) + odom->enc_left.tick;
	  }
	}
	else
	{
	  if(odom->enc_left.tick_prev - odom->enc_left.tick >= 0)
	  {
		  odom->vel.left = -(odom->enc_left.tick - odom->enc_left.tick_prev);
	  }
	  else
	  {
		  odom->vel.left = (odom->enc_left.MAX_ARR - odom->enc_left.tick) + odom->enc_left.tick_prev;
	  }
	}

	odom->vel.right = odom->vel.right * odom->enc_right.TICK2RPM;
	odom->vel.left = odom->vel.left * odom->enc_left.TICK2RPM;

	odom->enc_right.tick_prev = odom->enc_right.tick;
	odom->enc_left.tick_prev = odom->enc_left.tick;
}
