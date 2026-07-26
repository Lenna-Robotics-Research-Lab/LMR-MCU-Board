/**
 * @file rosserial.c
 * @brief This module handles the communication protocol for the robot.
 *
 * This function provides the functionality of rosserial on LCB of the robot.
 *
 * ============================================================================
 * PROTOCOL COMMAND STRUCTURE AND USAGE GUIDE
 * ============================================================================
 *
 * PACKET STRUCTURE (Incoming/Outgoing):
 * ┌────────────────────────────────────────────────────────────────────────┐
 * │ HEADER (5 bytes)             │ PAYLOAD (N bytes)      │ CHECKSUM (1)   │
 * ├──────┬──────┬──────┬─────────┬────────────────────────┬────────────────┤
 * │ SYNC │ SYNC │ LEN  │ LEN  	  │  HEADER   │ DATA       │ DATA           │
 * │ BYTE1│ BYTE2│ LOW  │ HIGH    │ CHECKSUM  │ (0 to N-1) │ CHECKSUM       │
 * │ 0xFF │ 0xFE │byte2 │byte3    │   byte4   │            │ (last byte)    │
 * └──────┴──────┴──────┴─────────┴───────────┴────────────┴────────────────┘
 *
 * BYTE-BY-BREAKDOWN:
 *
 * Byte 0 (SYNC1): Always 0xFF - Start of packet marker
 * Byte 1 (SYNC2): Always 0xFE - Validated in code (rxbuffer[1] == 0xFE)
 *                 Prevents all-zero packets and provides version control
 *                 for ROS Noetic/Melodic compatibility
 *
 * Bytes 2-3 (LENGTH): 16-bit payload length (little-endian)
 *         Byte 2 = Low byte of payload length
 *         Byte 3 = High byte of payload length
 *         Note: Payload = Function ID (2 bytes) + Data bytes
 *         Total packet length = Payload length + 8 (header + sync + checksum)
 *
 * Byte 4 (HEADER CHECKSUM): 255 - ((Byte2 + Byte3) % 256)
 *         Validates header integrity before receiving full packet
 *         Error code 0x02 if invalid
 *
 * Bytes 5-6 (FUNCTION ID): 2-byte function identifier (little-endian)
 *         Byte 5 = Low byte (primary function ID used in this implementation)
 *         Byte 6 = High byte (reserved for future expansion, currently unused)
 *
 * Bytes 7 to (N-1): DATA PAYLOAD
 *         Variable length data specific to each function
 *
 * Last Byte (DATA CHECKSUM): 255 - (sum of all data bytes % 256)
 *         Where "data bytes" includes Function ID (bytes 5-6) + Data payload
 *         Validates complete packet integrity
 *         Error code 0x03 if invalid
 *
 * ============================================================================
 * ERROR RESPONSE CODES:
 * ============================================================================
 * Error packets are 8 bytes with same header structure:
 * - 0xF1: Invalid sync byte (rxbuffer[1] != 0xFE) - Error code 0x01
 * - 0xF2: Invalid header checksum - Error code 0x02
 * - 0xF3: Invalid data checksum - Error code 0x03
 * Error packet format: {0xFF, 0xFE, 0x00, 0x00, 0xFF, ErrorCode, 0xFF, Checksum}
 *
 * ============================================================================
 * COMMAND EXAMPLES FOR QUERY MODE (Function ID 0x00):
 * ============================================================================
 *
 * QUERY COMMAND - Echo back test (Function ID: 0x00):
 * Purpose: Tests communication link by echoing received packet
 *
 * Example 1: Basic Query with no additional data
 * ┌──────┬──────┬──────┬──────┬───────┬──────┬──────┬───────┐
 * │ 0xFF │ 0xFE │ 0x00 │ 0x00 │  0xFF │ 0x00 │ 0x00 │  0xFF │
 * └──────┴──────┴──────┴──────┴───────┴──────┴──────┴───────┘
 *   SYNC1  SYNC2  LEN_L  LEN_H  H_CHK  ID_L   ID_H  D_CHK
 *
 * Byte breakdown:
 * - Bytes 0-1: 0xFF, 0xFE (sync markers)
 * - Bytes 2-3: 0x00, 0x00 (payload length = 0, no data beyond ID)
 * - Byte 4: 0xFF = 255 - ((0x00 + 0x00) % 256) (header checksum)
 * - Bytes 5-6: 0x00, 0x00 (function ID = 0x0000 for Query)
 * - Byte 7: 0xFF = 255 - ((0x00 + 0x00) % 256) (data checksum)
 *
 * Expected Response: Complete packet echoed back (8 bytes)
 *
 * Example 2: Query with custom test data (4 bytes)
 * ┌──────┬──────┬──────┬──────┬───────┬──────┬──────┬──────┬──────┬──────┬──────┬───────┐
 * │ 0xFF │ 0xFE │ 0x04 │ 0x00 │  0xFB │ 0x00 │ 0x00 │ 0x01 │ 0x02 │ 0x03 │ 0x04 │  0xF0 │
 * └──────┴──────┴──────┴──────┴───────┴──────┴──────┴──────┴──────┴──────┴──────┴───────┘
 *   SYNC1  SYNC2  LEN_L  LEN_H  H_CHK   ID_L   ID_H  DATA0 DATA1 DATA2 DATA3  D_CHK
 *
 * Byte breakdown:
 * - Bytes 0-1: 0xFF, 0xFE (sync markers)
 * - Bytes 2-3: 0x04, 0x00 (payload length = 4 bytes)
 * - Byte 4: 0xFB = 255 - ((0x04 + 0x00) % 256) = 251 (header checksum)
 * - Bytes 5-6: 0x00, 0x00 (function ID = 0x0000 for Query)
 * - Bytes 7-10: 0x01, 0x02, 0x03, 0x04 (test data)
 * - Byte 11: 0xF0 = 255 - ((0x00+0x00+0x01+0x02+0x03+0x04) % 256) = 255 - 10 = 245
 *             Data checksum includes bytes 5-10 (ID + data)
 *
 * Expected Response: Complete 12-byte packet echoed back
 *
 * ============================================================================
 * OTHER AVAILABLE COMMANDS:
 * ============================================================================
 *
 * 0x01 - ReadAll: Request all sensor data (odometry + IMU)
 *        Payload: No additional data required (just ID bytes)
 *        Example: {0xFF,0xFE,0x00,0x00,0xFF,0x01,0x00,0xFE}
 *        Response: 34-byte packet with sensor readings
 *
 * 0x02 - SetPID: Configure PID parameters
 *        Payload: Motor_select(1) + Kp(2) + Ki(2) + Kd(2) = 7 bytes
 *        Motor_select: 0x00=right motor, 0x01=left motor
 *        Example right motor: Kp=1.5, Ki=0.01, Kd=0.05 (scaled by 1000)
 *        {0xFF,0xFE,0x07,0x00,0xF8,0x02,0x00,0x00,0xDC,0x05,0x0A,0x00,0x32,0x00,...}
 *
 * 0x03 - GetPID: Request current PID parameters
 *        Payload: Motor_select(1 byte) - which motor to query
 *        Example: Query right motor (0x00)
 *        {0xFF,0xFE,0x01,0x00,0xFE,0x03,0x00,0x00,...}
 *
 * 0x04 - MotorSpeed: Set motor speeds (differential drive)
 *        Payload: Left speed(2) + Right speed(2) = 4 bytes
 *
 * @author Lenna Robotics Research Laboratory, Autonomous Systems Research Branch,
 *         Iran University of Science and Technology
 * @date January 27, 2026
 * @version 1.0
 * @link https://github.com/Lenna-Robotics-Research-Lab
 */

/*-------------------------- Includes ---------------------------------------- */

#include "rosserial.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
#include "pid.h"
#include "imu.h"
#include "odometry.h"
#include "stdio.h"
#include "motion.h"
/*-------------------------- Code Body ---------------------------------------- */

void LRL_ROSSerial_Init(rosserial_cfgType *rosserial_handle, UART_HandleTypeDef *huart)
{
	rosserial_handle->huart 			= huart;
    rosserial_handle->min_pkt_len 		= PACKET_HEADER_LENGTH;
    rosserial_handle->max_pkt_len 		= MAX_PACKET_LENGTH;

    rosserial_handle->err_hdl			= 0x00;

    rosserial_handle->packetReceived 	= 0;
    rosserial_handle->headerValid 		= 0;

    _LRL_Clear_Buffer(rosserial_handle);
    HAL_UART_Receive_IT(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->min_pkt_len);


}

void LRL_ROSSerial_Rx(rosserial_cfgType *rosserial_handle)
{
	if(!rosserial_handle->packetReceived)
	{
		if(rosserial_handle->rxbuffer[1] == 0xFE)
		{
			/*
			 * version control for rosserial for ROS noetic or melodic.
			 * this also would not allow invalid all zero packets pass
			 */

			uint8_t 	_checksum_calc = 0;
			uint16_t	_temp;

			_checksum_calc = 255 - ((rosserial_handle->rxbuffer[3]) + (rosserial_handle->rxbuffer[2]) % 256);

			if(_checksum_calc == rosserial_handle->rxbuffer[4])
			{
				_temp	= ((uint16_t)(rosserial_handle->rxbuffer[3]<<8)) | (uint16_t)(rosserial_handle->rxbuffer[2]);
				rosserial_handle->data_len	= _temp + 3; // this is the full packet length - the received initial 5
				rosserial_handle->pkt_len 	= _temp + 8;

				HAL_UART_Receive_IT(rosserial_handle->huart,
								   &rosserial_handle->rxbuffer[rosserial_handle->min_pkt_len],
								   rosserial_handle->data_len);

				rosserial_handle->headerValid = 1;
			}
			else
			{
				rosserial_handle->err_hdl = 0x02;
				rosserial_handle->headerValid = 0;
			}
			rosserial_handle->packetReceived = 1;
		}
		else
		{

			rosserial_handle->err_hdl = 0x01;
		}
	}
	else
	{
		HAL_UART_Receive_IT(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->min_pkt_len);
	}
}





void LRL_ROSSerial_Data_Handle(rosserial_cfgType *rosserial_handle, imu_statetype *imu, odom_cfgType *odom, pid_cfgType *pid)
{
	uint8_t _err_msg[9];
    if(rosserial_handle->packetReceived)
    {
	    if(rosserial_handle->headerValid && rosserial_handle->rxbuffer[rosserial_handle->pkt_len - 1] != 0)
	    {
		    memcpy(rosserial_handle->data, &rosserial_handle->rxbuffer[rosserial_handle->min_pkt_len], rosserial_handle->data_len);

		    uint8_t _data_sum = 0, _data_checksum = 0;

		    for(int i = 0; i<rosserial_handle->data_len; i++)
		    {
			    _data_sum 	+= rosserial_handle->data[i];
		    }

		    _data_checksum 	=  255 - (_data_sum % 256);

		    if(_data_checksum == rosserial_handle->data[rosserial_handle->data_len])
		    {
		  	   rosserial_handle->dataValid = 1;
		  	   _LRL_ROSSerial_Function(rosserial_handle, imu, odom, pid);
		  	   memset(rosserial_handle->rxbuffer, 0 , sizeof(rosserial_handle->rxbuffer));
		  	   memset(rosserial_handle->data, 0 , sizeof(rosserial_handle->data));
		    }
		    else
		    {
		    	rosserial_handle->err_hdl = 0x03;
		    }


		    rosserial_handle->headerValid = 0;
	    }

	    rosserial_handle->packetReceived = 0;

    }
    if(rosserial_handle->err_hdl == 0x01)
    {
    	_LRL_Clear_Buffer(rosserial_handle);
    	memcpy(_err_msg, (uint8_t[]){0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xF1, 0xFF, 0x0E}, 8);
    	HAL_UART_Transmit(rosserial_handle->huart, _err_msg, 8, 1);


    }
    else if(rosserial_handle->err_hdl == 0x02)
    {
    	_LRL_Clear_Buffer(rosserial_handle);
    	memcpy(_err_msg, (uint8_t[]){0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xF2, 0xFF, 0x0D}, 8);
    	HAL_UART_Transmit(rosserial_handle->huart, _err_msg, 8, 1);
    }
    else if(rosserial_handle->err_hdl== 0x03)
    {
    	_LRL_Clear_Buffer(rosserial_handle);
    	memcpy(_err_msg, (uint8_t[]){0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xF3, 0xFF, 0x0C}, 8);
    	HAL_UART_Transmit(rosserial_handle->huart, _err_msg, 8, 1);
    }
}

void _LRL_ROSSerial_Function(rosserial_cfgType *rosserial_handle, imu_statetype *imu, odom_cfgType *odom, pid_cfgType *pid)
{
	uint8_t _id;

	/*
	 * in rosserial the functions are passed by two bytes of ID however here the number of functions
	 * are limited to just 4. We did not want to change the overall packet frame of rosserial packet
	 * hence we used the packeting as it was but we only use the low byte as the indicator of
	 * our function ID.
	 */
	_id = rosserial_handle->data[0];
	if(_id == 0x00)
	{
		LRL_ROSSerial_Query(rosserial_handle);
	}
	else if(_id == 0x01)
	{
		LRL_ROSSerial_ReadAll(rosserial_handle, odom, imu);
	}
	else if(_id == 0x02)
	{
		LRL_ROSSerial_SetPID(rosserial_handle, pid);
	}
	else if(_id == 0x03)
	{
		LRL_ROSSerial_GetPID(rosserial_handle, pid);
	}
	else if(_id == 0x04)
	{
		LRL_ROSSerial_MotorSeped(rosserial_handle, pid);
	}
	else
	{
		rosserial_handle->dataValid = 0;

	}
}

void LRL_ROSSerial_Query(rosserial_cfgType *rosserial_handle)
{
	HAL_UART_Transmit(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->pkt_len, 10);
	rosserial_handle->dataValid = 0;
}


void LRL_ROSSerial_ReadAll(rosserial_cfgType *rosserial_handle, odom_cfgType *odom, imu_statetype *imu)
{
    LRL_IMU_MPUReadAll(imu);
    LRL_IMU_MagReadHeading(imu);
    LRL_Odometry_ReadAngularSpeed(odom);
	// Start of packet markers.
	rosserial_handle->txbuffer[0] 	= 0xFF;
	rosserial_handle->txbuffer[1] 	= 0xFE;

	// Message data length.
	rosserial_handle->txbuffer[2] 	= 0x1A; // low byte
	rosserial_handle->txbuffer[3] 	= 0x00; // high byte

	rosserial_handle->txbuffer[4] 	= 0xFF - 0x1A; // checksum for header validity

	rosserial_handle->txbuffer[5] 	= 0x01; // ID low byte
	rosserial_handle->txbuffer[6] 	= 0x00; // ID high byte
	// Pack odometry data.
	rosserial_handle->txbuffer[7] 	= (uint8_t)(odom->vel.left >> 8);
	rosserial_handle->txbuffer[8] 	= (uint8_t)(odom->vel.left & 0x00FF);
	rosserial_handle->txbuffer[9] 	= (uint8_t)(odom->vel.right >> 8);
	rosserial_handle->txbuffer[10] 	= (uint8_t)(odom->vel.right & 0x00FF);
	rosserial_handle->txbuffer[11] 	= (uint8_t)(odom->dist.left >> 8);
	rosserial_handle->txbuffer[12] 	= (uint8_t)(odom->dist.left & 0x00FF);
	rosserial_handle->txbuffer[13] 	= (uint8_t)(odom->dist.right >> 8);
	rosserial_handle->txbuffer[14] 	= (uint8_t)(odom->dist.right & 0x00FF);

	// Pack IMU accelerometer data.
	rosserial_handle->txbuffer[15] 	= (uint8_t)(imu->accel.x_calibrated >> 8);
	rosserial_handle->txbuffer[16] 	= (uint8_t)(imu->accel.x_calibrated & 0x00FF);
	rosserial_handle->txbuffer[17] 	= (uint8_t)(imu->accel.y_calibrated >> 8);
	rosserial_handle->txbuffer[18] 	= (uint8_t)(imu->accel.y_calibrated & 0x00FF);
	rosserial_handle->txbuffer[19] 	= (uint8_t)(imu->accel.z_calibrated >> 8);
	rosserial_handle->txbuffer[20] 	= (uint8_t)(imu->accel.z_calibrated & 0x00FF);

	// Pack IMU gyroscope data.
	rosserial_handle->txbuffer[21] 	= (uint8_t)(imu->gyro.x_calibrated >> 8);
	rosserial_handle->txbuffer[22] 	= (uint8_t)(imu->gyro.x_calibrated & 0x00FF);
	rosserial_handle->txbuffer[23] 	= (uint8_t)(imu->gyro.y_calibrated >> 8);
	rosserial_handle->txbuffer[24] 	= (uint8_t)(imu->gyro.y_calibrated & 0x00FF);
	rosserial_handle->txbuffer[25] 	= (uint8_t)(imu->gyro.z_calibrated >> 8);
	rosserial_handle->txbuffer[26] 	= (uint8_t)(imu->gyro.z_calibrated & 0x00FF);

	// Pack IMU angular position data.
	rosserial_handle->txbuffer[27] 	= (uint8_t)(imu->angle.x >> 8);
	rosserial_handle->txbuffer[28] 	= (uint8_t)(imu->angle.x & 0x00FF);
	rosserial_handle->txbuffer[29] 	= (uint8_t)(imu->angle.y >> 8);
	rosserial_handle->txbuffer[30] 	= (uint8_t)(imu->angle.y & 0x00FF);

	// Pack magnetometer heading data.
	rosserial_handle->txbuffer[31] 	= (uint8_t)(imu->mag.heading >> 8);
	rosserial_handle->txbuffer[32] 	= (uint8_t)(imu->mag.heading & 0x00FF);

	uint8_t _checksum = 0;

	for(int i = 0; i < (0x02 + 0x1A) ; i++)
	{
		_checksum += rosserial_handle->txbuffer[i+5];
	}

	rosserial_handle->txbuffer[33] 	= _checksum;

	// Transmit the complete packet.
	HAL_UART_Transmit(rosserial_handle->huart, rosserial_handle->txbuffer, 34, 10);
//	memset(rosserial_handle->txbuffer, 0 , sizeof(rosserial_handle->txbuffer));
	rosserial_handle->dataValid = 0;
}

void LRL_ROSSerial_SetPID(rosserial_cfgType *rosserial_handle, pid_cfgType *pid)
{
	if(rosserial_handle->data[1] == 0x00)
	{
		pid->Kp_r = (float)(
		    ((uint16_t)rosserial_handle->data[2]) |
		    ((uint16_t)rosserial_handle->data[3] << 8)
		) / PID_PRECISION;

		pid->Ki_r = (float)(
		    ((uint16_t)rosserial_handle->data[4]) |
		    ((uint16_t)rosserial_handle->data[5] << 8)
		) / PID_PRECISION;

		pid->Kd_r = (float)(
		    ((uint16_t)rosserial_handle->data[6]) |
		    ((uint16_t)rosserial_handle->data[7] << 8)
		) / PID_PRECISION;
	}
	else if(rosserial_handle->data[1] == 0x01)
	{
		pid->Kp_l = (float)(
		    ((uint16_t)rosserial_handle->data[2]) |
		    ((uint16_t)rosserial_handle->data[3] << 8)
		) / PID_PRECISION;

		pid->Ki_l = (float)(
		    ((uint16_t)rosserial_handle->data[4]) |
		    ((uint16_t)rosserial_handle->data[5] << 8)
		) / PID_PRECISION;

		pid->Kd_l = (float)(
		    ((uint16_t)rosserial_handle->data[6]) |
		    ((uint16_t)rosserial_handle->data[7] << 8)
		) / PID_PRECISION;
	}
	HAL_UART_Transmit(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->pkt_len, 10);
	rosserial_handle->dataValid = 0;

}

void LRL_ROSSerial_GetPID(rosserial_cfgType *rosserial_handle, pid_cfgType *pid_cfg)
{
	// Start of packet markers.
	rosserial_handle->txbuffer[0] 	= 0xFF;
	rosserial_handle->txbuffer[1] 	= 0xFE;

	// Message data length.
	rosserial_handle->txbuffer[2] 	= 0x03; // low byte
	rosserial_handle->txbuffer[3] 	= 0x00; // high byte

	rosserial_handle->txbuffer[4] 	= 0xFF - 0x03;

	rosserial_handle->txbuffer[5] 	= 0x03;

	uint16_t _temp;
	if(rosserial_handle->data[1] == 0x00)
	{
		rosserial_handle->txbuffer[6] 	= 0x00;
		_temp = (uint16_t)(pid_cfg->Kp_r * PID_PRECISION);
		rosserial_handle->txbuffer[7] 	= (uint8_t)(_temp);
		rosserial_handle->txbuffer[8] 	= (uint8_t)(_temp>>8);

		_temp = (uint16_t)(pid_cfg->Ki_r * PID_PRECISION);
		rosserial_handle->txbuffer[9] 	= (uint8_t)(_temp);
		rosserial_handle->txbuffer[10] 	= (uint8_t)(_temp>>8);

		_temp = (uint16_t)(pid_cfg->Kd_r * PID_PRECISION);
		rosserial_handle->txbuffer[11] 	=  (uint8_t)(_temp);
		rosserial_handle->txbuffer[12] 	=  (uint8_t)(_temp>>8);
	}
	else if(rosserial_handle->data[1] == 0x01)
	{
		rosserial_handle->txbuffer[6] 	= 0x01;
		_temp = (uint16_t)(pid_cfg->Kp_l * PID_PRECISION);
		rosserial_handle->txbuffer[7] 	= (uint8_t)(_temp);
		rosserial_handle->txbuffer[8] 	= (uint8_t)(_temp>>8);

		_temp = (uint16_t)(pid_cfg->Ki_l * PID_PRECISION);
		rosserial_handle->txbuffer[9] 	= (uint8_t)(_temp);
		rosserial_handle->txbuffer[10] 	= (uint8_t)(_temp>>8);

		_temp = (uint16_t)(pid_cfg->Kd_l * PID_PRECISION);
		rosserial_handle->txbuffer[11] 	=  (uint8_t)(_temp);
		rosserial_handle->txbuffer[12] 	=  (uint8_t)(_temp>>8);
	}
	uint8_t _checksum = 0;

	for(int i = 0; i < 8 ; i++)
	{
		_checksum += rosserial_handle->txbuffer[i+5];
	}

	rosserial_handle->txbuffer[13]	= _checksum;

	HAL_UART_Transmit(rosserial_handle->huart, rosserial_handle->txbuffer, 14, 10);
//	memset(rosserial_handle->txbuffer, 0 , sizeof(rosserial_handle->txbuffer));
	rosserial_handle->dataValid = 0;
}

void LRL_ROSSerial_MotorSeped(rosserial_cfgType *rosserial_handle, pid_cfgType *pid)
{
	int16_t _temp_ref_l=0, _temp_ref_r=0;
}

void _LRL_Clear_Buffer(rosserial_cfgType *rosserial_handle)
{
	do {
	  if(__HAL_UART_GET_FLAG(rosserial_handle->huart, UART_FLAG_RXNE)) {
		volatile uint8_t tmp = rosserial_handle->huart->Instance->DR;  // Read FIFO
		(void)tmp;
	  }
	} while(__HAL_UART_GET_FLAG(rosserial_handle->huart, UART_FLAG_RXNE));

//	    Clear ALL error flags
	__HAL_UART_CLEAR_OREFLAG(rosserial_handle->huart);
	__HAL_UART_CLEAR_PEFLAG(rosserial_handle->huart);
	__HAL_UART_CLEAR_FEFLAG(rosserial_handle->huart);
	__HAL_UART_CLEAR_NEFLAG(rosserial_handle->huart);

//	    Reset HAL state
	rosserial_handle->huart->RxState = HAL_UART_STATE_READY;
	rosserial_handle->huart->ErrorCode = 0;
	HAL_UART_Receive_IT(rosserial_handle->huart, rosserial_handle->rxbuffer, rosserial_handle->min_pkt_len);
	rosserial_handle->err_hdl = 0x00;
}

