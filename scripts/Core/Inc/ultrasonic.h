/*
 * ultrasonic.h
 *
 *  Created on: Oct 18, 2023
 *      Author: Lenna Robotics
 *      		Arian Hajizade 	rian.hajizadeh@gmail.com
 *      		Erfan Riazati 	erf.rzt@gmail.com
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

//====================== variables ======================

//uint8_t Capture_Flag = 0;

// used in ultrasonic callback as a variable

//====================== functions ======================

void LMR_US1_Init(void);
void LMR_US1_Trig(void);
void LMR_Delayus (uint16_t time);
uint8_t LMR_US1_Read(TIM_HandleTypeDef *htim);

#endif /* INC_ULTRASONIC_H_ */
