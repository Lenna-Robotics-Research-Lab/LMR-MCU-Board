/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "eth.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "mcu_layout.h"
#include "utilities.h"
#include "motion.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// this is for using printf
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// message bufffer used to send data for logging
uint8_t MSG[64];

// RX and TX MATLAB buffers
uint8_t input_speed[2] ; // step given by MATLAB code
uint16_t encoder_buff[2]; // putting encoder datas together

// temporary variables used for encoders
uint16_t left_enc_temp = 0;
uint16_t right_enc_temp = 0;

// speed calculated by encoders
uint16_t right_enc_diff = 0;
uint16_t left_enc_diff = 0;

// Notice :
// encoder_tick[0] -> left motor
// encoder_tick[1] -> right motor
// used for reading encoder ticks
uint16_t encoder_tick[2];

// this flag is used for UART transmit
uint8_t flag_tx = 0;
uint8_t sampling_time_flag = 0;



// ####################   Motor struct Value Setting   ###################

// defined in motion library
const motor_cfgType motor_right =
{
	MOTOR_PORT,
	MOTOR1_A_PIN,
	MOTOR_PORT,
	MOTOR1_B_PIN,
	&htim8,
	TIM_CHANNEL_1,
	1000,
	//1
};
const motor_cfgType motor_left =
{
	MOTOR_PORT,
	MOTOR2_A_PIN,
	MOTOR_PORT,
	MOTOR2_B_PIN,
	&htim8,
	TIM_CHANNEL_2,
	1000,
	//-1
};

const diffDrive_cfgType diff_robot =
{
	motor_right,
	motor_left,
	65,
	200
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ####################   UART Tx -> printf   ####################
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

// PWM and encoder
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

// timer to create 0.01 seconds non-blocking interrupt
  HAL_TIM_Base_Start_IT(&htim5);

// ####################   Encoder Initialization   ####################
  TIM2->CNT = 0;
  TIM3->CNT = 0;
  encoder_tick[0] = (TIM2->CNT);
  encoder_tick[1] = (TIM3->CNT);

// ####################   MATLAB Communication Initialization   ####################

// Initializing the MATLAB communication and identification
// Receiving from MATLAB SIMULINK system identification
// getting the motor speeds
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&input_speed, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

// ####################   Motor Test Scenarios   ####################

//	  LRL_Motion_Control(diff_robot, -100, 100);
//	  if(input_speed >= 50 && input_speed <= 100)
//	  {
//		  LRL_Motor_Speed(motor_left, -1*input_speed);
//	  }
//	  else
//	  {
//		  HAL_GPIO_TogglePin(BLINK_LED_PORT, BLINK_LED_PIN);
//		  HAL_Delay(100);
//		  LRL_Motor_Speed(motor_left, 0);
//	  }

//	  LRL_Motor_Speed(motor_right, input_speed);

// ####################   Encoder Reading   ####################

	  encoder_tick[0] = (TIM2->CNT); // Left Motor Encoder
	  encoder_tick[1] = (TIM3->CNT); // Right Motor Encoder

	  LRL_Motor_Speed(motor_right, input_speed[1]);
	  LRL_Motor_Speed(motor_left, input_speed[0]);

	  // Reading the Encoder for the right Motor

	  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3) == 0)
	  {
		  if(encoder_tick[1] - right_enc_temp >= 0)
		  {
			  right_enc_diff = encoder_tick[1] - right_enc_temp;
		  }
		  else
		  {
			  right_enc_diff = (48960 - right_enc_temp) + encoder_tick[1];
		  }
		  right_enc_temp = encoder_tick[1];
	  }
	  else
	  {
		  if(right_enc_temp - encoder_tick[1] >= 0)
		  {
			  right_enc_diff = -(encoder_tick[1] - right_enc_temp);
		  }
		  else
		  {
			  right_enc_diff = (48960 - encoder_tick[1]) + right_enc_temp;
		  }
		  right_enc_temp = encoder_tick[1];
	  }

		  // Reading the Encoder for the left Motor

	  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == 0)
	  {
		  if(encoder_tick[0] - left_enc_temp >= 0)
		  {
			  left_enc_diff = encoder_tick[0] - left_enc_temp;
		  }
		  else
		  {
			  left_enc_diff = (48960 - left_enc_temp) + encoder_tick[0];
		  }
		  left_enc_temp = encoder_tick[0];
	  }
	  else
	  {
		  if(left_enc_temp - encoder_tick[0] >= 0)
		  {
			  left_enc_diff = -(encoder_tick[0] - left_enc_temp);
		  }
		  else
		  {
			  left_enc_diff = (48960 - encoder_tick[0]) + left_enc_temp;
		  }
		  left_enc_temp = encoder_tick[0];
	  }

	  encoder_buff[0] = left_enc_diff;
	  encoder_buff[1] = right_enc_diff;

// ####################   Transmit Speed for MATLAB Identification   ####################

// Sending the speed read by encoder only if there is a receiving data
	  if(flag_tx == 1){
		  HAL_UART_Transmit(&huart1,(uint8_t *)&encoder_buff, sizeof(encoder_buff),10);
		  HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 1);
		  flag_tx = 0;
	  }

	  // Notice:
	  // Pay great ATTENTION to the delayed time hence the wrong data can be received in MATLAB
	  // Since the Data is read roughly every 0.01 second this amount of delay is necessary for correct transmission
	  HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


// ####################   UART Receive Callback   ####################

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&input_speed, 2);
	flag_tx = 1;
}

// ####################   Timer To Creat 0.01 Delay Callback   ####################

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim == &htim5)
	{
		sampling_time_flag = 1;
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
