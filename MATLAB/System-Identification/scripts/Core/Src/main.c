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
#include "ultrasonic.h"
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
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char MSG[64];

uint8_t input_speed ;// step given by MATLAB code
uint16_t enc_temp = 0 , enc_diff = 0;
uint16_t encoder_tick[2] = {0};

uint8_t flag_tx = 0;
const motor_cfgType motor_right =
{
	MOTOR_PORT,
	MOTOR1_A_PIN,
	MOTOR_PORT,
	MOTOR1_B_PIN,
	&htim8,
	TIM_CHANNEL_1,
	1000,
	1
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
	-1
};

const diffDrive_cfgType diff_robot =
{
	motor_right,
	motor_left,
	65,
	200
};

const ultrasonic_cfgType us_front =
{
	US1_TRIG_PORT,
	US1_TRIG_PIN,
	&htim4,
	TIM4,
	TIM_CHANNEL_3,
	168,
	1
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE BEGIN 2 */
  LRL_Delay_Init();			// TIMER Initialization for Delay us
  LRL_US_Init(us_front); 	// TIMER Initialization for Ultrasonics

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  //printf("Lenna Robotics Research Lab. \r\n");
  // HAL_Delay(1000);


  TIM2->CNT = 0;
  TIM3->CNT = 0;
  encoder_tick[0] = (TIM2->CNT);
  encoder_tick[1] = (TIM3->CNT);

  HAL_UART_Receive_IT(&huart1,&input_speed, 1);


  //HAL_UART_Transmit_IT(&huart1,(uint8_t *)&enc_diff, sizeof(enc_diff));

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
//
//	  TIM8->CCR2 = 1000;
//
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
//
//	  TIM8->CCR1 = 1000;


	  //LRL_Motion_Control(diff_robot, -100, 100);


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

	  LRL_Motor_Speed(motor_right, input_speed);

//	  LRL_Motor_Speed(motor_left, -50);
//	  HAL_Delay(1000);
//	  LRL_Motor_Speed(motor_right, 0);
//	  LRL_Motor_Speed(motor_left, 0);
//	  HAL_Delay(1000);
//	  LRL_Motor_Speed(motor_right, -100);
//	  LRL_Motor_Speed(motor_left, 50);
//	  HAL_Delay(1000);
//	  LRL_Motor_Speed(motor_right, 0);
//	  LRL_Motor_Speed(motor_left, 0);
//	  HAL_Delay(1000);

	  encoder_tick[0] = (TIM2->CNT);
	  encoder_tick[1] = (TIM3->CNT);

//	  if(encoder_tick[0] < 24480)
//	  {
//		  LRL_Motion_Control(diff_robot, -40,0);
//	  }
//	  else
//	  {
//		  LRL_Motion_Control(diff_robot, 0, 0);
//		  TIM2->CNT = 0;
//		  HAL_Delay(1000);
//	  }


	  if(encoder_tick[1] - enc_temp >= 0)
	  {
		  enc_diff = encoder_tick[1] - enc_temp;
	  }
	  else
	  {
		  enc_diff = (48960 - enc_temp) + encoder_tick[1];
	  }
	  enc_temp = encoder_tick[1];

	  if(flag_tx == 1){
		  HAL_UART_Transmit(&huart1,(uint8_t *)&enc_diff, sizeof(enc_diff),10);
		  flag_tx = 0;
	  }

	  //HAL_UART_Transmit(&huart1,(uint8_t *)&enc_diff, sizeof(enc_diff),10);
	  HAL_Delay(10);

//	  if(encoder_tick[0] - enc_temp >= 0)
//	  {
//		  enc_diff = encoder_tick[0] - enc_temp;
//	  }
//	  else
//	  {
//		  enc_diff = (48960 - enc_temp) + encoder_tick[0];
//	  }
//	  enc_temp = encoder_tick[0];
	  // angular_velocity = enc_diff * (6000 / 48960)


//	  sprintf(MSG, "encoder ticks: %04d\t%04d\r\n", encoder_tick[0], encoder_tick[1]);
//	  printf(MSG);


//	LRL_US_Trig(us_front);	// Trigging the Sensor for 15us
//
//	sprintf(MSG, "Distance:\t %05.1f \r\n", LRL_US_Read(us_front));
//	printf(MSG);
//
	  //HAL_Delay(1000);


//	  	sprintf(MSG, "Distance:\t %d \r\n", i);
//	  	printf(i);

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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// TIMER Input Capture Callback
	LRL_US_TMR_IC_ISR(htim, us_front);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1,&input_speed, 1);
	flag_tx = 1;
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	//
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// TIMER Overflow Callback
	LRL_US_TMR_OVF_ISR(htim, us_front);
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
