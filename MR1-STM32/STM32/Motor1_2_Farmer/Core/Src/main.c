/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDController MPID;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R 0.075 //m
#define lx 0.20  //m
#define ly 0.34 //m
#define CPR 911
#define Sample_time 10 // ms
#define pi 3.14
//#define kp 7.92
//#define ki 136.48
#define kp 1.92
#define ki 169.6
#define kd 0
#define Motor1 0
#define Motor2 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[4];
uint32_t TxMailbox;
uint16_t ReadValue;
int flag = 0;
uint8_t cntt;
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
float v1; // target speed of motor1
float v2; // target speed of motor2
float v3; // target speed of motor3
float v4; // target speed of motor4
uint16_t count[4]; // count pulse from encoder
uint16_t new_count[4];
uint8_t count_state[4];
uint16_t diff[4]; // difference between count and new_count in a sample time
uint16_t V1_out = 0;
uint16_t V2_out = 0;
uint16_t V3_out = 0;
uint16_t V4_out = 0;
float speedM[4];
float rdps[4];

float Motor1_speed ;
float Motor2_speed ;
float V1; // target speed of motor1
float V2; // target speed of motor2
float V3; // target speed of motor3
float V4; // target speed of motor4

float pwm_M1 = 0;
float pwm_M2 = 0;
float V;
float v;
float Vx;
float Vy;
float Vz;
uint16_t v1_out = 0;
uint16_t v2_out = 0;
uint16_t v3_out = 0;
uint16_t v4_out = 0;
uint16_t c;
uint8_t speed;
float theta = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float Motors_RPS( int j,float SampleTime, float N_round)
{

	new_count[Motor1] = TIM4->CNT;
	new_count[Motor2] = TIM3->CNT;

	count_state[Motor1] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
	count_state[Motor2] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	if (count_state[j])
	{
		if (new_count[j] <= count[j])
		{ // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		}
		else
		{
			diff[j] = (65536 - new_count[j]) + count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime)*-1;
	}
	else
	{
		if (new_count[j] >= count[j])
		{ // Check for counter overflow
			diff[j] = new_count[j] - count[j];
		}
		else
		{
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
	}

	rdps[j] = -2.0f * pi * speedM[j];
	count[j] = new_count[j];

	return rdps[j];
}
float map(float Input, float Min_Input, float Max_Input, float Min_Output,
		float Max_Output) {

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output)
			/ (Max_Input - Min_Input) + Min_Output);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
		cntt++;
		while (cntt - 100 > 0) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			cntt = 0;
		}


	if (RxHeader.StdId == 0x407) {
		RxData1 = (RxData[0] << 8) | RxData[1];
		RxData2 = (RxData[2] << 8) | RxData[3];
		RxData3 = (RxData[4] << 8) | RxData[5];
		RxData4 = (RxData[6] << 8) | RxData[7];
		V1 = RxData1;
		V2 = RxData2;
		V3 = RxData3;
		V4 = RxData4;

		Vx = map(V1, 0, 65535, -3.0, 3.0);
		Vy = map(V2, 0, 65535, -3.0, 3.0);
		Vz = map(V3, 0, 65535, -3.14, 3.14);
		speed = map(V4, 0, 65535, 0.0, 2.0);
		flag = 1;

	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PID_Init(&MPID, 2);
	MPID.T = 0.01; // T = 10ms
	MPID.limMax = 800;
	MPID.limMin = -800;
	MPID.limMaxInt = 700;
	MPID.limMinInt = -700;
	MPID.tau = 0; // for Kd
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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);

	// TIMER PWM Generation
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	// TIMER Encoder Mode
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	// CAN Transmit
	HAL_CAN_Start(&hcan);

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x215; //Slave1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {

		//Inverskinematic of Mecanum
		v1 = ((1 / R) * (Vx - Vy - (Vz * (lx + ly))));
		v2 = ((1 / R) * (Vx + Vy + (Vz * (lx + ly))));
		v3 = ((1 / R) * (Vx + Vy - (Vz * (lx + ly))));
		v4 = ((1 / R) * (Vx - Vy + (Vz * (lx + ly))));

		// Motor measurement speed
		Motor1_speed = Motors_RPS(Motor1, Sample_time, CPR); // sample time 10ms
		Motor2_speed = Motors_RPS(Motor2, Sample_time, CPR); // sample time 10ms
//
//		// PWM PID of Motor
		pwm_M1 = PID(&MPID, v1, Motor1_speed, kp, ki, kd, Motor1);
		pwm_M2 = PID(&MPID, v2, Motor2_speed, kp, ki, kd, Motor2);
//
//		// Control Motor
		if (pwm_M1 > 30)
		{
			TIM1->CCR4 = 0;
			TIM1->CCR3 = pwm_M1;
		}
		else if (pwm_M1 < -30)
		{
			TIM1->CCR4 = -1 * pwm_M1;
			TIM1->CCR3 = 0;
		}
		else
		{
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 0;
		}
		if (pwm_M2 > 30)
		{
			TIM1->CCR2 = pwm_M2;
			TIM1->CCR1 = 0;
		}
		else if (pwm_M2 < -30)
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = -1 * pwm_M2;
		}
		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
		}

		V1_out = map(Motor1_speed, -70, 70, 0, 65535);
		V2_out = map(Motor2_speed, -70, 70, 0, 65535);
//		v3_out = map(v3, 0, 600, 0, 65535);
//		v4_out = map(v4, 0, 600, 0, 65535);

		TxData[0] = ((V1_out & 0xFF00) >> 8);
		TxData[1] = (V1_out & 0x00FF);
		TxData[2] = ((V2_out & 0xFF00) >> 8);
		TxData[3] = (V2_out & 0x00FF);
		if (flag == 1)
		{
			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			flag = 0;
		}
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
