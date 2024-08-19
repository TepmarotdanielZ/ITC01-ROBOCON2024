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
#include "PID_Position.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDPosition pid;
PIDController MPID;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14159
#define r_1 0.026
#define r_2 0.005
#define Sample_time 10 //ms
#define CPR_1 16200
#define CPR_2 67

#define kp_1 1.0703  // PID RA GIM
#define ki_1 387.2466
#define kd_1 0

#define Kp 10.0  // PID Position
#define Ki 0.0001
#define Kd 0.008

#define kp_2 0.03072 // Bro RA
#define ki_2 0.7483
#define kd_2 0

#define Motor1 0
#define Motor2 1
#define X 0
#define dt 0.01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxData[8];
uint8_t TxData[4];
uint32_t TxMailbox;
uint16_t ReadValue;
volatile int flag = 0;
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
uint8_t collect;
float pwm_M1 = 0;
float pwm_M2 = 0;
float V;
float theta = 0.0;

//** Calculate Postion Motor
float V_standard;
float X_pos = 0.0;
float Vx;
float Output_Vx;
float X_end;
float X_end2;
int c;
uint8_t PushRice = 0;
uint8_t DropRice = 0;
uint8_t TakeRice = 0;
uint8_t state = 0;
uint8_t Push1;
uint8_t Push2;
int T;
uint16_t t;
int pwm;
float x;
float D;
uint16_t curre_time;
uint16_t prev_time;
uint8_t Position;
uint8_t State;
uint8_t Shooter;
float Push_manual;
uint16_t Adc_1;
uint16_t Adc_2;
float LaserX;
float LaserY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint32_t delay) {
    uint32_t start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < delay) {
        // You can add code here to perform other tasks while waiting
    }
}
//void delay_us(uint16_t au16_us)
//{
//    htim3.Instance->CNT = 0;
//    while (htim3.Instance->CNT < au16_us);
//}
//void delay_ms(uint16_t au16_ms)
//{
//    while(au16_ms > 0)
//    {
//    htim3.Instance->CNT = 0;
//    au16_ms--;
//    while (htim3.Instance->CNT < 1000);
//    }
//}
float Motors_RPS( int j,float SampleTime, float N_round)
{

	new_count[Motor1] = TIM3->CNT;
	new_count[Motor2] = TIM4->CNT;

	count_state[Motor1] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	count_state[Motor2] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
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
	if (RxHeader.StdId == 0x102)
	{
		RxData1 = (RxData[0] << 8) | RxData[1];
		X_end = map(RxData1, 0, 65535, -10.0, 10.0);
		if (X_end <= 0.001 && X_end >= -0.001)
		{
			X_end = 0.0;
		}
		PushRice = RxData[2];
		DropRice = RxData[4];
		Push_manual = map(RxData[5], 0, 255, -1.0, 1.0);
		Shooter = RxData[6];
//		V2 = 300*Shooter;
		flag = 1;
	}
	else if (RxHeader.StdId == 0x407) {
		Adc_1 = (RxData[0] << 8) | RxData[1];
		Adc_2 = (RxData[2] << 8) | RxData[3];
		LaserX = ((Adc_1 *0.01416) + 0.1963)/10; //aready
		LaserY = ((Adc_2 *0.01289) + 0.3364)/10;

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
	//PID_Position
//	PID_Position_Init(&pid, 1);
//	pid.limMax = 3.0;
//	pid.limMin = -3.0;
//	pid.limMaxInt = 3.0;
//	pid.limMinInt = -3.0;
//	pid.T = dt;
//	pid.alpha = 0.8;

	// PID Motor
	PID_Init(&MPID, 2);
	MPID.T = 0.01; // T = 10ms
	MPID.limMax = 1000;
	MPID.limMin = -1000;
	MPID.limMaxInt = 1000;
	MPID.limMinInt = -1000;
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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  // CAN Transmit
  	HAL_CAN_Start(&hcan);
  	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  	TxHeader.DLC = 8;
  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.RTR = CAN_RTR_DATA;
  	TxHeader.StdId = 0x213; //Slave1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//     HAL_Delay(150);

//	  T = (2.434*D - 0.0037)*1000;
//			if (X_end > 0.03 && (DropRice == 1 || PushRice == 1) ){
//				T = (2.434*X_end - 0.0037)*1000;
//				if (T == -3) {
//					T = 0;
//				}
//				pwm_M1 = 1000;
//				delay_ms(T);
//				pwm_M1 = 0;
//			}else if (X_end < 0.03 && (DropRice == 1 || PushRice == 1)) {
//				T = (-2.434*X_end - 0.0037)*1000;
//				if (T == -3) {
//					T = 0;
//				}
//				pwm_M1 = -1000;
//				delay_ms(T);
//				pwm_M1 = 0;
//			}
//			else if (Push_manual >=0.5) {
//				pwm_M1 = 700;
//					} else if (Push_manual <= -0.5) {
//						pwm_M1 = -1000;
//					} else {
//						pwm_M1 = 0;
//					}
//			HAL_Delay(10);


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

//		V_standard = (0.0612 * Motor1_speed);
//		Vx = V_standard * r_1; /*Vx: Velocity of motor*/
//		Vx = Motor1_speed*r_1;
//		X_pos = X_pos + Vx * dt;
//		Output_Vx = PID_Position(&pid, X_end, X_pos, Kp, Ki, Kd, X);

		//Motor Speed
		Motor1_speed = Motors_RPS(Motor1, Sample_time, CPR_1); // sample time 10ms
		Motor2_speed = Motors_RPS(Motor2, Sample_time, CPR_2); // sample time 10ms

		//V2 = 167.9030 * distance + 157.34028;  // Equation distance to Speed
		// PWM PID of Motor
//		pwm_M1 = PID(&MPID, V1 , Motor1_speed, kp_1, ki_1, kd_1, Motor1);
		pwm_M2 = PID(&MPID, V2, Motor2_speed, kp_2, ki_2, kd_2, Motor2);

		// Control Motor
		if (pwm_M2 > 20) {
			TIM1->CCR4 = pwm_M2;
			TIM1->CCR3 = 0;
		} else if (pwm_M2 < -20) {
			TIM1->CCR4 = 0;
			TIM1->CCR3 = -1 * pwm_M2;
		} else {
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 0;
		}
		if (pwm_M1 > 20) {
			TIM1->CCR2 = pwm_M1;
			TIM1->CCR1 = 0;
		} else if (pwm_M1 < -20) {
			TIM1->CCR2 = 0;
			TIM1->CCR1 = -1 * pwm_M1;
		} else {
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
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
