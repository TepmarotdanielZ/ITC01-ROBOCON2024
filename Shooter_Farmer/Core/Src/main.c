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
#include "cmsis_os.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "Accel_stepper.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//PID Postion Motor
#define Kp_X 1.33
#define Ki_X 0.0001
#define Kd_X 0.004
#define X 0
#define dt 0.01

//PID Speed Motor RA GIM
#define R 0.006
#define CPR_1 4929
#define kp_1 0.043
#define ki_1 1.436
#define kd_1 0
#define Motor1 0
#define PI 3.14159

//Methode2: Rotation Head
#define SPR  800    //step(per revelution)
#define TIM_FREQ 1000000 // [us]
#define Alpha (2*PI/SPR)
#define gear_ratio 19

// Shooter DC motor
#define H 0.45
#define r 0.005
#define CPR_2 67  // Motor bro Ra
#define Sample_time 10 // ms
#define kp_2 0.03072 // Bro RA
#define ki_2 0.7483
#define kd_2 0
#define Motor2 1
#define m 0.2 //[kg]
#define g 9.81 // [m/s^2]
#define b 1.225 // Air resistance constant

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Head Rotate: Steper Motor
uint16_t stepDelay = 0; // 1000us more delay means less speed
signed int set_theta1 = 200*19;
int counter=0;
extern int degree;
int deg;
extern double omega ;  // rad/s
extern double head ;
int Step = 0;
int step;

// CAN
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;
int datacheck = 0;
uint8_t cntt;
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;

//Calculate position


// DC MOTOR control
float pwm1;
float V_shooter;
uint16_t V1_out;
uint16_t V2_out;

// Receive from ESPCAN
float Vx;
float Vy;
float Omega;
uint8_t Mode;
uint8_t Position;
uint8_t Catch;
uint8_t Shotter;
uint8_t Push_manual;
uint8_t SpeedShooter;
uint8_t Push ;
uint8_t Stepper;
uint8_t Drop;

//
float V_standard;
float X_mea;
float X_in;
float V1;
float Output_Vx;
extern float X_end;
extern uint8_t PushRice;
extern uint8_t DropRice;
uint8_t push1;
extern uint8_t push2;
// Drop rice
extern float x;
extern float y;
extern float yaw;
uint8_t take_state;
extern float X_end ;
extern float X_goal ;
extern float X_goal1;
int j;
uint8_t StateP;
uint16_t cur_time =0;
uint16_t prev_time=0;
//uint8_t state2 =0;

//uint8_t RxDataESP[13];
extern uint8_t SpeedShooter ;
extern uint8_t state;
//extern float distance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float map(float Input, float Min_Input, float Max_Input, float Min_Output,
		float Max_Output) {

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output)
			/ (Max_Input - Min_Input) + Min_Output);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
		cntt++;
		while (cntt - 100 > 0) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			cntt = 0;
		}


	if (RxHeader.StdId == 0x401)
	{
		Vx = map(RxData[0], 0, 255, -3.0 , 3.0);
		Vy = map(RxData[1], 0, 255, -3.0 , 3.0);
		Omega = map(RxData[2], 0, 255, -3.0 , 3.0);
		Mode = RxData[3];
		Position = RxData[4];
		Catch = RxData[5];
		Push = RxData[6];
		Drop = RxData[7];

		if (Position == 11)
		{
			Stepper = RxData[6];
			Shotter = RxData[7];
		}

		datacheck = 1;
	}
	else if (RxHeader.StdId == 0x407) {
		StateP = RxData[6];

	}
}

//void microDelay (uint16_t delay)
//{
//
//  __HAL_TIM_SET_COUNTER(&htim8, 0);
//  while (__HAL_TIM_GET_COUNTER(&htim8) < delay);
//  //HAL_TIM_Base_Stop_IT(&htim12);
//}




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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  /*** Configure TIMER ***/
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim12);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /*** Configure CAN ***/
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x409;

  /** Configure ADC ***/
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM12){
//
		if (SpeedShooter >= 1)
		{
			pwm1 = 700;
		}
		else if (SpeedShooter < 1)
		{
			pwm1 = 0;
		}
		// Control Motor
		if (pwm1 > 10) {
			TIM3->CCR1 = 0;
			TIM3->CCR2 = pwm1;
		} else if (pwm1 < -10) {
			TIM3->CCR1 = -1 * pwm1;
			TIM3->CCR2 = 0;
		} else {
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
		}
		V1_out = map(X_end*10, -100.0, 100.0, 0, 65535);
		V2_out = map(SpeedShooter*100, 0.0, 1000.0, 0, 65535);
		TxData[0] = ((V1_out & 0xFF00) >> 8);
		TxData[1] = (V1_out & 0x00FF);
		TxData[2] = ((V2_out & 0xFF00) >> 8);
		TxData[3] = (V2_out & 0x00FF);
		TxData[4] = Push;
		TxData[5] = Drop;
//		TxData[6] = TakeRice;
//		TxData[7] = push2;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {  //stepper1

		if (omega != 0) {
			stepDelay = (Alpha * TIM_FREQ) / (omega);
			TIM2->ARR = stepDelay;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

		} else {
			TIM2->ARR = 10000;
		}
	}
  if (htim->Instance == TIM5)
  {
		if (Position == 1  ) {
			X_end = 0.45; // store 2 rice
//			X_end = -0.66;
		} else if (Position == 2  ) {
			X_end = 0.28; // store 4 rice
//			X_end = -0.44;
		} else if (Position == 3 ) {
			X_end = 0.09; // store 6 rice
//			X_end = -0.38;
		}
		else if (Position >=4)
		{
			X_end = 0.1;
		}
//		if (Push == 1) {
//			X_end = X_goal;
//		}
		j++;
  }
  /* USER CODE END Callback 1 */
}

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
