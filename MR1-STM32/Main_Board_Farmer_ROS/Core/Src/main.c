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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "bno055_stm32.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
typedef struct{
	uint32_t counter;
	uint32_t new_counter;
	uint8_t counter_status;
	float speed;
	float rdps;
	double distant;
}Encoder;
typedef struct {
	double Roll;
	double Pitch;
	double Yaw;
}Rotation;

Encoder encoderX;
Encoder encoderY;
Rotation Angle;
bno055_vector_t Q;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1456
#define r 0.03 //[Radius m]
#define CPR_X 1440
#define CPR_Y 1440
#define Rotary1 0
#define Rotary2 1
#define sampling_time  10 //[10ms]
#define dt 0.01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//** CAN VARIABLE ***//
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
uint8_t datacheck = 0;
uint8_t cntt;

// SENSOR FEEDBACK CAN
uint16_t AD_RES[2];
uint8_t laserX;
uint8_t laserY;
uint16_t rotaryX;
uint16_t rotaryY;
uint16_t imu;

// SENSOR FEEBACK READ
float W1;
float W2;
float theta;
double sinr_cosp;
double cosr_cosp;
double sinp;
double siny_cosp;
double cosy_cosp;
double angle;
float LaserX;
float LaserY;
float motor_air;
uint8_t catch;
uint8_t Position;
uint8_t state;
uint8_t state1;
uint8_t state2;
uint8_t state3;
uint16_t air1;
uint16_t air2;
uint16_t ari3;
// CHECK TIMER INTERRUPT WORK OR NOT
int c;
int j;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}
void read_encoder(Encoder *enc, TIM_HandleTypeDef* timer, float CPR){
	enc->new_counter = __HAL_TIM_GET_COUNTER(timer);
	enc->counter_status = __HAL_TIM_IS_TIM_COUNTING_DOWN(timer);
	int16_t count_change = enc->new_counter - enc->counter;
	if(enc->counter_status && count_change <0){
		count_change += 65536;
	}else if (!enc->counter_status && count_change > 0){
		count_change -= 65536;
	}
	enc->counter = enc->new_counter;
	enc->counter_status = (count_change >=0);
	enc->speed = (float)count_change*1000.0f/(CPR_X * sampling_time);
	enc->rdps = (float)count_change*2*PI*1000.0f/(CPR_X * sampling_time);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		cntt = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //*** CAN CONFIGURE ***//
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	TxHeader.DLC = 8; // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x407; //Id 0x7FF

	// TIMER Internal clock
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	// IMU
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	// Read Rotary encoder
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (Position ==1 && state == 0)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1); //Push
		  HAL_Delay(1000);
		  state = 1;
	  }
	  else if (state== 1){ // Motor air
		  motor_air = 500;
		  HAL_Delay(500);
		  state = 2;
	  }
	  else if (state == 2) // Air slide
	  {
		  motor_air = 0;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		  HAL_Delay(1000);
		  state = 3;
	  }
	  else if (state == 3) // griper
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
		  HAL_Delay(1000);
		  state = 4;
	  }
	  else if (catch == 1 && state == 4)
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
		  HAL_Delay(1000);
		  state = 5;
	  }
	  if(state>)
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, state);
	  HAL_Delay(air1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);

	  if (Position ==1 && state == 0)
	 	  {
	 		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1); //Push
	 		  HAL_Delay(1000);
//	 		  state = 1;
	 	  }
	 	  else if (state== 1){ // Motor air
	 		  motor_air = 500;
	 		  HAL_Delay(500);
	 		  state = 2;
	 	  }
	 	  else if (state == 2) // Air slide
	 	  {
	 		  motor_air = 0;
	 		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	 		  HAL_Delay(1000);
	 		  state = 3;
	 	  }
	 	  else if (state == 3) // griper
	 	  {
	 		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
	 		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
	 		  HAL_Delay(1000);
	 		  state = 4;
	 	  }
	 	  else if (catch == 1 && state == 4)
	 	  {
	 		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
	 		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
	 		  HAL_Delay(1000);
	 		  state = 5;
	 	  }


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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
		{
//			read_encoder(&encoderX, &htim2, CPR_X);
//			read_encoder(&encoderY, &htim4, CPR_Y);
//			W1 = (double) encoderX.rdps * r;
//			W2 = (double) encoderY.rdps * r;
		if (motor_air > 10){
					TIM1->CCR1 = motor_air;
					TIM1->CCR2 = 0;
				}
				else if  (motor_air < -10)
				{
					TIM1->CCR1 = 0;
					TIM1->CCR2 = -1*motor_air;
				}
				else {
					TIM1->CCR1 = 0;
					TIM1->CCR2 = 0;
				}
			Q = bno055_getVectorQuaternion();
			// yaw (z-axis rotation)
			siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
			cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
			Angle.Yaw = atan2(siny_cosp, cosy_cosp);
			theta = Angle.Yaw;
			HAL_ADC_Start_DMA(&hadc1, &AD_RES, 2);
//			LaserX = ((AD_RES[1] *0.01416) + 0.1963)/10; //aready
//			LaserY = ((AD_RES[0] *0.01289) + 0.3364)/10;
			j++;
		}
	if (htim->Instance == TIM3)
	{
		imu = map(theta, -3.14159, 3.14159, 0, 65535);
		TxData[0] = ((AD_RES[1] & 0xFF00) >> 8);
		TxData[1] = (AD_RES[1] & 0x00FF);
		TxData[2] = ((AD_RES[2] & 0xFF00) >> 8);
		TxData[3] = (AD_RES[2] & 0x00FF);
		TxData[4] = ((imu & 0xFF00) >> 8);
		TxData[5] = (imu & 0x00FF);


		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		c++;
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
