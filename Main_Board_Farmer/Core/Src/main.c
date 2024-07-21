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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_stm32.h"
#include "bno055.h"
#include "IIRFilter.h"
#include "math.h"
#include "PID_Position.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDPosition pid;
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
IIRFilter IR;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1456
#define r 0.03 //[Radius m]
#define ry 0.02
#define D 2*r //[Diameter m]
#define CPR_X 1440
#define CPR_Y 1440
#define Rotary1 0
#define Rotary2 1
#define DEBOUNCE_DELAY 400
#define sampling_time  10 //[10ms]
#define dt 0.01
#define Alpha 0.9f
#define Kp_X 1.4
#define Ki_X 0.0001
#define Kd_X 0.004

#define Kp_Y 2.0
#define Ki_Y 0.0001
#define Kd_Y 0.004

#define Kp_Yaw 1.4
#define Ki_Yaw 0.0001
#define Kd_Yaw 0.004

#define X 0
#define Y 1
#define YAW 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;


uint8_t TxData[8];
uint32_t TxMailbox;
uint16_t V1_out = 0;
uint16_t V2_out = 0;
uint16_t V3_out = 0;
uint16_t V4_out = 0;
uint8_t RxDataESP[13];
uint8_t RxData[8];
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float V3 = 0; // target speed of motor1
float V4 = 0; // target speed of motor2

uint8_t datacheck = 0;
uint8_t cntt;
float V_back;
float V1_back;
float V2_back;
float V3_back;
float V4_back;
//NRF
float Speed;
//Varible to read Encoder
float X_old_enR;
float Y_old_enR;
float Yaw_old_IMU;
float yaw_old_enR;
float X_enR = -0.0;
float Y_enR = 0.0;
float Yaw_IMU = 0.0;
float yaw_enR;
float v;
float W1;
float W2;
float Vx_enR;
float Vy_enR;
float thetaY;

// IMU
float theta;
double sinr_cosp;
double cosr_cosp;
double sinp;
double siny_cosp;
double cosy_cosp;
double angle; // Angle in degrees

float Vx;
float Vy;
float Omega;
float VxM;
float VyM;
float OmegaM;
float vx;
float vy;
float omega;


int j;
int k;
int b;
int p;
uint8_t s;
uint8_t c;
uint8_t d;
uint8_t DropRice;
uint8_t State;
uint8_t Mode;
uint8_t stop;
uint8_t Position;
float Output_max = 0.6;
float Integral_max = 0.6;
float Output_Vx;
float Output_Vy;
float Output_Omega;
float errorX = 0.0;
float errorY = 0.0;
float errorYaw = 0.0;
float goal[3];
float goal1[] = {0.45,0.0,0.0};
float goal2[] = {0.45,-1.38,0.0};
float goal3[] = {0.25,-1.38,0.0};
float goal4[] = {0.25,-2.38,0.0};
float goal5[] = {0.25,-3.38,0.0};
float goal6[] = {2.25,-3.35, 0.0};
float goal7[] = {2.25,-3.23,-1.57};
uint8_t espcan1;

// laser
uint16_t AD_RES[2];
float dcm_X;
float dcm_Y;
double laserx;
double lasery;
double target_laserx[] = {3.25, 3.35, 3.45, 1.071}; //m
double target_lasery[] = {2.2, 3.1, 3.99, 1.2, 1.63, 2.13, 2.60, 3.10, 3.58}; //m
double errorlaserx;
double errorlasery;
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
	if (RxHeader.StdId == 0x401)
		{
			VxM = map(RxData[0], 0, 255, -3.0 , 3.0);
			VyM = map(RxData[1], 0, 255, -3.0 , 3.0);
			OmegaM = map(RxData[2], 0, 255, -3.0 , 3.0);
			Mode = RxData[3];
			Position = RxData[4];
		if (VxM >= -0.05 && VxM <= 0.05) {
			VxM = 0.0;
		}
		if (VyM >= -0.05 && VyM <= 0.05) {
			VyM = 0.0;
		}
		if (OmegaM >= -0.05 && OmegaM <= 0.05) {
			OmegaM = 0.0;
		}
//			Catch = RxData[5];
//			Push = RxData[6];
//			Drop = RxData[7];
//
//			if (Position == 11)
//			{
//				Stepper = RxData[6];
//				Shotter = RxData[7];
//			}
//			else
//			{
//				Push = 0;
//				Drop = 0;
//			}
		}
	else if (RxHeader.StdId == 0x215) {
			RxData1 = (RxData[0] << 8) | RxData[1];
			RxData2 = (RxData[2] << 8) | RxData[3];
			V1_back = map(RxData1, 0, 65535, -70.0, 70.0);
			V2_back = map(RxData2, 0, 65535, -70.0, 70.0);
			if (V1_back >= 0.001 && V1_back <= -0.001)
			{
				V1_back = 0.0;
			}
			if (V2_back >= 0.001 && V2_back <= -0.001) {
				V2_back = 0.0;
			}
			datacheck = 1;
	}
	else if (RxHeader.StdId == 0x211) {
			RxData3 = (RxData[0] << 8) | RxData[1];
			RxData4 = (RxData[2] << 8) | RxData[3];
			V3_back = map(RxData3, 0, 65535, -70.0, 70.0);
			V4_back = map(RxData4, 0, 65535, -70.0, 70.0);
			if (V3_back >= 0.001 && V3_back <= -0.001)
			{
			V3_back = 0.0;
		}
		if (V4_back >= 0.001 && V4_back <= -0.001) {
			V4_back = 0.0;
		}
			datacheck = 1;
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
  PID_Position_Init(&pid, 3);
  pid.limMax = Output_max;
  pid.limMin = -Output_max;
  pid.limMaxInt = Integral_max;
  pid.limMinInt = -Integral_max;
  pid.T = dt;
  pid.alpha = 0.8;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  // CAN _Transmition
  	HAL_CAN_Start(&hcan1);
  	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  	TxHeader.DLC = 7; // data length
  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.RTR = CAN_RTR_DATA;
  	TxHeader.StdId = 0x407; //Id 0x7FF

  	// TIMER Internal clock
  	HAL_TIM_Base_Start_IT(&htim3);
  	HAL_TIM_Base_Start_IT(&htim12);
  	HAL_TIM_Base_Start_IT(&htim5);

  	// IMU
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	// Read Rotary encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	// UART
//	HAL_UART_Receive_IT(&huart3, RxDataESP, 13);
	// IRFilter
//	IIRFilter_Init(&IR, Alpha);
//  	NVIC_SystemReset();
//  	HAL_NVIC_SystemReset();

	// ADC
//	HAL_ADCEx_Calibration_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  read_encoder(&encoderX, &htim2, CPR_X);
//		read_encoder(&encoderY, &htim4, CPR_Y);
//		W1 = (double) encoderX.rdps * r;
//		W2 = (double) encoderY.rdps * r;
//		Q = bno055_getVectorQuaternion();
//		// yaw (z-axis rotation)
//		siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
//		cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
//		Angle.Yaw = atan2(siny_cosp, cosy_cosp);
//		//		theta = Angle.Yaw; // radians]
//		theta = Angle.Yaw;
//		Vx_enR = W1 * cosf(theta) - W2 * sinf(theta);
//		Vy_enR = W1 * sinf(theta) + W2 * cosf(theta);
//		X_enR = X_enR + Vx_enR * dt;
//		Y_enR = Y_enR + Vy_enR * dt;
//		HAL_ADC_Start_DMA(&hadc1, &AD_RES, 2);
//		laserx = ((AD_RES[1] *0.01416) + 0.1963)/10; //aready
//	  	lasery = ((AD_RES[0] *0.01289) + 0.3364)/10;
//
//	  	HAL_Delay(10);

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
	if (htim->Instance == TIM3)
	{
		//PID POSITION

		Output_Vx = PID_Position(&pid, goal[0], X_enR, Kp_X, Ki_X, Kd_X, X);
		Output_Vy = PID_Position(&pid, goal[1], Y_enR, Kp_Y, Ki_Y, Kd_Y, Y);
		Output_Omega = PID_Position(&pid, goal[2], theta, Kp_Yaw, Ki_Yaw, Kd_Yaw, YAW);
		vx = Output_Vx*cosf(theta) + Output_Vy*sinf(theta);
		vy = -Output_Vx*sinf(theta) + Output_Vy*cosf(theta);
		omega = Output_Omega;
		if (Mode == 1)
		{
			Vx = vx;
			Vy = vy;
			Omega = omega;
		}
		else if (Mode == 0)
		{
			Vx = VxM;
			Vy = VyM;
			Omega = OmegaM;
		}
		V1_out = map(Vx, -3.0, 3.0, 0, 65535);
		V2_out = map(Vy, -3.0, 3.0, 0, 65535);
		V3_out = map(Omega, -3.14, 3.14, 0, 65535);
		TxData[0] = ((V1_out & 0xFF00) >> 8);
		TxData[1] = (V1_out & 0x00FF);
		TxData[2] = ((V2_out & 0xFF00) >> 8);
		TxData[3] = (V2_out & 0x00FF);
		TxData[4] = ((V3_out & 0xFF00) >> 8);
		TxData[5] = (V3_out & 0x00FF);
		TxData[6] = State;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

	}
	if (htim->Instance == TIM5)
	{
		errorX = goal[0] - X_enR;
		errorY = goal[1] - Y_enR;
		errorYaw = goal[2] - theta;
		if (Position == 1 && State == 0) {
			goal[0] = goal1[0];
			goal[1] = goal1[1];
			goal[2] = goal1[2];
			State = 1;
		} else if (abs(errorX) < 0.05 && State == 1) {
			goal[0] = goal2[0];
			goal[1] = goal2[1];
			goal[2] = goal2[2];
			State = 2;
		} else if (abs(errorY) < 0.05 && State == 2) {
			errorlaserx = -target_laserx[0] + laserx;
			errorlasery = -target_lasery[0] + lasery;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = goal3[2];
			if (abs(errorlaserx)< 0.05 && abs(errorlasery)<0.05)
			{
				State = 3;
			}
		}
		else if (Position == 2 && State == 3) {
			errorlaserx = -target_laserx[1] + laserx;
			errorlasery = -target_lasery[1] + lasery;
			goal[0] = errorlaserx + X_enR;
			goal[1] = errorlasery + Y_enR;
			goal[2] = goal4[2];
			if (abs(errorlaserx)< 0.05 && abs(errorlasery)<0.05)
			{
				State = 4;
			}
		}
		else if (Position == 3 && State == 4) {
			errorlaserx = -target_laserx[2] + laserx;
			errorlasery = -target_lasery[2] + lasery;
			goal[0] = errorlaserx + X_enR;
			goal[1] = errorlasery + Y_enR;
			goal[2] = goal5[2];
			if (abs(errorlaserx) < 0.05 && abs(errorlasery) < 0.05) {
				State = 5;
			}
		}
		else if (Position == 4 && State == 5) {
			goal[0] = goal6[0];
			goal[1] = goal6[1];
			goal[2] = goal6[2];
			State = 6;
		} else if (abs(errorY) < 0.05 && State == 6) {
			goal[0] = goal6[0];
			goal[1] = goal6[1];
			goal[2] = goal7[2];
			State = 7;
		}
		else if (Position == 5 && State == 7) {
			errorlaserx = -target_laserx[3] + lasery;
			errorlasery = target_lasery[3] - laserx;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = -1.57;
			if (abs(errorlaserx) < 0.05 && abs(errorlasery) < 0.05 && State == 7) {
				State = 8;
			}
		}
		else if (Position == 6 && State == 8) {
			errorlaserx = -target_laserx[3] + lasery;
			errorlasery = target_lasery[4] - laserx;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = -1.57;
			if (abs(errorlaserx) < 0.05 && abs(errorlasery) < 0.05 && State == 8) {
					State = 9;
			}
		}
		else if (Position == 7 && State == 9 ) {
			errorlaserx = -target_laserx[3] + lasery;
			errorlasery = target_lasery[5] - laserx;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = -1.57;
			if (abs(errorlaserx) <= 0.05 && abs(errorlasery) <= 0.05 && State == 9) {
					State = 10;
			}
		}
		else if (Position == 8 && State == 10 ) {
			errorlaserx = -target_laserx[3] + lasery;
			errorlasery = target_lasery[6] - laserx;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = -1.57;
			if (abs(errorlaserx) <= 0.05 && abs(errorlasery) <= 0.05 && State == 10) {
					State = 11;
			}
		}
		else if (Position == 9 && State == 11 ) {
			errorlaserx = -target_laserx[3] + lasery;
			errorlasery = target_lasery[7] - laserx;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = -1.57;
			if (abs(errorlaserx) <= 0.03 && abs(errorlasery) <= 0.03 && State == 11) {
					State = 12;
			}
		}
		else if (Position == 10 && State == 12 ) {
			errorlaserx = -target_laserx[3] + lasery;
			errorlasery = target_lasery[8] - laserx;
			goal[0] = errorlaserx+X_enR;
			goal[1] = errorlasery+Y_enR;
			goal[2] = -1.57;
			if (abs(errorlaserx) <= 0.05 && abs(errorlasery) <= 0.05 && State == 12) {
					State = 13;
			}
		}
	}
	if (htim->Instance == TIM12)
	{
		read_encoder(&encoderX, &htim2, CPR_X);
	    read_encoder(&encoderY, &htim4, CPR_Y);
	    W1 = (double) encoderX.rdps * r;
	    W2 = (double) encoderY.rdps * r;
	    Q = bno055_getVectorQuaternion();
		// yaw (z-axis rotation)
		siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
		cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
		Angle.Yaw = atan2(siny_cosp, cosy_cosp);
		//		theta = Angle.Yaw; // radians]
		theta = Angle.Yaw;
		Vx_enR = W1*cosf(theta) - W2*sinf(theta);
		Vy_enR = W1*sinf(theta) + W2*cosf(theta);
		X_enR = X_enR + Vx_enR*dt;
		Y_enR = Y_enR + Vy_enR*dt;
		HAL_ADC_Start_DMA(&hadc1, &AD_RES, 2);
		laserx = ((AD_RES[1] *0.01416) + 0.1963)/10; //aready
		lasery = ((AD_RES[0] *0.01289) + 0.3364)/10;
		p++;

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
