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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf.h"
#include "JOYSTICK.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define JoyStick1  0
#define JoyStick2  1
#define DEBOUNCE_DELAY 400
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile extern int flag;
volatile int flag = 0 ;

//------------------------------uart-functions--------------------------------------
uint64_t RxpipeAddrs = 0x22223344BB;
const void* myAckPayload ;
int input_num = 0 ;
char input_data[32] ;
char c1;
char* c = &c1;
char c2;
char* c3 = &c2;
uint64_t TxpipeAddrs = 0x11223344BB;
char myTxData[32];
char AckPayload[32];
char myRxData[50];
uint16_t JoyStick1_XY[2]={0};
uint16_t JoyStick2_XY[2]={0};
int X1;
int X2;
int X3;
int X4;
int X5;
int X6;
int XL;
int YL;
int XR;
int YR;
uint8_t button = 0;
uint8_t mode = 0;
uint8_t V;
uint8_t Omega;
uint16_t potentiometre;
uint8_t Vx;
uint8_t Vy;
uint8_t Vz;
uint16_t ADC_Data=0;
float Speed = 0.0;
uint16_t read = 0;
int degree;
int deg;
int Deg;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t previousmillis = 0;
uint8_t counter;
uint8_t button1_state = GPIO_PIN_RESET;
uint8_t button2_state = GPIO_PIN_RESET;
uint8_t button3_state = GPIO_PIN_RESET;
uint8_t button4_state = GPIO_PIN_RESET;
uint8_t button5_state = GPIO_PIN_RESET;
uint8_t button6_state = GPIO_PIN_RESET;
uint8_t button7_state = GPIO_PIN_RESET;
uint8_t button8_state = GPIO_PIN_RESET;

uint8_t button1_last_state = GPIO_PIN_RESET;
uint8_t button2_last_state = GPIO_PIN_RESET;
uint8_t button3_last_state = GPIO_PIN_RESET;
uint8_t button4_last_state = GPIO_PIN_RESET;
uint8_t button5_last_state = GPIO_PIN_RESET;
uint8_t button6_last_state = GPIO_PIN_RESET;
uint8_t button7_last_state = GPIO_PIN_RESET;
uint8_t button8_last_state = GPIO_PIN_RESET;
int Auto;
int Menu;
int TakeRice;
int DropRice;
int Shooter;
int Stepper;
int SMR;
int PushRice;
int SpeedShooter;
uint8_t Pick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
long Map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

float map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

//void ReceiveMode(void){
//
//	if (NRF24_available()) {
//		NRF24_read(myRxData, 32);
//
//		NRF24_writeAckPayload(1, myAckPayload, 32);
//		myRxData[32] = '\r';
//		myRxData[32 + 1] = '\n';
//
//	}
//}

void TransferMode(void){
	//-----------------------------Tx-setting----------------------------
	//get_input();
	NRF24_stopListening();
	NRF24_openWritingPipe(TxpipeAddrs);
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_enableAckPayload();

	if (NRF24_write(myTxData, 32)) {
		NRF24_read(AckPayload, 32);
		//print("Transmitted Successfully\r\n");

		char myDataack[80];
		sprintf(myDataack, "AckPayload:  %s \r\n", AckPayload);
		//print(myDataack);
		flag = 0;
	}
}

void customDelay(uint32_t milliseconds) {
    uint32_t currentTick = HAL_GetTick();
    while ((HAL_GetTick() - currentTick) < milliseconds) {
        // Wait for the specified number of milliseconds to elapse
    }
}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	speed = map(ADC_Data, 0, 4095, 0, 6);
//
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //-----------------------------nrf-startup----------------------------------
    NRF24_begin(GPIOA, GPIOB, GPIO_PIN_4, GPIO_PIN_0, hspi1);
//    //-----------------------------Rx-setting----------------------------------
//      	NRF24_setAutoAck(true);
//      	NRF24_setChannel(52);
//      	NRF24_setPayloadSize(32);
//      	NRF24_openReadingPipe(1, RxpipeAddrs);
//      	NRF24_enableDynamicPayloads();
//      	NRF24_enableAckPayload();
//      	NRF24_startListening();
  // Joystick
    JoyStick_Init(JoyStick1);
    JoyStick_Init(JoyStick2);
   // TiM 10ms
    HAL_TIM_Base_Start_IT(&htim2);
  // Potentiometre ADC
    //
   /** Calibrate The ADC On Power-Up For Better Accuracy **/
//      HAL_ADCEx_Calibration_Start(&hadc1);
       //**Start ADC Conversion **/
//      HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //********* Manual ***********//
	  //** Potentiometer **//
//	  HAL_ADC_PollForConversion(&hadc1,2);
//	  read = HAL_ADC_GetValue(&hadc1);
//	  deg = map(read,0,4096,0,255);
	  //*** Joystick ***/
	  JoyStick_Read(JoyStick1, JoyStick1_XY);
	  JoyStick_Read(JoyStick2, JoyStick2_XY);
		XR = map(JoyStick1_XY[0], 0, 4096, -10, 10);
		YR = map(JoyStick1_XY[1], 0, 4096, -10, 10);
		XL = map(JoyStick2_XY[0], 0, 4096, -10, 10);
		YL = map(JoyStick2_XY[1], 0, 4096, -10, 10);

		if (XL >= 9 && YL == 0) {
			Vx = 20;
			Vy = 0;

		} else if (XL <= -9 && YL == 0) //Linear Velocity -X
				{
			Vx = 10;
			Vy = 0;

		} else if (YL >= 9 && XL == 0) //Angular Velocity +Y
				{
			Vx = 0;
			Vy = 20;

		} else if (YL <= -9 && XL == 0) //Angular Velocity -Y
				{
			Vx = 0;
			Vy = 10;

		} else if (XL >= 9 && YL >= 9) //Rotate Forward wheel_L
				{
			Vx = 20;
			Vy = 20;

		} else if (XL >= 9 && YL <= -9) //Rotate Forward wheel_R
				{
			Vx = 20;
			Vy = 10;

		} else if (XL <= -9 && YL <= -9) //Rotate Backward  Wheel_R
				{
			Vx = 10;
			Vy = 10;
		} else if (XL <= -9 && YL >= 9) //Rotate Backward Wheel_L
				{
			Vx = 10;
			Vy = 20;
		} else if ((XL < 1 && XL > -1) && (YL < 1 && YL > -1)) //Stop
				{
			Vx = 0;
			Vy = 0;
		}

		if (YR >= 9)
		{
			Vz = 20;
		}
		else if (YR <= -9)
		{
			Vz = 10;
		}
		else if (YR == 0)
		{
			Vz = 0;
		}

		//******** Auto ***********//
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))
				{
			Stepper = 10; // goal 1
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)) {
			Stepper = 20; // goal 2
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)){
			PushRice = 10;
		}
		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
			PushRice = 20;
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
		{
			Pick = 1;
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
		{
			DropRice = 1;
		}
		else
		{
			Stepper = 0;
			PushRice = 0;
			Pick = 0;
			DropRice = 0;
		}
		myTxData[0] = Vx;
		myTxData[1] = Vy;
		myTxData[2] = Vz;
		myTxData[3] = Speed;
		myTxData[4] = Stepper;
		myTxData[5] = PushRice;
		myTxData[6] = mode;
		myTxData[7] = Shooter;
		myTxData[8] = DropRice;
		myTxData[9] = TakeRice;
		myTxData[10] = SpeedShooter;
		myTxData[11] = Pick;


		/* Transmit message*/
		TransferMode();
		HAL_Delay(20); //[ms]
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	currentMillis = HAL_GetTick();
//	if (GPIO_Pin != GPIO_PIN_6 && currentMillis - previousMillis > DEBOUNCE_DELAY)
//	{
//		Speed++;
//		if (Speed > 6)
//		{
//			Speed = 0;
//		}
//		previousMillis = currentMillis;
//	}
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {

    /* BUTTON 1 (PB3) MODE AUTO = 1, MENU = 0 */

      button1_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
      if(button1_state == GPIO_PIN_RESET && button1_last_state == GPIO_PIN_SET) {
        mode++;
        if (mode > 1){
          mode = 0; /* Auto = 1 CHANGE MODE MENU */
        }
      }
      button1_last_state = button1_state;
    /* BUTTON 2  (PB4) DROP RICE */
//      button2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
//      if(button2_state == GPIO_PIN_RESET && button2_last_state == GPIO_PIN_SET) {
//        DropRice++;
//        if (DropRice > 1){
//          DropRice = 0;
//        }
//      }
      button2_last_state = button2_state;
    /* BUTTON 3 (PB5) TAKE RICE */
      button3_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
      if(button3_state == GPIO_PIN_RESET && button3_last_state == GPIO_PIN_SET) {
        TakeRice++;
        if (TakeRice > 2){
          TakeRice = 0;
        }
      }
      button3_last_state = button3_state;

    /* BUTTON 4 (PB8) COUNT SHOOTER */

      button4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
      if(button4_state == GPIO_PIN_RESET && button4_last_state == GPIO_PIN_SET) {
        Shooter++;
        if (Shooter > 6){
          Shooter = 0;
        }
      }
      button4_last_state = button4_state;

      button5_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
		if (button5_state == GPIO_PIN_RESET
				&& button5_last_state == GPIO_PIN_SET) {
			Speed++;
			if (Speed > 6) {
				Speed = 0;
			}
		}
		button5_last_state = button5_state;


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
