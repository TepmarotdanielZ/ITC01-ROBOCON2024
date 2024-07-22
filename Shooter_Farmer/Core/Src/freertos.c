/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "usart.h"
#include "math.h"
#include "gpio.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define gear_stepper 19
#define dt 0.01
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//------------------------------uart-functions--------------------------------------
volatile extern int flag;
volatile int flag = 0 ;
uint64_t RxpipeAddrs = 0x11223344BB;
const void* myAckPayload ;
int input_num = 0 ;
char input_data[32] ;
char c1;
char* c = &c1;
char c2;
char* c3 = &c2;
uint64_t TxpipeAddrs = 0x11223344AA;
char myTxData[32];
char AckPayload[32];
char myRxData[50];
uint8_t RxDataESP[13];
int degree;
float shooter;
double head;
double omega;

//** Laser **/
uint16_t AD_RES  = 0;
int centimeter;
float distance;


// Take Rice and Push Rice
extern uint8_t Position ;
extern uint8_t Push;
uint8_t STEP = 0;
extern uint8_t Drop;
extern uint8_t Catch ;
extern uint8_t Stepper;
extern uint8_t SpeedShooter;
uint8_t state = 0 ;
uint8_t Stock = 0;

uint8_t push = 0;
uint8_t state2;
uint8_t Shoot;
uint8_t DropState = 0;
uint8_t dropstate = 0;
uint8_t push2 = 0;

float X_end = 0.0;
float X_goal = 0.0;
float X_goal1 = 0.0;
//Drop Rice
float goal1[] = {2.15, -3.0, 1.57};
float goal2[] = {2.15, -2.4, 1.57};
float goal3[] = {2.15, -1.8, 1.57};
float goal4[] = {2.15, -1.2, 1.57};
float goal5[] = {2.15, -0.6, 1.57};
float goal6[] = {2.15, -0.0, 1.57};
extern float vx;
extern float vy;
extern float Omega;
float x;
float y;
float yaw;
float status;
extern uint8_t Drop;
extern uint8_t StateP;

/* USER CODE END Variables */
osThreadId Stepper_TaskHandle;
osThreadId Take_Rice_TaskHandle;
osThreadId Drop_RiceTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int Map(int Input, int Min_Input , int Max_Input ,int Min_Output, int Max_Output){

	return (int) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

/* USER CODE END FunctionPrototypes */

void Stepper_Init(void const * argument);
void Take_Rice_Init(void const * argument);
void Drop_RiceInit(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Stepper_Task */
  osThreadDef(Stepper_Task, Stepper_Init, osPriorityNormal, 0, 500);
  Stepper_TaskHandle = osThreadCreate(osThread(Stepper_Task), NULL);

  /* definition and creation of Take_Rice_Task */
  osThreadDef(Take_Rice_Task, Take_Rice_Init, osPriorityNormal, 0, 1200);
  Take_Rice_TaskHandle = osThreadCreate(osThread(Take_Rice_Task), NULL);

  /* definition and creation of Drop_RiceTask */
  osThreadDef(Drop_RiceTask, Drop_RiceInit, osPriorityNormal, 0, 500);
  Drop_RiceTaskHandle = osThreadCreate(osThread(Drop_RiceTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Stepper_Init */
/**
  * @brief  Function implementing the Stepper_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Stepper_Init */
void Stepper_Init(void const * argument)
{
  /* USER CODE BEGIN Stepper_Init */
  /* Infinite loop */
  for(;;)
  {
	  if (Stepper == 1) {
			head = 5.0;
		} else if (Stepper == 2) {
			head = -5.0;
		} else {
			head = 0;
		}
		if (head >= 0) {
			HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_SET);
			omega = head * gear_stepper;

		} else if (head < 0) {
			HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_RESET);
			omega = -1 * head * gear_stepper;
		}

    osDelay(10);
  }
  /* USER CODE END Stepper_Init */
}

/* USER CODE BEGIN Header_Take_Rice_Init */
/**
* @brief Function implementing the Take_Rice_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Take_Rice_Init */
void Take_Rice_Init(void const * argument)
{
  /* USER CODE BEGIN Take_Rice_Init */
  /* Infinite loop */
  for(;;)
  {


	if (((Position == 1 && StateP == 2) ||  (Position == 2 && StateP == 3) || (Position == 3 && StateP == 4)) && state == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //release
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); //Down
		state = 1;
	}
	else if (state == 1 && Catch ==1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //catch
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); // Down
		osDelay(1000);
		state = 2;
	}
	else if (state == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //cath
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // up
		osDelay(3000);
		state = 3;

	}
	else if (state == 3 )
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //release grip
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //Up
			osDelay(2000);
			state = 0;
		}



    osDelay(10);
  }
  /* USER CODE END Take_Rice_Init */
}

/* USER CODE BEGIN Header_Drop_RiceInit */
/**
* @brief Function implementing the Drop_RiceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Drop_RiceInit */
void Drop_RiceInit(void const * argument)
{
  /* USER CODE BEGIN Drop_RiceInit */

  /* Infinite loop */
  for(;;)
  {
	  if (Drop == 1) {

//	  	  			X_end = 0.1;
	  	  			state2 = 1;
	  }
//	  	  		} else if (Drop == 0 && Position>=4) {
//	  	  			X_end = 0.0;
//	  	  		}
	  if (state2 == 1){
			osDelay(1500);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); //Push
			osDelay(2000);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
			state2 = 0;
		}

    osDelay(10);
  }
  /* USER CODE END Drop_RiceInit */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
