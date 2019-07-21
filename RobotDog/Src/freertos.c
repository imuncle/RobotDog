/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "mpu6050.h"
#include "scs009.h"
#include "remote.h"
#include "robotcmd.h"
#include "body_task.h"
#include "head_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int test_position = 500;
extern int gyroBiasFound;
/* USER CODE END Variables */
osThreadId RobotCmdHandle;
osThreadId ImuHandle;
osThreadId BodyHandle;
osThreadId HeadHandle;
osThreadId RemoteHandle;
osTimerId LedHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void robotcmd_task(void const * argument);
void imu_task(void const * argument);
void body_task(void const * argument);
void head_task(void const * argument);
void remote_task(void const * argument);
void led_timer(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the timer(s) */
  /* definition and creation of Led */
  osTimerDef(Led, led_timer);
  LedHandle = osTimerCreate(osTimer(Led), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(LedHandle, 200);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of RobotCmd */
  osThreadDef(RobotCmd, robotcmd_task, osPriorityNormal, 0, 128);
  RobotCmdHandle = osThreadCreate(osThread(RobotCmd), NULL);

  /* definition and creation of Imu */
  osThreadDef(Imu, imu_task, osPriorityNormal, 0, 256);
  ImuHandle = osThreadCreate(osThread(Imu), NULL);

  /* definition and creation of Body */
  osThreadDef(Body, body_task, osPriorityNormal, 0, 512);
  BodyHandle = osThreadCreate(osThread(Body), NULL);

  /* definition and creation of Head */
  osThreadDef(Head, head_task, osPriorityNormal, 0, 512);
  HeadHandle = osThreadCreate(osThread(Head), NULL);

  /* definition and creation of Remote */
  osThreadDef(Remote, remote_task, osPriorityNormal, 0, 128);
  RemoteHandle = osThreadCreate(osThread(Remote), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_robotcmd_task */
/**
  * @brief  Function implementing the RobotCmd thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_robotcmd_task */
void robotcmd_task(void const * argument)
{

  /* USER CODE BEGIN robotcmd_task */
  /* Infinite loop */
  for(;;)
  {
    LedStateChange();
		BodyParamChange();
		HeadStateChange();
		osDelay(1);
  }
  /* USER CODE END robotcmd_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the Imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
void imu_task(void const * argument)
{
  /* USER CODE BEGIN imu_task */
	while(MPU6050_Init()){}
  /* Infinite loop */
  for(;;)
  {
		MPU6050_ReadData();
		imuDataHandle();
		if(gyroBiasFound)
		{
			imuUpdate(mpu6500.gyro);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		}
    osDelay(2);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_body_task */
/**
* @brief Function implementing the Body thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_body_task */
void body_task(void const * argument)
{
  /* USER CODE BEGIN body_task */
	BodyInit();
  /* Infinite loop */
  for(;;)
  {
		BodyChange();
		LegChange();
    ServoSendData();
		osDelay(5);
  }
  /* USER CODE END body_task */
}

/* USER CODE BEGIN Header_head_task */
/**
* @brief Function implementing the Head thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_head_task */
void head_task(void const * argument)
{
  /* USER CODE BEGIN head_task */
	HeadInit();
  /* Infinite loop */
  for(;;)
  {
    HeadChange();
		HeadCalc();
		HeadServoSendData();
		osDelay(5);
  }
  /* USER CODE END head_task */
}

/* USER CODE BEGIN Header_remote_task */
/**
* @brief Function implementing the Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_task */
void remote_task(void const * argument)
{
  /* USER CODE BEGIN remote_task */
	while(NRF24L01_Check()){};
	NRF24L01_RX_Mode();
  /* Infinite loop */
  for(;;)
  {
		RemoteReceive();
		osDelay(1);
  }
  /* USER CODE END remote_task */
}

/* led_timer function */
void led_timer(void const * argument)
{
  /* USER CODE BEGIN led_timer */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END led_timer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
