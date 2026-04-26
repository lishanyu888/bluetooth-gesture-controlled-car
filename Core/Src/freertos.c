/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "telemetry.h"

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

/* USER CODE END Variables */
/* Definitions for MPUTask */
osThreadId_t MPUTaskHandle;
const osThreadAttr_t MPUTask_attributes = {
  .name = "MPUTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BT24Task */
osThreadId_t BT24TaskHandle;
const osThreadAttr_t BT24Task_attributes = {
  .name = "BT24Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for WatchDogTask */
osThreadId_t WatchDogTaskHandle;
const osThreadAttr_t WatchDogTask_attributes = {
  .name = "WatchDogTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for YawQueue */
osMessageQueueId_t YawQueueHandle;
const osMessageQueueAttr_t YawQueue_attributes = {
  .name = "YawQueue"
};
/* Definitions for ShowQueue */
osMessageQueueId_t ShowQueueHandle;
const osMessageQueueAttr_t ShowQueue_attributes = {
  .name = "ShowQueue"
};
/* Definitions for BtCmdQueue */
osMessageQueueId_t BtCmdQueueHandle;
const osMessageQueueAttr_t BtCmdQueue_attributes = {
  .name = "BtCmdQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMPUTask(void *argument);
extern void StartMotorTask(void *argument);
extern void StartOLEDTask(void *argument);
extern void StartBT24Task(void *argument);
extern void StartWatchDogTask(void *argument);

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of YawQueue */
  YawQueueHandle = osMessageQueueNew (1, sizeof(YawData_t), &YawQueue_attributes);

  /* creation of ShowQueue */
  ShowQueueHandle = osMessageQueueNew (1, sizeof(ShowData_t), &ShowQueue_attributes);

  /* creation of BtCmdQueue */
  BtCmdQueueHandle = osMessageQueueNew (4, sizeof(BtCmd_t), &BtCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MPUTask */
  MPUTaskHandle = osThreadNew(StartMPUTask, NULL, &MPUTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of OLEDTask */
  OLEDTaskHandle = osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);

  /* creation of BT24Task */
  BT24TaskHandle = osThreadNew(StartBT24Task, NULL, &BT24Task_attributes);

  /* creation of WatchDogTask */
  WatchDogTaskHandle = osThreadNew(StartWatchDogTask, NULL, &WatchDogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMPUTask */
/**
  * @brief  Function implementing the MPUTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMPUTask */
__weak void StartMPUTask(void *argument)
{
  /* USER CODE BEGIN StartMPUTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMPUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

