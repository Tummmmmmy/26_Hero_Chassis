/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os2.h"

#include "bsp_power_meter.h"
#include "bsp_can.h"
#include "bsp_SuperCap.h"
#include "power_control.h"
#include "RM_JudgeSystem.h"
#include "bsp_ano_protocol.h"
#include "device_task.h"
#include "can_task.h"
#include "tim.h"
//#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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




///* USER CODE END Variables */
/* Definitions for All_Init */
osThreadId_t All_InitHandle;
const osThreadAttr_t All_Init_attributes = {
  .name = "All_Init",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CanDecodeTask */
osThreadId_t CanDecodeTaskHandle;
const osThreadAttr_t CanDecodeTask_attributes = {
  .name = "CanDecodeTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for AnoPrintTask */
osThreadId_t AnoPrintTaskHandle;
const osThreadAttr_t AnoPrintTask_attributes = {
  .name = "AnoPrintTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for SuperCapTask */
osThreadId_t SuperCapTaskHandle;
const osThreadAttr_t SuperCapTask_attributes = {
  .name = "SuperCapTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for DeviceTask */
osThreadId_t DeviceTaskHandle;
const osThreadAttr_t DeviceTask_attributes = {
  .name = "DeviceTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
uint32_t IMU_Buffer[ 512 ];
osStaticThreadDef_t IMUControlBlock;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .cb_mem = &IMUControlBlock,
  .cb_size = sizeof(IMUControlBlock),
  .stack_mem = &IMU_Buffer[0],
  .stack_size = sizeof(IMU_Buffer),
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for UITask */
osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for PrintEntryTask */
osThreadId_t PrintEntryTaskHandle;
const osThreadAttr_t PrintEntryTask_attributes = {
  .name = "PrintEntryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for anoPrintQueue */
osMessageQueueId_t anoPrintQueueHandle;
const osMessageQueueAttr_t anoPrintQueue_attributes = {
  .name = "anoPrintQueue"
};
/* Definitions for anoTxSem */
osSemaphoreId_t anoTxSemHandle;
const osSemaphoreAttr_t anoTxSem_attributes = {
  .name = "anoTxSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void All_Init_Run(void *argument);
extern void Chassis_Control_task(void *argument);
extern void Print_Task(void *argument);
extern void SuperCap_Task(void const * argument);
void Device_Task(void *argument);
extern void IMU_task(void *argument);
extern void Client_UI_Task(void *argument);
extern void Print_Entry_Task(void *argument);
extern void Monitor_Task(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of anoTxSem */
  anoTxSemHandle = osSemaphoreNew(1, 1, &anoTxSem_attributes);
	
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (20, sizeof(CAN_Rx_Message_t), &canRxQueue_attributes);

  /* creation of anoPrintQueue */
  anoPrintQueueHandle = osMessageQueueNew (10, sizeof(Print_Msg_t), &anoPrintQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of All_Init */
  All_InitHandle = osThreadNew(All_Init_Run, NULL, &All_Init_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(Chassis_Control_task, NULL, &Chassis_Task_attributes);

  /* creation of CanDecodeTask */
  CanDecodeTaskHandle = osThreadNew((osThreadFunc_t)Can_Decode_Task, NULL, &CanDecodeTask_attributes);

  /* creation of AnoPrintTask */
  AnoPrintTaskHandle = osThreadNew(Print_Task, NULL, &AnoPrintTask_attributes);

  /* creation of SuperCapTask */
  SuperCapTaskHandle = osThreadNew((osThreadFunc_t)SuperCap_Task, NULL, &SuperCapTask_attributes);

  /* creation of DeviceTask */
  DeviceTaskHandle = osThreadNew(Device_Task, NULL, &DeviceTask_attributes);

  /* creation of INSTask */
  INSTaskHandle = osThreadNew((osThreadFunc_t)IMU_task, NULL, &INSTask_attributes);
	
  /* creation of UITask */
  UITaskHandle = osThreadNew(Client_UI_Task, NULL, &UITask_attributes);
	
	/* creation of PrintEntryTask */
  PrintEntryTaskHandle = osThreadNew(Print_Entry_Task, NULL, &PrintEntryTask_attributes);
	
  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(Monitor_Task, NULL, &MonitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_All_Init_Run */
/**
  * @brief  Function implementing the All_Init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_All_Init_Run */
void All_Init_Run(void *argument)
{
  /* USER CODE BEGIN All_Init_Run */
	
	taskENTER_CRITICAL();  //进入临界区
	
	BSP_FDCAN_Config();
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,455);//比例阀   pwm1=1     比赛285   pwm6=3.5
	
	power_meter_init();
	
	JudgeSystem_Init(&huart1);
  extern volatile uint8_t init_finished;
  init_finished = 1; 
	
	supercap_init();
	
	Robot_Param_Init();
	
	taskEXIT_CRITICAL();   //退出临界区
	
	vTaskDelete(NULL);
  /* USER CODE END All_Init_Run */
}

/* USER CODE BEGIN Header_Device_Task */
/**
* @brief Function implementing the DeviceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Device_Task */
void Device_Task(void *argument)
{
  /* USER CODE BEGIN Device_Task */
  /* Infinite loop */
  for(;;)
  {
    Device_task();
  }
  /* USER CODE END Device_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
