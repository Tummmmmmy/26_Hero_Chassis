/**
 * @file control_task.c
 * @brief 机器人主控制任务
 *
 */

#include "control_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot_control.h"


void Chassis_Control_task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
	
  for(;;)
  {
		
      Robot_Control_Loop();
      
	    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}

}
