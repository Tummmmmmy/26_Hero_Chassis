/**
 * @file control_task.c
 * @brief 1ms 控制任务(Chassis_Control_task)
 *
 * 周期调用 Robot_Control_Loop 完成整机控制。
 */

#include "control_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot_control.h"

//控制任务
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
