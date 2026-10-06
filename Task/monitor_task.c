/**
 * @file monitor_task.c
 * @brief 电机在线检测任务
 *
 */

#include "monitor_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "chassis_control.h"
#include "gimbal_link.h"




//离线检测任务
void Monitor_Task(void const * argument)
{
    osDelay(1000); 
    
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

    for(;;)
    {
        uint32_t now = osKernelGetTickCount();

			  //底盘电机
        Chassis_Update_Online_Status(now);

			  //双板链路(云台 -> 底盘)
        Gimbal_Link_Update_Online_Status(now);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}


