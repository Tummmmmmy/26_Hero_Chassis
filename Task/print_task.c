/**
 * @file print_task.c
 * @brief 调试打印任务
 *
 */

#include "print_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_ano_protocol.h"
#include "chassis_control.h"

void Print_Entry_Task(void const * argument)
{
		osDelay(2000); 
		
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(10);
		
		for(;;)
		{
//				const Motor_Controller* motors = Chassis_Get_Motors();
//				VOFA_Print_Wave(10,

//				(float)motors[0].target_speed,
//				(float)motors[0].feedback.actual_speed,
//				(float)motors[0].target_current,

//				(float)motors[1].target_speed,
//				(float)motors[1].feedback.actual_speed,
//				(float)motors[1].target_current,

//				(float)motors[2].target_speed,
//				(float)motors[2].feedback.actual_speed,
//				(float)motors[2].target_current,

//				(float)motors[3].feedback.actual_speed
//				);
//(float)motors[3].target_speed,
//(float)motors[3].target_current
				
				vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}
}
