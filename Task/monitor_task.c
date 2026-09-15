#include "monitor_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"


extern Motor_Controller chassis_motors[CHASSIS_MOTOR_NUM];



/**
  * @brief  检测电机是否在线
  * @param  motor: 电机结构体指针
  * @param  current_tick: 当前系统时间戳 (ms)
  * @param  timeout_ms: 超时阈值 (ms)
  */
static inline void Check_Motor_Online(Motor_Controller* motor, uint32_t current_tick, uint32_t timeout_ms)
{
    if(current_tick - motor->last_update_tick > timeout_ms) 
    {
        motor->is_online = false;
    } 
    else 
    {
        motor->is_online = true;
    }
}



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
        for(int i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            Check_Motor_Online(&chassis_motors[i], now, 200);
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}




//// 离线检测任务 (100Hz 运行)
//void Monitor_Task(void const * argument)
//{
//    osDelay(1000); // 开机延时 1s，等外设上电稳定
//    
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//    const TickType_t TimeIncrement = pdMS_TO_TICKS(10); // 10ms 巡检一次

//    for(;;)
//    {
//        uint32_t now = osKernelGetTickCount();

//        // 1. 巡检 4 个底盘电机 (200ms 超时)
//        for(int i = 0; i < CHASSIS_MOTOR_NUM; i++)
//        {
//            Check_Motor_Online(&chassis_motors[i], now, 200);
//        }

//        // 2. 巡检双板通信 (来自云台的指令，200ms 超时)
//        if(now - chassis.last_update_tick > 200)
//        {
//            chassis.is_online = false;
//        }
//        else
//        {
//            chassis.is_online = true;
//        }

//        // 3. 巡检裁判系统 (裁判系统串口接收，500ms 超时)
//        Check_Device_Online(referee_last_update_tick, &is_referee_online, now, 500);

//        // 4. (可选) 巡检超级电容 / 功率计
//        // Check_Device_Online(supercap.last_update_tick, &supercap.is_online, now, 300);

//        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
//    }
//}
