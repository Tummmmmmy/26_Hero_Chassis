/**
 * @file device_task.c
 * @brief 设备指示任务
 *
 */

#include "device_task.h"
#include "ws2812.h"
#include "cmsis_os2.h"
#include "robot_control.h"


void Device_task(void)
{
	
	osDelay(500);
	
	while(1)
	{	

			switch(Robot_Get_Led_State())
			{
					case LED_BOOTING:
					  	WS2812_Set_Colour(255, 255, 0); //黄色
							break;

					case LED_DISABLED:
							WS2812_Set_Colour(255, 0, 0);   //红色
							break;

					case LED_ENABLED:
							WS2812_Set_Colour(0, 0, 255);   //蓝色
							break;
					
					case LED_CLIMB_ACTIVE:
						  WS2812_Set_Colour(128, 0, 128); //紫色
							break;
					
					case LED_CLIMB_FRONT:
				    	WS2812_Set_Colour(255, 100, 0); //橙色
							break;
							
					case LED_CLIMB_BACK:
							WS2812_Set_Colour(0, 255, 255); //青色
							break;
							
					default:
							WS2812_Set_Colour(255, 0, 0);
							osDelay(50);
							WS2812_Set_Colour(0, 0, 0);
							break;
				}
			osDelay(50);
	}
}

