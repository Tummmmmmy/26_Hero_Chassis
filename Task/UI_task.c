#include "UI_task.h"
#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "RM_JudgeSystem.h" 

/**
  * @brief  UIË¢ÐÂÈÎÎñ
  */
void Client_UI_Task(void const * argument)
{
	  osDelay(1000);

	  TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
	
	 uint16_t static_refresh_cnt = 0;
	uint16_t Dynamic_refresh_cnt = 0;
    for(;;)
    {      
			  UI_Dynamic(); 
			
			  static_refresh_cnt++;
        if(static_refresh_cnt >= 75)
        {
            static_refresh_cnt = 0;
            UI_Static();
        }
			
				
				
			  Dynamic_refresh_cnt++;
        if(Dynamic_refresh_cnt >= 10)
        {
            Dynamic_refresh_cnt = 0;
					  UI_Dynamic2(); 
            
        }
				
			
			  vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

