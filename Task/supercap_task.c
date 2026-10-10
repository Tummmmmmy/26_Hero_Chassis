/**
 * @file supercap_task.c
 * @brief 超级电容任务
 *
 */

#include "supercap_task.h"
#include "bsp_SuperCap.h"
#include "supercap_control.h"
#include "cmsis_os.h"


void SuperCap_Task(void const * argument)
{
    osDelay(2000);

    for(;;)
    {
        //离线检测
        supercap_update_offline(osKernelGetTickCount());

        //充放电决策
        SuperCap_Control_Update();

        //发送指令
        supercap_send_command(&hfdcan2);

        osDelay(50);
    }
}
