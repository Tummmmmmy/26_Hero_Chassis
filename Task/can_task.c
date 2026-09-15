/**
 * @file can_task.c
 * @brief CAN 接收解码任务(Can_Decode_Task)
 *
 * 从 canRxQueue 取帧, 交给 can_receive 路由分发。
 */

#include "can_task.h"
#include "cmsis_os.h"
#include "can_receive.h"

//CAN接收解码任务
void Can_Decode_Task(void const *argument)
{
    CAN_Rx_Message_t rx_msg;

    for(;;)
    {
        if(osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
        {
            CAN_Rx_Route(&rx_msg);
        }
    }
}
