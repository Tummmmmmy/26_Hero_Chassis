/**
 * @file can_task.c
 * @brief CAN 接收解码任务
 *
 */

#include "can_task.h"
#include "cmsis_os.h"
#include "can_receive.h"


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
