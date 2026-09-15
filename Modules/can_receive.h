/**
 * @file can_receive.h
 * @brief CAN 接收路由接口
 *
 * Can_Decode_Task 取出队列中的帧后, 调用 CAN_Rx_Route 分发到各模块。
 */
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

// 按总线/ID 分发CAN接收帧到对应模块
void CAN_Rx_Route(CAN_Rx_Message_t* msg);

#endif
