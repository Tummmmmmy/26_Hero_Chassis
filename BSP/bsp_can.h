/**
 * @file bsp_can.h
 * @brief FDCAN 硬件抽象层接口
 *
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "fdcan.h"
#include "cmsis_os2.h"

//CAN接收队列数据
typedef struct
{
    FDCAN_HandleTypeDef* hfdcan;
    uint32_t can_id;
    uint8_t data[8];
  	uint8_t  dlc;
} CAN_Rx_Message_t;

extern osMessageQueueId_t canRxQueueHandle;

// FDCAN 硬件层初始化
void BSP_FDCAN_Config(void);


// 底层发送一帧
void BSP_CAN_Send(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, uint8_t* data, uint32_t dlc);
#endif
