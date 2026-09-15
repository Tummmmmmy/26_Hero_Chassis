/**
 * @file bsp_can.h
 * @brief FDCAN 硬件抽象层接口
 *
 * 只负责 FDCAN 外设配置与底层收帧，不解析具体协议。
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "fdcan.h"

// FDCAN 硬件层初始化：配置滤波器、启动 FDCAN1/2/3，并开启 RX FIFO0 接收中断
void BSP_FDCAN_Config(void);


// 底层发送一帧（标准帧 + Classic CAN），自动检查 TX FIFO 是否空闲
void BSP_CAN_Send(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, uint8_t* data, uint32_t dlc);
#endif
