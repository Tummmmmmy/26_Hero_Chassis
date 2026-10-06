/**
 * @file can_transmit.h
 * @brief CAN 发送协议层接口
 *
 */
#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H

#include "main.h"

//底盘发云台
#define TX_GIMBAL_DATA_ID 0x304

//拨盘速度指令
#define DIAL_MOTOR_SPEED_COMMAND_ID 0x207

//拨盘模式指令
#define DM_DIAL_SEND_ID 0x07

//拨盘反馈
#define TX_DIAL_FDB_ID 0x302



//大疆电机控制指令发送
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

//发送给云台信息
void Send_Gimbal_Data(FDCAN_HandleTypeDef* hfdcan);

//转发云台发来的拨盘速度指令给达妙拨盘
void Dial_Relay_Speed_Cmd(FDCAN_HandleTypeDef* hfdcan, uint8_t* data);

//转发云台发来的拨盘模式指令给达妙拨盘
void Dial_Relay_Mode_Cmd(FDCAN_HandleTypeDef* hfdcan, uint8_t* data);

//转发达妙拨盘反馈给云台板
void Dial_Relay_Feedback(FDCAN_HandleTypeDef* hfdcan, uint8_t* data);

#endif
