/**
 * @file gimbal_link.h
 * @brief 底盘板与云台板的板间链路数据契约
 *
 * 云台板通过 CAN 下发整机指令(0x300)与 UI 数据(0x308), 本模块承载这两份
 * 输入快照的类型定义与解码, 并维护链路在线状态(200ms 超时)。字段与云台板
 * 发送侧的 Send_Chassis_Command / Send_Chassis_UI_Data 一一对应, 不得单独修改。
 */
#ifndef GIMBAL_LINK_H
#define GIMBAL_LINK_H

#include <stdint.h>
#include <stdbool.h>

//双板通信CAN ID
#define RX_GIMBAL_COMMAND_ID 0x300
#define RX_CHASSIS_UI_ID 0x308

//板间链路离线判定超时(ms)
#define GIMBAL_LINK_OFFLINE_TIMEOUT_MS 200

//双板通信接收结构体
typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t wz;
    uint8_t mode; //0:失能, 1:使能
	  uint8_t supercap_req;
	  uint8_t climb_mode;
	  uint8_t led_cmd;
	  uint8_t key_shift;//是否按下shift
	  uint8_t friction_state;//是否开启摩擦轮 0:关闭, 1:开启
	  uint8_t friction_speed;
	  uint8_t is_base_locked;//是否锁定基地
	  uint8_t battle_mode;   //键鼠模式(0: 失能, 1:跟随, 2: 底盘锁 3: 吊射)
    uint32_t cnt;
} Chassis_Cmd_t;

//双板通信接收结构体
typedef struct
{
    float yaw_deg;
    float pit_deg;
	  uint16_t friction_speed;
    uint32_t cnt;
} Chassis_UI_t;

extern volatile Chassis_Cmd_t chassis;
extern volatile Chassis_UI_t UI_Data;

//解码云台发来的整机指令(0x300)
void Gimbal_Link_Decode_Command(const uint8_t *data);

//解码云台发来的UI数据包(0x308)
void Gimbal_Link_Decode_UI(const uint8_t *data);

//双板链路在线巡检(Monitor_Task 调用)
void Gimbal_Link_Update_Online_Status(uint32_t current_tick);

//双板链路是否在线
bool Gimbal_Link_Is_Online(void);

#endif
