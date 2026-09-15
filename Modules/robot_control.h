/**
 * @file robot_control.h
 * @brief 底盘板整机调度接口与整机状态类型
 *
 * 底盘板的总控制流程(1ms): 失能检查 -> 使能切换 -> 上坎 -> 底盘运动
 * -> 功率控制 -> 统一 CAN 输出。各模块只提供状态与输出, 不自行发送 CAN。
 */
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

//机器人LED状态枚举
typedef enum {
    LED_BOOTING,        //启动中
    LED_DISABLED,       //失能(红色)
    LED_ENABLED,        //使能(蓝色)
	  LED_CLIMB_ACTIVE,   //
    LED_CLIMB_FRONT,    //前伸
    LED_CLIMB_BACK,     //后伸
} LED_State_e;


extern volatile LED_State_e led_state;

#endif
