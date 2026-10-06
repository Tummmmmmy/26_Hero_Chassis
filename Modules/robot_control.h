/**
 * @file robot_control.h
 * @brief 底盘板整机调度接口与整机状态类型
 *
 */
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

//LED状态
typedef enum {
    LED_BOOTING,        //启动中
    LED_DISABLED,       //失能
    LED_ENABLED,        //使能
	  LED_CLIMB_ACTIVE,   //
    LED_CLIMB_FRONT,    //前伸
    LED_CLIMB_BACK,     //后伸
} LED_State_e;


//初始化模块
void Robot_Param_Init(void);

//主控制循环
void Robot_Control_Loop(void);

//获取LED状态
LED_State_e Robot_Get_Led_State(void);

//设置LED状态
void Robot_Set_Led_State(LED_State_e state);

#endif
