/**
 * @file robot_control.c
 * @brief 底盘板整机调度实现
 *
 * 底盘板的总控制流程(1ms): 失能检查 -> 使能切换 -> 上坎 -> 底盘运动
 * -> 功率控制 -> 统一 CAN 输出。
 */

#include "robot_control.h"

//系统LED状态(device_task 通过 Robot_Get_Led_State 读取)
static volatile LED_State_e s_led_state = LED_BOOTING;

//获取系统LED状态
LED_State_e Robot_Get_Led_State(void)
{
    return s_led_state;
}


//设置系统LED状态
void Robot_Set_Led_State(LED_State_e state)
{
    s_led_state = state;
}
