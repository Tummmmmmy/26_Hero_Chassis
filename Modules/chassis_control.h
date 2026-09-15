/**
 * @file chassis_control.h
 * @brief 底盘麦克纳姆轮运动控制接口
 *
 * 麦轮运动学解算 + 4 个底盘电机速度环 PID + 电流分配。
 * 电机反馈与 PID 状态均归本模块所有, 对外只提供只读访问。
 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

#define CHASSIS_MOTOR_NUM 4

#endif
