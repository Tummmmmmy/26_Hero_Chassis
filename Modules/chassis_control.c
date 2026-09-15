/**
 * @file chassis_control.c
 * @brief 底盘麦克纳姆轮运动控制实现
 *
 * 麦轮运动学解算 + 4 个底盘电机速度环 PID + 电流分配。
 */

#include "chassis_control.h"
