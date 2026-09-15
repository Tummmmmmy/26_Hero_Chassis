/**
 * @file robot_control.c
 * @brief 底盘板整机调度实现
 *
 * 底盘板的总控制流程(1ms): 失能检查 -> 使能切换 -> 上坎 -> 底盘运动
 * -> 功率控制 -> 统一 CAN 输出。
 */

#include "robot_control.h"
