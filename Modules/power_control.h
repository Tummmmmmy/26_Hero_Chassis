/**
 * @file power_control.h
 * @brief 底盘功率控制接口
 *
 * 功率控制器内部状态(缓冲能量环、功率模型、分配结果)全部私有,
 * 对外只提供 初始化 / 周期计算 / 读取最终电流 三个接口。
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include <stdint.h>
#include "motor.h"

//初始化功率控制
void Power_Control_Init(void);

//功率控制(输入: 电机反馈 / 速度环目标电流 / 速度误差)
void Power_Control_Update(const Motor_Controller* motors, const int16_t* target_current, const float* speed_error_rpm);

//获取功率控制后的最终电流
void Power_Control_Get_Currents(int16_t* output_current);

#endif //POWER_CONTROL_H
