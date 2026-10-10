/**
 * @file power_control.h
 * @brief 底盘功率控制接口
 *
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include <stdint.h>
#include "motor.h"


//初始化功率控制
void Power_Control_Init(void);

//功率控制
void Power_Control_Update(const Motor_Controller* motors, const int16_t* target_current, const float* speed_error_rpm);

//获取功率控制后的最终电流
void Power_Control_Get_Currents(int16_t* output_current);

#endif //POWER_CONTROL_H
