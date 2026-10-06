/**
 * @file chassis_control.h
 * @brief 底盘麦克纳姆轮运动控制接口
 *
 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>
#include "motor.h"

#define CHASSIS_MOTOR_NUM 4


//底盘输出
typedef struct
{
    int16_t target_currents[CHASSIS_MOTOR_NUM];
    float   speed_errors_rpm[CHASSIS_MOTOR_NUM];
} Chassis_Output_t;



//初始化底盘
void Chassis_Control_Init(void);

//底盘使能
void Chassis_Enable(void);

//底盘失能
void Chassis_Disable(void);

//更新底盘控制
void Chassis_Control_Update(float vx, float vy, float wz, float max_output);

//记录功率控制后的最终电流
void Chassis_Set_Target_Currents(const int16_t* final_currents);

//映射电流
void Chassis_Map_Currents_To_Physical(const int16_t* logical_currents, int16_t* tx_currents);

//解码底盘电机的CAN反馈帧
void Chassis_Update_Feedback(const uint8_t* data, uint32_t can_id);

//底盘电机在线检测
void Chassis_Update_Online_Status(uint32_t current_tick);

//底盘输出
const Chassis_Output_t* Chassis_Get_Output(void);

//获取底盘电机信息
const Motor_Controller* Chassis_Get_Motors(void);

#endif
