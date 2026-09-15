/**
 * @file chassis_control.h
 * @brief 底盘麦克纳姆轮运动控制接口
 *
 * 麦轮运动学解算 + 4 个底盘电机速度环 PID + 电流分配。
 * 电机反馈与 PID 状态均归本模块所有, 对外只提供只读访问。
 * 模块不直接发送 CAN, 最终电流由整机调度层统一发送。
 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>
#include "main.h"

#define CHASSIS_MOTOR_NUM 4

//底盘输出快照(供功率控制与统一 CAN 发送读取)
typedef struct
{
    int16_t target_currents[CHASSIS_MOTOR_NUM]; //速度环输出电流(逻辑电机顺序)
    float   speed_errors_rpm[CHASSIS_MOTOR_NUM];//速度环误差(供功率控制分配)
} Chassis_Output_t;

//初始化底盘电机与速度环 PID
void Chassis_Control_Init(void);

//整机启动: 恢复速度环 PID 输出限幅
void Chassis_On_Robot_Start(void);

//整机失能: 目标与电流清零, PID 复位(输出限幅置 0)
void Chassis_On_Robot_Off(void);

//底盘控制: 麦轮解算 + 速度环计算, 结果写入输出快照
//vx/vy: 底盘前后/左右速度(m/s), wz: 自旋角速度(rad/s)
//max_output: 本周期电流限幅(超电在线时更大)
void Chassis_Control_Update(float vx, float vy, float wz, float max_output);

//记录功率控制后的最终电流(逻辑电机顺序, 供调试观测)
void Chassis_Set_Target_Currents(const int16_t* final_currents);

//把逻辑电机电流映射为 0x200 报文的物理 ID 顺序
void Chassis_Map_Currents_To_Physical(const int16_t* logical_currents, int16_t* tx_currents);

//0x201~0x204 底盘电机反馈解码
void Chassis_Update_Feedback(const uint8_t* data, uint32_t can_id);

//底盘电机在线检测(Monitor_Task 周期调用)
void Chassis_Update_Online_Status(uint32_t current_tick);

//获取输出快照(零拷贝只读)
const Chassis_Output_t* Chassis_Get_Output(void);

//获取底盘电机只读指针(供功率控制使用)
const Motor_Controller* Chassis_Get_Motors(void);

#endif
