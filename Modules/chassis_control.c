/**
 * @file chassis_control.c
 * @brief 底盘麦克纳姆轮运动控制实现
 *
 * 底盘 4 个电机的状态全部收拢在本模块:
 * - 麦轮运动学解算与速度环 PID
 * - 电流按物理 ID 的分配映射
 * - 0x201~0x204 反馈解码与多圈角度累计
 * 模块不直接发送 CAN, 最终电流由整机调度层统一发送。
 */
#include "chassis_control.h"
#include <string.h>

#define PI 3.14159265358979f

//底盘参数
static const float L = 0.43f;  //前后轮距(米)
static const float W = 0.38f; //左右轮距(米)
static const float R = 0.074f; //轮子半径(米)
static const float R_recprocal = 1 / R;//R的倒数
static const float RPS_TO_RPM = 60.0f / (2.0f * PI);

//PID参数
//底盘PID
static float chassis_kp = 19.0f;//19.0
static float chassis_ki = 2.0f;//0.01f
static float chassis_kd = 0.0f;
static float chassis_Integral_Limit = 3000;//2000
static float chassis_MaxOutput = 12000;//最大输出电流

//底盘电机(逻辑顺序)与对应的物理ID
static Motor_Controller chassis_motors[CHASSIS_MOTOR_NUM];
static const uint8_t chassis_motor_physical_id[CHASSIS_MOTOR_NUM]= {1, 2, 4, 3};//底盘电机对应的物理ID

//电机离线判定超时(ms)
#define CHASSIS_MOTOR_OFFLINE_TIMEOUT_MS 200

//输出快照
static Chassis_Output_t s_out;

//初始化电机
static void Chassis_Motor_Init(Motor_Controller* motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan);

//更新电机累计角度
static void Encoder_Angle_Update(Motor_Controller *motor, float encoder_range);

//麦轮运动解算
static void Chassis_Movement_Calculation(float Vx, float Vy, float Wz);

//检测电机是否在线
static void Chassis_Check_Motor_Online(Motor_Controller* motor, uint32_t current_tick, uint32_t timeout_ms);


//初始化电机
static void Chassis_Motor_Init(Motor_Controller* motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan)
{
    memset(&motor->feedback, 0, sizeof(Motor_Feedback));
    motor->id = id;
    motor->hfdcan = hfdcan;
    motor->target_world_angle = 0.0f;
    motor->mechanical_degree = 0.0f;
    motor->target_total_angle = 0.0f;
    motor->round_count = 0;
    motor->last_feedback_angle = 0;
    motor->current_total_angle = 0.0f;
    motor->target_speed = 0.0f;
    motor->target_current = 0;
}


//更新电机累计角度
static void Encoder_Angle_Update(Motor_Controller *motor, float encoder_range)
{
    float half_range = encoder_range * 0.5f;//半圈值

    if(motor->last_feedback_angle == 0 && motor->round_count == 0 && motor->feedback.angle != 0)
    {
        motor->last_feedback_angle = motor->feedback.angle;
        motor->current_total_angle = (float)motor->feedback.angle;
        motor->mechanical_degree = (float)motor->feedback.angle * (360.0f / encoder_range);
        return;
    }
    
    int32_t angle_diff = (int32_t)motor->feedback.angle - (int32_t)motor->last_feedback_angle;

    //过零处理
    if(angle_diff > half_range) motor->round_count--;
    else if (angle_diff < -half_range) motor->round_count++;


    motor->current_total_angle = (float)(motor->round_count * encoder_range) + (float)motor->feedback.angle;//总角度 = 圈数 * 每圈编码值 + 当前单圈编码值
    motor->mechanical_degree = (float)motor->feedback.angle * (360.0f / encoder_range);//编码器值转换为机械角度度数
    motor->last_feedback_angle = motor->feedback.angle;
}


//麦轮运动解算
static void Chassis_Movement_Calculation(float Vx, float Vy, float Wz)
{

    float motor_target_rps[CHASSIS_MOTOR_NUM];//(rad/s)

    motor_target_rps[0] = ( Vx + Vy + Wz * (L + W) * 0.5f) * R_recprocal;
    motor_target_rps[1] = (-Vx + Vy + Wz * (L + W) * 0.5f) * R_recprocal;
    motor_target_rps[2] = ( Vx - Vy + Wz * (L + W) * 0.5f) * R_recprocal;
    motor_target_rps[3] = (-Vx - Vy + Wz * (L + W) * 0.5f) * R_recprocal;

    //从rad/s 转换为rad/min
    for(int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        chassis_motors[i].target_speed = motor_target_rps[i] * RPS_TO_RPM;
    }
}


//初始化底盘电机与速度环 PID
void Chassis_Control_Init(void)
{
    //初始化底盘电机
    Chassis_Motor_Init(&chassis_motors[0], chassis_motor_physical_id[0], &hfdcan2);
    Chassis_Motor_Init(&chassis_motors[1], chassis_motor_physical_id[1], &hfdcan2);
    Chassis_Motor_Init(&chassis_motors[2], chassis_motor_physical_id[2], &hfdcan2);
    Chassis_Motor_Init(&chassis_motors[3], chassis_motor_physical_id[3], &hfdcan2);

    //初始化底盘电机PID
    for(int i=0; i<CHASSIS_MOTOR_NUM; i++)
    {
        PID_Init(&chassis_motors[i].pid_speed, chassis_kp, chassis_ki, chassis_kd, chassis_Integral_Limit, chassis_MaxOutput);
    }
}


//整机启动: 恢复速度环 PID 输出限幅
void Chassis_On_Robot_Start(void)
{
    for(int i=0; i<CHASSIS_MOTOR_NUM; i++) PID_Init(&chassis_motors[i].pid_speed, chassis_kp, chassis_ki, chassis_kd, chassis_Integral_Limit, chassis_MaxOutput);
}


//整机失能: 目标与电流清零, PID 复位(输出限幅置 0)
void Chassis_On_Robot_Off(void)
{
    for(int i=0; i<CHASSIS_MOTOR_NUM; i++)
    {
        chassis_motors[i].target_speed = 0;
        chassis_motors[i].target_current = 0;
        PID_Init(&chassis_motors[i].pid_speed, chassis_kp, chassis_ki, chassis_kd, chassis_Integral_Limit, 0);
    }

    memset(&s_out, 0, sizeof(Chassis_Output_t));
}


//底盘控制: 麦轮解算 + 速度环计算, 结果写入输出快照
void Chassis_Control_Update(float vx, float vy, float wz, float max_output)
{
    Chassis_Movement_Calculation(vx, vy, wz);

    for(int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        float target_rpm = chassis_motors[i].target_speed;
        float measured_rpm = (float)chassis_motors[i].feedback.actual_speed;

        chassis_motors[i].pid_speed.MaxOutput = max_output;

        s_out.target_currents[i] = (int16_t)PID_Calculate(&chassis_motors[i].pid_speed, target_rpm, measured_rpm);

        s_out.speed_errors_rpm[i] = chassis_motors[i].pid_speed.error;
    }
}


//记录功率控制后的最终电流(逻辑电机顺序, 供调试观测)
void Chassis_Set_Target_Currents(const int16_t* final_currents)
{
    for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        chassis_motors[i].target_current = final_currents[i];
    }
}


//把逻辑电机电流映射为 0x200 报文的物理 ID 顺序
void Chassis_Map_Currents_To_Physical(const int16_t* logical_currents, int16_t* tx_currents)
{
    for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        tx_currents[i] = 0;
    }

    for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        //获取当前逻辑电机对应的物理ID
        uint8_t physical_id = chassis_motor_physical_id[i];
        if(physical_id >= 1 && physical_id <= CHASSIS_MOTOR_NUM)
        {
            //将电流发给对应物理ID的电机
            tx_currents[physical_id - 1] = logical_currents[i];
        }
    }
}


//0x201~0x204 底盘电机反馈解码
void Chassis_Update_Feedback(const uint8_t* data, uint32_t can_id)
{
    uint8_t physical_id = can_id - 0x200;//算出物理id
    
    for(int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        if(chassis_motor_physical_id[i] == physical_id)
        {
            chassis_motors[i].feedback.angle = (uint16_t)(data[0] << 8) | data[1];
            chassis_motors[i].feedback.actual_speed = (int16_t)((data[2] << 8) | data[3]);
            chassis_motors[i].feedback.current = (int16_t)((data[4] << 8) | data[5]);
            chassis_motors[i].feedback.temperature = data[6];
            chassis_motors[i].last_update_tick = osKernelGetTickCount();
            Encoder_Angle_Update(&chassis_motors[i], 8192.0f);
            break;
        }
    }
}


//检测电机是否在线
static void Chassis_Check_Motor_Online(Motor_Controller* motor, uint32_t current_tick, uint32_t timeout_ms)
{
    if(current_tick - motor->last_update_tick > timeout_ms) 
    {
        motor->is_online = false;
    } 
    else 
    {
        motor->is_online = true;
    }
}


//底盘电机在线检测
void Chassis_Update_Online_Status(uint32_t current_tick)
{
    for(int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        Chassis_Check_Motor_Online(&chassis_motors[i], current_tick, CHASSIS_MOTOR_OFFLINE_TIMEOUT_MS);
    }
}


//获取输出快照(零拷贝只读)
const Chassis_Output_t* Chassis_Get_Output(void)
{
    return &s_out;
}


//获取底盘电机只读指针(供功率控制使用)
const Motor_Controller* Chassis_Get_Motors(void)
{
    return chassis_motors;
}
