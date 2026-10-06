/**
 * @file chassis_control.c
 * @brief 底盘麦克纳姆轮运动控制实现
 *
 */
#include "chassis_control.h"
#include <string.h>

#define PI 3.14159265358979f
static const float RPS_TO_RPM = 60.0f / (2.0f * PI);

//底盘参数
static const float L = 0.43f;  //前后轮距(米)
static const float W = 0.38f; //左右轮距(米)
static const float R = 0.074f; //轮子半径(米)
static const float R_recprocal = 1 / R;//R的倒数



//PID参数
//底盘PID
static float chassis_kp = 19.0f;//19.0
static float chassis_ki = 2.0f;//0.01f
static float chassis_kd = 0.0f;
static float chassis_Integral_Limit = 3000;//2000
static float chassis_MaxOutput = 12000;//最大输出电流



//底盘电机与对应的物理ID
static Motor_Controller chassis_motors[CHASSIS_MOTOR_NUM];
static const uint8_t chassis_motor_physical_id[CHASSIS_MOTOR_NUM]= {1, 2, 4, 3};//底盘电机对应的物理ID


#define CHASSIS_MOTOR_OFFLINE_TIMEOUT_MS 200

//底盘输出
static Chassis_Output_t output;

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


//初始化底盘
void Chassis_Control_Init(void)
{
    Chassis_Motor_Init(&chassis_motors[0], chassis_motor_physical_id[0], &hfdcan2);
    Chassis_Motor_Init(&chassis_motors[1], chassis_motor_physical_id[1], &hfdcan2);
    Chassis_Motor_Init(&chassis_motors[2], chassis_motor_physical_id[2], &hfdcan2);
    Chassis_Motor_Init(&chassis_motors[3], chassis_motor_physical_id[3], &hfdcan2);

    for(int i=0; i<CHASSIS_MOTOR_NUM; i++)
    {
        PID_Init(&chassis_motors[i].pid_speed, chassis_kp, chassis_ki, chassis_kd, chassis_Integral_Limit, chassis_MaxOutput);
    }
}



//底盘使能
void Chassis_Enable(void)
{
    for(int i=0; i<CHASSIS_MOTOR_NUM; i++) PID_Init(&chassis_motors[i].pid_speed, chassis_kp, chassis_ki, chassis_kd, chassis_Integral_Limit, chassis_MaxOutput);
}



//底盘失能
void Chassis_Disable(void)
{
    for(int i=0; i<CHASSIS_MOTOR_NUM; i++)
    {
        chassis_motors[i].target_speed = 0;
        chassis_motors[i].target_current = 0;
        PID_Init(&chassis_motors[i].pid_speed, chassis_kp, chassis_ki, chassis_kd, chassis_Integral_Limit, 0);
    }

    memset(&output, 0, sizeof(Chassis_Output_t));
}


//更新底盘控制
void Chassis_Control_Update(float vx, float vy, float wz, float max_output)
{
    Chassis_Movement_Calculation(vx, vy, wz);

    for(int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        float target_rpm = chassis_motors[i].target_speed;
        float measured_rpm = (float)chassis_motors[i].feedback.actual_speed;

        chassis_motors[i].pid_speed.MaxOutput = max_output;

        output.target_currents[i] = (int16_t)PID_Calculate(&chassis_motors[i].pid_speed, target_rpm, measured_rpm);

        output.speed_errors_rpm[i] = chassis_motors[i].pid_speed.error;
    }
}




//记录功率控制后的最终电流
void Chassis_Set_Target_Currents(const int16_t* final_currents)
{
    for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
    {
        chassis_motors[i].target_current = final_currents[i];
    }
}



//映射电流
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



//解码底盘电机的CAN反馈帧
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


//底盘输出
const Chassis_Output_t* Chassis_Get_Output(void)
{
    return &output;
}


//获取底盘电机信息
const Motor_Controller* Chassis_Get_Motors(void)
{
    return chassis_motors;
}
