#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bsp_power_meter.h"
#include <stdint.h>
#include "main.h"
#include "pid.h"

#define USE_RLS_ONLINE_IDENTIFICATION 0//0:固定参数 ,1:RLS 



//底盘功率控制结构体
typedef struct
{
    float final_power_limit;           //最终功率上限
    const int16_t* target_current_raw; //原始目标电流(-16384~16384)
//    const float* speed_error_rpm;      //电机速度误差
//    const int16_t* actual_speed_rpm;   //电机实际转速
    
    float predicted_total_power;       //预测总功率
    float allocated_total_power;       //最终分配总功率

    //模型参数
    float k1;
    float k2;
    float k3;

    int16_t final_current_raw[4];//功控后最终电流

} chassis_power_control_t;




//总功率控制结构体
typedef struct
{
    //裁判系统数据
    float referee_power_limit;      //功率上限
    float referee_power_buffer;     //缓冲能量
    uint8_t referee_offline_flag;   //离线标志

    //电容环
    PID_Controller buffer_pid;    //缓冲能量PID
    float buffer_pid_target;        //缓冲能量目标值
    //功率环
    chassis_power_control_t power_loop;
    
} power_control_t;


extern power_control_t power_controller;
extern float allocated_power_last;
extern float allocatable_power_budget;
extern uint8_t off; 

//初始化功率控制
void power_control_init(void);

//功率控制
void power_control_run(const Motor_Controller* motors, const int16_t* target_current, const float* speed_error_rpm );

//获取功率控制后的最终电流
void get_final_chassis_current(int16_t* output_current);


#endif //POWER_CONTROL_H
