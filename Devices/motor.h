/**
 * @file motor.h
 * @brief 电机通用数据结构
 *
 * Motor_Feedback / Motor_Controller 为全项目共用的电机反馈与控制结构体,
 * 由 Gimbal / Shooter / Chassis 等控制模块持有。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "fdcan.h"
#include "pid.h"

//电机反馈结构体
typedef struct
{
    uint16_t angle;
    int16_t actual_speed;
    int16_t current;
    uint8_t temperature;
    
    float position;//位置(rad)
    float velocity;//速度(rad/s)
    float torque;  //力矩(Nm)
} Motor_Feedback;



//电机控制结构体
typedef struct
{
    uint16_t id;                  //电机ID
    FDCAN_HandleTypeDef* hfdcan;     //CAN总线
    Motor_Feedback feedback;     //电机反馈
    
    PID_Controller pid_angle;    //位置环PID
    float target_world_angle;    //目标世界角度
    
    float target_total_angle;    //目标总累计角度(编码器值)
    int32_t round_count;         //累计圈数
    uint16_t last_feedback_angle;//上一次的单圈反馈角度(0-8191)
    float current_total_angle;   //当前的总累计角度
    float mechanical_degree;     //当前的机械角度(度)

    
    PID_Controller pid_speed;    //速度环PID
    float target_speed;          //速度环目标速度
    
    int16_t target_current;      //目标电流值
	  float target_torque;         //目标力矩
	
	  //DM
		uint16_t state;              //电机状态
  	float last_feedback_position;  //上一次的反馈位置 (rad, -12.5 to 12.5)
	  float current_total_angle_rad; //当前总累计角度(rad)
    float current_total_angle_deg; //当前总累计角度(deg)
    
    uint32_t last_update_tick;   //上一次收到电机报文时间
    bool is_online;             //在线状态标志位
} Motor_Controller;

#endif
