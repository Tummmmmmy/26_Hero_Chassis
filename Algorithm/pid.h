/**
 * @file pid.h
 * @brief 项目统一 PID 控制器
 *
 * 原来移植 IMU 时残留的哈工程 PID（pid_struct_t 等）已经移除，
 * 全项目只保留这一套 PID 实现。
 */

#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp, Ki, Kd;

    float MaxOutput;
    float Integral_Limit;

    float integral_separation_threshold;
    float dead_band;

    float target;
    float measured;
    float error;
    float pre_error;
    float pre_measured;
    float err_beforeLast;

    float p_out;
    float i_out;
    float d_out;
    float output;
} PID_Controller;

//初始化PID
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Integral_Limit, float MaxOutput);

//位置式pid计算
float PID_Calculate(PID_Controller *pid, float target, float measured);

//积分分离 + 微分先行的位置式pid
float PID_Cal_Enhanced(PID_Controller *pid, float target, float measured);

//增量式pid
float PID_Cal_Increment(PID_Controller *pid, float target, float measured);

#endif
