/**
 * @file pid.c
 * @brief PID 控制器实现
 */
#include "pid.h"
#include <string.h>
#include <math.h>

static float separation_threshold = 0.0f;


//初始化PID参数
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Integral_Limit, float MaxOutput)
{
    memset(pid, 0, sizeof(PID_Controller));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->MaxOutput = MaxOutput;
    pid->Integral_Limit = Integral_Limit;

    pid->integral_separation_threshold = separation_threshold;//积分分离阈值
}


//位置式PID计算
float PID_Calculate(PID_Controller *pid, float target, float measured )
{
    pid->target = target;
    pid->measured = measured;
    pid->error = pid->target - pid->measured;

    pid->p_out = pid->Kp * pid->error;

    pid->i_out += pid->Ki * pid->error;

	  //积分限幅
    if(pid->i_out > pid->Integral_Limit) pid->i_out = pid->Integral_Limit;
    else if(pid->i_out < -pid->Integral_Limit) pid->i_out = -pid->Integral_Limit;

    pid->d_out = pid->Kd * (pid->error - pid->pre_error);

    float output = pid->p_out + pid->i_out + pid->d_out;

	  //输出限幅
    if(output > pid->MaxOutput) {output = pid->MaxOutput;}
    else if(output < -pid->MaxOutput) {output = -pid->MaxOutput;}

    pid->pre_error = pid->error;
    return output;
}



//位置式PID(积分分离、微分先行)
float PID_Cal_Enhanced(PID_Controller *pid, float target, float measured)
{
	  pid->target = target;
    pid->measured = measured;
    pid->error = target - measured;

    pid->p_out = pid->Kp * pid->error;

    //积分分离
    if(fabs(pid->error) < pid->integral_separation_threshold)
    {
        pid->i_out += pid->Ki * pid->error;

        //积分限幅
        if(pid->i_out > pid->Integral_Limit) pid->i_out = pid->Integral_Limit;
        else if (pid->i_out < -pid->Integral_Limit) pid->i_out = -pid->Integral_Limit;
    }

    //微分先行
    pid->d_out = pid->Kd * (pid->pre_measured - measured); 

    pid->output = pid->p_out + pid->i_out + pid->d_out;

    pid->pre_error = pid->error;
    pid->pre_measured = measured;

    if(pid->output > pid->MaxOutput) pid->output = pid->MaxOutput;
    else if (pid->output < -pid->MaxOutput) pid->output = -pid->MaxOutput;

    return pid->output;
}



//增量式PID计算
float PID_Cal_Increment(PID_Controller *pid, float target, float measured)
{
    pid->target = target;
    pid->measured = measured;
    pid->error = pid->target - pid->measured;

    pid->p_out = pid->Kp * (pid->error - pid->pre_error);
    pid->i_out = pid->Ki * pid->error;
    pid->d_out = pid->Kd * (pid->error - 2.0f * pid->pre_error + pid->err_beforeLast);

    float delta_output = pid->p_out + pid->i_out + pid->d_out;

    pid->output += delta_output;

    //输出限幅
    if (pid->output > pid->MaxOutput) pid->output = pid->MaxOutput;
    if (pid->output < -pid->MaxOutput) pid->output = -pid->MaxOutput;

    pid->err_beforeLast = pid->pre_error;
    pid->pre_error = pid->error;

    return pid->output;
}