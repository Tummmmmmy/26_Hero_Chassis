/**
 * @file pid.h
 * @brief 通用 PID 控制器
 */

#ifndef PID_H
#define PID_H


/**
 * @brief PID控制器
 */
typedef struct
{
	float Kp, Ki, Kd;

	float MaxOutput;
	float Integral_Limit;

	float integral_separation_threshold; //积分分离阈值

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


/**
 * @brief 初始化PID参数
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Integral_Limit, float MaxOutput);


/**
 * @brief 位置式PID计算
 */
float PID_Calculate(PID_Controller *pid, float target, float measured);


/** @brief 位置式PID(积分分离、微分先行) */
float PID_Cal_Enhanced(PID_Controller *pid, float target, float measured);


/**
 * @brief 增量式PID计算
 */
float PID_Cal_Increment(PID_Controller *pid, float target, float measured);

#endif
