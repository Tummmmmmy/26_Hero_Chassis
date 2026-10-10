/**
 * @file power_control.c
 * @brief 底盘功率控制
 *
 */

#include "power_control.h"
#include "RM_JudgeSystem.h"
#include "bsp_SuperCap.h"
#include "gimbal_link.h"
#include <string.h>
#include <math.h>
#include "pid.h"

#define M3508_REDUCTION_RATIO (14.0f)//3508减速比
#define LOW_ERROR_MODE_THRESHOLD 50.0f

//功率模型与分配结果
typedef struct
{
    float final_power_limit;           //最终功率上限
    const int16_t* target_current_raw; //原始目标电流(-16384~16384)

    float predicted_total_power;       //预测总功率
    float allocated_total_power;       //最终分配总功率

    //模型参数
    float k1;
    float k2;
    float k3;

    int16_t final_current_raw[4];//功控后最终电流

} chassis_power_control_t;



//总功率控制内部状态
typedef struct
{
    //裁判系统数据
    float referee_power_limit;      //功率上限
    float referee_power_buffer;     //缓冲能量
    float allocated_power_last;     //上一周期分配出去的总功率

    //电容环
    PID_Controller buffer_pid;    //缓冲能量PID
    float buffer_pid_target;        //缓冲能量目标值
    //功率环
    chassis_power_control_t power_loop;

} power_control_t;



static power_control_t power_controller;



static float static_power;          //静态功率
static float final_cap_limit;       //超电限制功率
static float allocatable_power_budget;

//电容环PID参数
static float buffer_pid_kp = -20.0f;//能量与目标差1J，功率上限就变化20W
static float buffer_pid_ki = 0.0f;
static float buffer_pid_kd = -10.0f;
static float pid_max_i_out = 0.0f; //积分限幅
static float pid_max_output = 80.0f;//输出限幅


//计算最终电流值
static float calculate_current_by_power(float a, float b, float c, float target_direction);

//解析当前功率上限
static void Power_Resolve_Limit(void);

//计算最终功率上限
static void Power_Calc_Allocatable(void);

//计算总功率缩放系数
static float Power_Calc_Scale(float predicted_total_power, float budget);


//初始化功率控制
void Power_Control_Init(void)
{
    PID_Init(&power_controller.buffer_pid, buffer_pid_kp, buffer_pid_ki, buffer_pid_kd, pid_max_i_out, pid_max_output);
    power_controller.buffer_pid_target = 50.0f;//缓冲能量目标值

    //功率环参数
    power_controller.power_loop.k1 = 0.4112f;
    power_controller.power_loop.k2 = 0.1686f;
    power_controller.power_loop.k3 = 0.0358f;

    static_power = 4.0f;//静态功率
    power_controller.allocated_power_last = 0.0f;

    power_controller.power_loop.predicted_total_power = 0.0f;
    power_controller.power_loop.allocated_total_power = 0.0f;
    memset(power_controller.power_loop.final_current_raw, 0, sizeof(power_controller.power_loop.final_current_raw));
}


//解析当前功率上限
static void Power_Resolve_Limit(void)
{
    if(supercap_is_output_enabled())
    {
        //获取超电电量
        uint8_t cap_pct = supercap_get_cap_level();

        if(cap_pct > 50)
        {
            final_cap_limit = 390.0f;//比赛390     日常300
        }
        else if(cap_pct > 30)
        {
            final_cap_limit = 300.0f;//比赛300         220
        }
        else if(cap_pct > 10)
        {
            final_cap_limit = 250.0f;//比赛250         200
        }
        else
        {
            final_cap_limit = 150.0f;
        }

        //上坎
        if(chassis.climb_mode == 2 || chassis.climb_mode == 1)
        {
            final_cap_limit = 130.0f;
        }


        power_controller.referee_power_limit = final_cap_limit;
    }
    else power_controller.referee_power_limit = ext_game_robot_state.data.chassis_power_limit;
}



//计算最终功率上限
static void Power_Calc_Allocatable(void)
{
    chassis_power_control_t* loop = &power_controller.power_loop;

    //电容环(缓冲能量PID计算最终功率上限)
    if(!Judge_IsOnline())//裁判系统离线
    {
        loop->final_power_limit = 100.0f;
    }
    else
    {
        float buffer_pid_output = PID_Calculate(&power_controller.buffer_pid, power_controller.buffer_pid_target, power_controller.referee_power_buffer);

        //最终功率上限 = 裁判系统功率上限 + 缓冲能量PID调整量
        loop->final_power_limit = power_controller.referee_power_limit + buffer_pid_output;
    }

    allocatable_power_budget = loop->final_power_limit - static_power;
    if(allocatable_power_budget < 2.0f) allocatable_power_budget = 2.0f;
}


//计算总功率缩放系数
static float Power_Calc_Scale(float predicted_total_power, float budget)
{
    float power_scale = 1.0f;

    if(predicted_total_power > budget)//预测大于可分配预算
    {
        if(predicted_total_power > 1.0f)
        {
            power_scale = budget / predicted_total_power;
        }
        else power_scale = 0.0f;
    }

    return power_scale;
}


//功率控制
void Power_Control_Update(const Motor_Controller* motors, const int16_t* target_current, const float* speed_error_rpm)
{
    chassis_power_control_t* loop = &power_controller.power_loop;
    power_controller.referee_power_buffer = ext_power_heat_data.data.chassis_power_buffer;

	
    //1.解析当前功率上限
    Power_Resolve_Limit();

    loop->target_current_raw = target_current;


    //2.电容环(缓冲能量PID计算最终功率上限)
    Power_Calc_Allocatable();


	
    //3.功率环(差速分配)
    float target_current_A[4] = {0.0f};
    float actual_speed_rps[4] = {0.0f};
    float predicted_power[4] = {0.0f};
    loop->predicted_total_power = 0.0f;

    for(int i = 0; i < 4; i++)
    {
        target_current_A[i] = (float)loop->target_current_raw[i] * (20.0f / 16384.0f);//16384电流转I
        actual_speed_rps[i] = (float)motors[i].feedback.actual_speed / M3508_REDUCTION_RATIO * 0.104719755f;//RPM转RPS ,2 * PI / 60 = 0.104719755f

        //预测功率
        predicted_power[i] = loop->k1 * actual_speed_rps[i] * target_current_A[i] +
                             loop->k2 * target_current_A[i] * target_current_A[i] +
                             loop->k3 * actual_speed_rps[i] * actual_speed_rps[i];

        loop->predicted_total_power += predicted_power[i];
    }


    float total_output_speed_error_rpm = 0.001f;
    for(int i = 0; i < 4; i++)
    {
        total_output_speed_error_rpm += fabsf(speed_error_rpm[i] / M3508_REDUCTION_RATIO);
    }

    int is_low_error_mode = (total_output_speed_error_rpm < LOW_ERROR_MODE_THRESHOLD);


    //计算总功率缩放系数
    float power_scale = Power_Calc_Scale(loop->predicted_total_power, allocatable_power_budget);

		
    //根据差速选择不同的分配策略
    float final_power[4] = {0.0f};
    if(is_low_error_mode)
    {
        //等比例缩放
        for(int i = 0; i < 4; i++)
        {
            final_power[i] = predicted_power[i] * power_scale;
        }
    }
    else//差速分配
    {
        //计算缩放后的总功率
        float total_scaled_predicted_power = 0.0f;
        for(int i = 0; i < 4; i++)
        {
            if(predicted_power[i] > 0)
            {
                total_scaled_predicted_power += predicted_power[i] * power_scale;
            }
        }

        //差速分配
        for(int i = 0; i < 4; i++)
        {
            float scaled_power = predicted_power[i] * power_scale;

            if(scaled_power <= 0)
            {
                final_power[i] = scaled_power;//刹车轮直接应用缩放后的负功率
            }
            else
            {
                float error_proportion = fabsf(speed_error_rpm[i] / M3508_REDUCTION_RATIO) / total_output_speed_error_rpm;
                final_power[i] = total_scaled_predicted_power * error_proportion;
            }
        }
    }


    power_controller.allocated_power_last = loop->predicted_total_power;
    loop->allocated_total_power = 0;
    for(int i=0; i<4; ++i) loop->allocated_total_power += final_power[i];


    //4.计算最终电流
    for(int i = 0; i < 4; i++)
    {
        //功率模型: P = k1*ω*I + k2*I^2 + k3*ω^2
        //整理为一元二次方程: a*I^2 + b*I + c = 0
        //a = k2
        //b = k1 * ω
        //c = k3*ω^2 - P_final
        float w = actual_speed_rps[i];
        float p_final = final_power[i];


        float a = loop->k2;
        float b = loop->k1 * w;
        float c = loop->k3 * w * w - p_final;

        //计算最终电流I
        float final_current_A = calculate_current_by_power(a, b, c, target_current_A[i]);

        if(target_current_A[i] >= 0 && final_current_A < 0) final_current_A = 0;
        if(target_current_A[i] < 0 && final_current_A > 0) final_current_A = 0;
        if(final_current_A > 20.0f) final_current_A = 20.0f;
        if(final_current_A < -20.0f) final_current_A = -20.0f;

        loop->final_current_raw[i] = (int16_t)(final_current_A * (16384.0f / 20.0f));
    }

    //安全检查
    for(int i = 0; i < 4; i++)
    {
        if(loop->final_current_raw[i] > 16384) loop->final_current_raw[i] = 16384;
        else if(loop->final_current_raw[i] < -16384) loop->final_current_raw[i] = -16384;
    }
}



//计算最终电流值
static float calculate_current_by_power(float a, float b, float c, float target_direction)
{
    if(fabsf(a) < 1e-6f)
    {
        if(fabsf(b) > 1e-6f)
        {
            return -c / b;
        }
        return 0.0f;
    }

    float delta = b * b - 4 * a * c;
    if(delta < 0) return 0.0f;//无实数解


    //求解
    float sqrt_delta = sqrtf(delta);
    float sol1 = (-b + sqrt_delta) / (2.0f * a);
    float sol2 = (-b - sqrt_delta) / (2.0f * a);

    if(target_direction >= 0)//期望正电流
    {
        //判断是否正电流
        int sol1_valid = (sol1 >= -1e-6f);
        int sol2_valid = (sol2 >= -1e-6f);

        if(sol1_valid && sol2_valid)
        {
            return fminf(sol1, sol2);//选择更节能的小电流
        }
        else if(sol1_valid) return sol1;
        else if(sol2_valid) return sol2;
        else return 0.0f;
    }
    else//期望负电流
    {
        //判断是否负电流
        int sol1_valid = (sol1 <= 1e-6f);
        int sol2_valid = (sol2 <= 1e-6f);

        if(sol1_valid && sol2_valid)
        {
            return fmaxf(sol1, sol2);//选择更节能的小电流
        }
        else if (sol1_valid) return sol1;
        else if (sol2_valid) return sol2;
        else return 0.0f;
    }
}


//获取功率控制后的最终电流
void Power_Control_Get_Currents(int16_t* output_current)
{
    memcpy(output_current, power_controller.power_loop.final_current_raw, 4 * sizeof(int16_t));
}
