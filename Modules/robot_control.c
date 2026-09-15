/**
 * @file robot_control.c
 * @brief 底盘板整机调度实现
 *
 * 底盘板的总控制流程(1ms): 失能检查 -> 使能切换 -> 上坎 -> 底盘运动
 * -> 功率控制 -> 统一 CAN 输出; 上电初始化由 Robot_Param_Init 统一完成。
 */

#include "robot_control.h"
#include "main.h"
#include "chassis_control.h"
#include "climb_control.h"
#include "gimbal_link.h"
#include "can_transmit.h"
#include "power_control.h"
#include "bsp_SuperCap.h"
#include "bsp_power_meter.h"
#include "bsp_ano_protocol.h"
#include "RM_JudgeSystem.h"
#include <string.h>

//系统LED状态(device_task 通过 Robot_Get_Led_State 读取)
static volatile LED_State_e s_led_state = LED_BOOTING;

//获取系统LED状态
LED_State_e Robot_Get_Led_State(void)
{
    return s_led_state;
}


//设置系统LED状态
void Robot_Set_Led_State(LED_State_e state)
{
    s_led_state = state;
}


Motor_Controller gimbal_yaw_motor;


//初始化电机
void Motor_Init(Motor_Controller* motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan)
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


//初始化电机参数
void Robot_Param_Init(void)
{
    //初始化底盘电机
    Chassis_Control_Init();
		
		//Yaw(LK)
    Motor_Init(&gimbal_yaw_motor, 1 , &hfdcan1);
}

#define DATA_ACQUISITION_MODE 0 // 1: 开启数据采集, 0: 关闭

//1ms 整机控制循环(由 Chassis_Control_task 调用)
void Robot_Control_Loop(void)
{
	  static uint8_t last_mode = 0; 
	
    chassis.cnt++;
	
	
		//发送双板通信
		static uint32_t C = 0;
    C++;
		if(C % 100 == 0)
    {
				Send_Gimbal_Data(&hfdcan1); 
    }

		
		
		//失能
    if(chassis.mode == 0 || !Gimbal_Link_Is_Online() || (ext_game_robot_state.InfoUpdataFlag != 0 && ext_game_robot_state.data.mains_power_chassis_output == 0))
    {
        Chassis_On_Robot_Off();
        DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
				
				Climb_Control_Retract_All();//前后全收
				
				Robot_Set_Led_State(LED_DISABLED); // 失能 -> 红色
        last_mode = 0;
				
        return;
    }

		//重新使能
    if(last_mode == 0 && chassis.mode == 1)
    {
        Chassis_On_Robot_Start();
				power_control_init();
    }
    last_mode = 1;
		
	

    float vx = (float)chassis.vx * 0.01f;//还原数据
    float vy = (float)chassis.vy * 0.01f;
    float wz = (float)chassis.wz * 0.01f;
		
		
    //上坎
    Climb_Control_Update(chassis.climb_mode);
		
		//上坎/使能状态指示
    switch(Climb_Get_Phase())
    {
        case CLIMB_BACK_EXTEND:
            Robot_Set_Led_State(LED_CLIMB_BACK);
            break;

        case CLIMB_FRONT_EXTEND:
            Robot_Set_Led_State(LED_CLIMB_FRONT);
            break;

        default:
            if(chassis.led_cmd == 0) Robot_Set_Led_State(LED_CLIMB_ACTIVE);
            else Robot_Set_Led_State(LED_ENABLED);
            break;
    }
		
		//最大输出电流
		float current_max_output = 12000.0f;
    if(!g_supercap.offline_flag && g_supercap.tx_data.pack.is_cap_output == SUPERCAP_OUTPUT_ENABLE)
    {
        current_max_output = 15000.0f; 
    }
		
		//麦轮解算 + 速度环
    Chassis_Control_Update(vx, vy, wz, current_max_output);
		
    const Chassis_Output_t* chassis_out = Chassis_Get_Output();
		
		#if DATA_ACQUISITION_MODE == 1//数据采集模式
				
				static uint32_t printf_counter = 0;
				if (++printf_counter >= 5)
				{
						printf_counter = 0;
						
						const Motor_Controller* motors = Chassis_Get_Motors();
						float p_actual = power_meter.power;
						float sum_wI = 0.0f, sum_I2 = 0.0f, sum_w2 = 0.0f;
						for (int i = 0; i < 4; i++)
						{
								float i_actual_A = (float)motors[i].feedback.current * (20.0f / 16384.0f);
								float w_actual_rps = (float)motors[i].feedback.actual_speed / 19.0f * 0.104719755f;
								sum_wI += w_actual_rps * i_actual_A;
								sum_I2 += i_actual_A * i_actual_A;
								sum_w2 += w_actual_rps * w_actual_rps;
						}
						VOFA_Print_Wave(4, p_actual, sum_wI, sum_I2, sum_w2);
				}
				
				//数据采集模式不经功率控制, 直接发送速度环原始电流
				int16_t current_to_send[CHASSIS_MOTOR_NUM] = {0};
				Chassis_Map_Currents_To_Physical(chassis_out->target_currents, current_to_send);
				DJI_Motor_Send_Command(&hfdcan2, 0x200, current_to_send[0], current_to_send[1], current_to_send[2], current_to_send[3]);
				return;
		#endif

    //功率控制
    power_control_run(Chassis_Get_Motors(), chassis_out->target_currents, chassis_out->speed_errors_rpm);

    //获取最终电流
    int16_t final_currents[CHASSIS_MOTOR_NUM] = {0};
    get_final_chassis_current(final_currents);

    //发送电流
    Chassis_Set_Target_Currents(final_currents);
    int16_t currents_to_send[CHASSIS_MOTOR_NUM] = {0};
    Chassis_Map_Currents_To_Physical(final_currents, currents_to_send);
		DJI_Motor_Send_Command(&hfdcan2, 0x200, currents_to_send[0], currents_to_send[1], currents_to_send[2], currents_to_send[3]);
}
