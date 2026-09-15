/**
 * @file climb_control.c
 * @brief 气动上坎状态机实现
 *
 * 前/后气缸的伸缩状态机与 IO 控制。模块对外只给出当前上坎相位,
 * LED 显示由整机调度层决定。
 */
#include "climb_control.h"
#include "main.h"

//当前上坎相位
static Climb_Phase_e climb_state = CLIMB_ALL_RETRACT;


static inline void Front_Cylinder_Extend(void)
{
    HAL_GPIO_WritePin(FRONT_GPIO_Port, FRONT_Pin, GPIO_PIN_SET);
}

static inline void Front_Cylinder_Retract(void)
{
    HAL_GPIO_WritePin(FRONT_GPIO_Port, FRONT_Pin, GPIO_PIN_RESET);
}

static inline void Back_Cylinder_Extend(void)
{
    HAL_GPIO_WritePin(BACK_GPIO_Port, BACK_Pin, GPIO_PIN_SET);
}

static inline void Back_Cylinder_Retract(void)
{
    HAL_GPIO_WritePin(BACK_GPIO_Port, BACK_Pin, GPIO_PIN_RESET);
}


//气动上坎
void Climb_Control_Update(uint8_t climb_mode)
{
    static bool has_extend = 0;
    bool sensor_state = 0;
	
	  //判断动作
    if(climb_mode == 0)
		{
        climb_state = CLIMB_ALL_RETRACT;//全收
    }
		else if(climb_mode == 1)
		{
        if(climb_state == CLIMB_ALL_RETRACT)
				{
            climb_state = CLIMB_FRONT_EXTEND;
        }
    }
		else if(climb_mode == 2)
		{
        climb_state = CLIMB_BACK_EXTEND;//后伸
    } 

	
    switch(climb_state)
    {
        case CLIMB_BACK_EXTEND: //UP_前收后伸
					
			    	sensor_state = HAL_GPIO_ReadPin(CEJU_GPIO_Port, CEJU_Pin);
				
            Front_Cylinder_Retract();//前收
				
            if(sensor_state)//亮一个是1，亮两个触发是0
						{
							if(has_extend == 0) 
							{
									Back_Cylinder_Extend();//后伸
									has_extend = 1;
							}
            }
						else
						{
                Back_Cylinder_Retract();//后收
            }
            break;
			
        case CLIMB_FRONT_EXTEND: //MID_前伸后收
				
            Front_Cylinder_Extend();//前伸
            Back_Cylinder_Retract();//后收
            
            has_extend = 0;
            break;

        case CLIMB_ALL_RETRACT: //DOWM_全收
        default:
				
            Front_Cylinder_Retract();//前收
            Back_Cylinder_Retract(); //后收
            
            has_extend = 0;
            break;
    }
}


//前后气缸全收(整机失能时调用)
void Climb_Control_Retract_All(void)
{
    Front_Cylinder_Retract();//前收
    Back_Cylinder_Retract(); //后收
}


//获取当前上坎相位
Climb_Phase_e Climb_Get_Phase(void)
{
    return climb_state;
}
