/**
 * @file climb_control.h
 * @brief 气动上坎状态机接口
 *
 * 前/后气缸的伸缩状态机与 IO 控制。模块对外只给出当前上坎相位,
 * LED 显示由整机调度层决定。
 */
#ifndef CLIMB_CONTROL_H
#define CLIMB_CONTROL_H

#include <stdint.h>

typedef enum {
    CLIMB_ALL_RETRACT,  //全收
    CLIMB_FRONT_EXTEND, //前伸
    CLIMB_BACK_EXTEND   //后伸
} Climb_Phase_e;

//按上坎指令驱动前后气缸(1ms 周期调用)
//climb_mode: 0=全收, 1=前伸, 2=前收后伸
void Climb_Control_Update(uint8_t climb_mode);

//前后气缸全收(整机失能时调用)
void Climb_Control_Retract_All(void);

//获取当前上坎相位
Climb_Phase_e Climb_Get_Phase(void);

#endif
