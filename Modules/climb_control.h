/**
 * @file climb_control.h
 * @brief 气动上坎状态机接口
 *
 */
#ifndef CLIMB_CONTROL_H
#define CLIMB_CONTROL_H

#include <stdint.h>

typedef enum {
    CLIMB_ALL_RETRACT,  //全收
    CLIMB_FRONT_EXTEND, //前伸
    CLIMB_BACK_EXTEND   //后伸
} Climb_Phase_e;

//气动上坎控制
void Climb_Control_Update(uint8_t climb_mode);

//前后气缸全收
void Climb_Control_Retract_All(void);

//获取当前上坎状态
Climb_Phase_e Climb_Get_Phase(void);

#endif
