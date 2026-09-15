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

#endif
