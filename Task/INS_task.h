/**
 * @file INS_task.h
 * @brief IMU 姿态解算任务接口与 IMU_Data_t 定义
 */
 
#ifndef INS_TASK__H
#define INS_TASK__H

#include "main.h"

extern float total_yaw;
extern float pitch;
extern float roll;
extern uint8_t attitude_flag;
extern volatile uint8_t imu_ready_flag; //0:未就绪, 1:已就绪
extern float gyro[3];
extern float accel[3]; 

void IMU_task(void const * argument);

#endif
