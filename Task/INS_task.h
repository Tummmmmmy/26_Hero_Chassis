#ifndef INS_TASK__H
#define INS_TASK__H

#include "main.h"

extern float total_yaw;
extern float pitch;
extern float roll;
extern uint8_t attitude_flag;
extern volatile uint8_t imu_ready_flag; //0:Î´¾ÍÐ÷, 1:ÒÑ¾ÍÐ÷
extern float gyro[3];
extern float accel[3]; 

void IMU_task(void const * argument);

#endif
