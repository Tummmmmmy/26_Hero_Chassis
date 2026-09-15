/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "fdcan.h"
#include "pid.h"
#include <stdbool.h>

extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t canTxQueueHandle;

//CAN接收队列数据
typedef struct
{
    FDCAN_HandleTypeDef* hfdcan;
    uint32_t can_id;
    uint8_t data[8];
  	uint8_t  dlc;
} CAN_Rx_Message_t;



//电机反馈结构体
typedef struct
{
    uint16_t angle;
    int16_t actual_speed;
    int16_t current;
    uint8_t temperature;
    
    float position;//位置(rad)
    float velocity;//速度(rad/s)
    float torque;  //力矩(Nm)
} Motor_Feedback;



//电机控制结构体
typedef struct
{
    uint16_t id;                  //电机ID
    FDCAN_HandleTypeDef* hfdcan;     //CAN总线
    Motor_Feedback feedback;     //电机反馈
    
    PID_Controller pid_angle;    //位置环PID
    float target_world_angle;    //目标世界角度
    
    float target_total_angle;    //目标总累计角度(编码器值)
    int32_t round_count;         //累计圈数
    uint16_t last_feedback_angle;//上一次的单圈反馈角度(0-8191)
    float current_total_angle;   //当前的总累计角度
    float mechanical_degree;     //当前的机械角度(度)

    
    PID_Controller pid_speed;    //速度环PID
    float target_speed;          //速度环目标速度
    
    int16_t target_current;      //目标电流值
	  float target_torque;         //目标力矩
	
	  //DM
		uint16_t state;              //电机状态
  	float last_feedback_position;  //上一次的反馈位置 (rad, -12.5 to 12.5)
	  float current_total_angle_rad; //当前总累计角度(rad)
    float current_total_angle_deg; //当前总累计角度(deg)
    
    uint32_t last_update_tick;   //上一次收到电机报文时间
    bool is_online;             //在线状态标志位
} Motor_Controller;

//初始化
extern void Robot_Param_Init(void);

//FDCAN通用配置
extern void BSP_FDCAN_Config(void);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define FRONT_Pin GPIO_PIN_0
#define FRONT_GPIO_Port GPIOA
#define BACK_Pin GPIO_PIN_2
#define BACK_GPIO_Port GPIOA
#define CEJU_Pin GPIO_PIN_9
#define CEJU_GPIO_Port GPIOE
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
