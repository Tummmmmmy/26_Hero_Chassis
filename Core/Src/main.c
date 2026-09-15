/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>	
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp_power_meter.h"
#include "bsp_SuperCap.h"
#include "power_control.h"
#include "bsp_ano_protocol.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "RM_JudgeSystem.h"
#include "INS_task.h"
#include "Ramp.h"

#include "gimbal_link.h"
#include "robot_control.h"
#include "climb_control.h"
#include "chassis_control.h"
#include "can_transmit.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ABS(x) ((x) > 0 ? (x) : -(x))

#define PI 3.14159265358979f

const float DEG_TO_RAD = PI / 180.0f;
const float RAD_TO_DEG = 180.0f / PI;
const float ENCODER_TO_RAD = 2.0f * PI / 8192.0f;//编码值与弧度的转换系数
const float ENCODER_TO_DEG = 360.0f / 8192.0f;
const float DEG_TO_ENCODER = 8192.0f / 360.0f;


#define LK_CURRENT_TO_RAW  (2048.0f / 16.5f)  // Amps -> -2048~2048
#define LK_RAW_TO_CURRENT  (16.5f / 2048.0f)
#define LK_RAW_TO_DPS      1.0f               // 1 LSB = 1 degree/second
#define LK_DPS_TO_RPM      (1.0f / 6.0f)      // dps -> rpm


Motor_Controller gimbal_yaw_motor;


//双板通信CAN ID
#define RX_GIMBAL_COMMAND_ID 0x300
#define RX_CHASSIS_UI_ID 0x308

//接收的云台数据
volatile Chassis_Cmd_t chassis = {0,0,0,0,0,0,1,0,0};
volatile Chassis_UI_t UI_Data = {0,0,0,0};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//初始化电机
void Motor_Init(Motor_Controller* motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan);

//底盘控制循环
void Chassis_Control_Loop(void);

//CAN接收解码任务
void Can_Decode_Task(void const *argument);

//控制任务
void Chassis_Control_task(void const * argument);

//更新云台指令
void update_gimbal_command(const uint8_t *data);

//更新云台发来的UI数据包
void update_gimbal_ui_data(const uint8_t *data);

//更新云台Yaw电机
void update_gimbal_yaw(const uint8_t* data);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值
/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}




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

void Chassis_Control_Loop(void)
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
    if(chassis.mode == 0 || (ext_game_robot_state.InfoUpdataFlag != 0 && ext_game_robot_state.data.mains_power_chassis_output == 0))
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


//拨盘相关接收 CAN ID
#define DM_DIAL_RECEIVE_ID 0x17
#define RX_DIAL_CMD_ID 0x303
#define RX_DIAL_MODE_ID 0x301


//CAN接收解码任务
void Can_Decode_Task(void const *argument)
{
    CAN_Rx_Message_t rx_msg;

    for(;;)
    {
        if(osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
        {
            if(rx_msg.hfdcan->Instance == FDCAN1)
            {
								//双板通信
                if(rx_msg.can_id == RX_GIMBAL_COMMAND_ID) 
                {
                    update_gimbal_command(rx_msg.data);
                }
					
								//转发云台发来的拨盘速度控制指令
                else if(rx_msg.can_id == RX_DIAL_CMD_ID)
                {
                    Dial_Relay_Speed_Cmd(&hfdcan3, rx_msg.data);
                }
										/*	
								//转发云台发来的拨盘力矩控制指令
                else if(rx_msg.can_id == RX_DIAL_CMD_ID)
                {
									
									  FDCAN_TxHeaderTypeDef tx_header;
                    tx_header.Identifier = 0x07;
                    tx_header.IdType = FDCAN_STANDARD_ID;
                    tx_header.TxFrameType = FDCAN_DATA_FRAME;
                    tx_header.DataLength = FDCAN_DLC_BYTES_8;
                    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
                    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
                    tx_header.MessageMarker = 0;
                    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

                    if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) > 0)
                    {
                        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, rx_msg.data);
                    }
                }
								*/
								//转发云台发来的拨盘模式指令
                else if (rx_msg.can_id == RX_DIAL_MODE_ID)
                {
                    Dial_Relay_Mode_Cmd(&hfdcan3, rx_msg.data);
                }
								else if(rx_msg.can_id == RX_CHASSIS_UI_ID) 
								{
									  update_gimbal_ui_data(rx_msg.data);
								}
            }
            else if(rx_msg.hfdcan->Instance == FDCAN2)
            {
					 	     //底盘电机反馈
                 if(rx_msg.can_id >= 0x201 && rx_msg.can_id <= 0x204)
                 {
                     Chassis_Update_Feedback(rx_msg.data, rx_msg.can_id);
                 }
                //功率计反馈
                else if (rx_msg.can_id == 0x301)
                {
                    power_meter_decode(rx_msg.data);
                }
								//超电反馈
								else if (rx_msg.can_id == SUPERCAP_RX_ID)
								{
										supercap_rx_decode(rx_msg.data, rx_msg.dlc);
								}
            }
						//转发拨盘反馈给云台
             else if(rx_msg.hfdcan->Instance == FDCAN3)
            {
                if(rx_msg.can_id == DM_DIAL_RECEIVE_ID) 
                {
                    Dial_Relay_Feedback(&hfdcan1, rx_msg.data);
                }
            }
        }
    }
}





//控制任务
void Chassis_Control_task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
	
  for(;;)
  {
		
      Chassis_Control_Loop();
      
	    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}

}





void Print_Entry_Task(void const * argument)
{
		osDelay(2000); 
		
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(10);
		
		for(;;)
		{
//				VOFA_Print_Wave(10,

//				(float)chassis_motors[0].target_speed,
//				(float)chassis_motors[0].feedback.actual_speed,
//				(float)chassis_motors[0].target_current,

//				(float)chassis_motors[1].target_speed,
//				(float)chassis_motors[1].feedback.actual_speed,
//				(float)chassis_motors[1].target_current,

//				(float)chassis_motors[2].target_speed,
//				(float)chassis_motors[2].feedback.actual_speed,
//				(float)chassis_motors[2].target_current,

//				
//				(float)chassis_motors[3].feedback.actual_speed
//				);
//(float)chassis_motors[3].target_speed,
//(float)chassis_motors[3].target_current
				
				vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}
}








//云台发底盘数据
typedef union
{
    uint8_t data[8];
    
    __packed struct
    {
        int64_t  vx : 16;             //现在先16后面改成15(-16384 ~ 16383)（wz变速时不够）
        int64_t  vy : 16; 
        int64_t  wz : 16; 
        
        uint64_t mode           : 1;  //底盘状态(0=失能, 1=使能)
        uint64_t supercap_req   : 1;  //超电(0=关, 1=开)
        uint64_t climb_cmd      : 2;  //上坎指令(0~3)(0=全收, 1=前伸, 2=后伸)
        uint64_t led_cmd        : 1;  //LED指令
        uint64_t key_shift      : 1;  //Shift键状态(0=松开, 1=按下)
        uint64_t friction_state : 2;  // 0: 关, 1: 低速 (12m/s), 2: 高速 (16m/s)
			  uint64_t is_base_locked : 1;  //基地已锁定
			
			  uint64_t battle_mode    : 2;  //键鼠模式(0: 失能, 1:跟随, 2: 底盘锁 3: 吊射)
        
        uint64_t reserved       : 5; //剩5位
    } pack;
} Gimbal_To_Chassis_Msg_t;


//更新云台指令
void update_gimbal_command(const uint8_t *data)
{
    Gimbal_To_Chassis_Msg_t rx_msg;
    
    memcpy(rx_msg.data, data, 8);
    
    chassis.vx             = rx_msg.pack.vx;
    chassis.vy             = rx_msg.pack.vy;
    chassis.wz             = rx_msg.pack.wz;
    
    chassis.mode           = rx_msg.pack.mode; 
    chassis.supercap_req   = rx_msg.pack.supercap_req;
    chassis.climb_mode     = rx_msg.pack.climb_cmd;
    chassis.led_cmd        = rx_msg.pack.led_cmd;
    chassis.key_shift      = rx_msg.pack.key_shift; 
    chassis.friction_state = rx_msg.pack.friction_state;
	  chassis.is_base_locked = rx_msg.pack.is_base_locked;
	  chassis.battle_mode    = rx_msg.pack.battle_mode;
	
    chassis.cnt = 0; 
}




typedef union
{
    uint8_t data[8]; 
    
    __packed struct
    {
        uint64_t yaw_angle_x100   : 16; //Yawn轴机械角度
        int64_t  pitch_angle_x100 : 16; //Pitch俯仰机械角度

			  uint64_t friction_speed        : 13; //摩擦轮目标转速
			
        uint64_t reserved        : 19; //剩19位
    } pack;
} Gimbal_UI_Msg_t;




//更新云台发来的UI数据包
void update_gimbal_ui_data(const uint8_t *data)
{
    Gimbal_UI_Msg_t rx_ui;
    
    memcpy(rx_ui.data, data, 8);
    

    UI_Data.yaw_deg  = (float)rx_ui.pack.yaw_angle_x100 * 0.01f;
    UI_Data.pit_deg  = (float)rx_ui.pack.pitch_angle_x100 * 0.01f;
    UI_Data.friction_speed = rx_ui.pack.friction_speed;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_FDCAN1_Init();
  MX_USART10_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI6_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
