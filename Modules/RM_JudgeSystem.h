/**
 * @file RM_JudgeSystem.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _RM_JUDGESYSTEM_H
#define _RM_JUDGESYSTEM_H

#include "dma.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern void JudgeSystem_Init(UART_HandleTypeDef *huart);
extern void JudgeSystem_Callback(uint8_t *buffer, uint16_t size);


typedef struct
{
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} RM_Judge_t;
extern RM_Judge_t RM_Judge;

float GetVision_CP(void);

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);


/**
  * @brief  判断裁判系统当前是否在线
  * @retval true: 在线, false: 离线
  */
bool Judge_IsOnline(void);



//裁判系统预编译
#define JUDGE_20 20
#define JUDGE_25 25
#define JUDGE_VERSION JUDGE_25

#define JUDGE_DATA_ERROR 0
#define JUDGE_DATA_CORRECT 1
/*-------------------------------2020------------------------------------*/

/*-------------------------------2025------------------------------------*/
//#elif JUDGE_VERSION == JUDGE_25
//2025 裁判系统官方接口协议

//对应通信协议格式   frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
#define       LEN_HEADER        5/*frame_header*/
#define       LEN_CMDID         2/*cmd_id*/
#define       LEN_TAIL          2/*frame_tail*/
#define       LEN_Frame_Header  9

//起始字节，协议固定为0xA5
#define      JUDGE_FRAME_HEADER            0xA5
#define      JUDGESYSTEM_PACKSIZE 		    389u		//裁判系统包大小(354+35)


//裁判系统离线判定超时阈值
#define JUDGE_TIMEOUT_MS     1000u 



//红蓝方
#define BLUE 0
#define RED  1

//信息传输
#define TRUE 1
#define FALSE 0


/***********************************命令码ID*************************************/
/*

ID: 0x0001          Byte: 11      比赛状态数据                   发送频率 1Hz
ID: 0x0002          Byte: 1       比赛结果数据                   比赛结束后发送
ID: 0x0003          Byte: 32      比赛机器人血量数据             发送频率 1Hz
ID: 0x0004          Byte: 3       飞镖发射状态                   飞镖发射后发送
ID: 0x0005          Byte: 11      人工智能挑战赛加成与惩罚       发送频率 1Hz
ID: 0x0101          Byte: 4       场地事件数据                   事件改变后发送
ID: 0x0102          Byte: 4       场地补给站动作标识数据         动作改变后发送
//ID: 0X0103          Byte: 2       场地补给站预约子弹数据         参赛队发送，上限10Hz（RM对抗赛未开放） 
ID: 0X0104          Byte: 2       裁判警告数据                   警告发生后发送
ID: 0x0105          Byte: 1       飞镖发射口倒计时               发送频率 1Hz
ID: 0X0201          Byte: 27      机器人状态数据                 发送频率 10Hz
ID: 0X0202          Byte: 16      实时功率热量数据               发送频率 50Hz
ID: 0X0203          Byte: 16      机器人位置数据                 发送频率 10Hz
ID: 0X0204          Byte: 1       机器人增益数据                 增益状态改变后发送
ID: 0X0205          Byte: 2       空中机器人能量状态数据         发送频率 10Hz，只有空中机器人主控发送
ID: 0X0206          Byte: 1       伤害状态数据                   伤害发生后发送
ID: 0X0207          Byte: 7       实时射击数据                   子弹发射后发送
ID: 0X0208          Byte: 6       子弹剩余发送数                 无人机与哨兵发送  发送频率 1Hz
ID: 0X0209          Byte: 4       机器人RFID状态                 发送频率 1Hz
ID: 0x020A          Byte: 12      飞镖机器人客户端指令书         发送频率 10Hz
ID: 0X0301          Byte: n       机器人间交互数据               发送方触发发送    上限频率 10Hz
ID: 0x0302          Byte: n       自定义控制器交互数据接口       客户端触发发送    上限频率 30Hz
ID: 0x0303          Byte: 15      客户端小地图交互数据           触发发送
ID: 0x0304          Byte: 12      键盘、鼠标信息                 通过图传串口发送

*/

#define       Judge_Game_StatusData_01_ID              0x0001 //比赛状态数据
#define       Judge_Game_ResultData_02_ID              0x0002 //比赛结果数据
#define       Judge_Robot_HP_03_ID                     0x0003 //机器人血量数据
#define       Judge_Event_Data_101_ID                  0x0101 //场地事件数据
#define       Judge_Referee_Warning_104_ID             0x0104 //裁判警告数据，己方判罚/判负时触发发送
#define       Judge_Dart_Countdown_105_ID              0x0105 //飞镖发射相关数据
#define       Judge_Robot_State_201_ID                 0x0201 //机器人性能体系数据
#define       Judge_Power_Heat_202_ID                  0x0202 //实时底盘缓冲能量和射击热量数据
#define       Judge_Robot_Position_203_ID              0x0203 //机器人位置数据
#define       Judge_Robot_Buff_204_ID                  0x0204 //机器人增益和底盘能量数据
#define       Judge_Injury_State_206_ID                0x0206 //伤害状态数据
#define       Judge_RealTime_Shoot_207_ID              0x0207 //实时射击数据
#define       Judge_Remaining_Rounds_208_ID            0x0208 //允许发弹量
#define       Judge_Robot_RFID_209_ID                  0x0209 //机器人RFID模块状态
#define       Judge_Dart_Client_20A_ID                 0x020A //飞镖选手端指令数据
#define       Judge_Ground_Position_20B_ID             0x020B //地面机器人位置数据
#define       Judge_Radar_Mark_20C_ID                  0x020C //雷达标记进度数据
#define       Judge_Sentry_Info_20D_ID                 0x020D //哨兵自主决策信息同步
#define       Judge_Radar_Info_20E_ID                  0x020E //雷达自主决策信息同步
#define       Judge_Robot_Communicate_301_ID           0x0301 //机器人交互数据，发送方触发发送
#define       Judge_User_Defined_302_ID                0x0302 //自定义控制器与机器人交互数据，发送方触发发送
#define       Judge_Map_Interaction_303_ID             0x0303 //选手端小地图交互数据
#define       Judge_KeyMouse_Message_304_ID            0x0304 //键鼠遥控数据
#define       Judge_Map_Robot_305_ID                   0x0305 //选手端小地图接收雷达数据
#define       Judge_Custom_Client_306_ID               0x0306 //自定义控制器与选手端交互数据，发送方触发发送
#define       Judge_Map_Data_307_ID                    0x0307 //选手端小地图接收哨兵数据
#define       Judge_Custom_Info_308_ID                 0x0308 //选手端小地图接收机器人数据
#define       Judge_Robot_Custom_309_ID                0x0309 //自定义控制器接收机器人数据


/***************************DATA_Length*************************/

#define       Judge_Game_StatusData_01_BufLen              11 //比赛状态数据
#define       Judge_Game_ResultData_02_BufLen              1  //比赛结果数据
#define       Judge_Robot_HP_03_BufLen                     16 //机器人血量数据
#define       Judge_Event_Data_101_BufLen                  4  //场地事件数据
#define       Judge_Referee_Warning_104_BufLen             3  //裁判警告数据，己方判罚/判负时触发发送
#define       Judge_Dart_Countdown_105_BufLen              3  //飞镖发射相关数据
#define       Judge_Robot_State_201_BufLen                 17 //机器人性能体系数据
#define       Judge_Power_Heat_202_BufLen                  14 //实时底盘缓冲能量和射击热量数据
#define       Judge_Robot_Position_203_BufLen              12 //机器人位置数据
#define       Judge_Robot_Buff_204_BufLen                  7  //机器人增益和底盘能量数据
#define       Judge_Injury_State_206_BufLen                1  //伤害状态数据
#define       Judge_RealTime_Shoot_207_BufLen              7  //实时射击数据
#define       Judge_Remaining_Rounds_208_BufLen            8  //允许发弹量
#define       Judge_Robot_RFID_209_BufLen                  5  //机器人RFID模块状态
#define       Judge_Dart_Client_20A_BufLen                 6  //飞镖选手端指令数据
#define       Judge_Ground_Position_20B_BufLen             40 //地面机器人位置数据
#define       Judge_Radar_Mark_20C_BufLen                  2  //雷达标记进度数据
#define       Judge_Sentry_Info_20D_BufLen                 6  //哨兵自主决策信息同步
#define       Judge_Radar_Info_20E_BufLen                  1  //雷达自主决策信息同步
#define       Judge_Robot_Communicate_301_BufLen           127//机器人交互数据，发送方触发发送
#define       Judge_User_Defined_302_BufLen                30 //自定义控制器与机器人交互数据，发送方触发发送
#define       Judge_Map_Interaction_303_BufLen             15 //选手端小地图交互数据
#define       Judge_KeyMouse_Message_304_BufLen            12 //键鼠遥控数据
#define       Judge_Map_Robot_305_BufLen                   24 //选手端小地图接收雷达数据
#define       Judge_Custom_Client_306_BufLen               8  //自定义控制器与选手端交互数据，发送方触发发送
#define       Judge_Map_Data_307_BufLen                    103//选手端小地图接收哨兵数据
#define       Judge_Custom_Info_308_BufLen                 34 //选手端小地图接收机器人数据
#define       Judge_Robot_Custom_309_BufLen                30 //自定义控制器接收机器人数据

/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define       JudgeLength_Game_StatusData_01              Judge_Game_StatusData_01_BufLen   + LEN_Frame_Header //比赛状态数据
#define       JudgeLength_Game_ResultData_02              Judge_Game_ResultData_02_BufLen   + LEN_Frame_Header //比赛结果数据
#define       JudgeLength_Robot_HP_03                     Judge_Robot_HP_03_BufLen          + LEN_Frame_Header //机器人血量数据
#define       JudgeLength_Event_Data_101                  Judge_Event_Data_101_BufLen       + LEN_Frame_Header //场地事件数据
#define       JudgeLength_Referee_Warning_104             Judge_Referee_Warning_104_BufLen  + LEN_Frame_Header //裁判警告数据，己方判罚/判负时触发发送
#define       JudgeLength_Dart_Countdown_105              Judge_Dart_Countdown_105_BufLen   + LEN_Frame_Header //飞镖发射相关数据
#define       JudgeLength_Robot_State_201                 Judge_Robot_State_201_BufLen      + LEN_Frame_Header //机器人性能体系数据
#define       JudgeLength_Power_Heat_202                  Judge_Power_Heat_202_BufLen       + LEN_Frame_Header //实时底盘缓冲能量和射击热量数据
#define       JudgeLength_Robot_Position_203              Judge_Robot_Position_203_BufLen   + LEN_Frame_Header //机器人位置数据
#define       JudgeLength_Robot_Buff_204                  Judge_Robot_Buff_204_BufLen       + LEN_Frame_Header //机器人增益和底盘能量数据
#define       JudgeLength_Injury_State_206                Judge_Injury_State_206_BufLen     + LEN_Frame_Header //伤害状态数据
#define       JudgeLength_RealTime_Shoot_207              Judge_RealTime_Shoot_207_BufLen   + LEN_Frame_Header //实时射击数据
#define       JudgeLength_Remaining_Rounds_208            Judge_Remaining_Rounds_208_BufLen + LEN_Frame_Header //允许发弹量
#define       JudgeLength_Robot_RFID_209                  Judge_Robot_RFID_209_BufLen      + LEN_Frame_Header //机器人RFID模块状态
#define       JudgeLength_Dart_Client_20A                 Judge_Dart_Client_20A_BufLen      + LEN_Frame_Header //飞镖选手端指令数据
#define       JudgeLength_Ground_Position_20B             Judge_Ground_Position_20B_BufLen  + LEN_Frame_Header //地面机器人位置数据
#define       JudgeLength_Radar_Mark_20C                  Judge_Radar_Mark_20C_BufLen       + LEN_Frame_Header //雷达标记进度数据
#define       JudgeLength_Sentry_Info_20D                 Judge_Sentry_Info_20D_BufLen      + LEN_Frame_Header //哨兵自主决策信息同步
#define       JudgeLength_Radar_Info_20E                  Judge_Radar_Info_20E_BufLen       + LEN_Frame_Header //雷达自主决策信息同步
#define       JudgeLength_Robot_Communicate_301           Judge_Robot_Communicate_301_BufLen+ LEN_Frame_Header //机器人交互数据，发送方触发发送
#define       JudgeLength_User_Defined_302                Judge_User_Defined_302_BufLen     + LEN_Frame_Header //自定义控制器与机器人交互数据，发送方触发发送
#define       JudgeLength_Map_Interaction_303             Judge_Map_Interaction_303_BufLen  + LEN_Frame_Header //选手端小地图交互数据
#define       JudgeLength_KeyMouse_Message_304            Judge_KeyMouse_Message_304_BufLen + LEN_Frame_Header //键鼠遥控数据
#define       JudgeLength_Map_Robot_305                   Judge_Map_Robot_305_BufLen        + LEN_Frame_Header //选手端小地图接收雷达数据
#define       JudgeLength_Custom_Client_306               Judge_Custom_Client_306_BufLen    + LEN_Frame_Header //自定义控制器与选手端交互数据，发送方触发发送
#define       JudgeLength_Map_Data_307                    Judge_Map_Data_307_BufLen         + LEN_Frame_Header //选手端小地图接收哨兵数据
#define       JudgeLength_Custom_Info_308                 Judge_Custom_Info_308_BufLen      + LEN_Frame_Header //选手端小地图接收机器人数据
#define       JudgeLength_Robot_Custom_309                Judge_Robot_Custom_309_BufLen     + LEN_Frame_Header //自定义控制器接收机器人数据

/**************************************自定义UI机器人指示信息文字*****************************************/

#define       LED_YELLOWMeaning                 "Fric:Manual"
#define       LED_GREENMeaning                  "Shot:Spin:Maga"
#define       LED_ORANGEMeaning                 "ChassisMode:Normal"
#define       LED_PURPLEMeaning                 "Square:USE_SUP:SUP"
#define       LED_PINKMeaning                   "   NO!!       "

#define       LED_CYANMeaning                   "Resever"
#define       LED_BLACKMeaning                  "SuperCap"
#define       LED_WHITEMeaning                  "Windmill"
#define       LED_Init                          "         "

extern int YellowFlag;
extern int GreenFlag;
extern int OrangeFlag;
extern int PurpleFlag;
extern int PinkFlag;
/**************************************裁判系统结构体*****************************************/
/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

/* ID: 0x0001    Byte: 11     比赛状态数据 */
typedef struct
{
  union {
		uint8_t dataBuff[Judge_Game_StatusData_01_BufLen];
	 __packed struct {
			uint8_t game_type : 4;             //比赛类型
			uint8_t game_progress : 4;         //当前比赛阶段
			uint16_t stage_remain_time;        //当前阶段剩余时间  单位s
			uint64_t SyncTimeStamp;            //UNIX世间
		};
	}data;
	uint8_t infoUpdateFlag;
}ext_game_status_t;

/* ID: 0x0002    Byte: 1       比赛结果数据 */
typedef struct
{
	union{
	  uint8_t dataBuff[Judge_Game_ResultData_02_BufLen];
	 __packed struct
		{
			uint8_t winner;//0平局 1红胜 2蓝胜
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_result_t;

/* ID: 0x0003     Byte: 32     比赛机器人血量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[Judge_Robot_HP_03_BufLen];
		__packed struct
		{
				uint16_t ally_1_robot_HP;
				uint16_t ally_2_robot_HP; 
				uint16_t ally_3_robot_HP; 
				uint16_t ally_4_robot_HP; 
				uint16_t reserved; 
				uint16_t ally_7_robot_HP; 
				uint16_t ally_outpost_HP;
				uint16_t ally_base_HP; 	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_HP_t;

/* ID: 0x0101     Byte: 4       场地事件数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Event_Data_101_BufLen];
	 __packed struct
		{
			uint32_t event_type;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_event_data_t;


/* ID: 0X0104          Byte: 3       裁判警告数据 */
typedef struct
{
	union
	{
    uint8_t dataBuff[Judge_Referee_Warning_104_BufLen];
	  __packed struct
	 {
		 uint8_t level;//警告等级：
		 uint8_t foul_robot_id; //犯规机器人 ID：判负时，机器人 ID 为 0 黄牌、红牌时，机器人 ID 为犯规机器人 ID
		 uint8_t count;//违规次数
	 };
	}data;
	uint8_t InfoUpdataFlag;
}ext_referee_warning_t;

/* ID: 0x0105          Byte: 3       飞镖发射口倒计时  */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Dart_Countdown_105_BufLen];
	__packed struct
		{
			uint8_t dart_remaining_time;//15s 倒计时
			uint16_t dart_info;
		} ;
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_remaining_time_t;

/* ID: 0X0201          Byte: 13      机器人状态数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Robot_State_201_BufLen];
	__packed struct
		{
		  uint8_t robot_id;
      uint8_t robot_level;
      uint16_t remain_HP;//机器人剩余血量
      uint16_t max_HP;//机器人上限血量
      uint16_t shooter_barrel_cooling_value; //机器人枪口热量每秒冷却值
      uint16_t shooter_barrel_heat_limit;//机器人枪口热量上限
      uint16_t chassis_power_limit;  //机器人底盘功率上限
			float bullet_speed_limit;
			/*主控电源输出情况：
       0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
       1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
       2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；*/
      uint8_t mains_power_gimbal_output : 1;
      uint8_t mains_power_chassis_output : 1;
      uint8_t mains_power_shooter_output : 1;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_status_t;

/* ID: 0X0202          Byte: 16      实时功率热量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[Judge_Power_Heat_202_BufLen];
	__packed  struct
		{
			uint16_t reserved; //底盘输出电压 单位 毫伏
      uint16_t reserved2; //底盘输出电流 单位 毫安
      float reserved3;//底盘输出功率 单位 W 瓦
      uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
      uint16_t shooter_id1_17mm_cooling_heat;//枪口热量
      uint16_t shooter_id1_42mm_cooling_heat;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_power_heat_data_t;

/* ID: 0X0203          Byte: 12      机器人位置数据  */
typedef struct
{
  union
	{
		uint8_t dataBuff[Judge_Robot_Position_203_BufLen];
	__packed struct
		{
		 float x;//位置 x 坐标
     float y;//位置 y 坐标
     float yaw;//位置枪口
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_pos_t;

/* ID: 0X0204          Byte: 7       机器人增益数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Robot_Buff_204_BufLen];
	 __packed struct
		{
			uint8_t recovery_buff;     //回血增益
      uint8_t cooling_buff;      //射击热量冷却增益
      uint8_t defence_buff;      //防御增益
      uint8_t vulnerability_buff;//负防御增益
			uint16_t attack_buff;			 //攻击增益
			uint8_t remaining_energy_50 : 1;	 //剩余能量值反馈，>=50为1，否则为0
			uint8_t remaining_energy_30 : 1;	 //剩余能量值反馈，>=30为1，否则为0
			uint8_t remaining_energy_15 : 1;	 //剩余能量值反馈，>=15为1，否则为0
			uint8_t remaining_energy_5 : 1;	 //剩余能量值反馈，>=5为1，否则为0
			uint8_t remaining_energy_1 : 1;	 //剩余能量值反馈，>=1为1，否则为0
			
			uint8_t remaining_energy_01 : 1;	 //剩余能量值反馈，>=15为1，否则为0
			uint8_t remaining_energy_02 : 1;	 //剩余能量值反馈，>=5为1，否则为0
			uint8_t remaining_energy_03 : 1;	 //剩余能量值反馈，>=1为1，否则为0
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_buff_t;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[Judge_Injury_State_206_BufLen];
	 __packed struct
		{
		  uint8_t armor_id : 4;/*bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
                             五个装甲片，其他血量变化类型，该变量数值为 0。*/
      uint8_t hurt_type : 4;/*bit 4-7：血量变化类型,0x0 装甲伤害扣血；
                              0x1 模块掉线扣血；
                              0x2 超射速扣血；
                              0x3 超枪口热量扣血；
                              0x4 超底盘功率扣血；
                              0x5 装甲撞击扣血*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_hurt_t;

/* ID: 0X0207          Byte: 7       实时射击数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[Judge_RealTime_Shoot_207_BufLen];
	 __packed struct
		{
		  uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
      uint8_t shooter_id;//发射机构 ID
      uint8_t bullet_freq;//子弹射频 单位 Hz
      float bullet_speed;//子弹射速 单位 m/s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_shoot_data_t;

/* ID: 0X0208          Byte: 6       子弹剩余发送数 */
typedef struct
{
  union
	{
		uint8_t dataBuff[Judge_Remaining_Rounds_208_BufLen];
	 __packed struct
		{
		  uint16_t bullet_remaining_num_17mm;//17mm 子弹剩余发射数目
      uint16_t bullet_remaining_num_42mm;//42mm 子弹剩余发射数目
			uint16_t coin_remaining_num;//剩余金币数量
			uint16_t projectile_allowance_fortress;
//			堡垒增益点提供的储备17mm弹丸允许发弹量； 
//			该值与机器人是否实际占领堡垒无关 
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_bullet_remaining_t;

/* ID: 0X0209          Byte: 4       机器人RFID状态 */
/*bit 0：基地增益点 RFID 状态；
bit 1：高地增益点 RFID 状态；
bit 2：能量机关激活点 RFID 状态；
bit 3：飞坡增益点 RFID 状态；
bit 4：前哨岗增益点 RFID 状态；
bit 5：资源岛增益点 RFID 状态；
bit 6：补血点增益点 RFID 状态；
bit 7：工程机器人补血卡 RFID 状态；
RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能
获取对应的增益效果。*/
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Robot_RFID_209_BufLen];
	 __packed struct
		{
			uint32_t rfid_status;
			uint8_t rfid_status_2; 
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_rfid_status_t;

/* ID: 0x020A          Byte: 6      飞镖机器人客户端指令书 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Dart_Client_20A_BufLen];
	 __packed struct
		{
		  uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
      uint8_t reserved;
			uint16_t target_change_time;			 //切换击打目标时的比赛剩余时间
			uint16_t latest_launch_cmd_time;	 //最后一次操作手确定发射指令时的比赛剩余时间
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_client_cmd_t;

/* ID: 0x020B          Byte: 40      地面机器人坐标 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Ground_Position_20B_BufLen];
	__packed  struct
		{
		  float hero_x;      //己方英雄机器人位置x轴坐标
			float hero_y;      //己方英雄机器人位置y轴坐标
			float engineer_x;  //己方工程机器人位置x轴坐标
			float engineer_y;  //己方工程机器人位置y轴坐标
			float standard_3_x;//己方3号步兵机器人位置x轴坐标
			float standard_3_y;//己方3号步兵机器人位置y轴坐标
			float standard_4_x;//己方4号步兵机器人位置x轴坐标
			float standard_4_y;//己方4号步兵机器人位置y轴坐标
			float reserved;
			float reserved2;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_ground_robot_position_t;

/* ID: 0x020C          Byte: 1      机器人易伤情况 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Radar_Mark_20C_BufLen];
	 __packed struct
		{
			uint16_t mark_progress; //机器人易伤情况
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_radar_mark_data_t;

/* ID: 0x020D          Byte: 6      哨兵自主决策信息同步 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Sentry_Info_20D_BufLen];
	__packed struct
		{
			uint32_t sentry_info;
			uint16_t sentry_info_2;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_sentry_info_t;

/* ID: 0x020E          Byte: 1      雷达自主决策信息同步 */
typedef struct
{
	union
	{
		uint8_t dataBuff[Judge_Radar_Info_20E_BufLen];
	 __packed struct
		{
			uint8_t radar_info;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_radar_info_t;

/* 

交互数据包括一个统一的数据段头结构。
数据段包含了内容 ID，发送者以及接收者的 ID 和内容数据段，
整个交互数据的包总共长最大为 128 个字节，
减去 frame_header,cmd_id 和 frame_tail 共 9 个字节以及数据段头结构的 6 个字节，
故而发送的内容数据段最大为 113。 
每个机器人交互数据与自定义控制器数据上下行合计带宽不超过 5000 Byte。上下行发送频率分别不超过
30Hz。

机器人 ID：
1，英雄(红)；
2，工程(红)；
3/4/5，步兵(红)；
6，空中(红)；
7，哨兵(红)；
9，雷达站（红）；

101，英雄(蓝)；
102，工程(蓝)；
103/104/105，步兵(蓝)；
106，空中(蓝)；
107，哨兵(蓝)；
109，雷达站（蓝）。

客户端 ID：
0x0101 为英雄操作手客户端(红)；
0x0102，工程操作手客户端((红)；
0x0103/0x0104/0x0105，步兵操作手客户端(红)；
0x0106，空中操作手客户端((红)；

0x0165，英雄操作手客户端(蓝)；
0x0166，工程操作手客户端(蓝)；
0x0167/0x0168/0x0169，步兵操作手客户端步兵(蓝)；
0x016A，空中操作手客户端(蓝)

*/

/* 
  由于存在多个内容 ID，但整个 cmd_id 上行频率最大为 10Hz，
  学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			           备注 
	   0 			   2 		数据的内容 ID 	 0x0200~0x02FF 
										                 可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	   2 			   2 		发送者的 ID 	   需要校验发送者的 ID 正确性， 
	
	   4 			   2 		接收者的 ID 	   需要校验接收者的 ID 正确性，
										                 例如不能发送到敌对机器人的ID 
	
	   6 			   n 		数据段 			     n 需要小于 113 


*/

/* ID: 0X0301          Byte: n       机器人间交互数据 */
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* data */
typedef __packed struct
{
	uint8_t data[20];//数据段n小于113
} robot_interactive_data_t;

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0x0100   0x0101  0x0102  0x0103  0x0110  0x0104
	发送频率：上限 10Hz


*/


/***************************************/

//小地图交互信息  客户端接收信息 0x0305


void Judge_GetMessage(uint8_t *buffer, uint16_t length);
void Check_Judge(void);

extern RM_Judge_t JudgeSystem;



extern ext_game_status_t      ext_game_status;
extern ext_game_result_t     ext_game_result;
extern ext_game_robot_HP_t   ext_game_robot_HP;
extern ext_event_data_t ext_even_data;
extern ext_referee_warning_t         ext_referee_warning;
extern ext_dart_remaining_time_t     ext_dart_remaining_time;
extern ext_game_robot_status_t  ext_game_robot_state;
extern ext_power_heat_data_t  ext_power_heat_data;
extern ext_game_robot_pos_t  ext_game_robot_pos;
extern ext_buff_t Buff;
extern ext_robot_hurt_t  ext_robot_hurt;
extern ext_shoot_data_t  ext_shoot_data;
extern ext_bullet_remaining_t  ext_bullet_remaining;
extern ext_rfid_status_t        ext_rfid_status;
extern ext_dart_client_cmd_t   ext_dart_client_cmd;


extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];


//#endif //版本号
/*-----------------------------------------------------------------------*/


/* UI绘制内容cmdID */
#define UI_DataID_Delete 0x100	 // 客户端删除图形
#define UI_DataID_Draw1 0x101	 // 客户端绘制1个图形
#define UI_DataID_Draw2 0x102	 // 客户端绘制2个图形
#define UI_DataID_Draw5 0x103	 // 客户端绘制5个图形
#define UI_DataID_Draw7 0x104	 // 客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 // 客户端绘制字符图形

/* UI删除操作 */
#define UI_Delete_Invalid 0 // 空操作
#define UI_Delete_Layer 1	// 删除图层
#define UI_Delete_All 2		// 删除所有

/* UI图形操作 */
#define UI_Graph_invalid 0 // 空操作
#define UI_Graph_Add 1	   // 增加图形
#define UI_Graph_Change 2  // 修改图形
#define UI_Graph_Delete 3  // 删除图形

/* UI图形类型 */
#define UI_Graph_Line 0		 // 直线
#define UI_Graph_Rectangle 1 // 矩形
#define UI_Graph_Circle 2	 // 整圆
#define UI_Graph_Ellipse 3	 // 椭圆
#define UI_Graph_Arc 4		 // 圆弧
#define UI_Graph_Float 5	 // 浮点型
#define UI_Graph_Int 6		 // 整形
#define UI_Graph_String 7	 // 字符型

/* UI图形颜色 */
#define UI_Color_Main 0	  // 红蓝主色
#define UI_Color_Yellow 1 // 黄色
#define UI_Color_Green 2  // 绿色
#define UI_Color_Orange 3 // 橙色
#define UI_Color_Purple 4 // 紫色
#define UI_Color_Pink 5	  // 粉色
#define UI_Color_Cyan 6	  // 青色
#define UI_Color_Black 7  // 黑色
#define UI_Color_White 8  // 白色

#define HEADER_SOF 0xA5
#define CMD_ID_ROBOT 0x0301 // 机器人间通信
/* 图形数据 */
typedef __packed struct
{ 
  uint8_t graphic_name[3]; 
  uint32_t operate_tpye:3; 
  uint32_t graphic_tpye:3; 
  uint32_t layer:4; 
  uint32_t color:4; 
  uint32_t start_angle:9;
  uint32_t end_angle:9;
  uint32_t width:10; 
  uint32_t start_x:11; 
  uint32_t start_y:11; 
  uint32_t radius:10; 
  uint32_t end_x:11; 
  uint32_t end_y:11; 
} graphic_data_struct_t;

typedef __packed struct
{ 
  uint8_t graphic_name[3]; 
  uint32_t operate_tpye:3; 
  uint32_t graphic_tpye:3; 
  uint32_t layer:4; 
  uint32_t color:4; 
  uint32_t start_angle:9;
  uint32_t end_angle:9;
  uint32_t width:10; 
  uint32_t start_x:11; 
  uint32_t start_y:11; 
	int32_t data;
} ClientData_struct_t;

typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

/* 客户端绘制字符 机器人间通信：0x0301 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct;
  char data[30];
} ext_client_custom_character_t;


/* 客户端删除图形 机器人间通信：0x0301 */
typedef __packed struct
{
  uint8_t operate_tpye; 
  uint8_t layer; 
} ext_client_custom_graphic_delete_t;


/*机器人交互信息：0x0301*/
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
} ext_CommunatianData_t;

//帧头  命令码   数据段头结构  数据段   帧尾

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[1];//数据段
	uint16_t		 						FrameTail;//帧尾
} UI_Graph1_t;

typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[2];//数据段
	uint16_t		 						FrameTail;//帧尾
} UI_Graph2_t;


typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[5];//数据段
	uint16_t		 						FrameTail;//帧尾
} UI_Graph5_t;


typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t cilentData[7];//数据段
	uint16_t		 						FrameTail;//帧尾
} UI_Graph7_t;


typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_character_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
} UI_String_t;


//删除客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_graphic_delete_t cilentData;//数据段
	uint16_t		 						FrameTail;//帧尾
} ext_DeleteClientData_t;

void UI_Draw_Line(graphic_data_struct_t *Graph,
				  char GraphName[3],
				  uint8_t GraphOperate,
				  uint8_t Layer,
				  uint8_t Color,
				  uint16_t Width,
				  uint16_t StartX,
				  uint16_t StartY,
				  uint16_t EndX,
				  uint16_t EndY);
void UI_Draw_Rectangle(graphic_data_struct_t *Graph, // UI图形数据结构体指针
					   char GraphName[3],			 // 图形名 作为客户端的索引
					   uint8_t GraphOperate,		 // UI图形操作 对应UI_Graph_XXX的4种操作
					   uint8_t Layer,				 // UI图形图层 [0,9]
					   uint8_t Color,				 // UI图形颜色 对应UI_Color_XXX的9种颜色
					   uint16_t Width,				 // 线宽
					   uint16_t StartX,				 // 起始坐标X
					   uint16_t StartY,				 // 起始坐标Y
					   uint16_t EndX,				 // 截止坐标X
					   uint16_t EndY);				 // 截止坐标Y
void UI_Draw_Circle(graphic_data_struct_t *Graph,	 // UI图形数据结构体指针
					char GraphName[3],				 // 图形名 作为客户端的索引
					uint8_t GraphOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
					uint8_t Layer,					 // UI图形图层 [0,9]
					uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
					uint16_t Width,					 // 线宽
					uint16_t CenterX,				 // 圆心坐标X
					uint16_t CenterY,				 // 圆心坐标Y
					uint16_t Radius);				 // 半径
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,	 // UI图形数据结构体指针
					 char GraphName[3],				 // 图形名 作为客户端的索引
					 uint8_t GraphOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
					 uint8_t Layer,					 // UI图形图层 [0,9]
					 uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
					 uint16_t Width,				 // 线宽
					 uint16_t CenterX,				 // 圆心坐标X
					 uint16_t CenterY,				 // 圆心坐标Y
					 uint16_t XHalfAxis,			 // X半轴长
					 uint16_t YHalfAxis);			 // Y半轴长
void UI_Draw_Arc(graphic_data_struct_t *Graph,		 // UI图形数据结构体指针
				 char GraphName[3],					 // 图形名 作为客户端的索引
				 uint8_t GraphOperate,				 // UI图形操作 对应UI_Graph_XXX的4种操作
				 uint8_t Layer,						 // UI图形图层 [0,9]
				 uint8_t Color,						 // UI图形颜色 对应UI_Color_XXX的9种颜色
				 uint16_t StartAngle,				 // 起始角度 [0,360]
				 uint16_t EndAngle,					 // 截止角度 [0,360]
				 uint16_t Width,					 // 线宽
				 uint16_t CenterX,					 // 圆心坐标X
				 uint16_t CenterY,					 // 圆心坐标Y
				 uint16_t XHalfAxis,				 // X半轴长
				 uint16_t YHalfAxis);				 // Y半轴长
void UI_Draw_Float(graphic_data_struct_t *Graph,	 // UI图形数据结构体指针
				   char GraphName[3],				 // 图形名 作为客户端的索引
				   uint8_t GraphOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
				   uint8_t Layer,					 // UI图形图层 [0,9]
				   uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
				   uint16_t NumberSize,				 // 字体大小
				   uint16_t Significant,			 // 有效位数
				   uint16_t Width,					 // 线宽
				   uint16_t StartX,					 // 起始坐标X
				   uint16_t StartY,					 // 起始坐标Y
				   float FloatData);				 // 数字内容
void UI_Draw_Int(graphic_data_struct_t *Graph,		 // UI图形数据结构体指针
				 char GraphName[3],					 // 图形名 作为客户端的索引
				 uint8_t GraphOperate,				 // UI图形操作 对应UI_Graph_XXX的4种操作
				 uint8_t Layer,						 // UI图形图层 [0,9]
				 uint8_t Color,						 // UI图形颜色 对应UI_Color_XXX的9种颜色
				 uint16_t NumberSize,				 // 字体大小
				 uint16_t Width,					 // 线宽
				 uint16_t StartX,					 // 起始坐标X
				 uint16_t StartY,					 // 起始坐标Y
				 int32_t IntData);					 // 数字内容
//void UI_Draw_String(ClientData_struct_t *String,	 // UI图形数据结构体指针
//					char StringName[3],				 // 图形名 作为客户端的索引
//					uint8_t StringOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
//					uint8_t Layer,					 // UI图形图层 [0,9]
//					uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
//					uint16_t CharSize,				 // 字体大小
//					uint16_t StringLength,			 // 字符串长度
//					uint16_t Width,					 // 线宽
//					uint16_t StartX,				 // 起始坐标X
//					uint16_t StartY,				 // 起始坐标Y
//					char *StringData);				 // 字符串内容

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs);
//void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
//void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);



void UI_Draw_CrossHair(uint8_t RobotID);
_Bool is_red_or_blue(void);
void determine_ID(void);


void Standard_CrossHair(void);
void UI_Draw_CAP_Energy(void);
uint8_t UI_Cap_Text_Dynamic(void);
void Cap_Energy_String_T(uint8_t value);


void UI_Text(void);
void UI_All_Graph3(void);
void UI_All_Graph2(void);
void UI_All_Graph(void);
void UI_Static(void);
void UI_Text_Dynamic(void);
void UI_All_Graph2_Dynamic(void);
void UI_All_Graph3_Dynamic(void);
void UI_Dynamic(void);
void UI_Dynamic2(void);
#endif //头文件
