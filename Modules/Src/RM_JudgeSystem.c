#include "RM_JudgeSystem.h"
#include "drv_uart.h"
#include "bsp_SuperCap.h"



// 全局裁判系统管理对象指针
extern Struct_UART_Manage_Object UART1_Manage_Object;

static uint32_t judge_last_update_tick = 0; // 上一次成功接收并解析的时间戳


RM_Judge_t RM_Judge;

/* 裁判系统初始化 */
void JudgeSystem_Init(UART_HandleTypeDef *huart)
{
    // 使用 drv_uart 的双缓冲框架
    UART_Init(huart, JudgeSystem_Callback);
    
    //UART_Init 已经启动了DMA接收，不需要再启动
}

/* 裁判系统回调函数（由drv_uart在空闲中断中调用） */
void JudgeSystem_Callback(uint8_t *buffer, uint16_t size)
{
    // buffer: drv_uart准备好的数据缓冲区
    // size: 接收到的数据长度
    
    // 调用消息处理函数
    Judge_GetMessage(buffer,size);
    
    // 更新帧计数
    RM_Judge.InfoUpdateFrame++;
}



/**
  * @brief  判断裁判系统是否在线
  */
bool Judge_IsOnline(void)
{
    if(judge_last_update_tick == 0) return false;
    return ((osKernelGetTickCount() - judge_last_update_tick) <= JUDGE_TIMEOUT_MS);
}



/*************************************2025赛季裁判系统*********************************************/




/**************************************大疆CRC校验代码************************************************/
/*         Start            */
uint16_t wExpected;


ext_game_status_t ext_game_status;
ext_game_result_t ext_game_result;
ext_game_robot_HP_t ext_game_robot_HP;
ext_event_data_t ext_even_data;
ext_referee_warning_t ext_referee_warning;
ext_dart_remaining_time_t ext_dart_remaining_time;
ext_game_robot_status_t ext_game_robot_state;
ext_power_heat_data_t ext_power_heat_data;
ext_game_robot_pos_t ext_game_robot_pos;
ext_buff_t Buff;
ext_robot_hurt_t ext_robot_hurt;
ext_shoot_data_t ext_shoot_data;
ext_bullet_remaining_t ext_bullet_remaining;
ext_rfid_status_t ext_rfid_status;
ext_dart_client_cmd_t ext_dart_client_cmd;






//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
    {
        0x00,
        0x5e,
        0xbc,
        0xe2,
        0x61,
        0x3f,
        0xdd,
        0x83,
        0xc2,
        0x9c,
        0x7e,
        0x20,
        0xa3,
        0xfd,
        0x1f,
        0x41,
        0x9d,
        0xc3,
        0x21,
        0x7f,
        0xfc,
        0xa2,
        0x40,
        0x1e,
        0x5f,
        0x01,
        0xe3,
        0xbd,
        0x3e,
        0x60,
        0x82,
        0xdc,
        0x23,
        0x7d,
        0x9f,
        0xc1,
        0x42,
        0x1c,
        0xfe,
        0xa0,
        0xe1,
        0xbf,
        0x5d,
        0x03,
        0x80,
        0xde,
        0x3c,
        0x62,
        0xbe,
        0xe0,
        0x02,
        0x5c,
        0xdf,
        0x81,
        0x63,
        0x3d,
        0x7c,
        0x22,
        0xc0,
        0x9e,
        0x1d,
        0x43,
        0xa1,
        0xff,
        0x46,
        0x18,
        0xfa,
        0xa4,
        0x27,
        0x79,
        0x9b,
        0xc5,
        0x84,
        0xda,
        0x38,
        0x66,
        0xe5,
        0xbb,
        0x59,
        0x07,
        0xdb,
        0x85,
        0x67,
        0x39,
        0xba,
        0xe4,
        0x06,
        0x58,
        0x19,
        0x47,
        0xa5,
        0xfb,
        0x78,
        0x26,
        0xc4,
        0x9a,
        0x65,
        0x3b,
        0xd9,
        0x87,
        0x04,
        0x5a,
        0xb8,
        0xe6,
        0xa7,
        0xf9,
        0x1b,
        0x45,
        0xc6,
        0x98,
        0x7a,
        0x24,
        0xf8,
        0xa6,
        0x44,
        0x1a,
        0x99,
        0xc7,
        0x25,
        0x7b,
        0x3a,
        0x64,
        0x86,
        0xd8,
        0x5b,
        0x05,
        0xe7,
        0xb9,
        0x8c,
        0xd2,
        0x30,
        0x6e,
        0xed,
        0xb3,
        0x51,
        0x0f,
        0x4e,
        0x10,
        0xf2,
        0xac,
        0x2f,
        0x71,
        0x93,
        0xcd,
        0x11,
        0x4f,
        0xad,
        0xf3,
        0x70,
        0x2e,
        0xcc,
        0x92,
        0xd3,
        0x8d,
        0x6f,
        0x31,
        0xb2,
        0xec,
        0x0e,
        0x50,
        0xaf,
        0xf1,
        0x13,
        0x4d,
        0xce,
        0x90,
        0x72,
        0x2c,
        0x6d,
        0x33,
        0xd1,
        0x8f,
        0x0c,
        0x52,
        0xb0,
        0xee,
        0x32,
        0x6c,
        0x8e,
        0xd0,
        0x53,
        0x0d,
        0xef,
        0xb1,
        0xf0,
        0xae,
        0x4c,
        0x12,
        0x91,
        0xcf,
        0x2d,
        0x73,
        0xca,
        0x94,
        0x76,
        0x28,
        0xab,
        0xf5,
        0x17,
        0x49,
        0x08,
        0x56,
        0xb4,
        0xea,
        0x69,
        0x37,
        0xd5,
        0x8b,
        0x57,
        0x09,
        0xeb,
        0xb5,
        0x36,
        0x68,
        0x8a,
        0xd4,
        0x95,
        0xcb,
        0x29,
        0x77,
        0xf4,
        0xaa,
        0x48,
        0x16,
        0xe9,
        0xb7,
        0x55,
        0x0b,
        0x88,
        0xd6,
        0x34,
        0x6a,
        0x2b,
        0x75,
        0x97,
        0xc9,
        0x4a,
        0x14,
        0xf6,
        0xa8,
        0x74,
        0x2a,
        0xc8,
        0x96,
        0x15,
        0x4b,
        0xa9,
        0xf7,
        0xb6,
        0xe8,
        0x0a,
        0x54,
        0xd7,
        0x89,
        0x6b,
        0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
/*整数型转化为字符型*/
void int_into_char(char *Data, int16_t i)
{
    char data[10];
    sprintf(data, "%d", i);
    strcpy(Data, data);
}

/*           END            */
/**************************************大疆CRC校验代码************************************************/




//uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];
uint16_t rxBuff_index = 0;



void Judge_GetMessage(uint8_t *buffer, uint16_t length)
{
    for (int n = 0; n < length;)
    {
        if (buffer[n] == JUDGE_FRAME_HEADER)
        {
            switch (buffer[n + 5] | buffer[n + 6] << 8)
            {
            case Judge_Game_StatusData_01_ID: //比赛状态数据
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Game_StatusData_01))
                {
                    memcpy(ext_game_status.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Game_StatusData_01_BufLen]));
                    n += JudgeLength_Game_StatusData_01;
                    ext_game_status.infoUpdateFlag = 1;
									
									  judge_last_update_tick = osKernelGetTickCount();//更新时间戳
                }
                else
                    n++;
                break;
            case Judge_Robot_HP_03_ID: //机器人血量数据
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Robot_HP_03))
                {
                    memcpy(&ext_game_robot_HP.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Robot_HP_03_BufLen]));
                    n += JudgeLength_Robot_HP_03;
                    ext_game_robot_HP.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Event_Data_101_ID: //场地事件数据
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Event_Data_101))
                {
                    memcpy(&ext_even_data.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Event_Data_101_BufLen]));
                    n += JudgeLength_Event_Data_101;
                    ext_even_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Referee_Warning_104_ID: //裁判系统警告信息
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Referee_Warning_104))
                {
                    memcpy(&ext_referee_warning.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Referee_Warning_104_BufLen]));
                    n += JudgeLength_Referee_Warning_104;
                    ext_referee_warning.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_State_201_ID: //比赛机器人状态
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Robot_State_201))
                {
                    memcpy(&ext_game_robot_state.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Robot_State_201_BufLen]));
                    n += JudgeLength_Robot_State_201;
                    ext_game_robot_state.InfoUpdataFlag = 1;
									
									  judge_last_update_tick = osKernelGetTickCount();//更新时间戳
                }
                else
                    n++;
                break;
            case Judge_Power_Heat_202_ID: //实时功率热量
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Power_Heat_202))
                {
                    memcpy(&ext_power_heat_data.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Power_Heat_202_BufLen]));
                    n += JudgeLength_Power_Heat_202;
                    ext_power_heat_data.InfoUpdataFlag = 1;
									
								  	judge_last_update_tick = osKernelGetTickCount();//更新时间戳
                }
                else
                    n++;
                break;
            case Judge_Robot_Position_203_ID: //机器人位置
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Robot_Position_203))
                {
                    memcpy(&ext_game_robot_pos.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Robot_Position_203_BufLen]));
                    n += JudgeLength_Robot_Position_203;
                    ext_game_robot_pos.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Buff_204_ID: //机器人增益
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Robot_Buff_204))
                {
                    memcpy(&Buff.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Robot_Buff_204_BufLen]));
                    n += JudgeLength_Robot_Buff_204;
                    Buff.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Injury_State_206_ID: //伤害状态
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Injury_State_206))
                {
                    memcpy(&ext_robot_hurt.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Injury_State_206_BufLen]));
                    n += JudgeLength_Injury_State_206;
                    ext_robot_hurt.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_RealTime_Shoot_207_ID: //实时射击数据
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_RealTime_Shoot_207))
                {
                    memcpy(&ext_shoot_data.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_RealTime_Shoot_207_BufLen]));
                    n += JudgeLength_RealTime_Shoot_207;
                    ext_shoot_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Remaining_Rounds_208_ID: //子弹剩余数
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Remaining_Rounds_208))
                {
                    memcpy(&ext_bullet_remaining.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Remaining_Rounds_208_BufLen]));
                    n += JudgeLength_Remaining_Rounds_208;
                    ext_bullet_remaining.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_RFID_209_ID: //机器人RFID状态
                if (Verify_CRC16_Check_Sum(buffer + n, JudgeLength_Robot_RFID_209))
                {
                    memcpy(&ext_rfid_status.data.dataBuff, &buffer[n + 7], sizeof(uint8_t[Judge_Robot_RFID_209_BufLen]));
                    n += JudgeLength_Robot_RFID_209;
                    ext_rfid_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            default:
                n++;
                break;
            }
        }
        else
            n++;
    }
    RM_Judge.InfoUpdateFrame++;
}
/*==============================================================================
			  ##### UI    ͼ λ  ƺ    #####
  ==============================================================================
*/
/*==============================================================================
			  ##### UI    ͼ λ  ƺ    #####
  ==============================================================================
*/

//画直线
void UI_Draw_Line(graphic_data_struct_t *Graph,
				  char GraphName[3],
				  uint8_t GraphOperate,
				  uint8_t Layer,
				  uint8_t Color,
				  uint16_t Width,
				  uint16_t StartX,
				  uint16_t StartY,
				  uint16_t EndX,
				  uint16_t EndY)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Line;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}


//画矩阵(空心)
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,
					   char GraphName[3],
					   uint8_t GraphOperate,
					   uint8_t Layer,
					   uint8_t Color,
					   uint16_t Width,
					   uint16_t StartX,
					   uint16_t StartY,
					   uint16_t EndX,
					   uint16_t EndY)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Rectangle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

//画圆
void UI_Draw_Circle(graphic_data_struct_t *Graph,
					char GraphName[3],
					uint8_t GraphOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t Width,
					uint16_t CenterX,
					uint16_t CenterY,
					uint16_t Radius)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Circle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->radius = Radius;
}

//画椭圆
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,
					 char GraphName[3],
					 uint8_t GraphOperate,
					 uint8_t Layer,
					 uint8_t Color,
					 uint16_t Width,
					 uint16_t CenterX,
					 uint16_t CenterY,
					 uint16_t XHalfAxis,
					 uint16_t YHalfAxis)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Ellipse;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

//画圆弧
void UI_Draw_Arc(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t StartAngle,
				 uint16_t EndAngle,
				 uint16_t Width,
				 uint16_t CenterX,
				 uint16_t CenterY,
				 uint16_t XHalfAxis,
				 uint16_t YHalfAxis)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Arc;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = StartAngle;
	Graph->end_angle = EndAngle;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

/**
 * @brief    Ƹ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] NumberSize      С
 * @param[in] Significant   Чλ
 * @param[in] Width  ߿
 * @param[in] StartX   ʼ    X
 * @param[in] StartY   ʼ    Y
 * @param[in] FloatData
 */
//显示浮点数
void UI_Draw_Float(graphic_data_struct_t *Graph,
				   char GraphName[3],
				   uint8_t GraphOperate,
				   uint8_t Layer,
				   uint8_t Color,
				   uint16_t NumberSize,
				   uint16_t Significant,
				   uint16_t Width,
				   uint16_t StartX,
				   uint16_t StartY,
				   float FloatData)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Float;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->end_angle = Significant;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = ((IntData & 0x001ffc00) >> 10);
	Graph->end_y = ((IntData & 0xffe00000) >> 21);
}

//显示整数
void UI_Draw_Int(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t NumberSize,
				 uint16_t Width,
				 uint16_t StartX,
				 uint16_t StartY,
				 int32_t IntData)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Int;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = (IntData & 0x001ffc00) >> 10;
	Graph->end_y = (IntData & 0xffe00000) >> 21;
}

////显示字符串
//void UI_Draw_String(ClientData_struct_t *String,
//					char StringName[3],
//					uint8_t StringOperate,
//					uint8_t Layer,
//					uint8_t Color,
//					uint16_t CharSize,
//					uint16_t StringLength,
//					uint16_t Width,
//					uint16_t StartX,
//					uint16_t StartY,
//					char *StringData)
//{
//	String->graphic_name[0] = StringName[0];
//	String->graphic_name[1] = StringName[1];
//	String->graphic_name[2] = StringName[2];
//	String->operate_tpye = StringOperate;
//	String->graphic_tpye = UI_Graph_String;
//	String->layer = Layer;
//	String->color = Color;
//	String->start_angle = CharSize;
//	String->end_angle = StringLength;
//	String->width = Width;
//	String->start_x = StartX;
//	String->start_y = StartY;
//	memset(String->stringdata, 0, 30);
//	for (int i = 0; i < StringLength; i++)
//		String->stringdata[i] = *StringData++;
//}

///*==============================================================================
//			  ##### UI    ͼ     ͺ    #####
//  ==============================================================================
//*/


uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID

uint8_t Judge_Self_ID;        //当前机器人ID


/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
_Bool Color;
_Bool is_red_or_blue(void)
{
    Judge_Self_ID = ext_game_robot_state.data.robot_id; //读取当前机器人ID

    if(ext_game_robot_state.data.robot_id > 10)
    {
        return 0; //蓝方
    }
    else
    {
        return 1; //红方
    }
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
    Color = is_red_or_blue();
    if(Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0100 + ext_game_robot_state.data.robot_id; //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + ext_game_robot_state.data.robot_id; //计算客户端ID
    }
}


/**
 * @brief     ͼ
 * @param[in] Counter 1,2,5,7
 * @param[in] Graphs   Counter  һ µ UI_Graphx ṹ  ͷָ
 * @param[in] RobotID
 */
#define send_max_len 125
unsigned char CliendTxBuffer[send_max_len];
void UI_PushUp_Graphs(uint8_t Counter, void *Graphs)
{
	    static uint8_t datalength;
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //    ֻ  һ      ͼ

	/*     frame_header */
	Graph->txFrameHeader.SOF = HEADER_SOF;
	Graph->txFrameHeader.DataLength= sizeof(ext_student_interactive_header_data_t) + Counter * sizeof(ext_client_custom_graphic_single_t);
	Graph->txFrameHeader.Seq =0;
	memcpy(CliendTxBuffer, &Graph->txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码
	/*     cmd_id */
	Graph->CmdID = 0x0301;

	determine_ID();
	/*     student_interactive_header */
	if (Counter == 1)
		Graph->dataFrameHeader.data_cmd_id = UI_DataID_Draw1;
	else if (Counter == 2)
		Graph->dataFrameHeader.data_cmd_id = UI_DataID_Draw2;
	else if (Counter == 5)
		Graph->dataFrameHeader.data_cmd_id = UI_DataID_Draw5;
	else if (Counter == 7)
		Graph->dataFrameHeader.data_cmd_id = UI_DataID_Draw7;
	Graph->dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id;		   //  ǰ      ID
	Graph->dataFrameHeader.receiver_ID = Judge_SelfClient_ID; //  Ӧ      ID

	   memcpy(CliendTxBuffer + 5,
           (uint8_t *)&Graph->CmdID,
           (sizeof(Graph->CmdID) + sizeof(Graph->dataFrameHeader) + sizeof(graphic_data_struct_t)*Counter));
	/*     frame_tail   CRC16 */
	if (Counter == 1)
	{
		Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(UI_Graph1_t));
	}
	else if (Counter == 2)
	{
		Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(UI_Graph2_t));
	}
	else if (Counter == 5)
	{
		Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(UI_Graph5_t));
	}
	else if (Counter == 7)
	{
		Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(UI_Graph7_t));
	}

	// /* ʹ ô   PushUp      ϵͳ */
//	if (Counter == 1)
//		HAL_UART_Transmit_DMA(&huart1,CliendTxBuffer, sizeof(UI_Graph1_t));
//	else if (Counter == 2)
//		HAL_UART_Transmit_DMA(&huart1,CliendTxBuffer, sizeof(UI_Graph2_t));
//	else if (Counter == 5)
//		HAL_UART_Transmit_DMA(&huart1,CliendTxBuffer, sizeof(UI_Graph5_t));
//	else if (Counter == 7)
//		HAL_UART_Transmit_DMA(&huart1,CliendTxBuffer, sizeof(UI_Graph7_t));
	if (Counter == 1)
		HAL_UART_Transmit(&huart1,CliendTxBuffer, sizeof(UI_Graph1_t),20);
	else if (Counter == 2)
		HAL_UART_Transmit(&huart1,CliendTxBuffer, sizeof(UI_Graph2_t),20);
	else if (Counter == 5)
		HAL_UART_Transmit(&huart1,CliendTxBuffer, sizeof(UI_Graph5_t),20);
	else if (Counter == 7)
		HAL_UART_Transmit(&huart1,CliendTxBuffer, sizeof(UI_Graph7_t),20);
}



unsigned char String_Buff[200];
/**
 * @brief      ַ
 * @param[in] String
 * @param[in] RobotID
 */
void UI_PushUp_String(UI_String_t *String)
{
	static uint8_t String_datalength;

	String->txFrameHeader.SOF= HEADER_SOF;
	String->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	String->txFrameHeader.Seq = 0;
	
	 memcpy(String_Buff, &String->txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据

  Append_CRC8_Check_Sum(String_Buff, sizeof(xFrameHeader));

	/*     cmd_id */
	String->CmdID = 0x0301;
    determine_ID();

	/*     student_interactive_header */
	String->dataFrameHeader.data_cmd_id = UI_DataID_DrawChar;
	String->dataFrameHeader.sender_ID= ext_game_robot_state.data.robot_id;;			//  ǰ      ID
	String->dataFrameHeader.receiver_ID= Judge_SelfClient_ID; //  Ӧ      ID
 memcpy(String_Buff + 5,
           (uint8_t *)&String->CmdID,
           (sizeof(String->CmdID) + sizeof(String->dataFrameHeader) + sizeof(String->cilentData)));
    Append_CRC16_Check_Sum(String_Buff, sizeof(UI_String_t));
    String_datalength = sizeof(UI_String_t);

//    HAL_UART_Transmit_DMA(&huart1, String_Buff, String_datalength);
    HAL_UART_Transmit(&huart1, String_Buff, String_datalength,20);

}





/*******************************************************************************
/    内容ID          长度（头结构+内容数据段）            功能说明
/ 0x0200~0x02FF             6+n                         己方机器人间通信           
/    0x0100                 6+2                         客户端删除图形
/    0x0101                 6+15                        客户端绘制一个图形
/    0x0102                 6+30                        客户端绘制二个图形
/    0x0103                 6+75                        客户端绘制五个图形
/    0x0104                 6+105                       客户端绘制七个图形
/    0x0110                 6+45                        客户端绘制字符图形
********************************************************************************/
/*
	uint8_t graphic_name[3]; //图形名,在删除，修改等操作中，作为客户端的索引。
	uint32_t operate_tpye:3; //图形操作：0：空操作；1：增加；2：修改；3：删除；
uint32_t graphic_tpye:3; //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；5:浮点数 6：整型数 7：字符
	uint32_t layer:4; //图层数，0~9
	uint32_t color:4; //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	uint32_t start_angle:9; //：起始角度，单位：°，范围[0,360]；
	uint32_t end_angle:9; //终止角度，单位：°，范围[0,360]。
	uint32_t width:10; //线宽；0-9
	uint32_t start_x:11; //起点x坐标；
	uint32_t start_y:11; //起点y坐标。
	uint32_t radius:10; //字体大小或者半径；
	uint32_t end_x:11; //终点x坐标；
	uint32_t end_y:11; //终点y坐标。
*/



UI_Graph7_t ShootData;        // 静态线
UI_Graph2_t CapData;          // 超电(1线+1框)
UI_String_t UI_TextString;    // 子弹文字
UI_String_t UI_FricString;    // 摩擦轮文字
UI_String_t UI_CapTextString; // 超电文本
UI_String_t UI_Fric_Speed_String;// 摩擦轮转速
UI_Graph1_t LockData;         // 雷达吊射锁定
UI_String_t UI_GimbalAngleString;//云台角度文字
UI_String_t UI_ShootSpeedString; //弹速
void UI_Static(void)
{
    //静态线
		UI_Draw_Line(&ShootData.cilentData[0],(char[3]){1,0,0},UI_Graph_Add,0,UI_Color_Green,1,920,460,1000,460);
    UI_Draw_Line(&ShootData.cilentData[1],(char[3]){3,0,0},UI_Graph_Add,0,UI_Color_Green,1,880,440,1040,440);
    UI_Draw_Circle(&ShootData.cilentData[2],(char[3]){5,0,0},UI_Graph_Add,0,UI_Color_Green,1,960,540,30);

		//车距线
		UI_Draw_Line(&ShootData.cilentData[3],(char[3]){6,0,0},UI_Graph_Add,0,UI_Color_Green,1,1422,0,1168,400);
    UI_Draw_Line(&ShootData.cilentData[4],(char[3]){7,0,0},UI_Graph_Add,0,UI_Color_Green,1,493,0,772,400);
		
    UI_Draw_Line(&ShootData.cilentData[5], (char[3]){2,0,0}, UI_Graph_Add, 0, UI_Color_Green, 1, 772, 388, 1168, 388);
    UI_Draw_Line(&ShootData.cilentData[6], (char[3]){4,0,0}, UI_Graph_Add, 0, UI_Color_Green, 1, 960, 800, 960, 150);
		
    UI_PushUp_Graphs(7, &ShootData);
    osDelay(10);
	
	
    //雷达吊射锁定
    UI_Draw_Circle(&LockData.cilentData[0], (char[3]){14,0,0}, UI_Graph_Add, 1, UI_Color_Black, 1, 0, 0, 0);
    UI_PushUp_Graphs(1, &LockData);
    osDelay(10);
	

		
    //云台角度文本初始化
    UI_GimbalAngleString.cilentData.grapic_data_struct.graphic_name[0] = 16;
    UI_GimbalAngleString.cilentData.grapic_data_struct.graphic_name[1] = 0;
    UI_GimbalAngleString.cilentData.grapic_data_struct.graphic_name[2] = 0;
    UI_GimbalAngleString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Add;
    UI_GimbalAngleString.cilentData.grapic_data_struct.graphic_tpye = 7;
    UI_GimbalAngleString.cilentData.grapic_data_struct.layer = 0;
    UI_GimbalAngleString.cilentData.grapic_data_struct.color = UI_Color_Pink;
    UI_GimbalAngleString.cilentData.grapic_data_struct.start_angle = 25;        //字体较大
    UI_GimbalAngleString.cilentData.grapic_data_struct.width = 3;               //线宽
    UI_GimbalAngleString.cilentData.grapic_data_struct.start_x = 100;           //左侧
    UI_GimbalAngleString.cilentData.grapic_data_struct.start_y = 750;           //左上

		snprintf(UI_GimbalAngleString.cilentData.data, 30, "Y:0.00 P:0.00    ");
    UI_GimbalAngleString.cilentData.grapic_data_struct.end_angle = strlen(UI_GimbalAngleString.cilentData.data);
    UI_PushUp_String(&UI_GimbalAngleString);
    osDelay(10);
	
	
	
    //弹速文本初始化
    UI_ShootSpeedString.cilentData.grapic_data_struct.graphic_name[0] = 17;
    UI_ShootSpeedString.cilentData.grapic_data_struct.graphic_name[1] = 0;
    UI_ShootSpeedString.cilentData.grapic_data_struct.graphic_name[2] = 0;
    UI_ShootSpeedString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Add;
    UI_ShootSpeedString.cilentData.grapic_data_struct.graphic_tpye = 7;
    UI_ShootSpeedString.cilentData.grapic_data_struct.layer = 0;
    UI_ShootSpeedString.cilentData.grapic_data_struct.color = UI_Color_Pink; 
    UI_ShootSpeedString.cilentData.grapic_data_struct.start_angle = 25;  // 字体大小
    UI_ShootSpeedString.cilentData.grapic_data_struct.width = 3;
    UI_ShootSpeedString.cilentData.grapic_data_struct.start_x = 450;
    UI_ShootSpeedString.cilentData.grapic_data_struct.start_y = 630; 
    
    snprintf(UI_ShootSpeedString.cilentData.data, 30, " 0.00    ");
    UI_ShootSpeedString.cilentData.grapic_data_struct.end_angle = strlen(UI_ShootSpeedString.cilentData.data);
    UI_PushUp_String(&UI_ShootSpeedString);
    osDelay(10);
	
	
		
    //摩擦轮
    UI_FricString.cilentData.grapic_data_struct.graphic_name[0] = 12;
    UI_FricString.cilentData.grapic_data_struct.graphic_name[1] = 0;
    UI_FricString.cilentData.grapic_data_struct.graphic_name[2] = 0;
    UI_FricString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Add;
    UI_FricString.cilentData.grapic_data_struct.graphic_tpye = 7;
    UI_FricString.cilentData.grapic_data_struct.layer = 0;
    UI_FricString.cilentData.grapic_data_struct.color = UI_Color_White;
    UI_FricString.cilentData.grapic_data_struct.start_angle = 25;  
    UI_FricString.cilentData.grapic_data_struct.end_angle = 9;     
    UI_FricString.cilentData.grapic_data_struct.width = 3;
    UI_FricString.cilentData.grapic_data_struct.start_x = 450;
    UI_FricString.cilentData.grapic_data_struct.start_y = 690;
    UI_PushUp_String(&UI_FricString);
    osDelay(10);
		

    //超电进度条
    UI_Draw_Line(&CapData.cilentData[0], (char[3]){8,0,0}, UI_Graph_Add, 0, UI_Color_Green, 20, 650, 150, 1250, 150);//进度条填充线
    UI_Draw_Rectangle(&CapData.cilentData[1], (char[3]){9,0,0}, UI_Graph_Add, 0, UI_Color_Yellow, 2, 650, 140, 1250, 160);//黄色边框
    UI_PushUp_Graphs(2, &CapData);
    osDelay(10);

		
		
    //超电文字
    UI_CapTextString.cilentData.grapic_data_struct.graphic_name[0] = 13;
    UI_CapTextString.cilentData.grapic_data_struct.graphic_name[1] = 0;
    UI_CapTextString.cilentData.grapic_data_struct.graphic_name[2] = 0;
    UI_CapTextString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Add;
    UI_CapTextString.cilentData.grapic_data_struct.graphic_tpye = 7;
    UI_CapTextString.cilentData.grapic_data_struct.layer = 0;
    UI_CapTextString.cilentData.grapic_data_struct.color = UI_Color_Green;
    UI_CapTextString.cilentData.grapic_data_struct.start_angle = 22; // 字体大小
    UI_CapTextString.cilentData.grapic_data_struct.width = 2;        // 线宽
    UI_CapTextString.cilentData.grapic_data_struct.start_x = 1280;
    UI_CapTextString.cilentData.grapic_data_struct.start_y = 145;
    
    snprintf(UI_CapTextString.cilentData.data, 30, "CAP: 100%%  ");
    UI_CapTextString.cilentData.grapic_data_struct.end_angle = strlen(UI_CapTextString.cilentData.data);
    UI_PushUp_String(&UI_CapTextString);
    osDelay(10);
}




void UI_Dynamic()
{
        //雷达吊射锁定
        if(chassis.is_base_locked == 1)
				{
            UI_Draw_Circle(&LockData.cilentData[0], (char[3]){14,0,0}, UI_Graph_Change, 1, UI_Color_Cyan, 10, 960, 540, 80);
        }
				else
				{
            UI_Draw_Circle(&LockData.cilentData[0], (char[3]){14,0,0}, UI_Graph_Change, 1, UI_Color_Black, 1, 0, 0, 0);
        }
        UI_PushUp_Graphs(1, &LockData);
        osDelay(10);
	
				
				
        //云台角度
        UI_GimbalAngleString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Change;
        
        snprintf(UI_GimbalAngleString.cilentData.data, 30, "Y:%.2f P:%.2f    ", 
                 UI_Data.yaw_deg, UI_Data.pit_deg);
                 
        UI_GimbalAngleString.cilentData.grapic_data_struct.end_angle = strlen(UI_GimbalAngleString.cilentData.data);
        
        UI_PushUp_String(&UI_GimbalAngleString);
        osDelay(10);		
}



void UI_Dynamic2()
{	
				//弹速
        UI_ShootSpeedString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Change;
        snprintf(UI_ShootSpeedString.cilentData.data, 30, " %.2f   ", ext_shoot_data.data.bullet_speed);
        UI_ShootSpeedString.cilentData.grapic_data_struct.end_angle = strlen(UI_ShootSpeedString.cilentData.data);
        UI_PushUp_String(&UI_ShootSpeedString);
        osDelay(10);
	
	
	
				
        //摩擦轮
        UI_FricString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Change;
			
        if(chassis.friction_state == 0)
				{
					   UI_FricString.cilentData.grapic_data_struct.color = UI_Color_Pink;
             snprintf(UI_FricString.cilentData.data, 30, " OFF      ");
        } 
				else
				{
            UI_FricString.cilentData.grapic_data_struct.color = UI_Color_Pink;
            snprintf(UI_FricString.cilentData.data, 30, " %d      ", UI_Data.friction_speed);
				}
				UI_FricString.cilentData.grapic_data_struct.end_angle = strlen(UI_FricString.cilentData.data);
        UI_PushUp_String(&UI_FricString);
				osDelay(10);
				
				
				
	      //超电
				if(chassis.battle_mode != 3 && chassis.battle_mode != 2)
				{
						uint8_t current_color;
						if(g_supercap.rx_data.pack.cap_level > 50)
						{
								current_color = UI_Color_Green;   //绿色
						}
						else if (g_supercap.rx_data.pack.cap_level >= 30)
						{
								current_color = UI_Color_Yellow;  //黄色
						}
						else
						{
								current_color = UI_Color_Orange;  //橙色
						}

						//进度条
						int start_x = 650;
						int max_width = 600;
						int current_end_x = start_x + (g_supercap.rx_data.pack.cap_level * max_width / 100);
						
						if(current_end_x <= start_x) current_end_x = start_x + 1;

						UI_Draw_Line(&CapData.cilentData[0], (char[3]){8,0,0}, UI_Graph_Change, 0, current_color, 20, start_x, 150, current_end_x, 150);
						UI_PushUp_Graphs(2, &CapData);
						osDelay(10);
						
						
						
						//电量文字
						UI_CapTextString.cilentData.grapic_data_struct.operate_tpye = UI_Graph_Change;
						UI_CapTextString.cilentData.grapic_data_struct.color = current_color;
						
						snprintf(UI_CapTextString.cilentData.data, 30, "CAP:%d%%   ", g_supercap.rx_data.pack.cap_level);
						UI_CapTextString.cilentData.grapic_data_struct.end_angle = strlen(UI_CapTextString.cilentData.data);
						
						UI_PushUp_String(&UI_CapTextString);
						osDelay(10);
			   }		
}

