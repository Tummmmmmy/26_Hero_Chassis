/**
 * @file can_transmit.c
 * @brief CAN 发送协议层
 *
 * 负责把控制量打包成具体电机/设备协议并发送。
 * 底层 FDCAN 硬件收发由 BSP/bsp_can 提供。
 */
#include "can_transmit.h"
#include "bsp_can.h"
#include "RM_JudgeSystem.h"

/**
  * @brief  大疆电机控制指令发送
  * @param  hfdcan: FDCAN句柄指针
  * @param  std_id: 目标控制CAN ID
  * @param  current: 对应4个电机的目标电流
  */
//大疆电机控制指令
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t tx_data[8];

    tx_data[0] = (uint8_t)(current1 >> 8);
    tx_data[1] = (uint8_t)(current1 & 0xFF);
    tx_data[2] = (uint8_t)(current2 >> 8);
    tx_data[3] = (uint8_t)(current2 & 0xFF);
    tx_data[4] = (uint8_t)(current3 >> 8);
    tx_data[5] = (uint8_t)(current3 & 0xFF);
    tx_data[6] = (uint8_t)(current4 >> 8);
    tx_data[7] = (uint8_t)(current4 & 0xFF);

    //直接写入硬件 FIFO (检查是否有空位)
    BSP_CAN_Send(hfdcan, std_id, tx_data, FDCAN_DLC_BYTES_8);
}


//底盘发给云台的数据
typedef union
{
    uint8_t data[8];
    
    __packed struct
    {
        float bullet_speed;           //弹速
        
        uint32_t shooter_heat   : 10; //当前热量 (0~1023)
        uint32_t heat_limit     : 9;  //热量上限 (0~511)
        uint32_t cooling_value  : 7;  //冷却速率 (0~127)
        uint32_t color          : 1;  //颜色 (0=蓝, 1=红)
        uint32_t ref_online     : 1;  //裁判系统在线 (0=离线, 1=在线)
			
        uint32_t reserved       : 4;  //剩4位
    } pack;
} Chassis_To_Gimbal_Msg_t;

//发送给云台信息
void Send_Gimbal_Data(FDCAN_HandleTypeDef* hfdcan)
{
    Chassis_To_Gimbal_Msg_t tx_msg = {0};

    tx_msg.pack.bullet_speed = ext_shoot_data.data.bullet_speed;

    tx_msg.pack.shooter_heat  = (ext_power_heat_data.data.shooter_id1_42mm_cooling_heat > 1023) ? 1023 : ext_power_heat_data.data.shooter_id1_42mm_cooling_heat;
    tx_msg.pack.heat_limit    = (ext_game_robot_state.data.shooter_barrel_heat_limit > 511) ? 511 : ext_game_robot_state.data.shooter_barrel_heat_limit;
    tx_msg.pack.cooling_value = (ext_game_robot_state.data.shooter_barrel_cooling_value > 127) ? 127 : ext_game_robot_state.data.shooter_barrel_cooling_value;

    tx_msg.pack.color     = is_red_or_blue();
    tx_msg.pack.ref_online = Judge_IsOnline();

    //直接写入硬件 FIFO (检查是否有空位)
    BSP_CAN_Send(hfdcan, TX_GIMBAL_DATA_ID, tx_msg.data, FDCAN_DLC_BYTES_8);
}


// 转发云台发来的拨盘速度指令给达妙拨盘 (0x303 -> 0x207)
void Dial_Relay_Speed_Cmd(FDCAN_HandleTypeDef* hfdcan, uint8_t* data)
{
    BSP_CAN_Send(hfdcan, DIAL_MOTOR_SPEED_COMMAND_ID, data, FDCAN_DLC_BYTES_4);
}


// 转发云台发来的拨盘模式指令给达妙拨盘 (0x301 -> 0x07)
void Dial_Relay_Mode_Cmd(FDCAN_HandleTypeDef* hfdcan, uint8_t* data)
{
    BSP_CAN_Send(hfdcan, DM_DIAL_SEND_ID, data, FDCAN_DLC_BYTES_8);
}


// 转发达妙拨盘反馈给云台板 (0x17 -> 0x302)
void Dial_Relay_Feedback(FDCAN_HandleTypeDef* hfdcan, uint8_t* data)
{
    BSP_CAN_Send(hfdcan, TX_DIAL_FDB_ID, data, FDCAN_DLC_BYTES_8);
}
