/**
 * @file can_receive.c
 * @brief CAN 接收路由
 *
 * 只做分拣: 按总线/ID 把收到的 CAN 帧分发到对应模块的解码/转发接口,
 * 字节解析由各模块内部完成。
 */

#include "can_receive.h"
#include "fdcan.h"
#include "can_transmit.h"
#include "gimbal_link.h"
#include "chassis_control.h"
#include "bsp_power_meter.h"
#include "bsp_SuperCap.h"

//拨盘相关接收 CAN ID
#define DM_DIAL_RECEIVE_ID 0x17
#define RX_DIAL_CMD_ID 0x303
#define RX_DIAL_MODE_ID 0x301


//按总线与ID分发CAN接收帧
void CAN_Rx_Route(CAN_Rx_Message_t* msg)
{
    if(msg->hfdcan->Instance == FDCAN1)
    {
        //双板通信
        if(msg->can_id == RX_GIMBAL_COMMAND_ID)
        {
            Gimbal_Link_Decode_Command(msg->data);
        }

        //转发云台发来的拨盘速度控制指令
        else if(msg->can_id == RX_DIAL_CMD_ID)
        {
            Dial_Relay_Speed_Cmd(&hfdcan3, msg->data);
        }
        /* 【历史备用路径】拨盘力矩转发: 云台早期也下发过力矩指令(同为 0x303),
           当前拨盘走速度控制; 若改回力矩转发, 把上面的 Dial_Relay_Speed_Cmd(...)
           换成 BSP_CAN_Send(&hfdcan3, DM_DIAL_SEND_ID, msg->data, FDCAN_DLC_BYTES_8);
           (DM_DIAL_SEND_ID 见 can_transmit.h; 启用时本文件需 include "bsp_can.h") */
        //转发云台发来的拨盘模式指令
        else if (msg->can_id == RX_DIAL_MODE_ID)
        {
            Dial_Relay_Mode_Cmd(&hfdcan3, msg->data);
        }
        else if(msg->can_id == RX_CHASSIS_UI_ID)
        {
            Gimbal_Link_Decode_UI(msg->data);
        }
    }
    else if(msg->hfdcan->Instance == FDCAN2)
    {
        //底盘电机反馈
        if(msg->can_id >= 0x201 && msg->can_id <= 0x204)
        {
            Chassis_Update_Feedback(msg->data, msg->can_id);
        }
        //功率计反馈
        else if (msg->can_id == 0x301)
        {
            power_meter_decode(msg->data);
        }
        //超电反馈
        else if (msg->can_id == SUPERCAP_RX_ID)
        {
            supercap_rx_decode(msg->data, msg->dlc);
        }
    }
    //转发拨盘反馈给云台
    else if(msg->hfdcan->Instance == FDCAN3)
    {
        if(msg->can_id == DM_DIAL_RECEIVE_ID)
        {
            Dial_Relay_Feedback(&hfdcan1, msg->data);
        }
    }
}
