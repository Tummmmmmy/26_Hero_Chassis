/**
 * @file gimbal_link.c
 * @brief 底盘板与云台板板间链路实现
 *
 */

#include "gimbal_link.h"
#include "cmsis_os.h"
#include <string.h>

//接收的云台数据
volatile Chassis_Cmd_t chassis = {0,0,0,0,0,0,1,0,0};
volatile Chassis_UI_t UI_Data = {0,0,0};

//板间链路在线状态
static uint32_t last_rx_tick = 0;
static volatile bool is_online = false;


//云台发底盘数据
typedef union
{
    uint8_t data[8];
    
    __packed struct
    {
        int64_t  vx : 16;
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
void Gimbal_Link_Decode_Command(const uint8_t *data)
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

    last_rx_tick = osKernelGetTickCount();
    is_online = true;
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
void Gimbal_Link_Decode_UI(const uint8_t *data)
{
    Gimbal_UI_Msg_t rx_ui;
    
    memcpy(rx_ui.data, data, 8);
    

    UI_Data.yaw_deg  = (float)rx_ui.pack.yaw_angle_x100 * 0.01f;
    UI_Data.pit_deg  = (float)rx_ui.pack.pitch_angle_x100 * 0.01f;
    UI_Data.friction_speed = rx_ui.pack.friction_speed;
}


//双板通信链路在线巡检
void Gimbal_Link_Update_Online_Status(uint32_t current_tick)
{
    if(current_tick - last_rx_tick > GIMBAL_LINK_OFFLINE_TIMEOUT_MS)
    {
        is_online = false;
    }
}


//双板通信链路是否在线
bool Gimbal_Link_Is_Online(void)
{
    return is_online;
}
