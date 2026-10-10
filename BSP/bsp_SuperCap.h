/**
 * @file bsp_SuperCap.h
 * @brief 超级电容板级驱动接口
 *
 */

#ifndef BSP_SUPERCAP_H
#define BSP_SUPERCAP_H

#include <stdint.h>
#include <stdbool.h>
#include "fdcan.h"

//CAN ID
#define SUPERCAP_RX_ID 0x600
#define SUPERCAP_TX_ID 0x601

//控制指令
#define SUPERCAP_CHARGE_ENABLE   1
#define SUPERCAP_CHARGE_DISABLE  0
#define SUPERCAP_OUTPUT_ENABLE   1
#define SUPERCAP_OUTPUT_DISABLE  0

#define SUPERCAP_OFFLINE_TIMEOUT_MS 2000 //超电离线超时时间
#define SUPERCAP_MIN_OUTPUT_LEVEL   20   //允许放电的最低电量百分比


//超电指令
typedef struct
{
    float   charge_power;
    uint8_t charge_enable; 
    uint8_t is_cap_output;
} supercap_cmd_t;



//初始化超电
void supercap_init(void);

//CAN报文解析
void supercap_rx_decode(const uint8_t* rx_buf, uint8_t dlc);

//离线检测
void supercap_update_offline(uint32_t now_ms);

//超电是否离线
bool supercap_is_offline(void);

//超电是否处于放电输出状态
bool supercap_is_output_enabled(void);

//获取超电电量百分比
uint8_t supercap_get_cap_level(void);

//超电是否允许放电
uint8_t supercap_get_cap_usable(void);

//更新控制命令
void supercap_set_command(const supercap_cmd_t* cmd);

//发送超电控制指令
void supercap_send_command(FDCAN_HandleTypeDef* hfdcan);

#endif
