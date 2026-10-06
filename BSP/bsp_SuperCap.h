/**
 * @file bsp_SuperCap.h
 * @brief 超级电容板级驱动接口 (FDCAN2, 收 0x600 / 发 0x601)
 *
 * 本文件只负责超电报文收发与状态维护;
 * 充放电决策在 Modules/supercap_control.c, 周期任务在 Task/supercap_task.c。
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
#define SUPERCAP_MIN_OUTPUT_LEVEL   20  //允许放电的最低电量百分比

//对外命令(由 supercap_control 决策后写入)
typedef struct
{
    float   charge_power;  //充电功率(W)
    uint8_t charge_enable; //充电使能
    uint8_t is_cap_output; //超电放电输出使能
} supercap_cmd_t;

//初始化超电
void supercap_init(void);

//CAN报文解析
void supercap_rx_decode(const uint8_t* rx_buf, uint8_t dlc);

//离线检测(超时置离线), 由 SuperCap_Task 周期调用
void supercap_update_offline(uint32_t now_ms);

//超电是否离线
bool supercap_is_offline(void);

//超电是否处于放电输出状态
bool supercap_is_output_enabled(void);

//获取超电电量百分比
uint8_t supercap_get_cap_level(void);

//获取超电是否可用(允许放电)
uint8_t supercap_get_cap_usable(void);

//写入本周期充放电命令
void supercap_set_command(const supercap_cmd_t* cmd);

//发送超电控制指令(0x601)
void supercap_send_command(FDCAN_HandleTypeDef* hfdcan);

#endif
