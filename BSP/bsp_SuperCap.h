/**
 * @file bsp_SuperCap.h
 * @brief 超级电容驱动接口
 *
 */

#ifndef BSP_SUPERCAP_H
#define BSP_SUPERCAP_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>



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


//超电反馈数据
typedef union
{
    uint8_t data[8];
    struct
    {
        float   chassis_power;
        uint8_t chassis_buff;
        uint8_t cap_usable;
        uint8_t cap_level;
        uint8_t reserved;
    } __attribute__((packed)) pack;
} supercap_rx_data_t;



//超电控制数据
typedef union
{
    uint8_t data[8];
    struct
    {
        float   charge_power;
        uint8_t charge_enable;
        uint8_t is_cap_output;
        uint8_t reserved[2];
    } __attribute__((packed)) pack;
} supercap_tx_data_t;



//超电模块总控制器
typedef struct
{
    supercap_rx_data_t rx_data; //接收到的数据
    supercap_tx_data_t tx_data; //准备发送的数据

    bool offline_flag;          //离线标志
    uint32_t last_update_time;  //上次更新时间戳

} supercap_controller_t;

extern supercap_controller_t g_supercap;




//初始化超电
void supercap_init(void);

//CAN报文解析
void supercap_rx_decode(uint8_t* rx_buf, uint8_t dlc);

//超电任务
void SuperCap_Task(void const * argument);

#endif
