/**
 * @file bsp_SuperCap.c
 * @brief 超级电容板级驱动 (FDCAN2, 收 0x600 / 发 0x601)
 *
 * 本文件只负责超电报文收发与状态维护;
 * 充放电决策在 Modules/supercap_control.c, 周期任务在 Task/supercap_task.c。
 */

#include "bsp_SuperCap.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include <string.h>

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

//超电模块内部状态
typedef struct
{
    supercap_rx_data_t rx_data; //接收到的数据
    supercap_tx_data_t tx_data; //准备发送的数据

    bool offline_flag;          //离线标志
    uint32_t last_update_time;  //上次更新时间戳

} supercap_controller_t;

static supercap_controller_t s_supercap;

//初始化超电
void supercap_init(void)
{
    memset(&s_supercap, 0, sizeof(supercap_controller_t));
    s_supercap.offline_flag = true; //默认离线
}

//CAN报文解析
void supercap_rx_decode(const uint8_t* rx_buf, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;
    memcpy(s_supercap.rx_data.data, rx_buf, dlc);

    s_supercap.last_update_time = osKernelGetTickCount();
    s_supercap.offline_flag = false;
}

//离线检测
void supercap_update_offline(uint32_t now_ms)
{
    if(now_ms - s_supercap.last_update_time > SUPERCAP_OFFLINE_TIMEOUT_MS)
    {
        s_supercap.offline_flag = true;
    }
}

//超电是否离线
bool supercap_is_offline(void)
{
    return s_supercap.offline_flag;
}

//超电是否处于放电输出状态
bool supercap_is_output_enabled(void)
{
    return (s_supercap.tx_data.pack.is_cap_output == SUPERCAP_OUTPUT_ENABLE);
}

//获取超电电量百分比
uint8_t supercap_get_cap_level(void)
{
    return s_supercap.rx_data.pack.cap_level;
}

//获取超电是否可用(允许放电)
uint8_t supercap_get_cap_usable(void)
{
    return s_supercap.rx_data.pack.cap_usable;
}

//写入本周期充放电命令
void supercap_set_command(const supercap_cmd_t* cmd)
{
    s_supercap.tx_data.pack.charge_power = cmd->charge_power;
    s_supercap.tx_data.pack.charge_enable = cmd->charge_enable;
    s_supercap.tx_data.pack.is_cap_output = cmd->is_cap_output;
}

//发送超电控制指令
void supercap_send_command(FDCAN_HandleTypeDef* hfdcan)
{
    //0x601
    BSP_CAN_Send(hfdcan, SUPERCAP_TX_ID, s_supercap.tx_data.data, FDCAN_DLC_BYTES_8);
}
