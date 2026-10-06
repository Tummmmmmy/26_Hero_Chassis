/**
 * @file bsp_power_meter.c
 * @brief 功率计板级驱动 (FDCAN2, 报文 0x301)
 *
 * 功率计数据仅用于调试 / 功率模型数据采集, 不参与闭环控制;
 * 离线检测由 Monitor_Task 周期调用。
 */

#include "bsp_power_meter.h"
#include "cmsis_os2.h"
#include <string.h>

//功率计反馈数据结构体
typedef struct
{
    float voltage;          //电压
    float current;          //电流
    float power;            //功率

    uint32_t last_update_time;//上次更新的时间戳(ms)
    uint8_t offline_flag;     //离线标志位,0:在线,1:离线
} power_meter_data_t;

//功率计报文布局: 前4字节电压(mV), 后4字节电流(mA)
typedef union
{
    struct
    {
        float voltage_mV;
        float current_mA;
    } pack;
    uint8_t data_buffer[8];
} power_meter_union_t;

static power_meter_data_t s_power_meter;

//功率计初始化
void power_meter_init(void)
{
    s_power_meter.voltage = 0.0f;
    s_power_meter.current = 0.0f;
    s_power_meter.power = 0.0f;
    s_power_meter.last_update_time = 0;
    s_power_meter.offline_flag = 1;//默认离线
}

//解析功率计反馈报文
void power_meter_decode(const uint8_t* rx_data)
{
    power_meter_union_t temp_union;

    memcpy(temp_union.data_buffer, rx_data, 8);

    //单位换算
    s_power_meter.voltage = temp_union.pack.voltage_mV / 1000.0f;//mV -> V
    s_power_meter.current = temp_union.pack.current_mA / 1000.0f;//mA -> A
    //计算功率
    s_power_meter.power = s_power_meter.voltage * s_power_meter.current;

    s_power_meter.offline_flag = 0;
    s_power_meter.last_update_time = osKernelGetTickCount();
}

//功率计离线检测
void power_meter_check_offline(uint32_t timeout_ms)
{
    uint32_t current_tick = osKernelGetTickCount();
    if (current_tick - s_power_meter.last_update_time > timeout_ms)
    {
        s_power_meter.offline_flag = 1;
    }
}

//功率计是否离线
bool power_meter_is_offline(void)
{
    return (s_power_meter.offline_flag != 0);
}

//获取功率计功率(W)
float power_meter_get_power(void)
{
    return s_power_meter.power;
}
